#!/usr/bin/env python3
"""
test_hub_mon.py — Automated test suite for the SPIKE Prime Hub Monitor CLI.

Tests every shell command (original + Angel-style debug monitor commands).
Prints results live with PASS/FAIL verdicts.

Usage:
  python3 test_hub_mon.py              # auto-detect port
  python3 test_hub_mon.py /dev/ttyACM0 # explicit port

Requires: pyserial (pip install pyserial)
"""

import sys
import os
import time
import glob
import re
import binascii
import struct

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("FATAL: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# ── Configuration ──

BAUD = 115200
READ_TIMEOUT = 0.3
PROMPT = b"spike> "
SEPARATOR = "=" * 60


# ── Intelligent port detection ──

def find_hub_port() -> str:
    """
    Intelligently find the SPIKE Prime hub CDC serial port.
    Tries multiple strategies in order:
      1. LEGO VID (0x0694) — runtime RTIC firmware
      2. STM32 VID (0x0483) — if using stock ST USB PID
      3. Description matching ('spike', 'lego', 'stm32', 'cdc')
      4. Fallback: first /dev/ttyACM* that is openable
    """
    candidates = []

    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        vid = p.vid or 0
        pid = p.pid or 0
        tag = f"{p.device}  VID:PID={vid:04X}:{pid:04X}  [{p.description}]"

        # Strategy 1: LEGO VID
        if vid == 0x0694:
            print(f"  [LEGO device] {tag}")
            return p.device

        # Strategy 2: STM32 VID with CDC class
        if vid == 0x0483:
            print(f"  [STM32 device] {tag}")
            candidates.insert(0, p.device)
            continue

        # Strategy 3: name matching
        for keyword in ("spike", "lego", "stm32", "cdc", "acm"):
            if keyword in desc:
                print(f"  [name match: {keyword}] {tag}")
                candidates.append(p.device)
                break

    if candidates:
        print(f"  -> Using: {candidates[0]}")
        return candidates[0]

    # Strategy 4: brute-force ttyACM*
    acm_ports = sorted(glob.glob("/dev/ttyACM*"))
    for port in acm_ports:
        try:
            s = serial.Serial(port, BAUD, timeout=0.5)
            # Send a CR and see if we get a prompt back
            s.reset_input_buffer()
            s.write(b"\r\n")
            time.sleep(0.3)
            data = s.read(256)
            s.close()
            if b"spike>" in data or b"RTIC" in data or b"SPIKE" in data:
                print(f"  [probe OK] {port} (got shell prompt)")
                return port
            else:
                print(f"  [probe] {port} — no prompt (got {len(data)} bytes)")
        except (serial.SerialException, OSError) as e:
            print(f"  [skip] {port} — {e}")

    if acm_ports:
        print(f"  [fallback] {acm_ports[0]}")
        return acm_ports[0]

    return ""


# ── Serial connection ──

class HubConnection:
    def __init__(self, port: str):
        self.port = port
        self.ser = None
        self._open()

    def _open(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = serial.Serial(self.port, BAUD, timeout=READ_TIMEOUT,
                                 write_timeout=5.0, dsrdtr=False, rtscts=False)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def _recover(self):
        """Recover from USB CDC stall by closing/reopening the port."""
        print("    [recovering USB connection...]")
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        time.sleep(1.0)
        # Re-detect port in case it re-enumerated
        new_port = find_hub_port()
        if new_port:
            self.port = new_port
        self._open()
        # Flush and get fresh prompt
        self.ser.reset_input_buffer()
        self.ser.write(b"\r\n")
        time.sleep(0.5)
        self.ser.read(4096)  # discard
        print(f"    [recovered on {self.port}]")

    def send_command(self, cmd: str, timeout: float = 10.0) -> str:
        """Send a command, return cleaned response text.
        Auto-recovers from USB CDC stalls/timeouts."""
        for attempt in range(2):
            try:
                return self._send_command_inner(cmd, timeout)
            except (serial.SerialException, serial.SerialTimeoutException,
                    OSError, IOError) as e:
                if attempt == 0:
                    self._recover()
                else:
                    raise
        return ""

    def _send_command_inner(self, cmd: str, timeout: float) -> str:
        # Flush pending data
        self.ser.reset_input_buffer()
        time.sleep(0.05)

        self.ser.write(cmd.encode() + b"\r\n")
        self.ser.flush()

        buf = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                chunk = self.ser.read(512)
            except (serial.SerialException, OSError):
                break
            if chunk:
                buf.extend(chunk)
                if PROMPT in buf:
                    break
            elif buf:
                time.sleep(0.05)

        text = buf.decode(errors="replace")
        # Strip echo and prompt
        lines = text.split("\r\n")
        if lines and cmd.strip() in lines[0]:
            lines = lines[1:]
        cleaned = []
        for line in lines:
            line = line.replace("spike>", "").rstrip()
            if line:
                cleaned.append(line)
        return "\n".join(cleaned).strip()

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass


# ── Test runner ──

class TestRunner:
    def __init__(self, conn: HubConnection):
        self.conn = conn
        self.passed = 0
        self.failed = 0
        self.errors = []
        self.start_time = time.time()

    def log(self, msg: str):
        elapsed = time.time() - self.start_time
        print(f"[{elapsed:6.1f}s] {msg}")

    def test(self, name: str, cmd: str, *checks, timeout: float = 10.0):
        """
        Run a command and check the response.
        Each check is either:
          - a string: response must contain this substring (case-insensitive)
          - a callable: called with (response_text) -> bool
        """
        # Small inter-command delay to let USB CDC breathe
        time.sleep(0.15)
        self.log(f"TEST: {name}")
        self.log(f"  cmd> {cmd}")
        try:
            resp = self.conn.send_command(cmd, timeout=timeout)
        except Exception as e:
            self.log(f"  FAIL (exception: {e})")
            self.failed += 1
            self.errors.append((name, f"exception: {e}"))
            return ""

        # Show response (indented)
        for line in resp.split("\n"):
            self.log(f"  | {line}")

        # Run checks
        ok = True
        for check in checks:
            if callable(check):
                if not check(resp):
                    self.log(f"  FAIL (check function returned False)")
                    ok = False
            elif isinstance(check, str):
                if check.lower() not in resp.lower():
                    self.log(f"  FAIL (expected '{check}' not found)")
                    ok = False

        if ok:
            self.log(f"  PASS ✓")
            self.passed += 1
        else:
            self.failed += 1
            self.errors.append((name, f"response: {resp[:120]}"))

        return resp

    def summary(self):
        elapsed = time.time() - self.start_time
        total = self.passed + self.failed
        print()
        print(SEPARATOR)
        print(f"TEST SUMMARY  ({elapsed:.1f}s)")
        print(SEPARATOR)
        print(f"  Total:  {total}")
        print(f"  Passed: {self.passed}")
        print(f"  Failed: {self.failed}")
        if self.errors:
            print()
            print("  FAILURES:")
            for name, detail in self.errors:
                print(f"    - {name}: {detail[:80]}")
        print(SEPARATOR)
        return self.failed == 0


# ── Validation helpers ──

def has_hex(text: str) -> bool:
    """Response contains at least one 0x... hex value."""
    return bool(re.search(r'0x[0-9A-Fa-f]{2,8}', text))

def has_hex_dump(text: str) -> bool:
    """Response looks like a hex memory dump (addr: val val val val)."""
    return bool(re.search(r'[0-9A-Fa-f]{8}:\s+[0-9A-Fa-f]{8}', text))

def has_number(text: str) -> bool:
    """Response contains a number."""
    return bool(re.search(r'\d+', text))

def not_error(text: str) -> bool:
    """Response does not start with ERR."""
    return not text.strip().upper().startswith("ERR")

def contains_all(*substrings):
    """Return a checker that verifies all substrings are present."""
    def check(text: str) -> bool:
        lower = text.lower()
        return all(s.lower() in lower for s in substrings)
    return check

def matches_pattern(pattern: str):
    """Return a checker that verifies a regex pattern matches."""
    def check(text: str) -> bool:
        return bool(re.search(pattern, text, re.IGNORECASE))
    return check


# ── COBS encoder / upload helpers ──

def cobs_encode(data: bytes) -> bytes:
    """Encode data using Consistent Overhead Byte Stuffing."""
    output = bytearray()
    code_idx = 0
    code = 1
    output.append(0)  # placeholder for first code byte

    for byte in data:
        if byte == 0x00:
            output[code_idx] = code
            code_idx = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_idx] = code
                code_idx = len(output)
                output.append(0)
                code = 1

    output[code_idx] = code
    return bytes(output)


def upload_binary(conn: HubConnection, filepath: str) -> tuple:
    """Upload a binary via COBS. Returns (success, size, crc)."""
    with open(filepath, "rb") as f:
        data = f.read()

    local_crc = binascii.crc32(data) & 0xFFFFFFFF
    size = len(data)

    # Send upload command
    conn.ser.reset_input_buffer()
    time.sleep(0.05)
    conn.ser.write(f"upload {size}\r\n".encode())
    conn.ser.flush()

    # Wait for READY
    buf = bytearray()
    deadline = time.time() + 5.0
    while time.time() < deadline:
        chunk = conn.ser.read(256)
        if chunk:
            buf.extend(chunk)
            if b"READY" in buf:
                break
    if b"READY" not in buf:
        return (False, size, local_crc)

    time.sleep(0.05)

    # COBS encode and send
    encoded = cobs_encode(data) + b"\x00"
    CHUNK = 512
    sent = 0
    while sent < len(encoded):
        end = min(sent + CHUNK, len(encoded))
        conn.ser.write(encoded[sent:end])
        conn.ser.flush()
        sent = end
        time.sleep(0.002)

    # Wait for OK
    buf = bytearray()
    deadline = time.time() + 10.0
    while time.time() < deadline:
        chunk = conn.ser.read(512)
        if chunk:
            buf.extend(chunk)
            if b"spike>" in buf:
                break
    resp = buf.decode(errors="replace")
    ok = "OK " in resp
    return (ok, size, local_crc)


# ── Test suite ──

def run_all_tests(conn: HubConnection):
    t = TestRunner(conn)

    print(SEPARATOR)
    print("  SPIKE Prime Hub Monitor CLI — Full Test Suite")
    print("  Angel-debugger-style commands + original commands")
    print(SEPARATOR)
    print()

    # ── Phase 1: Basic / Original commands ──
    t.log("─── Phase 1: Original Shell Commands ───")

    t.test("help", "help",
           "Hub Monitor CLI",
           "help")

    t.test("info", "info",
           "STM32F413",
           "RTIC")

    t.test("uptime", "uptime",
           "ticks")

    t.test("btn (read buttons)", "btn",
           not_error)

    t.test("led heart", "led heart",
           "OK heart")

    t.test("led digit 3", "led 3",
           "OK digit")

    t.test("led check", "led check",
           "OK check")

    t.test("led clear", "led clear",
           "OK clear")

    t.test("px 0 50 (set pixel)", "px 0 50",
           "OK")

    t.test("px 0 0 (clear pixel)", "px 0 0",
           "OK")

    t.test("status green", "status green",
           "OK")

    t.test("status off", "status off",
           "OK")

    # ── Phase 2: Angel Monitor — Memory Commands ──
    t.log("")
    t.log("─── Phase 2: Angel Monitor — Memory ───")

    # Read vector table at start of our firmware
    t.test("md — vector table @ 0x08008000", "md 0x08008000 4",
           has_hex_dump,
           matches_pattern(r'08008000:'))

    # Read SRAM start
    t.test("md — SRAM1 start", "md 0x20000000 4",
           has_hex_dump)

    # Read peripheral (RCC)
    t.test("md — RCC base", "md 0x40023800 4",
           has_hex_dump)

    # Default count (16 words)
    resp = t.test("md — default count", "md 0x08008000",
                  has_hex_dump)
    # Should have 4 lines (16 words / 4 per line)
    dump_lines = [l for l in resp.split("\n") if re.match(r'\s*[0-9A-Fa-f]{8}:', l)]
    if len(dump_lines) >= 4:
        t.log(f"  (got {len(dump_lines)} dump lines — correct)")

    # Alignment error
    t.test("md — alignment error", "md 0x08008001",
           "aligned")

    # mw — write to SRAM2 (safe area near top — 0x2004_FF00)
    t.test("mw — write word", "mw 0x2004FF00 0xDEADBEEF",
           "OK",
           "DEADBEEF")

    # Verify the write via md
    t.test("md — verify write", "md 0x2004FF00 1",
           "DEADBEEF")

    # Write back zero
    t.test("mw — clear test word", "mw 0x2004FF00 0x00000000",
           "OK")

    # mw alignment error
    t.test("mw — alignment error", "mw 0x2004FF01 0x12345678",
           "aligned")

    # ── Phase 3: Angel Monitor — System Registers ──
    t.log("")
    t.log("─── Phase 3: Angel Monitor — Registers ───")

    t.test("regs — Cortex-M4 system regs", "regs",
           "CPUID",
           "VTOR",
           "SP:",
           matches_pattern(r'CPUID:\s+0x[0-9A-Fa-f]{8}'))

    # Verify CPUID is a Cortex-M4 (implementer=0x41=ARM, partno=0xC24)
    t.test("regs — CPUID decode", "regs",
           "impl=0x41")

    # ── Phase 4: Angel Monitor — Clocks ──
    t.log("")
    t.log("─── Phase 4: Angel Monitor — Clocks ───")

    t.test("clocks — RCC config", "clocks",
           "RCC Clock Config",
           "CR:",
           "PLLCFGR:",
           "HSE",
           "PLL",
           "SYSCLK",
           "USB")

    # Verify expected frequencies
    t.test("clocks — frequency check", "clocks",
           "96 MHz",    # SYSCLK
           "APB1")

    # ── Phase 5: Angel Monitor — GPIO ──
    t.log("")
    t.log("─── Phase 5: Angel Monitor — GPIO ───")

    t.test("gpio a — port A state", "gpio a",
           "GPIOA MODER",
           "OTYPER",
           "IDR",
           "ODR",
           "Pins:",
           "(I=in O=out A=AF N=analog)")

    t.test("gpio b — port B state", "gpio b",
           "GPIOB MODER",
           "Pins:")

    t.test("gpio c — port C state", "gpio c",
           "GPIOC MODER")

    t.test("gpio d", "gpio d",
           "GPIOD MODER")

    t.test("gpio e", "gpio e",
           "GPIOE MODER")

    t.test("gpio — bad port", "gpio x",
           "ERR")

    t.test("gpio — no arg", "gpio",
           "Usage")

    # ── Phase 6: Angel Monitor — Device ID & Flash ──
    t.log("")
    t.log("─── Phase 6: Angel Monitor — UID & Flash ───")

    t.test("uid — unique device ID", "uid",
           "UID:",
           matches_pattern(r'[0-9A-Fa-f]{8}-[0-9A-Fa-f]{8}-[0-9A-Fa-f]{8}'))

    t.test("flash — flash info", "flash",
           "Flash:",
           "1024",          # 1024 KB = 1 MB
           "OPTCR:",
           "ACR:",
           "RDP:")

    # ── Phase 7: Angel Monitor — ADC & Battery ──
    t.log("")
    t.log("─── Phase 7: Angel Monitor — ADC & Battery ───")

    t.test("adc 14 — center button ADC", "adc 14",
           "ADC ch14:",
           has_number)

    t.test("adc 1 — left/right button ADC", "adc 1",
           "ADC ch1:",
           has_number)

    t.test("adc 11 — battery voltage ADC", "adc 11",
           "ADC ch11:",
           has_number)

    t.test("adc — bad channel", "adc 16",
           "ERR")

    t.test("adc — no arg", "adc",
           "Usage")

    t.test("bat — battery readings", "bat",
           "Voltage:",
           "mV",
           "Current:",
           "NTC temp:",
           "USB chg:")

    # Verify battery voltage is reasonable (> 3000 mV, < 10000 mV)
    def bat_voltage_sane(text):
        m = re.search(r'Voltage:\s+(\d+)\s+mV', text)
        if not m:
            return False
        mv = int(m.group(1))
        return 1000 < mv < 12000  # generous range, USB powered or battery
    t.test("bat — voltage sanity", "bat", bat_voltage_sane)

    # ── Phase 8: Angel Monitor — CRC ──
    t.log("")
    t.log("─── Phase 8: Angel Monitor — CRC ───")

    # CRC of the vector table (first 16 bytes of firmware)
    resp1 = t.test("crc — firmware header", "crc 0x08008000 16",
                   "CRC32",
                   matches_pattern(r'0x[0-9A-Fa-f]{8}\.\.\+16'))

    # Run again — should be deterministic
    resp2 = t.test("crc — deterministic check", "crc 0x08008000 16",
                   "CRC32")

    # Parse and compare CRC values
    def extract_crc(text):
        m = re.search(r'CRC32.*?(0x[0-9A-Fa-f]{8})$', text, re.MULTILINE)
        return m.group(1) if m else None

    crc1 = extract_crc(resp1)
    crc2 = extract_crc(resp2)
    if crc1 and crc2 and crc1 == crc2:
        t.log(f"  CRC deterministic: {crc1} == {crc2} ✓")
    elif crc1 and crc2:
        t.log(f"  WARN: CRC mismatch {crc1} != {crc2}")

    # CRC of zero-length should still work (addr only matters at boundary)
    t.test("crc — zero length", "crc 0x08008000 0",
           "CRC32")

    # ── Phase 9: Angel Monitor — Fill & Verify ──
    t.log("")
    t.log("─── Phase 9: Angel Monitor — Fill & Verify ───")

    # Fill 4 words at top of SRAM2 with pattern
    t.test("fill — write pattern", "fill 0x2004FE00 4 0xCAFEBABE",
           "OK",
           "filled 4 words")

    # Verify with md
    t.test("md — verify fill", "md 0x2004FE00 4",
           "CAFEBABE")

    # Clean up — fill with 0
    t.test("fill — clear", "fill 0x2004FE00 4 0x00000000",
           "OK")

    # Verify cleared
    t.test("md — verify clear", "md 0x2004FE00 4",
           "00000000")

    # fill alignment error
    t.test("fill — alignment error", "fill 0x2004FE01 4 0x11111111",
           "aligned")

    # ── Phase 10: RAM tests ──
    t.log("")
    t.log("─── Phase 10: RAM Tests ───")

    t.test("raminfo", "raminfo",
           "SRAM1:",
           "SRAM2:",
           "SP:",
           "256K",
           "64K")

    t.test("ramtest safe", "ramtest safe",
           "PASS",
           timeout=15.0)

    t.test("bininfo (no upload)", "bininfo",
           not_error)

    # ── Phase 10b: Demo Upload & Execute ──
    t.log("")
    t.log("─── Phase 10b: Demo Upload & Execute ───")

    # Locate demo binary relative to this test script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    demo_bin = os.path.join(script_dir, "hub-ram-demos", "spike-demo.bin")

    if os.path.isfile(demo_bin):
        demo_size = os.path.getsize(demo_bin)
        with open(demo_bin, "rb") as f:
            demo_crc = binascii.crc32(f.read()) & 0xFFFFFFFF
        t.log(f"  demo: {demo_bin} ({demo_size} bytes, CRC 0x{demo_crc:08X})")

        # Upload
        t.log(f"  Uploading demo...")
        ok, size, crc = upload_binary(conn, demo_bin)
        if ok:
            t.log(f"  Upload OK ({size} bytes)")
            t.passed += 1
        else:
            t.log(f"  FAIL: upload did not succeed")
            t.failed += 1
            t.errors.append(("demo upload", "upload did not return OK"))

        # bininfo should now show the demo
        t.test("bininfo (after upload)", "bininfo",
               f"{demo_size}",
               matches_pattern(r'0x[0-9A-Fa-f]{8}'))

        # Execute demo via go
        resp = t.test("go (execute demo)", "go",
                      "Hello from SRAM2",
                      "blink",
                      matches_pattern(r'bat_raw=\d+'),
                      matches_pattern(r'=>\s+\d+'),
                      timeout=15.0)

        # Verify return value is a plausible ADC reading (0-4095)
        def adc_return_sane(text):
            m = re.search(r'=>\s+(\d+)', text)
            if not m:
                return False
            val = int(m.group(1))
            return 0 < val < 4096
        if resp:
            if adc_return_sane(resp):
                t.log(f"  Return value is valid ADC reading ✓")
            else:
                t.log(f"  WARN: return value outside expected ADC range")
    else:
        t.log(f"  SKIP: demo binary not found at {demo_bin}")
        t.log(f"  Build it: cd hub-ram-demos && cargo build --release && "
              f"arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/spike-demo spike-demo.bin")

    # ── Phase 11: Error handling ──
    t.log("")
    t.log("─── Phase 11: Error Handling ───")

    t.test("unknown command", "xyzzy",
           "ERR",
           "unknown")

    t.test("px — bad args", "px 30 50",
           "Usage")

    t.test("md — missing addr", "md",
           "Usage")

    t.test("mw — missing args", "mw",
           "Usage")

    t.test("crc — missing args", "crc",
           "Usage")

    t.test("fill — missing args", "fill",
           "Usage")

    t.test("status — bad color", "status purple",
           "ERR")

    t.test("led — bad pattern", "led batman",
           "ERR")

    # ── Phase 12: Visual confirm — leave hub in a recognizable state ──
    t.log("")
    t.log("─── Phase 12: Visual Confirmation ───")

    t.test("led check (final)", "led check",
           "OK check")

    t.test("status green (final)", "status green",
           "OK")

    # ── Done ──
    return t.summary()


# ── Main ──

def main():
    print(SEPARATOR)
    print("  SPIKE Prime Hub Monitor CLI — Automated Test Suite")
    print(SEPARATOR)
    print()

    # Determine port
    port = ""
    if len(sys.argv) > 1 and (sys.argv[1].startswith("/dev/") or sys.argv[1].startswith("COM")):
        port = sys.argv[1]
        print(f"Using explicit port: {port}")
    else:
        print("Auto-detecting hub port...")
        port = find_hub_port()

    if not port:
        print("FATAL: No hub port found. Is the hub connected and booted?")
        print("  Tip: after DFU flash, wait ~3s for USB CDC enumeration.")
        sys.exit(1)

    print(f"Connecting to {port}...")
    try:
        conn = HubConnection(port)
    except serial.SerialException as e:
        print(f"FATAL: Cannot open {port}: {e}")
        # Try other ports
        print("Scanning other ports...")
        port = find_hub_port()
        if port:
            conn = HubConnection(port)
        else:
            sys.exit(1)

    print(f"Connected to {port}")
    print()

    # Warm up: send a CR and wait for prompt
    print("Waiting for shell prompt...")
    conn.ser.write(b"\r\n")
    time.sleep(0.5)
    warmup = conn.ser.read(4096)
    warmup_text = warmup.decode(errors="replace")
    if "spike>" in warmup_text:
        print("Got shell prompt — hub is alive!")
    elif warmup_text:
        print(f"Hub responded ({len(warmup)} bytes): {warmup_text[:120]}")
        # Send another CR
        conn.ser.write(b"\r\n")
        time.sleep(0.3)
        more = conn.ser.read(1024)
        if b"spike>" in more:
            print("Got prompt on second try.")
    else:
        print("WARNING: No response from hub. Proceeding anyway...")
    print()

    try:
        success = run_all_tests(conn)
    finally:
        conn.close()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
