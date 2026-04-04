#!/usr/bin/env python3
"""Test sandbox SPSEL fix — verifies the hub stays alive after sandboxed go.

Usage:
    1. Put the hub in DFU mode OR leave it running on ttyACM0.
    2. Run:  python3 test_spsel_fix.py
    3. Watch the hub's heartbeat LED for 10 seconds after the demo finishes.
       If it keeps blinking, the fix works.

What this script does:
    - Detects whether hub is in DFU mode or on ttyACM0
    - If DFU: flashes target/spike-rtic.bin, waits for ttyACM0
    - Uploads /tmp/simple_test.bin (169 bytes)
    - Runs `go` (sandboxed), verifies output and return code 42
    - Waits 10 seconds, then runs `go` AGAIN to test repeated execution
    - Runs `uptime` to confirm hub is alive
    - Leaves the serial port CLEANLY closed
"""

import serial
import subprocess
import time
import sys
import os

PORT = "/dev/ttyACM0"
BAUD = 115200
FW_BIN = "target/spike-rtic.bin"
DEMO_BIN = "/tmp/simple_test.bin"


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while idx < len(data):
        start = idx
        while idx < len(data) and data[idx] != 0 and (idx - start) < 254:
            idx += 1
        code = idx - start + 1
        out.append(code)
        out.extend(data[start:idx])
        if idx < len(data) and data[idx] == 0:
            idx += 1
    out.append(0x00)
    return bytes(out)


def is_dfu():
    r = subprocess.run(["lsusb"], capture_output=True, text=True)
    return "0694:0011" in r.stdout


def is_ttyacm():
    return os.path.exists(PORT)


def flash_dfu():
    print(f"Flashing {FW_BIN} via DFU...")
    r = subprocess.run(
        ["sudo", "dfu-util", "-d", "0694:0011", "-a", "0",
         "-s", "0x08008000:leave", "-D", FW_BIN],
        capture_output=True, text=True, timeout=30,
    )
    if r.returncode != 0:
        print(f"DFU flash FAILED:\n{r.stderr}")
        sys.exit(1)
    print("Flash OK. Waiting for hub to boot...")
    for _ in range(20):
        time.sleep(0.5)
        if is_ttyacm():
            print(f"Hub on {PORT}")
            return
    print("ERROR: hub didn't appear on ttyACM0 after flash")
    sys.exit(1)


def drain(s, timeout=1.0):
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
        else:
            time.sleep(0.05)
    return buf.decode("ascii", errors="replace")


def send_cmd(s, cmd, timeout=5.0):
    s.reset_input_buffer()
    s.write((cmd + "\r\n").encode())
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            text = buf.decode("ascii", errors="replace")
            if "spike>" in text and len(text) > len(cmd) + 5:
                return text
        else:
            time.sleep(0.05)
    return buf.decode("ascii", errors="replace")


def upload_demo(s, binpath):
    with open(binpath, "rb") as f:
        data = f.read()
    print(f"  Uploading {len(data)} bytes from {binpath}...")

    s.reset_input_buffer()
    s.write(f"upload {len(data)}\r\n".encode())
    buf = b""
    end = time.time() + 3
    while time.time() < end:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            if b"READY" in buf or b"ERR" in buf:
                break
        else:
            time.sleep(0.05)

    if b"READY" not in buf:
        print(f"  ERROR: no READY. Got: {buf}")
        return False

    encoded = cobs_encode(data)
    s.write(encoded)

    buf = b""
    end = time.time() + 5
    while time.time() < end:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            if b"OK" in buf or b"ERR" in buf:
                time.sleep(0.2)
                n2 = s.in_waiting
                if n2 > 0:
                    buf += s.read(n2)
                break
        else:
            time.sleep(0.05)

    resp = buf.decode("ascii", errors="replace")
    if "OK" in resp:
        print(f"  Upload OK")
        drain(s, 0.5)
        return True
    else:
        print(f"  Upload FAILED: {resp}")
        return False


def run_go(s, label="go"):
    print(f"  Running `{label}`...")
    resp = send_cmd(s, "go", timeout=10.0)
    lines = resp.strip().split("\n")
    for l in lines:
        l = l.strip()
        if l and l != "go" and not l.endswith("spike>"):
            print(f"    {l}")

    if "=> 42" in resp:
        print(f"  PASS: demo returned 42")
        return True
    elif "DEAD" in resp:
        print(f"  FAIL: fault/abort detected")
        return False
    else:
        print(f"  UNKNOWN result in: {resp[:200]}")
        return False


def main():
    print("=== Sandbox SPSEL Fix Test ===\n")

    # Step 1: Get hub on ttyACM0
    if is_dfu():
        flash_dfu()
    elif is_ttyacm():
        print(f"Hub already on {PORT}")
    else:
        print("Hub not detected. Power on the hub and try again.")
        sys.exit(1)

    time.sleep(1)

    # Step 2: Open serial
    try:
        s = serial.Serial(PORT, BAUD, timeout=2)
    except Exception as e:
        print(f"Cannot open {PORT}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    s.reset_input_buffer()
    s.write(b"\r\n")
    banner = drain(s, 2.0)
    if "spike>" not in banner:
        s.write(b"\r\n")
        banner = drain(s, 3.0)
    if "spike>" not in banner:
        print(f"No prompt. Got: {banner[:200]}")
        s.close()
        sys.exit(1)
    print("Got spike> prompt.\n")

    # Step 3: Upload demo
    if not upload_demo(s, DEMO_BIN):
        s.close()
        sys.exit(1)

    # Step 4: First go (sandboxed)
    print("\n--- Test 1: first sandbox go ---")
    ok1 = run_go(s, "first go")

    # Step 5: Quick uptime check
    resp = send_cmd(s, "uptime", timeout=2)
    for l in resp.strip().split("\n"):
        l = l.strip()
        if "tick" in l:
            print(f"  uptime: {l}")

    # Step 6: Wait 10 seconds — heartbeat should keep blinking
    print("\n  Waiting 10s (watch heartbeat LED)...", flush=True)
    time.sleep(10)

    # Step 7: Check hub is still alive
    resp = send_cmd(s, "uptime", timeout=2)
    alive_after_wait = "tick" in resp
    if alive_after_wait:
        for l in resp.strip().split("\n"):
            if "tick" in l.strip():
                print(f"  uptime after wait: {l.strip()}")
        print("  PASS: hub alive after 10s")
    else:
        print("  FAIL: hub not responding after 10s wait")

    # Step 8: Second go (tests repeated sandbox execution)
    print("\n--- Test 2: second sandbox go ---")
    ok2 = run_go(s, "second go")

    # Step 9: Final uptime
    resp = send_cmd(s, "uptime", timeout=2)
    for l in resp.strip().split("\n"):
        if "tick" in l.strip():
            print(f"  final uptime: {l.strip()}")

    s.close()

    # Summary
    print("\n=== RESULTS ===")
    print(f"  First go:        {'PASS' if ok1 else 'FAIL'}")
    print(f"  Alive after 10s: {'PASS' if alive_after_wait else 'FAIL'}")
    print(f"  Second go:       {'PASS' if ok2 else 'FAIL'}")

    if ok1 and alive_after_wait and ok2:
        print("\n  ALL TESTS PASSED")
    else:
        print("\n  SOME TESTS FAILED")

    sys.exit(0 if (ok1 and alive_after_wait and ok2) else 1)


if __name__ == "__main__":
    main()
