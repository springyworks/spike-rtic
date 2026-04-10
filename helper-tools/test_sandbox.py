#!/usr/bin/env python3
"""
Test the MPU sandbox on SPIKE Prime hub.

Creates tiny ARM Thumb machine-code demos, uploads via COBS,
and runs them with `go` (sandboxed) and `go!` (privileged).

Demos:
  1. "hello" — writes "Hello from SRAM2!\n" via api->write_fn, returns 42
  2. "fault" — reads from SRAM1 (0x20000000), triggers MemManage fault
  3. "nop"   — just returns 0xCAFE

MonitorApi layout (32-bit ARM, all fields 4 bytes):
  +0x00  version        u32
  +0x04  context        *mut u8
  +0x08  write_fn       fn(ctx, data, len)
  +0x0C  delay_ms       fn(ms)
  +0x10  set_pixel      fn(idx, bri)
  +0x14  update_leds    fn()
  +0x18  read_adc       fn(ch) -> u32
  +0x1C  read_buttons   fn() -> u8
  +0x20  motor_set      fn(port, speed)
  +0x24  motor_brake    fn(port)
  +0x28  sensor_read    fn(buf, len) -> u32
  +0x2C  sensor_mode    fn(mode)
  +0x30  sound_play     fn(freq)
  +0x34  sound_stop     fn()
"""

import struct
import serial
import serial.tools.list_ports
import time
import sys
import os

# ── Project root: $PROJECT_ROOT or derived from this script's location ──
PROJECT_ROOT = os.environ.get(
    "PROJECT_ROOT",
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

# Shared port detection (prefer symlinks → sysfs → VID:PID scan)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spike_port import find_shell_port, open_serial, BAUD

PORT = find_shell_port() or "/dev/ttyACM0"
SRAM2_BASE = 0x20040000

# ── COBS encode ──────────────────────────────────────────────

def cobs_encode(data: bytes) -> bytes:
    """COBS encode: ensure no 0x00 in payload, terminate with 0x00."""
    out = bytearray()
    idx = 0
    while idx < len(data):
        # Find next zero (or end of data)
        start = idx
        while idx < len(data) and data[idx] != 0 and (idx - start) < 254:
            idx += 1
        code = idx - start + 1
        out.append(code)
        out.extend(data[start:idx])
        if idx < len(data) and data[idx] == 0:
            idx += 1  # skip the zero byte
    out.append(0x00)  # frame delimiter
    return bytes(out)


# ── ARM Thumb assembler helpers ──────────────────────────────

def thumb_movs(rd, imm8):
    """MOVS Rd, #imm8  (T1 encoding)"""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack("<H", 0x2000 | (rd << 8) | imm8)

def thumb_ldr_rn_imm5(rt, rn, imm5_words):
    """LDR Rt, [Rn, #imm]  where imm = imm5_words * 4"""
    assert 0 <= rt <= 7 and 0 <= rn <= 7 and 0 <= imm5_words <= 31
    return struct.pack("<H", 0x6800 | (imm5_words << 6) | (rn << 3) | rt)

def thumb_ldr_rn_off(rt, rn, offset_words):
    """LDR Rt, [Rn, #offset] — offset in words (×4)"""
    return thumb_ldr_rn_imm5(rt, rn, offset_words)

def thumb_str_rn_imm5(rt, rn, imm5_words):
    """STR Rt, [Rn, #imm]"""
    assert 0 <= rt <= 7 and 0 <= rn <= 7 and 0 <= imm5_words <= 31
    return struct.pack("<H", 0x6000 | (imm5_words << 6) | (rn << 3) | rt)

def thumb_mov_rd_rm(rd, rm):
    """MOV Rd, Rm (high registers ok)"""
    D = (rd >> 3) & 1
    rd_lo = rd & 7
    return struct.pack("<H", 0x4600 | (D << 7) | (rm << 3) | rd_lo)

def thumb_push(reg_list_low, lr=False):
    """PUSH {regs, LR}"""
    val = 0xB400 | reg_list_low
    if lr:
        val |= (1 << 8)
    return struct.pack("<H", val)

def thumb_pop(reg_list_low, pc=False):
    """POP {regs, PC}"""
    val = 0xBC00 | reg_list_low
    if pc:
        val |= (1 << 8)
    return struct.pack("<H", val)

def thumb_blx(rm):
    """BLX Rm"""
    return struct.pack("<H", 0x4780 | (rm << 3))

def thumb_bx_lr():
    """BX LR"""
    return struct.pack("<H", 0x4770)

def thumb_svc(imm8):
    """SVC #imm8"""
    return struct.pack("<H", 0xDF00 | (imm8 & 0xFF))

def thumb_nop():
    """NOP"""
    return struct.pack("<H", 0xBF00)

def thumb_add_sp_imm7(imm7_words):
    """ADD SP, SP, #imm (imm = imm7_words * 4)"""
    assert 0 <= imm7_words <= 127
    return struct.pack("<H", 0xB000 | imm7_words)

def thumb_sub_sp_imm7(imm7_words):
    """SUB SP, SP, #imm (imm = imm7_words * 4)"""
    assert 0 <= imm7_words <= 127
    return struct.pack("<H", 0xB080 | imm7_words)

def thumb_movw(rd, imm16):
    """MOVW Rd, #imm16 (Thumb-2, 32-bit)"""
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 1
    imm3 = (imm16 >> 8) & 7
    imm8 = imm16 & 0xFF
    hw1 = 0xF240 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack("<HH", hw1, hw2)

def thumb_movt(rd, imm16):
    """MOVT Rd, #imm16 (Thumb-2, 32-bit)"""
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 1
    imm3 = (imm16 >> 8) & 7
    imm8 = imm16 & 0xFF
    hw1 = 0xF2C0 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack("<HH", hw1, hw2)

def thumb_mov_imm32(rd, val):
    """Load 32-bit immediate into Rd using MOVW + MOVT"""
    code = thumb_movw(rd, val & 0xFFFF)
    if val > 0xFFFF:
        code += thumb_movt(rd, (val >> 16) & 0xFFFF)
    return code

def thumb_add_rd_rn_imm3(rd, rn, imm3):
    """ADDS Rd, Rn, #imm3"""
    assert 0 <= rd <= 7 and 0 <= rn <= 7 and 0 <= imm3 <= 7
    return struct.pack("<H", 0x1C00 | (imm3 << 6) | (rn << 3) | rd)


# ── Build demo binaries ─────────────────────────────────────

def build_nop_demo():
    """
    Simplest demo: just return 0xCAFE.

    extern "C" fn _start(api: *const MonitorApi) -> u32 {
        0xCAFE
    }
    """
    code = b""
    code += thumb_mov_imm32(0, 0xCAFE)  # r0 = 0xCAFE
    code += thumb_bx_lr()                # return r0
    return code


def build_hello_demo():
    """
    Demo that calls api->write_fn to print "Hello from SRAM2!\\r\\n"
    then returns 42.

    In sandboxed mode, write_fn is an SVC thunk:
      svc_write(_ctx, data, len) does SVC #0

    The demo code stores the string in SRAM2 (after the code itself),
    then calls api->write_fn(api->context, string_ptr, string_len).

    Register plan:
      r4 = api pointer (saved across calls)
      r0 = api->context  (1st arg to write_fn)
      r1 = string ptr    (2nd arg)
      r2 = string len    (3rd arg)
      r3 = write_fn ptr

    The string is appended after the code at a known offset.
    """
    # We'll place the string right after the code.  First build code
    # without knowing the exact offset, then patch.
    msg = b"Hello from SRAM2!\r\n"

    code = b""
    code += thumb_push(1 << 4, lr=True)       # push {r4, lr}
    code += thumb_mov_rd_rm(4, 0)             # r4 = api (save)

    # Load write_fn from api+0x08 (offset 2 words)
    code += thumb_ldr_rn_off(3, 4, 2)         # r3 = [r4+8] = write_fn

    # Load context from api+0x04 (offset 1 word)
    code += thumb_ldr_rn_off(0, 4, 1)         # r0 = [r4+4] = context

    # r1 = address of string (PC-relative, but easier to use absolute)
    # We know the string will be at SRAM2_BASE + len(code_before_string)
    # For now, put a placeholder for r1; we'll compute after
    code_so_far = len(code)
    # We need: movw r1, #lo; movt r1, #hi; movs r2, #len; blx r3
    # That's 4+4+2+2 = 12 bytes more of code, then movs r0, #42; pop {r4, pc}
    # Actually let's just compute:
    # remaining instructions before string:
    remaining_before_string = (
        4 +  # movw r1
        4 +  # movt r1
        2 +  # movs r2, #len
        2 +  # blx r3
        4 +  # movw r0, #42 (or movs)
        2    # pop {r4, pc}
    )  # = 18 bytes
    string_offset = code_so_far + remaining_before_string
    string_addr = SRAM2_BASE + string_offset

    code += thumb_mov_imm32(1, string_addr)   # r1 = &string
    code += thumb_movs(2, len(msg))            # r2 = len
    code += thumb_blx(3)                       # call write_fn(ctx, str, len)

    # Return 42
    code += thumb_mov_imm32(0, 42)             # r0 = 42
    code += thumb_pop(1 << 4, pc=True)         # pop {r4, pc}

    # Verify our offset calculation
    assert len(code) == string_offset, f"offset mismatch: {len(code)} != {string_offset}"

    # Append the string data
    code += msg

    # Pad to 4-byte alignment
    while len(code) % 4:
        code += b"\x00"

    return code


def build_fault_demo():
    """
    Demo that deliberately reads from SRAM1 (0x20000000) which is
    protected by MPU region 2 (priv-only).  This should trigger
    a MemManage fault in sandboxed mode, and work fine in privileged mode.

    extern "C" fn _start(api: *const MonitorApi) -> u32 {
        let val = *(0x20000000 as *const u32);  // BOOM in sandbox
        val
    }
    """
    code = b""
    code += thumb_mov_imm32(0, 0x20000000)   # r0 = 0x20000000
    code += thumb_ldr_rn_off(0, 0, 0)         # r0 = [r0] — read SRAM1
    code += thumb_bx_lr()                      # return r0
    return code


def build_periph_fault_demo():
    """
    Demo that reads from peripheral space (GPIOA at 0x40020000).
    MPU region 3 blocks this in sandbox mode.
    """
    code = b""
    code += thumb_mov_imm32(0, 0x40020000)   # r0 = GPIOA base
    code += thumb_ldr_rn_off(0, 0, 0)         # r0 = [r0] — read GPIO
    code += thumb_bx_lr()                      # return r0
    return code


# ── Serial communications ───────────────────────────────────

def open_serial_conn():
    """Open serial connection to hub."""
    s = open_serial(PORT, BAUD, timeout=2)
    time.sleep(0.3)
    s.reset_input_buffer()
    return s

def send_cmd(s, cmd, wait=1.0):
    """Send a shell command and read response."""
    s.reset_input_buffer()
    s.write((cmd + "\r\n").encode())
    time.sleep(wait)
    resp = s.read(s.in_waiting or 4096).decode("ascii", errors="replace")
    return resp

def upload_binary(s, data):
    """Upload binary via COBS protocol."""
    # Send upload command
    s.reset_input_buffer()
    s.write(f"upload {len(data)}\r\n".encode())
    time.sleep(0.5)
    resp = s.read(s.in_waiting or 1024).decode("ascii", errors="replace")
    print(f"  upload cmd response: {resp.strip()}")

    if "READY" not in resp:
        print("  ERROR: hub did not respond with READY")
        return False

    # COBS-encode and send
    encoded = cobs_encode(data)
    print(f"  sending {len(data)} bytes ({len(encoded)} COBS-encoded)")
    s.write(encoded)
    time.sleep(1.0)
    resp = s.read(s.in_waiting or 1024).decode("ascii", errors="replace")
    print(f"  upload result: {resp.strip()}")

    return "OK" in resp

def run_demo(s, cmd="go", wait=3.0):
    """Run demo with go or go! and return response."""
    s.reset_input_buffer()
    s.write((cmd + "\r\n").encode())
    time.sleep(wait)
    resp = s.read(s.in_waiting or 4096).decode("ascii", errors="replace")
    return resp


# ── Test runner ──────────────────────────────────────────────

def test_nop(s):
    """Test 1: NOP demo — just returns 0xCAFE."""
    print("\n" + "="*60)
    print("TEST 1: NOP demo (returns 0xCAFE)")
    print("="*60)

    demo = build_nop_demo()
    print(f"  demo size: {len(demo)} bytes")
    print(f"  hex: {demo.hex()}")

    if not upload_binary(s, demo):
        print("  UPLOAD FAILED")
        return False

    # Test sandboxed
    print("\n  --- go (sandboxed) ---")
    resp = run_demo(s, "go")
    print(f"  response: {resp.strip()}")
    ok_sand = "0x0000CAFE" in resp or "51966" in resp
    print(f"  PASS: {'YES' if ok_sand else 'NO'}")

    time.sleep(0.5)

    # Test privileged
    print("\n  --- go! (privileged) ---")
    resp = run_demo(s, "go!")
    print(f"  response: {resp.strip()}")
    ok_priv = "0x0000CAFE" in resp or "51966" in resp
    print(f"  PASS: {'YES' if ok_priv else 'NO'}")

    return ok_sand and ok_priv


def test_hello(s):
    """Test 2: Hello demo — prints via api->write_fn, returns 42."""
    print("\n" + "="*60)
    print("TEST 2: Hello demo (prints + returns 42)")
    print("="*60)

    demo = build_hello_demo()
    print(f"  demo size: {len(demo)} bytes")
    print(f"  hex: {demo.hex()}")

    if not upload_binary(s, demo):
        print("  UPLOAD FAILED")
        return False

    # Test sandboxed
    print("\n  --- go (sandboxed) ---")
    resp = run_demo(s, "go", wait=3.0)
    print(f"  response: {resp.strip()}")
    ok_sand = "Hello from SRAM2" in resp and "42" in resp
    print(f"  PASS: {'YES' if ok_sand else 'NO'}")

    time.sleep(0.5)

    # Re-upload (upload buffer was overwritten by demo output? no — demo output goes to demo_io buf)
    # Actually the upload buffer in SRAM2 still has the demo code.
    # Test privileged
    print("\n  --- go! (privileged) ---")
    resp = run_demo(s, "go!", wait=3.0)
    print(f"  response: {resp.strip()}")
    ok_priv = "Hello from SRAM2" in resp and "42" in resp
    print(f"  PASS: {'YES' if ok_priv else 'NO'}")

    return ok_sand and ok_priv


def test_fault_sram1(s):
    """Test 3: SRAM1 fault demo — should fault in sandbox, work in privileged."""
    print("\n" + "="*60)
    print("TEST 3: SRAM1 read (should FAULT in sandbox)")
    print("="*60)

    demo = build_fault_demo()
    print(f"  demo size: {len(demo)} bytes")
    print(f"  hex: {demo.hex()}")

    if not upload_binary(s, demo):
        print("  UPLOAD FAILED")
        return False

    # Test sandboxed — should fault!
    print("\n  --- go (sandboxed) — expecting FAULT ---")
    resp = run_demo(s, "go", wait=3.0)
    print(f"  response: {resp.strip()}")
    ok_fault = "FAULT" in resp or "DEAD" in resp
    print(f"  Faulted as expected: {'YES' if ok_fault else 'NO'}")

    time.sleep(0.5)

    # Test privileged — should work (returns value at 0x20000000)
    print("\n  --- go! (privileged) — should succeed ---")
    resp = run_demo(s, "go!", wait=3.0)
    print(f"  response: {resp.strip()}")
    # Should return some value (whatever is at 0x20000000), not a fault
    ok_priv = "=>" in resp and "FAULT" not in resp
    print(f"  Succeeded as expected: {'YES' if ok_priv else 'NO'}")

    return ok_fault and ok_priv


def test_fault_periph(s):
    """Test 4: Peripheral read — should fault in sandbox."""
    print("\n" + "="*60)
    print("TEST 4: Peripheral read (should FAULT in sandbox)")
    print("="*60)

    demo = build_periph_fault_demo()
    print(f"  demo size: {len(demo)} bytes")
    print(f"  hex: {demo.hex()}")

    if not upload_binary(s, demo):
        print("  UPLOAD FAILED")
        return False

    # Test sandboxed — should fault!
    print("\n  --- go (sandboxed) — expecting FAULT ---")
    resp = run_demo(s, "go", wait=3.0)
    print(f"  response: {resp.strip()}")
    ok_fault = "FAULT" in resp or "DEAD" in resp
    print(f"  Faulted as expected: {'YES' if ok_fault else 'NO'}")

    time.sleep(0.5)

    # Test privileged — should work
    print("\n  --- go! (privileged) — should succeed ---")
    resp = run_demo(s, "go!", wait=3.0)
    print(f"  response: {resp.strip()}")
    ok_priv = "=>" in resp and "FAULT" not in resp
    print(f"  Succeeded as expected: {'YES' if ok_priv else 'NO'}")

    return ok_fault and ok_priv


def main():
    print("SPIKE Prime MPU Sandbox Test Suite")
    print(f"Port: {PORT}")
    print()

    s = open_serial_conn()

    # Quick connectivity check
    resp = send_cmd(s, "uptime")
    print(f"Hub uptime: {resp.strip()}")

    results = {}

    results["nop"] = test_nop(s)
    time.sleep(1)

    results["hello"] = test_hello(s)
    time.sleep(1)

    results["fault_sram1"] = test_fault_sram1(s)
    time.sleep(1)

    results["fault_periph"] = test_fault_periph(s)
    time.sleep(1)

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    all_pass = True
    for name, ok in results.items():
        status = "PASS" if ok else "FAIL"
        print(f"  {name:20s} {status}")
        if not ok:
            all_pass = False

    print()
    if all_pass:
        print("ALL TESTS PASSED")
    else:
        print("SOME TESTS FAILED")

    # Final uptime to prove hub is still alive
    resp = send_cmd(s, "uptime")
    print(f"\nHub uptime after tests: {resp.strip()}")

    s.close()
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
