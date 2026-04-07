#!/usr/bin/env python3
"""Build, upload, and enter GDB RSP mode — one-shot pre-launch script.

Usage:  python3 debug_pipeline.py <binfile> [serial_port]

Steps:
  1. Test serial connection to SPIKE hub
  2. Upload the .bin file via COBS
  3. Send 'go' to launch demo (sandboxed RTIC task)
  4. Send 'gdb' to enter RSP mode
  5. Close serial port so arm-none-eabi-gdb can connect

Designed as VS Code preLaunchTask — GDB connects after this exits 0.
"""

import os
import serial
import serial.tools.list_ports
import sys
import time

LEGO_VID    = 0x0694
RUNTIME_PID = 0x0042
BAUD = 115200


def find_hub_port():
    """Auto-detect LEGO SPIKE hub serial port by VID:PID."""
    for p in serial.tools.list_ports.comports():
        if (p.vid or 0) == LEGO_VID and (p.pid or 0) == RUNTIME_PID:
            return p.device
    return None


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


def wait_for_prompt(s, timeout=8):
    """Drain output until 'spike>' appears."""
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            if buf.rstrip().endswith(b"spike>"):
                break
        else:
            time.sleep(0.05)
    return buf.decode("ascii", errors="replace")


def send_cmd(s, cmd, timeout=5):
    """Send shell command, wait for response + next prompt."""
    s.reset_input_buffer()
    s.write(cmd.encode() + b"\r\n")
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            text = buf.decode("ascii", errors="replace")
            if text.count("spike>") >= 1 and len(text) > len(cmd) + 10:
                break
        else:
            time.sleep(0.05)
    return buf.decode("ascii", errors="replace")


def fail(msg):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def main():
    if len(sys.argv) < 2:
        fail("Usage: debug_pipeline.py <binfile> [serial_port]")

    binfile = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) > 2 else find_hub_port()
    if not port:
        fail("LEGO SPIKE hub not found (VID:PID 0694:0042). Is it plugged in?")

    with open(binfile, "rb") as f:
        data = f.read()
    print(f"Binary: {binfile} ({len(data)} bytes)")

    # ── 1. Test serial connection ──
    print(f"Connecting to {port} ...")
    try:
        s = serial.Serial(port, BAUD, timeout=2)
    except serial.SerialException as e:
        fail(f"Cannot open {port}: {e}")

    # If hub is stuck in RSP mode from a prior session, send detach first
    time.sleep(0.1)
    s.reset_input_buffer()
    s.write(b"$D#44")
    s.flush()
    time.sleep(0.5)
    s.reset_input_buffer()

    # Send Ctrl-C + stop to kill any running demo from a prior session
    s.write(b"\x03")
    time.sleep(0.1)
    s.write(b"stop\r\n")
    time.sleep(0.3)
    s.reset_input_buffer()

    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(b"\r\n")
    banner = wait_for_prompt(s, timeout=8)
    if "spike>" not in banner:
        print("(waiting for hub boot...)")
        time.sleep(3)
        # Try Ctrl-C + stop again in case hub was in GDB after boot
        s.reset_input_buffer()
        s.write(b"\x03")
        time.sleep(0.1)
        s.write(b"$D#44")
        s.flush()
        time.sleep(0.3)
        s.write(b"stop\r\n")
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"\r\n")
        banner = wait_for_prompt(s, timeout=8)
    if "spike>" not in banner:
        s.close()
        fail(f"No spike> prompt. Got: {banner!r}")
    print("Hub connected — got prompt.")

    # ── 2. Upload binary via COBS ──
    s.reset_input_buffer()
    s.write(f"upload {len(data)}\r\n".encode())
    buf = b""
    deadline = time.time() + 3
    while time.time() < deadline:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            if b"READY" in buf or b"ERR" in buf:
                break
        else:
            time.sleep(0.05)
    resp = buf.decode("ascii", errors="replace")
    if "READY" not in resp:
        s.close()
        fail(f"Upload not ready: {resp.strip()}")
    print(f"Upload: {resp.strip()}")

    encoded = cobs_encode(data)
    s.write(encoded)
    buf = b""
    deadline = time.time() + 5
    while time.time() < deadline:
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
    if "OK" not in resp:
        s.close()
        fail(f"Upload failed: {resp.strip()}")
    print(f"Upload OK: {resp.strip()}")

    # ── 3. Send 'go' to start demo ──
    s.reset_input_buffer()
    s.write(b"go\r\n")
    time.sleep(0.3)
    n = s.in_waiting
    if n > 0:
        go_resp = s.read(n).decode("ascii", errors="replace")
        print(f"go: {go_resp.strip()}")
    else:
        print("go: sent (no echo)")

    # ── 4. Send 'gdb' to enter RSP mode ──
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write(b"gdb\r\n")
    time.sleep(0.3)
    n = s.in_waiting
    if n > 0:
        gdb_resp = s.read(n).decode("ascii", errors="replace")
        print(f"RSP: {gdb_resp.strip()}")
    else:
        print("RSP: gdb command sent")

    # ── 5. Release port for GDB ──
    s.close()

    # Configure tty for raw access (so cat/echo work after this script)
    import subprocess
    subprocess.run(
        ["stty", "-F", port, "115200", "raw", "-echo", "-hupcl"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # Create stable symlink so VS Code launch.json can use a fixed path
    symlink = "/tmp/spike-hub"
    try:
        if os.path.islink(symlink) or os.path.exists(symlink):
            os.unlink(symlink)
        os.symlink(port, symlink)
        print(f"Symlink {symlink} -> {port}")
    except OSError as e:
        print(f"Warning: could not create {symlink}: {e}")

    print(f"Port {port} released — ready for GDB.")


if __name__ == "__main__":
    main()
