#!/usr/bin/env python3
"""Upload a demo binary to SPIKE hub RAM, store it to external SPI flash,
then optionally execute it with 'spiflash run'.

Workflow:
  1. Open serial connection to hub (/dev/ttyACM0, 115200 baud)
  2. Send `upload <size>` shell command to enter COBS upload mode
  3. COBS-encode the binary (zero-byte stuffing) and transmit
  4. Hub acknowledges with OK + CRC32
  5. Send `spiflash store <hex_addr> <size>` to write RAM → SPI flash
  6. Optionally send `spiflash run <hex_addr> <size>` to execute from flash

The W25Q256JV SPI flash (32 MB) on the hub is sector-erasable (4 KB).
Multiple demos can be stored at different offsets (slots) for instant
boot without re-uploading.

Contents:
  - cobs_encode()  — COBS framing (0x00-free payload + 0x00 sentinel)
  - send_cmd()     — shell command helper with CRLF line ending
  - main()         — CLI entry: parse args, upload, store, run

Usage:
    python3 flash_store_run.py [binfile] [flash_addr_hex] [--run]

Examples:
    # Store color-seek at slot 0 (0x00000000) and run it:
    python3 flash_store_run.py examples/hub-ram-demos/target/spike-usr_bins/color_seek.bin 0 --run

    # Just store, don't run:
    python3 flash_store_run.py examples/hub-ram-demos/target/spike-usr_bins/color_seek.bin 0x10000
"""

import serial, serial.tools.list_ports, time, sys, os

# ── Project root: $PROJECT_ROOT or derived from this script's location ──
PROJECT_ROOT = os.environ.get(
    "PROJECT_ROOT",
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

# Shared port detection (prefer symlinks → sysfs → VID:PID scan)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spike_port import find_shell_port, open_serial, BAUD

PORT = find_shell_port() or "/dev/ttyACM0"

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


def send_cmd(s, cmd, wait=1.0):
    """Send a command (with CRLF), wait, return response text."""
    s.reset_input_buffer()
    s.write(f"{cmd}\r\n".encode())
    time.sleep(wait)
    n = s.in_waiting or 1
    return s.read(max(n, 1024)).decode("ascii", errors="replace")


def main():
    binfile = sys.argv[1] if len(sys.argv) > 1 else "examples/hub-ram-demos/target/color-seek.bin"
    flash_addr_str = sys.argv[2] if len(sys.argv) > 2 else "0"
    do_run = "--run" in sys.argv

    flash_addr = int(flash_addr_str, 0)

    with open(binfile, "rb") as f:
        data = f.read()
    size = len(data)
    print(f"Binary: {binfile} ({size} bytes)")
    print(f"Flash addr: 0x{flash_addr:08X}")

    s = open_serial(PORT, BAUD, timeout=2)
    time.sleep(0.5)
    s.reset_input_buffer()

    # Flush any boot banner — send CRLF and read prompt
    s.write(b"\r\n")
    time.sleep(0.5)
    s.read(s.in_waiting or 256)  # discard

    # ── Step 1: Upload binary to RAM via COBS ──
    print("\n[1/3] Uploading to RAM...")
    s.reset_input_buffer()
    s.write(f"upload {size}\r\n".encode())
    time.sleep(1.0)
    resp = s.read(s.in_waiting or 1024).decode("ascii", errors="replace")
    print(f"  {resp.strip()}")
    if "READY" not in resp:
        print("ERROR: no READY response")
        s.close()
        return

    encoded = cobs_encode(data)
    s.write(encoded)
    time.sleep(1.0)
    resp = s.read(s.in_waiting or 1024).decode("ascii", errors="replace")
    print(f"  {resp.strip()}")
    if "OK" not in resp:
        print("ERROR: upload failed")
        s.close()
        return

    # ── Step 2: Store RAM buffer to SPI flash ──
    print(f"\n[2/3] Storing to SPI flash @ 0x{flash_addr:08X}...")
    resp = send_cmd(s, f"spiflash store 0x{flash_addr:X} {size}", wait=5.0)
    for line in resp.strip().split("\n"):
        line = line.strip()
        if line:
            print(f"  {line}")

    # Verify with spiflash dir
    resp = send_cmd(s, "spiflash dir", wait=1.0)
    print(f"  Dir: {resp.strip()}")

    # ── Step 3: Run from SPI flash ──
    if do_run:
        print(f"\n[3/3] Running from flash: spiflash run 0x{flash_addr:X} {size}")

        # Select sensor port F first
        resp = send_cmd(s, "sensor f", wait=10.0)
        print(f"  Sensor: {resp.strip()[-80:]}")

        s.reset_input_buffer()
        s.write(f"spiflash run 0x{flash_addr:X} {size}\r\n".encode())

        print("\n--- Demo output (30s, Ctrl-C to stop) ---")
        deadline = time.time() + 30
        try:
            while time.time() < deadline:
                n = s.in_waiting
                if n > 0:
                    chunk = s.read(n).decode("ascii", errors="replace")
                    sys.stdout.write(chunk)
                    sys.stdout.flush()
                else:
                    time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n[interrupted]")
    else:
        print(f"\nStored! To run later: spiflash run 0x{flash_addr:X} {size}")

    s.close()
    print("\n--- Done ---")


if __name__ == "__main__":
    main()
