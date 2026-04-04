#!/usr/bin/env python3
"""Upload a .bin file to the SPIKE hub and run `go`, streaming output.

Usage
-----
    python3 upload_demo.py <binfile> [timeout_seconds]

Arguments:
    binfile          Path to a raw ARM binary (.bin) for SRAM2 execution.
                     Build one from hub-ram-demos:
                       cd examples/hub-ram-demos
                       cargo build --release --example <name>
                       arm-none-eabi-objcopy -O binary \\
                         target/thumbv7em-none-eabihf/release/examples/<name> \\
                         /tmp/<name>.bin
    timeout_seconds  How long to stream demo output (default: 30).

Examples:
    # Build & run the motor_goto demo
    cd examples/hub-ram-demos
    cargo build --release --example motor_goto
    arm-none-eabi-objcopy -O binary \\
        target/thumbv7em-none-eabihf/release/examples/motor_goto /tmp/motor_goto.bin
    cd ../..
    python3 upload_demo.py /tmp/motor_goto.bin 60

    # Quick run with default 30s timeout
    python3 upload_demo.py /tmp/color_seeker.bin

Flow:
    1. Opens /dev/ttyACM0 at 115200 baud
    2. Waits for the spike> prompt (drains boot banner)
    3. Sends 'upload <size>' — hub replies READY
    4. COBS-encodes the binary and sends it — hub replies OK + CRC
    5. Sends 'sensor f' to ensure color sensor is probed on port F
    6. Sends 'go' to launch the demo in sandboxed mode
    7. Streams demo output for <timeout> seconds (Ctrl-C to stop early)
"""

import serial, time, sys

PORT = "/dev/ttyACM0"
BAUD = 115200

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
    """Drain all banners/output until we see 'spike> ', return accumulated text."""
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            # Check if the prompt is at the end of buffer (possibly after \r\n)
            if buf.rstrip().endswith(b"spike>"):
                break
        else:
            time.sleep(0.05)
    return buf.decode("ascii", errors="replace")

def send_cmd(s, cmd, timeout=5):
    """Send a shell command (with CRLF), wait for full response up to next prompt."""
    s.reset_input_buffer()
    s.write(cmd.encode() + b"\r\n")
    buf = b""
    deadline = time.time() + timeout
    # Wait for the echo + response + next prompt
    while time.time() < deadline:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            # Response is complete when we see the next prompt after the echo
            text = buf.decode("ascii", errors="replace")
            # Skip past the echoed command, look for prompt after response
            if text.count("spike>") >= 1 and len(text) > len(cmd) + 10:
                break
        else:
            time.sleep(0.05)
    return buf.decode("ascii", errors="replace")


def main():
    binfile = sys.argv[1] if len(sys.argv) > 1 else "examples/hub-ram-demos/target/color-seek.bin"
    wait = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    sensor_port = sys.argv[3] if len(sys.argv) > 3 else "f"

    with open(binfile, "rb") as f:
        data = f.read()
    print(f"Binary: {binfile} ({len(data)} bytes)")

    s = serial.Serial(PORT, BAUD, timeout=2)

    # ── Robust prompt wait: drain all boot banners first ──
    time.sleep(0.3)
    s.reset_input_buffer()
    # Poke with CRLF to elicit a prompt
    s.write(b"\r\n")
    banner = wait_for_prompt(s, timeout=8)
    if "spike>" not in banner:
        # Second attempt — maybe the hub was mid-boot
        print("(waiting for hub boot...)")
        time.sleep(2)
        s.reset_input_buffer()
        s.write(b"\r\n")
        banner = wait_for_prompt(s, timeout=8)
    if "spike>" not in banner:
        print(f"ERROR: no spike> prompt. Got: {banner!r}")
        s.close()
        return
    print("Got prompt.")

    # ── Upload (special: shell enters COBS mode after READY, no prompt) ──
    s.reset_input_buffer()
    s.write(f"upload {len(data)}\r\n".encode())
    # Wait for "READY <n>" (shell won't send another prompt until COBS is done)
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
    print(f"Upload cmd: {resp.strip()}")
    if "READY" not in resp:
        print("ERROR: no READY")
        s.close()
        return

    # Send COBS-encoded binary
    encoded = cobs_encode(data)
    s.write(encoded)
    # Wait for "OK <n> CRC=..." + prompt
    buf = b""
    deadline = time.time() + 5
    while time.time() < deadline:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            if b"OK" in buf or b"ERR" in buf:
                # Give a moment for the full line + prompt
                time.sleep(0.2)
                n2 = s.in_waiting
                if n2 > 0:
                    buf += s.read(n2)
                break
        else:
            time.sleep(0.05)
    resp = buf.decode("ascii", errors="replace")
    print(f"Upload result: {resp.strip()}")
    if "OK" not in resp:
        print("ERROR: upload failed")
        s.close()
        return

    # Select sensor port before running
    s.reset_input_buffer()
    s.write(f"sensor {sensor_port}\r\n".encode())
    # Wait for probe to complete — LUMP sync can take 10+ seconds
    buf = b""
    deadline = time.time() + 15
    while time.time() < deadline:
        n = s.in_waiting
        if n > 0:
            buf += s.read(n)
            decoded = buf.decode("ascii", errors="replace")
            # Probe done when we see spike> after the "Probing" banner
            if "spike>" in decoded and ("Probing" in decoded or "mode" in decoded.lower()):
                break
        else:
            time.sleep(0.2)
    print(f"Sensor select: {buf.decode('ascii', errors='replace').strip()}")

    # Check sensor status
    resp = send_cmd(s, "sensor", timeout=3)
    print(f"Sensor status: {resp.strip()}")

    # Run
    s.reset_input_buffer()
    s.write(b"go\r\n")
    print(f"\n--- Running (streaming for {wait}s, Ctrl-C to stop) ---")

    deadline = time.time() + wait
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

    # Drain any remaining
    time.sleep(0.2)
    n = s.in_waiting
    if n > 0:
        sys.stdout.write(s.read(n).decode("ascii", errors="replace"))
        sys.stdout.flush()

    s.close()
    print("\n--- Done ---")

if __name__ == "__main__":
    main()
