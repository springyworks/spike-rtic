#!/usr/bin/env python3
"""
hub.py — SPIKE Prime Hub helper: detect state, flash, send commands.

Eliminates manual hub-state checking. Detects DFU / serial / disconnected
automatically and takes the right action.

Usage:
    python3 hub.py                     # show hub state
    python3 hub.py flash               # flash firmware (enters DFU if needed)
    python3 hub.py cmd "sensor e"      # send one command, print response
    python3 hub.py cmd "sensor stop" "sensor e" "sensor"  # multiple commands
    python3 hub.py flash+cmd "sensor e"  # flash, wait for reboot, send command
    python3 hub.py wait                # block until hub is in serial mode
    python3 hub.py monitor             # continuous monitor (Ctrl-C to stop)

Requires: pyserial (pip install pyserial)
"""

import sys
import os
import time
import subprocess
import glob

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("FATAL: pyserial not installed.  pip install pyserial")
    sys.exit(1)

# ── Constants ──
LEGO_VID     = 0x0694
DFU_PID      = 0x0011
RUNTIME_PID  = 0x0042
BAUD         = 115200
FLASH_ADDR   = "0x08008000"
BIN_PATH     = "target/spike-rtic.bin"
DFU_ARGS     = ["dfu-util", "-d", "0694:0011", "-a", "0",
                "-s", f"{FLASH_ADDR}:leave", "-D"]

# ANSI colors
C_RED    = "\033[91m"
C_GREEN  = "\033[92m"
C_YELLOW = "\033[93m"
C_CYAN   = "\033[96m"
C_RESET  = "\033[0m"

def hub_state():
    """Return ('dfu', dev), ('serial', port), or ('disconnected', None)."""
    # Check pyserial first for runtime CDC
    for p in serial.tools.list_ports.comports():
        vid = p.vid or 0
        pid = p.pid or 0
        if vid == LEGO_VID and pid == RUNTIME_PID:
            return ("serial", p.device)
    # Check lsusb for DFU (no serial port exposed)
    try:
        out = subprocess.check_output(["lsusb"], text=True, timeout=5)
        if f"{LEGO_VID:04x}:{DFU_PID:04x}" in out.lower():
            return ("dfu", None)
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return ("disconnected", None)


def print_state(state, detail):
    """Pretty-print hub state."""
    if state == "dfu":
        print(f"  Hub: {C_YELLOW}DFU mode{C_RESET}  (ready to flash)")
    elif state == "serial":
        print(f"  Hub: {C_GREEN}Running{C_RESET}  serial={C_CYAN}{detail}{C_RESET}")
    else:
        print(f"  Hub: {C_RED}Disconnected{C_RESET}  (power on or plug USB)")


def wait_for_state(target, timeout=30, quiet=False):
    """Block until hub reaches target state. Returns (state, detail)."""
    deadline = time.time() + timeout
    last_msg = ""
    while time.time() < deadline:
        st, det = hub_state()
        if st == target:
            if not quiet:
                print_state(st, det)
            return (st, det)
        msg = f"  Waiting for {target}... (currently {st})"
        if msg != last_msg and not quiet:
            print(msg, end="\r")
            last_msg = msg
        time.sleep(0.5)
    if not quiet:
        print(f"\n  {C_RED}Timeout waiting for {target}{C_RESET}")
    return hub_state()


def open_serial(port, timeout=0.5):
    """Open serial connection to hub."""
    ser = serial.Serial(port, BAUD, timeout=timeout)
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def send_cmd(ser, cmd, wait=1.5):
    """Send command, wait, return response text."""
    ser.reset_input_buffer()
    ser.write(f"{cmd}\r\n".encode())
    time.sleep(wait)
    n = ser.in_waiting or 1
    data = ser.read(max(n, 4096))
    return data.decode("ascii", errors="replace")


def do_flash(bin_path=BIN_PATH):
    """Flash firmware. Sends 'dfu' command if hub is running, then flashes."""
    if not os.path.isfile(bin_path):
        print(f"  {C_RED}Binary not found: {bin_path}{C_RESET}")
        print(f"  Run: cargo build --release && arm-none-eabi-objcopy ...")
        return False

    size = os.path.getsize(bin_path)
    print(f"  Binary: {bin_path} ({size} bytes)")

    st, det = hub_state()

    if st == "serial":
        print(f"  Hub is running — sending 'dfu' command...")
        try:
            ser = open_serial(det)
            send_cmd(ser, "dfu", wait=0.5)
            ser.close()
        except Exception as e:
            print(f"  {C_YELLOW}Warning: couldn't send dfu cmd: {e}{C_RESET}")
        # Wait for DFU mode
        print("  Waiting for DFU mode...")
        time.sleep(2)
        st, det = wait_for_state("dfu", timeout=15)
        if st != "dfu":
            print(f"  {C_RED}Hub didn't enter DFU. Please hold center button.{C_RESET}")
            return False

    elif st == "disconnected":
        print(f"  {C_RED}Hub not found. Power on and hold center button for DFU.{C_RESET}")
        st, _ = wait_for_state("dfu", timeout=30)
        if st != "dfu":
            return False

    # Now in DFU — flash
    print(f"  Flashing {size} bytes to {FLASH_ADDR}...")
    args = DFU_ARGS + [bin_path]
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=30)
        if r.returncode == 0:
            print(f"  {C_GREEN}Flash OK{C_RESET}")
        else:
            # dfu-util prints to stderr
            err = (r.stderr or r.stdout or "").strip()
            print(f"  {C_RED}Flash failed (rc={r.returncode}){C_RESET}")
            if err:
                for line in err.split("\n")[-5:]:
                    print(f"    {line}")
            return False
    except FileNotFoundError:
        print(f"  {C_RED}dfu-util not found. Install: sudo apt install dfu-util{C_RESET}")
        return False
    except subprocess.TimeoutExpired:
        print(f"  {C_RED}dfu-util timed out{C_RESET}")
        return False

    # Wait for reboot
    print("  Waiting for hub to reboot...")
    time.sleep(3)
    st, det = wait_for_state("serial", timeout=20)
    if st == "serial":
        print(f"  {C_GREEN}Hub ready on {det}{C_RESET}")
        return True
    else:
        print(f"  {C_YELLOW}Hub didn't appear as serial yet (state={st}){C_RESET}")
        return True  # flash itself succeeded


def do_cmd(commands, wait_per_cmd=2.0):
    """Send commands to hub. Auto-waits for serial if needed."""
    st, det = hub_state()
    if st == "dfu":
        print(f"  {C_YELLOW}Hub in DFU mode — can't send commands.{C_RESET}")
        return False
    if st == "disconnected":
        print("  Hub disconnected — waiting...")
        st, det = wait_for_state("serial", timeout=20)
    if st != "serial":
        print(f"  {C_RED}Hub not available (state={st}){C_RESET}")
        return False

    try:
        ser = open_serial(det)
    except Exception as e:
        print(f"  {C_RED}Can't open {det}: {e}{C_RESET}")
        return False

    # Drain any boot output
    time.sleep(0.3)
    ser.reset_input_buffer()

    for cmd in commands:
        print(f"  {C_CYAN}> {cmd}{C_RESET}")
        resp = send_cmd(ser, cmd, wait=wait_per_cmd)
        # Strip echo and prompt noise
        lines = resp.replace("\r", "").split("\n")
        for line in lines:
            stripped = line.strip()
            if stripped and stripped != cmd and not stripped.startswith("spike>"):
                print(f"    {stripped}")

    ser.close()
    return True


def cobs_encode(data):
    """COBS-encode data (zero-byte stuffing + sentinel)."""
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


def do_run(bin_path, pre_cmds=None, timeout_s=30):
    """Upload a demo binary and run it. Optionally send commands first (e.g. 'sensor e')."""
    if not os.path.isfile(bin_path):
        print(f"  {C_RED}Binary not found: {bin_path}{C_RESET}")
        return False

    with open(bin_path, "rb") as f:
        data = f.read()
    print(f"  Binary: {bin_path} ({len(data)} bytes)")

    st, det = hub_state()
    if st != "serial":
        print("  Waiting for serial...")
        st, det = wait_for_state("serial", timeout=20)
    if st != "serial":
        print(f"  {C_RED}Hub not available{C_RESET}")
        return False

    try:
        ser = open_serial(det)
    except Exception as e:
        print(f"  {C_RED}Can't open {det}: {e}{C_RESET}")
        return False

    # Get prompt
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"\r\n")
    time.sleep(0.5)
    ser.read(ser.in_waiting or 1)

    # Upload
    ser.reset_input_buffer()
    ser.write(f"upload {len(data)}\r\n".encode())
    time.sleep(1)
    resp = ser.read(ser.in_waiting or 1).decode("ascii", errors="replace")
    if "READY" not in resp:
        print(f"  {C_RED}Upload failed: {resp.strip()}{C_RESET}")
        ser.close()
        return False

    encoded = cobs_encode(data)
    ser.write(encoded)
    time.sleep(1)
    resp = ser.read(ser.in_waiting or 1).decode("ascii", errors="replace")
    ok_line = [l for l in resp.split("\n") if "OK" in l]
    print(f"  {C_GREEN}Uploaded: {ok_line[0].strip() if ok_line else resp.strip()}{C_RESET}")
    if "OK" not in resp:
        ser.close()
        return False

    # Pre-commands (e.g. 'sensor e') with wait
    if pre_cmds:
        for cmd in pre_cmds:
            print(f"  {C_CYAN}> {cmd}{C_RESET}")
            r = send_cmd(ser, cmd, wait=2.0)
            for line in r.replace("\r", "").split("\n"):
                s = line.strip()
                if s and s != cmd and not s.startswith("spike>"):
                    print(f"    {s}")
        # Wait for sensor sync if needed
        if any("sensor" in c for c in pre_cmds):
            print("  Waiting for sensor sync...")
            time.sleep(8)

    # Run
    ser.reset_input_buffer()
    ser.write(b"go\r\n")
    print(f"  {C_GREEN}Running ({timeout_s}s, Ctrl-C to stop){C_RESET}")

    deadline = time.time() + timeout_s
    try:
        while time.time() < deadline:
            n = ser.in_waiting
            if n > 0:
                chunk = ser.read(n).decode("ascii", errors="replace")
                sys.stdout.write(chunk)
                sys.stdout.flush()
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print(f"\n  {C_YELLOW}Interrupted{C_RESET}")

    time.sleep(0.2)
    n = ser.in_waiting
    if n > 0:
        sys.stdout.write(ser.read(n).decode("ascii", errors="replace"))
    ser.close()
    return True


def do_monitor():
    """Continuous output monitor — shows everything from hub."""
    st, det = hub_state()
    if st != "serial":
        print("  Waiting for serial...")
        st, det = wait_for_state("serial", timeout=30)
    if st != "serial":
        print(f"  {C_RED}Hub not available{C_RESET}")
        return

    ser = open_serial(det, timeout=0.1)
    print(f"  {C_GREEN}Monitoring {det} — Ctrl-C to stop{C_RESET}")
    try:
        while True:
            data = ser.read(1024)
            if data:
                sys.stdout.write(data.decode("ascii", errors="replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        print(f"\n  {C_YELLOW}Stopped.{C_RESET}")
    finally:
        ser.close()


def do_flash_and_cmd(commands, wait_per_cmd=2.0):
    """Flash firmware, wait for reboot, then send commands."""
    if not do_flash():
        return False
    # Extra settle time after reboot
    time.sleep(1)
    return do_cmd(commands, wait_per_cmd)


def main():
    args = sys.argv[1:]

    if not args:
        # Just show state
        st, det = hub_state()
        print_state(st, det)
        return

    verb = args[0].lower()

    if verb == "state" or verb == "status":
        st, det = hub_state()
        print_state(st, det)

    elif verb == "flash":
        bin_path = args[1] if len(args) > 1 else BIN_PATH
        do_flash(bin_path)

    elif verb == "cmd":
        if len(args) < 2:
            print("Usage: hub.py cmd <command> [command2] ...")
            return
        do_cmd(args[1:])

    elif verb == "flash+cmd":
        if len(args) < 2:
            print("Usage: hub.py flash+cmd <command> [command2] ...")
            return
        do_flash_and_cmd(args[1:])

    elif verb == "wait":
        target = args[1] if len(args) > 1 else "serial"
        wait_for_state(target, timeout=60)

    elif verb == "monitor" or verb == "mon":
        do_monitor()

    elif verb == "run":
        # hub.py run <binfile> [timeout] [--pre "cmd1" "cmd2" ...]
        if len(args) < 2:
            print("Usage: hub.py run <binfile> [timeout] [--pre cmd1 cmd2 ...]")
            return
        bin_path = args[1]
        timeout_s = 30
        pre_cmds = []
        i = 2
        while i < len(args):
            if args[i] == "--pre":
                pre_cmds = args[i+1:]
                break
            else:
                try:
                    timeout_s = int(args[i])
                except ValueError:
                    pass
            i += 1
        do_run(bin_path, pre_cmds if pre_cmds else None, timeout_s)

    else:
        print(f"Unknown verb: {verb}")
        print("Usage: hub.py [state|flash|cmd|flash+cmd|wait|monitor|run]")


if __name__ == "__main__":
    main()
