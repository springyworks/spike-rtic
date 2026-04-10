#!/usr/bin/env python3
"""hub_state.py -- Detect SPIKE Prime hub connection state.

Usage:
    python3 helper-tools/hub_state.py              # print state + exit
    python3 helper-tools/hub_state.py gdb           # enter RSP + launch gdb-multiarch
    python3 helper-tools/hub_state.py gdb-reconnect  # reconnect to hub already in RSP
    python3 helper-tools/hub_state.py gdb ELF_PATH   # enter RSP + gdb with symbols

States detected:
    0  disconnected   -- no LEGO USB device found
    1  dfu            -- hub in DFU bootloader (VID:PID 0694:0011)
    2  serial-shell   -- hub running, shell responds to \\r\\n
    3  serial-gdb     -- hub running, shell in GDB RSP mode (no prompt)
    4  serial-unknown -- hub serial present but no response (busy?)
    5  serial-busy    -- port exists but cannot open (another program has it)

Exit codes match the state number, so scripts can branch on $?.

Requires: pyserial  (pip install pyserial)
"""

import os
import sys
import subprocess
import time

# ── Project root: $PROJECT_ROOT or derived from this script's location ──
PROJECT_ROOT = os.environ.get(
    "PROJECT_ROOT",
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("FATAL: pyserial not installed.  pip install pyserial", file=sys.stderr)
    sys.exit(99)

# Shared port detection (prefer symlinks → sysfs → VID:PID scan)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spike_port import (
    find_shell_port, find_gdb_port as find_gdb_port_,
    is_dfu_present, free_port, open_serial,
    LEGO_VID, DFU_PID, RUNTIME_PID, BAUD,
    hub_status as print_port_table,
)

# ── Constants ──
PROBE_TIMEOUT = 0.8   # seconds to wait for shell prompt


def find_lego_serial():
    """Return shell port path if LEGO runtime CDC is present, else None."""
    return find_shell_port()


def probe_serial(port):
    """Open the serial port, send two CRs, classify what comes back.

    Returns one of:
        'shell'   -- got 'spike>' prompt back
        'gdb'     -- got RSP-like response ('+' or '$') or silence
        'unknown' -- got something else
        'busy'    -- couldn't open port (locked by another process)
    """
    try:
        ser = serial.Serial(port, BAUD, timeout=PROBE_TIMEOUT)
    except serial.SerialException:
        return "busy"

    try:
        ser.reset_input_buffer()

        # Send two bare CRs to prod the shell
        ser.write(b"\r\n\r\n")
        ser.flush()

        # Wait for response
        time.sleep(PROBE_TIMEOUT)
        n = ser.in_waiting
        if n == 0:
            # No response at all -- likely GDB RSP mode (it only
            # responds to '$' framed packets, ignores bare text)
            return "gdb"

        data = ser.read(n)
        text = data.decode("ascii", errors="replace")

        if "spike>" in text:
            return "shell"

        # RSP stub sends '+' for ACK or '$..#..' packets
        if data[:1] == b"+" or data[:1] == b"$":
            return "gdb"

        return "unknown"
    finally:
        ser.close()


def detect_state():
    """Return (state_name, state_code, port_or_None)."""
    port = find_lego_serial()
    if port:
        kind = probe_serial(port)
        if kind == "shell":
            return ("serial-shell", 2, port)
        elif kind == "gdb":
            return ("serial-gdb", 3, port)
        elif kind == "busy":
            return ("serial-busy", 5, port)
        else:
            return ("serial-unknown", 4, port)

    if is_dfu_present():
        return ("dfu", 1, None)

    return ("disconnected", 0, None)


def enter_rsp(port):
    """Send 'gdb' command to switch hub from shell to RSP mode."""
    try:
        ser = open_serial(port, BAUD, timeout=1)
        ser.reset_input_buffer()
        ser.write(b"gdb\r\n")
        ser.flush()
        time.sleep(0.4)
        resp = ser.read(ser.in_waiting or 1)
        ser.close()
        return resp
    except serial.SerialException as e:
        print(f"  Error sending 'gdb' command: {e}", file=sys.stderr)
        return None


def launch_gdb(port, elf_path=None):
    """Launch gdb-multiarch connected to the hub in RSP mode."""
    args = ["gdb-multiarch", "-q"]
    if elf_path:
        args.append(elf_path)
    args += [
        "-ex", "set pagination off",
        "-ex", "set architecture arm",
        "-ex", f"target remote {port}",
    ]
    print(f"  Launching: {' '.join(args)}")
    os.execvp("gdb-multiarch", args)


def kill_port_holder(port):
    """Find and kill the process holding `port`. Delegates to spike_port.free_port()."""
    return free_port(port)


# ── Main ──

def main():
    action = sys.argv[1] if len(sys.argv) > 1 else None
    elf_path = sys.argv[2] if len(sys.argv) > 2 else None

    state, code, port = detect_state()

    # ── Auto-kill blocking process if port is busy ──
    if state == "serial-busy" and port:
        print(f"  Port {port} is busy. Killing blocking process...")
        if kill_port_holder(port):
            # Re-probe after killing
            state, code, port = detect_state()
            print(f"  Re-probed: {state}")
        else:
            print(f"  Could not free {port}. Manual intervention needed.")

    # ── Just detect ──
    if action is None:
        # Show detailed port table first
        print_port_table()

        labels = {
            0: "\033[91mDisconnected\033[0m  -- no LEGO USB device",
            1: "\033[93mDFU mode\033[0m      -- ready to flash (0694:0011)",
            2: "\033[92mShell ready\033[0m   -- spike> prompt on ",
            3: "\033[96mGDB RSP mode\033[0m  -- Demon mode active on ",
            4: "\033[93mUnknown\033[0m       -- serial present, no prompt on ",
            5: "\033[93mPort busy\033[0m     -- serial locked by another program on ",
        }
        msg = labels[code]
        if port:
            msg += port
        print(f"  Hub state: {msg}")
        sys.exit(code)

    # ── Enter RSP + launch GDB ──
    if action == "gdb":
        if state == "disconnected":
            print("  Hub not connected. Plug in USB and power on.")
            sys.exit(0)
        if state == "dfu":
            print("  Hub is in DFU mode. Flash firmware first.")
            sys.exit(1)
        if state == "serial-gdb":
            print(f"  Hub already in GDB RSP mode on {port}.")
            launch_gdb(port, elf_path)
        if state == "serial-shell":
            print(f"  Hub in shell mode on {port}. Sending 'gdb' command...")
            resp = enter_rsp(port)
            if resp:
                print(f"  RSP entered. Response: {resp!r}")
            # Brief pause for mode switch
            time.sleep(0.2)
            launch_gdb(port, elf_path)
        # serial-unknown -- try entering RSP anyway
        print(f"  Hub state unclear ({state}). Attempting 'gdb' command...")
        enter_rsp(port)
        time.sleep(0.2)
        launch_gdb(port, elf_path)

    # ── Reconnect to hub already in RSP mode ──
    if action == "gdb-reconnect":
        if not port:
            print("  Hub not connected.")
            sys.exit(0)
        print(f"  Reconnecting to GDB RSP on {port}...")
        launch_gdb(port, elf_path)

    print(f"Unknown action: {action}")
    print("Usage: hub_state.py [gdb [ELF_PATH] | gdb-reconnect [ELF_PATH]]")
    sys.exit(1)


if __name__ == "__main__":
    main()
