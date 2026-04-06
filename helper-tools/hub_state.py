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

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("FATAL: pyserial not installed.  pip install pyserial", file=sys.stderr)
    sys.exit(99)

# ── Constants ──
LEGO_VID    = 0x0694
DFU_PID     = 0x0011
RUNTIME_PID = 0x0042
BAUD        = 115200
PROBE_TIMEOUT = 0.8   # seconds to wait for shell prompt


def find_lego_serial():
    """Return port path if LEGO runtime CDC is present, else None."""
    for p in serial.tools.list_ports.comports():
        if (p.vid or 0) == LEGO_VID and (p.pid or 0) == RUNTIME_PID:
            return p.device
    return None


def is_dfu_present():
    """Check lsusb for LEGO DFU device."""
    try:
        out = subprocess.check_output(["lsusb"], text=True, timeout=5)
        return f"{LEGO_VID:04x}:{DFU_PID:04x}" in out.lower()
    except Exception:
        return False


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
        ser = serial.Serial(port, BAUD, timeout=1)
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
    """Find and kill -9 the process holding `port`. Returns True if killed."""
    import signal
    try:
        out = subprocess.check_output(["fuser", port], stderr=subprocess.STDOUT, text=True, timeout=5)
    except FileNotFoundError:
        # fuser not installed — try lsof
        try:
            out = subprocess.check_output(["lsof", "-t", port], text=True, timeout=5)
        except Exception:
            print(f"  Cannot find process holding {port} (install fuser or lsof).")
            return False
    except subprocess.CalledProcessError:
        print(f"  No process found holding {port}.")
        return False
    except subprocess.TimeoutExpired:
        return False

    pids = [int(p) for p in out.split() if p.strip().isdigit()]
    if not pids:
        print(f"  No PIDs found for {port}.")
        return False

    my_pid = os.getpid()
    for pid in pids:
        if pid == my_pid:
            continue
        # Identify what we're killing
        try:
            cmdline = subprocess.check_output(
                ["ps", "-p", str(pid), "-o", "comm="], text=True, timeout=3
            ).strip()
        except Exception:
            cmdline = "?"
        print(f"  Killing PID {pid} ({cmdline}) holding {port}...")
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass  # already gone
        except PermissionError:
            print(f"  Permission denied killing PID {pid}. Try: sudo kill -9 {pid}")
            return False

    # Wait for port to become available
    time.sleep(0.5)
    return True


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
