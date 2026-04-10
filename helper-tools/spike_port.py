"""spike_port — Unified SPIKE Prime hub port detection.

Single source of truth for finding shell, GDB, and DFU ports across
all Python helper tools.

Detection priority (fastest & most reliable first):
  1. Udev symlinks:  /dev/spike-shell, /dev/spike-gdb
  2. Sysfs scan:     VID:PID 0694:0042 + bInterfaceNumber (00=shell, 02=gdb)
  3. pyserial scan:  VID:PID match (any interface — legacy fallback)
  4. Brute-force:    /dev/ttyACM* probe (last resort)

Usage:
    from spike_port import find_shell_port, find_gdb_port, hub_status

    port = find_shell_port()          # → "/dev/spike-shell" or "/dev/ttyACM0" etc.
    gdb  = find_gdb_port()            # → "/dev/spike-gdb" or "/dev/ttyACM1" etc.
    hub_status()                      # print diagnostic table
"""

import glob
import os
import subprocess
import sys

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # degrade gracefully — sysfs/symlink paths still work

# ── Constants ────────────────────────────────────────────────
LEGO_VID = 0x0694
DFU_PID = 0x0011
RUNTIME_PID = 0x0042
BAUD = 115200

# Udev symlinks (installed from helper-tools/99-spike-hub.rules)
SHELL_SYMLINK = "/dev/spike-shell"
GDB_SYMLINK = "/dev/spike-gdb"

# Interface numbers in the dual-CDC composite descriptor
SHELL_IFACE = "00"  # CDC function 0 → comm interface 0
GDB_IFACE = "02"  # CDC function 1 → comm interface 2

# ── ANSI helpers ─────────────────────────────────────────────
_C_RED = "\033[91m"
_C_GREEN = "\033[92m"
_C_YELLOW = "\033[93m"
_C_CYAN = "\033[96m"
_C_DIM = "\033[2m"
_C_BOLD = "\033[1m"
_C_RESET = "\033[0m"


# ── Core detection functions ─────────────────────────────────

def _sysfs_ports():
    """Scan sysfs for ttyACM devices with LEGO VID:PID.

    Returns list of (dev_path, iface_num) tuples, e.g.:
      [("/dev/ttyACM0", "00"), ("/dev/ttyACM1", "02")]
    """
    results = []
    tty_class = "/sys/class/tty"
    if not os.path.isdir(tty_class):
        return results
    for name in sorted(os.listdir(tty_class)):
        if not name.startswith("ttyACM"):
            continue
        base = f"{tty_class}/{name}/device/.."
        try:
            vid = open(f"{base}/idVendor").read().strip()
            pid = open(f"{base}/idProduct").read().strip()
        except (OSError, IOError):
            continue
        if vid != "0694" or pid != "0042":
            continue
        try:
            iface = open(f"{tty_class}/{name}/device/bInterfaceNumber").read().strip()
        except (OSError, IOError):
            iface = "??"
        results.append((f"/dev/{name}", iface))
    return results


def _find_by_interface(target_iface):
    """Find ttyACM port for a specific USB interface number.

    Args:
        target_iface: "00" for shell, "02" for GDB RSP

    Returns:
        Port path string or None.
    """
    # 1. Check udev symlink (fastest, most reliable)
    symlink = SHELL_SYMLINK if target_iface == SHELL_IFACE else GDB_SYMLINK
    if os.path.exists(symlink):
        return symlink

    # 2. Sysfs scan (works without udev rules installed)
    for dev, iface in _sysfs_ports():
        if iface == target_iface:
            return dev

    return None


def find_shell_port():
    """Find the shell serial port (CDC interface 0).

    Returns port path string, or None if not found.
    Priority: symlink → sysfs → pyserial VID:PID → first ttyACM*.
    """
    # Interface-aware detection
    port = _find_by_interface(SHELL_IFACE)
    if port:
        return port

    # Legacy fallback: any port with matching VID:PID
    if serial:
        for p in serial.tools.list_ports.comports():
            if (p.vid or 0) == LEGO_VID and (p.pid or 0) == RUNTIME_PID:
                return p.device

    return None


def find_gdb_port():
    """Find the GDB RSP serial port (CDC interface 2).

    Returns port path string, or None if not found.
    """
    return _find_by_interface(GDB_IFACE)


def find_hub_port():
    """Backward-compatible alias for find_shell_port().

    All existing tools that called find_hub_port() wanted the shell port.
    """
    return find_shell_port()


def is_dfu_present():
    """Check lsusb for LEGO DFU device (VID:PID 0694:0011)."""
    try:
        out = subprocess.check_output(["lsusb"], text=True, timeout=5)
        return f"{LEGO_VID:04x}:{DFU_PID:04x}" in out.lower()
    except Exception:
        return False


# ── Port-kicker ──────────────────────────────────────────────

def _find_port_holders(port):
    """Find PIDs holding `port` open (via fuser, then lsof fallback).

    Returns list of (pid, cmdline) tuples, excluding our own PID.
    """
    import signal
    my_pid = os.getpid()
    holders = []

    # Try fuser first
    try:
        out = subprocess.check_output(
            ["fuser", port], stderr=subprocess.DEVNULL, text=True, timeout=5
        )
        for tok in out.split():
            tok = tok.strip().rstrip("me")
            try:
                pid = int(tok)
            except ValueError:
                continue
            if pid != my_pid:
                holders.append(pid)
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        pass

    if not holders:
        # Fallback: lsof
        try:
            out = subprocess.check_output(
                ["lsof", "-t", port], stderr=subprocess.DEVNULL, text=True, timeout=5
            )
            for line in out.splitlines():
                try:
                    pid = int(line.strip())
                except ValueError:
                    continue
                if pid != my_pid:
                    holders.append(pid)
        except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
            pass

    # Resolve command names
    result = []
    for pid in holders:
        try:
            cmd = subprocess.check_output(
                ["ps", "-p", str(pid), "-o", "comm="], text=True, timeout=3
            ).strip()
        except Exception:
            cmd = "?"
        result.append((pid, cmd))
    return result


def free_port(port):
    """Kill any process holding `port`. SIGTERM first, SIGKILL if needed.

    Returns True if port is now free, False if we couldn't free it.
    """
    import signal
    import time

    holders = _find_port_holders(port)
    if not holders:
        return True

    print(f"  {_C_YELLOW}Port {port} locked by:{_C_RESET}", file=sys.stderr)
    for pid, cmd in holders:
        print(f"    PID {pid}: {cmd}", file=sys.stderr)

    # SIGTERM first
    for pid, _ in holders:
        try:
            os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(0.8)

    # Check survivors → SIGKILL
    remaining = _find_port_holders(port)
    if remaining:
        for pid, _ in remaining:
            print(f"    {_C_DIM}SIGKILL PID {pid}{_C_RESET}", file=sys.stderr)
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(0.5)

    still = _find_port_holders(port)
    if not still:
        print(f"  {_C_GREEN}✓ Port freed{_C_RESET}", file=sys.stderr)
        return True
    else:
        print(f"  {_C_RED}✗ Could not free {port}{_C_RESET}", file=sys.stderr)
        return False


def open_serial(port, baud=BAUD, timeout=2, auto_free=True):
    """Open a serial port, auto-killing any holder if needed.

    Args:
        port: Device path (e.g. "/dev/spike-shell")
        baud: Baud rate (default 115200)
        timeout: Read timeout in seconds
        auto_free: If True, kill port holder on busy and retry

    Returns:
        serial.Serial instance

    Raises:
        serial.SerialException if port can't be opened even after freeing
    """
    import time
    try:
        return serial.Serial(port, baud, timeout=timeout)
    except serial.SerialException:
        if not auto_free:
            raise
        # Port is busy — kick the holder and retry
        if free_port(port):
            time.sleep(0.3)
            return serial.Serial(port, baud, timeout=timeout)
        raise


# ── Diagnostic / status ──────────────────────────────────────

def list_all_ports():
    """Return diagnostic info about all USB serial ports.

    Returns list of dicts:
      [{"dev": "/dev/ttyACM0", "vid": "0694", "pid": "0042",
        "iface": "00", "role": "shell", "symlink": "/dev/spike-shell",
        "product": "Spike Prime RTIC", "busy": False}, ...]
    """
    results = []

    # Gather sysfs info for all ttyACM*
    tty_class = "/sys/class/tty"
    acm_devs = sorted(glob.glob("/dev/ttyACM*"))

    for dev in acm_devs:
        name = os.path.basename(dev)
        info = {"dev": dev, "vid": "????", "pid": "????", "iface": "??",
                "role": "unknown", "symlink": None, "product": "", "busy": False}

        # Read sysfs
        base = f"{tty_class}/{name}/device/.."
        try:
            info["vid"] = open(f"{base}/idVendor").read().strip()
            info["pid"] = open(f"{base}/idProduct").read().strip()
        except (OSError, IOError):
            pass
        try:
            info["iface"] = open(f"{tty_class}/{name}/device/bInterfaceNumber").read().strip()
        except (OSError, IOError):
            pass
        try:
            info["product"] = open(f"{base}/product").read().strip()
        except (OSError, IOError):
            pass

        # Classify role
        if info["vid"] == "0694" and info["pid"] == "0042":
            if info["iface"] == "00":
                info["role"] = "shell"
            elif info["iface"] == "02":
                info["role"] = "gdb"
            else:
                info["role"] = "spike?"

        # Check symlinks
        for sym in (SHELL_SYMLINK, GDB_SYMLINK):
            try:
                if os.path.realpath(sym) == os.path.realpath(dev):
                    info["symlink"] = sym
            except OSError:
                pass

        # Check if port is busy (can we open it?)
        if serial:
            try:
                s = serial.Serial(dev, BAUD, timeout=0.1)
                s.close()
            except serial.SerialException:
                info["busy"] = True

        results.append(info)

    return results


def hub_status(file=None):
    """Print a diagnostic table of hub connection state.

    Shows all ttyACM ports, their USB identity, interface role, symlink
    status, and whether they're busy. Also checks DFU mode.
    """
    if file is None:
        file = sys.stderr

    ports = list_all_ports()
    dfu = is_dfu_present()

    print(f"\n  {_C_BOLD}SPIKE Hub Connection Status{_C_RESET}", file=file)
    print(f"  {'─' * 62}", file=file)

    if not ports and not dfu:
        print(f"  {_C_RED}No ttyACM ports found. Hub disconnected?{_C_RESET}", file=file)
        # Check symlinks anyway
        for sym in (SHELL_SYMLINK, GDB_SYMLINK):
            if os.path.islink(sym):
                target = os.readlink(sym)
                print(f"  {_C_YELLOW}Stale symlink: {sym} → {target} (target missing){_C_RESET}",
                      file=file)
        print(file=file)
        return

    if dfu:
        print(f"  {_C_YELLOW}DFU mode detected{_C_RESET} (VID:PID 0694:0011)", file=file)
        print(file=file)

    if ports:
        # Header
        print(f"  {'Device':<16} {'VID:PID':<12} {'IF':<4} {'Role':<8} "
              f"{'Symlink':<20} {'Status':<10} {'Product'}", file=file)
        print(f"  {'─'*16} {'─'*11} {'─'*3} {'─'*7} "
              f"{'─'*19} {'─'*9} {'─'*20}", file=file)

        for p in ports:
            vid_pid = f"{p['vid']}:{p['pid']}"

            # Color-code role
            role = p["role"]
            if role == "shell":
                role_str = f"{_C_GREEN}{role}{_C_RESET}"
            elif role == "gdb":
                role_str = f"{_C_CYAN}{role}{_C_RESET}"
            else:
                role_str = f"{_C_DIM}{role}{_C_RESET}"

            sym = p["symlink"] or ""
            status = f"{_C_RED}BUSY{_C_RESET}" if p["busy"] else f"{_C_GREEN}free{_C_RESET}"
            product = p["product"]

            # Pad role_str accounting for ANSI codes
            role_pad = 8 + len(role_str) - len(role)
            print(f"  {p['dev']:<16} {vid_pid:<12} {p['iface']:<4} {role_str:<{role_pad}} "
                  f"{sym:<20} {status:<19} {product}", file=file)

    # Check for missing symlinks
    for sym, expected_role in [(SHELL_SYMLINK, "shell"), (GDB_SYMLINK, "gdb")]:
        linked = any(p["symlink"] == sym for p in ports)
        if not linked:
            spike_ports = [p for p in ports if p["vid"] == "0694"]
            if spike_ports:
                print(f"\n  {_C_YELLOW}⚠  {sym} not found. "
                      f"Install udev rules:{_C_RESET}", file=file)
                print(f"     sudo cp helper-tools/99-spike-hub.rules "
                      f"/etc/udev/rules.d/", file=file)
                print(f"     sudo udevadm control --reload-rules && "
                      f"sudo udevadm trigger", file=file)

    print(file=file)


# ── CLI entry point ──────────────────────────────────────────

if __name__ == "__main__":
    hub_status(file=sys.stdout)

    shell = find_shell_port()
    gdb = find_gdb_port()
    dfu = is_dfu_present()

    if shell:
        print(f"  Shell port: {_C_GREEN}{shell}{_C_RESET}")
    if gdb:
        print(f"  GDB port:   {_C_CYAN}{gdb}{_C_RESET}")
    if dfu:
        print(f"  DFU:        {_C_YELLOW}present{_C_RESET}")
    if not shell and not gdb and not dfu:
        print(f"  {_C_RED}Hub not detected{_C_RESET}")
    print()
