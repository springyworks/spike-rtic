#!/usr/bin/env python3
"""
spike_hub_controller.py  (shbc)
Unified host-side controller for SPIKE Prime RTIC hub.

Merges and replaces (deleted):
  - hub_state.py     (fuzzy state diagnostic → `diag` mode)
  - spike_hub.py     (interactive shell, upload → `shell`/`upload` modes)
  - test_hub_mon.py  (automated test runner → `test` mode)
  - test_cli.py      (single command → `cmd` mode)
  - test_go.py       (trivial go runner → `upload_demo.py`)

Features:
  - Singleton instance (lockfile /tmp/shbc.lock)
  - DFU firmware build + flash (cargo build + dfu-util)
  - Terminal-bell attention system for physical human actions
  - ttyACM* port-conflict detection and resolution (fuser/kill)
  - Environment-change awareness (USB hotplug, port drift, stale state)
  - Optional local Ollama LLM integration for smart decision making
  - Continuous ttyACM* monitoring with ASCII cursor-controlled TUI
  - Status bar with live hub state, uptime, battery, button readings
  - Command history, reconnect, hot-plug detection

Usage:
  python3 spike_hub_controller.py                    -- TUI mode (continuous)
  python3 spike_hub_controller.py shell              -- interactive shell
  python3 spike_hub_controller.py flash              -- build + DFU flash
  python3 spike_hub_controller.py upload firmware.bin -- upload binary
  python3 spike_hub_controller.py test               -- run test suite
  python3 spike_hub_controller.py diag               -- one-shot diagnostic
  python3 spike_hub_controller.py cmd "info"          -- single command

Requirements: pip install pyserial
Optional:     ollama with llama3.1 model (for LLM-assisted decisions)
"""

import sys
import os
import time
import glob
import struct
import signal
import errno
import re
import binascii
import subprocess
import select
import termios
import tty
import fcntl
import threading
import atexit
import json
import hashlib

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("FATAL: pyserial not installed.  Run: pip install pyserial")
    sys.exit(1)


# ============================================================
#  Constants
# ============================================================

BAUD = 115200
READ_TIMEOUT = 0.3
PROMPT = b"spike> "
PROMPT_STR = "spike> "
VERSION = "0.3.0"

# LEGO / STM32 USB identifiers
LEGO_VID = 0x0694
LEGO_DFU_PID = 0x0011
LEGO_RTIC_PID = 0x0042
STM32_DFU_VID = 0x0483
STM32_DFU_PID = 0xDF11

# Upload limits
MAX_UPLOAD = 64 * 1024

# Hub states
ST_DISCONNECTED = "DISCONNECTED"
ST_DFU = "DFU_MODE"
ST_BOOTING = "BOOTING"
ST_ALIVE = "ALIVE"
ST_STALLED = "CDC_STALLED"
ST_UNKNOWN = "UNKNOWN"

# Singleton lockfile
LOCKFILE = "/tmp/shbc.lock"

# Firmware paths (relative to script directory)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FW_ELF_REL = "target/thumbv7em-none-eabihf/release/spike-rtic"
FW_BIN_TMP = "/tmp/spike-rtic.bin"
DFU_FLASH_ADDR = "0x08008000"
DFU_VID_PID = "0694:0011"

# Ollama
OLLAMA_URL = "http://localhost:11434"
OLLAMA_MODEL = "llama3.1:8b"

# ANSI escape helpers (ASCII-only, no unicode box-drawing)
ESC = "\033"
CSI = ESC + "["
BEL = "\x07"


# ============================================================
#  ANSI terminal helpers (cursor control, all ASCII)
# ============================================================

def term_clear():
    sys.stdout.write(CSI + "2J" + CSI + "H")
    sys.stdout.flush()

def term_goto(row, col):
    sys.stdout.write(CSI + f"{row};{col}H")

def term_clear_line():
    sys.stdout.write(CSI + "2K")

def term_save_cursor():
    sys.stdout.write(ESC + "7")

def term_restore_cursor():
    sys.stdout.write(ESC + "8")

def term_hide_cursor():
    sys.stdout.write(CSI + "?25l")

def term_show_cursor():
    sys.stdout.write(CSI + "?25h")

def term_bold(text):
    return CSI + "1m" + text + CSI + "0m"

def term_dim(text):
    return CSI + "2m" + text + CSI + "0m"

def term_reverse(text):
    return CSI + "7m" + text + CSI + "0m"

def term_color(text, fg):
    """fg: 0=black 1=red 2=green 3=yellow 4=blue 5=magenta 6=cyan 7=white"""
    return CSI + f"3{fg}m" + text + CSI + "0m"

def term_size():
    """Return (rows, cols) of the terminal."""
    try:
        sz = os.get_terminal_size()
        return sz.lines, sz.columns
    except OSError:
        return 24, 80


# ============================================================
#  Singleton instance guard
# ============================================================

def _read_lockfile():
    """Return PID from lockfile, or None."""
    try:
        with open(LOCKFILE, "r") as f:
            return int(f.read().strip())
    except (FileNotFoundError, ValueError, OSError):
        return None


def _pid_alive(pid):
    """Check if a PID is running."""
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def acquire_singleton():
    """
    Ensure only one shbc instance is running.
    Returns True if we got the lock, False if another instance is live.
    """
    existing = _read_lockfile()
    if existing is not None and existing != os.getpid():
        if _pid_alive(existing):
            return False
    # Write our PID
    try:
        with open(LOCKFILE, "w") as f:
            f.write(str(os.getpid()))
        atexit.register(_release_singleton)
        return True
    except OSError:
        return True  # best-effort


def _release_singleton():
    """Remove lockfile on exit."""
    try:
        pid = _read_lockfile()
        if pid == os.getpid():
            os.unlink(LOCKFILE)
    except OSError:
        pass


# ============================================================
#  Terminal bell attention (physical human action needed)
# ============================================================

def ring_attention(message, timeout=120):
    """
    Ring terminal bell and display message until human presses Enter.
    Used when a physical action is needed (remove battery, press button, etc).
    Returns when human acknowledges.
    """
    print(f"\n{'=' * 60}")
    print(f"  >>> HUMAN ATTENTION REQUIRED <<<")
    print(f"  {message}")
    print(f"{'=' * 60}")
    print("  Press ENTER when done...")

    start = time.time()
    # Ring bell every 2 seconds
    while True:
        sys.stdout.write(BEL)
        sys.stdout.flush()
        # Non-blocking check for Enter with 2s intervals
        if sys.stdin in select.select([sys.stdin], [], [], 2.0)[0]:
            sys.stdin.readline()
            print("  OK, continuing...\n")
            return True
        if time.time() - start > timeout:
            print("  Timeout waiting for human response.")
            return False


def ring_attention_raw(message, fd_in, timeout=120):
    """
    Ring bell in raw terminal mode. Reads single byte from fd_in.
    Returns True when Enter pressed, False on timeout.
    """
    sys.stdout.write(f"\r\n{'=' * 50}\r\n")
    sys.stdout.write(f"  >>> HUMAN ATTENTION <<<\r\n")
    sys.stdout.write(f"  {message}\r\n")
    sys.stdout.write(f"  Press ENTER when done...\r\n")
    sys.stdout.write(f"{'=' * 50}\r\n")
    sys.stdout.flush()

    start = time.time()
    while time.time() - start < timeout:
        sys.stdout.write(BEL)
        sys.stdout.flush()
        rlist, _, _ = select.select([fd_in], [], [], 2.0)
        if rlist:
            try:
                b = os.read(fd_in if isinstance(fd_in, int) else fd_in.fileno(), 16)
                if b'\r' in b or b'\n' in b or b == b'\r' or b == b'\n':
                    sys.stdout.write("  OK, continuing...\r\n")
                    sys.stdout.flush()
                    return True
            except OSError:
                pass
    return False


# ============================================================
#  Port conflict detection and resolution
# ============================================================

def find_port_users(port):
    """
    Find processes that have a tty port open.
    Returns list of (pid, cmdline) tuples.
    """
    users = []
    if not port or not os.path.exists(port):
        return users

    # Try fuser first
    try:
        out = subprocess.check_output(
            ["fuser", port], stderr=subprocess.DEVNULL, text=True, timeout=5
        )
        for tok in out.split():
            tok = tok.strip().rstrip("m").rstrip("e")
            if tok.isdigit():
                pid = int(tok)
                if pid == os.getpid():
                    continue
                cmd = _pid_cmdline(pid)
                users.append((pid, cmd))
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass

    if users:
        return users

    # Fallback: lsof
    try:
        out = subprocess.check_output(
            ["lsof", "-t", port], stderr=subprocess.DEVNULL, text=True, timeout=5
        )
        for line in out.strip().split("\n"):
            line = line.strip()
            if line.isdigit():
                pid = int(line)
                if pid == os.getpid():
                    continue
                cmd = _pid_cmdline(pid)
                users.append((pid, cmd))
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass

    return users


def _pid_cmdline(pid):
    """Read /proc/<pid>/cmdline, return as string."""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            raw = f.read(512)
        return raw.replace(b"\x00", b" ").decode(errors="replace").strip()
    except OSError:
        return "?"


def kill_port_users(port, ask=True):
    """
    Find and optionally kill processes blocking a port.
    Returns True if port is now free.
    """
    users = find_port_users(port)
    if not users:
        return True

    print(f"\n  Port {port} is in use by:")
    for pid, cmd in users:
        print(f"    PID {pid}: {cmd[:60]}")

    if ask:
        ans = input("  Kill these processes? [y/N] ").strip().lower()
        if ans not in ("y", "yes"):
            print("  Skipped.")
            return False

    for pid, cmd in users:
        try:
            os.kill(pid, signal.SIGTERM)
            print(f"  Sent SIGTERM to PID {pid}")
        except (ProcessLookupError, PermissionError) as e:
            print(f"  Cannot kill PID {pid}: {e}")

    time.sleep(1.0)

    # Check if any survived
    remaining = find_port_users(port)
    if remaining:
        for pid, _ in remaining:
            try:
                os.kill(pid, signal.SIGKILL)
                print(f"  Sent SIGKILL to PID {pid}")
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(0.5)

    return not find_port_users(port)


# ============================================================
#  Environment snapshot & change detection
# ============================================================

class EnvSnapshot:
    """
    Capture a snapshot of USB and ttyACM state.
    Compare snapshots to detect human intervention or hotplug events.
    """
    def __init__(self):
        self.timestamp = time.time()
        self.acm_ports = sorted(glob.glob("/dev/ttyACM*"))
        self.usb_mode = detect_usb_mode()
        self.lsusb_hash = self._lsusb_hash()

    @staticmethod
    def _lsusb_hash():
        try:
            out = subprocess.check_output(["lsusb"], text=True, timeout=5)
            return hashlib.md5(out.encode()).hexdigest()[:12]
        except Exception:
            return "?"

    def diff(self, other):
        """
        Compare two snapshots. Returns list of change descriptions.
        """
        changes = []
        if set(self.acm_ports) != set(other.acm_ports):
            added = set(other.acm_ports) - set(self.acm_ports)
            removed = set(self.acm_ports) - set(other.acm_ports)
            if added:
                changes.append(f"port appeared: {', '.join(sorted(added))}")
            if removed:
                changes.append(f"port disappeared: {', '.join(sorted(removed))}")
        if self.usb_mode != other.usb_mode:
            changes.append(f"USB mode: {self.usb_mode} -> {other.usb_mode}")
        if self.lsusb_hash != other.lsusb_hash:
            changes.append("USB bus changed (device added/removed)")
        return changes


# ============================================================
#  Ollama LLM integration (optional, best-effort)
# ============================================================

def ollama_available():
    """Check if Ollama is reachable and has our model."""
    try:
        import urllib.request
        req = urllib.request.Request(f"{OLLAMA_URL}/api/tags", method="GET")
        resp = urllib.request.urlopen(req, timeout=3)
        data = json.loads(resp.read())
        models = [m.get("name", "") for m in data.get("models", [])]
        return any(OLLAMA_MODEL.split(":")[0] in m for m in models)
    except Exception:
        return False


def ollama_ask(question, context="", timeout=30):
    """
    Ask the local Ollama LLM a question. Returns answer string or None.
    Non-blocking best-effort — never stalls the controller.
    """
    try:
        import urllib.request
        prompt = (
            "You are an embedded-systems firmware assistant for a LEGO SPIKE Prime hub "
            "(STM32F413, Cortex-M4F, RTIC v2 firmware, USB CDC serial on /dev/ttyACM*).\n"
            "Answer concisely in 1-3 sentences. No markdown.\n\n"
        )
        if context:
            prompt += f"Context: {context}\n\n"
        prompt += f"Question: {question}"

        body = json.dumps({
            "model": OLLAMA_MODEL,
            "prompt": prompt,
            "stream": False,
            "options": {"temperature": 0.3, "num_predict": 200},
        }).encode()

        req = urllib.request.Request(
            f"{OLLAMA_URL}/api/generate",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        resp = urllib.request.urlopen(req, timeout=timeout)
        data = json.loads(resp.read())
        return data.get("response", "").strip()
    except Exception:
        return None


# ============================================================
#  DFU firmware flash
# ============================================================

def firmware_build(profile="release"):
    """
    Build firmware in the spike_rtic directory.
    Returns path to ELF on success, None on failure.
    """
    print(f"  Building firmware ({profile})...")
    cmd = ["cargo", "build"]
    if profile == "release":
        cmd.append("--release")
    try:
        result = subprocess.run(
            cmd, cwd=SCRIPT_DIR,
            capture_output=True, text=True, timeout=120,
        )
        if result.returncode != 0:
            print(f"  Build FAILED:\n{result.stderr[-500:]}")
            return None
        elf = os.path.join(SCRIPT_DIR, FW_ELF_REL)
        if os.path.isfile(elf):
            print(f"  Build OK: {elf}")
            return elf
        print("  Build OK but ELF not found")
        return None
    except subprocess.TimeoutExpired:
        print("  Build timed out")
        return None


def firmware_elf_to_bin(elf_path, bin_path=FW_BIN_TMP):
    """Convert ELF to raw binary. Returns bin path or None."""
    for tool in ["arm-none-eabi-objcopy", "rust-objcopy"]:
        try:
            subprocess.run(
                [tool, "-O", "binary", elf_path, bin_path],
                check=True, capture_output=True, timeout=30,
            )
            size = os.path.getsize(bin_path)
            print(f"  Binary: {bin_path} ({size} bytes)")
            return bin_path
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue
    print("  ERR: no objcopy tool found")
    return None


def wait_for_dfu(timeout=30):
    """Wait for hub to appear in DFU mode. Returns True if found."""
    print(f"  Waiting for DFU device ({DFU_VID_PID})...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        mode = detect_usb_mode()
        if mode == ST_DFU:
            print("  DFU device found.")
            return True
        time.sleep(1.0)
    print("  Timeout: DFU device not found.")
    return False


def dfu_flash(bin_path=FW_BIN_TMP):
    """Flash a binary via dfu-util. Returns True on success."""
    if not os.path.isfile(bin_path):
        print(f"  ERR: binary not found: {bin_path}")
        return False

    print(f"  Flashing {bin_path} to {DFU_FLASH_ADDR}...")
    try:
        result = subprocess.run(
            ["dfu-util", "-d", DFU_VID_PID, "-a", "0",
             "-s", f"{DFU_FLASH_ADDR}:leave", "-D", bin_path],
            capture_output=True, text=True, timeout=60,
        )
        output = result.stdout + result.stderr
        print(f"  dfu-util output:\n{output[-600:]}")
        if result.returncode == 0:
            print("  Flash OK! Hub will reset.")
            return True
        else:
            print(f"  Flash FAILED (exit {result.returncode})")
            return False
    except FileNotFoundError:
        print("  ERR: dfu-util not installed")
        return False
    except subprocess.TimeoutExpired:
        print("  ERR: dfu-util timed out")
        return False


def full_flash_sequence(ask_human=True, conn=None):
    """
    Complete firmware build + flash sequence.
    Handles: build → enter DFU → wait → flash → wait for reboot.
    Returns True on success.
    """
    print("\n" + "=" * 50)
    print("  FIRMWARE FLASH SEQUENCE")
    print("=" * 50)

    # 1. Build
    elf = firmware_build()
    if not elf:
        return False

    # 2. Convert to bin
    bin_path = firmware_elf_to_bin(elf)
    if not bin_path:
        return False

    # 3. Check current state
    mode = detect_usb_mode()

    if mode != ST_DFU:
        # Try sending 'dfu' command if connected
        entered_dfu = False
        if conn and conn.is_open:
            print("  Sending 'dfu' command to hub...")
            try:
                conn.write(b"dfu\r\n")
                time.sleep(1.0)
                conn.close()
                entered_dfu = True
            except Exception:
                pass

        if not entered_dfu:
            if ask_human:
                ring_attention(
                    "Put hub in DFU mode:\n"
                    "  Hold CENTER button, power on hub, release after 2s.\n"
                    "  (Or: press LEFT button while firmware is running)"
                )
            else:
                print("  Hub not in DFU mode. Cannot proceed without human help.")
                return False

        if not wait_for_dfu(timeout=30):
            return False

    # 4. Flash
    if not dfu_flash(bin_path):
        return False

    # 5. Wait for hub to reboot into runtime
    print("  Waiting for hub to reboot...")
    time.sleep(3.0)

    for _ in range(10):
        mode = detect_usb_mode()
        if mode == ST_ALIVE:
            print("  Hub is alive!")
            return True
        acm = find_acm_ports()
        if acm:
            print(f"  Port appeared: {acm[0]}")
            return True
        time.sleep(1.0)

    print("  Hub did not reappear (may need manual power cycle)")
    return True  # flash itself succeeded


# ============================================================
#  COBS encoder (pure Python)
# ============================================================

def cobs_encode(data):
    """Encode data using Consistent Overhead Byte Stuffing."""
    output = bytearray()
    code_idx = 0
    code = 1
    output.append(0)  # placeholder

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


def crc32(data):
    return binascii.crc32(data) & 0xFFFFFFFF


# ============================================================
#  Port detection
# ============================================================

def find_acm_ports():
    """Return sorted list of /dev/ttyACM* ports."""
    return sorted(glob.glob("/dev/ttyACM*"))


def detect_hub_port():
    """
    Detect the SPIKE Prime hub CDC serial port.
    Strategy: LEGO VID -> STM32 VID -> name match -> first ttyACM*.
    Returns (port_path, method_description) or (None, reason).
    """
    for p in serial.tools.list_ports.comports():
        vid = p.vid or 0
        if vid == LEGO_VID:
            return p.device, f"LEGO {vid:04X}:{(p.pid or 0):04X}"

    for p in serial.tools.list_ports.comports():
        vid = p.vid or 0
        if vid == STM32_DFU_VID:
            return p.device, f"STM32 {vid:04X}:{(p.pid or 0):04X}"

    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        for kw in ("spike", "lego", "stm32", "cdc"):
            if kw in desc:
                return p.device, f"name:{kw}"

    acm = find_acm_ports()
    if acm:
        return acm[0], "fallback:ttyACM0"

    return None, "no port found"


def detect_usb_mode():
    """Detect whether hub is in DFU or runtime mode via lsusb."""
    try:
        out = subprocess.check_output(["lsusb"], text=True, timeout=5)
    except Exception:
        return None

    for line in out.strip().split("\n"):
        low = line.lower()
        if f"{LEGO_VID:04x}:{LEGO_DFU_PID:04x}" in low:
            return ST_DFU
        if f"{STM32_DFU_VID:04x}:{STM32_DFU_PID:04x}" in low:
            return ST_DFU
        if f"{LEGO_VID:04x}:{LEGO_RTIC_PID:04x}" in low:
            return ST_ALIVE
    return None


# ============================================================
#  Hub Connection (resilient serial)
# ============================================================

class HubConnection:
    """Resilient serial connection to the SPIKE Prime hub."""

    def __init__(self, port=None):
        self.port = port
        self.ser = None
        self._last_error = ""
        if port:
            self._connect()

    def _connect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        if not self.port:
            self.port, _ = detect_hub_port()
        if not self.port:
            self._last_error = "no port"
            return False

        for attempt in range(3):
            try:
                self.ser = serial.Serial(
                    self.port, BAUD,
                    timeout=READ_TIMEOUT,
                    write_timeout=2.0,
                    dsrdtr=False, rtscts=False,
                )
                time.sleep(0.2)
                self.ser.reset_input_buffer()
                self._last_error = ""
                return True
            except serial.SerialException as e:
                self._last_error = str(e)
                time.sleep(0.5)

        return False

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def reconnect(self):
        old = self.port
        self.port, _ = detect_hub_port()
        if not self.port:
            self.port = old
        return self._connect()

    def write(self, data):
        for _ in range(2):
            if not self.is_open:
                self.reconnect()
            if not self.is_open:
                raise IOError("not connected")
            try:
                self.ser.write(data)
                self.ser.flush()
                return
            except (serial.SerialException, OSError):
                self.reconnect()
        raise IOError("write failed")

    def read_until(self, marker, timeout=5.0):
        if not self.is_open:
            return b""
        buf = bytearray()
        deadline = time.time() + timeout
        old_to = self.ser.timeout
        try:
            self.ser.timeout = min(0.5, timeout)
            while time.time() < deadline:
                try:
                    chunk = self.ser.read(256)
                except (serial.SerialException, OSError):
                    break
                if chunk:
                    buf.extend(chunk)
                    if marker in buf:
                        return bytes(buf)
            return bytes(buf)
        finally:
            if self.is_open:
                self.ser.timeout = old_to

    def read_available(self, timeout=0.5):
        if not self.is_open:
            return b""
        buf = bytearray()
        deadline = time.time() + timeout
        old_to = self.ser.timeout
        try:
            self.ser.timeout = 0.1
            while time.time() < deadline:
                try:
                    chunk = self.ser.read(512)
                except (serial.SerialException, OSError):
                    break
                if chunk:
                    buf.extend(chunk)
                else:
                    break
            return bytes(buf)
        finally:
            if self.is_open:
                self.ser.timeout = old_to

    def send_command(self, cmd, timeout=10.0):
        """Send shell command, return cleaned response text."""
        self.read_available(timeout=0.2)
        self.write(cmd.encode() + b"\r\n")
        resp = self.read_until(PROMPT, timeout=timeout)
        text = resp.decode(errors="replace")
        lines = text.split("\r\n")
        if lines and cmd.strip() in lines[0]:
            lines = lines[1:]
        cleaned = []
        for line in lines:
            line = line.replace(PROMPT_STR, "").rstrip()
            if line:
                cleaned.append(line)
        return "\n".join(cleaned).strip()

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass


# ============================================================
#  Upload
# ============================================================

def upload_file(conn, filepath):
    """Upload binary to hub via COBS. Returns True on success."""
    if not os.path.isfile(filepath):
        print(f"ERR: file not found: {filepath}")
        return False

    with open(filepath, "rb") as f:
        data = f.read()

    if not data:
        print("ERR: file is empty")
        return False
    if len(data) > MAX_UPLOAD:
        print(f"ERR: too large ({len(data)} B, max {MAX_UPLOAD})")
        return False

    local_crc = crc32(data)
    print(f"File : {filepath}")
    print(f"Size : {len(data)} B")
    print(f"CRC32: 0x{local_crc:08X}")

    conn.read_available(timeout=0.2)
    conn.write(f"upload {len(data)}\r\n".encode())

    resp = conn.read_until(b"READY", timeout=5.0)
    if b"READY" not in resp:
        print(f"ERR: no READY from hub: {resp.decode(errors='replace')!r}")
        return False
    print("Hub: READY")
    time.sleep(0.05)

    encoded = cobs_encode(data) + b"\x00"
    print(f"COBS: {len(encoded)} B (overhead {len(encoded) - len(data)})")

    CHUNK = 512
    sent = 0
    t0 = time.time()
    while sent < len(encoded):
        end = min(sent + CHUNK, len(encoded))
        try:
            conn.write(encoded[sent:end])
        except IOError as e:
            print(f"\nERR: write failed @ {sent}: {e}")
            return False
        sent = end
        pct = sent * 100 // len(encoded)
        bar = "#" * (pct // 5) + "-" * (20 - pct // 5)
        print(f"\r  [{bar}] {pct:3d}%", end="", flush=True)
        time.sleep(0.002)

    elapsed = time.time() - t0
    print(f"\n  Sent in {elapsed:.2f}s")

    resp = conn.read_until(PROMPT, timeout=10.0)
    resp_text = resp.decode(errors="replace").strip()
    if not resp_text:
        print("WARN: no response (timeout)")
        return False
    print(f"Hub: {resp_text}")

    if "OK " in resp_text:
        for line in resp_text.split("\n"):
            line = line.strip()
            if line.startswith("OK "):
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        hub_size = int(parts[1])
                        hub_crc = int(parts[2], 16)
                        if hub_size == len(data) and hub_crc == local_crc:
                            print(f"VERIFIED: size={hub_size} CRC=0x{hub_crc:08X}")
                            return True
                        else:
                            print(f"MISMATCH: expected {len(data)}/0x{local_crc:08X}")
                            return False
                    except (ValueError, IndexError):
                        pass
        return True

    if "ERR" in resp_text:
        return False

    return False


# ============================================================
#  State diagnostic (from hub_state.py, simplified)
# ============================================================

def diagnose_quick(port=None):
    """
    Quick hub diagnostic. Returns (state, port, details_dict).
    """
    details = {}

    # USB mode
    usb_mode = detect_usb_mode()
    details["usb"] = usb_mode or "none"

    if usb_mode == ST_DFU:
        return ST_DFU, port, details

    # Port detection
    if not port:
        port, method = detect_hub_port()
        details["detect"] = method
    if not port:
        return ST_DISCONNECTED, None, details

    details["port"] = port

    # Try to probe the port
    try:
        fd = os.open(port, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    except OSError as e:
        details["open_err"] = str(e)
        return ST_UNKNOWN, port, details

    # drain
    drained = b""
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                drained += chunk
        except OSError:
            break
        time.sleep(0.02)

    # send CR
    try:
        os.write(fd, b"\r\n")
    except OSError:
        os.close(fd)
        return ST_STALLED, port, details

    # read response
    response = b""
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                response += chunk
                if b"spike>" in response:
                    break
        except OSError:
            pass
        time.sleep(0.05)

    os.close(fd)

    if b"spike>" in response:
        details["prompt"] = True
        return ST_ALIVE, port, details
    elif response:
        details["partial"] = len(response)
        return ST_BOOTING, port, details
    else:
        return ST_STALLED, port, details


# ============================================================
#  Test runner (from test_hub_mon.py)
# ============================================================

def has_hex(text):
    return bool(re.search(r'0x[0-9A-Fa-f]{2,8}', text))

def has_number(text):
    return bool(re.search(r'\d+', text))

def not_error(text):
    return not text.strip().upper().startswith("ERR")


class TestRunner:
    def __init__(self, conn):
        self.conn = conn
        self.passed = 0
        self.failed = 0
        self.errors = []
        self.t0 = time.time()

    def log(self, msg):
        elapsed = time.time() - self.t0
        print(f"[{elapsed:6.1f}s] {msg}")

    def test(self, name, cmd, *checks, timeout=10.0):
        time.sleep(0.15)
        self.log(f"TEST: {name}")
        self.log(f"  cmd> {cmd}")
        try:
            resp = self.conn.send_command(cmd, timeout=timeout)
        except Exception as e:
            self.log(f"  FAIL (exception: {e})")
            self.failed += 1
            self.errors.append((name, str(e)))
            return ""

        for line in resp.split("\n"):
            self.log(f"  | {line}")

        ok = True
        for check in checks:
            if callable(check):
                if not check(resp):
                    self.log("  FAIL (check)")
                    ok = False
            elif isinstance(check, str):
                if check.lower() not in resp.lower():
                    self.log(f"  FAIL (expected '{check}')")
                    ok = False

        if ok:
            self.log("  PASS")
            self.passed += 1
        else:
            self.failed += 1
            self.errors.append((name, resp[:80]))
        return resp

    def summary(self):
        elapsed = time.time() - self.t0
        total = self.passed + self.failed
        sep = "=" * 50
        print(f"\n{sep}")
        print(f"RESULTS  ({elapsed:.1f}s)")
        print(sep)
        print(f"  Total:  {total}")
        print(f"  Passed: {self.passed}")
        print(f"  Failed: {self.failed}")
        if self.errors:
            print("\n  FAILURES:")
            for n, d in self.errors:
                print(f"    - {n}: {d[:60]}")
        print(sep)
        return self.failed == 0


def run_core_tests(conn):
    """Run a core subset of tests (quick smoke test)."""
    t = TestRunner(conn)

    t.test("help", "help", "Hub Monitor CLI", "help")
    t.test("info", "info", "STM32F413", "RTIC")
    t.test("uptime", "uptime", "ticks")
    t.test("btn", "btn", not_error)
    t.test("led heart", "led heart", "OK heart")
    t.test("led clear", "led clear", "OK clear")
    t.test("bat", "bat", "Voltage:", "mV")
    t.test("regs", "regs", "CPUID", "VTOR")
    t.test("clocks", "clocks", "RCC Clock Config", "96 MHz")
    t.test("uid", "uid", "UID:")
    t.test("raminfo", "raminfo", "SRAM1:", "SRAM2:")
    t.test("unknown cmd", "xyzzy", "ERR", "unknown")
    t.test("led check (final)", "led check", "OK check")
    t.test("status green", "status green", "OK")

    return t.summary()


# ============================================================
#  TUI -- Continuous ASCII terminal interface
# ============================================================

class TUI:
    """
    Full-screen ASCII terminal UI for continuous hub monitoring.

    Layout (all ASCII, cursor-controlled):
      Row 1:   Title bar (reverse video)
      Row 2:   Hub state / port / uptime / battery
      Row 3:   Separator ---
      Row 4-N: Scrolling output area (hub responses)
      Row N+1: Separator ---
      Row N+2: Input line (spike> ...)
      Row N+3: Status hints bar
    """

    def __init__(self):
        self.conn = None
        self.port = None
        self.state = ST_DISCONNECTED
        self.running = True
        self.lines = []          # scrollback buffer
        self.input_buf = ""
        self.history = []
        self.hist_idx = -1
        self.uptime = "?"
        self.battery = "?"
        self.last_poll = 0
        self.connect_retry_at = 0
        self._old_termios = None
        self._last_draw = 0
        self._env_snap = None       # environment snapshot
        self._env_check_at = 0
        self._llm_available = None   # cached ollama check

    def run(self):
        fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(fd)
        try:
            tty.setraw(fd, termios.TCSANOW)
            # Make stdin non-blocking
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

            term_hide_cursor()
            term_clear()
            self._initial_connect()
            self._main_loop()
        finally:
            term_show_cursor()
            termios.tcsetattr(fd, termios.TCSANOW, self._old_termios)
            sys.stdout.write("\r\n")
            sys.stdout.flush()
            if self.conn:
                self.conn.close()

    def _initial_connect(self):
        self._env_snap = EnvSnapshot()
        self.port, method = detect_hub_port()
        if self.port:
            # Check for port conflicts
            users = find_port_users(self.port)
            if users:
                for pid, cmd in users:
                    self._add_line(f"[WARN] port {self.port} in use by PID {pid}: {cmd[:50]}")
                # Auto-kill in TUI mode (we own the port)
                self._add_line("[auto-killing blocking processes]")
                for pid, _ in users:
                    try:
                        os.kill(pid, signal.SIGTERM)
                        self._add_line(f"  SIGTERM -> PID {pid}")
                    except (ProcessLookupError, PermissionError) as e:
                        self._add_line(f"  cannot kill PID {pid}: {e}")
                time.sleep(1.0)

            self.conn = HubConnection(self.port)
            if self.conn.is_open:
                self.state = ST_ALIVE
                self._add_line(f"[connected] {self.port} ({method})")
                # get initial prompt
                try:
                    self.conn.write(b"\r\n")
                    resp = self.conn.read_until(PROMPT, timeout=2.0)
                    if resp:
                        text = resp.decode(errors="replace").strip()
                        if text:
                            for line in text.split("\r\n"):
                                clean = line.replace(PROMPT_STR, "").rstrip()
                                if clean:
                                    self._add_line(clean)
                except Exception:
                    pass
            else:
                self.state = ST_DISCONNECTED
                self._add_line(f"[failed] {self.port}: {self.conn._last_error}")
        else:
            # Check DFU mode
            usb_mode = detect_usb_mode()
            if usb_mode == ST_DFU:
                self.state = ST_DFU
                self._add_line("[hub in DFU mode]  Use !flash to build+flash firmware")
            else:
                self.state = ST_DISCONNECTED
                self._add_line("[no hub detected] -- waiting for ttyACM*...")

    def _main_loop(self):
        while self.running:
            now = time.time()

            # Read keyboard input (non-blocking)
            self._read_input()

            # Periodic poll (every 5s)
            if self.state == ST_ALIVE and now - self.last_poll >= 5.0:
                self._poll_hub()
                self.last_poll = now

            # Environment change detection (every 4s)
            if now - self._env_check_at >= 4.0:
                self._check_environment()
                self._env_check_at = now

            # Auto-reconnect (every 3s when disconnected)
            if self.state == ST_DISCONNECTED and now >= self.connect_retry_at:
                self._try_reconnect()
                self.connect_retry_at = now + 3.0

            # Read any incoming data from hub
            if self.conn and self.conn.is_open:
                self._drain_incoming()

            # Redraw (throttle to ~20 fps)
            if now - self._last_draw >= 0.05:
                self._draw()
                self._last_draw = now

            time.sleep(0.02)

    def _read_input(self):
        fd = sys.stdin.fileno()
        try:
            data = os.read(fd, 256)
        except OSError:
            return

        for b in data:
            if b == 3:  # Ctrl-C
                self.running = False
                return
            elif b == 4:  # Ctrl-D
                self.running = False
                return
            elif b == 2:  # Ctrl-B -> ring attention bell
                sys.stdout.write(BEL)
                sys.stdout.flush()
                self._add_line("[bell] Ctrl-B: terminal bell sent")
                continue
            elif b == 13:  # Enter
                cmd = self.input_buf.strip()
                self.input_buf = ""
                self.hist_idx = -1
                if cmd:
                    self.history.append(cmd)
                    self._execute_command(cmd)
            elif b == 127 or b == 8:  # Backspace
                if self.input_buf:
                    self.input_buf = self.input_buf[:-1]
            elif b == 27:  # ESC sequence
                # Try to read rest of sequence
                try:
                    b2 = os.read(fd, 1)
                    if b2 == b'[':
                        b3 = os.read(fd, 1)
                        if b3 == b'A':  # Up arrow
                            if self.history:
                                if self.hist_idx == -1:
                                    self.hist_idx = len(self.history) - 1
                                elif self.hist_idx > 0:
                                    self.hist_idx -= 1
                                self.input_buf = self.history[self.hist_idx]
                        elif b3 == b'B':  # Down arrow
                            if self.hist_idx >= 0:
                                self.hist_idx += 1
                                if self.hist_idx < len(self.history):
                                    self.input_buf = self.history[self.hist_idx]
                                else:
                                    self.hist_idx = -1
                                    self.input_buf = ""
                except OSError:
                    pass
            elif 32 <= b <= 126:  # Printable ASCII
                self.input_buf += chr(b)

    def _execute_command(self, cmd):
        self._add_line(f"spike> {cmd}")

        # Host commands
        if cmd.startswith("!") or cmd.startswith("/"):
            prefix = cmd[1:].lower().split()[0] if len(cmd) > 1 else ""
            if prefix in ("quit", "exit", "q"):
                self.running = False
                return
            elif prefix == "reconnect":
                self._try_reconnect()
                return
            elif prefix == "diag":
                state, port, details = diagnose_quick(self.port)
                self._add_line(f"[diag] state={state} port={port}")
                for k, v in details.items():
                    self._add_line(f"  {k}: {v}")
                return
            elif prefix == "test":
                self._add_line("[running core tests...]")
                if self.conn and self.conn.is_open:
                    ok = run_core_tests(self.conn)
                    self._add_line(f"[tests: {'PASS' if ok else 'FAIL'}]")
                else:
                    self._add_line("[ERR: not connected]")
                return
            elif prefix == "upload":
                parts = cmd[1:].split(None, 1)
                if len(parts) < 2:
                    self._add_line("Usage: !upload <filepath>")
                    return
                fpath = os.path.expanduser(parts[1].strip())
                if not os.path.isabs(fpath):
                    fpath = os.path.join(os.getcwd(), fpath)
                if self.conn and self.conn.is_open:
                    ok = upload_file(self.conn, fpath)
                    self._add_line(f"[upload: {'OK' if ok else 'FAIL'}]")
                else:
                    self._add_line("[ERR: not connected]")
                return
            elif prefix == "help":
                self._add_line("Host commands:")
                self._add_line("  !quit        exit shbc")
                self._add_line("  !reconnect   reconnect to hub")
                self._add_line("  !diag        run state diagnostic")
                self._add_line("  !test        run core test suite")
                self._add_line("  !upload <f>  upload binary via COBS")
                self._add_line("  !flash       build + DFU flash firmware")
                self._add_line("  !portinfo    show port usage info")
                self._add_line("  !portkill    kill processes blocking port")
                self._add_line("  !attention   ring bell for human")
                self._add_line("  !env         show environment snapshot")
                self._add_line("  !llm <q>     ask local Ollama LLM")
                self._add_line("  Ctrl-B       terminal bell ping")
                return
            elif prefix == "flash":
                self._add_line("[flash] starting build + DFU sequence...")
                self._host_flash()
                return
            elif prefix == "portinfo":
                self._host_portinfo()
                return
            elif prefix == "portkill":
                self._host_portkill()
                return
            elif prefix == "attention":
                msg = cmd[1:].split(None, 1)
                text = msg[1] if len(msg) > 1 else "Human attention needed!"
                self._host_attention(text)
                return
            elif prefix == "env":
                self._host_env()
                return
            elif prefix == "llm":
                parts = cmd[1:].split(None, 1)
                question = parts[1] if len(parts) > 1 else "What is the hub state?"
                self._host_llm(question)
                return
            else:
                self._add_line(f"[unknown host cmd: {prefix}] try !help")
                return

        # Send to hub
        if not self.conn or not self.conn.is_open:
            self._add_line("[ERR: not connected]")
            return

        try:
            timeout = 30.0 if cmd.startswith("ramtest") else 10.0
            resp = self.conn.send_command(cmd, timeout=timeout)
            if resp:
                for line in resp.split("\n"):
                    self._add_line(line.rstrip())
        except IOError as e:
            self._add_line(f"[err: {e}]")
            self.state = ST_STALLED

    def _host_flash(self):
        """Build firmware and flash via DFU."""
        # Build
        self._add_line("[flash] building firmware (release)...")
        elf = firmware_build()
        if not elf:
            self._add_line("[flash] BUILD FAILED")
            return
        self._add_line(f"[flash] ELF: {elf}")

        # Convert
        bin_path = firmware_elf_to_bin(elf)
        if not bin_path:
            self._add_line("[flash] objcopy FAILED")
            return
        self._add_line(f"[flash] BIN: {bin_path} ({os.path.getsize(bin_path)} B)")

        # Check DFU mode
        mode = detect_usb_mode()
        if mode != ST_DFU:
            # Try 'dfu' command on connected hub
            if self.conn and self.conn.is_open:
                self._add_line("[flash] sending 'dfu' command to hub...")
                try:
                    self.conn.write(b"dfu\r\n")
                    time.sleep(1.5)
                    self.conn.close()
                    self.state = ST_DFU
                except Exception:
                    pass

            if detect_usb_mode() != ST_DFU:
                self._add_line("[flash] hub not in DFU mode")
                self._add_line("[flash] ring bell -- put hub in DFU mode manually")
                sys.stdout.write(BEL)
                sys.stdout.flush()

            if not wait_for_dfu(timeout=30):
                self._add_line("[flash] DFU timeout")
                return
        else:
            self._add_line("[flash] hub already in DFU mode")

        # Flash
        self._add_line("[flash] flashing via dfu-util...")
        if dfu_flash(bin_path):
            self._add_line("[flash] SUCCESS -- hub will reboot")
            self.state = ST_BOOTING
            time.sleep(3.0)
            self._try_reconnect()
        else:
            self._add_line("[flash] FAILED")

    def _host_portinfo(self):
        """Show info about ttyACM ports and their users."""
        acm = find_acm_ports()
        if not acm:
            self._add_line("[portinfo] no ttyACM ports found")
            return
        for p in acm:
            users = find_port_users(p)
            if users:
                for pid, cmd in users:
                    self._add_line(f"  {p}: PID {pid} = {cmd[:50]}")
            else:
                self._add_line(f"  {p}: free")

    def _host_portkill(self):
        """Kill processes blocking the hub port."""
        port = self.port
        if not port:
            port, _ = detect_hub_port()
        if not port:
            self._add_line("[portkill] no port to check")
            return
        users = find_port_users(port)
        if not users:
            self._add_line(f"[portkill] {port} is free")
            return
        for pid, cmd in users:
            self._add_line(f"[portkill] killing PID {pid}: {cmd[:50]}")
            try:
                os.kill(pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError) as e:
                self._add_line(f"  error: {e}")
        time.sleep(1.0)
        remaining = find_port_users(port)
        if remaining:
            for pid, _ in remaining:
                try:
                    os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        self._add_line(f"[portkill] done, port should be free now")

    def _host_attention(self, message):
        """Ring terminal bell repeatedly."""
        for _ in range(5):
            sys.stdout.write(BEL)
            sys.stdout.flush()
            time.sleep(0.5)
        self._add_line(f"[attention] {message}")
        self._add_line("[attention] bell sent x5 -- human, please act")

    def _host_env(self):
        """Show environment snapshot and changes."""
        snap = EnvSnapshot()
        self._add_line(f"[env] ACM ports: {snap.acm_ports}")
        self._add_line(f"[env] USB mode: {snap.usb_mode}")
        self._add_line(f"[env] USB hash: {snap.lsusb_hash}")
        if self._env_snap:
            changes = self._env_snap.diff(snap)
            if changes:
                for c in changes:
                    self._add_line(f"[env] CHANGED: {c}")
            else:
                self._add_line("[env] no changes since last snapshot")
        self._env_snap = snap

    def _host_llm(self, question):
        """Ask Ollama LLM a question."""
        if self._llm_available is None:
            self._llm_available = ollama_available()
        if not self._llm_available:
            self._add_line("[llm] Ollama not available (is 'ollama serve' running?)")
            return

        context = (f"Hub state={self.state}, port={self.port}, "
                   f"uptime={self.uptime}, battery={self.battery}")
        self._add_line(f"[llm] asking: {question[:60]}...")
        answer = ollama_ask(question, context=context, timeout=30)
        if answer:
            for line in answer.split("\n"):
                self._add_line(f"[llm] {line.rstrip()}")
        else:
            self._add_line("[llm] no response (timeout or error)")

    def _check_environment(self):
        """Detect USB / port changes since last snapshot."""
        if self._env_snap is None:
            self._env_snap = EnvSnapshot()
            return

        new_snap = EnvSnapshot()
        changes = self._env_snap.diff(new_snap)
        if changes:
            for c in changes:
                self._add_line(f"[env] {c}")
            # React to changes
            if new_snap.usb_mode == ST_DFU and self.state != ST_DFU:
                self.state = ST_DFU
                self._add_line("[env] hub switched to DFU mode")
                if self.conn:
                    self.conn.close()
                    self.conn = None
            elif any("appeared" in c for c in changes) and self.state != ST_ALIVE:
                self._add_line("[env] new port -- attempting reconnect")
                self._try_reconnect()
            elif any("disappeared" in c for c in changes) and self.state == ST_ALIVE:
                self._add_line("[env] port lost -- hub disconnected?")
                self.state = ST_DISCONNECTED
        self._env_snap = new_snap

    def _poll_hub(self):
        """Poll uptime and battery from the hub."""
        if not self.conn or not self.conn.is_open:
            return
        try:
            resp = self.conn.send_command("uptime", timeout=3.0)
            if resp:
                m = re.search(r'(\d+)\s+ticks', resp)
                if m:
                    self.uptime = m.group(1)
        except Exception:
            pass

        try:
            resp = self.conn.send_command("bat", timeout=3.0)
            if resp:
                m = re.search(r'Voltage:\s+(\d+)\s+mV', resp)
                if m:
                    self.battery = f"{m.group(1)} mV"
        except Exception:
            pass

    def _try_reconnect(self):
        old_port = self.port
        self.port, method = detect_hub_port()
        if not self.port:
            self.port = old_port
            return

        if self.conn:
            self.conn.port = self.port
            ok = self.conn.reconnect()
        else:
            self.conn = HubConnection(self.port)
            ok = self.conn.is_open

        if ok:
            self.state = ST_ALIVE
            self._add_line(f"[reconnected] {self.port}")
            # Flush and get prompt
            try:
                self.conn.write(b"\r\n")
                self.conn.read_until(PROMPT, timeout=2.0)
            except Exception:
                pass
        else:
            self.state = ST_DISCONNECTED

    def _drain_incoming(self):
        """Read any unsolicited data from the hub serial port."""
        if not self.conn or not self.conn.is_open:
            return
        try:
            old_to = self.conn.ser.timeout
            self.conn.ser.timeout = 0
            try:
                chunk = self.conn.ser.read(1024)
            except Exception:
                chunk = b""
            self.conn.ser.timeout = old_to
            if chunk:
                text = chunk.decode(errors="replace")
                for line in text.split("\r\n"):
                    clean = line.rstrip()
                    if clean and clean != PROMPT_STR.strip():
                        self._add_line(clean)
        except Exception:
            pass

    def _add_line(self, text):
        self.lines.append(text)
        # Keep scrollback bounded
        if len(self.lines) > 2000:
            self.lines = self.lines[-1500:]

    def _draw(self):
        rows, cols = term_size()

        # Minimal fallback for very small terminals
        if rows < 5 or cols < 20:
            term_goto(1, 1)
            sys.stdout.write(f"Terminal too small ({cols}x{rows}), need 20x5+")
            sys.stdout.flush()
            return

        # Row 1: Title bar
        term_goto(1, 1)
        title = f" shbc v{VERSION} "
        if cols >= 40:
            title = f" SPIKE RTIC Hub Controller v{VERSION} "
        title = title[:cols]
        pad = " " * max(0, cols - len(title))
        sys.stdout.write(term_reverse(title + pad))

        # Row 2: Status bar
        term_goto(2, 1)
        term_clear_line()

        state_str = self.state
        if self.state == ST_ALIVE:
            state_str = term_color("ALIVE", 2)
        elif self.state == ST_DISCONNECTED:
            state_str = term_color("DISCONNECTED", 1)
        elif self.state == ST_DFU:
            state_str = term_color("DFU", 3)
        elif self.state == ST_STALLED:
            state_str = term_color("STALLED", 1)

        port_str = self.port or "---"
        if cols >= 60:
            status_line = (f" State: {state_str}  Port: {port_str}"
                           f"  Up: {self.uptime}  Bat: {self.battery}")
        else:
            status_line = f" {state_str} {port_str}"
        sys.stdout.write(status_line[:cols])

        # Row 3: separator
        term_goto(3, 1)
        sys.stdout.write("-" * cols)

        # Bottom area
        input_row = rows - 1
        hints_row = rows
        sep_row = rows - 2
        output_area = max(1, rows - 5)  # rows 4..sep_row-1

        # Row sep_row: separator
        term_goto(sep_row, 1)
        sys.stdout.write("-" * cols)

        # Scrolling output area: bottom-aligned (latest lines near input)
        visible = self.lines[-(output_area):]
        blank_rows = output_area - len(visible)
        for i in range(output_area):
            term_goto(4 + i, 1)
            term_clear_line()
            vi = i - blank_rows
            if 0 <= vi < len(visible):
                sys.stdout.write(visible[vi][:cols])

        # Input row
        term_goto(input_row, 1)
        term_clear_line()
        prompt_display = f"spike> {self.input_buf}"
        sys.stdout.write(prompt_display[:cols])

        # Hints row
        term_goto(hints_row, 1)
        term_clear_line()
        if cols >= 60:
            hints = term_dim(
                " Ctrl-C:quit  Ctrl-B:bell  !help  !flash  !llm <q>  !env"
            )
        else:
            hints = term_dim(" Ctrl-C:quit !help !flash")
        sys.stdout.write(hints[:cols])

        sys.stdout.flush()


# ============================================================
#  CLI modes
# ============================================================

def mode_tui():
    """Full-screen continuous TUI mode."""
    ui = TUI()
    ui.run()


def mode_shell(port=None):
    """Interactive shell (like spike_hub.py)."""
    if not port:
        port, method = detect_hub_port()
        if not port:
            print("No hub found. Specify port.")
            return 1
        print(f"Detected: {port} ({method})")

    conn = HubConnection(port)
    if not conn.is_open:
        print(f"Cannot open {port}")
        return 1

    print(f"Connected: {port}")
    print("Type commands (help, info, etc.)  Ctrl-D to exit.")
    print("  !upload <file>  !reconnect  !quit")
    print()

    # Get initial prompt
    conn.write(b"\r\n")
    resp = conn.read_until(PROMPT, timeout=2.0)
    if resp:
        sys.stdout.write(resp.decode(errors="replace"))
        sys.stdout.flush()

    try:
        import readline as _rl
    except ImportError:
        pass

    while True:
        try:
            line = input("")
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        line = line.strip()
        if not line:
            try:
                conn.write(b"\r\n")
                resp = conn.read_until(PROMPT, timeout=2.0)
                if resp:
                    sys.stdout.write(resp.decode(errors="replace"))
                    sys.stdout.flush()
            except IOError:
                print("[disconnected]")
                conn.reconnect()
            continue

        if line.startswith("!"):
            parts = line[1:].split(None, 1)
            host_cmd = parts[0].lower() if parts else ""
            if host_cmd in ("quit", "exit", "q"):
                break
            elif host_cmd == "reconnect":
                conn.reconnect()
            elif host_cmd == "upload":
                if len(parts) < 2:
                    print("Usage: !upload <filepath>")
                    continue
                fpath = os.path.expanduser(parts[1].strip())
                if not os.path.isabs(fpath):
                    fpath = os.path.join(os.getcwd(), fpath)
                ok = upload_file(conn, fpath)
                print("Upload:", "OK" if ok else "FAIL")
            else:
                print(f"Unknown: !{host_cmd}")
            continue

        try:
            timeout = 30.0 if line.startswith("ramtest") else 10.0
            resp = conn.send_command(line, timeout=timeout)
            if resp:
                print(resp)
        except IOError as e:
            print(f"[error: {e}]")
            conn.reconnect()

    conn.close()
    return 0


def mode_diag(port=None):
    """One-shot state diagnostic."""
    print("=" * 50)
    print("  SPIKE Prime Hub -- State Diagnostic")
    print("=" * 50)

    state, port, details = diagnose_quick(port)

    print(f"\n  State : {state}")
    print(f"  Port  : {port or '---'}")
    for k, v in details.items():
        print(f"  {k:10s}: {v}")

    if state == ST_DFU:
        print("\n  Ready to flash:")
        print("  dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D firmware.bin")
    elif state == ST_ALIVE:
        print(f"\n  Hub is alive. Connect: picocom {port}")
    elif state == ST_DISCONNECTED:
        print("\n  No hub detected. Power on and plug USB.")
    elif state == ST_STALLED:
        print("\n  CDC stalled. Unplug USB, wait 2s, replug.")

    return 0 if state == ST_ALIVE else 1


def mode_test(port=None):
    """Run automated test suite."""
    if not port:
        port, _ = detect_hub_port()
    if not port:
        print("No hub found.")
        return 1

    conn = HubConnection(port)
    if not conn.is_open:
        print(f"Cannot open {port}")
        return 1

    # Warm up
    conn.write(b"\r\n")
    time.sleep(0.5)
    conn.read_available(timeout=0.5)

    print(f"Running core tests on {port}...\n")
    ok = run_core_tests(conn)
    conn.close()
    return 0 if ok else 1


def mode_cmd(port, cmd):
    """Execute a single command and exit."""
    if not port:
        port, _ = detect_hub_port()
    if not port:
        print("No hub found.")
        return 1

    conn = HubConnection(port)
    if not conn.is_open:
        print(f"Cannot open {port}")
        return 1

    timeout = 30.0 if cmd.startswith("ramtest") else 10.0
    resp = conn.send_command(cmd, timeout=timeout)
    print(resp)
    conn.close()
    return 0


# ============================================================
#  Main
# ============================================================

def main():
    # Singleton guard
    if not acquire_singleton():
        pid = _read_lockfile()
        print(f"Another shbc instance is running (PID {pid}). Exiting.")
        return 1

    signal.signal(signal.SIGINT, lambda s, f: None)

    args = sys.argv[1:]

    # Extract port if first arg looks like a device path
    port = None
    if args and (args[0].startswith("/dev/") or args[0].startswith("COM")):
        port = args.pop(0)

    mode = args[0].lower() if args else "tui"
    rest = args[1:]

    if mode == "tui":
        mode_tui()
        return 0
    elif mode == "shell":
        return mode_shell(port)
    elif mode == "diag":
        return mode_diag(port)
    elif mode == "test":
        return mode_test(port)
    elif mode == "flash":
        return 0 if full_flash_sequence(ask_human=True) else 1
    elif mode == "upload":
        if not rest:
            print("Usage: spike_hub_controller.py upload <file>")
            return 1
        if not port:
            port, _ = detect_hub_port()
        if not port:
            print("No hub found.")
            return 1
        conn = HubConnection(port)
        if not conn.is_open:
            return 1
        ok = upload_file(conn, rest[0])
        conn.close()
        return 0 if ok else 1
    elif mode == "cmd":
        cmd = " ".join(rest)
        if not cmd:
            print("Usage: spike_hub_controller.py cmd <command>")
            return 1
        return mode_cmd(port, cmd)
    elif mode in ("help", "-h", "--help"):
        print(f"spike_hub_controller.py v{VERSION}")
        print()
        print("Usage:")
        print("  spike_hub_controller.py              -- TUI (continuous monitor)")
        print("  spike_hub_controller.py shell         -- interactive shell")
        print("  spike_hub_controller.py upload <file> -- upload binary via COBS")
        print("  spike_hub_controller.py test          -- run core test suite")
        print("  spike_hub_controller.py diag          -- one-shot diagnostic")
        print("  spike_hub_controller.py flash          -- build + DFU flash firmware")
        print("  spike_hub_controller.py cmd <command> -- execute single command")
        print()
        print("Options:")
        print("  /dev/ttyACMx as first arg to specify port explicitly")
        return 0
    else:
        # Assume it's a direct command
        cmd = " ".join([mode] + rest)
        return mode_cmd(port, cmd)


if __name__ == "__main__":
    sys.exit(main())
