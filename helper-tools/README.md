# helper-tools/ — Python & Shell Scripts

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Dev Notes](../dev_notes/)

---

## Two Legs

This project leans on **two legs** for tooling:

1. **Python** — quick, interactive, great for serial port wrangling,
   COBS encoding, test automation, and rapid prototyping.  Python
   scripts are ideal for Copilot-assisted one-off experiments and
   glue code that talks to the hub over USB CDC serial.

2. **xtask** (Rust) — the Rust way.  Build, objcopy, upload, debug
   pipeline, and VS Code integration all live in
   [`examples/hub-ram-demos/xtask/`](../examples/hub-ram-demos/xtask/)
   as a proper Cargo binary.  The xtask handles port detection,
   process lifecycle, and GDB/LLDB bridge setup — no Python needed
   for the standard F5 workflow.

**When to use which:**
- `xtask build/upload/debug` for day-to-day demo development
- Python scripts for diagnostics, testing, flash storage, and
  host-driven demo choreography

---

## Serial Port Detection

The Linux USB CDC enumerator assigns `/dev/ttyACMx` numbers
dynamically — after a disconnect/reconnect the hub might appear as
`ttyACM0`, `ttyACM1`, or `ttyACM2`.  Three strategies handle this:

| Strategy | How | Used by |
|----------|-----|---------|
| **udev symlinks** | `aux/99-spike-hub.rules` creates stable `/dev/spike-shell` and `/dev/spike-gdb` | `test_gdb_proto.py`, manual use |
| **VID:PID auto-detect** | pyserial `comports()` scans for `0694:0042` | All Python scripts |
| **CLI override** | Pass port as argument: `python3 script.py /dev/ttyACM1` | Most scripts |

### Installing the udev rules

```bash
sudo cp aux/99-spike-hub.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

After this, plugging in the hub always gives you:
- `/dev/spike-shell` → shell/REPL serial port (USB interface 00)
- `/dev/spike-gdb`   → GDB RSP debug port (USB interface 02)

No more guessing which `ttyACMx` is which.

---

## Scripts

### Core Tools

| Script | Purpose |
|--------|---------|
| `hub.py` | Swiss-army CLI — `state`, `flash`, `cmd`, `run`, `monitor` |
| `hub_state.py` | Detect hub state (disconnected/DFU/shell/GDB) — used by VS Code task |
| `upload_demo.py` | Upload `.bin` via COBS + run `go` — used by root xtask |
| `debug_pipeline.py` | Pre-launch: upload → go → enter RSP → release port for GDB |

### Tests

| Script | Purpose |
|--------|---------|
| `test_hub_mon.py` | Automated 65-test suite for all shell commands |
| `test_sandbox.py` | MPU sandbox tests with hand-built ARM Thumb machine code |
| `test_gdb_proto.py` | Raw GDB RSP protocol test (requires socat + udev symlinks) |

### Demo & Utilities

| Script | Purpose |
|--------|---------|
| `color_ballet.py` | Host-side motor + color sensor choreography via shell commands |
| `flash_store_run.py` | Upload to RAM → store to SPI flash → optionally run |
| `flash.sh` | One-liner: cargo build + objcopy + dfu-util flash |
| `attention.sh` | Ring terminal bell until user presses Enter |

### OLD/

Superseded or one-off scripts moved here for reference:

| Script | Why moved |
|--------|-----------|
| `spike_hub_controller.py` | 2K-line never-completed TUI — superseded by `hub.py` |
| `enter_rsp.py` | Superseded by `debug_pipeline.py` |
| `test_spsel_fix.py` | One-off SPSEL regression test |
| `test_serial_linux.sh` | One-off line-ending test (bash/stty) |
| `build.rs` | Orphan Cargo build script — not used |

---

## Quick Reference

```bash
# Check hub state
python3 $PROJECT_ROOT/helper-tools/hub_state.py

# Upload and run a demo
python3 $PROJECT_ROOT/helper-tools/upload_demo.py \
    $PROJECT_ROOT/examples/hub-ram-demos/target/spike-usr_bins/dual_motors.bin

# Flash firmware (enter DFU mode first)
$PROJECT_ROOT/helper-tools/flash.sh

# Run all tests
python3 $PROJECT_ROOT/helper-tools/test_hub_mon.py

# Run MPU sandbox tests
python3 $PROJECT_ROOT/helper-tools/test_sandbox.py
```

---

*See also:* [aux/99-spike-hub.rules](../aux/99-spike-hub.rules) for
udev symlink setup.
