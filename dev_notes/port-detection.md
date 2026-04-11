# USB Serial Port Detection

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---


## Problem

The firmware exposes a dual CDC-ACM USB composite device (VID:PID `0694:0042`):
- **Interface 0** — Shell (commands, COBS upload, demo I/O)
- **Interface 2** — GDB RSP (remote serial protocol for debugging)

Both appear as `/dev/ttyACM*` ports. Eight Python tools and two Rust xtasks
had copy-pasted port detection code, each with different heuristics and
failure modes. A blind VID:PID match could grab the GDB port instead of
shell, causing silent failures.

## Solution: Detection Priority

All tools now follow the same 4-tier detection priority:

1. **Udev symlinks** — `/dev/spike-shell`, `/dev/spike-gdb` (instant, no scanning)
2. **Sysfs interface scan** — read `bInterfaceNumber` from `/sys/class/tty/ttyACM*/device/` to distinguish IF 00 (shell) from IF 02 (gdb)
3. **pyserial / serialport crate VID:PID** — match LEGO VID `0694`, runtime PID `0042`
4. **Brute-force** — try all `/dev/ttyACM*` (last resort)

## Udev Rules

File: `helper-tools/99-spike-hub.rules`

```
# Creates /dev/spike-shell (IF 00) and /dev/spike-gdb (IF 02)
```

Install:
```sh
sudo cp helper-tools/99-spike-hub.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Shared Python Module

`helper-tools/spike_port.py` — single source of truth for all Python tools.

Key functions:
- `find_shell_port()` → `/dev/spike-shell` or ttyACM* with IF 00
- `find_gdb_port()` → `/dev/spike-gdb` or ttyACM* with IF 02
- `find_hub_port()` → backward-compat alias for `find_shell_port()`
- `is_dfu_present()` → checks for DFU PID `0011`
- `hub_status()` → prints diagnostic table (see below)

Constants: `LEGO_VID=0x0694`, `DFU_PID=0x0011`, `RUNTIME_PID=0x0042`, `BAUD=115200`

## Rust Xtasks

Both xtasks (`xtask/src/main.rs` and `examples/hub-ram-demos/xtask/src/main.rs`)
use the same priority: symlink → sysfs → serialport crate → brute-force.

The demo xtask's `port` subcommand prints a full diagnostic table.
The root xtask's `status` subcommand prints the same.

## Diagnostic Table

Both Python (`python3 helper-tools/spike_port.py`) and Rust (`cargo xtask port`
from the demo xtask) print the same table:

```
  SPIKE Hub Connection Status
  ──────────────────────────────────────────────────────────────
  Device           VID:PID      IF   Role     Symlink              Status     Product
  ───────────────  ───────────  ───  ───────  ───────────────────  ─────────  ────────────────────
  /dev/ttyACM0     0694:0042    00   shell    /dev/spike-shell     free       Spike Prime RTIC
  /dev/ttyACM1     0694:0042    02   gdb      /dev/spike-gdb       free       Spike Prime RTIC

  Shell port: /dev/spike-shell
  GDB port:   /dev/spike-gdb
```

## Performance

Both pre-compiled, symlink path (typical case):

| Tool   | Time  |
|--------|-------|
| Rust   | 12 ms |
| Python | 48 ms |

## Files Modified

- `helper-tools/spike_port.py` — NEW shared module
- `helper-tools/upload_demo.py` — uses spike_port
- `helper-tools/color_ballet.py` — uses spike_port
- `helper-tools/test_sandbox.py` — uses spike_port
- `helper-tools/flash_store_run.py` — uses spike_port
- `helper-tools/debug_pipeline.py` — uses spike_port
- `helper-tools/hub_state.py` — uses spike_port
- `helper-tools/hub.py` — uses spike_port
- `helper-tools/test_gdb_proto.py` — uses spike_port
- `helper-tools/test_hub_mon.py` — uses spike_port
- `xtask/src/main.rs` — enhanced detect_hub() + cmd_status()
- `examples/hub-ram-demos/xtask/src/main.rs` — enhanced cmd_port() with diagnostic table
