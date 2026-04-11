# demo-xtask — SPIKE Hub RAM Demo Manager

[← Main README](../../../README.md) · [User Manual](../../../USER_MANUAL.md) · [Reference Manual](../../../REFERENCE_MANUAL.md) · [API Reference](../../../spike-hub-api/README.md) · [RAM Demos](../README.md) · [Helper Tools](../../../helper-tools/README.md) · [Dev Notes](../../../dev_notes/)

---

Build, upload, debug, and manage RAM demo binaries for the SPIKE hub.
Handles serial port detection, COBS upload, GDB RSP mode entry, and
automatic port-conflict resolution.

## Usage

From the project root , see also:printenv  PROJECT_ROOT
>>  /home/rustuser/projects/rust/spike-rtic

```sh
cargo demo                  # interactive: pick demo → pick action
cargo demo <name>           # pick action for a specific demo
cargo demo debug <name>     # build → objcopy → upload → go → enter GDB RSP mode
cargo demo build <name>     # build → objcopy (.bin output)
cargo demo upload <name>    # build → objcopy → upload → go (run demo)
cargo demo list             # list available demo names
cargo demo port             # show hub port diagnostic table
cargo demo free-port        # kill processes blocking the hub serial port
```

`cargo example` is an alias for `cargo demo` (both work).

### What each command does

| Command | Description |
|---------|-------------|
| *(none)* | Interactive picker — lists all demos, then offers build/upload/debug |
| `<name>` | Jump to action picker for a named demo |
| `debug <name>` | Full debug pipeline: build → objcopy → upload → `go` → enter RSP mode → create `/tmp/spike-hub` symlink. Designed as a VS Code `preLaunchTask` — exits after RSP entry so GDB/LLDB can connect. |
| `build <name>` | Compile the demo + `arm-none-eabi-objcopy` to `.bin`. Output in `target/spike-usr_bins/` |
| `upload <name>` | Build + upload via COBS over serial + send `go` to start execution |
| `list` | List all `.rs` files in `examples/` that are valid demo targets |
| `port` | Full port diagnostic: all ttyACM ports with VID:PID, interface number, role (shell/gdb), symlink, busy status |
| `free-port` | Detect and kill (SIGTERM → SIGKILL) any process holding the hub serial port (stale picocom, screen, Python scripts) |

### Robustness features

- **Auto port-free:** If another process holds the serial port, it is
  detected via `fuser`/`lsof` and killed automatically before proceeding.
- **State probing:** Before assuming the hub's mode, probe bytes are sent
  to classify the response as Shell, RSP/GDB, or Unknown — then the
  recovery sequence adapts.
- **Port detection priority:** Udev symlinks → sysfs interface scan →
  serialport crate VID:PID → brute-force. See
  [dev_notes/port-detection.md](../../../dev_notes/port-detection.md).

## The `debug` Pipeline

The `debug` command is the key workflow for interactive debugging:

```
cargo demo debug my_demo
```

1. `cargo build --release --example my_demo` (cross-compile for ARM)
2. `arm-none-eabi-objcopy -O binary ... my_demo.bin`
3. Open serial port → COBS upload the `.bin`
4. Send `go\r` to start the demo
5. Send `rsp\r` to enter GDB RSP mode
6. Create symlink `/tmp/spike-hub → /dev/ttyACMx`
7. Exit (port released for GDB/LLDB to connect)

In VS Code, this is configured as a `preLaunchTask` so pressing F5 does:
debug pipeline → GDB/LLDB attach → breakpoints work.

## Port Diagnostic Table

`cargo demo port` prints:

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

## VS Code Integration

Registered in `.cargo/config.toml` as two aliases:

```toml
[alias]
demo    = "run --manifest-path examples/hub-ram-demos/xtask/Cargo.toml --target x86_64-unknown-linux-gnu --"
example = "run --manifest-path examples/hub-ram-demos/xtask/Cargo.toml --target x86_64-unknown-linux-gnu --"
```

VS Code tasks (in the workspace file) call these for build/upload/debug.
The **Run Terminal Command** extension adds right-click context menu items
that run the currently open demo file through this xtask.

## Recommended VS Code Extensions

| Extension | What it does for demo development |
|-----------|----------------------------------|
| [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer) | Code intelligence for `#![no_std]` demo code. The workspace `rust-analyzer.linkedProjects` includes this xtask. |
| [CodeLLDB](https://marketplace.visualstudio.com/items?itemName=vadimcn.vscode-lldb) | Debug adapter — connects to the hub after `cargo demo debug` enters RSP mode |
| [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) | ARM peripheral register views, memory inspector, SVD support |
| [Run Terminal Command](https://marketplace.visualstudio.com/items?itemName=adrianwilczynski.terminal-commands) | Right-click a demo `.rs` file → "Debug demo" / "Upload demo" / "Build demo" |
| [Error Lens](https://marketplace.visualstudio.com/items?itemName=usernamehw.errorlens) | See `#![no_std]` compile errors inline without hovering |
| [Serial Monitor](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor) | Quick serial terminal to the hub shell — send commands, see demo output |
| [Hex Editor](https://marketplace.visualstudio.com/items?itemName=ms-vscode.hexeditor) | Inspect uploaded `.bin` files |
| [Even Better TOML](https://marketplace.visualstudio.com/items?itemName=tamasfe.even-better-toml) | Cargo.toml and linker script validation |

### Minimal setup

```
rust-analyzer + CodeLLDB + Run Terminal Command
```

## Prerequisites

- `arm-none-eabi-objcopy` — extracts `.bin` from ELF
- Udev rules installed — `sudo cp helper-tools/99-spike-hub.rules /etc/udev/rules.d/`
- Hub running SPIKE RTIC firmware (flashed via `cargo xtask flash`)

## Dependencies

- `serialport = "4"` — USB serial port enumeration and I/O
- Pure Rust, no C dependencies
