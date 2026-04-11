# xtask — SPIKE Hub Firmware Manager

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---

Interactive CLI for firmware workflows: build, flash, upload demos, serial  
shell, and hub diagnostics. Zero external dependencies (pure `std`).

## Usage

From the project root:

```
cargo xtask              # interactive menu
cargo xtask flash        # build release firmware + flash via dfu-util
cargo xtask build        # build firmware (release) without flashing
cargo xtask build-demos  # build all RAM demos
cargo xtask upload       # pick a demo interactively, build + upload
cargo xtask upload NAME  # build + upload a specific demo
cargo xtask connect      # open picocom serial shell to the hub
cargo xtask status       # show hub state, port map, and tool availability
```

### What each command does

| Command | Description |
| --- | --- |
| _(none)_ | Interactive menu — pick from all available actions |
| `flash` | `cargo build --release` → `objcopy` → `dfu-util` flash to `0x08008000` |
| `build` | `cargo build --release` only (no flash) |
| `build-demos` | Builds all demos in `examples/hub-ram-demos/` |
| `upload [NAME]` | Build demo → objcopy `.bin` → COBS upload → `go` (run it) |
| `connect` | Opens `picocom` at 115200 baud on the detected shell port |
| `status` | Port diagnostic table (Device, VID:PID, IF#, Role, Symlink, Status), DFU check, tool versions |
| `run-elf` | Called by `cargo run` via `runner.sh` — shows flash dialog for the just-built ELF |

### How `cargo run` works

The `.cargo/config.toml` sets `runner = "bash xtask/runner.sh"` for the ARM  
target. When you `cargo run --release`, the runner script forwards the ELF  
path to `cargo xtask run-elf`, which offers an interactive flash dialog  
instead of trying to execute the ARM binary on x86.

## Port Detection

The `status` command prints a full port diagnostic table showing all  
`ttyACM*` ports with USB identity, interface number, role (shell/gdb),  
symlink status, and busy state. Prefers udev symlinks (`/dev/spike-shell`,  
`/dev/spike-gdb`) with fallback to sysfs scanning. See  
[dev\_notes/port-detection.md](../dev_notes/port-detection.md).

## VS Code Integration

This xtask is registered in `.cargo/config.toml` as an alias:

```
[alias]
xtask = "run --manifest-path xtask/Cargo.toml --target x86_64-unknown-linux-gnu --"
```

VS Code tasks in `.vscode/tasks.json` and the workspace file call into it.

## Recommended VS Code Extensions

These extensions make firmware development significantly easier:

| Extension | What it does |
| --- | --- |
| [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer) | Rust language server — completions, errors, inlay hints, go-to-definition. Essential. |
| [CodeLLDB](https://marketplace.visualstudio.com/items?itemName=vadimcn.vscode-lldb) | Debug adapter for LLDB — used for GDB RSP sessions with the hub |
| [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) | ARM Cortex-M debug UI — register views, peripheral inspector, SWO/ITM |
| [Run Terminal Command](https://marketplace.visualstudio.com/items?itemName=adrianwilczynski.terminal-commands) | Right-click context menu for "Debug demo", "Upload demo", etc. (configured in workspace file) |
| [Error Lens](https://marketplace.visualstudio.com/items?itemName=usernamehw.errorlens) | Inline compiler errors — see `rust-analyzer` diagnostics without hovering |
| [Even Better TOML](https://marketplace.visualstudio.com/items?itemName=tamasfe.even-better-toml) | Syntax highlighting + validation for `Cargo.toml`, `.cargo/config.toml` |
| [Serial Monitor](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor) | Built-in serial terminal — alternative to picocom for quick shell access |
| [Hex Editor](https://marketplace.visualstudio.com/items?itemName=ms-vscode.hexeditor) | Inspect `.bin` files and raw serial dumps |
| [GitLens](https://marketplace.visualstudio.com/items?itemName=eamodio.gitlens) | Git blame, history, and diff — useful when tracking down regressions |

### Minimal setup

```
rust-analyzer + CodeLLDB + Run Terminal Command
```

These three cover: code intelligence, debugging, and one-click demo  
build/upload/debug from the editor.

## Prerequisites

*   `arm-none-eabi-objcopy` (or `rust-objcopy`) — for `.bin` extraction
*   `dfu-util` — for flashing firmware
*   `picocom` — for `connect` command (serial terminal)
*   Udev rules installed — see `helper-tools/99-spike-hub.rules`