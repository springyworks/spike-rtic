# Copilot Instructions — SPIKE RTIC Project

## Principles


- **Read the docs first:** Read all the readms and check AIcoder-thinkings/spike-rules.md' ; maybe consult `dev_notes/` and this file before making changes. 
- **No busy-waits in firmware:** All waits must use `Mono::delay().await`. Only user apps (SRAM2 demos) may use `delay_ms` via SVC, which internally chunks waits and maintains sensor keepalive.
- **Firmware is infrastructure, not application:** Application logic belongs in user apps (hub-ram-demos), not firmware tasks. Firmware exposes APIs (MonitorApi); user apps consume them.
- **Don't break what works:** Understand existing code before changing it. Never remove stable, working code as a shortcut. If a fix for X breaks Y, revert.
- **Motor: open-loop PWM has physics limits:** No hacks to "fix" stiction; see `dev_notes/pybricks_lowspeed_motor_analysis.md`.
- **Sensor: stay in mode 5, classify in software:** Avoid mode switches during operation. See `dev_notes/sensor-fixes.md`.
- **LUMP keepalive < 250 ms:** Never block the keepalive path. See `demo_maintain()` and `sensor_poll`.
- **Shell uses `\r` not `\r\n`:** All shell commands end with `\r` (CR). `\r\n` is not accepted.
- **Flash with dfu-util only:** Use the documented dfu-util command. User enters DFU mode manually.
- **Don't over-engineer:** No speculative features or unrelated changes. Ask if scope is unclear. Keep docs in `dev_notes/`.
- **Every demo must check API version:** All RAM demo `_start` functions must check `api.version >= N` before using fields added in version N. Prevents silent crashes from firmware/demo version mismatch.
- **No stale .bin files:** Binaries live only in `target/` build outputs and `target/spike-usr_bins/`. Never leave `.bin` files in the repo root, `examples/hub-ram-demos/`, or other ad-hoc locations — stale bins cause phantom bugs.

## Project Structure

- **Firmware:** `src/` — RTIC app, drivers, shell, upload, etc.
- **Shared API:** `spike-hub-api/` — MonitorApi callback table, constants.
- **User Apps:** `examples/hub-ram-demos/` — RAM demo binaries, built and uploaded separately.
- **Pre-built bins:** `examples/hub-ram-demos/target/spike-usr_bins/` — objcopy'd .bin files ready for upload.
- **Helper Tools:** `helper-tools/` — ALL Python (.py) and shell (.sh) scripts live here.
- **AI Memory:** `AIcoder-thinkings/` — AI agent memory stores and reasoning logs.
- **Documentation:** `dev_notes/` — All design notes, rules, and analysis.

## Build & Test

- **Build firmware:**
  ```sh
  cd $PROJECT_ROOT
  cargo build --release
  ```
- **Build RAM demos:**
  ```sh
  cd $PROJECT_ROOT/examples/hub-ram-demos
  cargo build --release
  ```
- **Extract demo binary:**
  ```sh
  arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/<demo> <demo>.bin
  ```
- **Flash firmware:**
  ```sh
  dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D $PROJECT_ROOT/target/spike-rtic.bin
  ```
- **Upload & run demo:**
  ```sh
  python3 $PROJECT_ROOT/helper-tools/upload_demo.py <demo>.bin
  ```

## Key Conventions

- **No `static mut` for app state:** Use RTIC resources (`#[shared]`, `#[local]`). Only exception: `UPLOAD_BUF` in `.sram2`.
- **MonitorApi is Rust, not C:** Uses `extern "C"`/`#[repr(C)]` for stable ABI, not actual C code.
- **All waits are async:** Use `Mono::delay().await` everywhere except user apps.
- **Trace vs. printf:** Use trace buffer for fast, atomic logging; use `write_fn` for human-readable output.

## Links

- [Main README](../README.md)
- [RAM Demos README](../examples/hub-ram-demos/README.md)
- [MonitorApi API](../spike-hub-api/README.md)
- [Dev Notes](../dev_notes/)

---

*For agent customization, see `dev_notes/copilot-self-rules.md` for the canonical rules. Link, don't duplicate.*
