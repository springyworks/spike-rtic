# hub-ram-demos — RAM Demo Binaries for SPIKE Prime Hub

Small `no_std` Rust programs that run from SRAM2 on the LEGO SPIKE Prime
hub.  Uploaded via COBS over USB CDC, executed via the firmware's `go`
command — no reflash required.

> This is the **Demon resident monitor** pattern from the 1980s ARM world:
> the host pushes a raw binary into target RAM, the monitor hands off
> control, and the demo talks back through a C-ABI callback table.

<!-- TOC -->

## Contents

- [1. Quick Start](#1-quick-start)
  - [1.1 Build](#11-build)
  - [1.2 Upload & Run](#12-upload--run)
- [2. How It Works](#2-how-it-works)
  - [2.1 Execution Flow](#21-execution-flow)
  - [2.2 MonitorApi Callback Table](#22-monitorapi-callback-table)
  - [2.3 Sandboxed vs. Privileged](#23-sandboxed-vs-privileged)
  - [2.4 Using the Trace Buffer](#24-using-the-trace-buffer)
- [3. Available Demos](#3-available-demos)
- [4. Writing a New Demo](#4-writing-a-new-demo)
  - [4.1 Minimal Template](#41-minimal-template)
  - [4.2 Build & Extract Binary](#42-build--extract-binary)
  - [4.3 Adding to Cargo.toml](#43-adding-to-cargotoml)
- [5. Linker Script](#5-linker-script)
- [6. Important Notes](#6-important-notes)
  - [6.1 Execution Context](#61-execution-context)
  - [6.2 Recovery from Crashes](#62-recovery-from-crashes)
- [7. Dependencies](#7-dependencies)

<!-- /TOC -->

---

## 1. Quick Start

### 1.1 Build

```bash
# From anywhere — $PROJECT_ROOT is auto-set by .bashrc on every cd
cd $PROJECT_ROOT/examples/hub-ram-demos
cargo build --release
```

Extract a raw binary for upload (examples live under `examples/`,
Cargo uses underscores in filenames):

```bash
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/examples/dual_motors \
    target/spike-usr_bins/dual_motors.bin
```

### 1.2 Upload & Run

Power on the hub and connect a terminal to `/dev/ttyACM0` (or
`/dev/ttyACM1`).  You get the `spike>` prompt.

```bash
# From the host:
python3 $PROJECT_ROOT/helper-tools/upload_demo.py dual_motors.bin
# Or with the controller:
python3 $PROJECT_ROOT/helper-tools/spike_hub_controller.py upload dual_motors.bin

# In the hub shell:
spike> bininfo
RAM bin: 412 B @ 0x20040000 CRC 0xABCD1234
spike> go
```

Or upload manually:

```
spike> upload 412
READY 412
<send COBS-encoded bytes>
OK 412 CRC=0xABCD1234
spike> go
```

For DFU flashing of the firmware itself, see the
[main README §7](../../README.md#7-dfu--recovery).

---

## 2. How It Works

### 2.1 Execution Flow

```
  host                          hub (RTIC firmware)
  ────                          ───────────────────
  helper-tools/upload_demo.py demo.bin  → COBS decode into SRAM2 @ 0x20040000
  spike> go                     → build MonitorApi, call _start(api)
                                ← demo prints via api.write_fn()
                                ← demo returns u32 exit code
  spike>                        → shell resumes
```

The firmware stays running.  The demo is a guest that borrows the
monitor's I/O, LEDs, ADC, motors, sensors, and speaker through the
`MonitorApi` callback struct.

**Process management:** the shell stays fully interactive while a demo
runs (like a Unix background job).  You can type `uptime`, `ps`, or
any shell command during execution.  Kill a running demo with:

| Method | Signal | Behavior |
| ------ | ------ | -------- |
| **`kill`** or **`kill -15`** | SIGTERM | Cooperative: demo exits at next API call |
| **`kill -2`** | SIGINT | Same as SIGTERM |
| **`kill -9`** | SIGKILL | Immediate: force-kill + brake motors + stop sound |
| **Ctrl-C** | SIGTERM | Sets ABORT flag |
| **Ring button** hold 1–6 s | SIGKILL | Hard-kill via `force_kill_sandbox` |
| **Right button** | SIGKILL + relaunch | Kill running demo, then launch from SPI flash |

All kill paths terminate the demo (exit code `0xDEAD0001`), brake
motors, and stop sound.  **Demos never need exit-on-button logic** —
the firmware handles all lifecycle management.

See the [main README §5.5](../../README.md#55-process-management)
for ring button zone details and the re-run workflow.

### 2.2 MonitorApi Callback Table

The `MonitorApi` is a `#[repr(C)]` struct defined in the shared
[`spike-hub-api`](../../spike-hub-api/) crate.  Current version: **10**
(24 fields, 96 bytes on 32-bit ARM).

| # | Field | Signature | SVC | Since |
|---|-------|-----------|-----|-------|
| 0 | `version` | `u32` | — | v1 |
| 1 | `context` | `*mut u8` | — | v1 |
| 2 | `write_fn` | `fn(ctx, ptr, len)` | 0 | v1 |
| 3 | `delay_ms` | `fn(ms)` | 1 | v1 |
| 4 | `set_pixel` | `fn(index, brightness)` | 2 | v1 |
| 5 | `update_leds` | `fn()` | 3 | v1 |
| 6 | `read_adc` | `fn(ch) → u32` | 4 | v1 |
| 7 | `read_buttons` | `fn() → u8` | 5 | v1 |
| 8 | `motor_set` | `fn(port, speed)` | 6 | v2 |
| 9 | `motor_brake` | `fn(port)` | 7 | v2 |
| 10 | `sensor_read` | `fn(buf, len) → u32` | 8 | v3 |
| 11 | `sensor_mode` | `fn(mode)` | 9 | v3 |
| 12 | `sound_play` | `fn(freq)` | 10 | v4 |
| 13 | `sound_stop` | `fn()` | 11 | v4 |
| 14 | `trace_record` | `fn(tag, val, arg)` | 12 | v5 |
| 15 | `rtty_say` | `fn(ptr, len)` | 13 | v6 |
| 16 | `rtty_busy` | `fn() → u32` | 14 | v6 |
| 17 | `motor_position` | `fn() → i32` | 15 | v7 |
| 18 | `motor_goto` | `fn(port, deg) → i32` | 16 | v7 |
| 19 | `port_read` | `fn(port, buf, len) → u32` | 17 | v8 |
| 20 | `sensor_light` | `fn(r, g, b)` | 18 | v9 |
| 21 | `imu_init` | `fn() → u32` | 19 | v10 |
| 22 | `imu_read` | `fn(buf, len) → u32` | 20 | v10 |
| 23 | `set_hub_led` | `fn(r, g, b)` | 21 | v10 |

See the [main README §5.3](../../README.md#53-monitorapi--callback-table) for details.

### 2.3 Sandboxed vs. Privileged

| Command | Mode        | MPU  | API mechanism       | Use case           |
| ------- | ----------- | ---- | ------------------- | ------------------ |
| `go`    | Sandboxed   | On   | SVC trap → callbacks| User apps          |
| `go!`   | Privileged  | Off  | Direct fn pointers  | Firmware dev/debug |

`go` runs the demo in **unprivileged Thread mode** with MPU protection.
API calls go through SVC traps so the demo can only access SRAM2 and
the callback table.  A crash triggers a MemManage fault → clean recovery.

`go!` runs the demo **privileged** with full memory access.  Useful for
testing firmware-level code, but a crash may require a hard reset
(lift the battery — see [§6.2](#62-recovery-from-crashes) and the
[main README §7.3](../../README.md#73-hard-reset-battery-lift)).

### 2.4 Using the Trace Buffer

Demos can record lightweight trace entries into the firmware's 2 KB
ring buffer using `trace_record`:

```rust
// Record a user trace (tag=0x80)
(api.trace_record)(0x80, my_val, my_arg);
```

This writes 8 bytes atomically — much faster than `write_fn` for
high-frequency instrumentation.  Dump traces later with `trace` in
the shell.  Enable/disable with `trace on` / `trace off`.

See the [main README §5.4](../../README.md#54-trace-buffer) for
tag definitions and usage patterns.

---

## 3. Available Demos

| Demo               | Source                           | API  | What It Does |
| ------------------ | -------------------------------- | ---- | ------------ |
| `spike-demo`       | `src/main.rs`                    | v2+  | Motor ramp/coast/reverse/brake on port B |
| `dual-motors`      | `examples/dual_motors.rs`        | v2+  | Choreographed A+B motor dance with LED status |
| `motor-buttons`    | `examples/motor_buttons.rs`      | v2+  | Button-controlled dual motors (interactive) |
| `color-chaos`      | `examples/color_chaos.rs`        | v4+  | RGBI sensor → derivative-driven motors + RTTY FSK |
| `color-seek`       | `examples/color_seek.rs`         | v3+  | Fast RGBI color classification, motor seek |
| `color-seeker`     | `examples/color_seeker.rs`       | v7+  | Rotational color scanner: motor_goto steps + classify |
| `motor-goto`       | `examples/motor_goto.rs`         | v7+  | Closed-loop position control: goto 90°/0°/360°/0° |
| `motor-sensor-test`| `examples/motor_sensor_test.rs`  | v7+  | Motor + sensor integration test |
| `gdb-exercise`     | `examples/gdb_exercise.rs`       | v2+  | GDB debug target: fibonacci, color classify, motor ramp, LED cross |
| `gdb-simple`       | `examples/gdb_simple.rs`         | v2+  | Minimal GDB test: tight loop for breakpoint/step testing |
| `pwm-diag`         | `examples/pwm_diag.rs`           | v2+  | PWM diagnostics on motor ports |
| `reg-dump`         | `examples/reg_dump.rs`           | v2+  | Hardware register inspection |
| `sensor-dump`      | `examples/sensor_dump.rs`        | v3+  | Raw RGBI hex dump + ring buffer diagnostics |

**Default port layout:** motors on ports **A** and **B**, color sensor
on port **F**.

### Debugging a Demo with GDB

Build and debug `gdb_exercise` (or any demo) using the debug pipeline:

```bash
# Build
cargo build --example gdb_exercise --release
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/examples/gdb_exercise \
    target/spike-usr_bins/gdb_exercise.bin

# Upload + enter RSP mode (auto-detects port)
python3 ../../helper-tools/debug_pipeline.py \
    target/spike-usr_bins/gdb_exercise.bin

# Connect GDB with symbols
gdb-multiarch \
    target/thumbv7em-none-eabihf/release/examples/gdb_exercise \
    -ex "set architecture arm" \
    -ex "target remote /dev/ttyACM0"
```

VS Code: press **F5** — the workspace tasks automate the entire
build → objcopy → upload → GDB attach cycle.

See [USER\_MANUAL.md §2.8](../../USER_MANUAL.md#28-gdb-remote-debug)
for full GDB usage and
[dev\_notes/gdb-debugging.md](../../dev_notes/gdb-debugging.md) for
the architecture.

---

## 4. Writing a New Demo

### 4.1 Minimal Template

```rust
#![no_std]
#![no_main]

use spike_hub_api::{MonitorApi, BTN_CENTER};

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    // Check API version
    if api.version < 7 {
        api.print(b"ERR: need API v7+\r\n");
        return 1;
    }

    api.print(b"Hello from SRAM2!\r\n");

    // Blink pixel 12 three times
    for _ in 0..3 {
        (api.set_pixel)(12, 80);
        (api.update_leds)();
        (api.delay_ms)(200);
        (api.set_pixel)(12, 0);
        (api.update_leds)();
        (api.delay_ms)(200);
    }

    // Record a trace entry
    (api.trace_record)(0x80, 0, 42);

    0  // exit code
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
```

### 4.2 Build & Extract Binary

```bash
cargo build --release --bin my-demo
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/my-demo my-demo.bin
```

### 4.3 Adding to Cargo.toml

Add an `[[example]]` entry (demos in `examples/` are auto-detected)
or add a `[[bin]]` entry for demos in `src/bin/`:

```toml
[[bin]]
name = "my-demo"
path = "src/bin/my_demo.rs"
test = false
bench = false
```

---

## 5. Linker Script

All demos link to **0x20040000** (SRAM2, 64 KB minus 16 bytes reserved
for DFU magic at 0x2004FFF0):

```
ENTRY(_start)
MEMORY { RAM : ORIGIN = 0x20040000, LENGTH = 0xFFF0 }
SECTIONS {
    .text : { *(.text._start) *(.text .text.*) } > RAM
    .rodata : { *(.rodata .rodata.*) } > RAM
    .data : { *(.data .data.*) } > RAM
    .bss : { *(.bss .bss.*) } > RAM
}
```

---

## 6. Important Notes

### 6.1 Execution Context

- The demo runs as an RTIC task at priority 1.  The USB ISR (priority 2)
  continues draining output while the demo runs.
- **Shell stays interactive** — you can run any command (`ps`, `uptime`,
  `kill`, etc.) while a demo runs.  The demo is a background job.
- **Kill/Ctrl-C/button abort** — the firmware can terminate any demo
  at any time.  SIGTERM/SIGINT (`kill`, `kill -2`, Ctrl-C) set the ABORT
  flag checked by every MonitorApi callback.  SIGKILL (`kill -9`, ring
  button hold 1–6 s, right button) uses `force_kill_sandbox` to hijack
  the exception return — kills even tight `loop {}` demos instantly.
  Demos don't need to poll for exit conditions.
- **No vector table** — exceptions and interrupts are handled by the
  firmware.  The demo is a function call, not a full firmware.
- **Stack is shared** — the demo uses the monitor's stack.  Avoid large
  stack allocations.
- The `go` command automatically sets bit 0 of the address (Thumb BLX
  requirement on Cortex-M).
- The last 16 bytes of SRAM2 (`0x2004FFF0`) are reserved for the DFU
  bootloader magic word.  Don't use them.

### 6.2 Recovery from Crashes

| Situation | Recovery |
| --------- | -------- |
| Demo killed (`kill -15` / Ctrl-C) | ABORT flag → demo exits at next API call (exit `0xDEAD0001`) |
| Demo force-killed (`kill -9` / ring button / right button) | `force_kill_sandbox` → immediate exit, motors braked |
| Sandboxed demo crash (`go`) | MemManage fault → clean recovery, shell resumes automatically |
| Privileged demo crash (`go!`) | May need `reset` or hard reset |
| Hub completely stuck | Lift battery out for 1 second, re-insert |

After lifting the battery, hold **center** to boot normally or
**center + left** for DFU mode.  See the
[main README §7.3](../../README.md#73-hard-reset-battery-lift) for details.

---

## 7. Dependencies

- [`spike-hub-api`](../../spike-hub-api/) crate — shared `MonitorApi`
  struct definition.  Located at `../../spike-hub-api` relative to this
  directory.
- Rust nightly toolchain with `thumbv7em-none-eabihf` target.
- `arm-none-eabi-objcopy` (or `rust-objcopy`) for binary extraction.
