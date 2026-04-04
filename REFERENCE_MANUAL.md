# LEGO SPIKE Prime Hub — Reference Manual

Technical reference for the RTIC v2 firmware internals: hardware specifications,
memory layout, architecture, the MonitorApi callback table, and design decisions.

For getting started, shell commands, and how-to guides, see [USER_MANUAL.md](USER_MANUAL.md).

---

## Contents

- [1. Hardware](#1-hardware)
  - [1.1 Microcontroller & Memory](#11-microcontroller--memory)
  - [1.2 Peripherals](#12-peripherals)
  - [1.3 External SPI Flash (32 MB)](#13-external-spi-flash-32-mb)
- [2. Memory Map](#2-memory-map)
  - [2.1 Internal Flash (1 MB)](#21-internal-flash-1-mb)
  - [2.2 SRAM (320 KB)](#22-sram-320-kb)
  - [2.3 External SPI Flash (32 MB)](#23-external-spi-flash-32-mb)
- [3. Architecture](#3-architecture)
  - [3.1 Source Tree](#31-source-tree)
  - [3.2 RTIC Conventions](#32-rtic-conventions)
  - [3.3 MonitorApi — Callback Table](#33-monitorapi--callback-table)
  - [3.4 Trace Buffer](#34-trace-buffer)
  - [3.5 Abort Mechanism](#35-abort-mechanism)
  - [3.6 DWT Self-Hosted Watchpoints](#36-dwt-self-hosted-watchpoints)
  - [3.7 GDB RSP Stub](#37-gdb-rsp-stub)
- [4. Design Philosophy](#4-design-philosophy)
  - [4.1 Teletype Heritage](#41-teletype-heritage)
  - [4.2 COBS Binary Upload](#42-cobs-binary-upload)
  - [4.3 Demon Heritage](#43-demon-heritage)
  - [4.4 MCP-Assisted Development](#44-mcp-assisted-development)
- [5. Safety & Sandbox](#5-safety--sandbox)
- [6. Performance Notes](#6-performance-notes)
- [7. Achievement Status](#7-achievement-status)
- [8. Code Provenance](#8-code-provenance)

---

## 1. Hardware

### 1.1 Microcontroller & Memory

| Component       | Details                                       |
| --------------- | --------------------------------------------- |
| MCU             | STM32F413VGT6 (Cortex-M4F @ 96 MHz)          |
| Internal Flash  | 1 MB (32 KB bootloader + 992 KB firmware)     |
| Internal RAM    | 320 KB (256 KB SRAM1 + 64 KB SRAM2)          |
| External Flash  | 32 MB Winbond W25Q256JV on SPI2               |
| Crystal         | 16 MHz HSE → 96 MHz PLL                       |

### 1.2 Peripherals

| Peripheral | Details                                                |
| ---------- | ------------------------------------------------------ |
| Display    | 5×5 LED matrix via TLC5955 (SPI1)                      |
| USB        | OTG FS — CDC serial (VID:PID 0x0694:0x0042)            |
| DFU        | LEGO bootloader (VID:PID 0x0694:0x0011)                |
| Buttons    | Center/Left/Right via ADC resistor ladder              |
| Battery    | LiPo with voltage, current, NTC via ADC                |
| Motors     | 6× H-bridge PWM (TIM1/TIM2/TIM3/TIM12, ports A–F)    |
| Sensors    | 6× LPF2 LUMP UART (2400 sync → 115200 data)           |
| Speaker    | Piezo via TIM4 PWM + DAC1 triangle wave                |
| IMU        | LSM6DS3TR-C 6-DOF (I2C2 bit-bang)                      |

### 1.3 External SPI Flash (32 MB)

The hub has a **Winbond W25Q256JV** (32 MB) on SPI2:

| Pin  | GPIO | Function |
| ---- | ---- | -------- |
| CS   | PB12 | Chip select (active low) |
| SCK  | PB13 | SPI clock (12 MHz) |
| MISO | PC2  | Master-In Slave-Out |
| MOSI | PC3  | Master-Out Slave-In |

The driver (`ext_flash.rs`) provides JEDEC ID read, byte/page read,
page program (256 B), sector erase (4 KB), block erase (64 KB), and
chip erase.  All operations use 4-byte addressing (required for >16 MB).

In the stock LEGO firmware this flash stores MicroPython scripts and
assets.  In our RTIC firmware it is available as 32 MB of external
block storage for demos, data logging, or firmware images.

---

## 2. Memory Map

### 2.1 Internal Flash (1 MB)

```
0x08000000  ┌─────────────────────────┐
            │  LEGO DFU Bootloader    │  32 KB (factory, DO NOT ERASE)
0x08008000  ├─────────────────────────┤
            │  RTIC Firmware          │  ~140 KB (current build)
            ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤
            │  Free                   │  ~852 KB available
0x08100000  └─────────────────────────┘
```

> **Plenty of room:** the firmware uses ~140 KB of the 992 KB available.
> New commands and drivers add only a few hundred bytes each.

### 2.2 SRAM (320 KB)

```
0x20000000  ┌─────────────────────────┐
            │  .data + .bss           │  firmware statics
            │  (painted with 0xDEAD   │  ~200 KB sentinel zone
            │   BEEF sentinels at     │  (for high-water mark)
            │   boot)                 │
            │  ▓▓ MSP guard band ▓▓   │  32 B, MPU region 5 (AP=NONE)
            │  ↓ MSP stack (grows ↓)  │
0x20040000  ├─────────────────────────┤  256 KB SRAM1
            │  Upload/Demo buffer     │  64 KB SRAM2
            │  ▓▓ PSP guard band ▓▓   │  32 B, MPU region 6 (AP=NONE)
            │  ↑ Demo PSP stack (4K)  │  0x2004E000–0x2004F000
0x2004FFE0  │  Fault marker (12 B)    │  survives reset
0x2004FFF0  │  DFU magic (16 B)      │
0x20050000  └─────────────────────────┘
```

**Guard bands:** 32-byte MPU regions set to AP=NONE (no access at any
privilege level).  Any touch triggers MemManage — catches stack overflows
before they corrupt adjacent memory.

**Stack painting:** At boot, ~200 KB of unused SRAM1 is painted with
`0xDEADBEEF` sentinels.  The `info` command scans for the first
overwritten sentinel to report high-water mark.

**Fault marker:** If the MSP guard band fires (firmware stack overflow),
the MemManage handler writes MMFSR + faulting address to SRAM2 (survives
reset).  On next boot, `info` reports the previous fault.

### 2.3 External SPI Flash (32 MB)

```
0x00000000  ┌─────────────────────────┐
            │  Winbond W25Q256JV      │  32 MB
            │  (SPI2, 12 MHz clock)   │  4 KB sectors, 256 B pages
0x02000000  └─────────────────────────┘
```

Accessed via `spiflash` shell commands or the `ext_flash` module.

---

## 3. Architecture

### 3.1 Source Tree

```
spike_rtic/
├── src/
│   ├── main.rs           RTIC app: init, USB ISR, async tasks
│   ├── shell.rs          Command parser + 4 KB output buffer
│   ├── clocks.rs         PLL config (HSE 16 → 96 MHz)
│   ├── power.rs          Power hold, shutdown, DFU re-entry
│   ├── pins.rs           Complete GPIO/peripheral pin mapping
│   ├── reg.rs            Register read/write/modify helpers
│   ├── led_matrix.rs     TLC5955 SPI driver + 5×5 patterns
│   ├── usb_serial.rs     USB OTG FS CDC-ACM (0x0694:0x0042)
│   ├── upload.rs         COBS receiver + CRC-32
│   ├── ram_test.rs       March C- memory test
│   ├── motor.rs          H-bridge PWM (6 ports, dynamic pin)
│   ├── sensor.rs         LPF2 LUMP async (2400→115200 baud)
│   ├── sound.rs          Piezo TIM4 PWM, async beep_tone
│   ├── ext_flash.rs      W25Q256JV SPI2 driver (32 MB)
│   ├── rtty.rs           RTTY 45.45 baud Baudot encoder
│   ├── trace.rs          RAM trace buffer (256×8B ring)
│   ├── sandbox.rs        MPU sandbox + SVC dispatch + stack guard bands
│   ├── imu.rs            LSM6DS3TR-C 6-DOF IMU (I2C2 bit-bang)
│   ├── user_app_io.rs    Demo callbacks, abort (setjmp/longjmp), lifecycle
│   ├── servo.rs          Pybricks-style servo (closed-loop)
│   ├── task_state.rs     AtomicU32 task active/idle tracker
│   ├── dwt.rs            DWT self-hosted hardware watchpoints
│   └── gdb_rsp.rs        GDB Remote Serial Protocol stub
├── spike-hub-api/        Shared MonitorApi crate (§3.3)
├── examples/
│   └── hub-ram-demos/    RAM demo examples
├── helper-tools/         All Python & shell helper scripts
├── AIcoder-thinkings/    AI agent memory & reasoning logs
├── dev_notes/            Design notes and analysis
└── memory.x              Linker script (flash @ 0x08008000)
```

### 3.2 RTIC Conventions

- All mutable state lives in `#[shared]` or `#[local]` resources.
- **No `static mut` for application state.** The sole `static mut`
  (`UPLOAD_BUF`) is pinned to `#[link_section = ".sram2"]` — a
  hardware constraint RTIC's resource system doesn't support.
- The `extern "C"` / `#[repr(C)]` in MonitorApi is **not C code** —
  it's the stable ABI for separately-compiled Rust binaries.
- All waits use `Mono::delay().await` — no spin loops, no busy waits.

**Priority scheme:**

| Priority | Tasks |
| -------- | ----- |
| 3 | UART ISRs (sensor) |
| 2 | All firmware RTIC tasks (heartbeat, sensor, motor, shell), USB ISR |
| 1 | User-facing tasks (run_demo, test_all) |

SVC only works from Thread mode.  SVCall (priority 15) is lower than
any RTIC dispatcher.  Sandbox demos run from `#[idle]` (Thread mode).

### 3.3 MonitorApi — Callback Table

The firmware passes a `*const MonitorApi` to RAM demos.  This
`#[repr(C)]` struct is the demo's only interface to the hardware.
Defined in the shared [`spike-hub-api`](spike-hub-api/) crate.

Current version: **`API_VERSION = 10`** (24 fields, 96 bytes on 32-bit ARM).

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

**Sandboxed** (`go`): demo runs unprivileged with MPU; API calls go
through SVC traps.  **Privileged** (`go!`): direct function pointers,
no MPU — for firmware development.

### 3.4 Trace Buffer

The firmware has a lightweight **RAM trace buffer** for post-mortem
debugging — a fast alternative to printf that doesn't block execution.

- **256 entries × 8 bytes = 2 KB** ring buffer in SRAM1
- Each entry: `tag` (u8), `val` (u8), `arg` (u16), `tick` (u32)
- Atomic writes, no locks — safe from any interrupt priority
- Dump via `trace` shell command

Demos can record traces via `api.trace_record(TAG_USER, val, arg)`.
Use `trace on` to enable, `trace` to dump, `trace legend` for tag names.

Tags: `01`=SVC, `02`=SVC_RET, `10`=SENSOR_READ, `11`=SENSOR_DATA,
`12`=KEEPALIVE, `13`=LUMP_MSG, `20`=DELAY, `40`=DEMO_START/END,
`80`=USER (demo custom traces).

> **When to use tracing vs. printf:** tracing records 8 bytes atomically
> with zero I/O — typically 10–20 cycles.  A `write_fn` call goes
> through 32 KB of ring-buffered CDC output — much slower.  Use tracing
> for tight loops and timing-sensitive code; use `write_fn` for
> human-readable progress messages.

### 3.5 Abort Mechanism

The firmware uses ARM `setjmp`/`longjmp` (implemented in `global_asm!`)
to yank control from a running demo:

1. `run_demo` calls `abort_setjmp()` before entering the demo
2. When ABORT is requested, the next MonitorApi callback calls
   `abort_longjmp()` — restoring registers (r4–r11, SP, LR) to the
   saved context
3. `run_demo` sees `jumped != 0`, cleans up (brake motors, stop sound),
   and returns to the shell

This works because every possible callback path (write, delay, motor,
sound, sensor, RTTY) checks the ABORT flag and longjmps if set.

### 3.6 DWT Self-Hosted Watchpoints

The `dwt.rs` module implements self-hosted hardware watchpoints using
the Cortex-M4 **Data Watchpoint and Trace** (DWT) unit — no JTAG or
SWD debugger required.

**How it works:**

- The DWT has 4 hardware comparators at `0xE000_1000` base
- Each comparator can match on data read, data write, data read/write,
  or instruction address (PC match)
- When a match occurs, the **DebugMonitor** exception (priority 0x40)
  fires — lower than firmware tasks, so it doesn't interfere with RTIC
- The handler scans all 4 MATCHED bits, records hit counts and the
  stacked PC in atomic variables, then returns

**DWT initialization (`dwt::init()`):**

1. Unlock DWT via `DWT_LAR = 0xC5ACCE55`
2. Enable TRCENA in DEMCR (`0xE000_EDFC`)
3. Enable MON_EN in DEMCR (DebugMonitor exception)
4. Enable DWT cycle counter
5. Set DebugMonitor priority to `0x40` (below RTIC dispatchers)

**Key registers:**

| Register | Address | Purpose |
| -------- | ------- | ------- |
| DWT_CTRL | 0xE0001000 | Comparator count (bits 31:28), cycle counter enable |
| DWT_COMPn | 0xE0001020 + 16n | Comparator value (address to watch) |
| DWT_MASKn | 0xE0001024 + 16n | Address mask (power-of-2 alignment) |
| DWT_FUNCTIONn | 0xE0001028 + 16n | Function select + MATCHED bit |
| DEMCR | 0xE000EDFC | Debug enable: TRCENA, MON_EN |
| DWT_LAR | 0xE0001FB0 | Lock Access Register (write 0xC5ACCE55) |

**Watchpoint functions (DWT_FUNCTIONn bits 3:0):**

| Value | Name | Meaning |
| ----- | ---- | ------- |
| 0 | Disabled | Comparator off |
| 4 | PcMatch | Instruction address match |
| 5 | DataRead | Data read watchpoint |
| 6 | DataWrite | Data write watchpoint |
| 7 | DataRW | Data read or write watchpoint |

### 3.7 GDB RSP Stub

The `gdb_rsp.rs` module implements a **GDB Remote Serial Protocol**
stub — "Demon mode" — for remote debugging over USB CDC serial.

**Architecture:**

- `GdbStub` struct manages the RSP state machine
- `enter()` switches shell into RSP mode (all bytes routed to RSP parser)
- `feed(data, resp_buf) -> usize` processes incoming bytes, builds responses
- `dispatch()` handles complete RSP packets
- `RxState` enum: Idle → Data → Checksum1 → Checksum2
- Packet framing: `$data#checksum` with `+`/`-` ACK/NAK

**Memory/register access:**

- `m` reads arbitrary memory regions, hex-encoded
- `M` writes arbitrary memory regions
- `g` reads all 17 ARM registers (r0–r12, SP, LR, PC, xPSR)
- `G` writes all 17 ARM registers

**DWT integration:**

- `Z2,addr,kind` → `dwt::set_watchpoint()` with write function
- `Z3,addr,kind` → `dwt::set_watchpoint()` with read function
- `Z4,addr,kind` → `dwt::set_watchpoint()` with read/write function
- `z2/3/4,addr,kind` → `dwt::clear_watchpoint()`
- Address mask (`DWT_MASKn`) computed from the `kind` (size) parameter

**Capabilities reported by `qSupported`:**

```
PacketSize=256;hwbreak-;swbreak-
```

**Constants:** `PKT_BUF_SIZE=600`, `RESP_BUF_SIZE=600`.

---

## 4. Design Philosophy

### 4.1 Teletype Heritage

The shell speaks plain ASCII — no ANSI escapes, no UTF-8, no colour
codes.  Every response fits in 80 columns.  Output uses `\r\n` (CRLF).

**Input flexibility:** the parser accepts `\r` (CR), `\n` (LF), or
`\r\n` (CRLF).  This means standard Linux tools (`echo`, `printf`,
`picocom`, `screen`, `stty`) all work out of the box.

### 4.2 COBS Binary Upload

**Consistent Overhead Byte Stuffing** (COBS) encodes arbitrary
binaries so that zero bytes never appear in the payload.  A single
`0x00` byte serves as an unambiguous frame delimiter.

```
Host:  raw binary → COBS encode → append 0x00
Hub:   receive until 0x00 → COBS decode in-place → verify CRC-32
```

The upload buffer lives at `0x2004_0000` (SRAM2, 64 KB).

### 4.3 Demon Heritage

The predecessor to ARM Angel was **Demon** — the original ARM debug
monitor.  Before Angel existed, ARM systems used Demon, a ROM-resident
debug monitor that provided:

- A command-line shell running on the SoC itself
- Memory/register inspection
- Breakpoints
- Downloading code over serial
- A simple monitor protocol used by early ARM tools

This firmware follows the same philosophy: a resident monitor that
provides full hardware access through a serial shell, extended with
DWT hardware watchpoints and a GDB RSP protocol stub for remote
debugging — all self-hosted, no external debugger required.

### 4.4 MCP-Assisted Development

This firmware was developed with **Model Context Protocol** (MCP)
tooling — GitHub Copilot with MCP server integration for direct hub
interaction.  The development loop:

1. Copilot reads firmware source and copilot-instructions
2. Copilot edits code, builds with `cargo build --release`
3. Human enters DFU mode (center button hold or left button)
4. Copilot flashes via `dfu-util` and verifies with `lsusb`
5. Copilot tests commands over `/dev/ttyACM0`

---

## 5. Safety & Sandbox

- **MPU always on with 7 regions.**  The MPU is permanently enabled
  (PRIVDEFENA=1) — not just during demo execution.  Regions 0–4
  enforce the sandbox (SRAM2 RW, Flash RO+exec, SRAM1 priv-only,
  peripherals priv-only, PPB priv-only).  Region 5 is a 32-byte
  guard band at the bottom of the firmware MSP stack.  Region 6 is
  a 32-byte guard band below the demo PSP stack.  Both are AP=NONE
  (no access at any privilege level) — any touch triggers MemManage.

- **Stack painting + high-water mark.**  At boot the firmware paints
  ~200 KB of unused stack space with 0xDEADBEEF sentinels.  The
  `info` shell command reports how many bytes the stack has actually
  used (scanning for the first overwritten sentinel).

- **Previous-boot fault marker.**  If the MSP guard band fires
  (firmware stack overflow), the MemManage handler writes a fault
  marker to SRAM2 (survives reset).  On the next boot, `info`
  reports the MMFSR and faulting address.

- **SVC dispatch trusts arguments.** The SVC thunks do no parameter
  validation on pointers passed through the API.  A sandboxed demo
  can't write SRAM1, but it could pass a garbage pointer that would
  fault when the SVC handler dereferences it in privileged mode.

- **`go!` is fully privileged.** No SVC, direct function pointers.
  A bug = firmware crash.  Intentional for development only.

- **Abort uses `longjmp`** — safe because demos are leaf functions
  with no Rust destructors and no access to kernel resources.

---

## 6. Performance Notes

- **SPI flash is polled, not DMA.** Reads and writes block the CPU
  byte-by-byte.  32 MB flash at 12 MHz SPI could be much faster
  with DMA transfers.
- **No power management.** The MCU runs at 96 MHz always.  No
  dynamic clock scaling, no sleep during idle.
- **LED matrix refresh is synchronous.** The TLC5955 SPI1 update
  blocks while pushing 576 bits.  Not a problem at current refresh
  rates.

---

## 7. Achievement Status

| Milestone                                            | Status |
| ---------------------------------------------------- | ------ |
| RTIC v2 firmware boots on SPIKE Prime hub            | Done   |
| USB CDC serial shell with 50+ commands               | Done   |
| 5×5 LED matrix — patterns, digits, per-pixel         | Done   |
| ADC: battery voltage/current/NTC, buttons            | Done   |
| COBS binary upload + CRC-32 verification             | Done   |
| Demon-style `go` — execute RAM demos (sandbox + priv)| Done   |
| March C- RAM tests (SRAM1/SRAM2)                     | Done   |
| Automated test suite — 65/65 tests pass              | Done   |
| Motor H-bridge PWM driver (all 6 ports)              | Done   |
| MonitorApi callbacks for RAM demos (v5)              | Done   |
| MonitorApi v10: IMU, port_read, sensor_light, hub_led | Done   |
| LPF2 LUMP sensor driver (async ISR, 115200 baud)    | Done   |
| Color sensor sync + continuous data streaming        | Done   |
| Per-port ring buffers (6 × 512 B SPSC)              | Done   |
| Motor position tracking (motor_poll task)            | Done   |
| Closed-loop motor_goto positioning (±9°)             | Done   |
| Sound driver — async non-blocking `beep_tone`        | Done   |
| External SPI flash driver (W25Q256JV, 32 MB)         | Done   |
| `spiflash` shell commands (read/write/erase)         | Done   |
| RAM trace buffer (256 entries, atomic, zero-cost)    | Done   |
| Trace API for demos (`trace_record` in MonitorApi)   | Done   |
| `version` / `time` / `unixtime` commands             | Done   |
| `ps` task tree + `ls` memory map                     | Done   |
| Process management: kill -2/-9/-15, Ctrl-C, ringbutton SIGKILL | Done   |
| Interactive shell during demo (background jobs)      | Done   |
| setjmp/longjmp abort + force_kill_sandbox (kill -9)  | Done   |
| MPU sandbox: fault recovery, re-run without reset    | Done   |
| SPI flash store/load/run workflow                    | Done   |
| DWT self-hosted hardware watchpoints (4 comparators) | Done   |
| GDB RSP stub over USB CDC ("Demon mode")            | Done   |

---

## 8. Code Provenance

- **AI-assisted codebase.** This firmware was developed with Claude
  Opus 4.6 via MCP tooling.  The human provided direction, hardware
  knowledge, and testing — the AI wrote most of the Rust code.
  This is stated openly because it affects how you should trust it:
  the code works and passes 65 automated tests, but it hasn't had
  the multi-year battle-testing that projects like Pybricks have.
- **Pre-release quality.** This is a proof-of-concept that grew
  into a functional system.  It is not production firmware.  Use
  it to learn, experiment, and build on — not to ship.

---

*See also:* [USER_MANUAL.md](USER_MANUAL.md) for getting started,
shell commands, and how-to guides.
