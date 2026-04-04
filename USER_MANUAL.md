# LEGO SPIKE Prime Hub — User Manual

Bare-metal Rust firmware for the **LEGO SPIKE Prime Hub** using **RTIC v2**.
An interactive **Demon-style debug monitor** over USB CDC serial — control
motors, read sensors, dump memory, upload binaries, and access 32 MB of
external SPI flash — all without a hardware debugger.

> **Status:** proof-of-concept, improving steadily.
> **Firmware size:** ~140 KB of 992 KB — plenty of room for new features.

---

## Contents

- [1. Quick Start](#1-quick-start)
  - [1.1 Build & Flash](#11-build--flash)
  - [1.2 Connect to the Shell](#12-connect-to-the-shell)
  - [1.3 Upload & Run a RAM Demo](#13-upload--run-a-ram-demo)
- [2. Shell Commands](#2-shell-commands)
  - [2.1 General](#21-general)
  - [2.2 Upload & Execute](#22-upload--execute)
  - [2.3 Demon Debug Monitor](#23-demon-debug-monitor)
  - [2.4 Motor Control](#24-motor-control)
  - [2.5 Sensor (LPF2 LUMP)](#25-sensor-lpf2-lump)
  - [2.6 External Flash (SPI2)](#26-external-flash-spi2)
  - [2.7 DWT Watchpoints](#27-dwt-watchpoints)
  - [2.8 GDB Remote Debug](#28-gdb-remote-debug)
  - [2.9 System & Misc](#29-system--misc)
- [3. Process Management](#3-process-management)
  - [3.1 Three Ways to Kill a Running Demo](#31-three-ways-to-kill-a-running-demo)
  - [3.2 Ring Button Zones](#32-ring-button-zones)
  - [3.3 Short Press Behavior](#33-short-press-behavior--1-s)
  - [3.4 Interactive Shell During Demo](#34-interactive-shell-during-demo)
- [4. RAM Demos](#4-ram-demos)
  - [4.1 How It Works](#41-how-it-works)
  - [4.2 Writing a New Demo](#42-writing-a-new-demo)
  - [4.3 Available Demos](#43-available-demos)
- [5. DFU & Recovery](#5-dfu--recovery)
  - [5.1 Entering DFU Mode](#51-entering-dfu-mode)
  - [5.2 Flashing](#52-flashing)
  - [5.3 Hard Reset (Battery Lift)](#53-hard-reset-battery-lift)
- [6. Python Tools](#6-python-tools)
- [7. Known Limitations](#7-known-limitations)
  - [7.1 Motor Control](#71-motor-control)
  - [7.2 Sensors](#72-sensors)
  - [7.3 Connectivity](#73-connectivity)
  - [7.4 Reliability](#74-reliability)
- [8. Credits](#8-credits)

---

## 1. Quick Start

### 1.1 Build & Flash

```bash
# Build firmware (from project root)
cargo build --release

# Convert to raw binary (ALWAYS output to target/spike-rtic.bin — nowhere else!)
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/spike-rtic \
    target/spike-rtic.bin

# Enter DFU mode (see §5.1) then flash:
dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D target/spike-rtic.bin
```

Or use the script: `./helper-tools/flash.sh`

> **⚠ STALE BIN DANGER:** Never `objcopy` to `firmware.bin`, `lego-spike-prime.bin`,
> or any path outside `target/`.  Stale bins cause phantom faults and silent
> version mismatches.  The **only** valid firmware binary path is:
> `target/spike-rtic.bin`

### 1.2 Connect to the Shell

```bash
picocom /dev/ttyACM0
# or
screen /dev/ttyACM0 115200
# or
python3 helper-tools/spike_hub_controller.py shell  # interactive shell with upload support
```

Type `help` at the `spike>` prompt for a full command list.

### 1.3 Upload & Run a RAM Demo

```bash
# Build a demo
cd examples/hub-ram-demos
cargo build --release
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/dual-motors dual-motors.bin

# Upload and execute
python3 helper-tools/upload_demo.py examples/hub-ram-demos/target/spike-usr_bins/dual_motors.bin
# Or with the controller:
python3 helper-tools/spike_hub_controller.py upload examples/hub-ram-demos/target/spike-usr_bins/dual_motors.bin
```

For full details see the [hub-ram-demos README](examples/hub-ram-demos/README.md)
and [§4. RAM Demos](#4-ram-demos).

---

## 2. Shell Commands

The shell accepts **any standard line ending**: `\r` (CR), `\n` (LF),
or `\r\n` (CRLF).  Works with picocom, screen, echo, printf, Python
serial — whatever you have.  All output uses `\r\n` (CRLF).

### 2.1 General

| Command          | Description |
| ---------------- | ----------- |
| `help` / `?`     | Print full command list |
| `info`           | MCU info, stack high-water mark, previous-boot fault marker |
| `version`        | Firmware version, API version, and STM32 unique ID |
| `time`           | Human-readable uptime: `Hh MMm SSs` |
| `unixtime`       | Pseudo unix timestamp (fixed epoch + uptime in seconds) |
| `uptime`         | Raw tick count since boot (1 tick = 1 s) |
| `led <pattern>`  | Show pattern: `heart` `check` `cross` `arrow` `usb` `clear` `all` |
| `led <0-9>`      | Show digit on 5×5 matrix |
| `px <i> <b>`     | Set pixel `i` (0–24) to brightness `b` (0–100) |
| `status <color>` | Status LED: `red` `green` `blue` `white` `off` |
| `btn`            | Read button state (CENTER / LEFT / RIGHT) |
| `beep [f] [ms]`  | Play tone (default 1000 Hz 200 ms, non-blocking) |
| `load`           | Battery + system load overview |

### 2.2 Upload & Execute

| Command         | Description |
| --------------- | ----------- |
| `upload [size]` | Receive COBS binary upload into SRAM2 |
| `bininfo`       | Info on last uploaded binary (size, CRC, address) |
| `go [addr]`     | Execute RAM demo sandboxed (MPU + SVC, unprivileged) |
| `go! [addr]`    | Execute RAM demo privileged (direct fn pointers, no MPU) |
| `kill [-2\|-9\|-15]` | Signal running demo — `-15`=SIGTERM (default), `-2`=SIGINT, `-9`=SIGKILL |
| `raminfo`       | SRAM map & usage stats |
| `ramtest [arg]` | RAM test: `safe` `sram2` `addr` `all` |

For the full upload-execute workflow see [§4.1](#41-how-it-works) and
the [hub-ram-demos README](examples/hub-ram-demos/README.md).

### 2.3 Demon Debug Monitor

| Command                | Description |
| ---------------------- | ----------- |
| `md <addr> [n]`        | Hex dump `n` words (default 16) |
| `mw <addr> <val>`      | Write 32-bit word to address |
| `regs`                 | Cortex-M4 system registers (CPUID, VTOR, BASEPRI, SP…) |
| `clocks`               | RCC / PLL / bus clock config |
| `gpio <a-e>`           | GPIO port state (MODER, IDR, ODR, AFR) |
| `uid`                  | STM32 96-bit unique device ID |
| `flash`                | Flash size, option bytes, RDP level |
| `adc <ch>`             | Raw ADC channel 0–15 (12-bit) |
| `bat`                  | Battery voltage/current/NTC/USB charger |
| `crc <addr> <len>`     | CRC-32 of memory range |
| `fill <a> <n> <v>`     | Fill `n` words at addr with value |
| `trace [on\|off\|clear\|legend]` | SVC/sensor trace buffer |

### 2.4 Motor Control

| Command                | Description |
| ---------------------- | ----------- |
| `motor <a-f> <speed>`  | Drive motor (–100…+100, 0 = coast) |
| `motor <a-f> brake`    | Active brake (both H-bridge sides HIGH) |
| `motor <a-f> pos`      | Read motor position (works with motor_poll auto-sync) |
| `motor <a-f> pid <deg>` | PID position hold (requires `sensor <port>` first) |
| `motor <a-f> ramp`     | Ramp test: 5–100% forward, 1.5s each step |
| `motor <a-f> trial`    | Systematic 14-stage PWM trial |
| `motor <a-f> diag`     | Show PWM timer registers |
| `servo <a-f> hold [d]` | Pybricks-style observer+PID hold at d degrees |
| `servo <a-f> stop`     | Stop servo, brake motor |
| `pid`                  | Show PID gains |
| `pid <kp\|ki\|kd\|sc\|dz> <val>` | Set PID parameter live |
| `stune`                | Show servo PID gains |
| `stune <kp\|ki\|kd\|icm\|amax\|ptol> <val>` | Set servo parameter |
| `stune reset`          | Revert to motor-type defaults |

### 2.5 Sensor (LPF2 LUMP)

| Command                   | Description |
| ------------------------- | ----------- |
| `sensor <a-f>`            | Probe port for LPF2 sensor (LUMP sync) |
| `sensor`                  | Show current sensor data (type, mode, values) |
| `sensor mode <0-9>`       | Switch LUMP mode |
| `sensor diag <a-f>`       | Dump GPIO/UART register state for port |
| `sensor raw <a-f> [baud]` | Raw byte sniff at given baud rate |
| `sensor stop`             | Stop sensor polling |
| `ports`                   | Show all 6 port states |
| `light <r> <g> <b>`      | Color sensor LED (0–100 each) |

### 2.6 External Flash (SPI2)

| Command                       | Description |
| ----------------------------- | ----------- |
| `spiflash`                    | JEDEC ID + status registers |
| `spiflash read <addr> [n]`    | Hex dump `n` bytes (max 256) |
| `spiflash erase <addr>`       | Erase 4 KB sector |
| `spiflash write <addr> <v>`   | Write 4 bytes (big-endian u32) |
| `spiflash store <addr> [sz]`  | Copy RAM upload buffer → flash slot |
| `spiflash load <addr> <sz>`   | Copy flash → RAM upload buffer |
| `spiflash run <addr> <sz>`    | Load from flash + execute (sandboxed) |
| `spiflash dir`                | List stored binaries in flash slots |

The 32 MB W25Q256JV is fully accessible.  Addresses: `0x00000000`
to `0x01FFFFFF`.

### 2.7 DWT Watchpoints

Self-hosted hardware watchpoints using the Cortex-M4 Data Watchpoint
and Trace (DWT) unit.  4 comparators, no JTAG/SWD required.

| Command                          | Description |
| -------------------------------- | ----------- |
| `dwt status`                     | Show all 4 DWT comparators + hit counts |
| `dwt set <n> <addr> [r\|w\|rw\|pc]` | Arm watchpoint `n` (0–3) at address |
| `dwt clear [n]`                  | Disarm watchpoint `n`, or all if no arg |
| `dwt init`                       | Re-initialize DWT + DebugMonitor |

**Example — catch a write to 0x20001000:**

```
spike> dwt set 0 0x20001000 w
DWT#0 = 0x20001000 write
spike> mw 0x20001000 0xDEADBEEF
spike> dwt status
DWT: 4 comparators, 1 DebugMon hits
  #0: addr=0x20001000 mask=0 func=write match=1 hits=1 lastPC=0x00000000
  #1: addr=0x00000000 mask=0 func=off match=0 hits=0 lastPC=0x00000000
  ...
```

The DebugMonitor exception fires on watchpoint hits, recording the
hit count and attempting to capture the stacked PC.  Watchpoint
functions: `r` (data read), `w` (data write), `rw` (data read/write),
`pc` (instruction address match), `off` (disable).

### 2.8 GDB Remote Debug

Enter the **GDB Remote Serial Protocol** (RSP) stub — "Demon mode" —
for remote debugging over USB CDC serial.  Compatible with GDB, or
any tool speaking RSP.

| Command    | Description |
| ---------- | ----------- |
| `gdb`      | Enter GDB RSP mode |
| `demon`    | Enter GDB RSP mode (alias) |

**In GDB RSP mode**, the shell routes all bytes to the RSP parser.
Send `$D#44` (detach) or Ctrl-C to return to the shell.

**Supported RSP commands:**

| Packet | Action |
| ------ | ------ |
| `?`    | Stop reason |
| `g`    | Read all registers (r0–r12, SP, LR, PC, xPSR) |
| `G`    | Write all registers |
| `m addr,len` | Read memory (hex encoded) |
| `M addr,len:data` | Write memory |
| `Z2/3/4,addr,kind` | Set DWT hardware watchpoint (write/read/access) |
| `z2/3/4,addr,kind` | Remove DWT hardware watchpoint |
| `c`    | Continue execution |
| `s`    | Single step |
| `D`    | Detach (return to shell) |
| `k`    | Kill (return to shell) |
| `qSupported` | Capability query |

**Example with pyserial:**

```python
import serial
ser = serial.Serial('/dev/ttyACM0', timeout=1)
ser.write(b'gdb\r')           # enter RSP mode
ser.write(b'$?#3f')           # query stop reason
print(ser.read(100))           # → +$S00#b3
ser.write(b'$m20000000,10#--') # read 16 bytes at 0x20000000
ser.write(b'$D#44')            # detach → back to shell
```

### 2.9 System & Misc

| Command           | Description |
| ----------------- | ----------- |
| `ps`              | RTIC task tree with active/idle state |
| `ls`              | ROM/RAM/SPI-flash memory map with usage stats |
| `test_all`        | Choreographed hardware ballet (LEDs, motors, sensors, sound) |
| `reconnect-ser`   | Drop USB serial for 2 s (terminal reconnect) |
| `dfu`             | Power off with DFU re-entry instructions |
| `off`             | Clean power off (release PA13) |
| `sleep`           | Low-power stop mode |
| `reset`           | Soft reset (SYSRESETREQ) |
| `bye` / `quit`    | End session (= reconnect-ser) |
| `imu [init]`      | Init LSM6DS3TR-C IMU, show WHO_AM_I |
| `imu read`        | Read accelerometer + gyro XYZ |
| `rtty <text>`     | Send Baudot RTTY (45.45 baud FSK audio) |

---

## 3. Process Management

The firmware provides **Unix-style process management** — user-apps
never need exit-on-button logic.  The firmware controls task lifecycle;
user-apps are just functions that run until they return (or the firmware
kills them).

### 3.1 Three Ways to Kill a Running Demo

| Method | Signal | Mechanism |
| ------ | ------ | --------- |
| **`kill` command** | SIGTERM (default), SIGINT, or SIGKILL | Type `kill`, `kill -9`, etc. at `spike>` |
| **Ctrl-C** | SIGTERM | Send `0x03` via USB serial |
| **Ring button** | SIGKILL | Hold center 1–6 s, release |
| **Right button** | SIGKILL + relaunch | Kill running demo, then launch from SPI flash |

The `kill` shell command supports Unix-style signals:

| Command | Signal | Behavior |
| ------- | ------ | -------- |
| `kill` or `kill -15` | SIGTERM | Cooperative: sets ABORT flag, demo exits at next API call |
| `kill -2` | SIGINT | Same as SIGTERM (polite interrupt) |
| `kill -9` | SIGKILL | Immediate: `force_kill_sandbox` + brake motors + stop sound |

SIGTERM/SIGINT: every MonitorApi callback checks the ABORT flag and
longjmps back to `run_demo`.  A demo doing `loop { delay_ms(100); }`
exits within 100 ms.

SIGKILL: hijacks the Cortex-M exception return to redirect execution
to the abort landing, killing even tight `loop {}` demos that never
call any API.

### 3.2 Ring Button Zones

The center ring button uses **hold duration** to select an action.
While held, the LED pulses in the zone's color with a tick sound at
each second boundary.  Action fires **on release**.

| Duration | While Held | On Release | Audio Feedback |
| -------- | ---------- | ---------- | -------------- |
| < 1 s | ⚪ white pulse | **Pause / Resume** (running) or **Re-run** last demo (idle) | Pause: 880→660 Hz double beep ⬇. Resume: 1760 Hz chirp ⬆ |
| 1 – 3 s | 🟠 orange pulse | **SIGKILL** — hard-kill running demo | 660 Hz tick per second |
| 3 – 6 s | 🔴 red pulse | **SIGKILL** — hard-kill running demo | 440 Hz tick per second  |
| 6 – 8 s | 🟣 purple pulse | **Power off** — deep sleep | 220 Hz tick, then 440→220→110 Hz descending on release |
| ≥ 8 s | *(auto-triggers)* | **Forced shutdown** — safety net | 880→440→220 Hz, then power cut |

**Release feedback — LED + sound:**

| Action | LED Color | Sound Pattern |
| ------ | --------- | ------------- |
| **Pause** | 🟡 yellow | 880→660 Hz double beep (descending) |
| **Resume** | 🔵 cyan | 1760 Hz single chirp |
| **Re-run** (short press, idle) | 🟢 green | 660→1320 Hz ascending chirp |
| **Kill** (1–6 s release) | 🔴 red flash | 880→440→220 Hz triple descend |
| **Nothing to kill** (1–6 s, idle) | — | 440 Hz short beep |
| **Power off** (6+ s release) | — | 440→220→110 Hz deep descend |

### 3.3 Short Press Behavior (< 1 s)

The short press action depends on whether a demo is running:

```
Demo running?
├─ YES → toggle pause/resume
│        🟡 PAUSE:  yellow LED + 880→660 Hz ⬇
│        🔵 RESUME: cyan LED + 1760 Hz ⬆
└─ NO  → last_run_addr set?
         ├─ YES → re-run last demo (fresh, sandboxed)
         │        🟢 green flash + 660→1320 Hz ⬆
         └─ NO  → toggle global pause
                  (same audio as pause/resume above)
```

**Re-run workflow:** Upload a demo → `go` → demo runs → kill it (1-3 s
hold) → short press → demo re-launches from clean state.  No need to
re-upload or type `go` again.

### 3.4 Interactive Shell During Demo

The shell remains **fully interactive** while a demo runs — like a
Unix background job.  You can type `uptime`, `ps`, `sensor`, `kill`,
or any other command while the demo is executing.  `ps` shows the
demo's status:

```
spike> ps
  run_demo  ACTIVE    ← demo running
  shell_tick  -
  heartbeat  ACTIVE
spike> kill
Abort requested
=> 3735879681 (0xDEAD0001)
spike>
```

---

## 4. RAM Demos

### 4.1 How It Works

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
monitor's I/O, LEDs, motors, sensors, and speaker through the
`MonitorApi` callback table.

### 4.2 Writing a New Demo

1. Add a `[[bin]]` entry in `hub-ram-demos/Cargo.toml`.
2. Create `examples/my_demo.rs` with `#![no_std]`, `#![no_main]`.
3. Use `spike_hub_api::MonitorApi` for all hardware access.
4. Export `pub extern "C" fn _start(api: *const MonitorApi) -> u32`.
5. Build → objcopy → upload → `go`.

**Important:** Every demo must check `api.version >= N` before using
fields added in version N.  This prevents silent crashes from
firmware/demo version mismatches.

See the [hub-ram-demos README](examples/hub-ram-demos/README.md) for
full build instructions and the complete demo template.

### 4.3 Available Demos

| Demo             | Source                        | API  | What It Does |
| ---------------- | ----------------------------- | ---- | ------------ |
| `spike-demo`     | `src/main.rs`                 | v2+  | Motor ramp/coast/reverse/brake on port B |
| `dual-motors`    | `examples/dual_motors.rs`     | v2+  | Choreographed A+B motor dance with LEDs |
| `motor-buttons`  | `examples/motor_buttons.rs`   | v2+  | Button-controlled dual motors (interactive) |
| `color-chaos`    | `examples/color_chaos.rs`     | v4+  | RGBI sensor → derivative-driven motors + RTTY FSK audio |
| `color-seek`     | `examples/color_seek.rs`      | v3+  | Fast RGBI color classification, motor seek |
| `color-seeker`   | `examples/color_seeker.rs`    | v7+  | Rotational color scanner: motor_goto steps + color classify |
| `sensor-dump`    | `examples/sensor_dump.rs`     | v3+  | Raw RGBI hex dump + ring buffer diagnostics |
| `motor-goto`     | `examples/motor_goto.rs`      | v7+  | Closed-loop position control: goto 90°/0°/360°/0° |
| `motor-sensor-test` | `examples/motor_sensor_test.rs` | v7+ | Motor + sensor integration test |
| `pwm-diag`       | `examples/pwm_diag.rs`        | v2+  | PWM diagnostics on motor ports |
| `reg-dump`       | `examples/reg_dump.rs`        | v2+  | Hardware register inspection |

All demos are in [`examples/hub-ram-demos/`](examples/hub-ram-demos/).

---

## 5. DFU & Recovery

### 5.1 Entering DFU Mode

Three ways to enter DFU:

1. **Button hold:** hold **left button** while firmware is running
2. **Power-on combo:** hold **center + left** while inserting USB / pressing center

The hub re-enumerates as `0694:0011` (LEGO DFU device).
Verify with: `lsusb | grep 0694`

### 5.2 Flashing

```bash
dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D target/spike-rtic.bin
```

> **Important:** always use `-d 0694:0011` (LEGO VID:PID), not the
> generic ST DFU address.  The firmware starts at `0x08008000` (after
> the 32 KB LEGO bootloader).  Always flash `target/spike-rtic.bin`
> — never a stale `firmware.bin` or `lego-spike-prime.bin`.

### 5.3 Hard Reset (Battery Lift)

If the hub is completely stuck (no USB device, buttons unresponsive):

1. **Lift the battery out** for one second
2. Re-insert the battery
3. Hold **center** to boot normally, or **center + left** for DFU

> The SPIKE Prime hub uses a soft-latch power circuit: PA13 holds a
> MOSFET gate to keep power on after the center button is released.
> If the firmware hangs before asserting PA13 (or hangs with PA13
> held), removing the battery is the **only** way to cut power.

---

## 6. Python Tools

All scripts live in **`helper-tools/`**.  Run them from the project root:

| Script                                | Purpose |
| ------------------------------------- | ------- |
| `helper-tools/spike_hub_controller.py`| Unified controller: TUI, shell, flash, upload, test, diag |
| `helper-tools/upload_demo.py`         | Lightweight demo upload + run (for automation) |
| `helper-tools/test_hub_mon.py`        | Automated 65-test suite covering all shell commands |
| `helper-tools/test_sandbox.py`        | MPU sandbox tests with hand-built ARM Thumb binaries |
| `helper-tools/color_ballet.py`        | Motor/sensor demo — search and elegantly alternate colors |
| `helper-tools/flash_store_run.py`     | Upload binary to RAM, store to SPI flash, optionally run |
| `helper-tools/flash.sh`              | Build firmware + DFU flash in one step |
| `helper-tools/attention.sh`           | Ring terminal bell until user presses Enter |

Connect and upload in one step:

```bash
python3 helper-tools/spike_hub_controller.py upload examples/hub-ram-demos/target/spike-usr_bins/dual_motors.bin
# Or for demo workflow (upload + probe sensor + go + stream output):
python3 helper-tools/upload_demo.py examples/hub-ram-demos/target/spike-usr_bins/dual_motors.bin
```

---

## 7. Known Limitations

This section exists because honest documentation prevents wasted time.
If something here matters to your use case, you know upfront.

### 7.1 Motor Control

- **No real PID loop.** `motor_goto` uses a simple proportional
  controller with deadband.  It reaches ±9° accuracy, which is
  functional but nowhere near Pybricks' 100 Hz PID with integral
  accumulation and feedforward.
- **Stiction below ~25% PWM is real.** Small motors won't move at
  low duty cycles.  Open-loop PWM cannot fix this — only closed-loop
  control with integral windup can.
- **No speed control.** Only position and raw PWM.  No velocity
  profiling, no acceleration ramps, no S-curves.

### 7.2 Sensors

- **One sensor at a time.** The LUMP driver currently handles a
  single port (typically F).  All 6 UART ports have ring buffers,
  but only one runs the full LUMP sync + keepalive state machine.
- **LUMP keepalive is fragile.** If anything blocks the keepalive
  path for >250 ms, the sensor goes silent and needs a full re-sync
  (500–1000 ms).
- **No gyro / IMU driver.** The hub has an LSM6DS3TR-C (I2C) but
  the driver is minimal.  No orientation data yet.

### 7.3 Connectivity

- **No Bluetooth.** The hub has a TI CC2564C BLE chip.  We don't
  touch it.  USB serial only.
- **USB CDC can be finicky.** CDC enumeration sometimes fails on
  reconnect.  The `host_synced` mechanism prevents banner garbling,
  but if the host opens/closes the port rapidly, the hub may need
  a `reconnect-ser` or a power cycle.

### 7.4 Reliability

- **Hardware watchdog (~5 s).** The IWDG runs with a ~5 s timeout,
  fed by the heartbeat task.  If the firmware locks up, the watchdog
  resets the hub automatically.
- **24 `unwrap()` calls in firmware.** Each is a potential panic
  (= instant reboot to bootloader).  They're in contexts believed
  safe, but not formally proven.
- **MemManage fault handler with two-branch recovery.**  Sandbox
  faults redirect to the abort landing for clean recovery.  Firmware
  stack overflow faults kill motor PWM, store a fault marker in SRAM2,
  and trigger a system reset.  The `info` command reports fault details.

---

## 8. Credits

- Pin mappings from [pybricks-micropython](https://github.com/pybricks/pybricks-micropython)
- [LUMP protocol reference](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/port_lump.c) — the gold-standard implementation
- RTIC people, and all Rust giants out there
- Claude Opus 4.6 (March 2026) — MCP-assisted firmware development

---

*See also:* [REFERENCE_MANUAL.md](REFERENCE_MANUAL.md) for hardware specs,
memory maps, architecture details, and the MonitorApi callback table.
