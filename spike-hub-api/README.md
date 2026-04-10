# spike-hub-api — Shared MonitorApi for SPIKE Prime Hub

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md)

---

Defines the `MonitorApi` callback table — the contract between the
RTIC monitor firmware and uploaded RAM demos.

Both sides depend on this crate so the `#[repr(C)]` struct layout is
guaranteed identical across independent compilations.

<!-- TOC -->

## Contents

- [1. Overview](#1-overview)
- [2. API Fields](#2-api-fields)
- [3. Usage](#3-usage)
  - [3.1 Demo Side](#31-demo-side)
  - [3.2 Firmware Side](#32-firmware-side)
  - [3.3 Trace Recording](#33-trace-recording)
- [4. Version History](#4-version-history)
- [5. Constants](#5-constants)

<!-- /TOC -->

---

## 1. Overview

RAM demos are **separately compiled** Rust binaries uploaded to SRAM2
at runtime.  Two independent `rustc` invocations cannot share Rust's
internal ABI — it's unstable and changes between compiler versions.
The **C ABI** (`extern "C"`) is the standard stable calling convention
on ARM, used by ARM Demon and Angel debug monitors since the 1980s.

**No C compiler or libc is involved.  Both sides are pure Rust.**

See the [main firmware README](../../README.md#53-monitorapi--callback-table)
for the architecture overview, and the
[hub-ram-demos README](../../examples/hub-ram-demos/README.md) for
the full build-upload-execute workflow.

---

## 2. API Fields

Current version: **`API_VERSION = 12`** (26 fields, 104 bytes on 32-bit ARM).

| Offset | Field          | Type / Signature                    | Since | Description |
| ------ | -------------- | ----------------------------------- | ----- | ----------- |
| 0x00   | `version`      | `u32`                               | v1 | API version — check before using fields |
| 0x04   | `context`      | `*mut u8`                           | v1 | Opaque — pass to `write_fn` |
| 0x08   | `write_fn`     | `fn(ctx, ptr, len)`                 | v1 | Print bytes to USB CDC serial |
| 0x0C   | `delay_ms`     | `fn(ms: u32)`                       | v1 | Blocking delay with sensor keepalive |
| 0x10   | `set_pixel`    | `fn(index: u32, brightness: u32)`   | v1 | LED matrix pixel (0–24, 0–100) |
| 0x14   | `update_leds`  | `fn()`                              | v1 | Push pixel buffer to TLC5955 |
| 0x18   | `read_adc`     | `fn(channel: u32) → u32`           | v1 | Raw 12-bit ADC value |
| 0x1C   | `read_buttons` | `fn() → u8`                        | v1 | Button bitmask (CENTER\|LEFT\|RIGHT) |
| 0x20   | `motor_set`    | `fn(port: u32, speed: i32)`         | v2 | Motor speed (–100…+100) |
| 0x24   | `motor_brake`  | `fn(port: u32)`                     | v2 | Active brake |
| 0x28   | `sensor_read`  | `fn(buf: *mut u8, len: u32) → u32` | v3 | Read LUMP data from ring buffer |
| 0x2C   | `sensor_mode`  | `fn(mode: u32)`                     | v3 | Switch LUMP mode (0–9) |
| 0x30   | `sound_play`   | `fn(freq_hz: u32)`                  | v4 | Start tone (Hz) via TIM6+DAC |
| 0x34   | `sound_stop`   | `fn()`                              | v4 | Silence speaker |
| 0x38   | `trace_record` | `fn(tag: u8, val: u8, arg: u16)`    | v5 | Record RAM trace entry |
| 0x3C   | `rtty_say`     | `fn(ptr: *const u8, len: u32)`      | v6 | RTTY FSK broadcast |
| 0x40   | `rtty_busy`    | `fn() → u32`                       | v6 | RTTY still transmitting? |
| 0x44   | `motor_position` | `fn() → i32`                     | v7 | Motor A cumulative degrees |
| 0x48   | `motor_goto`   | `fn(port: u32, degrees: i32) → i32` | v7 | Closed-loop goto position |
| 0x4C   | `port_read`    | `fn(port: u32, buf: *mut u8, len: u32) → u32` | v8 | Read any port's LUMP ring buffer |
| 0x50   | `sensor_light` | `fn(r: u8, g: u8, b: u8)`          | v9 | Set color sensor LED (RGB) |
| 0x54   | `imu_init`     | `fn() → u32`                       | v10 | Initialize LSM6DS3TR-C IMU |
| 0x58   | `imu_read`     | `fn(buf: *mut u8, len: u32) → u32` | v10 | Read IMU data (accel + gyro) |
| 0x5C   | `set_hub_led`  | `fn(r: u8, g: u8, b: u8)`          | v10 | Set status LED color (RGB) |
| 0x60   | `wait_event`   | `fn(mask: u32, timeout_ms: u32) → u32` | v11 | Block until event or timeout |
| 0x64   | `read_input`   | `fn(buf: *mut u8, len: u32) → u32` | v12 | Read host input from `send` command |
---

## 3. Usage

### 3.1 Demo Side

```rust
use spike_hub_api::{MonitorApi, BTN_CENTER};

#[no_mangle]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };
    if api.version < 5 { return 1; }

    api.print(b"Hello from RAM!\r\n");
    (api.set_pixel)(12, 80);
    (api.update_leds)();
    (api.delay_ms)(500);

    0  // exit code
}
```

### 3.2 Firmware Side

```rust
use spike_hub_api::{MonitorApi, API_VERSION};

let api = MonitorApi {
    version: API_VERSION,
    context: shell_ptr as *mut u8,
    write_fn: my_write,
    // ... all other fields ...
};
let result = demo_entry(&api);
```

### 3.3 Trace Recording

Use `trace_record` for lightweight instrumentation (8 bytes per entry,
atomic, ~10–20 CPU cycles):

```rust
// tag=0x80 (TAG_USER), val=loop_count, arg=sensor_reading
(api.trace_record)(0x80, count as u8, reading as u16);
```

Dump traces with `trace` in the shell.  See the
[main README §5.4](../../README.md#54-trace-buffer) for details.

> **Process management:** the firmware can kill a running demo at any
> time via `kill [-2|-9|-15]`, Ctrl-C, or the ring button.  SIGTERM/SIGINT
> set an ABORT flag checked by every MonitorApi callback.  SIGKILL (`kill -9`,
> ring button hold 1–6 s) uses `force_kill_sandbox` to hijack the exception
> return — kills even tight `loop {}` demos.  Demos do not need exit-on-button
> logic — the firmware handles lifecycle.
> See the [main README §5.5](../../README.md#55-process-management).

---

## 4. Version History

| Version | Changes |
| ------- | ------- |
| 1       | Initial: write_fn, delay_ms, set_pixel, update_leds, read_adc, read_buttons |
| 2       | Added motor_set, motor_brake |
| 3       | Added sensor_read, sensor_mode |
| 4       | Added sound_play, sound_stop |
| 5       | Added trace_record |
| 6       | Added rtty_say, rtty_busy |
| 7       | Added motor_position, motor_goto |
| 8       | Added port_read (read any port's LUMP ring buffer) |
| 9       | Added sensor_light (set color sensor LED RGB) |
| 10      | Added imu_init, imu_read, set_hub_led (IMU + status LED) |
| 11      | Added wait_event — event-driven blocking with EVT_SENSOR/BUTTON/MOTOR/TIMEOUT |
| 12      | Added read_input, EVT_INPUT — bidirectional host→demo text channel via `send` |

---

## 5. Constants

| Constant          | Value        | Description |
| ----------------- | ------------ | ----------- |
| `API_VERSION`     | 12           | Current struct layout version |
| `BTN_CENTER`      | 0x01         | Center button flag |
| `BTN_LEFT`        | 0x02         | Left button flag |
| `BTN_RIGHT`       | 0x04         | Right button flag |
| `UPLOAD_BUF_ADDR` | 0x2004_0000  | SRAM2 upload buffer base |
| `UPLOAD_BUF_SIZE` | 65536        | Upload buffer size (64 KB) |
| `EVT_SENSOR`      | `1 << 0`     | New sensor data available |
| `EVT_BUTTON`      | `1 << 1`     | Button state changed |
| `EVT_MOTOR`       | `1 << 2`     | Motor position changed |
| `EVT_TIMEOUT`     | `1 << 3`     | Timeout expired (no event fired) |
| `EVT_INPUT`       | `1 << 4`     | Input data from host `send` command |
