//! Shared API definitions for LEGO SPIKE Prime hub firmware and RAM demos.
//!
//! This crate defines the **MonitorApi** callback table — the contract
//! between the monitor firmware and uploaded RAM demos.  Both sides
//! depend on this crate so the `#[repr(C)]` struct layout is guaranteed
//! identical.
//!
//! # Why `#[repr(C)]`?
//!
//! The demo is a **separately compiled** Rust binary uploaded to SRAM2
//! at runtime.  Two independent `rustc` invocations cannot share Rust's
//! internal ABI (it's unstable and changes between compiler versions).
//! The C ABI is the standard stable calling convention on ARM — the same
//! approach used by ARM Angel (1995), U-Boot, and every ROM monitor.
//!
//! **No C compiler or libc is involved.  Both sides are pure Rust.**
//!
//! # Architecture (inspired by [pybricks](https://github.com/pybricks/pybricks-micropython))
//!
//! Pybricks uses a layered vtable architecture:
//!   - `pbdrv` — hardware drivers (GPIO, PWM, SPI, ADC)
//!   - `pbio`  — hardware-agnostic I/O (servo loops, light animations)
//!   - `pbsys` — system services (status LED, buttons, battery, storage)
//!
//! We use a **single flat callback table** (`MonitorApi`) because we have
//! one boundary (firmware ↔ demo) rather than multiple internal layers.
//! As the API grows, related calls can be grouped into sub-structs while
//! maintaining `#[repr(C)]` compatibility.
//!
//! # Usage
//!
//! **Firmware side** (populates the struct):
//! ```rust,ignore
//! use spike_hub_api::{MonitorApi, API_VERSION};
//!
//! let api = MonitorApi {
//!     version: API_VERSION,
//!     context: shell_ptr as *mut u8,
//!     write_fn: my_write_callback,
//!     // ...
//! };
//! let result = demo_entry(&api);
//! ```
//!
//! **Demo side** (uses the struct):
//! ```rust,ignore
//! use spike_hub_api::MonitorApi;
//!
//! #[no_mangle]
//! pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
//!     let api = unsafe { &*api };
//!     api.print(b"Hello from RAM!\r\n");
//!     0
//! }
//! ```

#![no_std]

// ── Version ────────────────────────────────────────────────────

/// Current API version.  Bumped when the struct layout changes.
pub const API_VERSION: u32 = 12;

// ── Button flags ───────────────────────────────────────────────

/// Center button pressed.
pub const BTN_CENTER: u8 = 0x01;
/// Left button pressed.
pub const BTN_LEFT: u8 = 0x02;
/// Right button pressed.
pub const BTN_RIGHT: u8 = 0x04;

// ── Upload buffer ──────────────────────────────────────────────

// ── Event flags for wait_event() ────────────────────────────────

/// New sensor data available on the active LUMP port.
pub const EVT_SENSOR: u32 = 1 << 0;
/// Button state changed (any of center/left/right).
pub const EVT_BUTTON: u32 = 1 << 1;
/// Motor position changed (any port with active poll).
pub const EVT_MOTOR: u32 = 1 << 2;
/// Timeout expired (no other event fired within the deadline).
pub const EVT_TIMEOUT: u32 = 1 << 3;

/// Input data available from `send` shell command.
pub const EVT_INPUT: u32 = 1 << 4;

// ── Privileged demo marker ─────────────────────────────────────

/// Magic word placed in `.demo_header` section by demos that require
/// privileged mode (`go!`).  Firmware checks the first word of the
/// binary; if it matches, `go` (sandboxed) is refused with an error.
///
/// Value: ASCII "PRIV" (0x56495250 little-endian).
pub const PRIV_MAGIC: u32 = 0x5652_4950; // "PRIV" as [u8; 4] LE

// ── Upload buffer ──────────────────────────────────────────────

/// Base address of the upload buffer in SRAM2.
/// Demos are linked to this address.
pub const UPLOAD_BUF_ADDR: u32 = 0x2004_0000;

/// Size of the upload buffer (64 KB).
pub const UPLOAD_BUF_SIZE: usize = 64 * 1024;

// ── MonitorApi — callback table for RAM demos ─────────────

/// C-ABI callback table passed to RAM demos executed via `go`.
///
/// The firmware populates this struct on the stack and passes a pointer
/// as the sole argument to the demo entry point:
///
/// ```text
/// extern "C" fn _start(api: *const MonitorApi) -> u32
/// ```
///
/// # Fields
///
/// | Offset | Field          | Signature                            |
/// |--------|----------------|--------------------------------------|
/// | 0x00   | `version`      | `u32` — API version (currently 1)    |
/// | 0x04   | `context`      | `*mut u8` — opaque, pass to write_fn |
/// | 0x08   | `write_fn`     | `fn(ctx, ptr, len)`                  |
/// | 0x0C   | `delay_ms`     | `fn(ms)`                             |
/// | 0x10   | `set_pixel`    | `fn(index, brightness)`              |
/// | 0x14   | `update_leds`  | `fn()`                               |
/// | 0x18   | `read_adc`     | `fn(channel) -> u32`                 |
/// | 0x1C   | `read_buttons` | `fn() -> u8`                         |
/// | 0x20   | `motor_set`    | `fn(port, speed)`                    |
/// | 0x24   | `motor_brake`  | `fn(port)`                           |
/// | 0x28   | `sensor_read`  | `fn(buf, len) -> u32`                |
/// | 0x2C   | `sensor_mode`  | `fn(mode)`                           |
/// | 0x30   | `sound_play`   | `fn(freq_hz)`                        |
/// | 0x34   | `sound_stop`   | `fn()`                               |
/// | 0x38   | `trace_record` | `fn(tag, val, arg)`                  |
/// | 0x3C   | `rtty_say`     | `fn(data, len)`                      |
/// | 0x40   | `rtty_busy`    | `fn() -> u32`                        |
/// | 0x44   | `motor_position` | `fn() -> i32`                      |
/// | 0x48   | `motor_goto`   | `fn(port, degrees) -> i32`           |
/// | 0x4C   | `port_read`    | `fn(port, buf, len) -> u32`          |
/// | 0x50   | `sensor_light` | `fn(r, g, b)`                        |
/// | 0x54   | `imu_init`     | `fn() -> u32`                        |
/// | 0x58   | `imu_read`     | `fn(buf, len) -> u32`                |
/// | 0x5C   | `set_hub_led`  | `fn(r, g, b)`                        |
/// | 0x60   | `wait_event`   | `fn(mask, timeout_ms) -> u32`        |
/// | 0x64   | `read_input`   | `fn(buf, len) -> u32`                |
///
/// Size: 104 bytes on 32-bit ARM.
#[repr(C)]
pub struct MonitorApi {
    /// API version — check this before using any fields.
    /// Current version: [`API_VERSION`] (1).
    pub version: u32,

    /// Opaque context pointer — must be passed as the first argument
    /// to `write_fn`.  Do not dereference or interpret.
    pub context: *mut u8,

    /// Write bytes to the USB CDC serial output.
    ///
    /// - `ctx`: pass `self.context`
    /// - `data`: pointer to byte buffer
    /// - `len`: number of bytes to write
    ///
    /// Output is buffered (2 KB) and drains asynchronously after the
    /// demo returns.
    pub write_fn: extern "C" fn(ctx: *mut u8, data: *const u8, len: u32),

    /// Busy-wait delay in milliseconds (calibrated for 96 MHz).
    pub delay_ms: extern "C" fn(ms: u32),

    /// Set LED matrix pixel brightness.
    ///
    /// - `index`: pixel 0–24 (5×5 matrix, row-major)
    /// - `brightness`: 0–100
    pub set_pixel: extern "C" fn(index: u32, brightness: u32),

    /// Push current pixel buffer to the TLC5955 LED driver.
    /// Call after `set_pixel` to make changes visible.
    pub update_leds: extern "C" fn(),

    /// Read a raw ADC channel (0–15).  Returns 12-bit value (0–4095).
    ///
    /// Useful channels on SPIKE Prime:
    /// - ch1: left/right/BT button ladder
    /// - ch3: USB charger current
    /// - ch8: battery NTC temperature
    /// - ch10: battery current
    /// - ch11: battery voltage
    /// - ch14: center button
    pub read_adc: extern "C" fn(channel: u32) -> u32,

    /// Read button state.  Returns bitfield:
    /// - bit 0 ([`BTN_CENTER`]): center button
    /// - bit 1 ([`BTN_LEFT`]): left button
    /// - bit 2 ([`BTN_RIGHT`]): right button
    pub read_buttons: extern "C" fn() -> u8,

    /// Set motor speed on a port.
    ///
    /// - `port`: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F
    /// - `speed`: -100 to +100 (positive=forward, negative=reverse, 0=coast)
    pub motor_set: extern "C" fn(port: u32, speed: i32),

    /// Brake motor on a port (both H-bridge sides driven high).
    ///
    /// - `port`: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F
    pub motor_brake: extern "C" fn(port: u32),

    /// Read latest sensor data from the active LUMP port.
    ///
    /// The firmware's UART ISR continues filling the ring buffer
    /// even while a demo runs.  This callback drains the ring
    /// buffer, sends keepalive NACKs, and copies the latest
    /// data payload into `buf`.
    ///
    /// - `buf`: pointer to caller's buffer (at least 32 bytes)
    /// - `len`: size of caller's buffer
    /// - Returns: number of valid bytes written (0 if no data)
    ///
    /// For mode 5 (RGBI): returns 8 bytes (4 x i16 LE: R, G, B, I).
    pub sensor_read: extern "C" fn(buf: *mut u8, len: u32) -> u32,

    /// Switch the active sensor to a different LUMP mode.
    ///
    /// - `mode`: LUMP mode number (0-9)
    pub sensor_mode: extern "C" fn(mode: u32),

    /// Start playing a tone at the given frequency (Hz).
    ///
    /// Uses TIM6 + DAC1 triangle-wave generator — the tone plays
    /// entirely in hardware with zero CPU cost.  Call `sound_stop`
    /// to silence.  Calling `sound_play` with a new frequency
    /// switches instantly (glitch-free frequency change).
    pub sound_play: extern "C" fn(freq_hz: u32),

    /// Stop the speaker (silence).
    pub sound_stop: extern "C" fn(),

    /// Record a trace entry into the firmware's RAM trace buffer.
    ///
    /// Entries are 8 bytes (tag, val, arg, tick) stored in a 256-entry
    /// ring buffer in SRAM1.  Dump with `trace` shell command.
    ///
    /// Use `tag = 0x80` (TAG_USER) for demo-specific traces.
    /// - `tag`: what happened (0x80 = user trace)
    /// - `val`: secondary value (u8)
    /// - `arg`: primary argument (u16)
    ///
    /// No-op if tracing is disabled (`trace off` in shell).
    /// Safe to call at any frequency — atomic writes, no locks.
    pub trace_record: extern "C" fn(tag: u8, val: u8, arg: u16),

    /// Queue a message for RTTY (Baudot FSK) transmission.
    ///
    /// The message is copied into a firmware buffer and transmitted
    /// asynchronously by the `rtty_tx` RTIC task at priority 2 —
    /// it preempts the demo and plays FSK tones via hardware DAC
    /// with zero CPU busy-wait.  Returns immediately.
    ///
    /// Check `rtty_busy` before calling to avoid overwriting an
    /// in-progress transmission.
    ///
    /// - `data`: pointer to ASCII message (A-Z, 0-9, space)
    /// - `len`: number of bytes (max 80)
    pub rtty_say: extern "C" fn(data: *const u8, len: u32),

    /// Check if an RTTY transmission is in progress.
    ///
    /// Returns 1 if the `rtty_tx` task is currently transmitting,
    /// 0 if idle and ready for a new message.
    pub rtty_busy: extern "C" fn() -> u32,

    /// Read motor position in cumulative degrees (i32).
    ///
    /// Returns the latest position from the motor port's LUMP data
    /// (mode 2 = POS).  The motor port is set up by the firmware at
    /// boot (default: port A).  The ring buffer is drained and a
    /// keepalive is sent as a side effect.
    ///
    /// Returns 0 if no motor is connected or no data received yet.
    pub motor_position: extern "C" fn() -> i32,

    /// Drive motor to target position in degrees.  **Blocking.**
    ///
    /// Uses a two-phase ramp+nudge controller:
    ///   1. Approach: ramp from max speed down to stiction compensation
    ///   2. Recovery: nudge pulses to converge on target
    ///
    /// - `port`: 0=A, 1=B (must match motor_poll's active port)
    /// - `degrees`: target cumulative position in degrees
    /// - Returns: final position error in degrees (target - actual)
    ///
    /// Typical accuracy: ±2–5° with Technic M Angular motor.
    /// Maintains sensor keepalive during the blocking operation.
    pub motor_goto: extern "C" fn(port: u32, degrees: i32) -> i32,

    /// Read latest sensor data from a specific port (0=A .. 5=F).
    ///
    /// Reads from PORT_STATES[] — works for any port that has an
    /// active poll task (sensor_poll, sensor_poll2, motor_poll, etc).
    /// Does NOT drain the ring buffer or send keepalive — the port's
    /// poll task handles that independently.
    ///
    /// - `port`: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F
    /// - `buf`: pointer to caller's buffer (at least 32 bytes)
    /// - `len`: size of caller's buffer
    /// - Returns: number of valid bytes written (0 if no data)
    pub port_read: extern "C" fn(port: u32, buf: *mut u8, len: u32) -> u32,

    /// Set color sensor LED brightness (LUMP mode 3 write).
    ///
    /// Sends a LUMP DATA command to the active sensor port.
    /// The color sensor has 3 LEDs controlled by (r, g, b) values 0-100.
    /// - `r`: red LED brightness (0-100)
    /// - `g`: green LED brightness (0-100)
    /// - `b`: blue LED brightness (0-100)
    pub sensor_light: extern "C" fn(r: u32, g: u32, b: u32),

    /// Initialize the internal 6-DOF IMU (LSM6DS3TR-C).
    ///
    /// Configures I2C2 bit-bang, checks WHO_AM_I, sets accel to 104Hz ±2g
    /// and gyro to 104Hz ±250dps.  Returns WHO_AM_I value (0x6A) on
    /// success, 0 on failure.  Safe to call multiple times (no-op if
    /// already initialized).
    pub imu_init: extern "C" fn() -> u32,

    /// Read IMU data: 12 bytes = accel XYZ (3×i16 LE) + gyro XYZ (3×i16 LE).
    ///
    /// - `buf`: pointer to caller's buffer (at least 12 bytes)
    /// - `len`: size of caller's buffer
    /// - Returns: number of bytes written (12 on success, 0 on failure)
    ///
    /// Acceleration: ±2g range, 0.061 mg/LSB.
    /// Angular rate: ±250 dps range, 8.75 mdps/LSB.
    pub imu_read: extern "C" fn(buf: *mut u8, len: u32) -> u32,

    /// Set RGB color of the hub battery LED (tiny LED near USB port).
    ///
    /// Controls BATTERY_LED RGB via TLC5955 channels (2,1,0).
    /// Values 0-100 mapped to 16-bit PWM internally.
    /// - `r`: red brightness 0-100
    /// - `g`: green brightness 0-100
    /// - `b`: blue brightness 0-100
    pub set_hub_led: extern "C" fn(r: u32, g: u32, b: u32),

    /// Block until an event in `mask` fires, or `timeout_ms` expires.
    ///
    /// Returns a bitmask of which events fired ([`EVT_SENSOR`],
    /// [`EVT_BUTTON`], [`EVT_MOTOR`]).  Returns [`EVT_TIMEOUT`] if no
    /// event fired before the deadline.  Maintains sensor keepalive and
    /// respects abort/pause internally — the caller never polls.
    ///
    /// - `mask`: OR of `EVT_*` constants — which events to listen for
    /// - `timeout_ms`: maximum wait time (0 = check once, no wait)
    ///
    /// # Example (demo side)
    /// ```rust,ignore
    /// loop {
    ///     let evt = (api.wait_event)(EVT_SENSOR | EVT_BUTTON, 500);
    ///     if evt & EVT_BUTTON != 0 {
    ///         if (api.read_buttons)() & BTN_CENTER != 0 { break; }
    ///     }
    ///     if evt & EVT_SENSOR != 0 {
    ///         let n = (api.sensor_read)(buf.as_mut_ptr(), 32);
    ///         // process sensor data — it's guaranteed fresh
    ///     }
    ///     if evt & EVT_TIMEOUT != 0 {
    ///         // heartbeat / periodic work
    ///     }
    /// }
    /// ```
    ///
    /// Added in API v11.
    pub wait_event: extern "C" fn(mask: u32, timeout_ms: u32) -> u32,

    /// Read pending input bytes from the host shell `send` command.
    ///
    /// Copies up to `len` bytes from the 128-byte input ring buffer
    /// into `buf`.  Returns number of bytes actually read (0 if empty).
    /// Non-blocking — returns immediately.
    ///
    /// Use `wait_event(EVT_INPUT, timeout)` to sleep until input arrives.
    ///
    /// - `buf`: pointer to caller's buffer
    /// - `len`: size of caller's buffer
    /// - Returns: number of bytes read
    ///
    /// Added in API v12.
    pub read_input: extern "C" fn(buf: *mut u8, len: u32) -> u32,
}

// Compile-time layout check: 26 fields × 4 bytes = 104 bytes on 32-bit ARM.
const _: () = assert!(core::mem::size_of::<MonitorApi>() == 104);

impl MonitorApi {
    /// Convenience: write a byte slice to CDC serial output.
    pub fn print(&self, msg: &[u8]) {
        (self.write_fn)(self.context, msg.as_ptr(), msg.len() as u32);
    }
}
