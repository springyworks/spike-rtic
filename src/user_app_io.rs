//! Demo subprocess I/O — output buffer and lifecycle for RAM demos.
//!
//! RAM demos uploaded to SRAM2 run as a **lower-priority RTIC task**
//! (priority 1), so the USB interrupt (priority 2) naturally preempts
//! them and drains output in real-time.  No cooperative yielding
//! needed — Cortex-M preemption does the work.
//!
//! This module provides:
//!   - A 32 KB linear output buffer with atomic read/write cursors
//!     (single-producer at priority 1, single-consumer at priority 2)
//!   - Demo lifecycle management (running flag, pending entry address)
//!   - [`build_api()`] to construct a [`MonitorApi`] with demo-specific
//!     callbacks that route output here instead of the shell buffer
//!
//! ## Memory usage
//!
//! 32 KB buffer + a few atomics ≈ 32 KB of SRAM1 (out of 256 KB).

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

use spike_hub_api::MonitorApi;

use crate::{led_matrix, motor, power, sensor, sound, rtty};

// ── Input ring buffer (host → demo) ────────────────────────

/// 128-byte ring buffer for `send` command text.
/// Producer: shell/USB ISR (priority 2).  Consumer: demo task (priority 1).
/// Higher-priority producer can preempt consumer at any point — safe
/// because producer only advances IN_W, consumer only advances IN_R.
const IN_SIZE: usize = 128;
static mut IN_BUF: [u8; IN_SIZE] = [0; IN_SIZE];
/// Write cursor (producer only).
static IN_W: AtomicUsize = AtomicUsize::new(0);
/// Read cursor (consumer only).
static IN_R: AtomicUsize = AtomicUsize::new(0);

// ── Output buffer ──────────────────────────────────────────

/// 32 KB linear buffer — enough for very long demo output.
/// USB at ~12 Mbit/s drains this far faster than demos produce text.
const BUF_SIZE: usize = 32 * 1024;

/// The buffer lives in .bss (SRAM1).  Accessed only by the producer
/// (demo task, priority 1) and consumer (USB ISR, priority 2).
/// Safety: single-producer / single-consumer with atomic cursors;
/// higher-priority consumer can preempt producer at any point.
static mut BUF: [u8; BUF_SIZE] = [0; BUF_SIZE];

/// Write cursor — only the producer (demo task) increments this.
static W_POS: AtomicUsize = AtomicUsize::new(0);

/// Read cursor — only the consumer (USB ISR) increments this.
static R_POS: AtomicUsize = AtomicUsize::new(0);

// ── Lifecycle ──────────────────────────────────────────────

/// `true` while a demo is executing.
static RUNNING: AtomicBool = AtomicBool::new(false);

/// Abort flag — set by firmware (button ISR / priority 2) to signal the
/// running demo to exit.  Checked by `demo_read_buttons` which injects
/// BTN_CENTER so the demo's own button-check logic triggers a clean exit.
static ABORT: AtomicBool = AtomicBool::new(false);

/// Pending demo entry address (0 = none).
/// Written by `go` command (inside USB ISR / priority 2).
/// Read-and-cleared by the USB ISR to spawn `run_demo`.
static PENDING: AtomicU32 = AtomicU32::new(0);

/// True if the pending demo should run sandboxed (MPU + SVC).
/// False for privileged `go!` mode.
static SANDBOX_MODE: AtomicBool = AtomicBool::new(true);

/// True if the pending demo should run in debug mode (Thread mode,
/// privileged, no MPU — allows DebugMonitor to halt for GDB breakpoints).
static DEBUG_MODE: AtomicBool = AtomicBool::new(false);

/// Last demo entry address — set in `request()` so button short-press
/// can re-run the last demo without knowing which binary was loaded.
static LAST_RUN_ADDR: AtomicU32 = AtomicU32::new(0);

/// Pending RTTY request from demo — USB ISR checks this and spawns rtty_tx.
static RTTY_PENDING: AtomicBool = AtomicBool::new(false);

// ── Abort longjmp mechanism ─────────────────────────────────
//
// When a user-app is killed (Ctrl-C / kill / button), callbacks longjmp
// back to the setjmp point in run_demo, terminating the demo immediately.
// This is the Unix approach: the kernel yanks control — the process never
// needs to cooperate.

/// Saved callee-saved registers + SP + LR for longjmp.
static mut ABORT_JMPBUF: [u32; 10] = [0; 10];

/// True when ABORT_JMPBUF holds a valid recovery context.
static JMPBUF_VALID: AtomicBool = AtomicBool::new(false);

// ── Public API (lifecycle) ─────────────────────────────────

/// Request a demo launch at `addr`.  Called from `go` / `go!` command.
pub fn request(addr: u32, sandboxed: bool) {
    LAST_RUN_ADDR.store(addr, Ordering::Release);
    SANDBOX_MODE.store(sandboxed, Ordering::Release);
    DEBUG_MODE.store(false, Ordering::Release);
    PENDING.store(addr, Ordering::Release);
}

/// Request a demo launch in debug mode (Thread mode, privileged, no MPU).
/// The demo runs from idle so DebugMonitor can halt it for GDB breakpoints.
pub fn request_debug(addr: u32) {
    LAST_RUN_ADDR.store(addr, Ordering::Release);
    SANDBOX_MODE.store(false, Ordering::Release);
    DEBUG_MODE.store(true, Ordering::Release);
    PENDING.store(addr, Ordering::Release);
}

/// Last demo entry address (0 = never run).  For button re-run.
pub fn last_run_addr() -> u32 {
    LAST_RUN_ADDR.load(Ordering::Acquire)
}

/// Check if the pending/running demo is in sandbox mode.
pub fn is_sandbox_mode() -> bool {
    SANDBOX_MODE.load(Ordering::Acquire)
}

/// Check if the pending/running demo is in debug mode
/// (Thread mode, privileged, no MPU — for GDB breakpoints).
pub fn is_debug_mode() -> bool {
    DEBUG_MODE.load(Ordering::Acquire)
}

/// Take a pending RTTY spawn request (returns true once, then false).
/// Called from USB ISR to spawn rtty_tx.
pub fn take_rtty_pending() -> bool {
    RTTY_PENDING.swap(false, Ordering::AcqRel)
}

/// Request an RTTY spawn (sets the pending flag).
/// Called from SVCall handler for sandboxed demos.
pub fn request_rtty_spawn() {
    RTTY_PENDING.store(true, Ordering::Release);
}

/// Take a pending demo request (returns 0 if none).
/// Only takes NON-sandboxed, NON-debug (privileged) requests for the RTIC task.
/// Sandboxed and debug requests are left for the idle loop.
pub fn take_privileged_request() -> Option<u32> {
    if SANDBOX_MODE.load(Ordering::Acquire) || DEBUG_MODE.load(Ordering::Acquire) {
        return None; // leave for idle
    }
    let addr = PENDING.swap(0, Ordering::AcqRel);
    if addr != 0 { Some(addr) } else { None }
}

/// Take a pending SANDBOXED demo request (returns 0 if none).
/// Called from the idle loop (Thread mode).
pub fn take_sandbox_request() -> Option<u32> {
    if !SANDBOX_MODE.load(Ordering::Acquire) {
        return None; // not a sandbox request
    }
    let addr = PENDING.swap(0, Ordering::AcqRel);
    if addr != 0 { Some(addr) } else { None }
}

/// Take a pending DEBUG demo request (returns 0 if none).
/// Called from the USB ISR to spawn run_demo (which delegates to idle).
pub fn take_debug_request() -> Option<u32> {
    if !DEBUG_MODE.load(Ordering::Acquire) {
        return None;
    }
    let addr = PENDING.swap(0, Ordering::AcqRel);
    if addr != 0 { Some(addr) } else { None }
}

/// Is a demo currently running?
pub fn is_running() -> bool {
    RUNNING.load(Ordering::Acquire)
}

/// Set the running flag (called by `run_demo` task).
pub fn set_running(v: bool) {
    RUNNING.store(v, Ordering::Release);
}

/// Request abort of the running demo (called from higher-priority context).
pub fn request_abort() {
    ABORT.store(true, Ordering::Release);
}

/// Check if abort has been requested.
pub fn is_aborted() -> bool {
    ABORT.load(Ordering::Acquire)
}

// ── Abort longjmp public API ───────────────────────────────

// setjmp/longjmp in pure asm — global_asm avoids naked fn restrictions.
// r0 = buf pointer (C ABI), passed by caller.
core::arch::global_asm!(
    ".thumb_func",
    ".global abort_setjmp",
    "abort_setjmp:",
    "stmia r0!, {{r4-r11}}",
    "mov r2, sp",
    "str r2, [r0]",
    "str lr, [r0, #4]",
    "movs r0, #0",
    "bx lr",

    ".thumb_func",
    ".global abort_longjmp",
    "abort_longjmp:",
    "ldmia r0!, {{r4-r11}}",
    "ldr r2, [r0]",
    "ldr lr, [r0, #4]",
    "mov sp, r2",
    "movs r0, #1",
    "bx lr",
);

extern "C" {
    /// setjmp: save r4-r11, SP, LR into buf. Returns 0 on direct call.
    /// On longjmp, execution resumes here with return value 1.
    pub fn abort_setjmp(buf: *mut u32) -> u32;
    /// longjmp: restore saved context and return 1 to setjmp caller.
    fn abort_longjmp(buf: *const u32) -> !;
}

/// Get pointer to the global jmp_buf (for run_demo to pass to abort_setjmp).
pub fn jmpbuf_ptr() -> *mut u32 {
    core::ptr::addr_of_mut!(ABORT_JMPBUF).cast()
}

/// Mark jmp_buf valid/invalid.
pub fn set_jmpbuf_valid(v: bool) {
    JMPBUF_VALID.store(v, Ordering::Release);
}

/// If abort requested and jmp_buf valid, longjmp back to run_demo.
/// This terminates the demo immediately — it never returns.
fn abort_longjmp_if_requested() {
    if ABORT.load(Ordering::Acquire) && JMPBUF_VALID.load(Ordering::Acquire) {
        JMPBUF_VALID.store(false, Ordering::Release);
        unsafe { abort_longjmp(core::ptr::addr_of!(ABORT_JMPBUF).cast()); }
    }
}

// ── Public API (buffer) ────────────────────────────────────

/// Reset read/write cursors.  Call before starting a new demo.
pub fn reset() {
    W_POS.store(0, Ordering::Release);
    R_POS.store(0, Ordering::Release);
    IN_W.store(0, Ordering::Release);
    IN_R.store(0, Ordering::Release);
    ABORT.store(false, Ordering::Release);
}

/// Producer: append bytes to the output buffer.
///
/// If the buffer is full at the tail, spin-waits for the USB ISR
/// (priority 2) to drain and reset the cursors.  Since USB is higher
/// priority, this always makes progress.
pub fn write(data: &[u8]) {
    let mut off = 0;
    while off < data.len() {
        let w = W_POS.load(Ordering::Relaxed);
        let free = BUF_SIZE - w;
        if free > 0 {
            let n = (data.len() - off).min(free);
            // Safety: we are the sole producer; consumer only reads [R..W).
            unsafe {
                BUF[w..w + n].copy_from_slice(&data[off..off + n]);
            }
            W_POS.store(w + n, Ordering::Release);
            off += n;
        } else {
            // Buffer full at tail.  USB ISR will preempt us, drain,
            // and reset cursors to 0 when fully drained.
            cortex_m::asm::nop();
        }
    }
}

/// Consumer: return pending bytes to send over USB.
/// Called from USB ISR (priority 2).
pub fn pending() -> &'static [u8] {
    let r = R_POS.load(Ordering::Relaxed);
    let w = W_POS.load(Ordering::Acquire);
    if w > r {
        // Safety: producer only writes beyond W; we only read [R..W).
        unsafe { &BUF[r..w] }
    } else {
        &[]
    }
}

/// Consumer: advance the read cursor by `n` bytes.
/// If fully drained, reset both cursors to 0 to reclaim space.
///
/// Safe because we (consumer, priority 2) are the highest accessor
/// of these atomics.  While we execute, the producer (priority 1)
/// is suspended and cannot modify W_POS.
pub fn advance(n: usize) {
    let new_r = R_POS.load(Ordering::Relaxed) + n;
    R_POS.store(new_r, Ordering::Release);

    // If fully drained, reclaim the buffer.
    // Producer is suspended (lower priority) so W is stable.
    if new_r == W_POS.load(Ordering::Relaxed) {
        R_POS.store(0, Ordering::Relaxed);
        W_POS.store(0, Ordering::Relaxed);
    }
}

// ── Input buffer public API ────────────────────────────────

/// Producer: push bytes from the host shell into the input ring.
/// Called from shell `send` command (USB ISR context, priority 2).
/// Returns number of bytes actually written (may be < data.len if full).
pub fn push_input(data: &[u8]) -> usize {
    let mut written = 0;
    for &b in data {
        let w = IN_W.load(Ordering::Relaxed);
        let r = IN_R.load(Ordering::Acquire);
        let next_w = (w + 1) % IN_SIZE;
        if next_w == r {
            break; // ring full
        }
        unsafe { IN_BUF[w] = b; }
        IN_W.store(next_w, Ordering::Release);
        written += 1;
    }
    written
}

/// Check if input data is available (for EVT_INPUT detection).
pub fn input_available() -> bool {
    IN_W.load(Ordering::Acquire) != IN_R.load(Ordering::Acquire)
}

// ── MonitorApi construction ────────────────────────────────

/// Build a [`MonitorApi`] whose `write_fn` routes to this module's
/// atomic buffer instead of the shell's internal buffer.
///
/// All other callbacks (delay, LEDs, ADC, buttons, motors) are
/// identical to the interactive shell — they call the same hardware
/// drivers.
pub fn build_api() -> MonitorApi {
    MonitorApi {
        version: spike_hub_api::API_VERSION,
        context: core::ptr::null_mut(), // unused — we route to statics
        write_fn: demo_write_cb,
        delay_ms: demo_delay_ms,
        set_pixel: demo_set_pixel,
        update_leds: demo_update_leds,
        read_adc: demo_read_adc,
        read_buttons: demo_read_buttons,
        motor_set: demo_motor_set,
        motor_brake: demo_motor_brake,
        sensor_read: demo_sensor_read,
        sensor_mode: demo_sensor_mode,
        sound_play: demo_sound_play,
        sound_stop: demo_sound_stop,
        trace_record: demo_trace_record,
        rtty_say: demo_rtty_say,
        rtty_busy: demo_rtty_busy,
        motor_position: demo_motor_position,
        motor_goto: demo_motor_goto,
        port_read: demo_port_read,
        sensor_light: demo_sensor_light,
        imu_init: demo_imu_init,
        imu_read: demo_imu_read,
        set_hub_led: demo_set_hub_led,
        wait_event: demo_wait_event,
        read_input: demo_read_input,
    }
}

// ── Callbacks (extern "C" for ABI stability with separately compiled demos) ──
//
// `extern "C"` is required because the demo binary is compiled
// separately and loaded at runtime.  Both sides are pure Rust —
// no C compiler or libc is involved.  The C calling convention is
// just the stable ABI contract (same approach as ARM Demon/Angel,
// U-Boot, and every ROM monitor on ARM).

extern "C" fn demo_write_cb(_ctx: *mut u8, data: *const u8, len: u32) {
    abort_longjmp_if_requested();
    if !data.is_null() && len > 0 {
        let slice = unsafe { core::slice::from_raw_parts(data, len as usize) };
        write(slice);
        // Pend USB ISR so it drains this output to the host immediately,
        // instead of waiting for the next host-initiated USB event.
        cortex_m::peripheral::NVIC::pend(stm32f4::stm32f413::Interrupt::OTG_FS);
    }
}

extern "C" fn demo_delay_ms(ms: u32) {
    // If abort requested, longjmp back to run_demo — terminates demo.
    abort_longjmp_if_requested();

    let mut remaining = ms;
    while remaining > 0 {
        // ── Firmware-enforced pause ──
        // When paused, hold the user-app here with sensor keepalive.
        // button_poll (priority 2) always preempts us and can toggle
        // the pause flag or set ABORT to kill.
        while crate::is_paused() && !ABORT.load(Ordering::Acquire) {
            power::delay_ms(20);
            sensor::demo_maintain();
        }

        // Check abort again after potential pause
        if ABORT.load(Ordering::Acquire) {
            sensor::demo_maintain();
            return;
        }

        let chunk = if remaining >= 20 { 20 } else { remaining };
        power::delay_ms(chunk);
        remaining -= chunk;
        sensor::demo_maintain();
    }
}

extern "C" fn demo_set_pixel(idx: u32, bri: u32) {
    if idx < 25 && bri <= 100 {
        led_matrix::set_pixel(idx as usize, bri as u16);
    }
}

extern "C" fn demo_update_leds() {
    unsafe { led_matrix::update() };
}

extern "C" fn demo_read_adc(ch: u32) -> u32 {
    if ch <= 18 { crate::read_adc(ch) } else { 0 }
}

extern "C" fn demo_read_buttons() -> u8 {
    // If firmware requested abort, inject BTN_CENTER so the demo's own
    // button check triggers a clean exit — UNIX SIGINT style.
    if ABORT.load(Ordering::Acquire) {
        return spike_hub_api::BTN_CENTER;
    }
    crate::read_buttons()
}

extern "C" fn demo_motor_set(port: u32, speed: i32) {
    abort_longjmp_if_requested();
    motor::set(port, speed);
}

extern "C" fn demo_motor_brake(port: u32) {
    motor::brake(port);
}

extern "C" fn demo_sensor_read(buf: *mut u8, len: u32) -> u32 {
    if buf.is_null() || len == 0 { return 0; }
    let slice = unsafe { core::slice::from_raw_parts_mut(buf, len as usize) };
    sensor::demo_sensor_read(slice) as u32
}

extern "C" fn demo_sensor_mode(mode: u32) {
    sensor::demo_sensor_mode(mode as u8);
}

extern "C" fn demo_sound_play(freq_hz: u32) {
    abort_longjmp_if_requested();
    sound::play(freq_hz);
}

extern "C" fn demo_sound_stop() {
    sound::stop();  // always allow stop
}

extern "C" fn demo_trace_record(tag: u8, val: u8, arg: u16) {
    crate::trace::record(tag, val, arg);
}

extern "C" fn demo_rtty_say(data: *const u8, len: u32) {
    abort_longjmp_if_requested();
    if data.is_null() || len == 0 { return; }
    // Don't overwrite an in-progress transmission
    if rtty::is_busy() { return; }
    let slice = unsafe { core::slice::from_raw_parts(data, len.min(80) as usize) };
    rtty::set_message(slice);
    RTTY_PENDING.store(true, Ordering::Release);
}

extern "C" fn demo_rtty_busy() -> u32 {
    abort_longjmp_if_requested();
    if rtty::is_busy() { 1 } else { 0 }
}

extern "C" fn demo_motor_position() -> i32 {
    sensor::demo_motor_position()
}

extern "C" fn demo_motor_goto(port: u32, degrees: i32) -> i32 {
    demo_motor_goto_inner(port, degrees)
}

pub extern "C" fn demo_port_read(port: u32, buf: *mut u8, len: u32) -> u32 {
    if buf.is_null() || len == 0 || port > 5 { return 0; }
    let st = sensor::port_state(port as usize);
    if st.status != sensor::Status::Data || st.data_len == 0 { return 0; }
    let n = st.data_len.min(len as usize);
    let slice = unsafe { core::slice::from_raw_parts_mut(buf, n) };
    slice.copy_from_slice(&st.data[..n]);
    n as u32
}

extern "C" fn demo_sensor_light(r: u32, g: u32, b: u32) {
    sensor::demo_sensor_light(r.min(100) as u8, g.min(100) as u8, b.min(100) as u8);
}

extern "C" fn demo_imu_init() -> u32 {
    crate::imu::init()
}

extern "C" fn demo_imu_read(buf: *mut u8, len: u32) -> u32 {
    if buf.is_null() || len < 12 { return 0; }
    let slice = unsafe { core::slice::from_raw_parts_mut(buf, len as usize) };
    crate::imu::read(slice)
}

extern "C" fn demo_set_hub_led(r: u32, g: u32, b: u32) {
    let r16 = ((r.min(100) as u32) * 0xFFFF / 100) as u16;
    let g16 = ((g.min(100) as u32) * 0xFFFF / 100) as u16;
    let b16 = ((b.min(100) as u32) * 0xFFFF / 100) as u16;
    led_matrix::set_status_rgb(crate::pins::BATTERY_LED, r16, g16, b16);
    unsafe { led_matrix::update(); }
}

/// Block until an event in `mask` fires, or `timeout_ms` expires.
///
/// Returns a bitmask of fired events (EVT_SENSOR, EVT_BUTTON, EVT_MOTOR)
/// or EVT_TIMEOUT if the deadline expired first.  Maintains sensor
/// keepalive, respects pause/abort, and never busy-waits — the 10ms
/// chunks are real MCU sleep via `power::delay_ms`.
///
/// Used by both the direct callback (`go!` privileged mode) and the
/// SVC #22 handler (`go` sandboxed mode).
pub extern "C" fn demo_wait_event(mask: u32, timeout_ms: u32) -> u32 {
    // Abort check
    abort_longjmp_if_requested();
    if ABORT.load(Ordering::Acquire) {
        return spike_hub_api::EVT_TIMEOUT;
    }

    // Snapshot current state — events fire on *change*
    let snap_buttons = crate::read_buttons();
    let snap_sensor_seq = sensor::port_state(sensor::demo_active_port()).data_seq;
    let snap_motor_pos = sensor::demo_motor_position();

    let mut remaining = timeout_ms.min(30_000);
    loop {
        // ── Firmware-enforced pause (same pattern as delay_ms) ──
        while crate::is_paused() && !ABORT.load(Ordering::Acquire) {
            crate::power::delay_ms(20);
            sensor::demo_maintain();
            crate::watchdog::feed();
        }
        if ABORT.load(Ordering::Acquire) {
            return spike_hub_api::EVT_TIMEOUT;
        }

        let mut fired = 0u32;

        // Check sensor data freshness (sequence number changed?)
        if mask & spike_hub_api::EVT_SENSOR != 0 {
            let cur = sensor::port_state(sensor::demo_active_port()).data_seq;
            if cur != snap_sensor_seq {
                fired |= spike_hub_api::EVT_SENSOR;
            }
        }

        // Check button state change
        if mask & spike_hub_api::EVT_BUTTON != 0 {
            let cur = crate::read_buttons();
            if cur != snap_buttons {
                fired |= spike_hub_api::EVT_BUTTON;
            }
        }

        // Check motor position change
        if mask & spike_hub_api::EVT_MOTOR != 0 {
            let cur = sensor::demo_motor_position();
            if cur != snap_motor_pos {
                fired |= spike_hub_api::EVT_MOTOR;
            }
        }

        // Check input buffer has data
        if mask & spike_hub_api::EVT_INPUT != 0 {
            if input_available() {
                fired |= spike_hub_api::EVT_INPUT;
            }
        }

        if fired != 0 {
            return fired;
        }

        if remaining == 0 {
            return spike_hub_api::EVT_TIMEOUT;
        }

        // Sleep 10ms, maintain keepalive — NOT busy-wait
        let chunk = remaining.min(10);
        crate::power::delay_ms(chunk);
        remaining -= chunk;
        sensor::demo_maintain();
        crate::watchdog::feed();
    }
}

/// Consumer: read bytes from the input ring into caller's buffer.
/// Called from demo context (priority 1).  Returns bytes read.
pub extern "C" fn demo_read_input(buf: *mut u8, len: u32) -> u32 {
    abort_longjmp_if_requested();
    if buf.is_null() || len == 0 { return 0; }
    let mut n = 0u32;
    while n < len {
        let r = IN_R.load(Ordering::Relaxed);
        let w = IN_W.load(Ordering::Acquire);
        if r == w { break; } // empty
        unsafe {
            *buf.add(n as usize) = IN_BUF[r];
        }
        IN_R.store((r + 1) % IN_SIZE, Ordering::Release);
        n += 1;
    }
    n
}

/// Blocking ramp+nudge controller for motor positioning.
/// Public so the SVC handler in sandbox.rs can call it directly.
///
/// Two-phase approach:
///   Phase 1 (approach): ramp from omax → sc over decel zone, brake at threshold
///   Phase 2 (recovery): nudge pulses (drive at sc for 100ms, brake for 200ms)
///
/// Maintains both sensor and motor keepalives during the blocking loop.
pub fn demo_motor_goto_inner(port: u32, degrees: i32) -> i32 {
    // Controller parameters (baked defaults from motor.rs Pid)
    let omax: i32 = 40;
    let sc: i32 = 35;
    let brake_zone: i32 = 15;
    let decel_zone: i32 = brake_zone * 5; // 75°
    let settle_hold_ms: u32 = 2000;
    let nudge_len: u32 = 5;   // iters × 20ms = 100ms
    let stall_wait: u32 = 10; // iters × 20ms = 200ms

    let init_pos = sensor::demo_motor_position();
    let distance = (degrees - init_pos).abs();
    let approach_ms = ((distance as u64 * 1000) / 200).min(15000) as u32;
    let total_ms = approach_ms + settle_hold_ms + 5000;
    let total_iters = (total_ms + 19) / 20;

    let mut settled_at: Option<u32> = None;
    let mut approach_done = false;
    let mut stall_count: u32 = 0;
    let mut nudging: u32 = 0;
    let mut prev_pos = init_pos;

    for iter in 0..total_iters.max(200) {
        let t_ms = iter * 20;

        // Abort check: brake and bail out immediately
        if ABORT.load(Ordering::Acquire) {
            motor::brake(port);
            return degrees - sensor::demo_motor_position();
        }

        // Firmware-enforced pause: hold here with keepalive
        while crate::is_paused() && !ABORT.load(Ordering::Acquire) {
            motor::brake(port);
            power::delay_ms(20);
            sensor::demo_maintain();
        }

        let pos = sensor::demo_motor_position();
        let err = degrees - pos;
        let abs_err = err.abs();

        // Track stall
        if pos == prev_pos {
            stall_count += 1;
        } else {
            stall_count = 0;
        }
        prev_pos = pos;

        if abs_err <= brake_zone {
            // In brake zone — hold brake, start settle timer
            motor::brake(port);
            approach_done = true;
            nudging = 0;
            if settled_at.is_none() {
                settled_at = Some(t_ms);
            }
            if let Some(sat) = settled_at {
                if t_ms - sat >= settle_hold_ms {
                    break;
                }
            }
        } else if !approach_done {
            // Phase 1: approach via decel ramp
            settled_at = None;
            let out_mag = if abs_err >= decel_zone {
                omax
            } else {
                let range = decel_zone - brake_zone;
                let frac = abs_err - brake_zone;
                sc + (omax - sc) * frac / range.max(1)
            };
            let out = if err > 0 { out_mag } else { -out_mag };
            motor::set(port, out);
        } else if nudging > 0 {
            // Phase 2: nudge pulse in progress
            nudging -= 1;
            let out = if err > 0 { sc } else { -sc };
            motor::set(port, out);
            if nudging == 0 {
                motor::brake(port);
                stall_count = 0;
            }
        } else if stall_count >= stall_wait && sc > 0 {
            // Phase 2: stalled outside brake zone → start nudge
            nudging = nudge_len;
            stall_count = 0;
            let out = if err > 0 { sc } else { -sc };
            motor::set(port, out);
        } else {
            // Phase 2: braking while waiting
            motor::brake(port);
            settled_at = None;
        }

        // 20ms delay with dual-port keepalive
        power::delay_ms(20);
        sensor::demo_maintain();
    }

    motor::brake(port);
    let final_pos = sensor::demo_motor_position();
    degrees - final_pos
}
