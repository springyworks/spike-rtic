//! Lightweight RTIC task state tracker.
//!
//! Single `AtomicU32` bitfield — zero cost when not queried.
//! Each bit represents one software task's active/idle state.
//! ISRs (hardware-bound) are always registered and not tracked here.
//!
//! ## Usage
//! ```ignore
//! task_state::mark_active(task_state::HEARTBEAT);
//! // ... task runs ...
//! task_state::mark_idle(task_state::HEARTBEAT);  // one-shot tasks only
//! ```

use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};

static ACTIVE: AtomicU32 = AtomicU32::new(0);

/// When true, port_detect pauses auto-probing (manual sensor command active).
static AUTO_DETECT_PAUSED: AtomicBool = AtomicBool::new(false);

/// USB activity flags — set by USB ISR, cleared by heartbeat.
static USB_RX_SEEN: AtomicBool = AtomicBool::new(false);
static USB_TX_SEEN: AtomicBool = AtomicBool::new(false);

/// When true, sensor_poll should abort immediately (even during handshake).
static SENSOR_ABORT: AtomicBool = AtomicBool::new(false);

// Software task bit positions
pub const HEARTBEAT: u8 = 0;
pub const SEND_BANNER: u8 = 1;
pub const SHELL_TICK: u8 = 2;
pub const BOOT_BEEP: u8 = 3;
pub const BEEP_TONE: u8 = 4;
pub const BUTTON_POLL: u8 = 5;
pub const SENSOR_POLL: u8 = 6;
pub const RUN_DEMO: u8 = 7;
pub const RTTY_TX: u8 = 8;
pub const TEST_ALL: u8 = 9;
pub const RECONNECT_SER: u8 = 10;
pub const PID_RUN: u8 = 11;
pub const SERVO_RUN: u8 = 12;
pub const MOTOR_POLL: u8 = 13;
pub const SENSOR_POLL2: u8 = 14;
pub const MOTOR_POLL2: u8 = 15;

#[inline]
pub fn mark_active(id: u8) {
    ACTIVE.fetch_or(1 << id, Ordering::Relaxed);
}

#[inline]
pub fn mark_idle(id: u8) {
    ACTIVE.fetch_and(!(1 << id), Ordering::Relaxed);
}

#[inline]
pub fn is_active(id: u8) -> bool {
    ACTIVE.load(Ordering::Relaxed) & (1 << id) != 0
}

/// Pause auto-detect (port_detect will skip spawning sensor_poll).
#[inline]
pub fn pause_auto_detect() {
    AUTO_DETECT_PAUSED.store(true, Ordering::Relaxed);
}

/// Resume auto-detect.
#[inline]
pub fn resume_auto_detect() {
    AUTO_DETECT_PAUSED.store(false, Ordering::Relaxed);
}

/// Check if auto-detect is paused.
#[inline]
pub fn is_auto_detect_paused() -> bool {
    AUTO_DETECT_PAUSED.load(Ordering::Relaxed)
}

/// Request sensor_poll to abort (even during handshake).
#[inline]
pub fn request_sensor_abort() {
    SENSOR_ABORT.store(true, Ordering::Relaxed);
}

/// Clear the sensor abort flag (called by sensor_poll on entry).
#[inline]
pub fn clear_sensor_abort() {
    SENSOR_ABORT.store(false, Ordering::Relaxed);
}

/// Check if sensor abort has been requested.
#[inline]
pub fn is_sensor_abort() -> bool {
    SENSOR_ABORT.load(Ordering::Relaxed)
}

// ── USB activity indicator ────────────────────────────────────

/// Mark USB RX activity (host sent data).
#[inline]
pub fn mark_usb_rx() {
    USB_RX_SEEN.store(true, Ordering::Relaxed);
}

/// Mark USB TX activity (firmware sent data to host).
#[inline]
pub fn mark_usb_tx() {
    USB_TX_SEEN.store(true, Ordering::Relaxed);
}

/// Consume USB activity flags. Returns (rx_seen, tx_seen) and clears both.
#[inline]
pub fn take_usb_activity() -> (bool, bool) {
    let rx = USB_RX_SEEN.swap(false, Ordering::Relaxed);
    let tx = USB_TX_SEEN.swap(false, Ordering::Relaxed);
    (rx, tx)
}
