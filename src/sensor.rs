#![allow(dead_code)]
//! LUMP (LEGO UART Message Protocol) sensor driver.
//!
//! Translated from pybricks-micropython `lib/pbio/src/port_lump.c`.
//! Supports LPF2 (Powered Up) UART sensors on SPIKE Prime hub ports A–F.
//!
//! ## Protocol overview
//!
//! 1. Hub sends SPEED command at 115200 baud.
//! 2. If device ACKs → continue at 115200; else fall back to 2400 baud sync.
//! 3. Read TYPE, MODES, SPEED, INFO messages (mode names, formats).
//! 4. Device sends ACK → hub replies ACK → switch baud to negotiated rate.
//! 5. DATA mode: hub sends NACK keepalive every 100 ms, device streams data.
//!
//! ## Hardware
//!
//! Each port has a UART, a buffer-enable pin (active LOW), and two
//! device-ID sense pins (p5/p6).  The buffer-enable gates the UART
//! TX/RX lines through a level-shifting buffer on the hub PCB.

use crate::pins;
use crate::reg::{reg_modify, reg_read, reg_write};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicUsize, Ordering};
use core::cell::UnsafeCell;
use core::ptr::addr_of_mut;

// ── Global per-port state (lock-free) ──
// Each port's state is written by exactly one poll task and read by
// shell/demo/servo tasks.  Uses UnsafeCell for interior mutability;
// safety relies on single-writer-per-port discipline.

struct PortStateCell(UnsafeCell<SensorState>);
unsafe impl Sync for PortStateCell {}

static PORT_STATES: [PortStateCell; 6] = [
    PortStateCell(UnsafeCell::new(SensorState::new())),
    PortStateCell(UnsafeCell::new(SensorState::new())),
    PortStateCell(UnsafeCell::new(SensorState::new())),
    PortStateCell(UnsafeCell::new(SensorState::new())),
    PortStateCell(UnsafeCell::new(SensorState::new())),
    PortStateCell(UnsafeCell::new(SensorState::new())),
];

// ── Per-port abort flags ──
static PORT_ABORT: [AtomicBool; 6] = [
    AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false),
    AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false),
];

// ── Per-port cached type_id (for fast re-probe) ──
static LAST_TYPE: [AtomicU8; 6] = [
    AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0),
    AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0),
];

// ── Per-port cached capability flags (INFO_NAME FLAGS0) ──
static LAST_CAPS: [AtomicU8; 6] = [
    AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0),
    AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0),
];

/// Return cached type_id for a port (0 = unknown).
pub fn cached_port_type(idx: usize) -> u8 {
    if idx < 6 { LAST_TYPE[idx].load(Ordering::Acquire) } else { 0 }
}

/// Cache the type_id after a successful handshake.
pub fn cache_port_type(idx: usize, type_id: u8) {
    if idx < 6 { LAST_TYPE[idx].store(type_id, Ordering::Release); }
}

/// Return cached capability flags for a port (0 = none).
pub fn cached_port_caps(idx: usize) -> u8 {
    if idx < 6 { LAST_CAPS[idx].load(Ordering::Acquire) } else { 0 }
}

/// Cache capability flags after a successful handshake.
pub fn cache_port_caps(idx: usize, caps: u8) {
    if idx < 6 { LAST_CAPS[idx].store(caps, Ordering::Release); }
}

// INFO_NAME FLAGS0 bits for sensor power requirements.
pub const NEEDS_SUPPLY_PIN1: u8 = 1 << 6; // Pin1(+), Pin2(-)
pub const NEEDS_SUPPLY_PIN2: u8 = 1 << 7; // Pin1(-), Pin2(+)

/// Apply motor-pin power for sensors that need it (e.g. ultrasonic transducers).
/// Called after handshake with the parsed capability flags.
pub fn apply_pin_power(port_idx: u8, caps: u8) {
    let idx = port_idx as u32;
    if caps & NEEDS_SUPPLY_PIN1 != 0 {
        // Pin1(+), Pin2(-) = motor reverse
        crate::motor::set(idx, -100);
    } else if caps & NEEDS_SUPPLY_PIN2 != 0 {
        // Pin1(-), Pin2(+) = motor forward
        crate::motor::set(idx, 100);
    }
}

/// Request abort for a specific port's poll task.
pub fn request_port_abort(idx: usize) {
    if idx < 6 { PORT_ABORT[idx].store(true, Ordering::Release); }
}

/// Clear the abort flag (called by poll task on entry).
pub fn clear_port_abort(idx: usize) {
    if idx < 6 { PORT_ABORT[idx].store(false, Ordering::Release); }
}

/// Check if abort has been requested for a port.
pub fn is_port_abort(idx: usize) -> bool {
    if idx < 6 { PORT_ABORT[idx].load(Ordering::Acquire) } else { true }
}

/// Read a port's sensor state (lock-free, returns a copy).
pub fn port_state(idx: usize) -> SensorState {
    unsafe { core::ptr::read_volatile(PORT_STATES[idx].0.get()) }
}

/// Write a port's sensor state (call only from that port's poll task).
pub fn set_port_state(idx: usize, state: &SensorState) {
    unsafe { core::ptr::write_volatile(PORT_STATES[idx].0.get(), *state) };
}

/// Read just the status of a port (lightweight check).
pub fn port_status(idx: usize) -> Status {
    unsafe { (*PORT_STATES[idx].0.get()).status }
}

/// Set a port's status to None (for external abort/stop).
pub fn port_set_none(idx: usize) {
    unsafe { (*PORT_STATES[idx].0.get()).status = Status::None };
}

// ── USART register offsets (same for UART4–10 on STM32F413) ──
const SR: u32 = 0x00;
const DR: u32 = 0x04;
const BRR: u32 = 0x08;
const CR1: u32 = 0x0C;
const CR3: u32 = 0x14;

// SR bits
const SR_RXNE: u32 = 1 << 5;
const SR_TXE: u32 = 1 << 7;
const SR_ORE: u32 = 1 << 3;

// CR1 bits
const CR1_UE: u32 = 1 << 13;
const CR1_TE: u32 = 1 << 3;
const CR1_RE: u32 = 1 << 2;
const CR1_RXNEIE: u32 = 1 << 5;

// ── Per-port interrupt-driven RX ring buffers ──
// Each UART port (A–F) has its own SPSC ring buffer.
// ISR (priority 3) is sole producer, task (priority 1) is sole consumer.
const NUM_PORTS: usize = 6;
const RX_BUF_SIZE: usize = 512; // must be power of 2; 6 × 512 = 3 KB total
static mut RX_BUFS: [[u8; RX_BUF_SIZE]; NUM_PORTS] = [[0; RX_BUF_SIZE]; NUM_PORTS];
static RX_HEADS: [AtomicUsize; NUM_PORTS] = [
    AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0),
    AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0),
];
static RX_TAILS: [AtomicUsize; NUM_PORTS] = [
    AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0),
    AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0),
];

// ── Per-port ISR diagnostics ──
static ISR_RX_COUNT: [AtomicU32; NUM_PORTS] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
];
static ISR_ORE_COUNT: [AtomicU32; NUM_PORTS] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
];

/// Get ISR byte count for a port (diagnostic).
pub fn isr_rx_count(port: usize) -> u32 {
    ISR_RX_COUNT[port].load(Ordering::Relaxed)
}
/// Get ISR overrun count for a port (diagnostic).
pub fn isr_ore_count(port: usize) -> u32 {
    ISR_ORE_COUNT[port].load(Ordering::Relaxed)
}

/// Called from each RTIC UART ISR handler.
/// Reads the received byte from the UART data register and stores
/// it in the port's ring buffer.  Runs at highest priority — never preempted.
pub unsafe fn uart_isr_handler(uart_base: u32, port: usize) {
    let sr = reg_read(uart_base, SR);
    // Always read DR if RXNE is set — even when ORE is also set.
    // The byte in DR is valid; ORE means a *previous* shift-register
    // byte was lost, not the current DR byte.
    if sr & SR_RXNE != 0 {
        let byte = reg_read(uart_base, DR) as u8;
        ISR_RX_COUNT[port].fetch_add(1, Ordering::Relaxed);
        if sr & SR_ORE != 0 {
            ISR_ORE_COUNT[port].fetch_add(1, Ordering::Relaxed);
        }
        let head = RX_HEADS[port].load(Ordering::Relaxed);
        let next = (head + 1) & (RX_BUF_SIZE - 1);
        let tail = RX_TAILS[port].load(Ordering::Acquire);
        if next != tail {
            RX_BUFS[port][head] = byte;
            RX_HEADS[port].store(next, Ordering::Release);
        }
    } else if sr & SR_ORE != 0 {
        // ORE without RXNE: clear by reading DR
        let _ = reg_read(uart_base, DR);
        ISR_ORE_COUNT[port].fetch_add(1, Ordering::Relaxed);
    }
}

/// Flush (discard) all bytes in a port's RX ring buffer.
pub fn rx_flush(port: usize) {
    RX_TAILS[port].store(RX_HEADS[port].load(Ordering::Acquire), Ordering::Release);
}

/// Read one byte from a port's ring buffer (non-blocking).
pub fn rx_read_byte(port: usize) -> Option<u8> {
    let tail = RX_TAILS[port].load(Ordering::Relaxed);
    let head = RX_HEADS[port].load(Ordering::Acquire);
    if tail == head {
        return None;
    }
    let byte = unsafe { RX_BUFS[port][tail] };
    RX_TAILS[port].store((tail + 1) & (RX_BUF_SIZE - 1), Ordering::Release);
    Some(byte)
}

/// Number of bytes available in a port's ring buffer.
fn rx_available(port: usize) -> usize {
    let tail = RX_TAILS[port].load(Ordering::Relaxed);
    let head = RX_HEADS[port].load(Ordering::Acquire);
    (head.wrapping_sub(tail)) & (RX_BUF_SIZE - 1)
}

/// Public wrapper for diagnostics.
pub fn rx_available_pub(port: usize) -> usize {
    rx_available(port)
}

/// Public peek for diagnostics — read byte at offset n without consuming.
pub fn rx_peek_at_pub(port: usize, n: usize) -> u8 {
    rx_peek_at(port, n)
}

/// Peek at byte at offset `n` past tail without consuming.
fn rx_peek_at(port: usize, n: usize) -> u8 {
    let tail = RX_TAILS[port].load(Ordering::Relaxed);
    unsafe { RX_BUFS[port][(tail + n) & (RX_BUF_SIZE - 1)] }
}

/// Advance tail by `n` bytes (consume without reading).
fn rx_skip(port: usize, n: usize) {
    let tail = RX_TAILS[port].load(Ordering::Relaxed);
    RX_TAILS[port].store((tail + n) & (RX_BUF_SIZE - 1), Ordering::Release);
}

// ── Pending light write (shell → sensor_poll) ──
// Packed as 0x01_RR_GG_BB in a u32.  0 = no pending request.
static PENDING_LIGHT: AtomicU32 = AtomicU32::new(0);

/// Queue a light-write for the next sensor_poll keepalive cycle.
pub fn queue_sensor_light(r: u8, g: u8, b: u8) {
    let packed = 0x0100_0000
        | ((r as u32) << 16)
        | ((g as u32) << 8)
        | (b as u32);
    PENDING_LIGHT.store(packed, Ordering::Release);
}

/// If a light write is pending, send it on `sp` and return true.
/// Called from sensor_poll at keepalive time.
///
/// For color sensors: controls LED power via the motor H-bridge pins.
/// - Brightness > 0: apply motor-pin power, then send LUMP DATA mode 3.
/// - Brightness = 0: coast motor pins (hardware LED power cut).
///
/// The sensor's internal MCU automatically lights LEDs in mode 5 (RGB_I)
/// when the motor pins supply power.  Coasting the pins is the only
/// reliable way to turn them off without a mode switch.
pub fn send_pending_light(sp: &SensorPort, port_idx: usize) -> bool {
    let packed = PENDING_LIGHT.swap(0, Ordering::AcqRel);
    if packed == 0 { return false; }
    let r = ((packed >> 16) & 0xFF) as u8;
    let g = ((packed >> 8) & 0xFF) as u8;
    let b = (packed & 0xFF) as u8;

    if r == 0 && g == 0 && b == 0 {
        // Hardware LED off: coast motor pins → no power to LEDs.
        crate::motor::set(port_idx as u32, 0);
        return true;
    }

    // Power the motor pins so the LEDs have a supply.
    // Use cached capabilities to pick the right polarity;
    // fall back to forward (+100) if caps say nothing.
    let caps = cached_port_caps(port_idx);
    if caps & NEEDS_SUPPLY_PIN1 != 0 {
        crate::motor::set(port_idx as u32, -100);
    } else {
        crate::motor::set(port_idx as u32, 100);
    }

    // 1) EXT_MODE prefix: CMD | SIZE_1 | EXT_MODE, ext_byte, checksum
    let ext_hdr = LUMP_MSG_TYPE_CMD | LUMP_MSG_SIZE_1 | LUMP_CMD_EXT_MODE; // 0x46
    let ext_val: u8 = 0; // mode 3 < 8 → extension byte = 0
    let ext_chk = 0xFF ^ ext_hdr ^ ext_val;
    let ext_pkt = [ext_hdr, ext_val, ext_chk];
    unsafe { uart_tx(sp, &ext_pkt); }

    // 2) DATA message: mode 3, 3 bytes RGB (padded to 4)
    let mut buf = [0u8; MAX_MSG];
    let data = [r, g, b];
    let len = build_msg(&mut buf, LUMP_MSG_TYPE_DATA, MODE_LIGHT, &data);
    unsafe { uart_tx(sp, &buf[..len]); }
    true
}

// ── Demo-mode sensor access ──
// When a RAM demo is running, sensor_poll is stalled (same priority).
// The ISR keeps filling the ring buffer.  These statics let demo
// callbacks drain the buffer and send keepalives independently.

/// Which port the demo reads from (0xFF = not set).
static DEMO_PORT_IDX: AtomicU8 = AtomicU8::new(0xFF);

/// Demo's own sensor state (separate from sensor_poll's copy).
static mut DEMO_STATE: SensorState = SensorState::new();

/// Keepalive counter — send NACK every 3rd call to `demo_sensor_read`.
static DEMO_KA_CTR: AtomicU8 = AtomicU8::new(0);

/// Remaining demo_maintain() calls to suppress keepalives after mode switch.
/// Each call is ~20ms, so 5 calls ≈ 100ms of suppression.
static DEMO_KA_SUPPRESS: AtomicU8 = AtomicU8::new(0);

/// Prepare demo sensor access for a given port.
/// Called by `run_demo` before launching the demo entry point.
pub fn demo_set_port(idx: u8) {
    DEMO_PORT_IDX.store(idx, Ordering::Release);
    DEMO_KA_CTR.store(0, Ordering::Relaxed);
    unsafe { *addr_of_mut!(DEMO_STATE) = SensorState::new(); }
    if idx < NUM_PORTS as u8 {
        rx_flush(idx as usize);
    }
}

/// Return the demo's active sensor port index (0–5), or 0xFF if unset.
pub fn demo_active_port() -> usize {
    DEMO_PORT_IDX.load(Ordering::Acquire) as usize
}

/// Demo callback: drain ring buffer, keepalive, return latest data.
/// Returns number of valid bytes copied into `buf`.
pub fn demo_sensor_read(buf: &mut [u8]) -> usize {
    let idx = DEMO_PORT_IDX.load(Ordering::Acquire);
    if idx > 5 || buf.is_empty() { return 0; }

    let state = unsafe { &mut *addr_of_mut!(DEMO_STATE) };
    state.port_idx = idx;
    lump_poll_data(state);

    // Keepalive every 3rd call, but respect mode-switch suppress window.
    let ctr = DEMO_KA_CTR.load(Ordering::Relaxed).wrapping_add(1);
    DEMO_KA_CTR.store(ctr, Ordering::Relaxed);
    if ctr % 3 == 0 && DEMO_KA_SUPPRESS.load(Ordering::Relaxed) == 0 {
        unsafe { lump_keepalive(PORTS[idx as usize]); }
        crate::trace::record(crate::trace::TAG_KEEPALIVE, 1, idx as u16);
    }

    crate::trace::record(
        crate::trace::TAG_SENSOR_DATA,
        state.data_len as u8,
        if state.data_len >= 2 {
            u16::from_le_bytes([state.data[0], state.data[1]])
        } else if state.data_len == 1 {
            state.data[0] as u16
        } else {
            0xFFFF
        },
    );

    let len = state.data_len.min(buf.len());
    if len > 0 {
        buf[..len].copy_from_slice(&state.data[..len]);
    }
    len
}

/// Demo callback: switch sensor mode on the active port.
/// Suppresses keepalives for ~100ms (5 × 20ms maintain calls) to let
/// the sensor process the SELECT command without interference.
pub fn demo_sensor_mode(mode: u8) {
    let idx = DEMO_PORT_IDX.load(Ordering::Acquire);
    if idx > 5 { return; }
    // Clear stale data from previous mode
    let state = unsafe { &mut *addr_of_mut!(DEMO_STATE) };
    state.data_len = 0;
    state.data_received = false;
    // Suppress keepalives for ~100ms (5 maintain calls at 20ms each)
    DEMO_KA_SUPPRESS.store(5, Ordering::Release);
    unsafe { lump_select_mode(PORTS[idx as usize], mode); }
}

/// Send LUMP DATA message to set color sensor LED brightness.
///
/// The color sensor (type 61) mode 3 ("LIGHT") accepts 3× i8 values
/// controlling the three LEDs.  This sends a DATA write command with
/// mode 3 in the header and 3 bytes of brightness (0-100 each).
///
/// The sensor must already be in a mode that accepts the light command,
/// or this acts as a write-to-mode-3 command.
pub fn demo_sensor_light(r: u8, g: u8, b: u8) {
    let idx = DEMO_PORT_IDX.load(Ordering::Acquire);
    if idx > 5 { return; }
    let sp = PORTS[idx as usize];

    // 1) EXT_MODE prefix (required by Powered Up sensors before DATA writes)
    let ext_hdr = LUMP_MSG_TYPE_CMD | LUMP_MSG_SIZE_1 | LUMP_CMD_EXT_MODE;
    let ext_val: u8 = 0;
    let ext_chk = 0xFF ^ ext_hdr ^ ext_val;
    let ext_pkt = [ext_hdr, ext_val, ext_chk];
    unsafe { uart_tx(sp, &ext_pkt); }

    // 2) DATA message: mode 3, 3 bytes RGB
    let mut buf = [0u8; MAX_MSG];
    let data = [r, g, b];
    let len = build_msg(&mut buf, LUMP_MSG_TYPE_DATA, MODE_LIGHT, &data);
    unsafe { uart_tx(sp, &buf[..len]); }
}

/// Drain ring buffer + send keepalive during SVC delay_ms busy-wait.
///
/// Called from the SVCall handler every ~20 ms during demo delay_ms.
/// This is critical because SVCall runs at the same NVIC priority as
/// sensor_poll (both priority 15), so sensor_poll cannot preempt the
/// SVCall busy-wait.  Without this, the sensor receives no keepalives
/// during delay_ms and disconnects after ~250 ms.
///
/// Keepalives are suppressed for ~100ms after a mode switch (SELECT)
/// to let the sensor process the mode change without interference.
pub fn demo_maintain() {
    let idx = DEMO_PORT_IDX.load(Ordering::Acquire);
    if idx > 5 { return; }
    let state = unsafe { &mut *addr_of_mut!(DEMO_STATE) };
    state.port_idx = idx;
    lump_poll_data(state);
    // Decrement suppress counter; only send keepalive when it reaches 0
    let suppress = DEMO_KA_SUPPRESS.load(Ordering::Relaxed);
    if suppress > 0 {
        DEMO_KA_SUPPRESS.store(suppress - 1, Ordering::Relaxed);
    } else {
        unsafe { lump_keepalive(PORTS[idx as usize]); }
    }
    crate::trace::record(crate::trace::TAG_DELAY_KA, 2, idx as u16);

    // Also maintain the motor port if active
    demo_motor_maintain();
}

/// Clear demo port binding (called after demo exits).
pub fn demo_clear_port() {
    DEMO_PORT_IDX.store(0xFF, Ordering::Release);
}

// ── Demo motor port (independent second LUMP channel) ──
// Allows demos to read motor position from one port while reading
// a color sensor on another.  The ISR feeds each port's ring buffer
// independently; these statics provide the drain/keepalive path
// for the motor port during demo execution.

/// Which port the demo's motor is on (0xFF = not set).
static DEMO_MOTOR_PORT_IDX: AtomicU8 = AtomicU8::new(0xFF);

/// Demo's motor sensor state (separate from DEMO_STATE).
static mut DEMO_MOTOR_STATE: SensorState = SensorState::new();

/// Keepalive counter for motor port.
static DEMO_MOTOR_KA_CTR: AtomicU8 = AtomicU8::new(0);

/// Prepare demo motor access for a given port.
pub fn demo_set_motor_port(idx: u8) {
    DEMO_MOTOR_PORT_IDX.store(idx, Ordering::Release);
    DEMO_MOTOR_KA_CTR.store(0, Ordering::Relaxed);
    unsafe {
        *addr_of_mut!(DEMO_MOTOR_STATE) = SensorState::new();
        (*addr_of_mut!(DEMO_MOTOR_STATE)).port_idx = idx;
    }
    if idx < NUM_PORTS as u8 {
        rx_flush(idx as usize);
    }
}

/// Drain motor port ring buffer + send keepalive.
/// Called from demo_delay_ms every ~20ms alongside demo_maintain.
pub fn demo_motor_maintain() {
    let idx = DEMO_MOTOR_PORT_IDX.load(Ordering::Acquire);
    if idx > 5 { return; }
    let state = unsafe { &mut *addr_of_mut!(DEMO_MOTOR_STATE) };
    state.port_idx = idx;
    lump_poll_data(state);
    let ctr = DEMO_MOTOR_KA_CTR.load(Ordering::Relaxed).wrapping_add(1);
    DEMO_MOTOR_KA_CTR.store(ctr, Ordering::Relaxed);
    if ctr % 5 == 0 {
        unsafe { lump_keepalive(PORTS[idx as usize]); }
    }
}

/// Read motor position in degrees from the motor port's LUMP data.
/// Drains the ring buffer first to get fresh data.
/// Returns cumulative degrees (i32), or 0 if no motor data.
pub fn demo_motor_position() -> i32 {
    let idx = DEMO_MOTOR_PORT_IDX.load(Ordering::Acquire);
    if idx > 5 { return 0; }
    let state = unsafe { &mut *addr_of_mut!(DEMO_MOTOR_STATE) };
    state.port_idx = idx;
    lump_poll_data(state);
    state.motor_pos_degrees()
}

/// Clear demo motor port binding (called after demo exits).
pub fn demo_clear_motor_port() {
    DEMO_MOTOR_PORT_IDX.store(0xFF, Ordering::Release);
}

// ── RCC ──
const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_APB1ENR: u32 = 0x40;
const RCC_APB2ENR: u32 = 0x44;

// ── LUMP protocol constants ──
const LUMP_MSG_TYPE_SYS: u8 = 0x00;
const LUMP_MSG_TYPE_CMD: u8 = 0x40;
const LUMP_MSG_TYPE_INFO: u8 = 0x80;
const LUMP_MSG_TYPE_DATA: u8 = 0xC0;

const LUMP_MSG_TYPE_MASK: u8 = 0xC0;
const LUMP_MSG_SIZE_MASK: u8 = 0x38;
const LUMP_MSG_CMD_MASK: u8 = 0x07;

const LUMP_SYS_NACK: u8 = 0x02;
const LUMP_SYS_ACK: u8 = 0x04;

const LUMP_CMD_TYPE: u8 = 0;
const LUMP_CMD_SPEED: u8 = 2;
const LUMP_CMD_SELECT: u8 = 3;
const LUMP_CMD_EXT_MODE: u8 = 6;

const LUMP_MSG_SIZE_1: u8 = 0 << 3;
const LUMP_MSG_SIZE_4: u8 = 2 << 3;

pub const SPEED_LPF2: u32 = 115200;
const LUMP_SPEED_LPF2: u32 = 115200;
const LUMP_SPEED_MIN: u32 = 2400;

const LUMP_TYPE_MIN: u8 = 29;
const LUMP_TYPE_MAX: u8 = 101;

// keepalive interval and IO timeout (ms)
const KEEPALIVE_MS: u32 = 100;
const IO_TIMEOUT_MS: u32 = 250;

// ── Known device types ──
pub const TYPE_SPIKE_COLOR_SENSOR: u8 = 61;
pub const TYPE_SPIKE_ULTRASONIC: u8 = 62;
pub const TYPE_SPIKE_FORCE: u8 = 63;
pub const TYPE_SPIKE_M_MOTOR: u8 = 48;
pub const TYPE_SPIKE_L_MOTOR: u8 = 49;
pub const TYPE_SPIKE_S_MOTOR: u8 = 65;
pub const TYPE_TECHNIC_M_ANGULAR: u8 = 75;
pub const TYPE_TECHNIC_L_ANGULAR: u8 = 76;

// Color sensor modes
pub const MODE_COLOR: u8 = 0;   // 1× i8 — discrete color ID
pub const MODE_REFLT: u8 = 1;   // 1× i8 — reflected light %
pub const MODE_AMBI: u8 = 2;    // 1× i8 — ambient light %
pub const MODE_LIGHT: u8 = 3;   // 3× i8 — write: LED control
pub const MODE_RREFL: u8 = 4;   // 2× i16 — raw reflected
pub const MODE_RGB_I: u8 = 5;   // 4× i16 — raw RGBI
pub const MODE_HSV: u8 = 6;     // 3× i16 — HSV
pub const MODE_SHSV: u8 = 7;    // 4× i16 — SHSV

// Ultrasonic sensor modes (type 62)
pub const MODE_US_DISTL: u8 = 0;   // 1× i16 — distance long (mm)
pub const MODE_US_DISTS: u8 = 1;   // 1× i16 — distance short (mm)
pub const MODE_US_SINGL: u8 = 2;   // 1× i16 — single shot (mm)
pub const MODE_US_LISTN: u8 = 3;   // 1× i8  — listen (boolean)
pub const MODE_US_TRAW: u8 = 4;    // 1× i32 — raw time (µs)
pub const MODE_US_LIGHT: u8 = 5;   // 4× i8  — LED brightness (write)

// Motor modes (relative motors: M/L/S Motor, type 48/49/65)
pub const MODE_MOTOR_POWER: u8 = 0;  // 1× i8 — power %
pub const MODE_MOTOR_SPEED: u8 = 1;  // 1× i8 — speed %
pub const MODE_MOTOR_POS: u8 = 2;    // 1× i32 — cumulative degrees

// Motor modes (absolute/angular motors: Technic M/L Angular, type 75/76)
pub const MODE_ABS_MOTOR_POWER: u8 = 0;  // 1× i8 — power %
pub const MODE_ABS_MOTOR_SPEED: u8 = 1;  // 1× i8 — speed %
pub const MODE_ABS_MOTOR_POS: u8 = 2;    // 1× i32 — cumulative degrees
pub const MODE_ABS_MOTOR_APOS: u8 = 3;   // 1× i16 — absolute angle (decideg)
pub const MODE_ABS_MOTOR_CALIB: u8 = 4;  // combo mode (pos+apos+speed)
// ── Public protocol constants (async sync in main.rs) ──
pub const TYPE_HEADER: u8 = LUMP_MSG_TYPE_CMD | LUMP_CMD_TYPE; // 0x40
pub const SYS_ACK: u8 = LUMP_SYS_ACK; // 0x04
// ── Port hardware description ──
pub struct SensorPort {
    pub uart_base: u32,
    pub apb_enr: u32,       // RCC_APBxENR offset
    pub apb_bit: u32,       // bit in APBxENR
    pub apb_clk: u32,       // APB clock in Hz (for baud rate calc)
    pub tx_port: u32,
    pub tx_pin: u32,
    pub rx_port: u32,
    pub rx_pin: u32,
    pub tx_af: u32,
    pub rx_af: u32,
    pub buf_port: u32,      // buffer enable pin (active LOW)
    pub buf_pin: u32,
    pub id1_port: u32,      // ID1 sense pin (low = device present)
    pub id1_pin: u32,
}

pub const SENSOR_PORT_A: SensorPort = SensorPort {
    uart_base: 0x4000_7800, // UART7
    apb_enr: RCC_APB1ENR, apb_bit: 30, apb_clk: 48_000_000,
    tx_port: pins::GPIOE, tx_pin: 8,
    rx_port: pins::GPIOE, rx_pin: 7,
    tx_af: 8, rx_af: 8,
    buf_port: pins::GPIOA, buf_pin: 10,
    id1_port: pins::GPIOD, id1_pin: 7,  // PD7
};

pub const SENSOR_PORT_B: SensorPort = SensorPort {
    uart_base: 0x4000_4C00, // UART4
    apb_enr: RCC_APB1ENR, apb_bit: 19, apb_clk: 48_000_000,
    tx_port: pins::GPIOD, tx_pin: 1,
    rx_port: pins::GPIOD, rx_pin: 0,
    tx_af: 11, rx_af: 11,
    buf_port: pins::GPIOA, buf_pin: 8,
    id1_port: pins::GPIOD, id1_pin: 9,  // PD9
};

pub const SENSOR_PORT_C: SensorPort = SensorPort {
    uart_base: 0x4000_7C00, // UART8
    apb_enr: RCC_APB1ENR, apb_bit: 31, apb_clk: 48_000_000,
    tx_port: pins::GPIOE, tx_pin: 1,
    rx_port: pins::GPIOE, rx_pin: 0,
    tx_af: 8, rx_af: 8,
    buf_port: pins::GPIOE, buf_pin: 5,
    id1_port: pins::GPIOD, id1_pin: 11, // PD11
};

pub const SENSOR_PORT_D: SensorPort = SensorPort {
    uart_base: 0x4000_5000, // UART5
    apb_enr: RCC_APB1ENR, apb_bit: 20, apb_clk: 48_000_000,
    tx_port: pins::GPIOC, tx_pin: 12,
    rx_port: pins::GPIOD, rx_pin: 2,
    tx_af: 8, rx_af: 8,
    buf_port: pins::GPIOB, buf_pin: 2,
    id1_port: pins::GPIOC, id1_pin: 15, // PC15
};

pub const SENSOR_PORT_E: SensorPort = SensorPort {
    uart_base: 0x4001_1C00, // UART10
    apb_enr: RCC_APB2ENR, apb_bit: 7, apb_clk: 96_000_000,
    tx_port: pins::GPIOE, tx_pin: 3,
    rx_port: pins::GPIOE, rx_pin: 2,
    tx_af: 11, rx_af: 11,
    buf_port: pins::GPIOB, buf_pin: 5,
    id1_port: pins::GPIOC, id1_pin: 13, // PC13
};

pub const SENSOR_PORT_F: SensorPort = SensorPort {
    uart_base: 0x4001_1800, // UART9
    apb_enr: RCC_APB2ENR, apb_bit: 6, apb_clk: 96_000_000,
    tx_port: pins::GPIOD, tx_pin: 15,
    rx_port: pins::GPIOD, rx_pin: 14,
    tx_af: 11, rx_af: 11,
    buf_port: pins::GPIOC, buf_pin: 5,
    id1_port: pins::GPIOC, id1_pin: 11, // PC11
};

pub const PORTS: [&SensorPort; 6] = [
    &SENSOR_PORT_A, &SENSOR_PORT_B, &SENSOR_PORT_C,
    &SENSOR_PORT_D, &SENSOR_PORT_E, &SENSOR_PORT_F,
];

/// Convert port letter to index (0–5).
pub fn port_index(ch: u8) -> Option<usize> {
    match ch {
        b'a'..=b'f' => Some((ch - b'a') as usize),
        b'A'..=b'F' => Some((ch - b'A') as usize),
        _ => None,
    }
}

/// Look up port index (0–5) from a SensorPort reference by matching uart_base.
fn port_index_from_sp(sp: &SensorPort) -> Option<usize> {
    PORTS.iter().position(|p| p.uart_base == sp.uart_base)
}

// ── Sensor state (one instance per active port) ──

/// LUMP data buffer — max 32 bytes per LUMP spec.
pub const MAX_MSG: usize = 35; // header + 32 data + checksum + margin

/// Sensor connection status.
#[derive(Clone, Copy, PartialEq)]
pub enum Status {
    None,
    Syncing,
    Data,
    Error,
}

/// Per-port sensor state.
#[derive(Clone, Copy)]
pub struct SensorState {
    pub status: Status,
    pub type_id: u8,
    pub mode: u8,
    pub num_modes: u8,
    pub new_baud: u32,
    /// Latest binary data from the device (up to 32 bytes).
    pub data: [u8; 32],
    pub data_len: usize,
    /// Whether we received data since last keepalive.
    pub data_received: bool,
    /// Monotonically increasing counter, bumped on each new DATA message.
    pub data_seq: u32,
    /// Timestamp of last keepalive sent (SysTick ms).
    pub last_keepalive: u32,
    /// Which port index (0–5) this state belongs to.
    pub port_idx: u8,
    /// Debug: step at which sync failed (for diagnostics).
    pub debug_step: u8,
    /// Debug: first byte received during sync (0xFF if none).
    pub debug_byte: u8,
    /// Debug: first 8 bytes received during TYPE search.
    pub debug_bytes: [u8; 8],
    pub debug_bytes_len: u8,
    // ── lump_poll_data diagnostics ──
    pub diag_polls: u32,
    pub diag_sys: u32,
    pub diag_bad_size: u32,
    pub diag_bad_chk: u32,
    pub diag_data_msgs: u32,
    /// First non-SYS header byte seen in poll (0 = none yet).
    pub diag_first_hdr: u8,
    /// Last 8 raw DATA message headers (ring, newest at [diag_hdr_idx-1]).
    pub diag_last_hdrs: [u8; 8],
    pub diag_hdr_idx: u8,
    /// CMD-type messages seen since last reset.
    pub diag_cmd_msgs: u32,
    /// Last raw DATA message (header + payload + checksum), up to 12 bytes.
    pub diag_last_raw: [u8; 12],
    pub diag_last_raw_len: u8,
    /// EXT_MODE value from last CMD EXT_MODE message (Pybricks rx_ext_mode).
    pub ext_mode: u8,
    /// ORed FLAGS0 from INFO_NAME messages (NEEDS_SUPPLY_PIN1/PIN2 etc.).
    pub capabilities: u8,
}

impl SensorState {
    pub const fn new() -> Self {
        Self {
            status: Status::None,
            type_id: 0,
            mode: 0,
            num_modes: 1,
            new_baud: LUMP_SPEED_LPF2,
            data: [0; 32],
            data_len: 0,
            data_received: false,
            data_seq: 0,
            last_keepalive: 0,
            port_idx: 0,
            debug_step: 0,
            debug_byte: 0xFF,
            debug_bytes: [0; 8],
            debug_bytes_len: 0,
            diag_polls: 0,
            diag_sys: 0,
            diag_bad_size: 0,
            diag_bad_chk: 0,
            diag_data_msgs: 0,
            diag_first_hdr: 0,
            diag_last_hdrs: [0; 8],
            diag_hdr_idx: 0,
            diag_cmd_msgs: 0,
            diag_last_raw: [0; 12],
            diag_last_raw_len: 0,
            ext_mode: 0,
            capabilities: 0,
        }
    }

    /// Get RGBI values (mode 5) from data buffer.
    /// Returns (red, green, blue, intensity) as raw u16 values.
    pub fn rgbi(&self) -> (u16, u16, u16, u16) {
        if self.data_len >= 8 {
            let r = u16::from_le_bytes([self.data[0], self.data[1]]);
            let g = u16::from_le_bytes([self.data[2], self.data[3]]);
            let b = u16::from_le_bytes([self.data[4], self.data[5]]);
            let i = u16::from_le_bytes([self.data[6], self.data[7]]);
            (r, g, b, i)
        } else {
            (0, 0, 0, 0)
        }
    }

    /// Get ultrasonic distance in mm (modes 0, 1, 2).
    /// Returns distance as i16 (0–2000mm typical, -1 if no data).
    pub fn distance_mm(&self) -> i16 {
        if self.data_len >= 2 {
            i16::from_le_bytes([self.data[0], self.data[1]])
        } else {
            -1
        }
    }

    /// Get HSV values (mode 6) from data buffer.
    pub fn hsv(&self) -> (u16, u16, u16) {
        if self.data_len >= 6 {
            let h = u16::from_le_bytes([self.data[0], self.data[1]]);
            let s = u16::from_le_bytes([self.data[2], self.data[3]]);
            let v = u16::from_le_bytes([self.data[4], self.data[5]]);
            (h, s, v)
        } else {
            (0, 0, 0)
        }
    }

    /// Get single-byte value (modes 0, 1, 2).
    pub fn value_i8(&self) -> i8 {
        if self.data_len >= 1 {
            self.data[0] as i8
        } else {
            -1
        }
    }

    /// Returns true if this device is an ultrasonic sensor.
    pub fn is_ultrasonic(&self) -> bool {
        self.type_id == TYPE_SPIKE_ULTRASONIC
    }

    pub fn type_name(&self) -> &'static str {
        match self.type_id {
            TYPE_SPIKE_COLOR_SENSOR => "Color Sensor",
            TYPE_SPIKE_ULTRASONIC => "Ultrasonic",
            TYPE_SPIKE_FORCE => "Force Sensor",
            TYPE_SPIKE_M_MOTOR => "M Motor",
            TYPE_SPIKE_L_MOTOR => "L Motor",
            TYPE_SPIKE_S_MOTOR => "S Motor",
            TYPE_TECHNIC_M_ANGULAR => "Technic M Angular",
            TYPE_TECHNIC_L_ANGULAR => "Technic L Angular",
            _ => "Unknown",
        }
    }

    /// Returns true if this device is a motor (relative or absolute).
    pub fn is_motor(&self) -> bool {
        matches!(self.type_id,
            TYPE_SPIKE_M_MOTOR | TYPE_SPIKE_L_MOTOR | TYPE_SPIKE_S_MOTOR |
            TYPE_TECHNIC_M_ANGULAR | TYPE_TECHNIC_L_ANGULAR)
    }

    /// Returns true if this device is an absolute (angular) motor.
    pub fn is_abs_motor(&self) -> bool {
        matches!(self.type_id, TYPE_TECHNIC_M_ANGULAR | TYPE_TECHNIC_L_ANGULAR)
    }

    /// Get motor position in degrees from the data buffer.
    /// Works for mode 2 (POS) on both relative and absolute motors.
    /// Returns cumulative degrees (i32).
    pub fn motor_pos_degrees(&self) -> i32 {
        if self.data_len >= 4 {
            i32::from_le_bytes([self.data[0], self.data[1], self.data[2], self.data[3]])
        } else {
            0
        }
    }

    /// Get absolute motor angle in decidegrees from CALIB mode data.
    /// Only valid for absolute motors in mode 4 (CALIB).
    /// Returns (cumulative_degrees, absolute_decideg, speed_pct).
    pub fn motor_calib_data(&self) -> (i32, i16, i8) {
        if self.data_len >= 7 {
            let pos = i32::from_le_bytes([self.data[0], self.data[1], self.data[2], self.data[3]]);
            let apos = i16::from_le_bytes([self.data[4], self.data[5]]);
            let speed = self.data[6] as i8;
            (pos, apos, speed)
        } else if self.data_len >= 4 {
            let pos = i32::from_le_bytes([self.data[0], self.data[1], self.data[2], self.data[3]]);
            (pos, 0, 0)
        } else {
            (0, 0, 0)
        }
    }
}

// ── Per-port async state machine ──
// Each sensor port runs through these states.  Every state boundary
// is an `await` point so the RTIC executor can schedule other tasks.
// After each `.await` the state machine checks readiness/abort and
// transitions accordingly.

/// LUMP connection state machine — one per port.
#[derive(Clone, Copy, PartialEq)]
pub enum LumpState {
    /// Port idle / not started.
    Idle,
    /// Disconnect pins, wait for sensor LUMP watchdog reset (≥500 ms).
    Disconnect,
    /// Re-enable buffer, second disconnect pulse, wait 1500 ms.
    DisconnectWait,
    /// Start UART at 2400 baud, enable buffer, flush.
    SyncInit,
    /// Scanning for TYPE message byte at 2400 baud.
    SyncType,
    /// Reading INFO messages until device sends ACK.
    SyncInfo,
    /// Send ACK, switch baud to negotiated speed.
    SyncAck,
    /// SELECT target mode, then wait for stale-data delay.
    ModeSelect,
    /// Waiting for stale data to flush after mode switch.
    StaleWait,
    /// Normal operation — poll data + keepalive.
    Data,
    /// Data watchdog fired (no DATA received in time) → will re-sync.
    WatchdogErr,
    /// Fast re-probe — send keepalive at 115200, check for DATA.
    FastProbe,
    /// Sensor declared dead — clean up, optionally retry full sync.
    Dead,
}

/// Stale-data delay after mode switch (ms).
/// Ultrasonic needs 50 ms; color sensor 30 ms.  We use 50 ms for all.
pub const STALE_DATA_DELAY_MS: u32 = 50;

/// Data watchdog: if no DATA received within this many keepalive cycles
/// (each ~100 ms), declare sensor dead.  2 cycles = 200 ms.
pub const DATA_WATCHDOG_CYCLES: u8 = 2;

/// Return the default mode for a known device type.
pub fn default_mode_for_type(type_id: u8) -> u8 {
    match type_id {
        TYPE_SPIKE_COLOR_SENSOR => MODE_RGB_I,
        TYPE_SPIKE_ULTRASONIC => MODE_US_DISTL,
        TYPE_SPIKE_M_MOTOR | TYPE_SPIKE_L_MOTOR | TYPE_SPIKE_S_MOTOR => MODE_MOTOR_POS,
        TYPE_TECHNIC_M_ANGULAR | TYPE_TECHNIC_L_ANGULAR => MODE_MOTOR_POS,
        _ => 0,
    }
}

// ── Low-level UART helpers ──

/// Configure a GPIO pin as alternate function.
unsafe fn pin_af(port: u32, pin: u32, af: u32) {
    reg_modify(port, pins::MODER, 3 << (pin * 2), 2 << (pin * 2));
    reg_modify(port, pins::OSPEEDR, 3 << (pin * 2), 2 << (pin * 2)); // high speed
    reg_modify(port, pins::PUPDR, 3 << (pin * 2), 0); // no pull
    if pin < 8 {
        let shift = pin * 4;
        reg_modify(port, pins::AFRL, 0xF << shift, af << shift);
    } else {
        let shift = (pin - 8) * 4;
        reg_modify(port, pins::AFRH, 0xF << shift, af << shift);
    }
}

/// Configure a GPIO pin as floating input (MODER=00, no pull).
unsafe fn pin_input(port: u32, pin: u32) {
    reg_modify(port, pins::MODER, 3 << (pin * 2), 0);
    reg_modify(port, pins::PUPDR, 3 << (pin * 2), 0); // no pull
}

/// Configure a GPIO pin as push-pull output.
unsafe fn pin_output(port: u32, pin: u32) {
    reg_modify(port, pins::MODER, 3 << (pin * 2), 1 << (pin * 2));
    reg_modify(port, pins::OSPEEDR, 3 << (pin * 2), 2 << (pin * 2)); // high speed
}

unsafe fn pin_set(port: u32, pin: u32) {
    reg_write(port, pins::BSRR, 1 << pin);
}

unsafe fn pin_clear(port: u32, pin: u32) {
    reg_write(port, pins::BSRR, 1 << (pin + 16));
}

/// Disconnect sensor: TX/RX → input, buffer HIGH (disabled), UART off.
/// After calling this, wait ≥ 500 ms for the sensor's LUMP watchdog
/// to fire and reset the state machine.
pub unsafe fn disconnect(sp: &SensorPort) {
    reg_write(sp.uart_base, CR1, 0);
    pin_input(sp.tx_port, sp.tx_pin);
    pin_input(sp.rx_port, sp.rx_pin);
    pin_output(sp.buf_port, sp.buf_pin);
    pin_set(sp.buf_port, sp.buf_pin);
}

/// Check if a device is physically present on a port by reading the ID1 pin.
/// ID1 is pulled low when a LEGO device is plugged in.
/// Call `init_id1_pin` once before using this.
pub fn device_present(sp: &SensorPort) -> bool {
    unsafe { (reg_read(sp.id1_port, pins::IDR) & (1 << sp.id1_pin)) == 0 }
}

/// Configure ID1 pin as input with pull-up (high = nothing, low = device).
pub unsafe fn init_id1_pin(sp: &SensorPort) {
    pin_input(sp.id1_port, sp.id1_pin);
    // Pull-up so open pin reads high (no device)
    reg_modify(sp.id1_port, pins::PUPDR, 3 << (sp.id1_pin * 2), 1 << (sp.id1_pin * 2));
}

/// Enable the level-shifting buffer (active LOW).
pub unsafe fn buffer_enable(sp: &SensorPort) {
    pin_output(sp.buf_port, sp.buf_pin);
    pin_clear(sp.buf_port, sp.buf_pin);
}

/// Initialize UART hardware for a sensor port.
/// Does NOT enable RXNE interrupt or the buffer — caller controls those.
pub unsafe fn uart_init(sp: &SensorPort, baud: u32) {
    // Enable GPIO clocks for all ports used (A, B, C, D, E)
    reg_modify(RCC, RCC_AHB1ENR, 0,
        (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4));

    // Enable UART clock
    reg_modify(RCC, sp.apb_enr, 0, 1 << sp.apb_bit);
    let _ = reg_read(RCC, sp.apb_enr); // wait for clock to settle

    // Configure TX/RX pins as alternate function
    pin_af(sp.tx_port, sp.tx_pin, sp.tx_af);
    pin_af(sp.rx_port, sp.rx_pin, sp.rx_af);

    // Configure UART: disable first
    reg_write(sp.uart_base, CR1, 0);

    // Set baud rate: BRR = fCLK / baud
    let brr = (sp.apb_clk + baud / 2) / baud;
    reg_write(sp.uart_base, BRR, brr);

    // No hardware flow control
    reg_write(sp.uart_base, CR3, 0);

    // Enable UART with TX and RX (no RXNEIE — polling mode for sync)
    reg_write(sp.uart_base, CR1, CR1_UE | CR1_TE | CR1_RE);
}

/// Change baud rate on an already-initialized UART.
pub unsafe fn uart_set_baud(sp: &SensorPort, baud: u32) {
    // Disable UART completely
    reg_write(sp.uart_base, CR1, 0);
    // Set new baud
    let brr = (sp.apb_clk + baud / 2) / baud;
    reg_write(sp.uart_base, BRR, brr);
    // Drain any stale bytes from the hardware
    while reg_read(sp.uart_base, SR) & SR_RXNE != 0 {
        let _ = reg_read(sp.uart_base, DR);
    }
    // Re-enable (preserve RXNEIE if it was on)
    reg_write(sp.uart_base, CR1, CR1_UE | CR1_TE | CR1_RE);
}

/// Flush the UART RX: drain hardware FIFO.
pub unsafe fn uart_flush(sp: &SensorPort) {
    // Drain hardware
    while reg_read(sp.uart_base, SR) & SR_RXNE != 0 {
        let _ = reg_read(sp.uart_base, DR);
    }
    // Clear overrun flag by reading SR then DR
    let _ = reg_read(sp.uart_base, SR);
    let _ = reg_read(sp.uart_base, DR);
}

/// Send one byte (blocking, with short timeout).
unsafe fn uart_tx_byte(sp: &SensorPort, byte: u8) {
    // Wait for TXE
    let mut timeout = 100_000u32;
    while reg_read(sp.uart_base, SR) & SR_TXE == 0 {
        timeout -= 1;
        if timeout == 0 { return; }
    }
    reg_write(sp.uart_base, DR, byte as u32);
}

/// Send a byte slice.
pub unsafe fn uart_tx(sp: &SensorPort, data: &[u8]) {
    for &b in data {
        uart_tx_byte(sp, b);
    }
    // Wait for transmission complete
    let mut timeout = 200_000u32;
    while reg_read(sp.uart_base, SR) & (1 << 6) == 0 { // TC bit
        timeout -= 1;
        if timeout == 0 { return; }
    }
}

/// Enable RXNE interrupt for ISR-driven reception.
/// Flushes hardware FIFO and ring buffer, then enables RXNEIE.
pub unsafe fn uart_start_irq_rx(sp: &SensorPort) {
    // Drain any stale bytes from hardware
    while reg_read(sp.uart_base, SR) & SR_RXNE != 0 {
        let _ = reg_read(sp.uart_base, DR);
    }
    if let Some(idx) = port_index_from_sp(sp) {
        rx_flush(idx);
    }
    reg_modify(sp.uart_base, CR1, 0, CR1_RXNEIE);
}

// ── LUMP protocol helpers ──

/// Compute expected message size from the header byte.
pub fn msg_size(header: u8) -> usize {
    if header & LUMP_MSG_TYPE_MASK == LUMP_MSG_TYPE_SYS {
        return 1;
    }
    let size_code = (header & LUMP_MSG_SIZE_MASK) >> 3;
    let payload = 1usize << size_code; // 1, 2, 4, 8, 16, 32
    let extra = if header & LUMP_MSG_TYPE_MASK == LUMP_MSG_TYPE_INFO { 1 } else { 0 };
    payload + 2 + extra // payload + header + checksum + info_cmd_byte
}

/// Verify checksum of a LUMP message. Returns true if valid.
pub fn check_msg(buf: &[u8], len: usize) -> bool {
    if len < 2 { return false; }
    let mut chk: u8 = 0xFF;
    for i in 0..len - 1 {
        chk ^= buf[i];
    }
    chk == buf[len - 1]
}

/// Build a LUMP TX message. Returns the total message length.
fn build_msg(buf: &mut [u8; MAX_MSG], msg_type: u8, cmd: u8, data: &[u8]) -> usize {
    let len = data.len();
    let size_code = match len {
        0..=1 => LUMP_MSG_SIZE_1,
        2 => 1 << 3,
        3..=4 => 2 << 3,
        5..=8 => 3 << 3,
        9..=16 => 4 << 3,
        _ => 5 << 3,
    };
    let padded = match len {
        0..=1 => 1,
        2 => 2,
        3..=4 => 4,
        5..=8 => 8,
        9..=16 => 16,
        _ => 32,
    };

    let header = (msg_type & LUMP_MSG_TYPE_MASK) | (size_code & LUMP_MSG_SIZE_MASK) | (cmd & LUMP_MSG_CMD_MASK);
    buf[0] = header;
    let mut chk: u8 = 0xFF ^ header;
    for i in 0..padded {
        let b = if i < len { data[i] } else { 0 };
        buf[1 + i] = b;
        chk ^= b;
    }
    buf[1 + padded] = chk;
    2 + padded
}

// ── LUMP sync ──
// Sync is now fully async in main.rs sensor_poll task.
// The ISR ring buffer is enabled from the start; the async task
// uses Mono::delay() for all waits — zero busy-wait polling.

/// Send a mode SELECT command.
pub unsafe fn lump_select_mode(sp: &SensorPort, mode: u8) {
    let mut buf = [0u8; MAX_MSG];

    // For extended modes (> 7), send EXT_MODE first
    if mode > 7 {
        let ext_hdr = LUMP_MSG_TYPE_CMD | LUMP_MSG_SIZE_1 | LUMP_CMD_EXT_MODE;
        let ext_data: u8 = 8;
        let ext_chk = 0xFF ^ ext_hdr ^ ext_data;
        let pkt = [ext_hdr, ext_data, ext_chk];
        uart_tx(sp, &pkt);
    }

    let len = build_msg(&mut buf, LUMP_MSG_TYPE_CMD, LUMP_CMD_SELECT, &[mode & 0x07]);
    uart_tx(sp, &buf[..len]);
}

/// Send SPEED(115200) command — tells the device we speak high-speed.
pub unsafe fn send_speed_cmd(sp: &SensorPort) {
    let mut buf = [0u8; MAX_MSG];
    let speed_bytes = LUMP_SPEED_LPF2.to_le_bytes();
    let len = build_msg(&mut buf, LUMP_MSG_TYPE_CMD, LUMP_CMD_SPEED, &speed_bytes);
    uart_tx(sp, &buf[..len]);
}

/// Send NACK keepalive to keep the sensor streaming.
pub unsafe fn lump_keepalive(sp: &SensorPort) {
    uart_tx(sp, &[LUMP_SYS_NACK]);
}

/// Process incoming data from ISR ring buffer. Non-blocking.
/// All bytes come from the RXNE interrupt handler — no direct UART access.
///
/// Uses peek-before-consume strategy: we never advance the ring buffer
/// tail until we have verified the complete message (header + payload +
/// checksum) is present AND valid. This prevents stream desynchronisation
/// caused by partially consuming an incomplete message.
///
/// Returns true if new data was received.
pub fn lump_poll_data(state: &mut SensorState) -> bool {
    let port = state.port_idx as usize;
    let mut got_data = false;
    let mut buf = [0u8; MAX_MSG];

    state.diag_polls = state.diag_polls.wrapping_add(1);

    let mut iterations = 0;
    while iterations < 20 {
        iterations += 1;

        let avail = rx_available(port);
        if avail == 0 {
            break;
        }

        let hdr_byte = rx_peek_at(port, 0);
        let msg_type = hdr_byte & LUMP_MSG_TYPE_MASK;

        // SYS messages are 1 byte — skip them (ACK/NACK keepalives)
        if msg_type == LUMP_MSG_TYPE_SYS {
            state.diag_sys = state.diag_sys.wrapping_add(1);
            rx_skip(port, 1);
            continue;
        }

        // Record first non-SYS header for diagnostics
        if state.diag_first_hdr == 0 {
            state.diag_first_hdr = hdr_byte;
        }

        let msize = msg_size(hdr_byte);
        if msize < 2 || msize > MAX_MSG {
            // Invalid header — skip this byte and try to resync
            state.diag_bad_size = state.diag_bad_size.wrapping_add(1);
            rx_skip(port, 1);
            continue;
        }

        // Wait for the full message to arrive in the ring buffer
        if avail < msize {
            break; // come back next poll cycle
        }

        // Peek all bytes into local buffer (still not consumed)
        for i in 0..msize {
            buf[i] = rx_peek_at(port, i);
        }

        // Validate checksum before consuming anything
        if !check_msg(&buf, msize) {
            // Bad checksum — skip just the header byte and try to resync
            state.diag_bad_chk = state.diag_bad_chk.wrapping_add(1);
            rx_skip(port, 1);
            continue;
        }

        // Message is complete and valid — consume it
        rx_skip(port, msize);

        // Record header in ring for diagnostics
        let hi = (state.diag_hdr_idx % 8) as usize;
        state.diag_last_hdrs[hi] = hdr_byte;
        state.diag_hdr_idx = state.diag_hdr_idx.wrapping_add(1);

        // Handle DATA messages
        if msg_type == LUMP_MSG_TYPE_DATA {
            // Actual mode = ext_mode * 8 + header_mode (like Pybricks rx_ext_mode)
            let hdr_mode = hdr_byte & LUMP_MSG_CMD_MASK;
            let actual_mode = (state.ext_mode as u8).wrapping_mul(8).wrapping_add(hdr_mode);
            let data_len = msize - 2;
            state.diag_data_msgs = state.diag_data_msgs.wrapping_add(1);

            // Store last raw message for diagnostics
            let raw_len = if msize > 12 { 12 } else { msize };
            state.diag_last_raw[..raw_len].copy_from_slice(&buf[..raw_len]);
            state.diag_last_raw_len = raw_len as u8;

            if data_len <= 32 {
                state.data[..data_len].copy_from_slice(&buf[1..1 + data_len]);
                state.data_len = data_len;
                state.mode = actual_mode;
                state.data_received = true;
                state.data_seq = state.data_seq.wrapping_add(1);
                got_data = true;
            }
            // Reset ext_mode after consuming it (applies to one DATA only)
            state.ext_mode = 0;
        } else if msg_type == LUMP_MSG_TYPE_CMD {
            state.diag_cmd_msgs = state.diag_cmd_msgs.wrapping_add(1);
            // Track EXT_MODE: cmd=6, 1 byte payload
            let cmd = hdr_byte & LUMP_MSG_CMD_MASK;
            if cmd == LUMP_CMD_EXT_MODE && msize >= 3 {
                state.ext_mode = buf[1];
            }
        }
    }

    got_data
}

/// Disable the UART and buffer for a port (cleanup).
pub unsafe fn uart_deinit(sp: &SensorPort) {
    // Disable UART (clears RXNEIE too)
    reg_write(sp.uart_base, CR1, 0);
    // Disable buffer (HIGH = disabled)
    pin_set(sp.buf_port, sp.buf_pin);
    // Flush ring buffer for this port
    if let Some(idx) = port_index_from_sp(sp) {
        rx_flush(idx);
    }
}

/// Raw byte sniff: power-cycle port, enable UART at given baud,
/// wait `ms` milliseconds, return up to 64 raw bytes received.
/// Used for debugging sensor communication.
pub unsafe fn raw_sniff(sp: &SensorPort, baud: u32, ms: u32, out: &mut [u8; 64]) -> usize {
    // Disable buffer first
    reg_modify(RCC, RCC_AHB1ENR, 0,
        (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4));
    pin_output(sp.buf_port, sp.buf_pin);
    pin_set(sp.buf_port, sp.buf_pin); // buffer HIGH = disabled
    reg_write(sp.uart_base, CR1, 0);  // disable UART
    crate::power::delay_ms(200);

    // Setup UART but don't enable yet
    reg_modify(RCC, sp.apb_enr, 0, 1 << sp.apb_bit);
    let _ = reg_read(RCC, sp.apb_enr);
    pin_af(sp.tx_port, sp.tx_pin, sp.tx_af);
    pin_af(sp.rx_port, sp.rx_pin, sp.rx_af);

    // Enable buffer
    pin_output(sp.buf_port, sp.buf_pin);
    pin_clear(sp.buf_port, sp.buf_pin); // buffer LOW = enabled
    crate::power::delay_ms(10);

    // Enable UART
    let brr = (sp.apb_clk + baud / 2) / baud;
    reg_write(sp.uart_base, BRR, brr);
    reg_write(sp.uart_base, CR3, 0);
    reg_write(sp.uart_base, CR1, CR1_UE | CR1_TE | CR1_RE);
    uart_flush(sp);

    // Optionally send SPEED command at 115200 if we're sniffing at that rate
    if baud == LUMP_SPEED_LPF2 {
        let mut buf = [0u8; MAX_MSG];
        let speed_bytes = LUMP_SPEED_LPF2.to_le_bytes();
        let len = build_msg(&mut buf, LUMP_MSG_TYPE_CMD, LUMP_CMD_SPEED, &speed_bytes);
        uart_tx(sp, &buf[..len]);
        uart_flush(sp);
    }

    // Receive raw bytes
    let mut count = 0usize;
    let iters_per_ms = 10_000u32;
    let total_iters = ms.saturating_mul(iters_per_ms);
    let mut remaining = total_iters;

    while count < 64 && remaining > 0 {
        let sr = reg_read(sp.uart_base, SR);
        if sr & SR_ORE != 0 {
            let _ = reg_read(sp.uart_base, DR);
            remaining = remaining.wrapping_sub(1);
            continue;
        }
        if sr & SR_RXNE != 0 {
            out[count] = reg_read(sp.uart_base, DR) as u8;
            count += 1;
        }
        remaining = remaining.wrapping_sub(1);
    }

    // Cleanup
    reg_write(sp.uart_base, CR1, 0);
    pin_set(sp.buf_port, sp.buf_pin);

    count
}
