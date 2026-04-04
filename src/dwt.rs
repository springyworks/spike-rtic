//! Self-hosted DWT (Data Watchpoint and Trace) — hardware address watchpoints.
//!
//! The Cortex-M4 DWT has 4 comparators that can trigger on memory read, write,
//! or read/write to a specific address.  Normally these are set by an external
//! debugger via JTAG/SWD, but since the registers are memory-mapped, firmware
//! can configure them directly ("self-hosted debugging").
//!
//! Instead of halting the CPU (which needs a debugger to resume), we enable
//! the DebugMonitor exception so watchpoint hits fire an ISR we can catch.
//!
//! # Usage
//!   1. Call `init()` once at startup to enable cycle counter + DebugMonitor.
//!   2. Use `set_watchpoint(n, addr, function)` to arm a comparator.
//!   3. The `DebugMonitor` exception handler logs the hit.
//!   4. Use `clear_watchpoint(n)` or `clear_all()` to disarm.
//!
//! # Register map (Cortex-M4 DWT)
//!   - DWT_CTRL    0xE000_1000  — control (CYCCNTENA, NUMCOMP, etc.)
//!   - DWT_CYCCNT  0xE000_1004  — cycle counter
//!   - DWT_COMPn   0xE000_1020 + 16*n  — comparator value
//!   - DWT_MASKn   0xE000_1024 + 16*n  — address mask (ignore low bits)
//!   - DWT_FUNCn   0xE000_1028 + 16*n  — function (trigger type)
//!   - DWT_LAR     0xE000_1FB0  — lock access (write 0xC5ACCE55 to unlock)
//!
//! # DEMCR (0xE000_EDFC)
//!   - Bit 24: TRCENA   — global trace/DWT enable
//!   - Bit 16: MON_EN   — DebugMonitor exception enable
//!   - Bit 19: MON_REQ  — pend DebugMonitor (manual trigger)

use core::sync::atomic::{AtomicU32, Ordering};

// ── DWT base addresses ──
const DWT_BASE: u32 = 0xE000_1000;
const DWT_CTRL: *const u32 = 0xE000_1000 as *const u32;
const DWT_LAR: *mut u32 = 0xE000_1FB0 as *mut u32;

// Per-comparator register offsets from DWT_BASE
const fn comp_addr(n: u32) -> *mut u32 { (0xE000_1020 + 16 * n) as *mut u32 }
const fn mask_addr(n: u32) -> *mut u32 { (0xE000_1024 + 16 * n) as *mut u32 }
const fn func_addr(n: u32) -> *mut u32 { (0xE000_1028 + 16 * n) as *mut u32 }

// DEMCR
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;

// NVIC priority register for DebugMonitor (exception #12, IRQ -4)
// SCB->SHPR3 bits [7:0] = DebugMonitor priority
// Actually: SCB->SHPR[1] byte offset 0 = DebugMonitor
// DebugMonitor is exception 12 → SHPR index = (12-4) = 8 → SHPR[8] at 0xE000_ED18 + 8
// Alternatively: SHPR byte 8 is at 0xE000_ED20
// SHPR registers: 0xE000_ED18 (SHPR1), 0xE000_ED1C (SHPR2), 0xE000_ED20 (SHPR3)
// DebugMonitor = exception 12 → SHPR byte (12-4) = 8 → word at 0xE000_ED20, bits [7:0]
const SCB_SHPR3: *mut u32 = 0xE000_ED20 as *mut u32;

/// DWT comparator function codes (DWT_FUNCTIONx bits [3:0]).
/// Cortex-M4 TRM Table C1-15.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WatchFunc {
    /// Disabled (no watchpoint).
    Disabled   = 0b0000,
    /// Trigger on PC match (instruction fetch from address).
    PcMatch    = 0b0100,
    /// Trigger on data read from address.
    DataRead   = 0b0101,
    /// Trigger on data write to address.
    DataWrite  = 0b0110,
    /// Trigger on data read or write to address.
    DataRW     = 0b0111,
}

impl WatchFunc {
    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "off" | "disable" | "0" => Some(Self::Disabled),
            "pc" | "exec" | "x"    => Some(Self::PcMatch),
            "r" | "read"           => Some(Self::DataRead),
            "w" | "write"          => Some(Self::DataWrite),
            "rw" | "readwrite"     => Some(Self::DataRW),
            _                      => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Disabled  => "off",
            Self::PcMatch   => "pc",
            Self::DataRead  => "read",
            Self::DataWrite => "write",
            Self::DataRW    => "rw",
        }
    }
}

/// Number of DWT comparators available (read from DWT_CTRL[31:28]).
pub fn num_comparators() -> u8 {
    let ctrl = unsafe { core::ptr::read_volatile(DWT_CTRL) };
    ((ctrl >> 28) & 0xF) as u8
}

// ── Hit tracking ──
// When the DebugMonitor fires, we record which comparator matched and the PC.
static HIT_COUNT: [AtomicU32; 4] = [
    AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0),
];
static LAST_HIT_PC: [AtomicU32; 4] = [
    AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0),
];
/// Total DebugMonitor invocations (including spurious).
static DEBUGMON_COUNT: AtomicU32 = AtomicU32::new(0);

/// Initialize DWT for self-hosted watchpoints.
///
/// - Unlocks DWT (LAR write).
/// - Enables TRCENA in DEMCR (global trace enable).
/// - Enables MON_EN in DEMCR (DebugMonitor exception).
/// - Enables CYCCNTENA in DWT_CTRL (cycle counter, useful for timestamps).
/// - Sets DebugMonitor priority (lower than fault handlers, higher than tasks).
///
/// Safe to call multiple times.
pub unsafe fn init() {
    // 1. Unlock DWT (CoreSight LAR key)
    core::ptr::write_volatile(DWT_LAR, 0xC5AC_CE55);

    // 2. Enable TRCENA (bit 24) + MON_EN (bit 16) in DEMCR
    let demcr = core::ptr::read_volatile(DEMCR);
    core::ptr::write_volatile(DEMCR, demcr | (1 << 24) | (1 << 16));

    // 3. Enable cycle counter (DWT_CTRL bit 0 = CYCCNTENA)
    let ctrl = core::ptr::read_volatile(DWT_BASE as *mut u32);
    core::ptr::write_volatile(DWT_BASE as *mut u32, ctrl | 1);

    // 4. Set DebugMonitor priority.
    //    We want it above normal tasks (priority 0xF0) but below faults.
    //    Use priority 0x40 (NVIC priority 4) — same ballpark as MemManage.
    //    SHPR3 bits [7:0] = DebugMonitor priority.
    let shpr3 = core::ptr::read_volatile(SCB_SHPR3);
    core::ptr::write_volatile(SCB_SHPR3, (shpr3 & !0xFF) | 0x40);

    // 5. Clear all comparators
    for n in 0..4u32 {
        core::ptr::write_volatile(func_addr(n), 0);
        core::ptr::write_volatile(comp_addr(n), 0);
        core::ptr::write_volatile(mask_addr(n), 0);
    }
}

/// Arm a DWT watchpoint.
///
/// - `n`: comparator index (0–3)
/// - `addr`: the memory address to watch
/// - `mask`: address mask (0 = exact match, 1 = ignore bit 0, etc.)
/// - `func`: trigger type
///
/// Returns `true` if armed successfully.
pub unsafe fn set_watchpoint(n: u32, addr: u32, mask: u32, func: WatchFunc) -> bool {
    if n >= 4 { return false; }

    // Disable comparator first
    core::ptr::write_volatile(func_addr(n), 0);

    // Set address and mask
    core::ptr::write_volatile(comp_addr(n), addr);
    core::ptr::write_volatile(mask_addr(n), mask);

    // Reset hit counter for this comparator
    HIT_COUNT[n as usize].store(0, Ordering::Relaxed);
    LAST_HIT_PC[n as usize].store(0, Ordering::Relaxed);

    // Arm: write function code
    if func != WatchFunc::Disabled {
        core::ptr::write_volatile(func_addr(n), func as u32);
    }

    true
}

/// Disarm a single watchpoint.
pub unsafe fn clear_watchpoint(n: u32) {
    if n < 4 {
        core::ptr::write_volatile(func_addr(n), 0);
    }
}

/// Disarm all watchpoints.
pub unsafe fn clear_all() {
    for n in 0..4u32 {
        core::ptr::write_volatile(func_addr(n), 0);
    }
}

/// Read the current state of a comparator.
/// Returns (address, mask, function_raw, matched_flag).
pub fn read_watchpoint(n: u32) -> (u32, u32, u32, bool) {
    if n >= 4 { return (0, 0, 0, false); }
    unsafe {
        let addr = core::ptr::read_volatile(comp_addr(n) as *const u32);
        let mask = core::ptr::read_volatile(mask_addr(n) as *const u32);
        let func = core::ptr::read_volatile(func_addr(n) as *const u32);
        let matched = func & (1 << 24) != 0; // MATCHED bit
        (addr, mask, func & 0xF, matched)
    }
}

/// Get hit count and last PC for a comparator.
pub fn hit_info(n: usize) -> (u32, u32) {
    if n >= 4 { return (0, 0); }
    (
        HIT_COUNT[n].load(Ordering::Relaxed),
        LAST_HIT_PC[n].load(Ordering::Relaxed),
    )
}

/// Total DebugMonitor invocations.
pub fn debugmon_count() -> u32 {
    DEBUGMON_COUNT.load(Ordering::Relaxed)
}

/// DebugMonitor exception handler.
///
/// Called when a DWT watchpoint fires.  Scans all 4 comparators for the
/// MATCHED bit (DWT_FUNCTIONx bit 24, auto-cleared on read) and records
/// the hit.
///
/// The stacked PC from the exception frame shows which instruction triggered
/// the watchpoint.
#[allow(non_snake_case)]
pub unsafe extern "C" fn DebugMonitor_handler() {
    DEBUGMON_COUNT.fetch_add(1, Ordering::Relaxed);

    // Read stacked PC from exception frame.
    // DebugMonitor uses MSP (we're in Handler mode).
    let sp: u32;
    core::arch::asm!("mrs {}, MSP", out(reg) sp);
    // Exception frame: R0, R1, R2, R3, R12, LR, PC, xPSR
    let stacked_pc = core::ptr::read_volatile((sp + 24) as *const u32);

    // Check each comparator's MATCHED bit (cleared on read of FUNCTION reg)
    for n in 0..4u32 {
        let func = core::ptr::read_volatile(func_addr(n));
        if func & (1 << 24) != 0 {
            // This comparator fired
            HIT_COUNT[n as usize].fetch_add(1, Ordering::Relaxed);
            LAST_HIT_PC[n as usize].store(stacked_pc, Ordering::Relaxed);
        }
    }
}
