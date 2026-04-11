#![allow(dead_code)]
//! Flash Patch and Breakpoint (FPB) unit — hardware instruction breakpoints.
//!
//! The Cortex-M4 FPB provides hardware breakpoints that trigger the
//! DebugMonitor exception when the CPU fetches an instruction from a
//! matching address.  This gives us Z1 (hardware breakpoint) support
//! for the GDB RSP stub — no JTAG/SWD required.
//!
//! The STM32F413 FPB has:
//!   - 6 instruction (code) comparators   (NUMCODE in FP_CTRL[7:4])
//!   - 2 literal comparators              (NUMLIT  in FP_CTRL[11:8])
//!   - Revision 1 (v1): comparator format has REPLACE bits [31:30]
//!
//! We only use instruction comparators for breakpoints.
//!
//! # FPB register map
//!   - FP_CTRL    0xE000_2000  — control (ENABLE, KEY, NUMCODE, NUMLIT, REV)
//!   - FP_REMAP   0xE000_2004  — remap base (not used)
//!   - FP_COMPn   0xE000_2008 + 4*n  — comparator n
//!
//! # FP_CTRL bits
//!   - [0]    ENABLE — FPB globally enabled
//!   - [1]    KEY    — must be 1 for write to take effect
//!   - [7:4]  NUM_CODE — number of instruction comparators (read-only)
//!   - [11:8] NUM_LIT  — number of literal comparators (read-only)
//!   - [31:28] REV    — revision (0 = v1, 1 = v2)
//!
//! # FP_COMPn bits (revision 1 — FPBv1, Cortex-M3/M4)
//!   - [0]     ENABLE — this comparator enabled
//!   - [1]     reserved (0)
//!   - [28:2]  COMP   — bits [28:2] of the instruction address
//!   - [31:30] REPLACE — 00=remap, 01=bkpt on lower halfword,
//!     10=bkpt on upper halfword, 11=bkpt on both
//!
//! For breakpoints: we want REPLACE = 0b11 (breakpoint on word access)
//! which works for both Thumb-2 (32-bit) and Thumb (16-bit) instructions.
//!
//! # Important: address range
//!   FPBv1 can only match addresses in the code region 0x0000_0000 to
//!   0x1FFF_FFFF.  For SRAM2 demos at 0x2004_0000, the FPB cannot
//!   set breakpoints directly — the address is outside the code region.
//!   However, GDB can use software breakpoints (Z0) by patching
//!   memory with BKPT instructions if we report swbreak support.
//!
//!   For Flash-resident firmware code (0x0800_xxxx), FPB breakpoints
//!   work perfectly.
//!
//! # Usage
//!   1. `init()` — enable FPB + DebugMonitor
//!   2. `set_breakpoint(addr)` — arms a free comparator, returns slot
//!   3. `clear_breakpoint(addr)` — disarms by address match
//!   4. `clear_all()` — disarms all comparators
//!   5. DebugMonitor fires on instruction fetch → GDB gets T05

use core::sync::atomic::{AtomicU32, Ordering};

// ── FPB registers ──
const FP_CTRL: *mut u32 = 0xE000_2000 as *mut u32;
const FP_REMAP: *mut u32 = 0xE000_2004 as *mut u32;

/// Base address of comparator array.
const fn fp_comp_addr(n: u32) -> *mut u32 {
    (0xE000_2008 + 4 * n) as *mut u32
}

// DEMCR — shared with DWT; we just need to ensure MON_EN is set
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;

// ── Constants ──

/// KEY bit — must be 1 for writes to FP_CTRL to take effect.
const FP_CTRL_KEY: u32 = 1 << 1;
/// ENABLE bit — global FPB enable.
const FP_CTRL_ENABLE: u32 = 1 << 0;

/// FPBv1 REPLACE field: breakpoint on both halfwords of the word.
const REPLACE_BKPT_BOTH: u32 = 0b11 << 30;
/// FPBv1 REPLACE field: breakpoint on lower halfword.
const REPLACE_BKPT_LOWER: u32 = 0b01 << 30;
/// FPBv1 REPLACE field: breakpoint on upper halfword.
const REPLACE_BKPT_UPPER: u32 = 0b10 << 30;

/// Maximum instruction comparators we track.
const MAX_CODE_COMPS: usize = 8;

// ── Hit tracking ──
static BREAKPOINT_HIT_COUNT: AtomicU32 = AtomicU32::new(0);
static LAST_BREAKPOINT_PC: AtomicU32 = AtomicU32::new(0);

/// Number of instruction (code) comparators (from FP_CTRL[7:4]).
pub fn num_code_comparators() -> u32 {
    let ctrl = unsafe { core::ptr::read_volatile(FP_CTRL as *const u32) };
    // NUMCODE is split: bits [7:4] give the low 4 bits,
    // and for v1 on Cortex-M4, this is the full count (up to 8).
    (ctrl >> 4) & 0xF
}

/// FPB revision (0 = v1, 1 = v2).
pub fn revision() -> u32 {
    let ctrl = unsafe { core::ptr::read_volatile(FP_CTRL as *const u32) };
    (ctrl >> 28) & 0xF
}

/// Initialize the FPB unit.
///
/// - Enables FPB globally.
/// - Ensures DebugMonitor exception is enabled (DEMCR MON_EN).
/// - Disables all comparators.
///
/// Safe to call multiple times.  DWT `init()` should be called first
/// (it sets up TRCENA and DebugMonitor priority).
pub unsafe fn init() {
    // Ensure TRCENA (bit 24) + MON_EN (bit 16) in DEMCR
    let demcr = core::ptr::read_volatile(DEMCR);
    core::ptr::write_volatile(DEMCR, demcr | (1 << 24) | (1 << 16));

    // Disable all comparators first
    let num = num_code_comparators();
    for n in 0..num {
        core::ptr::write_volatile(fp_comp_addr(n), 0);
    }

    // Enable FPB globally (KEY + ENABLE)
    core::ptr::write_volatile(FP_CTRL, FP_CTRL_KEY | FP_CTRL_ENABLE);
}

/// Set a hardware breakpoint at the given address.
///
/// Finds a free instruction comparator and arms it.
/// Returns `Some(slot)` on success, `None` if all comparators are in use
/// or the address is outside the FPBv1 range (> 0x1FFF_FFFF).
pub unsafe fn set_breakpoint(addr: u32) -> Option<u32> {
    // FPBv1 can only match addresses 0x0000_0000 to 0x1FFF_FFFF
    if addr >= 0x2000_0000 {
        return None;
    }

    let num = num_code_comparators();

    // Check if already set at this address
    for n in 0..num {
        let comp = core::ptr::read_volatile(fp_comp_addr(n) as *const u32);
        if comp & 1 != 0 {
            let comp_addr = comp & 0x1FFF_FFFC; // bits [28:2] shifted
            if comp_addr == (addr & 0x1FFF_FFFC) {
                // Already set — return the existing slot
                return Some(n);
            }
        }
    }

    // Find a free comparator
    for n in 0..num {
        let comp = core::ptr::read_volatile(fp_comp_addr(n) as *const u32);
        if comp & 1 == 0 {
            // Free — arm it
            // FPBv1 format: REPLACE[31:30] | COMP[28:2] | ENABLE[0]
            // For Thumb code, bit[1] selects upper/lower halfword:
            //   addr bit[1] = 0 → REPLACE = 01 (lower halfword)
            //   addr bit[1] = 1 → REPLACE = 10 (upper halfword)
            let replace = if addr & 2 == 0 {
                REPLACE_BKPT_LOWER
            } else {
                REPLACE_BKPT_UPPER
            };
            let val = replace | (addr & 0x1FFF_FFFC) | 1;
            core::ptr::write_volatile(fp_comp_addr(n), val);
            return Some(n);
        }
    }

    None // all comparators in use
}

/// Remove a hardware breakpoint at the given address.
///
/// Returns `true` if a matching comparator was found and cleared.
pub unsafe fn clear_breakpoint(addr: u32) -> bool {
    let num = num_code_comparators();
    let target = addr & 0x1FFF_FFFC;

    for n in 0..num {
        let comp = core::ptr::read_volatile(fp_comp_addr(n) as *const u32);
        if comp & 1 != 0 {
            let comp_addr = comp & 0x1FFF_FFFC;
            if comp_addr == target {
                core::ptr::write_volatile(fp_comp_addr(n), 0);
                return true;
            }
        }
    }

    false
}

/// Disable all FPB comparators (but leave FPB globally enabled).
pub unsafe fn clear_all() {
    let num = num_code_comparators();
    for n in 0..num {
        core::ptr::write_volatile(fp_comp_addr(n), 0);
    }
}

/// Read the state of a comparator.
/// Returns `(address, enabled)`.
pub fn read_comparator(n: u32) -> (u32, bool) {
    let num = num_code_comparators();
    if n >= num {
        return (0, false);
    }
    let comp = unsafe { core::ptr::read_volatile(fp_comp_addr(n) as *const u32) };
    let enabled = comp & 1 != 0;
    let addr = comp & 0x1FFF_FFFC;
    // Reconstruct bit[1] from REPLACE field
    let replace = (comp >> 30) & 0x3;
    let bit1 = if replace == 0b10 { 2u32 } else { 0u32 };
    (addr | bit1, enabled)
}

/// Total breakpoint hits recorded.
pub fn breakpoint_hit_count() -> u32 {
    BREAKPOINT_HIT_COUNT.load(Ordering::Relaxed)
}

/// Last PC at which a breakpoint fired.
pub fn last_breakpoint_pc() -> u32 {
    LAST_BREAKPOINT_PC.load(Ordering::Relaxed)
}

/// Called from the DebugMonitor handler to record FPB breakpoint hits.
///
/// The DebugMonitor in dwt.rs should call this after checking DWT
/// comparators, to also handle FPB hits.
pub fn record_hit(stacked_pc: u32) {
    BREAKPOINT_HIT_COUNT.fetch_add(1, Ordering::Relaxed);
    LAST_BREAKPOINT_PC.store(stacked_pc, Ordering::Relaxed);
}

/// Check if any FPB comparator matched the given PC.
/// Used by the DebugMonitor handler to distinguish FPB hits from DWT hits.
pub fn check_hit(pc: u32) -> bool {
    let num = num_code_comparators();
    for n in 0..num {
        let (addr, enabled) = read_comparator(n);
        if enabled && (addr & 0x1FFF_FFFE) == (pc & 0x1FFF_FFFE) {
            return true;
        }
    }
    false
}
