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
//!   - Bit 17: MON_PEND — pend DebugMonitor exception
//!   - Bit 18: MON_STEP — single-step on DebugMonitor return
//!   - Bit 19: MON_REQ  — software semaphore (does NOT pend!)

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

// ── Saved register context from DebugMonitor exception ──
// The exception frame auto-stacks R0-R3, R12, LR, PC, xPSR.
// We manually save R4-R11 + SP in the handler.
// Layout: [R0..R12, SP, LR, PC, xPSR]  (17 words)
static SAVED_REGS: [AtomicU32; 17] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), // R0-R3
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), // R4-R7
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), // R8-R11
    AtomicU32::new(0), // R12
    AtomicU32::new(0), // SP (R13) — pre-exception SP
    AtomicU32::new(0), // LR (R14)
    AtomicU32::new(0), // PC (R15)
    AtomicU32::new(0), // xPSR
];
/// True when SAVED_REGS contains a valid snapshot from DebugMonitor.
static REGS_VALID: AtomicU32 = AtomicU32::new(0);

/// If non-zero, the DebugMon handler will set MON_STEP in DEMCR
/// before returning, causing a single-step.
static SINGLE_STEP_REQUEST: AtomicU32 = AtomicU32::new(0);

/// When non-zero, the DebugMonitor handler has halted (spin-waiting).
/// The RSP stub reads this to know the target is truly stopped.
static TARGET_HALTED: AtomicU32 = AtomicU32::new(0);

/// Set to non-zero by the RSP stub to tell the halted handler to resume.
static RESUME_REQUEST: AtomicU32 = AtomicU32::new(0);

/// Non-zero when GDB RSP is actively listening.  The DebugMonitor
/// handler checks this before halting — if no GDB is connected,
/// the handler just clears stale debug state and returns immediately
/// instead of spinning in a WFE loop that nobody will ever resume.
static GDB_ACTIVE: AtomicU32 = AtomicU32::new(0);

/// Read saved register `n` (0=R0 .. 15=PC, 16=xPSR).
pub fn saved_reg(n: usize) -> u32 {
    if n < 17 { SAVED_REGS[n].load(Ordering::Relaxed) } else { 0 }
}

/// True if saved registers are valid (DebugMon has fired at least once).
pub fn regs_valid() -> bool {
    REGS_VALID.load(Ordering::Relaxed) != 0
}

/// Request single-step: next DebugMon return will set MON_STEP.
pub fn request_single_step() {
    SINGLE_STEP_REQUEST.store(1, Ordering::Relaxed);
}

/// Clear single-step request (used after normal continue).
pub fn clear_single_step() {
    SINGLE_STEP_REQUEST.store(0, Ordering::Relaxed);
}

/// True if the target is halted inside the DebugMonitor handler.
pub fn is_halted() -> bool {
    TARGET_HALTED.load(Ordering::Relaxed) != 0
}

/// Tell the halted DebugMonitor handler to resume execution.
pub fn resume_target() {
    RESUME_REQUEST.store(1, Ordering::Release);
    // DSB ensures the store is visible; SEV wakes the handler's WFE.
    unsafe { core::arch::asm!("dsb", "sev"); }
}

/// Mark GDB as actively listening.  Allows DebugMonitor to halt.
pub fn set_gdb_active(active: bool) {
    GDB_ACTIVE.store(if active { 1 } else { 0 }, Ordering::Release);
}

/// Full debug-state cleanup for GDB detach / auto-detach.
///
/// Clears SINGLE_STEP_REQUEST, MON_STEP (DEMCR bit 18) and
/// MON_PEND (DEMCR bit 17), then marks GDB inactive so any
/// stray DebugMonitor invocations become harmless no-ops.
pub unsafe fn cleanup_for_detach() {
    clear_single_step();
    let demcr = core::ptr::read_volatile(DEMCR);
    core::ptr::write_volatile(DEMCR, demcr & !((1 << 18) | (1 << 17)));
    GDB_ACTIVE.store(0, Ordering::Release);
}

/// Pend the DebugMonitor exception (MON_PEND, DEMCR bit 17).
/// The exception fires as soon as the CPU returns to code at lower
/// priority than DebugMonitor (i.e. Thread mode where demos run).
/// Used to halt the target when entering GDB RSP mode.
///
/// Note: bit 19 (MON_REQ) is just a software semaphore — it does NOT
/// pend the exception.  Bit 17 (MON_PEND) is the real pending bit.
pub fn pend_debugmon() {
    unsafe {
        let demcr = core::ptr::read_volatile(DEMCR);
        core::ptr::write_volatile(DEMCR, demcr | (1 << 17)); // MON_PEND
    }
}

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
    //    Demos run from #[idle] (Thread mode, lowest priority), so DebugMon
    //    can preempt them at any priority.  But the handler spin-waits when
    //    halted, so USB ISR (RTIC priority 2 = NVIC 0xE0) must be able to
    //    preempt to deliver GDB 'continue' commands.
    //    RTIC logical-to-NVIC mapping: ((16 - prio) << 4).
    //      - RTIC pri 2 → 0xE0  (USB ISR, heartbeat, sensor)
    //      - RTIC pri 1 → 0xF0  (run_demo, test_all)
    //    Use 0xF0 — the lowest HW priority — so all RTIC tasks preempt.
    //    SHPR3 bits [7:0] = DebugMonitor priority.
    let shpr3 = core::ptr::read_volatile(SCB_SHPR3);
    core::ptr::write_volatile(SCB_SHPR3, (shpr3 & !0xFF) | 0xF0);

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
/// Called when a DWT watchpoint or FPB breakpoint fires (or after a single step).
/// Saves the full register context from the exception frame and manually-pushed
/// R4-R11 so the GDB RSP stub can serve register reads.
///
/// The Cortex-M hardware auto-stacks: R0, R1, R2, R3, R12, LR, PC, xPSR.
/// We manually read R4-R11 and the pre-exception SP.
///
/// **Stack selection:** Sandboxed demos run in Thread mode using PSP
/// (CONTROL.SPSEL=1).  The hardware exception frame (R0,R1,R2,R3,R12,
/// LR,PC,xPSR) is pushed onto whichever stack was active pre-exception
/// (PSP for demos, MSP for idle/privileged).  EXC_RETURN bit 2 tells
/// us which: 0=MSP, 1=PSP.  The trampoline reads both stacks and
/// passes both to the inner handler.
#[allow(non_snake_case)]
#[unsafe(naked)]
pub unsafe extern "C" fn DebugMonitor_handler() {
    // Naked trampoline:
    //   1. Save R4-R11 and LR (EXC_RETURN) on MSP (handler mode always uses MSP)
    //   2. Check EXC_RETURN (LR) bit 2 to find the exception frame stack
    //   3. Pass r0 = pointer to saved R4-R11 on MSP
    //          r1 = pointer to exception frame (on MSP or PSP)
    //          r2 = 1 if exception frame is on PSP, 0 if MSP
    core::arch::naked_asm!(
        "push {{r4-r11, lr}}",  // save R4-R11 + EXC_RETURN on MSP (9 words)
        "mov  r0, sp",          // r0 = pointer to saved R4-R11 on MSP
        "tst  lr, #4",          // test EXC_RETURN bit 2 (stack selection)
        "ite  eq",
        "addeq r1, sp, #36",   // bit2=0: MSP — exception frame above 9-word push
        "mrsne r1, PSP",       // bit2=1: PSP — exception frame is on PSP
        "ite  eq",
        "moveq r2, #0",        // r2 = 0 (MSP)
        "movne r2, #1",        // r2 = 1 (PSP)
        "bl   {handler}",      // call the real handler (clobbers LR)
        "pop  {{r4-r11, lr}}",  // restore R4-R11 + EXC_RETURN
        "bx   lr",             // return from exception with correct EXC_RETURN
        handler = sym debugmon_inner,
    );
}

/// Inner DebugMonitor handler.
///
/// Arguments:
///   `regs_ptr`  — pointer to R4..R11 saved on MSP by the trampoline
///   `frame_ptr` — pointer to the hardware exception frame (R0,R1,R2,R3,R12,LR,PC,xPSR)
///                 This is on MSP or PSP depending on what Thread mode was using.
///   `on_psp`    — 1 if exception frame is on PSP, 0 if on MSP
unsafe extern "C" fn debugmon_inner(regs_ptr: *const u32, frame_ptr: *const u32, on_psp: u32) {
    let _ = on_psp; // available for future register-write support
    DEBUGMON_COUNT.fetch_add(1, Ordering::Relaxed);

    // Read R4-R11 from our manually-pushed save area on MSP
    let r4  = core::ptr::read_volatile(regs_ptr.add(0));
    let r5  = core::ptr::read_volatile(regs_ptr.add(1));
    let r6  = core::ptr::read_volatile(regs_ptr.add(2));
    let r7  = core::ptr::read_volatile(regs_ptr.add(3));
    let r8  = core::ptr::read_volatile(regs_ptr.add(4));
    let r9  = core::ptr::read_volatile(regs_ptr.add(5));
    let r10 = core::ptr::read_volatile(regs_ptr.add(6));
    let r11 = core::ptr::read_volatile(regs_ptr.add(7));

    // Read hardware exception frame (on whichever stack was active)
    let r0   = core::ptr::read_volatile(frame_ptr.add(0));
    let r1   = core::ptr::read_volatile(frame_ptr.add(1));
    let r2   = core::ptr::read_volatile(frame_ptr.add(2));
    let r3   = core::ptr::read_volatile(frame_ptr.add(3));
    let r12  = core::ptr::read_volatile(frame_ptr.add(4));
    let lr   = core::ptr::read_volatile(frame_ptr.add(5));
    let pc   = core::ptr::read_volatile(frame_ptr.add(6));
    let xpsr = core::ptr::read_volatile(frame_ptr.add(7));

    // Pre-exception SP: exception frame + 8 words, on whichever stack
    let pre_sp = frame_ptr as u32 + 8 * 4;

    // Store into SAVED_REGS [R0..R12, SP, LR, PC, xPSR]
    SAVED_REGS[0].store(r0, Ordering::Relaxed);
    SAVED_REGS[1].store(r1, Ordering::Relaxed);
    SAVED_REGS[2].store(r2, Ordering::Relaxed);
    SAVED_REGS[3].store(r3, Ordering::Relaxed);
    SAVED_REGS[4].store(r4, Ordering::Relaxed);
    SAVED_REGS[5].store(r5, Ordering::Relaxed);
    SAVED_REGS[6].store(r6, Ordering::Relaxed);
    SAVED_REGS[7].store(r7, Ordering::Relaxed);
    SAVED_REGS[8].store(r8, Ordering::Relaxed);
    SAVED_REGS[9].store(r9, Ordering::Relaxed);
    SAVED_REGS[10].store(r10, Ordering::Relaxed);
    SAVED_REGS[11].store(r11, Ordering::Relaxed);
    SAVED_REGS[12].store(r12, Ordering::Relaxed);
    SAVED_REGS[13].store(pre_sp, Ordering::Relaxed);
    SAVED_REGS[14].store(lr, Ordering::Relaxed);
    SAVED_REGS[15].store(pc, Ordering::Relaxed);
    SAVED_REGS[16].store(xpsr, Ordering::Relaxed);
    REGS_VALID.store(1, Ordering::Relaxed);

    // Check each DWT comparator's MATCHED bit (cleared on read of FUNCTION reg)
    let mut dwt_hit = false;
    for n in 0..4u32 {
        let func = core::ptr::read_volatile(func_addr(n));
        if func & (1 << 24) != 0 {
            HIT_COUNT[n as usize].fetch_add(1, Ordering::Relaxed);
            LAST_HIT_PC[n as usize].store(pc, Ordering::Relaxed);
            dwt_hit = true;
        }
    }

    // If no DWT comparator matched, this was likely an FPB breakpoint or BKPT
    if !dwt_hit {
        crate::fpb::record_hit(pc);
    }

    // Handle single-step: if requested, set MON_STEP in DEMCR.
    // MON_STEP (bit 18) causes the CPU to DebugMon after executing
    // exactly one instruction upon return from this exception.
    if SINGLE_STEP_REQUEST.load(Ordering::Relaxed) != 0 {
        SINGLE_STEP_REQUEST.store(0, Ordering::Relaxed);
        let demcr = core::ptr::read_volatile(DEMCR);
        core::ptr::write_volatile(DEMCR, demcr | (1 << 18)); // MON_STEP
    } else {
        // Clear MON_STEP so normal continue doesn't keep stepping
        let demcr = core::ptr::read_volatile(DEMCR);
        core::ptr::write_volatile(DEMCR, demcr & !(1 << 18));
    }

    // ── Guard: skip halt if no GDB listener ──
    // After detach or USB disconnect, GDB_ACTIVE is cleared.
    // If DebugMonitor re-fires (stale MON_STEP, MON_PEND, or DWT hit),
    // halting here would freeze the demo forever since nobody will send
    // RESUME_REQUEST.  Clean up and return immediately instead.
    if GDB_ACTIVE.load(Ordering::Acquire) == 0 {
        // Clear MON_STEP so we don't immediately re-fire
        let demcr = core::ptr::read_volatile(DEMCR);
        core::ptr::write_volatile(DEMCR, demcr & !((1 << 18) | (1 << 17)));
        return;
    }

    // ── Halt: spin-wait until the RSP stub says resume ──
    // This is what makes GDB "halt" actually work — the target thread
    // is frozen inside this exception handler until GDB sends 'c' or 's'.
    RESUME_REQUEST.store(0, Ordering::Relaxed);
    TARGET_HALTED.store(1, Ordering::Release);

    // Spin-wait with WFE to save power. The RSP stub sets RESUME_REQUEST
    // and the USB interrupt wakes us via SEV (event register).
    while RESUME_REQUEST.load(Ordering::Acquire) == 0 {
        core::arch::asm!("wfe");
    }

    TARGET_HALTED.store(0, Ordering::Release);

    // If single-step was requested during the halt, set MON_STEP now
    if SINGLE_STEP_REQUEST.load(Ordering::Relaxed) != 0 {
        SINGLE_STEP_REQUEST.store(0, Ordering::Relaxed);
        let demcr = core::ptr::read_volatile(DEMCR);
        core::ptr::write_volatile(DEMCR, demcr | (1 << 18));
    }
}
