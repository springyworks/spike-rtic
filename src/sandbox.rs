#![allow(dead_code)]
//! MPU sandbox for user-uploaded RAM demos.
//!
//! Protects the Demon monitor firmware from buggy/malicious user code
//! uploaded to SRAM2.  Uses Cortex-M4 MPU + unprivileged Thread mode
//! so a crashing demo triggers MemManage fault → clean recovery instead
//! of requiring a battery-lift hard reset.
//!
//! ## Privilege levels
//!
//! | Command | Mode       | MPU  | API mechanism        | Use case           |
//! |---------|------------|------|----------------------|--------------------|
//! | `go`    | Sandboxed  | On   | SVC trap → callbacks | User apps          |
//! | `go!`   | Privileged | Off  | Direct fn pointers   | Firmware dev/debug |
//!
//! ## MPU regions (STM32F413 Cortex-M4, 8 regions available)
//!
//! | # | Range                    | Size  | Access         | Purpose            |
//! |---|--------------------------|-------|----------------|--------------------|
//! | 0 | 0x2004_0000–0x2004_FFFF | 64 KB | RW, unpriv+priv| Demo sandbox SRAM2 |
//! | 1 | 0x0800_0000–0x080F_FFFF | 1 MB  | RO+exec, all   | Flash (code fetch) |
//! | 2 | 0x2000_0000–0x2003_FFFF | 256KB | Priv-only RW   | SRAM1 (firmware)   |
//! | 3 | 0x4000_0000–0x5FFF_FFFF | 512MB | Priv-only RW   | Peripherals        |
//! | 4 | 0xE000_0000–0xE00F_FFFF | 1 MB  | Priv-only RW   | PPB (NVIC/SCB/MPU) |
//!
//! ## SVC calling convention
//!
//! Demo code calls API via `SVC #N` where N is the function index:
//!
//! | SVC# | Function       | Args (r0–r3)                 |
//! |------|----------------|------------------------------|
//! | 0    | write          | r0=ptr, r1=len               |
//! | 1    | delay_ms       | r0=ms                        |
//! | 2    | set_pixel      | r0=index, r1=brightness      |
//! | 3    | update_leds    | (none)                        |
//! | 4    | read_adc       | r0=channel; returns r0=value  |
//! | 5    | read_buttons   | returns r0=flags              |
//! | 6    | motor_set      | r0=port, r1=speed            |
//! | 7    | motor_brake    | r0=port                      |
//! | 8    | sensor_read    | r0=buf, r1=len; returns r0=n |
//! | 9    | sensor_mode    | r0=mode                      |
//! | 10   | sound_play     | r0=freq_hz                   |
//! | 11   | sound_stop     | (none)                        |

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::user_app_io;

// ── MPU register addresses (Cortex-M4 PPB) ────────────────

const MPU_TYPE: *const u32 = 0xE000_ED90 as *const u32;
const MPU_CTRL: *mut u32   = 0xE000_ED94 as *mut u32;
const MPU_RNR:  *mut u32   = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32   = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32   = 0xE000_EDA0 as *mut u32;

// ── SCB addresses for fault enable ─────────────────────────

const SCB_SHCSR: *mut u32 = 0xE000_ED24 as *mut u32;

// ── MPU RASR helper bits ───────────────────────────────────

/// SIZE field encoding: size = 2^(N+1).
/// 64 KB  = 2^16 → N = 15
/// 256 KB = 2^18 → N = 17
/// 1 MB   = 2^20 → N = 19
/// 512 MB = 2^29 → N = 28
const fn size_field(n: u32) -> u32 { (n & 0x1F) << 1 }

const RASR_ENABLE: u32 = 1 << 0;
const RASR_XN:     u32 = 1 << 28; // Execute-Never

// AP field (bits 26:24)
const AP_RW_RW:    u32 = 0b011 << 24; // Full access (priv + unpriv)
const AP_RW_NONE:  u32 = 0b001 << 24; // Priv RW, unpriv no access
const AP_RO_RO:    u32 = 0b110 << 24; // Priv RO, unpriv RO
const AP_RW_RO:    u32 = 0b010 << 24; // Priv RW, unpriv RO
const AP_NONE:     u32 = 0b000 << 24; // No access, any privilege

// TEX/S/C/B for normal memory (write-back, write-allocate, shareable)
const NORMAL_MEM: u32 = (0b000 << 19) | (1 << 18) | (1 << 17) | (1 << 16);
// TEX/S/C/B for device memory (strongly-ordered for peripherals)
const DEVICE_MEM: u32 = (0b000 << 19) | (1 << 18) | (0 << 17) | (1 << 16);

// ── Stack guard constants ──────────────────────────────────

/// Sentinel pattern for stack painting and high-water detection.
const STACK_SENTINEL: u32 = 0xDEAD_BEEF;

/// Bottom of painted stack zone.  Set by `stack_paint()` at init.
static STACK_PAINT_BOTTOM: AtomicU32 = AtomicU32::new(0);
/// Number of sentinel words painted.
static STACK_PAINT_WORDS: AtomicU32 = AtomicU32::new(0);

// ── Sandbox state ──────────────────────────────────────────

/// True while a demo is running in sandbox mode.
static SANDBOXED: AtomicBool = AtomicBool::new(false);

/// Fault info: non-zero if the last sandbox run faulted.
/// Contains the MMFSR value (first 8 bits) and MMFAR validity.
static FAULT_INFO: AtomicU32 = AtomicU32::new(0);

/// Stores MMFAR (faulting address) when a MemManage fault occurs.
static FAULT_ADDR: AtomicU32 = AtomicU32::new(0);

// ── Idle ↔ task communication for Thread-mode sandbox ──────
//
// The sandbox MUST run in Thread mode because SVC from Handler mode
// (RTIC ISR) causes HardFault — SVCall priority (0xF0) is lower
// than any RTIC dispatcher.  The run_demo task requests execution
// via these atomics; the #[idle] loop picks it up and runs it in
// Thread mode where SVC works correctly.

/// Address of demo to run (0 = none pending).
static DEMO_PENDING: AtomicU32 = AtomicU32::new(0);
/// Result value from the last sandbox run.
static DEMO_RESULT: AtomicU32 = AtomicU32::new(0);
/// True when idle has finished the sandbox run.
static DEMO_DONE: AtomicBool = AtomicBool::new(false);
/// True if the sandbox run faulted (result contains MMFSR).
static DEMO_WAS_FAULT: AtomicBool = AtomicBool::new(false);

/// True when the pending DEMO_PENDING is a debug run (privileged Thread
/// mode, no MPU) instead of a sandboxed run.
static DEBUG_RUN: AtomicBool = AtomicBool::new(false);

/// Saved abort context: [0] = MSP before demo, [1] = resume PC (label 2).
/// SVCall handler reads these to build a fake MSP exception frame on kill.
static mut ABORT_CTX: [u32; 2] = [0; 2];

/// Request sandbox execution from idle (Thread mode).
pub fn request_sandbox(addr: u32) {
    DEMO_DONE.store(false, Ordering::Relaxed);
    DEMO_WAS_FAULT.store(false, Ordering::Relaxed);
    DEMO_RESULT.store(0, Ordering::Relaxed);
    // Release: all stores above visible before addr is published
    DEMO_PENDING.store(addr, Ordering::Release);
}

/// Request debug execution from idle (Thread mode, privileged, no MPU).
/// Uses the same DEMO_PENDING/DEMO_DONE mechanism as sandbox.
pub fn request_debug_run(addr: u32) {
    DEBUG_RUN.store(true, Ordering::Relaxed);
    DEMO_DONE.store(false, Ordering::Relaxed);
    DEMO_WAS_FAULT.store(false, Ordering::Relaxed);
    DEMO_RESULT.store(0, Ordering::Relaxed);
    DEMO_PENDING.store(addr, Ordering::Release);
}

/// Check if idle has a pending sandbox request.
pub fn demo_pending() -> u32 {
    DEMO_PENDING.load(Ordering::Acquire)
}

/// Mark sandbox as complete (called from idle after run_sandboxed).
pub fn complete_sandbox(result: u32, faulted: bool) {
    DEMO_RESULT.store(result, Ordering::Relaxed);
    DEMO_WAS_FAULT.store(faulted, Ordering::Relaxed);
    DEMO_PENDING.store(0, Ordering::Relaxed);
    // Release: all stores above visible before done flag
    DEMO_DONE.store(true, Ordering::Release);
}

/// Check if sandbox execution is complete.
pub fn sandbox_done() -> bool {
    DEMO_DONE.load(Ordering::Acquire)
}

/// Get sandbox result and fault status. Call after sandbox_done().
pub fn sandbox_result() -> (u32, bool) {
    (DEMO_RESULT.load(Ordering::Relaxed), DEMO_WAS_FAULT.load(Ordering::Relaxed))
}

/// Check if the pending demo is a debug run (privileged Thread mode).
pub fn is_debug_run() -> bool {
    DEBUG_RUN.load(Ordering::Acquire)
}

/// Run a demo in Thread mode, privileged, no MPU.
/// This allows DebugMonitor to halt the demo for GDB breakpoints.
///
/// # Safety
/// `addr` must point to a valid Thumb function in SRAM2.
pub unsafe fn run_debug(addr: u32, api: *const spike_hub_api::MonitorApi) -> u32 {
    let entry: extern "C" fn(*const spike_hub_api::MonitorApi) -> u32 =
        core::mem::transmute(addr | 1);
    entry(api)
}

/// Check if MPU is available on this chip.
pub fn mpu_present() -> bool {
    let mpu_type = unsafe { core::ptr::read_volatile(MPU_TYPE) };
    // DREGION field (bits 15:8) = number of MPU regions
    (mpu_type >> 8) & 0xFF > 0
}

/// Number of MPU regions available.
pub fn mpu_regions() -> u8 {
    let mpu_type = unsafe { core::ptr::read_volatile(MPU_TYPE) };
    ((mpu_type >> 8) & 0xFF) as u8
}

/// Configure the 5 MPU regions for demo sandboxing.
/// Does NOT enable the MPU yet — call `mpu_enable()` separately.
///
/// # Safety
/// Must be called from privileged mode (init or handler).
pub unsafe fn mpu_configure() {
    // Disable MPU while configuring
    core::ptr::write_volatile(MPU_CTRL, 0);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Region 0: SRAM2 (0x2004_0000, 64 KB) — RW for everyone, XN
    // Demo data lives here.  Execute-never because demo code is
    // fetched from flash (demos are compiled to run from SRAM2
    // but on Cortex-M4 we can allow execute from SRAM2 too).
    // Actually — demos DO execute from SRAM2, so we need exec permission.
    core::ptr::write_volatile(MPU_RNR, 0);
    core::ptr::write_volatile(MPU_RBAR, 0x2004_0000);
    core::ptr::write_volatile(MPU_RASR,
        AP_RW_RW | NORMAL_MEM | size_field(15) | RASR_ENABLE);
    // size_field(15) = 2^16 = 64 KB

    // Region 1: Flash (0x0800_0000, 1 MB) — RO+exec for everyone
    // Demo code is fetched from here (vector table, firmware code).
    // User demos in SRAM2 call SVC to trap into firmware code in flash.
    core::ptr::write_volatile(MPU_RNR, 1);
    core::ptr::write_volatile(MPU_RBAR, 0x0800_0000);
    core::ptr::write_volatile(MPU_RASR,
        AP_RO_RO | NORMAL_MEM | size_field(19) | RASR_ENABLE);
    // size_field(19) = 2^20 = 1 MB

    // Region 2: SRAM1 (0x2000_0000, 256 KB) — privileged only
    // Firmware .bss, .data, stack, ring buffers, USB descriptors.
    core::ptr::write_volatile(MPU_RNR, 2);
    core::ptr::write_volatile(MPU_RBAR, 0x2000_0000);
    core::ptr::write_volatile(MPU_RASR,
        AP_RW_NONE | NORMAL_MEM | RASR_XN | size_field(17) | RASR_ENABLE);
    // size_field(17) = 2^18 = 256 KB

    // Region 3: Peripherals (0x4000_0000, 512 MB) — privileged only
    // UART, SPI, USB, GPIO, ADC, TIM, DAC — all privileged.
    core::ptr::write_volatile(MPU_RNR, 3);
    core::ptr::write_volatile(MPU_RBAR, 0x4000_0000);
    core::ptr::write_volatile(MPU_RASR,
        AP_RW_NONE | DEVICE_MEM | RASR_XN | size_field(28) | RASR_ENABLE);
    // size_field(28) = 2^29 = 512 MB

    // Region 4: PPB (0xE000_0000, 1 MB) — privileged only
    // NVIC, SCB, MPU, SysTick — must not be accessible to demos.
    // Note: PPB is always privileged-only by default on Cortex-M,
    // but an explicit region makes it clear and catches wayward accesses.
    core::ptr::write_volatile(MPU_RNR, 4);
    core::ptr::write_volatile(MPU_RBAR, 0xE000_0000);
    core::ptr::write_volatile(MPU_RASR,
        AP_RW_NONE | DEVICE_MEM | RASR_XN | size_field(19) | RASR_ENABLE);
    // size_field(19) = 2^20 = 1 MB

    // Region 5: MSP stack guard band — 32 bytes, no access
    // Placed at _stack_end (linker symbol = end of .bss/.uninit).
    // If the firmware MSP grows past its intended limit into .bss,
    // this triggers a MemManage fault instead of silent corruption.
    // Aligned UP to 32 bytes — round up so the guard sits entirely in
    // stack space and never overlaps the end of .bss/.uninit.
    extern "C" { static _stack_end: u32; }
    let guard_base = ((&_stack_end as *const u32 as u32) + 31) & !0x1F;
    core::ptr::write_volatile(MPU_RNR, 5);
    core::ptr::write_volatile(MPU_RBAR, guard_base);
    core::ptr::write_volatile(MPU_RASR,
        AP_NONE | NORMAL_MEM | RASR_XN | size_field(4) | RASR_ENABLE);
    // size_field(4) = 2^5 = 32 bytes, AP_NONE = no access at any privilege

    // Region 6: Demo PSP stack guard band — 32 bytes, no access
    // Demo stack lives in top 4 KB of SRAM2 (0x2004_E000–0x2004_F000,
    // PSP initialized to 0x2004_F000).  Below that is demo code/data.
    // This guard catches demo stack overflow before it corrupts the
    // demo's own .text section.
    core::ptr::write_volatile(MPU_RNR, 6);
    core::ptr::write_volatile(MPU_RBAR, 0x2004_DFE0);
    core::ptr::write_volatile(MPU_RASR,
        AP_NONE | NORMAL_MEM | RASR_XN | size_field(4) | RASR_ENABLE);
    // 0x2004_DFE0 = 32-byte aligned, just below demo stack bottom

    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Enable the MPU with PRIVDEFENA = 1 (privileged code gets default map).
///
/// # Safety
/// Must be called after `mpu_configure()`, from privileged mode.
pub unsafe fn mpu_enable() {
    // Bit 0 = ENABLE, Bit 2 = PRIVDEFENA (priv gets default background map)
    core::ptr::write_volatile(MPU_CTRL, (1 << 0) | (1 << 2));
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Disable the MPU entirely (for privileged `go!` mode).
///
/// # Safety
/// Must be called from privileged mode.
pub unsafe fn mpu_disable() {
    core::ptr::write_volatile(MPU_CTRL, 0);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Enable MemManage fault handler (otherwise faults escalate to HardFault).
///
/// # Safety
/// Must be called from privileged mode.
pub unsafe fn enable_memmanage() {
    let shcsr = core::ptr::read_volatile(SCB_SHCSR);
    core::ptr::write_volatile(SCB_SHCSR, shcsr | (1 << 16)); // MEMFAULTENA
}

// ── Stack painting and high-water measurement ──────────────

/// Paint the stack region between _stack_end and the current MSP
/// with a known sentinel pattern.  Call once from init().
///
/// Leaves a 256-byte headroom above _stack_end so the paint loop
/// itself doesn't trip the guard band.
///
/// # Safety
/// Must be called early in init, before tasks are spawned.
pub unsafe fn stack_paint() {
    extern "C" { static _stack_end: u32; }
    let bottom = (&_stack_end as *const u32 as u32) + 256; // headroom above guard
    let msp: u32;
    core::arch::asm!("mrs {}, MSP", out(reg) msp);
    // Paint from bottom up to 512 bytes below current MSP (safety margin)
    let top = msp.saturating_sub(512);
    if top <= bottom {
        return; // not enough room
    }
    let words = (top - bottom) / 4;
    let base = bottom as *mut u32;
    for i in 0..words {
        core::ptr::write_volatile(base.add(i as usize), STACK_SENTINEL);
    }
    STACK_PAINT_BOTTOM.store(bottom, Ordering::Release);
    STACK_PAINT_WORDS.store(words, Ordering::Release);
}

/// Scan the painted stack zone and return (used_bytes, total_painted_bytes).
/// `used_bytes` is how far the stack has penetrated into the painted zone.
/// Returns (0, 0) if stack was never painted.
pub fn stack_high_water() -> (u32, u32) {
    let bottom = STACK_PAINT_BOTTOM.load(Ordering::Acquire);
    let words = STACK_PAINT_WORDS.load(Ordering::Acquire);
    if bottom == 0 || words == 0 {
        return (0, 0);
    }
    let total = words * 4;
    // Scan upward from bottom to find first non-sentinel (= high-water mark)
    let base = bottom as *const u32;
    let mut intact = 0u32;
    for i in 0..words {
        if unsafe { core::ptr::read_volatile(base.add(i as usize)) } != STACK_SENTINEL {
            break;
        }
        intact += 1;
    }
    let used = (words - intact) * 4;
    (used, total)
}

/// Check if a stack-fault marker was left by a previous boot's MemManage
/// handler.  If found, print a diagnostic via trace and clear it.
///
/// # Safety
/// Reads from SRAM2 reserved area.  Call once in init().
pub unsafe fn check_fault_marker() {
    const FAULT_MARKER_ADDR: *mut u32 = 0x2004_FFE0 as *mut u32;
    const FAULT_MARKER_MAGIC: u32 = 0xDEAD_5AFC;

    let magic = core::ptr::read_volatile(FAULT_MARKER_ADDR);
    if magic == FAULT_MARKER_MAGIC {
        let mmfsr = core::ptr::read_volatile(FAULT_MARKER_ADDR.add(1));
        let addr = core::ptr::read_volatile(FAULT_MARKER_ADDR.add(2));
        // Clear the marker so we don't report it again
        core::ptr::write_volatile(FAULT_MARKER_ADDR, 0);
        core::ptr::write_volatile(FAULT_MARKER_ADDR.add(1), 0);
        core::ptr::write_volatile(FAULT_MARKER_ADDR.add(2), 0);
        // Store for later retrieval by shell `info` command
        LAST_STACK_FAULT_MMFSR.store(mmfsr, Ordering::Release);
        LAST_STACK_FAULT_ADDR.store(addr, Ordering::Release);
    }
}

/// Previous boot's stack fault info (0 = no fault).
static LAST_STACK_FAULT_MMFSR: AtomicU32 = AtomicU32::new(0);
static LAST_STACK_FAULT_ADDR: AtomicU32 = AtomicU32::new(0);

/// Return (mmfsr, fault_addr) from previous boot's stack overflow, if any.
/// Both zero means no previous fault.
pub fn last_stack_fault() -> (u32, u32) {
    (
        LAST_STACK_FAULT_MMFSR.load(Ordering::Acquire),
        LAST_STACK_FAULT_ADDR.load(Ordering::Acquire),
    )
}

// ── Demo execution trampoline ──────────────────────────────

/// Run a demo in sandboxed (unprivileged) mode.
///
/// 1. Enable MPU
/// 2. Set PSP to SRAM2 demo stack, switch to unprivileged + PSP
/// 3. Call demo entry point
/// 4. SVC #100 restores privileged mode
/// 5. Disable MPU, check for faults
///
/// Returns `Ok(result)` on clean return, `Err(fault_info)` on MemManage fault.
///
/// # Safety
/// `addr` must point to a valid Thumb function in SRAM2.
/// The MonitorApi passed to the demo uses SVC thunks instead of direct
/// function pointers.
pub unsafe fn run_sandboxed(addr: u32, api: *const spike_hub_api::MonitorApi) -> Result<u32, u32> {
    SANDBOXED.store(true, Ordering::Release);
    FAULT_INFO.store(0, Ordering::Release);
    FAULT_ADDR.store(0, Ordering::Release);

    // Copy MonitorApi to SRAM2 so the unprivileged demo can read it.
    // The original api struct lives on the caller's stack in SRAM1
    // (protected, priv-only).  We place the copy at 0x2004_FF00
    // (above the demo stack in SRAM2's top reserved area).
    const API_SRAM2_ADDR: u32 = 0x2004_FF00;
    let api_sram2 = API_SRAM2_ADDR as *mut u8;
    core::ptr::copy_nonoverlapping(
        api as *const u8,
        api_sram2,
        core::mem::size_of::<spike_hub_api::MonitorApi>(),
    );
    let api_for_demo = API_SRAM2_ADDR as *const spike_hub_api::MonitorApi;

    // Enable MPU (idempotent — already enabled at init with PRIVDEFENA=1,
    // but harmless to re-assert in case something disabled it)
    mpu_enable();

    // The entire privilege-drop → demo-call → privilege-restore
    // sequence MUST be in a single asm block to prevent the compiler
    // from generating stack spills in SRAM1 while we're unprivileged.
    //
    // Design:
    //   - Set PSP to top of SRAM2 demo stack (0x2004_F000)
    //   - Set CONTROL = 3 (nPRIV=1, SPSEL=1) → unprivileged + PSP
    //   - BLX to demo entry (demo uses PSP in SRAM2 for its stack)
    //   - Save result to r4 (callee-saved, survives SVC exception)
    //   - SVC #100 → handler restores CONTROL=0 (privileged + MSP)
    //   - Move r4 back to r0
    //
    // The compiler saves/restores r4, r5 (callee-saved) on MSP
    // BEFORE dropping privilege and AFTER restoring it.

    let result: u32;
    core::arch::asm!(
        // Save abort context: MSP and resume address (label 2).
        // On kill, SVCall handler builds a fake exception frame on MSP
        // at saved_msp-32 and returns via EXC_RETURN=0xFFFFFFF9
        // (Thread mode + MSP), landing at label 2.
        "mrs r2, MSP",
        "str r2, [r3]",          // ABORT_CTX[0] = MSP
        "adr r2, 2f",
        "str r2, [r3, #4]",      // ABORT_CTX[1] = label 2 address
        // Set up PSP to demo stack in SRAM2 (last 4 KB)
        "movw r2, #0xF000",
        "movt r2, #0x2004",
        "msr PSP, r2",
        // Drop to unprivileged + PSP
        // CONTROL bit 0 = nPRIV (1=unpriv), bit 1 = SPSEL (1=PSP)
        "mov r2, #3",
        "msr CONTROL, r2",
        "isb",
        // Call demo: r0 = api pointer, entry in r5
        "blx r5",
        // Demo returned: r0 = result.  Save in r4 (survives SVC).
        "mov r4, r0",
        // SVC #100: restore privileged mode (CONTROL.nPRIV=0)
        "svc #100",
        // EXC_RETURN (0xFFFFFFFD) overrides CONTROL.SPSEL back to 1
        // (PSP) on exception return.  Explicitly switch to MSP so
        // subsequent sp-relative accesses hit the correct stack frame.
        "mov r2, #0",
        "msr CONTROL, r2",
        "isb",
        // Back to Thread/Privileged/MSP. Restore result.
        "mov r0, r4",
        // Label 2: abort landing. On kill, exception return resumes
        // here in Thread/Privileged/MSP with r0 = 0xDEAD_ABCD.
        // Normal path also falls through here with r0 = demo result.
        "2:",
        // Operands
        inlateout("r0") api_for_demo => result,
        inlateout("r5") (addr | 1) => _,
        // r3 = pointer to ABORT_CTX for storing MSP + PC
        inlateout("r3") (&raw mut ABORT_CTX) as u32 => _,
        // Clobbers: demo is a separately compiled binary that may
        // trash ALL registers (r0-r12, lr).  r6 is reserved by LLVM,
        // r7 is the Thumb frame pointer — neither can appear here.
        out("r1") _, out("r2") _,
        out("r4") _,
        out("r8") _, out("r9") _, out("r10") _, out("r11") _,
        out("r12") _, out("lr") _,
    );

    // Now we're privileged + MSP (normal return or abort landing)
    ABORT_CTX = [0; 2];
    // MPU stays enabled (guard bands must remain active).
    // PRIVDEFENA=1 means privileged code has full access anyway.
    SANDBOXED.store(false, Ordering::Release);

    let fault = FAULT_INFO.load(Ordering::Acquire);
    if fault != 0 {
        Err(fault)
    } else {
        Ok(result)
    }
}

/// Check if we're currently running a sandboxed demo.
pub fn is_sandboxed() -> bool {
    SANDBOXED.load(Ordering::Acquire)
}

/// Get the fault address from the last sandbox crash (if valid).
pub fn last_fault_addr() -> u32 {
    FAULT_ADDR.load(Ordering::Relaxed)
}

/// Get the fault info from the last sandbox crash.
pub fn last_fault_info() -> u32 {
    FAULT_INFO.load(Ordering::Relaxed)
}

// ── SVCall handler ─────────────────────────────────────────

/// SVCall exception handler.
///
/// Dispatches SVC calls from sandboxed demos to the MonitorApi callbacks.
/// Runs in Handler mode (always privileged), so it can access all
/// peripherals and SRAM1 on behalf of the demo.
///
/// The SVC number is extracted from the instruction that caused the trap.
/// Arguments are in r0–r3 on the exception frame (MSP since SPSEL=0).
///
/// # Important
/// This is registered as `#[exception]` in main.rs.
/// RTIC doesn't use SVCall, so we can claim it.
#[allow(non_snake_case)]
pub unsafe extern "C" fn SVCall_handler() {
    // The stacked PC points to the instruction AFTER the SVC.
    // SVC number is in the low byte of the SVC instruction (at PC-2).
    //
    // The exception frame is on MSP or PSP depending on SPSEL at the
    // time of the SVC.  Check EXC_RETURN bit 2 to determine which:
    //   bit 2 = 0 → frame on MSP
    //   bit 2 = 1 → frame on PSP
    // Exception frame layout: [r0, r1, r2, r3, r12, LR, PC, xPSR]

    let frame_ptr: *mut u32;
    core::arch::asm!(
        "tst lr, #4",
        "ite eq",
        "mrseq {0}, MSP",
        "mrsne {0}, PSP",
        out(reg) frame_ptr,
    );

    // Read stacked registers
    let r0 = *frame_ptr.add(0);
    let r1 = *frame_ptr.add(1);
    let r2 = *frame_ptr.add(2);
    let _r3 = *frame_ptr.add(3);
    let pc = *frame_ptr.add(6);

    // SVC number is at (PC - 2), low byte
    let svc_num = *((pc - 2) as *const u8);

    crate::trace::record(crate::trace::TAG_SVC, svc_num, r0 as u16);

    // ── Abort redirect ─────────────────────────────────────
    //
    // On kill: redirect to label 2 in run_sandboxed via ABORT_CTX.
    // Uses the shared redirect_to_abort_landing() mechanism.
    if svc_num != 100 && user_app_io::is_aborted() && is_sandboxed() {
        if ABORT_CTX[1] != 0 {
            redirect_to_abort_landing(0xDEAD_ABCD);
        }
    }

    let ret: u32 = match svc_num {
        // SVC #0: write(ptr, len)
        0 => {
            let data = r0 as *const u8;
            let len = r1;
            if !data.is_null() && len > 0 && len <= 4096 {
                // Validate pointer is in SRAM2 range
                let end = r0.wrapping_add(len);
                if r0 >= 0x2004_0000 && end <= 0x2005_0000 {
                    let slice = core::slice::from_raw_parts(data, len as usize);
                    user_app_io::write(slice);
                    // Pend USB ISR to drain output immediately
                    cortex_m::peripheral::NVIC::pend(stm32f4::stm32f413::Interrupt::OTG_FS);
                }
            }
            0
        }
        // SVC #1: delay_ms(ms)
        //
        // Chunks delay into 20 ms segments with sensor keepalive.
        // Firmware-enforced pause: holds the user-app here while
        // is_paused() is true.  button_poll (priority 2) can always
        // preempt and toggle pause or set ABORT.
        // On ABORT: returns immediately so the app exits quickly.
        1 => {
            // Abort — return immediately
            if user_app_io::is_aborted() {
                crate::sensor::demo_maintain();
                0
            } else {
                let ms = r0.min(30_000);
                crate::trace::record(crate::trace::TAG_DELAY_START, 0, ms as u16);
                let chunks = ms / 20;
                for _ in 0..chunks {
                    // Firmware-enforced pause with keepalive
                    while crate::is_paused() && !user_app_io::is_aborted() {
                        crate::power::delay_ms(20);
                        crate::sensor::demo_maintain();
                        crate::watchdog::feed();
                    }
                    if user_app_io::is_aborted() {
                        crate::sensor::demo_maintain();
                        break;
                    }
                    crate::power::delay_ms(20);
                    crate::sensor::demo_maintain();
                    crate::watchdog::feed();
                }
                if !user_app_io::is_aborted() {
                    let remainder = ms % 20;
                    if remainder > 0 {
                        crate::power::delay_ms(remainder);
                    }
                }
                0
            }
        }
        // SVC #2: set_pixel(index, brightness)
        2 => {
            if r0 < 25 && r1 <= 100 {
                crate::led_matrix::set_pixel(r0 as usize, r1 as u16);
            }
            0
        }
        // SVC #3: update_leds()
        3 => {
            crate::led_matrix::update();
            0
        }
        // SVC #4: read_adc(channel) -> value
        4 => {
            if r0 <= 15 { crate::read_adc(r0) } else { 0 }
        }
        // SVC #5: read_buttons() -> flags
        5 => {
            if user_app_io::is_aborted() {
                spike_hub_api::BTN_CENTER as u32
            } else {
                crate::read_buttons() as u32
            }
        }
        // SVC #6: motor_set(port, speed)
        6 => {
            if !crate::user_app_io::is_aborted() {
                crate::motor::set(r0, r1 as i32);
            }
            0
        }
        // SVC #7: motor_brake(port)
        7 => {
            crate::motor::brake(r0);
            0
        }
        // SVC #8: sensor_read(buf, len) -> n
        8 => {
            let buf = r0 as *mut u8;
            let len = r1;
            if !buf.is_null() && len > 0 && len <= 256 {
                let end = r0.wrapping_add(len);
                if r0 >= 0x2004_0000 && end <= 0x2005_0000 {
                    let slice = core::slice::from_raw_parts_mut(buf, len as usize);
                    crate::sensor::demo_sensor_read(slice) as u32
                } else {
                    0
                }
            } else {
                0
            }
        }
        // SVC #9: sensor_mode(mode)
        9 => {
            crate::sensor::demo_sensor_mode(r0 as u8);
            0
        }
        // SVC #10: sound_play(freq_hz)
        10 => {
            if !crate::user_app_io::is_aborted() {
                crate::sound::play(r0);
            }
            0
        }
        // SVC #11: sound_stop()
        11 => {
            crate::sound::stop();
            0
        }
        // SVC #12: trace_record(tag, val, arg)
        12 => {
            let tag = (r0 & 0xFF) as u8;
            let val = (r1 & 0xFF) as u8;
            let arg = (r2 & 0xFFFF) as u16;
            crate::trace::record(tag, val, arg);
            0
        }
        // SVC #13: rtty_say(data, len)
        13 => {
            if !crate::user_app_io::is_aborted() && r0 != 0 && r1 > 0 {
                let len = r1.min(80) as usize;
                let slice = core::slice::from_raw_parts(r0 as *const u8, len);
                if !crate::rtty::is_busy() {
                    crate::rtty::set_message(slice);
                    crate::user_app_io::request_rtty_spawn();
                }
            }
            0
        }
        // SVC #14: rtty_busy() -> u32
        14 => {
            if crate::user_app_io::is_aborted() { 0 }
            else if crate::rtty::is_busy() { 1 } else { 0 }
        }
        // SVC #15: motor_position() -> i32
        15 => {
            crate::sensor::demo_motor_position() as u32
        }
        // SVC #16: motor_goto(port, degrees) -> i32 (final error)
        16 => {
            let port = *frame_ptr.add(0);
            let degrees = *frame_ptr.add(1) as i32;
            crate::user_app_io::demo_motor_goto_inner(port, degrees) as u32
        }
        // SVC #17: port_read(port, buf, len) -> n
        17 => {
            let port = *frame_ptr.add(0);
            let buf = *frame_ptr.add(1) as *mut u8;
            let len = *frame_ptr.add(2);
            crate::user_app_io::demo_port_read(port, buf, len)
        }
        // SVC #18: sensor_light(r, g, b)
        18 => {
            let r = (r0 & 0xFF).min(100) as u8;
            let g = (r1 & 0xFF).min(100) as u8;
            let b = (r2 & 0xFF).min(100) as u8;
            crate::sensor::demo_sensor_light(r, g, b);
            0
        }
        // SVC #19: imu_init() -> who_am_i
        19 => {
            crate::imu::init()
        }
        // SVC #20: imu_read(buf, len) -> n
        20 => {
            let buf = r0 as *mut u8;
            let len = r1;
            if !buf.is_null() && len >= 12 {
                let end = r0.wrapping_add(len);
                if r0 >= 0x2004_0000 && end <= 0x2005_0000 {
                    let slice = core::slice::from_raw_parts_mut(buf, len as usize);
                    crate::imu::read(slice)
                } else {
                    0
                }
            } else {
                0
            }
        }
        // SVC #21: set_hub_led(r, g, b) — BATTERY_LED (tiny LED near USB port)
        21 => {
            let r16 = ((r0.min(100)) * 0xFFFF / 100) as u16;
            let g16 = ((r1.min(100)) * 0xFFFF / 100) as u16;
            let b16 = ((r2.min(100)) * 0xFFFF / 100) as u16;
            crate::led_matrix::set_status_rgb(crate::pins::BATTERY_LED, r16, g16, b16);
            crate::led_matrix::update();
            0
        }
        // SVC #100: restore privileged mode (demo returning)
        100 => {
            // Restore CONTROL to privileged + MSP (nPRIV=0, SPSEL=0)
            core::arch::asm!(
                "mov r0, #0",
                "msr CONTROL, r0",
                "isb",
                out("r0") _,
            );
            // Skip r0 write-back — the caller saved the demo result
            // in r4 before the SVC, so stacked r0 doesn't matter.
            return;
        }
        _ => {
            // Unknown SVC — ignore
            0
        }
    };

    // Store return value in the stacked r0
    *frame_ptr.add(0) = ret;
}

// ── Abort landing: build fake MSP frame → Thread/Privileged/MSP ──
//
// Used by three paths that need to forcibly terminate the sandbox:
//   1. SVCall handler — demo called SVC while ABORT flag set
//   2. MemManage handler — demo triggered MPU fault
//   3. force_kill_sandbox() — hard kill from button handler or shell
//
// All three build an 8-word exception frame on MSP at ABORT_CTX[0]-32,
// set EXC_RETURN = 0xFFFFFFF9 (Thread + MSP), and `bx lr`.  Hardware
// pops the fake frame → label 2 in run_sandboxed with r0 = sentinel.

/// Build a fake exception frame on MSP and redirect to label 2 (abort
/// landing) in `run_sandboxed`.  **Never returns** — does `bx lr` with
/// EXC_RETURN 0xFFFFFFF9 which switches to Thread/Privileged/MSP.
///
/// `sentinel` is placed in r0 so run_sandboxed sees it as the result.
///
/// # Safety
/// Must only be called from Handler mode while ABORT_CTX is valid.
unsafe fn redirect_to_abort_landing(sentinel: u32) -> ! {
    let saved_msp = ABORT_CTX[0];
    let saved_pc  = ABORT_CTX[1];

    // Restore privileged Thread mode (nPRIV=0, SPSEL=0).
    core::arch::asm!("mov r0, #0", "msr CONTROL, r0", "isb", out("r0") _);

    // Disable MPU so Thread mode can proceed safely
    core::ptr::write_volatile(MPU_CTRL, 0);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    SANDBOXED.store(false, Ordering::Release);

    // Build 8-word exception frame on MSP just below saved MSP.
    let new_msp = saved_msp - 32;
    let frame = new_msp as *mut u32;
    *frame.add(0) = sentinel;     // r0
    *frame.add(1) = 0;            // r1
    *frame.add(2) = 0;            // r2
    *frame.add(3) = 0;            // r3
    *frame.add(4) = 0;            // r12
    *frame.add(5) = 0;            // LR (unused)
    *frame.add(6) = saved_pc;     // PC = label 2 in run_sandboxed
    *frame.add(7) = 0x0100_0000;  // xPSR: Thumb bit set

    // Set MSP to the fake frame and exception-return to Thread/MSP.
    // Hardware pops the frame → MSP = saved_msp → label 2.
    //
    // CRITICAL: Reset BASEPRI to 0 before bx lr.  If force_kill_sandbox
    // is called from an ISR holding an RTIC lock (e.g. USB ISR running
    // shell → kill -9), BASEPRI is elevated to mask priority ≤ 2.
    // The exception return does NOT restore BASEPRI (it's not in the
    // exception frame).  Without clearing it, Thread mode resumes with
    // priority 2 tasks permanently masked → heartbeat starved →
    // watchdog fires after ~5 seconds.
    core::arch::asm!(
        "mov r0, #0",
        "msr BASEPRI, r0",
        "msr MSP, {msp}",
        "mov lr, {exc}",
        "bx lr",
        msp = in(reg) new_msp,
        exc = in(reg) 0xFFFFFFF9u32,
        options(noreturn),
    );
}

/// Force-kill a sandboxed demo from Handler mode (priority ≥ 2).
///
/// This is the `kill -9` for the sandbox.  Called when the cooperative
/// abort (ABORT flag + SVC check) doesn't work — e.g. the demo is in
/// a tight compute loop that never calls any API.
///
/// The button handler (priority 2) always preempts Thread mode, so the
/// demo's execution context is frozen on PSP.  We hijack the exception
/// return to land at label 2 in `run_sandboxed` on MSP, terminating the
/// demo immediately.
///
/// Returns `true` if the kill was performed, `false` if no sandbox to kill.
///
/// # Safety
/// Must be called from Handler mode (RTIC ISR context).
pub unsafe fn force_kill_sandbox() -> bool {
    if !SANDBOXED.load(Ordering::Acquire) {
        return false;
    }
    let saved_pc = ABORT_CTX[1];
    if saved_pc == 0 {
        return false; // no valid abort context
    }
    // Record that this was a forced kill (not a fault)
    FAULT_INFO.store(0, Ordering::Release);
    user_app_io::request_abort(); // ensure ABORT flag is set for cleanup
    redirect_to_abort_landing(0xDEAD_ABCD);
}

// ── MemManage fault handler ────────────────────────────────

/// MemManage fault handler — catches MPU violations.
///
/// Two cases:
///   1. **Sandbox active (demo PSP fault):** Record fault info and redirect
///      to the abort landing (label 2 in `run_sandboxed`), returning
///      `Err(mmfsr)` to the caller.  The monitor continues normally.
///
///   2. **No sandbox (firmware MSP fault — stack guard hit):** The monitor
///      itself has overflowed.  No state can be trusted.  Disable motor
///      PWM via direct register writes, store a fault marker in SRAM2's
///      reserved area, and trigger a system reset (AIRCR).
///
/// # Important
/// Registered as `#[exception]` in main.rs.  Guard band regions 5+6
/// are always enabled when the MPU is on.
#[allow(non_snake_case)]
pub unsafe extern "C" fn MemManage_handler() {
    // MMFSR is in SCB->CFSR bits [7:0]
    const SCB_CFSR: *mut u32 = 0xE000_ED28 as *mut u32;
    const SCB_MMFAR: *const u32 = 0xE000_ED34 as *const u32;
    const SCB_AIRCR: *mut u32 = 0xE000_ED0C as *mut u32;

    let cfsr = core::ptr::read_volatile(SCB_CFSR);
    let mmfsr = cfsr & 0xFF;

    // Record fault info
    FAULT_INFO.store(mmfsr, Ordering::Release);

    // If MMARVALID (bit 7), record the faulting address
    if mmfsr & (1 << 7) != 0 {
        FAULT_ADDR.store(core::ptr::read_volatile(SCB_MMFAR), Ordering::Release);
    }

    // Clear MMFSR bits by writing 1s
    core::ptr::write_volatile(SCB_CFSR, mmfsr);

    // ── Case 1: Sandbox active → redirect to abort landing ──
    if SANDBOXED.load(Ordering::Acquire) {
        let saved_pc = ABORT_CTX[1];
        if saved_pc != 0 {
            redirect_to_abort_landing(0xDEAD_DEAD);
            // noreturn ↑
        }
        // Fallback: no valid abort context (shouldn't happen during sandbox).
        // Restore privileged mode but keep MPU on (guard bands).
        core::arch::asm!("mov r0, #0", "msr CONTROL, r0", "isb", out("r0") _);
        SANDBOXED.store(false, Ordering::Release);
        return;
    }

    // ── Case 2: No sandbox → firmware MSP guard hit ──
    // The monitor's own stack has overflowed.  Nothing can be trusted.
    //
    // Kill motor PWM via direct TIM register writes (no function calls
    // that might touch corrupted memory):
    //   TIM1_BDTR  (0x4001_0044) — clear MOE (bit 15) to disable outputs
    //   TIM3_CCER  (0x4000_0420) — clear all CC*E bits
    //   TIM4_CCER  (0x4000_0820) — clear all CC*E bits
    core::ptr::write_volatile(0x4001_0044 as *mut u32, 0); // TIM1 BDTR: MOE=0
    core::ptr::write_volatile(0x4000_0420 as *mut u32, 0); // TIM3 CCER: all off
    core::ptr::write_volatile(0x4000_0820 as *mut u32, 0); // TIM4 CCER: all off

    // Write fault marker to SRAM2 reserved area (survives reset).
    // init() can read this and print a diagnostic on next boot.
    // Address 0x2004_FFE0 is in the DFU-reserved tail of SRAM2.
    const FAULT_MARKER_ADDR: *mut u32 = 0x2004_FFE0 as *mut u32;
    const FAULT_MARKER_MAGIC: u32 = 0xDEAD_5AFC; // "DEAD SAFC" = stack fault code
    core::ptr::write_volatile(FAULT_MARKER_ADDR, FAULT_MARKER_MAGIC);
    core::ptr::write_volatile(FAULT_MARKER_ADDR.add(1), mmfsr);
    let fault_addr = FAULT_ADDR.load(Ordering::Acquire);
    core::ptr::write_volatile(FAULT_MARKER_ADDR.add(2), fault_addr);

    // System reset via AIRCR (VECTKEY=0x05FA, SYSRESETREQ=bit 2)
    cortex_m::asm::dsb();
    core::ptr::write_volatile(SCB_AIRCR, 0x05FA_0004);
    loop { cortex_m::asm::nop(); } // wait for reset
}

// ── SVC thunk API for sandboxed demos ──────────────────────

/// Build a MonitorApi with SVC-based thunks for sandboxed execution.
///
/// Each callback does `SVC #N` instead of a direct function call.
/// When the demo calls e.g. `api.write_fn(ctx, ptr, len)`, it
/// executes `SVC #0` which traps into the SVCall handler running
/// in privileged mode.
pub fn build_sandboxed_api() -> spike_hub_api::MonitorApi {
    spike_hub_api::MonitorApi {
        version: spike_hub_api::API_VERSION,
        context: core::ptr::null_mut(),
        write_fn: svc_write,
        delay_ms: svc_delay_ms,
        set_pixel: svc_set_pixel,
        update_leds: svc_update_leds,
        read_adc: svc_read_adc,
        read_buttons: svc_read_buttons,
        motor_set: svc_motor_set,
        motor_brake: svc_motor_brake,
        sensor_read: svc_sensor_read,
        sensor_mode: svc_sensor_mode,
        sound_play: svc_sound_play,
        sound_stop: svc_sound_stop,
        trace_record: svc_trace_record,
        rtty_say: svc_rtty_say,
        rtty_busy: svc_rtty_busy,
        motor_position: svc_motor_position,
        motor_goto: svc_motor_goto,
        port_read: svc_port_read,
        sensor_light: svc_sensor_light,
        imu_init: svc_imu_init,
        imu_read: svc_imu_read,
        set_hub_led: svc_set_hub_led,
    }
}

// ── SVC thunk functions ────────────────────────────────────
//
// These are `extern "C"` to match the MonitorApi function signatures.
// Each puts arguments in r0–r3 and invokes `SVC #N`.
// The SVCall handler reads the stacked frame and dispatches.

extern "C" fn svc_write(_ctx: *mut u8, data: *const u8, len: u32) {
    unsafe {
        core::arch::asm!(
            "svc #0",
            in("r0") data,
            in("r1") len,
            // r0,r1 are clobbered by the exception frame save/restore
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_delay_ms(ms: u32) {
    unsafe {
        core::arch::asm!(
            "svc #1",
            in("r0") ms,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_set_pixel(index: u32, brightness: u32) {
    unsafe {
        core::arch::asm!(
            "svc #2",
            in("r0") index,
            in("r1") brightness,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_update_leds() {
    unsafe {
        core::arch::asm!(
            "svc #3",
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_read_adc(channel: u32) -> u32 {
    let ret: u32;
    unsafe {
        core::arch::asm!(
            "svc #4",
            inlateout("r0") channel => ret,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
    ret
}

extern "C" fn svc_read_buttons() -> u8 {
    let ret: u32;
    unsafe {
        core::arch::asm!(
            "svc #5",
            lateout("r0") ret,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
    ret as u8
}

extern "C" fn svc_motor_set(port: u32, speed: i32) {
    unsafe {
        core::arch::asm!(
            "svc #6",
            in("r0") port,
            in("r1") speed,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_motor_brake(port: u32) {
    unsafe {
        core::arch::asm!(
            "svc #7",
            in("r0") port,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_sensor_read(buf: *mut u8, len: u32) -> u32 {
    let ret: u32;
    unsafe {
        core::arch::asm!(
            "svc #8",
            inlateout("r0") buf => ret,
            in("r1") len,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
    ret
}

extern "C" fn svc_sensor_mode(mode: u32) {
    unsafe {
        core::arch::asm!(
            "svc #9",
            in("r0") mode,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_sound_play(freq_hz: u32) {
    unsafe {
        core::arch::asm!(
            "svc #10",
            in("r0") freq_hz,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_sound_stop() {
    unsafe {
        core::arch::asm!(
            "svc #11",
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_trace_record(tag: u8, val: u8, arg: u16) {
    unsafe {
        core::arch::asm!(
            "svc #12",
            in("r0") tag as u32,
            in("r1") val as u32,
            in("r2") arg as u32,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_rtty_say(data: *const u8, len: u32) {
    unsafe {
        core::arch::asm!(
            "svc #13",
            in("r0") data,
            in("r1") len,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_rtty_busy() -> u32 {
    let result: u32;
    unsafe {
        core::arch::asm!(
            "svc #14",
            lateout("r0") result,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
    result
}

extern "C" fn svc_motor_position() -> i32 {
    let result: u32;
    unsafe {
        core::arch::asm!(
            "svc #15",
            lateout("r0") result,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
    result as i32
}

extern "C" fn svc_motor_goto(port: u32, degrees: i32) -> i32 {
    let result: u32;
    unsafe {
        core::arch::asm!(
            "svc #16",
            inlateout("r0") port => result,
            inlateout("r1") degrees as u32 => _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
    result as i32
}

extern "C" fn svc_port_read(port: u32, buf: *mut u8, len: u32) -> u32 {
    let result: u32;
    unsafe {
        core::arch::asm!(
            "svc #17",
            inlateout("r0") port => result,
            inlateout("r1") buf => _,
            inlateout("r2") len => _,
            lateout("r3") _,
        );
    }
    result
}

extern "C" fn svc_sensor_light(r: u32, g: u32, b: u32) {
    unsafe {
        core::arch::asm!(
            "svc #18",
            in("r0") r,
            in("r1") g,
            in("r2") b,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}

extern "C" fn svc_imu_init() -> u32 {
    let result: u32;
    unsafe {
        core::arch::asm!(
            "svc #19",
            lateout("r0") result,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
    result
}

extern "C" fn svc_imu_read(buf: *mut u8, len: u32) -> u32 {
    let result: u32;
    unsafe {
        core::arch::asm!(
            "svc #20",
            inlateout("r0") buf => result,
            inlateout("r1") len => _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
    result
}

extern "C" fn svc_set_hub_led(r: u32, g: u32, b: u32) {
    unsafe {
        core::arch::asm!(
            "svc #21",
            in("r0") r,
            in("r1") g,
            in("r2") b,
            lateout("r0") _,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
        );
    }
}
