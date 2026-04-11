#![allow(dead_code)]
//! Independent Watchdog (IWDG) for automatic hub reset on hang.
//!
//! The STM32F413 IWDG is clocked from the ~32 KHz LSI oscillator and
//! runs independently of the main clock.  Once started it cannot be
//! stopped — if not fed within the timeout the MCU hard-resets.
//!
//! Configuration: prescaler /128, reload 1250 → ~5 second timeout.
//! Heartbeat feeds it every 500 ms, SVC delay feeds it every 20 ms.
//! If the firmware completely locks up (HardFault, infinite loop,
//! corrupted stack), the IWDG fires and the hub reboots cleanly.

use crate::periph::{OncePeripheral, Iwdg};

// ── Module-owned peripheral token (set once in start) ──
static IWDG_P: OncePeripheral<Iwdg> = OncePeripheral::new();

// ── IWDG register offsets ──
const IWDG_KR:  u32 = 0x00;
const IWDG_PR:  u32 = 0x04;
const IWDG_RLR: u32 = 0x08;
// const IWDG_SR:  u32 = 0x0C;  // unused

/// Start the IWDG with ~5 second timeout.
///
/// Takes exclusive ownership of the IWDG peripheral token.
/// Must be called once during init.
pub fn start() {
    // Take exclusive ownership of IWDG
    IWDG_P.set(Iwdg::take());
    let iwdg = IWDG_P.get();

    // Start the watchdog — this also enables the LSI oscillator.
    // Default timeout is ~512 ms (prescaler /4, reload 0xFFF).
    iwdg.write(IWDG_KR, 0xCCCC);

    // Enable register access
    iwdg.write(IWDG_KR, 0x5555);

    // Prescaler /128: LSI ~32 KHz / 128 = 250 Hz
    iwdg.write(IWDG_PR, 0b101); // /128

    // Reload 1250: 1250 / 250 = 5.0 seconds
    iwdg.write(IWDG_RLR, 1250);

    // Feed immediately — the new prescaler/reload values will take
    // effect within a few LSI cycles (well before first timeout).
    feed();
}

/// Feed (reload) the watchdog.  Resets the countdown.
///
/// Call this from heartbeat and from SVC delay_ms to keep the
/// watchdog happy during normal operation and demo execution.
#[inline(always)]
pub fn feed() {
    IWDG_P.get().write(IWDG_KR, 0xAAAA);
}

// ── Reset source detection ──
//
// RCC_CSR at 0x4002_3874:
//   bit 29 = IWDGRSTF  (Independent Watchdog reset)
//   bit 31 = LPWRRSTF  (Low-power reset)
//   bit 28 = WWDGRSTF  (Window Watchdog reset)
//   bit 27 = PORRSTF   (Power-on / power-down reset)
//   bit 26 = PINRSTF   (NRST pin reset)
//   bit 25 = BORRSTF   (Brown-out reset)
//   bit 24 = RMVF      (write 1 to clear all reset flags)
const RCC_CSR: *mut u32 = 0x4002_3874 as *mut u32;

/// Check if the previous reset was caused by the IWDG watchdog.
/// Clears the reset flags so the check is one-shot.
///
/// Call early in init, before starting the new IWDG.
pub fn was_iwdg_reset() -> bool {
    unsafe {
        let csr = core::ptr::read_volatile(RCC_CSR);
        // Clear all reset flags (write 1 to RMVF, bit 24)
        core::ptr::write_volatile(RCC_CSR, csr | (1 << 24));
        csr & (1 << 29) != 0
    }
}

/// Play a distinctive high-to-low glide to signal a watchdog reset.
///
/// Uses direct register writes to TIM6/DAC — no allocations, no
/// interrupts needed, works even from a partially garbled state.
/// Total duration: ~300 ms (6 steps × 50 ms).
pub fn watchdog_reset_chime() {
    // Descending tones: 2 kHz → 200 Hz in 6 steps
    const FREQS: [u32; 6] = [2000, 1600, 1200, 800, 400, 200];
    for &f in &FREQS {
        crate::sound::play(f);
        crate::power::delay_ms(50);
    }
    crate::sound::stop();
}

// ── Watchdog post-mortem crash record (SRAM2, survives reset) ──
//
// Layout at 0x2004_FFC0 (8 words = 32 bytes):
//   [0] magic   — 0xDEAD_D069 ("DEAD DOG") if record is valid
//   [1] uptime  — heartbeat tick count at last feed
//   [2] basepri — BASEPRI register value (non-zero = preemption masked)
//   [3] tasks   — task_state bitmap (which tasks were active)
//   [4] gdb_active — 1 if GDB RSP stub was active
//   [5] sandboxed — 1 if sandbox was running
//   [6] primask — PRIMASK (1 = interrupts globally disabled)
//   [7] lr      — Link Register at snapshot time
//
// Written by `snapshot()` on every heartbeat feed.  If the watchdog
// fires because heartbeat is blocked, the last snapshot survives reset.
// `read_crash_record()` retrieves it on the next boot.

const CRASH_REC_ADDR: *mut u32 = 0x2004_FFC0 as *mut u32;
const CRASH_REC_MAGIC: u32 = 0xDEAD_D069;

/// Snapshot key system state to SRAM2.  Call from heartbeat on every feed.
///
/// If the watchdog later fires (heartbeat blocked), this snapshot is
/// the post-mortem: it shows what was happening just before the hang.
///
/// # Safety
/// Writes to SRAM2 reserved area.
pub unsafe fn snapshot(uptime_ticks: u32) {
    let basepri: u32;
    let primask: u32;
    let lr: u32;
    core::arch::asm!(
        "mrs {b}, BASEPRI",
        "mrs {p}, PRIMASK",
        "mov {l}, lr",
        b = out(reg) basepri,
        p = out(reg) primask,
        l = out(reg) lr,
    );

    let tasks = crate::task_state::active_bitmap();
    let gdb = if crate::dwt::gdb_is_active() { 1u32 } else { 0 };
    let sandboxed = if crate::sandbox::is_sandboxed() { 1u32 } else { 0 };

    core::ptr::write_volatile(CRASH_REC_ADDR,          CRASH_REC_MAGIC);
    core::ptr::write_volatile(CRASH_REC_ADDR.add(1),   uptime_ticks);
    core::ptr::write_volatile(CRASH_REC_ADDR.add(2),   basepri);
    core::ptr::write_volatile(CRASH_REC_ADDR.add(3),   tasks);
    core::ptr::write_volatile(CRASH_REC_ADDR.add(4),   gdb);
    core::ptr::write_volatile(CRASH_REC_ADDR.add(5),   sandboxed);
    core::ptr::write_volatile(CRASH_REC_ADDR.add(6),   primask);
    core::ptr::write_volatile(CRASH_REC_ADDR.add(7),   lr);
}

/// Post-mortem crash record from a previous watchdog reset.
#[derive(Debug)]
pub struct CrashRecord {
    pub uptime: u32,
    pub basepri: u32,
    pub tasks: u32,
    pub gdb_active: bool,
    pub sandboxed: bool,
    pub primask: u32,
    pub lr: u32,
}

/// Read the crash record left by a previous boot's watchdog reset.
/// Returns `None` if the magic is wrong (power cycle / no crash).
/// Clears the record so it's one-shot.
///
/// # Safety
/// Reads from SRAM2 reserved area.
pub unsafe fn read_crash_record() -> Option<CrashRecord> {
    let magic = core::ptr::read_volatile(CRASH_REC_ADDR);
    if magic != CRASH_REC_MAGIC {
        return None;
    }

    let rec = CrashRecord {
        uptime:     core::ptr::read_volatile(CRASH_REC_ADDR.add(1)),
        basepri:    core::ptr::read_volatile(CRASH_REC_ADDR.add(2)),
        tasks:      core::ptr::read_volatile(CRASH_REC_ADDR.add(3)),
        gdb_active: core::ptr::read_volatile(CRASH_REC_ADDR.add(4)) != 0,
        sandboxed:  core::ptr::read_volatile(CRASH_REC_ADDR.add(5)) != 0,
        primask:    core::ptr::read_volatile(CRASH_REC_ADDR.add(6)),
        lr:         core::ptr::read_volatile(CRASH_REC_ADDR.add(7)),
    };

    // Clear so we don't report it again
    for i in 0..8 {
        core::ptr::write_volatile(CRASH_REC_ADDR.add(i), 0);
    }

    Some(rec)
}

use core::sync::atomic::{AtomicU32, Ordering};

/// Stored crash record from previous boot (set during init).
static CRASH_UPTIME: AtomicU32 = AtomicU32::new(0);
static CRASH_BASEPRI: AtomicU32 = AtomicU32::new(0);
static CRASH_TASKS: AtomicU32 = AtomicU32::new(0);
static CRASH_FLAGS: AtomicU32 = AtomicU32::new(0); // bit0=gdb, bit1=sandboxed, bit2=valid
static CRASH_PRIMASK: AtomicU32 = AtomicU32::new(0);
static CRASH_LR: AtomicU32 = AtomicU32::new(0);

/// Read crash record from SRAM2 and store in atomics for later display.
/// Call once during init (before SRAM2 is reused).
///
/// # Safety
/// Reads from SRAM2 reserved area.
pub unsafe fn read_and_store_crash_record() {
    if let Some(rec) = read_crash_record() {
        CRASH_UPTIME.store(rec.uptime, Ordering::Release);
        CRASH_BASEPRI.store(rec.basepri, Ordering::Release);
        CRASH_TASKS.store(rec.tasks, Ordering::Release);
        CRASH_PRIMASK.store(rec.primask, Ordering::Release);
        CRASH_LR.store(rec.lr, Ordering::Release);
        let mut flags = 0x4u32; // bit2 = valid
        if rec.gdb_active { flags |= 0x1; }
        if rec.sandboxed { flags |= 0x2; }
        CRASH_FLAGS.store(flags, Ordering::Release);
    }
}

/// Retrieve the stored crash record, if any.
/// Returns None if no watchdog reset occurred on previous boot.
pub fn last_crash_record() -> Option<CrashRecord> {
    let flags = CRASH_FLAGS.load(Ordering::Acquire);
    if flags & 0x4 == 0 {
        return None;
    }
    Some(CrashRecord {
        uptime:     CRASH_UPTIME.load(Ordering::Acquire),
        basepri:    CRASH_BASEPRI.load(Ordering::Acquire),
        tasks:      CRASH_TASKS.load(Ordering::Acquire),
        gdb_active: flags & 0x1 != 0,
        sandboxed:  flags & 0x2 != 0,
        primask:    CRASH_PRIMASK.load(Ordering::Acquire),
        lr:         CRASH_LR.load(Ordering::Acquire),
    })
}
