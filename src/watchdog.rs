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

const IWDG_KR:  *mut u32 = 0x4000_3000 as *mut u32;
const IWDG_PR:  *mut u32 = 0x4000_3004 as *mut u32;
const IWDG_RLR: *mut u32 = 0x4000_3008 as *mut u32;
const IWDG_SR:  *const u32 = 0x4000_300C as *const u32;

/// Start the IWDG with ~5 second timeout.
///
/// Must be called once during init.  After this, `feed()` must be
/// called before the timeout expires or the MCU resets.
///
/// # Safety
/// Writes to IWDG registers.  Must be called from privileged mode.
pub unsafe fn start() {
    // Start the watchdog — this also enables the LSI oscillator.
    // Default timeout is ~512 ms (prescaler /4, reload 0xFFF).
    core::ptr::write_volatile(IWDG_KR, 0xCCCC);

    // Enable register access
    core::ptr::write_volatile(IWDG_KR, 0x5555);

    // Prescaler /128: LSI ~32 KHz / 128 = 250 Hz
    core::ptr::write_volatile(IWDG_PR, 0b101); // /128

    // Reload 1250: 1250 / 250 = 5.0 seconds
    core::ptr::write_volatile(IWDG_RLR, 1250);

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
    unsafe {
        core::ptr::write_volatile(IWDG_KR, 0xAAAA);
    }
}
