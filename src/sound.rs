#![allow(dead_code)]
//! Non-blocking speaker driver for LEGO SPIKE Prime Hub.
//!
//! Uses the on-board DAC1 channel 1 (PA4) routed through an amplifier
//! enabled by PC10.  Tone generation is **fully hardware-driven**:
//! TIM6 triggers the DAC's built-in triangle wave generator, so the
//! speaker plays without any CPU involvement.  An RTIC async task
//! just manages start/stop timing.
//!
//! ## Hardware
//!   - PA4  → DAC1_OUT1 (analog output)
//!   - PC10 → Amplifier enable (HIGH = on)
//!   - TIM6 → DAC trigger source (TIM6_TRGO)
//!
//! ## Usage
//!   ```ignore
//!   sound::init();          // once at boot
//!   sound::play(1000);      // start 1 kHz tone
//!   // ... non-blocking, tone plays in hardware ...
//!   sound::stop();          // silence
//!   ```

use crate::pins;
use crate::reg::{reg_modify, reg_read, reg_write};

// ── Peripheral addresses ──
const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_APB1ENR: u32 = 0x40;

const DAC_BASE: u32 = 0x4000_7400;
const DAC_CR: u32 = 0x00;
const DAC_DHR12R1: u32 = 0x08;

const TIM6_BASE: u32 = 0x4000_1000;
const TIM6_CR1: u32 = 0x00;
const TIM6_CR2: u32 = 0x04;
const TIM6_PSC: u32 = 0x28;
const TIM6_ARR: u32 = 0x2C;

// DAC_CR bit positions for channel 1
const EN1: u32 = 0;       // DAC channel 1 enable
const TEN1: u32 = 2;      // Trigger enable
const TSEL1_SHIFT: u32 = 3; // Trigger selection (3 bits)
const WAVE1_SHIFT: u32 = 6; // Wave generation (2 bits)
const MAMP1_SHIFT: u32 = 8; // Mask/amplitude selector (4 bits)

// TIM6 TRGO trigger selection for DAC: TSEL1 = 0b000
const TSEL1_TIM6: u32 = 0b000;

// Triangle wave amplitude: MAMP=9 → 10-bit (0..1023)
// One full cycle = 2 × 1023 = 2046 trigger events.
// Frequency = TIM6_rate / (ARR+1) / 2046.
const MAMP: u32 = 9;
const CYCLE_EVENTS: u32 = 2 * ((1 << (MAMP + 1)) - 1); // 2046

/// Initialize DAC1 channel 1 and TIM6.  Call once during boot.
pub unsafe fn init() {
    // Enable clocks: GPIOA, GPIOC (for amp), DAC, TIM6
    reg_modify(RCC, RCC_AHB1ENR, 0, (1 << 0) | (1 << 2)); // GPIOA + GPIOC
    reg_modify(RCC, RCC_APB1ENR, 0, (1 << 29) | (1 << 4)); // DAC + TIM6
    let _ = reg_read(RCC, RCC_APB1ENR);

    // PA4 = analog mode (DAC output)
    reg_modify(pins::GPIOA, pins::MODER, 3 << 8, 3 << 8);

    // PC10 = output, push-pull (amplifier enable), initially LOW (off)
    reg_modify(pins::GPIOC, pins::MODER, 3 << 20, 1 << 20);
    reg_write(pins::GPIOC, pins::BSRR, 1 << (10 + 16)); // PC10 LOW

    // TIM6: master mode = Update (TRGO on each update event)
    reg_write(TIM6_BASE, TIM6_CR2, 0b010 << 4); // MMS = 010 (Update)
    // Don't enable TIM6 yet — play() will start it.

    // DAC channel 1: triangle wave, TIM6 trigger, disabled for now.
    // We configure everything except EN1, which play() sets.
    let cr_val = (1 << TEN1)                      // trigger enable
               | (TSEL1_TIM6 << TSEL1_SHIFT)      // TIM6 TRGO
               | (0b10 << WAVE1_SHIFT)             // triangle wave
               | (MAMP << MAMP1_SHIFT);            // amplitude
    reg_write(DAC_BASE, DAC_CR, cr_val);

    // Set DAC base value to 0 — triangle swings 0..1023
    reg_write(DAC_BASE, DAC_DHR12R1, 0);
}

/// Start playing a tone at the given frequency (Hz).
///
/// The tone plays entirely in hardware (TIM6 → DAC triangle wave →
/// amplifier → speaker).  Call `stop()` to silence.
pub fn play(freq_hz: u32) {
    if freq_hz == 0 { return; }

    // ARR = (96 MHz / (freq × CYCLE_EVENTS)) - 1
    // TIM6 runs at APB1 timer clock = 96 MHz (48 MHz APB1 × 2 prescaler)
    let arr = (96_000_000 / (freq_hz * CYCLE_EVENTS)).saturating_sub(1);
    // Clamp to valid 16-bit range
    let arr = if arr > 0xFFFF { 0xFFFF } else if arr == 0 { 1 } else { arr };

    unsafe {
        // Configure TIM6
        reg_write(TIM6_BASE, TIM6_CR1, 0);   // stop
        reg_write(TIM6_BASE, TIM6_PSC, 0);   // no prescaler
        reg_write(TIM6_BASE, TIM6_ARR, arr);
        reg_write(TIM6_BASE, TIM6_CR1, 1);   // enable

        // Enable DAC channel 1
        reg_modify(DAC_BASE, DAC_CR, 0, 1 << EN1);

        // Enable amplifier
        reg_write(pins::GPIOC, pins::BSRR, 1 << 10); // PC10 HIGH
    }
}
/// Play a tone at half amplitude (MAMP=8, 9-bit triangle).
/// Used by RTTY to keep volume down.
pub fn play_quiet(freq_hz: u32) {
    if freq_hz == 0 { return; }

    const QUIET_MAMP: u32 = 8;
    const QUIET_CYCLE: u32 = 2 * ((1 << (QUIET_MAMP + 1)) - 1); // 1022

    let arr = (96_000_000 / (freq_hz * QUIET_CYCLE)).saturating_sub(1);
    let arr = if arr > 0xFFFF { 0xFFFF } else if arr == 0 { 1 } else { arr };

    unsafe {
        reg_write(TIM6_BASE, TIM6_CR1, 0);
        reg_write(TIM6_BASE, TIM6_PSC, 0);
        reg_write(TIM6_BASE, TIM6_ARR, arr);
        reg_write(TIM6_BASE, TIM6_CR1, 1);

        // Reconfigure DAC CR with lower MAMP
        let cr_val = (1 << EN1)
                   | (1 << TEN1)
                   | (TSEL1_TIM6 << TSEL1_SHIFT)
                   | (0b10 << WAVE1_SHIFT)
                   | (QUIET_MAMP << MAMP1_SHIFT);
        reg_write(DAC_BASE, DAC_CR, cr_val);
        reg_write(DAC_BASE, DAC_DHR12R1, 0);

        reg_write(pins::GPIOC, pins::BSRR, 1 << 10);
    }
}

/// Stop the tone and disable the amplifier.
pub fn stop() {
    unsafe {
        // Disable TIM6
        reg_write(TIM6_BASE, TIM6_CR1, 0);

        // Disable DAC channel 1 (preserves other CR bits)
        reg_modify(DAC_BASE, DAC_CR, 1 << EN1, 0);

        // Set DAC output to mid-scale briefly, then 0, to avoid pop
        reg_write(DAC_BASE, DAC_DHR12R1, 0);

        // Disable amplifier
        reg_write(pins::GPIOC, pins::BSRR, 1 << (10 + 16)); // PC10 LOW

        // Restore full-amplitude DAC CR for next play() call
        let cr_val = (1 << TEN1)
                   | (TSEL1_TIM6 << TSEL1_SHIFT)
                   | (0b10 << WAVE1_SHIFT)
                   | (MAMP << MAMP1_SHIFT);
        reg_write(DAC_BASE, DAC_CR, cr_val);
    }
}

/// Enable the amplifier only (for testing).
pub fn amp_on() {
    unsafe { reg_write(pins::GPIOC, pins::BSRR, 1 << 10); }
}

/// Disable the amplifier.
pub fn amp_off() {
    unsafe { reg_write(pins::GPIOC, pins::BSRR, 1 << (10 + 16)); }
}

/// Check if a tone is currently playing (TIM6 enabled).
pub fn is_playing() -> bool {
    unsafe { reg_read(TIM6_BASE, TIM6_CR1) & 1 != 0 }
}
