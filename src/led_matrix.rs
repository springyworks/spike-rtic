//! TLC5955 LED matrix driver for LEGO SPIKE Prime Hub.
//!
//! The hub has a TLC5955 48-channel constant-current LED driver connected
//! via SPI1 (PA5/PA6/PA7). Latch on PA15. GSCLK on PB15 via TIM12_CH2
//! at 9.6 MHz (96 MHz / 10).
//!
//! 25 white LEDs in a 5×5 matrix + 4 RGB status indicators = 37 LEDs
//! using 37 of the 48 channels.

use crate::pins;
use crate::reg::{reg_modify, reg_read, reg_write};

const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_APB1ENR: u32 = 0x40;
const RCC_APB2ENR: u32 = 0x44;

const SPI1: u32 = 0x4001_3000;
const SPI_CR1: u32 = 0x00;
const SPI_SR: u32 = 0x08;
const SPI_DR: u32 = 0x0C;

const TIM12: u32 = 0x4000_1800;
const TIM_CR1: u32 = 0x00;
const TIM_CCMR1: u32 = 0x18;
const TIM_CCER: u32 = 0x20;
const TIM_PSC: u32 = 0x28;
const TIM_ARR: u32 = 0x2C;
const TIM_CCR2: u32 = 0x38;
const TIM_EGR: u32 = 0x14;

/// TLC5955 frame size in bytes (769 bits → 97 bytes, MSB-padded).
const FRAME_SIZE: usize = 97;

/// TLC5955 control register latch data.
/// Sets: DC=127 (dot correction), MC=0, BC=127 (global brightness).
const CONTROL_LATCH: [u8; FRAME_SIZE] = {
    let mut d = [0xFFu8; FRAME_SIZE];
    d[0] = 0x01;
    d[1] = 0x96;
    let mut i = 2;
    while i < 50 {
        d[i] = 0x00;
        i += 1;
    }
    d[50] = 0x06;
    d[51] = 0x7F;
    d[52] = 0xFF;
    d[53] = 0xFE;
    d[54] = 0x00;
    d
};

/// Grayscale frame buffer: 48 channels × 16-bit = 96 bytes + 1 header byte.
static mut GS_BUF: [u8; FRAME_SIZE] = [0u8; FRAME_SIZE];

/// Initialize SPI1, TIM12 GSCLK, and the TLC5955.
///
/// # Safety
/// Writes to peripheral registers. Call once during init.
pub unsafe fn init() {
    // Enable clocks: GPIOA, GPIOB, SPI1, TIM12
    reg_modify(RCC, RCC_AHB1ENR, 0, (1 << 0) | (1 << 1)); // GPIOA + GPIOB
    reg_modify(RCC, RCC_APB2ENR, 0, 1 << 12); // SPI1
    reg_modify(RCC, RCC_APB1ENR, 0, 1 << 6);  // TIM12
    let _ = reg_read(RCC, RCC_APB1ENR); // wait for clocks

    // PA5, PA6, PA7 → AF5 (SPI1), very high speed
    reg_modify(
        pins::GPIOA,
        pins::MODER,
        (3 << 10) | (3 << 12) | (3 << 14),
        (2 << 10) | (2 << 12) | (2 << 14),
    );
    reg_modify(
        pins::GPIOA,
        pins::OSPEEDR,
        (3 << 10) | (3 << 12) | (3 << 14),
        (3 << 10) | (3 << 12) | (3 << 14),
    );
    reg_modify(
        pins::GPIOA,
        pins::AFRL,
        (0xF << 20) | (0xF << 24) | (0xF << 28),
        (5 << 20) | (5 << 24) | (5 << 28),
    );

    // PA15 → output (latch), start LOW
    reg_modify(pins::GPIOA, pins::MODER, 3 << 30, 1 << 30);
    reg_write(pins::GPIOA, pins::BSRR, 1 << (15 + 16));

    // PB15 → AF9 (TIM12_CH2) for GSCLK
    reg_modify(pins::GPIOB, pins::MODER, 3 << 30, 2 << 30);
    reg_modify(pins::GPIOB, pins::AFRH, 0xF << 28, 9 << 28);

    // SPI1: master, 8-bit, software SS, CPOL=0, CPHA=0, br=/2 (48 MHz)
    reg_write(
        SPI1,
        SPI_CR1,
        (1 << 2)  // MSTR
        | (1 << 3)  // BR = /2 (bit 5:3 = 0b001, but bit3=1 for /4... let's use /4 for safety)
        | (1 << 6)  // SPE
        | (1 << 8)  // SSI
        | (1 << 9), // SSM
    );

    // TIM12_CH2: 9.6 MHz GSCLK (PSC=0, ARR=9 → 96 MHz/10 = 9.6 MHz)
    reg_write(TIM12, TIM_PSC, 0);
    reg_write(TIM12, TIM_ARR, 9);
    reg_write(TIM12, TIM_CCR2, 4); // ~50% duty
    reg_write(TIM12, TIM_CCMR1, (6 << 12) | (1 << 11)); // OC2M=PWM1, OC2PE
    reg_write(TIM12, TIM_CCER, 1 << 4); // CC2E
    reg_write(TIM12, TIM_EGR, 1); // UG — load shadow registers
    reg_write(TIM12, TIM_CR1, 1); // CEN

    // Send control register latch twice (TLC5955 spec)
    spi_send(&CONTROL_LATCH);
    latch();
    spi_send(&CONTROL_LATCH);
    latch();
}

/// Send a byte array over SPI1 (blocking).
unsafe fn spi_send(data: &[u8]) {
    for &byte in data {
        while reg_read(SPI1, SPI_SR) & (1 << 1) == 0 {} // TXE
        reg_write(SPI1, SPI_DR, byte as u32);
    }
    while reg_read(SPI1, SPI_SR) & (1 << 1) == 0 {} // TXE
    while reg_read(SPI1, SPI_SR) & (1 << 7) != 0 {} // BSY
    let _ = reg_read(SPI1, SPI_DR); // drain RXNE
    let _ = reg_read(SPI1, SPI_SR);
}

/// Pulse the latch pin (PA15) to transfer data to TLC5955 outputs.
unsafe fn latch() {
    reg_write(pins::GPIOA, pins::BSRR, 1 << 15);
    reg_write(pins::GPIOA, pins::BSRR, 1 << (15 + 16));
}

/// Set a single matrix pixel (0–24) brightness (0–100).
/// Uses quadratic gamma: duty = brightness² × 65535 / 10000.
pub fn set_pixel(index: usize, brightness: u16) {
    if index >= 25 {
        return;
    }
    let ch = pins::MATRIX_CHANNELS[index] as usize;
    let duty = (u16::MAX as u32) * (brightness as u32) * (brightness as u32) / 10000;
    let duty = duty.min(u16::MAX as u32) as u16;
    set_channel_raw(ch, duty);
}

/// Set a raw 16-bit grayscale value on a TLC5955 channel (0–47).
pub fn set_channel_raw(ch: usize, value: u16) {
    if ch >= 48 {
        return;
    }
    unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(GS_BUF);
        buf[ch * 2 + 1] = (value >> 8) as u8;
        buf[ch * 2 + 2] = value as u8;
    }
}

/// Set a status LED RGB value. `led` is one of the (R,G,B) channel tuples
/// from `pins`, e.g. `pins::STATUS_TOP`.
pub fn set_status_rgb(led: (u8, u8, u8), r: u16, g: u16, b: u16) {
    set_channel_raw(led.0 as usize, r);
    set_channel_raw(led.1 as usize, g);
    set_channel_raw(led.2 as usize, b);
}

/// Verify TIM12 (GSCLK) and SPI1 are still running; re-enable if not.
///
/// Returns `true` if repair was needed.
///
/// # Safety
/// Reads/writes peripheral registers.
pub unsafe fn check_health() -> bool {
    let mut repaired = false;

    // Check TIM12 CEN (bit 0)
    if reg_read(TIM12, TIM_CR1) & 1 == 0 {
        // GSCLK stopped — full TIM12 re-init
        reg_modify(RCC, RCC_APB1ENR, 0, 1 << 6); // ensure clock
        reg_write(TIM12, TIM_PSC, 0);
        reg_write(TIM12, TIM_ARR, 9);
        reg_write(TIM12, TIM_CCR2, 4);
        reg_write(TIM12, TIM_CCMR1, (6 << 12) | (1 << 11));
        reg_write(TIM12, TIM_CCER, 1 << 4);
        reg_write(TIM12, TIM_EGR, 1);
        reg_write(TIM12, TIM_CR1, 1);
        repaired = true;
    }

    // Check SPI1 SPE (bit 6) and MSTR (bit 2)
    let cr1 = reg_read(SPI1, SPI_CR1);
    if cr1 & (1 << 6) == 0 || cr1 & (1 << 2) == 0 {
        // SPI1 lost enable or master mode — re-init
        reg_modify(RCC, RCC_APB2ENR, 0, 1 << 12); // ensure clock
        reg_write(
            SPI1,
            SPI_CR1,
            (1 << 2)  // MSTR
            | (1 << 3)  // BR
            | (1 << 6)  // SPE
            | (1 << 8)  // SSI
            | (1 << 9), // SSM
        );
        repaired = true;
    }

    // Check PB15 is still AF mode (MODER bits 31:30 == 0b10)
    if reg_read(pins::GPIOB, pins::MODER) >> 30 & 3 != 2 {
        reg_modify(pins::GPIOB, pins::MODER, 3 << 30, 2 << 30);
        reg_modify(pins::GPIOB, pins::AFRH, 0xF << 28, 9 << 28);
        repaired = true;
    }

    if repaired {
        // Re-send control latch so TLC5955 is in correct mode
        spi_send(&CONTROL_LATCH);
        latch();
        spi_send(&CONTROL_LATCH);
        latch();
    }

    repaired
}

/// Diagnostic snapshot: returns (tim12_cr1, spi1_cr1, spi1_sr, pb15_moder_bits).
pub fn diag() -> (u32, u32, u32, u32) {
    unsafe {
        let tim12_cr1 = reg_read(TIM12, TIM_CR1);
        let spi1_cr1 = reg_read(SPI1, SPI_CR1);
        let spi1_sr = reg_read(SPI1, SPI_SR);
        let pb15_moder = (reg_read(pins::GPIOB, pins::MODER) >> 30) & 3;
        (tim12_cr1, spi1_cr1, spi1_sr, pb15_moder)
    }
}

/// Flush the grayscale buffer to the TLC5955.
///
/// Includes a health check: if TIM12 or SPI1 got knocked out
/// (e.g. by a user demo or peripheral fault), re-enables them
/// before sending data.
///
/// # Safety
/// Accesses SPI1 registers and the global GS_BUF.
pub unsafe fn update() {
    check_health();
    spi_send(core::slice::from_raw_parts(
        core::ptr::addr_of!(GS_BUF) as *const u8,
        FRAME_SIZE,
    ));
    latch();
}

/// Clear all LEDs (all channels to 0) and update.
pub unsafe fn clear() {
    let buf = &mut *core::ptr::addr_of_mut!(GS_BUF);
    for i in 0..FRAME_SIZE {
        buf[i] = 0;
    }
    update();
}

/// Display a 5×5 boolean pattern on the matrix at given brightness.
pub fn show_pattern(pattern: &[u8; 25], brightness: u16) {
    for (i, &val) in pattern.iter().enumerate() {
        set_pixel(i, if val != 0 { brightness } else { 0 });
    }
}

// ── Morse code battery-LED status indicator ──
//
// Two-letter Morse codes blinked on the battery LED:
//   CH = Charging    (C: -.-. H: ....)
//   HI = Complete    (H: .... I: ..)
//   NG = Fault       (N: -.   G: --.)
//   BT = Battery     (B: -... T: -)
//
// Timing: each element = 1 base unit (250 ms nominal).
// dit = 1 unit ON, dah = 3 units ON.
// Farnsworth spacing: element timing at ~10 WPM (1 unit = 125 ms),
// but letter gaps stretched to 7 units (875 ms) and repeat pause
// to 24 units (3 s) for clear character separation.
//
// Dit = 1 unit ON, Dah = 3 units ON, element gap = 1 unit OFF.
// Letter gap = 7 units OFF (Farnsworth).  Repeat pause = 24 units OFF.

/// CH — Charging: C(-.-.) H(....)
const MORSE_CH: &[u8] = &[
    1,1,1, 0, 1, 0, 1,1,1, 0, 1,           // C
    0,0,0,0,0,0,0,                          // letter gap (Farnsworth)
    1, 0, 1, 0, 1, 0, 1,                    // H
    0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,  // 3 s pause
];

/// HI — Complete: H(....) I(..)
const MORSE_HI: &[u8] = &[
    1, 0, 1, 0, 1, 0, 1,                    // H
    0,0,0,0,0,0,0,                          // letter gap (Farnsworth)
    1, 0, 1,                                // I
    0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
];

/// NG — Fault: N(-.) G(--.)
const MORSE_NG: &[u8] = &[
    1,1,1, 0, 1,                             // N
    0,0,0,0,0,0,0,                          // letter gap (Farnsworth)
    1,1,1, 0, 1,1,1, 0, 1,                  // G
    0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
];

/// BT — Battery/Discharging: B(-...) T(-)
const MORSE_BT: &[u8] = &[
    1,1,1, 0, 1, 0, 1, 0, 1,                // B
    0,0,0,0,0,0,0,                          // letter gap (Farnsworth)
    1,1,1,                                  // T
    0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
];

/// Morse status indices (match ChargerStatus variants).
pub const MORSE_CHARGING: u8 = 0;
pub const MORSE_COMPLETE: u8 = 1;
pub const MORSE_FAULT:    u8 = 2;
pub const MORSE_BATTERY:  u8 = 3;

/// Morse-code blinker state for the battery LED.
///
/// Call `set_status()` each heartbeat to track charger state,
/// then `is_mark()` + `advance()` once per 125 ms unit (Farnsworth).
pub struct MorseBlinker {
    pos: u8,
    status: u8,
}

impl MorseBlinker {
    pub const fn new() -> Self {
        Self { pos: 0, status: 0 }
    }

    fn pattern(&self) -> &'static [u8] {
        match self.status {
            MORSE_CHARGING => MORSE_CH,
            MORSE_COMPLETE => MORSE_HI,
            MORSE_FAULT    => MORSE_NG,
            _              => MORSE_BT,
        }
    }

    /// Update status. Resets to pattern start when it changes.
    pub fn set_status(&mut self, s: u8) {
        if s != self.status {
            self.status = s;
            self.pos = 0;
        }
    }

    /// `true` if the current position is a Morse mark (bright flash).
    pub fn is_mark(&self) -> bool {
        let p = self.pattern();
        (self.pos as usize) < p.len() && p[self.pos as usize] != 0
    }

    /// Advance to the next position (wraps at end of pattern).
    pub fn advance(&mut self) {
        self.pos += 1;
        if self.pos as usize >= self.pattern().len() {
            self.pos = 0;
        }
    }
}

/// Built-in display patterns for debug visualization.
pub mod patterns {
    pub const HEART: [u8; 25] = [
        0, 1, 0, 1, 0,
        1, 1, 1, 1, 1,
        1, 1, 1, 1, 1,
        0, 1, 1, 1, 0,
        0, 0, 1, 0, 0,
    ];

    pub const CHECK: [u8; 25] = [
        0, 0, 0, 0, 1,
        0, 0, 0, 1, 0,
        0, 0, 1, 0, 0,
        1, 0, 1, 0, 0,
        0, 1, 0, 0, 0,
    ];

    pub const CROSS: [u8; 25] = [
        1, 0, 0, 0, 1,
        0, 1, 0, 1, 0,
        0, 0, 1, 0, 0,
        0, 1, 0, 1, 0,
        1, 0, 0, 0, 1,
    ];

    pub const ARROW_UP: [u8; 25] = [
        0, 0, 1, 0, 0,
        0, 1, 1, 1, 0,
        1, 0, 1, 0, 1,
        0, 0, 1, 0, 0,
        0, 0, 1, 0, 0,
    ];

    pub const USB_ICON: [u8; 25] = [
        0, 0, 1, 0, 0,
        0, 1, 1, 1, 0,
        0, 0, 1, 0, 0,
        0, 1, 0, 1, 0,
        1, 0, 0, 0, 1,
    ];

    pub const DFU_ICON: [u8; 25] = [
        1, 1, 0, 1, 1,
        1, 0, 0, 0, 1,
        1, 0, 1, 0, 1,
        1, 0, 0, 0, 1,
        1, 1, 1, 1, 1,
    ];

    pub const ALL_ON: [u8; 25] = [1; 25];

    /// Show a number 0–9 on the 5×5 matrix (simple bitmap font).
    pub fn digit(n: u8) -> [u8; 25] {
        match n {
            0 => [
                0,1,1,1,0,
                1,0,0,1,1,
                1,0,1,0,1,
                1,1,0,0,1,
                0,1,1,1,0,
            ],
            1 => [
                0,0,1,0,0,
                0,1,1,0,0,
                0,0,1,0,0,
                0,0,1,0,0,
                0,1,1,1,0,
            ],
            2 => [
                0,1,1,1,0,
                1,0,0,0,1,
                0,0,1,1,0,
                0,1,0,0,0,
                1,1,1,1,1,
            ],
            3 => [
                1,1,1,1,0,
                0,0,0,0,1,
                0,1,1,1,0,
                0,0,0,0,1,
                1,1,1,1,0,
            ],
            4 => [
                1,0,0,1,0,
                1,0,0,1,0,
                1,1,1,1,1,
                0,0,0,1,0,
                0,0,0,1,0,
            ],
            5 => [
                1,1,1,1,1,
                1,0,0,0,0,
                1,1,1,1,0,
                0,0,0,0,1,
                1,1,1,1,0,
            ],
            6 => [
                0,1,1,1,0,
                1,0,0,0,0,
                1,1,1,1,0,
                1,0,0,0,1,
                0,1,1,1,0,
            ],
            7 => [
                1,1,1,1,1,
                0,0,0,1,0,
                0,0,1,0,0,
                0,1,0,0,0,
                0,1,0,0,0,
            ],
            8 => [
                0,1,1,1,0,
                1,0,0,0,1,
                0,1,1,1,0,
                1,0,0,0,1,
                0,1,1,1,0,
            ],
            9 => [
                0,1,1,1,0,
                1,0,0,0,1,
                0,1,1,1,1,
                0,0,0,0,1,
                0,1,1,1,0,
            ],
            _ => [0; 25],
        }
    }
}
