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

/// Flush the grayscale buffer to the TLC5955.
///
/// # Safety
/// Accesses SPI1 registers and the global GS_BUF.
pub unsafe fn update() {
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
