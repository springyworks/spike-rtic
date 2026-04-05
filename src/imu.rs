#![allow(dead_code)]
//! LSM6DS3TR-C IMU driver via I2C2 (bit-bang).
//!
//! The SPIKE Prime hub has an LSM6DS3TR-C 6-DOF accelerometer/gyroscope
//! on I2C2 (PB3=SDA, PB10=SCL).  Since we don't use the STM32 I2C
//! peripheral elsewhere and setup is complex, we bit-bang I2C which is
//! simpler and perfectly fine at the ~100 kHz speeds we need.
//!
//! ## Data format
//!
//! `imu_read` returns 12 bytes: accel XYZ (3×i16 LE) + gyro XYZ (3×i16 LE).
//! Acceleration in raw LSB (default ±2g → 0.061 mg/LSB).
//! Angular rate in raw LSB (default ±250 dps → 8.75 mdps/LSB).

use crate::pins;
use crate::reg::{reg_read, reg_modify, reg_write};
use core::sync::atomic::{AtomicBool, Ordering};

// ── I2C2 pins (bit-bang) ──
const SDA_PORT: u32 = pins::IMU_SDA_PORT; // GPIOB
const SDA_PIN: u32 = pins::IMU_SDA_PIN;   // 3
const SCL_PORT: u32 = pins::IMU_SCL_PORT; // GPIOB
const SCL_PIN: u32 = pins::IMU_SCL_PIN;   // 10

// ── LSM6DS3TR-C registers ──
const IMU_ADDR: u8 = 0x6A; // 7-bit I2C address (SDO/SA0 = GND)
const WHO_AM_I: u8 = 0x0F;
const WHO_AM_I_VAL: u8 = 0x6A; // LSM6DS3TR-C
const CTRL1_XL: u8 = 0x10; // Accel control
const CTRL2_G: u8 = 0x11;  // Gyro control
const STATUS_REG: u8 = 0x1E;
const OUTX_L_G: u8 = 0x22; // Gyro X low byte (6 bytes: G_X, G_Y, G_Z)
const OUTX_L_XL: u8 = 0x28; // Accel X low byte (6 bytes: XL_X, XL_Y, XL_Z)

static INITIALIZED: AtomicBool = AtomicBool::new(false);

// ── Bit-bang I2C helpers ──

#[inline(always)]
fn delay_i2c() {
    // ~5 µs at 96 MHz → ~480 cycles.  Gives ~100 kHz I2C clock.
    for _ in 0..120 {
        cortex_m::asm::nop();
    }
}

/// Configure SDA and SCL as open-drain GPIO outputs (high = release).
unsafe fn gpio_init() {
    // Enable GPIOB clock (bit 1 of AHB1ENR)
    reg_modify(0x4002_3800, 0x30, 0, 1 << 1);

    // SDA (PB3): output, open-drain, pull-up, high speed
    let sda = SDA_PIN;
    reg_modify(SDA_PORT, pins::MODER, 3 << (sda * 2), 1 << (sda * 2)); // output
    reg_modify(SDA_PORT, pins::OTYPER, 0, 1 << sda); // open-drain
    reg_modify(SDA_PORT, pins::OSPEEDR, 0, 3 << (sda * 2)); // high speed
    reg_modify(SDA_PORT, pins::PUPDR, 3 << (sda * 2), 1 << (sda * 2)); // pull-up

    // SCL (PB10): output, open-drain, pull-up, high speed
    let scl = SCL_PIN;
    reg_modify(SCL_PORT, pins::MODER, 3 << (scl * 2), 1 << (scl * 2)); // output
    reg_modify(SCL_PORT, pins::OTYPER, 0, 1 << scl); // open-drain
    reg_modify(SCL_PORT, pins::OSPEEDR, 0, 3 << (scl * 2)); // high speed
    reg_modify(SCL_PORT, pins::PUPDR, 3 << (scl * 2), 1 << (scl * 2)); // pull-up

    // Start with both lines high (released)
    sda_high();
    scl_high();
    delay_i2c();
    delay_i2c();
}

#[inline(always)]
unsafe fn sda_high() {
    reg_write(SDA_PORT, pins::BSRR, 1 << SDA_PIN); // set
}
#[inline(always)]
unsafe fn sda_low() {
    reg_write(SDA_PORT, pins::BSRR, 1 << (SDA_PIN + 16)); // reset
}
#[inline(always)]
unsafe fn scl_high() {
    reg_write(SCL_PORT, pins::BSRR, 1 << SCL_PIN);
}
#[inline(always)]
unsafe fn scl_low() {
    reg_write(SCL_PORT, pins::BSRR, 1 << (SCL_PIN + 16));
}
#[inline(always)]
unsafe fn sda_read() -> bool {
    // Switch SDA to input momentarily
    reg_modify(SDA_PORT, pins::MODER, 3 << (SDA_PIN * 2), 0); // input
    delay_i2c();
    let val = reg_read(SDA_PORT, pins::IDR) & (1 << SDA_PIN) != 0;
    reg_modify(SDA_PORT, pins::MODER, 3 << (SDA_PIN * 2), 1 << (SDA_PIN * 2)); // back to output
    val
}

unsafe fn i2c_start() {
    sda_high();
    scl_high();
    delay_i2c();
    sda_low(); // SDA goes low while SCL is high = START
    delay_i2c();
    scl_low();
    delay_i2c();
}

unsafe fn i2c_stop() {
    sda_low();
    delay_i2c();
    scl_high();
    delay_i2c();
    sda_high(); // SDA goes high while SCL is high = STOP
    delay_i2c();
}

/// Send one byte, return ACK (true = ACK received).
unsafe fn i2c_write_byte(byte: u8) -> bool {
    for bit in (0..8).rev() {
        if byte & (1 << bit) != 0 {
            sda_high();
        } else {
            sda_low();
        }
        delay_i2c();
        scl_high();
        delay_i2c();
        scl_low();
        delay_i2c();
    }
    // Read ACK (device pulls SDA low)
    sda_high(); // release SDA
    // Switch to input to read ACK
    reg_modify(SDA_PORT, pins::MODER, 3 << (SDA_PIN * 2), 0);
    delay_i2c();
    scl_high();
    delay_i2c();
    let ack = reg_read(SDA_PORT, pins::IDR) & (1 << SDA_PIN) == 0; // low = ACK
    scl_low();
    // Back to output
    reg_modify(SDA_PORT, pins::MODER, 3 << (SDA_PIN * 2), 1 << (SDA_PIN * 2));
    delay_i2c();
    ack
}

/// Read one byte, send ACK or NACK.
unsafe fn i2c_read_byte(ack: bool) -> u8 {
    let mut byte: u8 = 0;
    // Switch SDA to input
    reg_modify(SDA_PORT, pins::MODER, 3 << (SDA_PIN * 2), 0);
    for bit in (0..8).rev() {
        delay_i2c();
        scl_high();
        delay_i2c();
        if reg_read(SDA_PORT, pins::IDR) & (1 << SDA_PIN) != 0 {
            byte |= 1 << bit;
        }
        scl_low();
    }
    // Back to output for ACK/NACK
    reg_modify(SDA_PORT, pins::MODER, 3 << (SDA_PIN * 2), 1 << (SDA_PIN * 2));
    if ack {
        sda_low(); // ACK
    } else {
        sda_high(); // NACK
    }
    delay_i2c();
    scl_high();
    delay_i2c();
    scl_low();
    sda_high(); // release
    delay_i2c();
    byte
}

// ── High-level I2C register access ──

/// Write one register.
unsafe fn write_reg(reg: u8, val: u8) -> bool {
    i2c_start();
    if !i2c_write_byte((IMU_ADDR << 1) | 0) { i2c_stop(); return false; }
    if !i2c_write_byte(reg) { i2c_stop(); return false; }
    if !i2c_write_byte(val) { i2c_stop(); return false; }
    i2c_stop();
    true
}

/// Read one register.
unsafe fn read_reg(reg: u8) -> Option<u8> {
    i2c_start();
    if !i2c_write_byte((IMU_ADDR << 1) | 0) { i2c_stop(); return None; }
    if !i2c_write_byte(reg) { i2c_stop(); return None; }
    i2c_start(); // repeated start
    if !i2c_write_byte((IMU_ADDR << 1) | 1) { i2c_stop(); return None; }
    let val = i2c_read_byte(false); // NACK (single byte read)
    i2c_stop();
    Some(val)
}

/// Read multiple consecutive registers.
unsafe fn read_regs(start_reg: u8, buf: &mut [u8]) -> bool {
    if buf.is_empty() { return true; }
    i2c_start();
    if !i2c_write_byte((IMU_ADDR << 1) | 0) { i2c_stop(); return false; }
    if !i2c_write_byte(start_reg) { i2c_stop(); return false; }
    i2c_start(); // repeated start
    if !i2c_write_byte((IMU_ADDR << 1) | 1) { i2c_stop(); return false; }
    for i in 0..buf.len() {
        let last = i == buf.len() - 1;
        buf[i] = i2c_read_byte(!last); // ACK all except last
    }
    i2c_stop();
    true
}

// ── Public API ──

/// Initialize the IMU.  Returns WHO_AM_I value on success, 0 on failure.
pub fn init() -> u32 {
    if INITIALIZED.load(Ordering::Relaxed) {
        return WHO_AM_I_VAL as u32;
    }

    unsafe {
        gpio_init();

        // Check WHO_AM_I
        let who = match read_reg(WHO_AM_I) {
            Some(v) => v,
            None => return 0,
        };
        if who != WHO_AM_I_VAL {
            // Try alternate address 0x6B
            return 0;
        }

        // Configure accelerometer: 104 Hz, ±2g
        // CTRL1_XL = 0x40: ODR=104Hz(0100), FS=±2g(00), BW=400Hz(00)
        if !write_reg(CTRL1_XL, 0x40) { return 0; }

        // Configure gyroscope: 104 Hz, ±250 dps
        // CTRL2_G = 0x40: ODR=104Hz(0100), FS=250dps(00)
        if !write_reg(CTRL2_G, 0x40) { return 0; }

        INITIALIZED.store(true, Ordering::Release);
        who as u32
    }
}

/// Read 12 bytes: accel XYZ (6 bytes, i16 LE) + gyro XYZ (6 bytes, i16 LE).
/// Returns number of bytes written (12 on success, 0 on failure).
pub fn read(buf: &mut [u8]) -> u32 {
    if !INITIALIZED.load(Ordering::Relaxed) {
        return 0;
    }
    if buf.len() < 12 {
        return 0;
    }

    unsafe {
        // Read gyro (6 bytes from 0x22) then accel (6 bytes from 0x28)
        // Output order: accel XYZ, gyro XYZ (more intuitive)
        if !read_regs(OUTX_L_XL, &mut buf[0..6]) { return 0; }
        if !read_regs(OUTX_L_G, &mut buf[6..12]) { return 0; }
        12
    }
}

/// Check if IMU is initialized and responding.
pub fn is_ready() -> bool {
    INITIALIZED.load(Ordering::Relaxed)
}
