#![allow(dead_code)]
//! W25Q256JV external SPI flash driver (32 MB).
//!
//! The LEGO SPIKE Prime Hub has a Winbond W25Q256JV on SPI2:
//!   - CS:   PB12
//!   - SCK:  PB13
//!   - MISO: PC2
//!   - MOSI: PC3
//!
//! This driver provides:
//!   - JEDEC ID read (chip identification)
//!   - Status register read
//!   - Byte/page read (up to 32 MB address space)
//!   - Page program (256-byte pages)
//!   - Sector erase (4 KB), block erase (64 KB), chip erase
//!
//! The W25Q256JV uses 4-byte addressing (required for >16 MB).
//! All operations use standard SPI mode 0 (CPOL=0, CPHA=0).

use crate::pins;
use crate::reg::{reg_modify, reg_read, reg_write};

// ── SPI2 base ──────────────────────────────────────────────
const SPI2: u32 = 0x4000_3800;
const SPI_CR1: u32 = 0x00;
const SPI_CR2: u32 = 0x04;
const SPI_SR: u32 = 0x08;
const SPI_DR: u32 = 0x0C;

// ── RCC ────────────────────────────────────────────────────
const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_APB1ENR: u32 = 0x40;

// ── W25Q commands ──────────────────────────────────────────
const CMD_JEDEC_ID: u8 = 0x9F;
const CMD_READ_SR1: u8 = 0x05;
const CMD_READ_SR2: u8 = 0x35;
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_READ_DATA_4B: u8 = 0x13; // 4-byte address read
const CMD_PAGE_PROGRAM_4B: u8 = 0x12; // 4-byte address page program
const CMD_SECTOR_ERASE_4B: u8 = 0x21; // 4-byte address 4KB erase
const CMD_BLOCK_ERASE_64K_4B: u8 = 0xDC; // 4-byte address 64KB erase
const CMD_CHIP_ERASE: u8 = 0xC7;
const CMD_ENTER_4BYTE: u8 = 0xB7;
const CMD_POWER_UP: u8 = 0xAB;
const CMD_POWER_DOWN: u8 = 0xB9;
const CMD_READ_SR3: u8 = 0x15;
const CMD_WRITE_SR1: u8 = 0x01; // Write Status Register 1 (+ optional SR2)
const CMD_WRITE_SR2: u8 = 0x31; // Write Status Register 2
const CMD_WRITE_SR3: u8 = 0x11; // Write Status Register 3
const CMD_VOLATILE_SR_WRITE_ENABLE: u8 = 0x50;
const CMD_GLOBAL_BLOCK_UNLOCK: u8 = 0x98;

// ── Status bits ────────────────────────────────────────────
const SR1_BUSY: u8 = 0x01;
const SR1_WEL: u8 = 0x02;  // Write Enable Latch
// SR1 bits[6:2] = SEC, TB, BP2, BP1, BP0 — block protect
const SR1_BP_MASK: u8 = 0x7C; // bits 2-6
// SR3 bit 2 = WPS (Write Protect Selection): 0=use BP bits, 1=use individual block locks
const SR3_WPS: u8 = 0x04;

// ── Flash geometry ─────────────────────────────────────────
/// Total flash size in bytes (32 MB).
pub const FLASH_SIZE: u32 = 32 * 1024 * 1024;
/// Page size (256 bytes) — smallest writable unit.
pub const PAGE_SIZE: u32 = 256;
/// Sector size (4 KB) — smallest erasable unit.
pub const SECTOR_SIZE: u32 = 4 * 1024;
/// Block size (64 KB).
pub const BLOCK_SIZE: u32 = 64 * 1024;

/// JEDEC ID for W25Q256JV: manufacturer 0xEF, device 0x4019.
pub const EXPECTED_JEDEC: u32 = 0x00EF_4019;

// ── Low-level SPI helpers ──────────────────────────────────

fn cs_low() {
    // PB12 = 0 (active low)
    unsafe {
        reg_write(
            pins::FLASH_CS_PORT,
            pins::BSRR,
            1 << (pins::FLASH_CS_PIN + 16),
        );
    }
}

fn cs_high() {
    // PB12 = 1
    unsafe {
        reg_write(pins::FLASH_CS_PORT, pins::BSRR, 1 << pins::FLASH_CS_PIN);
    }
}

/// Transfer one byte over SPI2 (full duplex).
fn spi_xfer(tx: u8) -> u8 {
    unsafe {
        // Wait TXE
        while reg_read(SPI2, SPI_SR) & (1 << 1) == 0 {}
        // Write byte (DR is 8-bit accessible, but we use 32-bit reg)
        core::ptr::write_volatile((SPI2 + SPI_DR) as *mut u8, tx);
        // Wait RXNE
        while reg_read(SPI2, SPI_SR) & (1 << 0) == 0 {}
        core::ptr::read_volatile((SPI2 + SPI_DR) as *const u8)
    }
}

/// Send a byte, discard the received byte.
#[inline]
fn spi_send(b: u8) {
    let _ = spi_xfer(b);
}

/// Receive a byte (sends 0xFF as dummy).
#[inline]
fn spi_recv() -> u8 {
    spi_xfer(0xFF)
}

// ── Initialisation ─────────────────────────────────────────

/// Initialise SPI2 and the W25Q256 flash.
///
/// # Safety
/// Call once during early init.  Modifies RCC, GPIO, and SPI2 registers.
pub unsafe fn init() {
    // Enable clocks: GPIOB, GPIOC, SPI2
    reg_modify(RCC, RCC_AHB1ENR, 0, (1 << 1) | (1 << 2)); // GPIOB + GPIOC
    reg_modify(RCC, RCC_APB1ENR, 0, 1 << 14); // SPI2 on APB1
    let _ = reg_read(RCC, RCC_APB1ENR); // readback barrier

    // ── CS pin (PB12): output push-pull, high (deselected) ──
    cs_high();
    reg_modify(
        pins::FLASH_CS_PORT,
        pins::MODER,
        3 << (pins::FLASH_CS_PIN * 2),
        1 << (pins::FLASH_CS_PIN * 2), // output
    );
    reg_modify(
        pins::FLASH_CS_PORT,
        pins::OSPEEDR,
        3 << (pins::FLASH_CS_PIN * 2),
        3 << (pins::FLASH_CS_PIN * 2), // very high speed
    );

    // ── SCK pin (PB13): AF5 (SPI2) ──
    reg_modify(
        pins::FLASH_SCK_PORT,
        pins::MODER,
        3 << (pins::FLASH_SCK_PIN * 2),
        2 << (pins::FLASH_SCK_PIN * 2), // alternate function
    );
    reg_modify(
        pins::FLASH_SCK_PORT,
        pins::OSPEEDR,
        3 << (pins::FLASH_SCK_PIN * 2),
        3 << (pins::FLASH_SCK_PIN * 2),
    );
    // PB13 is in AFRH (pin 13, bit group [23:20])
    reg_modify(
        pins::FLASH_SCK_PORT,
        pins::AFRH,
        0xF << ((pins::FLASH_SCK_PIN - 8) * 4),
        5 << ((pins::FLASH_SCK_PIN - 8) * 4), // AF5
    );

    // ── MISO pin (PC2): AF5 (SPI2), input (pull-up) ──
    reg_modify(
        pins::FLASH_MISO_PORT,
        pins::MODER,
        3 << (pins::FLASH_MISO_PIN * 2),
        2 << (pins::FLASH_MISO_PIN * 2),
    );
    reg_modify(
        pins::FLASH_MISO_PORT,
        pins::PUPDR,
        3 << (pins::FLASH_MISO_PIN * 2),
        1 << (pins::FLASH_MISO_PIN * 2), // pull-up
    );
    // PC2 is in AFRL (pin 2, bit group [11:8])
    reg_modify(
        pins::FLASH_MISO_PORT,
        pins::AFRL,
        0xF << (pins::FLASH_MISO_PIN * 4),
        5 << (pins::FLASH_MISO_PIN * 4), // AF5
    );

    // ── MOSI pin (PC3): AF5 (SPI2) ──
    reg_modify(
        pins::FLASH_MOSI_PORT,
        pins::MODER,
        3 << (pins::FLASH_MOSI_PIN * 2),
        2 << (pins::FLASH_MOSI_PIN * 2),
    );
    reg_modify(
        pins::FLASH_MOSI_PORT,
        pins::OSPEEDR,
        3 << (pins::FLASH_MOSI_PIN * 2),
        3 << (pins::FLASH_MOSI_PIN * 2),
    );
    reg_modify(
        pins::FLASH_MOSI_PORT,
        pins::AFRL,
        0xF << (pins::FLASH_MOSI_PIN * 4),
        5 << (pins::FLASH_MOSI_PIN * 4), // AF5
    );

    // ── SPI2 configuration ──
    // Master mode, software NSS, 8-bit, CPOL=0 CPHA=0
    // APB1 = 48 MHz, prescaler /4 = 12 MHz SPI clock
    // (W25Q256JV supports up to 133 MHz, so 12 MHz is conservative)
    reg_write(SPI2, SPI_CR1, 0); // reset
    reg_write(
        SPI2,
        SPI_CR1,
        (1 << 2)  // MSTR: master
        | (1 << 8)  // SSI: internal slave select high
        | (1 << 9)  // SSM: software slave management
        | (1 << 6)  // SPE: enable
        | (1 << 3), // BR = /4 (001 << 3 = 0x08, but bit 3 alone = /4... actually:
                     // BR[2:0] = 001 → /4.  bit 3 is BR[0], bits 4:5 are BR[1:2]
                     // 001 = 0x08 shifted... Let me be explicit:
    );

    // Redo: CR1 bits for BR[2:0] at positions [5:3]:
    //   000 = /2, 001 = /4, 010 = /8, 011 = /16, ...
    // We want /4 → BR = 001 → bits [5:3] = 0b001 → (1 << 3)
    let cr1 = (1 << 2)        // MSTR
            | (0b001 << 3)    // BR = /4 → 48/4 = 12 MHz
            | (1 << 8)        // SSI
            | (1 << 9)        // SSM
            | (1 << 6);       // SPE
    reg_write(SPI2, SPI_CR1, cr1);
    reg_write(SPI2, SPI_CR2, 0);

    // Release from deep power-down
    cs_low();
    spi_send(CMD_POWER_UP);
    cs_high();

    // Short delay for wake-up (tRES1 = 3 µs typical)
    for _ in 0..1000 {
        cortex_m::asm::nop();
    }

    // Enter 4-byte address mode (required for >16 MB)
    cs_low();
    spi_send(CMD_ENTER_4BYTE);
    cs_high();

    // Clear any write protection left by the LEGO factory firmware
    unlock();
}

// ── Chip identification ────────────────────────────────────

/// Read JEDEC ID (manufacturer + device).
/// Returns 3 bytes packed: [MFR, DEV_HI, DEV_LO] → 0x00_EF_40_19 for W25Q256JV.
pub fn read_jedec_id() -> u32 {
    cs_low();
    spi_send(CMD_JEDEC_ID);
    let mfr = spi_recv();
    let dev_hi = spi_recv();
    let dev_lo = spi_recv();
    cs_high();
    ((mfr as u32) << 16) | ((dev_hi as u32) << 8) | (dev_lo as u32)
}

/// Read status register 1.
pub fn read_sr1() -> u8 {
    cs_low();
    spi_send(CMD_READ_SR1);
    let sr = spi_recv();
    cs_high();
    sr
}

/// Read status register 2.
pub fn read_sr2() -> u8 {
    cs_low();
    spi_send(CMD_READ_SR2);
    let sr = spi_recv();
    cs_high();
    sr
}

/// Read status register 3.
pub fn read_sr3() -> u8 {
    cs_low();
    spi_send(CMD_READ_SR3);
    let sr = spi_recv();
    cs_high();
    sr
}

/// Write status register 1 (clears BP bits to unlock sectors).
fn write_sr1(val: u8) {
    write_enable();
    cs_low();
    spi_send(CMD_WRITE_SR1);
    spi_send(val);
    cs_high();
    wait_ready(100_000);
}

/// Write status register 2.
#[allow(dead_code)]
fn write_sr2(val: u8) {
    write_enable();
    cs_low();
    spi_send(CMD_WRITE_SR2);
    spi_send(val);
    cs_high();
    wait_ready(100_000);
}

/// Write status register 3.
#[allow(dead_code)]
fn write_sr3(val: u8) {
    write_enable();
    cs_low();
    spi_send(CMD_WRITE_SR3);
    spi_send(val);
    cs_high();
    wait_ready(100_000);
}

/// Remove all write protection from the flash.
///
/// The LEGO factory firmware typically sets BP0-BP3 in SR1 to
/// protect the entire flash.  This function:
///   1. Clears BP bits in SR1 (block protect = none)
///   2. If WPS bit is set in SR3, issues Global Block Unlock
///
/// After calling this, all sectors are writable.
pub fn unlock() {
    let sr1 = read_sr1();
    let sr3 = read_sr3();

    // If any block-protect bits are set, clear them
    if sr1 & SR1_BP_MASK != 0 {
        // Write SR1 with BP bits cleared, preserving SRP0 (bit 7)
        let new_sr1 = sr1 & !SR1_BP_MASK;
        write_sr1(new_sr1);
    }

    // If WPS=1, individual block locks are active → issue global unlock
    if sr3 & SR3_WPS != 0 {
        write_enable();
        cs_low();
        spi_send(CMD_GLOBAL_BLOCK_UNLOCK);
        cs_high();
        wait_ready(100_000);
    }
}

/// Check if the flash is busy (write/erase in progress).
#[inline]
pub fn is_busy() -> bool {
    read_sr1() & SR1_BUSY != 0
}

/// Poll until flash is not busy, with a maximum iteration count.
/// Returns true if ready, false if timeout.
pub fn wait_ready(max_polls: u32) -> bool {
    for _ in 0..max_polls {
        if !is_busy() {
            return true;
        }
    }
    false
}

// ── Write enable ───────────────────────────────────────────

fn write_enable() {
    cs_low();
    spi_send(CMD_WRITE_ENABLE);
    cs_high();
    // W25Q256JV requires tSHSL >= 50 ns (CS# high time) before next command.
    // At 96 MHz (~10 ns/cycle), 6 NOPs provide ~62 ns margin.
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();
}

/// Diagnostic: try write_enable and return (WEL_before, WEL_after, SR1_full).
pub fn diag_write_enable() -> (bool, bool, u8) {
    let sr_before = read_sr1();
    let wel_before = sr_before & SR1_WEL != 0;
    write_enable();
    let sr_after = read_sr1();
    let wel_after = sr_after & SR1_WEL != 0;
    (wel_before, wel_after, sr_after)
}

/// Diagnostic: erase sector 0 and write+read test pattern.
/// Returns (erase_worked, write_worked, sr1_after_wel, read_back).
pub fn diag_write_test() -> (bool, bool, u8, [u8; 4]) {
    // Erase sector 0
    sector_erase(0);

    // Verify erased
    let mut buf = [0u8; 4];
    read(0, &mut buf);
    let erase_ok = buf == [0xFF, 0xFF, 0xFF, 0xFF];

    // Now do write_enable and check WEL
    write_enable();
    let sr1 = read_sr1();

    // Do page program manually (inline, not via page_program fn)
    // so we can be sure of the sequence
    cs_low();
    spi_send(CMD_PAGE_PROGRAM_4B);  // 0x12
    spi_send(0x00); // addr[31:24]
    spi_send(0x00); // addr[23:16]
    spi_send(0x00); // addr[15:8]
    spi_send(0x00); // addr[7:0]
    spi_send(0xDE);
    spi_send(0xAD);
    spi_send(0xBE);
    spi_send(0xEF);
    cs_high();

    // Wait for completion
    wait_ready(100_000);

    // Read back
    let mut result = [0u8; 4];
    read(0, &mut result);
    let write_ok = result == [0xDE, 0xAD, 0xBE, 0xEF];

    (erase_ok, write_ok, sr1, result)
}

/// Diagnostic: test the page_program() function (not inline) at address 0.
/// Erases sector 0, writes 4 bytes via page_program(), reads back.
/// Returns (erase_ok, write_ok, readback).
pub fn diag_page_program_test() -> (bool, bool, [u8; 4]) {
    sector_erase(0);
    let mut buf = [0u8; 4];
    read(0, &mut buf);
    let erase_ok = buf == [0xFF; 4];

    let pattern: [u8; 4] = [0xCA, 0xFE, 0xBA, 0xBE];
    page_program(0, &pattern);

    let mut result = [0u8; 4];
    read(0, &mut result);
    let write_ok = result == pattern;

    (erase_ok, write_ok, result)
}

/// Diagnostic: test the write() function (multi-page) at address 0.
/// Erases sector 0, writes 512 bytes (2 pages) via write(), reads back first 8 bytes.
/// Returns (erase_ok, bytes_match, first_8_readback).
pub fn diag_write_multi_test() -> (bool, bool, [u8; 8]) {
    sector_erase(0);
    let mut buf = [0u8; 4];
    read(0, &mut buf);
    let erase_ok = buf == [0xFF; 4];

    // Create a test pattern: 512 bytes incrementing
    let mut data = [0u8; 512];
    for (i, b) in data.iter_mut().enumerate() {
        *b = i as u8;
    }
    write(0, &data);

    let mut result = [0u8; 8];
    read(0, &mut result);
    let expected = [0u8, 1, 2, 3, 4, 5, 6, 7];
    let ok = result == expected;

    (erase_ok, ok, result)
}

// ── Read ───────────────────────────────────────────────────

/// Read `buf.len()` bytes from the given 4-byte address.
///
/// Address range: 0x0000_0000 .. 0x01FF_FFFF (32 MB).
pub fn read(addr: u32, buf: &mut [u8]) {
    cs_low();
    spi_send(CMD_READ_DATA_4B);
    spi_send((addr >> 24) as u8);
    spi_send((addr >> 16) as u8);
    spi_send((addr >> 8) as u8);
    spi_send(addr as u8);
    for b in buf.iter_mut() {
        *b = spi_recv();
    }
    cs_high();
}

// ── Write (page program) ──────────────────────────────────

/// Program up to 256 bytes within a single page.
///
/// The caller must ensure:
///   - `addr` is page-aligned (or data fits within the page boundary)
///   - The target region has been erased (all 0xFF)
///   - `data.len() <= 256`
///
/// Blocks until the write completes.
pub fn page_program(addr: u32, data: &[u8]) {
    if data.is_empty() || data.len() > PAGE_SIZE as usize {
        return;
    }
    write_enable();
    cs_low();
    spi_send(CMD_PAGE_PROGRAM_4B);
    spi_send((addr >> 24) as u8);
    spi_send((addr >> 16) as u8);
    spi_send((addr >> 8) as u8);
    spi_send(addr as u8);
    for &b in data {
        spi_send(b);
    }
    cs_high();
    // Page program takes max 3 ms
    wait_ready(100_000);
}

// ── Erase ──────────────────────────────────────────────────

/// Erase a 4 KB sector containing `addr`.
///
/// Blocks until erase completes (typical 45 ms, max 400 ms).
pub fn sector_erase(addr: u32) {
    write_enable();
    cs_low();
    spi_send(CMD_SECTOR_ERASE_4B);
    spi_send((addr >> 24) as u8);
    spi_send((addr >> 16) as u8);
    spi_send((addr >> 8) as u8);
    spi_send(addr as u8);
    cs_high();
    wait_ready(5_000_000); // up to 400 ms
}

/// Erase a 64 KB block containing `addr`.
///
/// Blocks until erase completes (typical 150 ms, max 2000 ms).
pub fn block_erase(addr: u32) {
    write_enable();
    cs_low();
    spi_send(CMD_BLOCK_ERASE_64K_4B);
    spi_send((addr >> 24) as u8);
    spi_send((addr >> 16) as u8);
    spi_send((addr >> 8) as u8);
    spi_send(addr as u8);
    cs_high();
    wait_ready(50_000_000); // up to 2 s
}

/// Erase entire chip (all 32 MB → 0xFF).
///
/// **WARNING**: takes 40–200 seconds.  Blocks while waiting.
pub fn chip_erase() {
    write_enable();
    cs_low();
    spi_send(CMD_CHIP_ERASE);
    cs_high();
    // Chip erase can take up to 200 s.  We poll in a loop.
    // At ~1 µs per poll, 500M polls ≈ 500 s headroom.
    wait_ready(500_000_000);
}

// ── Write multi-page ───────────────────────────────────────

/// Write arbitrary-length data across page boundaries.
///
/// Automatically handles page-aligned writes.  The target region
/// must be erased beforehand.
pub fn write(mut addr: u32, data: &[u8]) {
    let mut offset = 0usize;
    while offset < data.len() {
        // Bytes remaining in current page
        let page_remain = (PAGE_SIZE - (addr % PAGE_SIZE)) as usize;
        let chunk = core::cmp::min(page_remain, data.len() - offset);
        page_program(addr, &data[offset..offset + chunk]);
        addr += chunk as u32;
        offset += chunk;
    }
}

// ── Power management ───────────────────────────────────────

/// Put the flash into deep power-down mode (~1 µA).
pub fn power_down() {
    cs_low();
    spi_send(CMD_POWER_DOWN);
    cs_high();
}

/// Wake the flash from deep power-down.
pub fn power_up() {
    cs_low();
    spi_send(CMD_POWER_UP);
    cs_high();
    // tRES1 = 3 µs
    for _ in 0..1000 {
        cortex_m::asm::nop();
    }
}
