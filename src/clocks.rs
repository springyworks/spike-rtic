//! Clock configuration for LEGO SPIKE Prime Hub.
//!
//! HSE = 16 MHz crystal → PLL → 96 MHz SYSCLK, 48 MHz USB.
//!
//! PLL config: M=16, N=192, P=2, Q=4
//!   VCO_in  = 16 MHz / 16 = 1 MHz
//!   VCO_out = 1 MHz × 192 = 192 MHz
//!   SYSCLK  = 192 MHz / 2 = 96 MHz
//!   USB_CLK = 192 MHz / 4 = 48 MHz
//!
//! Bus clocks: APB1 = 48 MHz (/2), APB2 = 96 MHz (/1)

use crate::reg::{reg_modify, reg_read, reg_write};

const RCC: u32 = 0x4002_3800;
const FLASH_R: u32 = 0x4002_3C00;

// RCC registers
const CR: u32 = 0x00;
const PLLCFGR: u32 = 0x04;
const CFGR: u32 = 0x08;

// Flash
const ACR: u32 = 0x00;

/// Initialize clocks: HSE 16 MHz → PLL → 96 MHz SYSCLK.
///
/// # Safety
/// Must be called once during early init. Modifies RCC and FLASH registers.
pub unsafe fn init_clocks() {
    // 3 wait states for 96 MHz at 3.3V (2.7–3.6V range)
    reg_modify(FLASH_R, ACR, 0xF, 3);

    // Enable HSE (16 MHz crystal)
    reg_modify(RCC, CR, 0, 1 << 16); // HSEON
    while reg_read(RCC, CR) & (1 << 17) == 0 {} // wait HSERDY

    // Configure PLL: M=16, N=192, P=/2, Q=4, source=HSE
    #[allow(clippy::identity_op)]
    let pllcfgr = 16          // PLLM = 16
        | (192 << 6)          // PLLN = 192
        | (0 << 16)           // PLLP = /2 (0 = /2)
        | (1 << 22)           // PLLSRC = HSE
        | (4 << 24);          // PLLQ = 4 (for USB 48 MHz)
    reg_write(RCC, PLLCFGR, pllcfgr);

    // Enable PLL
    reg_modify(RCC, CR, 0, 1 << 24); // PLLON
    while reg_read(RCC, CR) & (1 << 25) == 0 {} // wait PLLRDY

    // APB1 prescaler = /2 (max 50 MHz for APB1), APB2 = /1
    reg_modify(RCC, CFGR, 0xFCF0, 0x1000); // PPRE1 = /2

    // Switch system clock to PLL
    reg_modify(RCC, CFGR, 0x3, 0x2); // SW = PLL
    while (reg_read(RCC, CFGR) & 0xC) != 0x8 {} // wait SWS = PLL
}
