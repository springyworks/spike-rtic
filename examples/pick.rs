
//! Dummy example — the cargo runner intercepts this and launches the
//! interactive demo picker (`cargo example`).
//!
//! Usage:
//!   cargo run --example pick
//!
//! This exists so `cargo run --example` lists something useful.
//! The actual binary is never executed on hardware.

#![no_std]
#![no_main]

use panic_halt as _;
use stm32f4 as _; // pull in interrupt vectors

#[cortex_m_rt::entry]
fn main() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
