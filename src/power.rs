//! Power management and DFU re-entry for LEGO SPIKE Prime Hub.
//!
//! The hub's power is software-controlled:
//!   PA13 HIGH = powered on, PA13 LOW = instant shutdown.
//!   This pin must be asserted before *anything* else in init.
//!
//! PA14 enables 3.3 V to the I/O port connectors (motor/sensor).
//!
//! ## Shutdown architecture
//!
//! The hub uses a soft-latch power circuit: PA13 holds a MOSFET that
//! feeds the main 3.3 V rail.  Releasing PA13 (LOW) cuts power
//! completely — the MCU is de-energised and draws **zero** current.
//! Pressing the ring button re-latches power and boots the MCU.
//!
//! **Important**: when USB VBUS is present, the hub stays powered
//! through USB regardless of PA13.  True zero-current off requires
//! USB disconnected.
//!
//! `clean_shutdown()` turns off all peripherals (motors, LEDs, amp,
//! port VCC) before pulling PA13 LOW so nothing back-feeds current
//! through IO pins.
//!
//! ## DFU re-entry
//!
//! Write magic 0xDEADB007 to 0x2004_FFF0, then SYSRESETREQ.
//! The LEGO bootloader checks this address on boot and enters
//! USB DFU mode (VID:PID 0x0694:0x0011).

use crate::led_matrix;
use crate::motor;
use crate::pins;
use crate::reg::{reg_modify, reg_read, reg_write};
use crate::sound;

const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;

/// Magic address and value checked by the LEGO/Rust bootloader.
const DFU_MAGIC_ADDR: *mut u32 = 0x2004_FFF0 as *mut u32;
const DFU_MAGIC_VALUE: u32 = 0xDEAD_B007;

/// Assert PA13 HIGH to keep the hub powered on.
///
/// # Safety
/// Must be called as early as possible in init (before .bss/.data ideally).
pub unsafe fn power_hold() {
    // Enable GPIOA clock
    reg_modify(RCC, RCC_AHB1ENR, 0, 1 << 0);
    let _ = reg_read(RCC, RCC_AHB1ENR); // dummy read for clock propagation
    let _ = reg_read(RCC, RCC_AHB1ENR);

    // PA13 = output, push-pull, HIGH
    // Set BSRR *before* MODER to prevent a LOW glitch: if ODR[13]
    // is 0 (reset default) and we switch to output first, the pin
    // drives LOW momentarily which unlatches the power circuit.
    reg_write(pins::GPIOA, pins::BSRR, 1 << 13);           // ODR[13] = 1
    reg_modify(pins::GPIOA, pins::MODER, 3 << 26, 1 << 26); // output mode
}

/// Enable 3.3V to I/O ports (PA14).
pub unsafe fn enable_port_vcc() {
    reg_modify(pins::GPIOA, pins::MODER, 3 << 28, 1 << 28);
    reg_write(pins::GPIOA, pins::BSRR, 1 << 14);
}

/// Disable all peripherals to ensure zero parasitic current on shutdown.
unsafe fn peripherals_off() {
    // Stop all motors (coast)
    for i in 0..6u32 {
        motor::set(i, 0);
    }

    // Kill speaker
    sound::stop();

    // Clear entire LED matrix + status LEDs
    led_matrix::clear();

    // Disable port 3.3 V (PA14 LOW)
    reg_write(pins::GPIOA, pins::BSRR, 1 << (14 + 16));

    // Disable Bluetooth module (PA2 LOW)
    reg_modify(pins::GPIOA, pins::MODER, 3 << 4, 1 << 4); // output
    reg_write(pins::GPIOA, pins::BSRR, 1 << (2 + 16));

    // Disable USB OTG FS peripheral so host sees a disconnect.
    // OTG_FS_GCCFG: clear PWRDWN to cut the internal pull-up on DP,
    // then disable the OTG clock entirely.
    let otg_fs_gccfg: u32 = 0x5000_0038;
    core::ptr::write_volatile(otg_fs_gccfg as *mut u32, 0); // power down PHY
    // Disable OTG FS clock (RCC_AHB2ENR bit 7)
    let rcc_ahb2enr: u32 = 0x34;
    reg_modify(RCC, rcc_ahb2enr, 1 << 7, 0);

    // Small settle time
    delay_ms(50);
}

/// Check if USB VBUS is present (PA9 HIGH = USB connected).
///
/// PA9 is configured as input by the OTG init code.  When USB is
/// plugged in, VBUS drives PA9 HIGH through a voltage divider.
pub fn vbus_present() -> bool {
    unsafe { reg_read(pins::GPIOA, pins::IDR) & (1 << 9) != 0 }
}

/// Check if center button (ring button) is pressed.
/// Uses ADC ch14 — same method as `read_buttons()` in main.
fn center_button_pressed() -> bool {
    crate::read_adc(14) <= pins::BUTTON_CENTER_THRESHOLD
}

/// Clean power off: disable all peripherals, then release PA13.
///
/// If USB VBUS is present, enters a low-power charging idle loop
/// instead of powering off (because PA13 LOW has no effect while
/// USB keeps the MCU alive, and the IWDG would force a reboot).
///
/// Charging idle behavior:
///   - All peripherals off (motors, LEDs, sound, port VCC, BT, USB PHY)
///   - Battery LED: green 1s on / 1s off — visible charging indicator
///   - Watchdog fed every 200ms to prevent IWDG reset
///   - Button press → full reboot (SYSRESETREQ)
///   - USB removed → true PA13 LOW power off (zero current)
pub fn clean_shutdown() -> ! {
    unsafe { peripherals_off() };

    if vbus_present() {
        charging_idle();
    } else {
        shutdown();
    }
}

/// Minimal idle loop while USB is connected and user requested power-off.
///
/// Keeps the MCU alive (feeding watchdog) but with all peripherals
/// disabled, so almost all USB power goes to battery charging.
/// The battery LED blinks green to indicate charging state.
fn charging_idle() -> ! {
    // PA13 LOW — doesn't cut power (USB keeps us alive) but
    // ensures we truly power off the instant USB is unplugged.
    unsafe {
        reg_write(pins::GPIOA, pins::BSRR, 1 << (13 + 16));
    }

    let mut tick: u32 = 0;
    loop {
        crate::watchdog::feed();

        // Blink battery LED green: on for 1s, off for 1s (5 ticks each at 200ms)
        let phase = (tick / 5) % 2;
        unsafe {
            if phase == 0 {
                led_matrix::set_status_rgb(pins::BATTERY_LED, 0, 0x6000, 0);
            } else {
                led_matrix::set_status_rgb(pins::BATTERY_LED, 0, 0, 0);
            }
            led_matrix::update();
        }

        // Check exit conditions
        if center_button_pressed() {
            // User wants to boot — full reset
            cortex_m::peripheral::SCB::sys_reset();
        }

        if !vbus_present() {
            // USB unplugged — true power off now
            shutdown();
        }

        delay_ms(200);
        tick += 1;
    }
}

/// Power off the hub by releasing PA13.
pub fn shutdown() -> ! {
    unsafe {
        reg_write(pins::GPIOA, pins::BSRR, 1 << (13 + 16)); // PA13 LOW
    }
    loop {
        cortex_m::asm::wfi();
    }
}



/// Write DFU magic to RAM and reset into the LEGO bootloader.
/// The hub will enumerate as USB DFU device VID:PID 0x0694:0x0011.
/// Flash with: `dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D firmware.bin`
pub fn enter_dfu() -> ! {
    unsafe {
        // Disconnect USB so host sees a clean detach before the
        // bootloader re-enumerates as the DFU device.
        let otg_fs_gccfg: u32 = 0x5000_0038;
        core::ptr::write_volatile(otg_fs_gccfg as *mut u32, 0);
        delay_ms(20);

        // Write DFU magic — bootloader checks this on startup.
        core::ptr::write_volatile(DFU_MAGIC_ADDR, DFU_MAGIC_VALUE);
        cortex_m::asm::dsb(); // ensure write reaches SRAM
    }
    cortex_m::peripheral::SCB::sys_reset();
}

/// Simple busy-wait delay (calibrated for 96 MHz HCLK).
pub fn delay_ms(ms: u32) {
    for _ in 0..ms {
        cortex_m::asm::delay(96_000);
    }
}

/// Read battery voltage via ADC ch11 and return millivolts.
pub fn battery_mv() -> u32 {
    let raw = crate::read_adc(11);
    // Voltage divider: Vbat → 100k/100k, so Vbat = raw * 3300 / 4095 * 2
    // Simplified: raw * 6600 / 4095
    raw * 6600 / 4095
}

/// Power off with a descending chime and purple LED flash.
///
/// The SPIKE Prime hub has no intermediate sleep state — the MCU
/// is either fully running or fully off (PA13 LOW, zero current).
/// Pressing the ring button re-latches the soft-latch MOSFET and
/// boots the MCU from scratch.
///
/// Does **not** return.
pub fn deep_sleep() -> ! {
    unsafe {
        // Stop motors
        for i in 0..6u32 {
            motor::set(i, 0);
        }

        // Purple status LED
        led_matrix::set_status_rgb(pins::STATUS_TOP, 0x8000, 0, 0xFFFF);
        led_matrix::update();

        // Descending chime
        sound::play(660);
        delay_ms(80);
        sound::play(440);
        delay_ms(80);
        sound::play(330);
        delay_ms(80);
        sound::stop();
    }

    // Full power off — wake is a cold boot via ring button
    clean_shutdown();
}

/// Li-ion critical low threshold (mV).  Below this the battery
/// risks damage and the hub should shut down immediately.
pub const BATTERY_CRITICAL_MV: u32 = 3400;
