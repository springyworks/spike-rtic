#![allow(dead_code)]
//! Power management, battery charger, and DFU re-entry for LEGO SPIKE Prime Hub.
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
//! ## Battery charger (MP2639A)
//!
//! The charger is hardware-configured (resistors set charge voltage).
//! Firmware controls MODE (charge/discharge) via TLC5955 channel 14,
//! reads /CHG through a resistor ladder on ADC ch14, and drives the
//! ISET pin via TIM5 CH1 PWM on PA0 to set the charge current limit.
//! Without the ISET PWM, the pin floats and the charger delivers no
//! current — this is why charging only worked in the LEGO bootloader.
//!
//! The MP2639A has a built-in safety timer (typically 5–10 hours).
//! If the battery never reaches the termination threshold (because
//! the hub is actively running), the timer expires and the charger
//! enters a fault state (/CHG blinks 0.5 Hz).  To prevent this,
//! the firmware pauses and restarts charging every hour — exactly
//! as Pybricks does.
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
const RCC_APB1ENR: u32 = 0x40;

// ── TIM5 (ISET PWM for MP2639A charge current) ──
// Pybricks: TIM5 CH1 on PA0 (AF2), prescaler=10, period=100 → 96 kHz PWM.
// Duty 100 = max charge current, 15 = 500 mA USB, 2 = 100 mA, 0 = disable.
const TIM5: u32 = 0x4000_0C00;
const TIM_CR1: u32 = 0x00;
const TIM_CCMR1: u32 = 0x18;
const TIM_CCER: u32 = 0x20;
const TIM_PSC: u32 = 0x28;
const TIM_ARR: u32 = 0x2C;
const TIM_CCR1: u32 = 0x34;

/// Magic address and value checked by the LEGO/Rust bootloader.
const DFU_MAGIC_ADDR: *mut u32 = 0x2004_FFF0 as *mut u32;
const DFU_MAGIC_VALUE: u32 = 0xDEAD_B007;

// ── Battery charger (MP2639A) state machine ──

/// Charger status as decoded from the MP2639A /CHG (STAT) signal.
#[derive(Clone, Copy, PartialEq)]
pub enum ChargerStatus {
    /// No USB or charger disabled.
    Discharging,
    /// /CHG LOW — actively charging the battery.
    Charging,
    /// /CHG Hi-Z — charge cycle complete.
    Complete,
    /// /CHG blinking 0.5 Hz — NTC error or safety timer expired.
    Fault,
}

/// Persistent charger state, updated by `charger_tick()` every heartbeat.
pub struct ChargerState {
    /// Circular buffer of recent /CHG samples (true = charging).
    samples: [bool; 7],
    /// Next write position in `samples`.
    idx: u8,
    /// Number of heartbeat ticks with charger enabled (for timeout).
    charge_ticks: u32,
    /// Remaining pause ticks when restarting the charge cycle.
    pause_ticks: u32,
    /// Current decoded status.
    pub status: ChargerStatus,
    /// Whether the MODE pin is set to charge mode.
    mode_enabled: bool,
}

/// Restart charging every 60 minutes to reset the MP2639A safety timer.
const CHARGE_TIMEOUT_TICKS: u32 = 60 * 60 * 2; // 1 hour at 500 ms/tick
/// Pause charging for 30 seconds during the restart window.
const CHARGE_PAUSE_TICKS: u32 = 30 * 2; // 30 s at 500 ms/tick

impl ChargerState {
    pub const fn new() -> Self {
        Self {
            samples: [false; 7],
            idx: 0,
            charge_ticks: 0,
            pause_ticks: 0,
            status: ChargerStatus::Discharging,
            mode_enabled: false,
        }
    }

    /// Call every heartbeat tick (500 ms).  Manages the charger MODE pin,
    /// samples the /CHG signal, detects faults, and handles the periodic
    /// charge-restart cycle.
    pub fn tick(&mut self) {
        let usb = vbus_present();

        // Enable/disable charger based on USB presence.
        if usb && self.pause_ticks == 0 {
            if !self.mode_enabled {
                charger_enable(true);
                self.mode_enabled = true;
            }
        } else if !usb {
            if self.mode_enabled {
                charger_enable(false);
                self.mode_enabled = false;
            }
            self.status = ChargerStatus::Discharging;
            self.charge_ticks = 0;
            self.pause_ticks = 0;
            return;
        }

        // Handle pause period (charge cycle restart).
        if self.pause_ticks > 0 {
            self.pause_ticks -= 1;
            if self.pause_ticks == 0 {
                // Re-enable charging after pause.
                charger_enable(true);
                self.mode_enabled = true;
                self.charge_ticks = 0;
            }
            self.status = ChargerStatus::Discharging;
            return;
        }

        // Sample the /CHG signal from the resistor ladder.
        let (_btn, chg) = read_button_and_chg();
        self.samples[self.idx as usize] = chg;
        self.idx += 1;
        if self.idx >= self.samples.len() as u8 {
            self.idx = 0;
        }

        // Count transitions in the sample window for fault detection.
        // A physical 0.5 Hz blink creates ≥3 transitions in 7 × 500 ms = 3.5 s.
        let mut transitions = 0u8;
        for i in 1..self.samples.len() {
            if self.samples[i] != self.samples[i - 1] {
                transitions += 1;
            }
        }
        // Wrap-around edge.
        if self.samples[0] != self.samples[self.samples.len() - 1] {
            transitions += 1;
        }

        if transitions > 2 {
            self.status = ChargerStatus::Fault;
        } else if chg {
            self.status = ChargerStatus::Charging;
        } else if battery_mv() >= 8300 {
            // Battery near full and /CHG not active → charge complete.
            self.status = ChargerStatus::Complete;
        } else {
            // USB present, ISET active, but /CHG ladder marginal —
            // assume charging (confirmed by voltage-rise empirical data).
            self.status = ChargerStatus::Charging;
        }

        self.charge_ticks += 1;

        // Periodic charge restart to prevent safety-timer fault.
        if self.charge_ticks >= CHARGE_TIMEOUT_TICKS {
            charger_enable(false);
            self.mode_enabled = false;
            self.pause_ticks = CHARGE_PAUSE_TICKS;
        }
    }
}

/// Initialise TIM5 CH1 (PA0) as PWM output for the MP2639A ISET pin.
///
/// ISET controls the charge current limit.  Without this PWM, the ISET
/// pin floats and the charger won't deliver current while the hub is
/// running (charging only works in the LEGO bootloader which configures
/// TIM5).
///
/// Pybricks config: prescaler 10, period 100 → 96 kHz PWM from 96 MHz
/// APB1 timer clock.  Duty cycle 100 = max charge current.
///
/// # Safety
/// Must be called after clocks are initialised (APB1 at 48 MHz, timer
/// clock 96 MHz due to APB1 prescaler ≠ 1).
pub unsafe fn init_charger_iset() {
    // Enable TIM5 clock (APB1ENR bit 3).
    reg_modify(RCC, RCC_APB1ENR, 0, 1 << 3);
    let _ = reg_read(RCC, RCC_APB1ENR); // dummy read for clock propagation

    // PA0 → AF2 (TIM5_CH1).
    // GPIOA clock is already enabled (power_hold does this).
    // MODER[1:0] = 0b10 (alternate function)
    reg_modify(pins::GPIOA, pins::MODER, 3, 2);
    // AFRL[3:0] = 2 (AF2)
    reg_modify(pins::GPIOA, pins::AFRL, 0xF, 2);

    // Configure TIM5:
    //   PSC = 9  → 96 MHz / (9+1) = 9.6 MHz tick.
    //   ARR = 100 → 9.6 MHz / 100 = 96 kHz PWM.
    //   Pybricks stores prescalar=10 in platform data and writes PSC = prescalar-1.
    reg_write(TIM5, TIM_PSC, 9);    // prescaler register (divide by 10)
    reg_write(TIM5, TIM_ARR, 100);  // period (auto-reload)
    reg_write(TIM5, TIM_CCR1, 100); // duty = 100% (max charge current)

    // CCMR1: OC1M = 0b110 (PWM mode 1), OC1PE = 1 (preload enable)
    reg_write(TIM5, TIM_CCMR1, (0b110 << 4) | (1 << 3));
    // CCER: CC1E = 1 (output enable), CC1P = 0 (active high)
    reg_modify(TIM5, TIM_CCER, 0, 1 << 0);
    // CR1: CEN = 1 (counter enable), ARPE = 1 (auto-reload preload)
    reg_write(TIM5, TIM_CR1, (1 << 7) | (1 << 0));
    // EGR UG: force reload of shadow registers (PSC, ARR, CCR1)
    reg_write(TIM5, 0x14, 1 << 0); // EGR offset = 0x14, UG bit = 0

    // ── MODE kick: toggle MODE HIGH → LOW to restart the charge cycle.
    // The MP2639A won't restart charging after entering sleep/complete
    // unless it sees a rising edge on MODE (discharge) followed by
    // returning to LOW (charge).  The datasheet specifies ~100 ms for
    // mode transitions, so we hold HIGH for 500 ms to be safe.
    led_matrix::set_channel_raw(pins::CHARGER_MODE_CH as usize, 0xFFFF); // MODE HIGH
    led_matrix::update();
    delay_ms(500);
    led_matrix::set_channel_raw(pins::CHARGER_MODE_CH as usize, 0);      // MODE LOW
    led_matrix::update();
    delay_ms(200); // allow charger to settle into charge mode
}

/// Set the ISET PWM duty cycle (0–100).
///
/// 100 = maximum charge current, 0 = charging disabled.
fn iset_set_duty(duty: u32) {
    unsafe { reg_write(TIM5, TIM_CCR1, duty) };
}

/// Read the current ISET PWM duty cycle (0–100).
pub fn iset_duty() -> u32 {
    unsafe { reg_read(TIM5, TIM_CCR1) }
}

/// Enable or disable the MP2639A charger via TLC5955 channel 14
/// and ISET PWM duty cycle.
///
/// ch14 = 0     → MODE LOW  → charge (buck) mode.
/// ch14 = 0xFFFF → MODE HIGH → disabled / discharge (boost).
fn charger_enable(enable: bool) {
    let val: u16 = if enable { 0 } else { 0xFFFF };
    led_matrix::set_channel_raw(pins::CHARGER_MODE_CH as usize, val);
    // Set ISET: max current when charging, zero when disabled.
    iset_set_duty(if enable { 100 } else { 0 });
    // The next `led_matrix::update()` in heartbeat will latch the MODE pin.
}

/// Decode the resistor ladder on ADC ch14 and return
/// `(center_button_pressed, chg_active)`.
///
/// The /CHG signal is inverted: when /CHG pin is LOW the charger
/// is active, and the resistor ladder sets the CH_2 flag.
pub fn read_button_and_chg() -> (bool, bool) {
    let raw = crate::read_adc(14);
    let flags = decode_rlad(raw, &pins::RLAD_CENTER);
    let button  = flags & 0x01 != 0; // CH_0
    let chg     = flags & 0x04 != 0; // CH_2
    (button, chg)
}

/// Decode a resistor-ladder ADC reading into a 3-bit flags word.
///
/// The ladder encodes 3 active-low inputs as a Gray-code voltage
/// divider.  Thresholds are ordered highest to lowest; the decode
/// table mirrors Pybricks `resistor_ladder.c`.
fn decode_rlad(value: u32, levels: &[u32; 8]) -> u8 {
    if value > levels[0] { 0b000 }         // nothing
    else if value > levels[1] { 0b100 }    // CH_2 only
    else if value > levels[2] { 0b010 }    // CH_1 only
    else if value > levels[3] { 0b110 }    // CH_1 + CH_2
    else if value > levels[4] { 0b001 }    // CH_0 only
    else if value > levels[5] { 0b101 }    // CH_0 + CH_2
    else if value > levels[6] { 0b011 }    // CH_0 + CH_1
    else if value > levels[7] { 0b111 }    // all three
    else { 0 }                              // below range
}

/// Check whether the battery NTC thermistor is in the healthy range.
///
/// The MP2639A suspends charging when the NTC voltage leaves 30–74 %
/// of VCC.  Returns `true` if NTC is within that window.
pub fn ntc_ok() -> bool {
    let raw = crate::read_adc(8);
    (pins::NTC_LOW_THRESHOLD..=pins::NTC_HIGH_THRESHOLD).contains(&raw)
}

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
///
/// LED behaviour — Morse status on battery LED:
///   - Charging:  green Morse "CH" (C: -.-. H: ....)
///   - Complete:  green Morse "HI" (H: .... I: ..)
///   - Fault:     amber Morse "NG" (N: -.   G: --.)
///   - Smart overlay: status-top dim cyan pulse every 5 s (alive indicator).
fn charging_idle() -> ! {
    // PA13 LOW — doesn't cut power (USB keeps us alive) but
    // ensures we truly power off the instant USB is unplugged.
    unsafe {
        reg_write(pins::GPIOA, pins::BSRR, 1 << (13 + 16));
    }

    // Ensure charger is enabled in charge mode.
    charger_enable(true);

    let mut tick: u32 = 0;
    let mut chg_state = ChargerState::new();
    // Force mode_enabled so tick() doesn't re-set it redundantly.
    chg_state.mode_enabled = true;
    let mut morse = led_matrix::MorseBlinker::new();

    loop {
        crate::watchdog::feed();

        // Charger tick every 4 iterations (500 ms at 125 ms loop)
        if tick.is_multiple_of(4) {
            chg_state.tick();
        }

        // Update Morse pattern from charger status
        let morse_id = match chg_state.status {
            ChargerStatus::Charging    => led_matrix::MORSE_CHARGING,
            ChargerStatus::Complete    => led_matrix::MORSE_COMPLETE,
            ChargerStatus::Fault       => led_matrix::MORSE_FAULT,
            ChargerStatus::Discharging => led_matrix::MORSE_BATTERY,
        };
        morse.set_status(morse_id);

        // Bright/dim colours for Morse mark/space
        let (bright, dim): ((u16, u16, u16), (u16, u16, u16)) = match chg_state.status {
            ChargerStatus::Charging => (
                (0, 0x6000, 0), (0, 0x0800, 0)
            ),
            ChargerStatus::Complete => (
                (0, 0x4000, 0), (0, 0x1000, 0)
            ),
            ChargerStatus::Fault => (
                (0x6000, 0x2000, 0), (0x0800, 0x0400, 0)
            ),
            ChargerStatus::Discharging => (
                (0, 0, 0), (0, 0, 0)
            ),
        };

        let (r, g, b) = if morse.is_mark() { bright } else { dim };
        morse.advance();

        unsafe {
            led_matrix::set_status_rgb(pins::BATTERY_LED, r, g, b);

            // Smart overlay: brief dim cyan pulse on STATUS_TOP every 5 s
            // to show the hub is alive even though "off".
            // At 125 ms/tick, 40 ticks = 5 s.
            if tick.is_multiple_of(40) {
                led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0x1000, 0x1000);
            } else if tick % 40 == 1 {
                led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0, 0);
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

        delay_ms(125);
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
    // Voltage divider is 3:1 (200k/100k).  Pybricks uses 9900/4096.
    // Full-scale: ADC 4095 → 9900 mV ≈ 8.4 V (2S fully charged).
    raw * 9900 / 4096
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

/// 2S Li-ion critical low threshold (mV).  Below this the cells
/// risk damage and the hub should shut down immediately.
/// MP2639A UV cutoff is 5750 mV; we shut down slightly above.
pub const BATTERY_CRITICAL_MV: u32 = 5800;
