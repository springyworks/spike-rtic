//! RTIC v2 firmware for LEGO SPIKE Prime Hub (STM32F413VGT6).
//!
//! This is a bare-metal debug/development firmware that exposes the
//! hub's hardware — 5×5 LED matrix, buttons, battery, flash —
//! through an interactive serial shell over USB CDC-ACM.  A Demon-style
//! resident debug monitor with GDB RSP support — inspired by the classic
//! ARM Demon ROM monitor.
//!
//! ## Hardware
//!   - MCU: STM32F413VGT6 — Cortex-M4F @ 96 MHz, 1 MB Flash,
//!     320 KB SRAM (256 KB SRAM1 + 64 KB SRAM2)
//!   - LED matrix: TLC5955 48-channel LED driver on SPI1
//!   - USB: OTG FS (CDC-ACM), VID:PID 0x0694:0x0042
//!   - Buttons: center (ADC ch14 on PC4), left/right/BT resistor
//!     ladder (ADC ch1 on PA1)
//!   - Battery: voltage (ch11), current (ch10), NTC (ch8),
//!     USB charger (ch3)
//!
//! ## Flash via DFU
//!   1. Enter DFU: hold center button during power-on, or `dfu` cmd,
//!      or hold left button while running.
//!   2. `dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D firmware.bin`
//!   (Hub uses LEGO DFU bootloader at VID:PID 0x0694:0x0011)
//!
//! ## Connect to the shell
//!   picocom /dev/ttyACM0   (or screen, minicom, PuTTY, etc.)
//!   Type `help` for available commands.

#![no_std]
#![no_main]

use panic_halt as _;

/// Earliest possible PA13 power-latch assert.
///
/// Runs *before* .bss/.data init — the very first user code after
/// reset.  Without this, booting from battery (no USB VBUS) can
/// lose power before `init()` reaches `power::power_hold()`.
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    power::power_hold();
}

mod clocks;
mod user_app_io;
mod ext_flash;
mod imu;
mod led_matrix;
mod dwt;
mod fpb;
mod gdb_rsp;
mod motor;
mod pins;
mod power;
mod ram_test;
mod reg;
mod rtty;
mod sandbox;
mod sensor;
mod servo;
mod shell;
mod sound;
mod task_state;
mod trace;
mod upload;
mod usb_serial;
mod watchdog;

use reg::{reg_modify, reg_read, reg_write};

// ── Global pause/continue state ──
static PAUSED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

pub fn is_paused() -> bool {
    PAUSED.load(core::sync::atomic::Ordering::Relaxed)
}

pub fn toggle_pause() -> bool {
    let was = PAUSED.fetch_xor(true, core::sync::atomic::Ordering::Relaxed);
    !was // returns new state: true = now paused
}

// ── Peripheral base addresses ──
const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_APB2ENR: u32 = 0x44;
const ADC1: u32 = 0x4001_2000;
const ADC_SR: u32 = 0x00;
const ADC_CR2: u32 = 0x08;
const ADC_SMPR1: u32 = 0x0C;
const ADC_SMPR2: u32 = 0x10;
const ADC_SQR1: u32 = 0x2C;
const ADC_SQR3: u32 = 0x34;
const ADC_DR: u32 = 0x4C;
const ADC_CCR: u32 = 0x04; // Common control register (at ADC1 + 0x300)

// ── Button flags (re-exported from shared API crate) ──
pub use spike_hub_api::{BTN_CENTER, BTN_LEFT, BTN_RIGHT};

/// Initialize ADC1 for button and battery readings.
unsafe fn init_button_adc() {
    reg_modify(RCC, RCC_AHB1ENR, 0, (1 << 0) | (1 << 1) | (1 << 2)); // GPIOA+B+C
    reg_modify(RCC, RCC_APB2ENR, 0, 1 << 8); // ADC1
    let _ = reg_read(RCC, RCC_APB2ENR);

    // Configure all used analog pins to MODER=0b11 (analog):
    // PA1 (ch1)  — button left/right/BT ladder
    // PA3 (ch3)  — USB charger current sense
    reg_modify(pins::GPIOA, pins::MODER, (3 << 2) | (3 << 6),
                                          (3 << 2) | (3 << 6));
    // PB0 (ch8)  — battery NTC thermistor
    reg_modify(pins::GPIOB, pins::MODER, 3 << 0, 3 << 0);
    // PC0 (ch10) — battery current
    // PC1 (ch11) — battery voltage
    // PC4 (ch14) — center button + /CHG resistor ladder
    reg_modify(pins::GPIOC, pins::MODER,
        (3 << 0) | (3 << 2) | (3 << 8),
        (3 << 0) | (3 << 2) | (3 << 8));

    // Single conversion, 480-cycle sample time for all analog channels.
    // SMPR2 covers ch0–ch9 (3 bits each).
    // SMPR1 covers ch10–ch18 (3 bits each).
    // Set all to 7 (480 cycles) — the whole register.
    reg_write(ADC1, ADC_SQR1, 0);
    reg_write(ADC1, ADC_SMPR2, 0x3FFF_FFFF); // ch0–ch9 all 480 cycles
    reg_write(ADC1, ADC_SMPR1, 0x07FF_FFFF); // ch10–ch17 all 480 cycles
    // Enable internal temp sensor + VREFINT (TSVREFE bit in ADC_CCR)
    reg_modify(ADC1 + 0x300, ADC_CCR, 0, 1 << 23); // TSVREFE = 1
    reg_modify(ADC1, ADC_CR2, 0, 1 << 0); // ADON
}

/// Read a single ADC channel (blocking).
pub fn read_adc(channel: u32) -> u32 {
    unsafe {
        reg_write(ADC1, ADC_SQR3, channel);
        // Dummy conversion: after switching channels the S/H cap
        // still holds the previous channel's charge.  Discard it.
        reg_modify(ADC1, ADC_CR2, 0, 1 << 30); // SWSTART
        while reg_read(ADC1, ADC_SR) & (1 << 1) == 0 {} // wait EOC
        let _ = reg_read(ADC1, ADC_DR);         // discard
        // Real conversion
        reg_modify(ADC1, ADC_CR2, 0, 1 << 30); // SWSTART
        while reg_read(ADC1, ADC_SR) & (1 << 1) == 0 {} // wait EOC
        reg_read(ADC1, ADC_DR)
    }
}

/// Read all hub buttons, returns BTN_CENTER | BTN_LEFT | BTN_RIGHT flags.
pub fn read_buttons() -> u8 {
    let mut flags = 0u8;

    if read_adc(14) <= pins::BUTTON_CENTER_THRESHOLD {
        flags |= BTN_CENTER;
    }

    let v = read_adc(1);
    if v <= pins::LR_LEVELS[0] {
        if v > pins::LR_LEVELS[1] {
            // BT only — ignore
        } else if v > pins::LR_LEVELS[2] {
            flags |= BTN_RIGHT;
        } else if v > pins::LR_LEVELS[3] {
            flags |= BTN_RIGHT;
        } else if v > pins::LR_LEVELS[4] {
            flags |= BTN_LEFT;
        } else if v > pins::LR_LEVELS[5] {
            flags |= BTN_LEFT;
        } else if v > pins::LR_LEVELS[6] {
            flags |= BTN_LEFT | BTN_RIGHT;
        } else if v > pins::LR_LEVELS[7] {
            flags |= BTN_LEFT | BTN_RIGHT;
        }
    }

    flags
}

// ════════════════════════════════════════════════════════════════
// RTIC Application
// ════════════════════════════════════════════════════════════════

#[rtic::app(device = stm32f4::stm32f413, dispatchers = [USART1, USART2, USART3])]
mod app {
    use super::*;
    use rtic_monotonics::systick::prelude::*;
    use synopsys_usb_otg::UsbBus;
    use usb_device::prelude::*;
    use usbd_serial::SerialPort;

    systick_monotonic!(Mono, 1000); // 1 kHz tick

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBus<usb_serial::SpikeUsbOtg>>,
        serial: SerialPort<'static, UsbBus<usb_serial::SpikeUsbOtg>>,
        gdb_serial: SerialPort<'static, UsbBus<usb_serial::SpikeUsbOtg>>,
        shell: shell::Shell,
        sensor: sensor::SensorState,
        motor_state: sensor::SensorState,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // ── Power hold — must be first! ──
        unsafe { power::power_hold() };

        // ── Clock tree: 96 MHz from 16 MHz HSE ──
        unsafe { clocks::init_clocks() };

        // ── Enable port power ──
        unsafe { power::enable_port_vcc() };

        // ── LED matrix (TLC5955 on SPI1) ──
        unsafe { led_matrix::init() };

        // Show boot pattern: heart
        led_matrix::show_pattern(&led_matrix::patterns::HEART, 40);
        unsafe { led_matrix::update() };

        // ── Buttons (ADC) ──
        unsafe { init_button_adc() };

        // ── Charger ISET PWM (TIM5 CH1 on PA0) ──
        unsafe { power::init_charger_iset() };

        // ── Motor ports (TIM1/TIM3/TIM4 PWM) ──
        unsafe { motor::init() };

        // ── Speaker (DAC1 + TIM6 triangle wave) ──
        unsafe { sound::init() };

        // ── Watchdog-reset detection ──
        // Must be checked BEFORE starting the new IWDG (which would
        // clear the old state).  Sound must be initialized first.
        if watchdog::was_iwdg_reset() {
            watchdog::watchdog_reset_chime();
            // Read the crash record before SRAM2 is overwritten
            unsafe { watchdog::read_and_store_crash_record(); }
        }

        // ── External SPI flash (W25Q256JV, 32 MB on SPI2) ──
        unsafe { ext_flash::init() };

        // ── Stack painting (before MPU, while stack is shallow) ──
        unsafe { sandbox::stack_paint() };

        // ── MPU sandbox for RAM demos + stack guard bands ──
        if sandbox::mpu_present() {
            unsafe {
                sandbox::mpu_configure();
                sandbox::enable_memmanage();
                // Enable MPU permanently with PRIVDEFENA=1.
                // Privileged code gets the default memory map everywhere
                // EXCEPT the guard band regions (5+6) which trap any access.
                // This means guard bands protect even when no demo is running.
                sandbox::mpu_enable();
            }
        }

        // ── Check for stack-fault marker from previous boot ──
        unsafe { sandbox::check_fault_marker() };

        // ── DWT self-hosted watchpoints (DebugMonitor, no JTAG needed) ──
        unsafe { dwt::init() };

        // Set SVCall priority to lowest so UART ISRs (priority 3)
        // can preempt during SVC delay_ms / sensor operations.
        // SCB->SHPR2 bits [31:24] = SVCall priority.
        // STM32F413 has 4 priority bits in the MSBs, so 0xF0 = priority 15.
        unsafe {
            let shpr2 = 0xE000_ED1C as *mut u32;
            let val = core::ptr::read_volatile(shpr2);
            core::ptr::write_volatile(shpr2, (val & 0x00FF_FFFF) | (0xF0 << 24));
        }

        // ── Hardware watchdog (~5 s timeout) ──
        // Resets the hub automatically if firmware locks up.
        // Fed by heartbeat (500 ms) and SVC delay (20 ms chunks).
        unsafe { watchdog::start() };

        // ── USB CDC serial ──
        let usb_bus = unsafe { usb_serial::init() };
        let (serial, gdb_serial, usb_dev) = usb_serial::create_device(usb_bus);

        // ── Start monotonic (96 MHz HCLK) ──
        Mono::start(cx.core.SYST, 96_000_000);

        // ── Spawn periodic tasks ──
        heartbeat::spawn().ok();
        shell_tick::spawn().ok();
        button_poll::spawn().ok();
        usb_flush::spawn().ok();

        // ── Boot chime: short beep to confirm speaker works ──
        boot_beep::spawn().ok();

        // ── Port F hotplug detection (auto-spawns sensor_poll) ──
        port_detect::spawn().ok();

        // ── Port A motor detection (auto-spawns motor_poll) ──
        motor_detect::spawn().ok();

        // ── Status LED: green = booted OK ──
        led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0x6000, 0);
        unsafe { led_matrix::update() };

        (
            Shared {
                usb_dev,
                serial,
                gdb_serial,
                shell: shell::Shell::new(),
                sensor: sensor::SensorState::new(),
                motor_state: sensor::SensorState::new(),
            },
            Local {},
        )
    }


    /// Idle loop — runs in Thread mode (lowest execution context).
    ///
    /// Sandbox demos execute here because SVC requires Thread mode:
    /// SVCall priority (0xF0) is lower than any RTIC dispatcher, so
    /// executing SVC from Handler mode (any RTIC task) causes HardFault.
    /// Thread mode has the lowest execution priority, so SVCall always
    /// fires successfully, and ALL firmware RTIC tasks preempt freely.
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            let addr = sandbox::demo_pending();
            if addr != 0 {
                if sandbox::is_debug_run() {
                    // Debug mode: privileged Thread mode, no MPU.
                    // DebugMonitor can halt this for GDB breakpoints.
                    let api = user_app_io::build_api();
                    let result = unsafe { sandbox::run_debug(addr, &api) };
                    sandbox::complete_sandbox(result, false);
                } else {
                    let api = sandbox::build_sandboxed_api();
                    match unsafe { sandbox::run_sandboxed(addr, &api) } {
                        Ok(result) => {
                            sandbox::complete_sandbox(result, false);
                        }
                        Err(fault) => {
                            sandbox::complete_sandbox(fault, true);
                        }
                    }
                }
            } else {
                cortex_m::asm::wfi();
            }
        }
    }

    /// USB OTG FS interrupt — poll USB device and handle CDC data.
    ///
    /// Three output sources are drained on every poll:
    ///   1. Shell interactive output (shell.pending())
    ///   2. Demo subprocess output (user_app_io::pending())
    ///   3. Spawns run_demo if shell requested via `go`
    ///
    /// Firmware ISRs (priority 2+) preempt user-app tasks (priority 1)
    /// and Thread-mode sandbox, so output drains in real-time.
    #[task(binds = OTG_FS, shared = [usb_dev, serial, gdb_serial, shell, sensor, motor_state], priority = 2)]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        (cx.shared.usb_dev, cx.shared.serial, cx.shared.gdb_serial, cx.shared.shell, cx.shared.sensor, cx.shared.motor_state).lock(
            |usb_dev, serial, gdb_serial, shell, sensor_state, motor_state| {
                let had_activity = usb_dev.poll(&mut [serial, gdb_serial]);

                // ── Host (re)connect detection ──
                // DTR rising edge: host opened the serial port.
                // USB not Configured: cable unplugged or bus reset.
                // Either resets host_synced so feed() clears stale output.
                shell.check_dtr(serial.dtr());
                if usb_dev.state() != usb_device::device::UsbDeviceState::Configured {
                    shell.usb_disconnected();
                }

                // ── Shell I/O (serial / ttyACM0) ──
                let mut gdb_fed = false;
                if had_activity {
                    let mut buf = [0u8; 64];
                    match serial.read(&mut buf) {
                        Ok(count) if count > 0 => {
                            task_state::mark_usb_rx();
                            shell.feed(&buf[..count], sensor_state, motor_state);
                        }
                        _ => {}
                    }

                    // ── GDB I/O (gdb_serial / ttyACM1) ──
                    match gdb_serial.read(&mut buf) {
                        Ok(count) if count > 0 => {
                            task_state::mark_usb_rx();
                            shell.feed_gdb(&buf[..count]);
                            gdb_fed = true;
                        }
                        _ => {}
                    }
                }

                // Poll GDB halt status — T05 responses go to gdb_out.
                // Skip when feed_gdb() just ran: it already processed halt
                // detection, and the DebugMonitor handler (lower priority)
                // hasn't had a chance to clear TARGET_HALTED yet — polling
                // now would send a stale T05 with old registers.
                if !gdb_fed {
                    shell.poll_gdb();
                }

                // If shell requested a sensor probe, spawn the task
                if let Some(req) = shell.take_sensor_request() {
                    if req == 0xFF {
                        // Stop: abort all active sensor/motor ports
                        for i in 0..6usize {
                            sensor::request_port_abort(i);
                            sensor::port_set_none(i);
                        }
                        sensor_state.status = sensor::Status::None;
                        motor_state.status = sensor::Status::None;
                        // Deinit UARTs for ports that had active sensors
                        for i in 0..6usize {
                            let sp = sensor::PORTS[i];
                            unsafe { sensor::uart_deinit(sp) };
                        }
                    } else if req & 0x80 != 0 {
                        // Mode switch: 0x80 | mode
                        let mode = req & 0x7F;
                        let sp = sensor::PORTS[sensor_state.port_idx as usize];
                        unsafe { sensor::lump_select_mode(sp, mode) };
                        sensor_state.mode = mode;
                    } else if req < 6 {
                        sensor_poll::spawn(req).ok();
                    }
                }

                // Deferred probe: after sensor_poll stops, launch new probe
                if let Some(port) = shell.take_stop_then_probe() {
                    if !task_state::is_active(task_state::SENSOR_POLL) {
                        // Sensor_poll already stopped — probe immediately
                        sensor_poll::spawn(port).ok();
                    } else {
                        // Put it back — ISR will retry next tick
                        shell.set_stop_then_probe(port);
                    }
                }

                // If shell requested a beep, spawn the async tone task
                if let Some((freq, dur)) = shell.take_beep_request() {
                    beep_tone::spawn(freq, dur).ok();
                }

                // If shell requested PID run, spawn the async PID task
                if let Some((port_idx, target)) = shell.take_pid_request() {
                    let pid_cfg = shell.pid_config;
                    pid_run::spawn(port_idx, target, pid_cfg).ok();
                }

                // If shell requested servo run, spawn the async servo task
                if let Some((port_idx, target)) = shell.take_servo_request() {
                    servo_run::spawn(port_idx, target).ok();
                }

                // If shell requested RTTY transmission, spawn the task
                if shell.take_rtty_request() {
                    rtty_tx::spawn().ok();
                }

                // If demo requested RTTY transmission, spawn the task
                if user_app_io::take_rtty_pending() {
                    rtty_tx::spawn().ok();
                }

                // If shell requested test_all ballet, spawn the task
                if shell.take_test_all_request() {
                    test_all::spawn().ok();
                }

                // If shell requested USB reconnect, spawn the task
                if shell.take_reconnect_request() {
                    reconnect_ser::spawn().ok();
                }

                // If shell requested a demo launch (sandboxed or privileged),
                // spawn the subprocess as an RTIC task (always preemptible).
                if let Some(addr) = user_app_io::take_privileged_request() {
                    run_demo::spawn(addr).ok();
                }
                if let Some(addr) = user_app_io::take_sandbox_request() {
                    run_demo::spawn(addr).ok();
                }
                if let Some(addr) = user_app_io::take_debug_request() {
                    run_demo::spawn(addr).ok();
                }

                // Drain demo subprocess output to shell port (ttyACM0).
                // With dual-CDC, demo output always goes to the shell port —
                // no mode gating needed.
                let demo_out = user_app_io::pending();
                if !demo_out.is_empty() {
                    match serial.write(demo_out) {
                        Ok(n) if n > 0 => { task_state::mark_usb_tx(); user_app_io::advance(n); }
                        _ => {}
                    }
                }

                // Drain shell output to shell port (ttyACM0).
                // Gated by host_synced — see comment below.
                if shell.is_synced() {
                    let pending = shell.pending();
                    if !pending.is_empty() {
                        match serial.write(pending) {
                            Ok(n) if n > 0 => { task_state::mark_usb_tx(); shell.advance(n); }
                            _ => {}
                        }
                    }
                }

                // Drain GDB RSP output to GDB port (ttyACM1).
                let gdb_pending = shell.gdb_pending();
                if !gdb_pending.is_empty() {
                    match gdb_serial.write(gdb_pending) {
                        Ok(n) if n > 0 => { task_state::mark_usb_tx(); shell.gdb_advance(n); }
                        _ => {}
                    }
                }
            },
        );
    }

    /// Heartbeat: blink center pixel, manage charger, update battery LED.
    ///
    /// The battery LED now shows a layered display:
    ///   Base pattern — driven by charger status (LEGO-like):
    ///     Charging:     green 1 s on / 1 s off
    ///     Complete:     solid dim green
    ///     Fault:        amber blink
    ///     Discharging:  off (or dim battery-level colour)
    ///   Smart overlay — brief colour pulse when USB activity detected:
    ///     RX:   green flash   (1 tick = 500 ms)
    ///     TX:   blue flash
    ///     Both: cyan flash
    #[task(shared = [shell], priority = 2)]
    async fn heartbeat(mut cx: heartbeat::Context) {
        task_state::mark_active(task_state::HEARTBEAT);
        let mut on = false;
        let mut sent_banner = false;
        let mut ticks: u32 = 0;
        let mut charger = power::ChargerState::new();
        let mut low_bat_warned = false;
        let mut morse = led_matrix::MorseBlinker::new();

        loop {
            on = !on;
            ticks = ticks.wrapping_add(1);
            led_matrix::set_pixel(12, if on { 30 } else { 5 });

            // ── Charger state machine (500 ms tick) ──
            charger.tick();

            // ── Morse battery LED ──
            // Map charger status to Morse pattern (CH/HI/NG/BT)
            let morse_id = match charger.status {
                power::ChargerStatus::Charging    => led_matrix::MORSE_CHARGING,
                power::ChargerStatus::Complete    => led_matrix::MORSE_COMPLETE,
                power::ChargerStatus::Fault       => led_matrix::MORSE_FAULT,
                power::ChargerStatus::Discharging => led_matrix::MORSE_BATTERY,
            };
            morse.set_status(morse_id);

            // Bright (mark) and dim (space) colours for this status
            let (bright, dim): ((u16, u16, u16), (u16, u16, u16)) = match charger.status {
                power::ChargerStatus::Charging => (
                    (0, 0x6000, 0), (0, 0x0800, 0)      // green
                ),
                power::ChargerStatus::Complete => (
                    (0, 0x4000, 0), (0, 0x1000, 0)      // green (dimmer mark)
                ),
                power::ChargerStatus::Fault => (
                    (0x6000, 0x2000, 0), (0x0800, 0x0400, 0) // amber
                ),
                power::ChargerStatus::Discharging => {
                    let mv = power::battery_mv();
                    if mv > 7600 {
                        ((0, 0x4000, 0), (0, 0x0400, 0))    // green
                    } else if mv > 7000 {
                        ((0x4000, 0x2000, 0), (0x0400, 0x0200, 0)) // amber
                    } else {
                        ((0x8000, 0, 0), (0x0800, 0, 0))    // red
                    }
                }
            };

            // Four 125 ms sub-ticks per heartbeat (Farnsworth Morse)
            for _ in 0..4u8 {
                let (r, g, b) = if morse.is_mark() { bright } else { dim };
                morse.advance();

                // USB data flash: brief blue pulse, then restore Morse colour
                let (rx, tx) = task_state::take_usb_activity();
                if rx || tx {
                    led_matrix::set_status_rgb(pins::BATTERY_LED, 0, 0, 0x6000);
                    unsafe { led_matrix::update() };
                    Mono::delay(80.millis()).await;
                    led_matrix::set_status_rgb(pins::BATTERY_LED, r, g, b);
                    unsafe { led_matrix::update() };
                    Mono::delay(45.millis()).await;
                } else {
                    led_matrix::set_status_rgb(pins::BATTERY_LED, r, g, b);
                    unsafe { led_matrix::update() };
                    Mono::delay(125.millis()).await;
                }
            }

            // ── Low battery warning / shutdown ──
            if !power::vbus_present() && ticks % 20 == 0 {
                let mv = power::battery_mv();
                if mv < power::BATTERY_CRITICAL_MV && mv > 1000 {
                    // Below 5800 mV (2.9V/cell) — shut down to protect the cells.
                    power::deep_sleep();
                } else if mv < 6200 && !low_bat_warned {
                    low_bat_warned = true;
                    // Brief warning beep
                    sound::play(440);
                    Mono::delay(80.millis()).await;
                    sound::stop();
                }
            }

            cx.shared.shell.lock(|shell| {
                shell.tick();
            });

            // Send welcome banner once USB is likely enumerated (~2 seconds)
            if !sent_banner {
                static BANNER_DELAY: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
                let count = BANNER_DELAY.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if count >= 4 {
                    sent_banner = true;
                    send_banner::spawn().ok();
                }
            }

            watchdog::feed();
            unsafe { watchdog::snapshot(ticks); }
        }
    }

    /// Send the welcome banner once USB is enumerated.
    /// Pushes banner through shell output buffer to prevent garbling
    /// if input arrives simultaneously.
    #[task(shared = [shell], priority = 2)]
    async fn send_banner(mut cx: send_banner::Context) {
        task_state::mark_active(task_state::SEND_BANNER);
        cx.shared.shell.lock(|shell| {
            shell.send_banner();
        });
        task_state::mark_idle(task_state::SEND_BANNER);
    }

    /// Periodic shell tick (uptime counter).
    #[task(shared = [shell], priority = 2)]
    async fn shell_tick(mut cx: shell_tick::Context) {
        task_state::mark_active(task_state::SHELL_TICK);
        loop {
            cx.shared.shell.lock(|shell| {
                shell.tick();
            });
            Mono::delay(1000.millis()).await;
        }
    }

    /// Periodic USB flush: pend the OTG_FS ISR every 5ms so pending
    /// shell and user-app output drains continuously without waiting
    /// for the host to send data first.
    #[task(priority = 2)]
    async fn usb_flush(_cx: usb_flush::Context) {
        loop {
            Mono::delay(5.millis()).await;
            cortex_m::peripheral::NVIC::pend(stm32f4::stm32f413::Interrupt::OTG_FS);
        }
    }

    /// Boot chime — short rising two-tone beep after init.
    #[task(priority = 2)]
    async fn boot_beep(_cx: boot_beep::Context) {
        task_state::mark_active(task_state::BOOT_BEEP);
        // Wait for USB to settle, then play a short chirp
        Mono::delay(500.millis()).await;
        sound::play(880);   // A5
        Mono::delay(80.millis()).await;
        sound::play(1760);  // A6
        Mono::delay(80.millis()).await;
        sound::stop();
        task_state::mark_idle(task_state::BOOT_BEEP);
    }

    /// Async PID position-hold task.
    ///
    /// Simple PD controller: P on position error, D on measured speed
    /// (derivative-on-measurement). Output capped at out_max (default 40%)
    /// so the motor can never build excessive momentum.
    ///
    /// No trajectory generator — the output cap naturally limits approach
    /// speed. Strong Kd braking prevents overshoot.
    ///
    /// Runs at 20ms (50 Hz) matching the LUMP sensor update rate.
    #[task(shared = [shell, sensor], priority = 2)]
    async fn pid_run(
        mut cx: pid_run::Context,
        port_idx: u8,
        target: i32,
        pid_config: motor::Pid,
    ) {
        task_state::mark_active(task_state::PID_RUN);

        // Read initial position from shared sensor state
        let init_pos = cx.shared.sensor.lock(|s| s.motor_pos_degrees());

        // Simple P-only controller with brake deadband.
        // Why no D: speed estimation from 20ms-sampled 50ms LUMP data is
        // too noisy — the D term oscillates wildly and destabilizes.
        // Why no I: not needed for basic go-to-position; stiction limits
        // accuracy to ~5° anyway. P gets us close, brake holds us there.
        //
        // Can be improved later with filtered speed estimation or
        // running PID only on fresh sensor data (data_seq changes).

        // All tunable via shell: `pid sc`, `pid omax`, `pid et`
        //
        // Two-phase ramp+nudge controller:
        //   Phase 1 (approach): ramp from omax → sc over decel zone, then brake
        //   Phase 2 (recovery): after first brake, only nudge-pulses to converge
        //
        // Nudge = drive at sc for nudge_len iters (100ms), then coast for
        // stall_wait iters (200ms). Each nudge moves ~2-5°, preventing
        // the wild oscillation that constant sc drive causes.
        let omax = pid_config.out_max.max(10);
        let sc = pid_config.stiction_comp.max(0); // 0 = no nudge
        let brake_zone = pid_config.error_threshold.max(2);
        let decel_zone = brake_zone * 5; // start slowing 5x brake_zone away
        let settle_hold_ms = 2000i32;
        let nudge_len = 5u32;   // iters of nudge pulse (100ms)
        let stall_wait = 10u32; // iters between nudges (200ms)

        let distance = (target - init_pos).abs();
        let approach_ms = (distance as i64 * 1000 / 200).min(15000) as i32;
        let total_ms = approach_ms + settle_hold_ms + 5000; // extra time for nudges
        let total_iters = ((total_ms as u32) + 19) / 20;

        cx.shared.shell.lock(|shell| {
            let mut t = [0u8; 96];
            let mut w = crate::shell::BufWriter::new(&mut t);
            let _ = core::fmt::Write::write_fmt(
                &mut w,
                format_args!("  Ramp: {}->{}deg omax={} sc={} brk<{}deg decel<{}deg\r\n",
                    init_pos, target, omax, sc, brake_zone, decel_zone),
            );
            shell.push(w.written());
        });

        let mut settled_at: Option<u32> = None;
        let mut approach_done = false; // true after first time entering brake zone
        let mut stall_count = 0u32;
        let mut nudging = 0u32; // remaining nudge iters (countdown)
        let mut prev_pos = init_pos;

        for iter in 0u32..total_iters.max(200) {
            let t_ms = iter * 20;

            let pos = cx.shared.sensor.lock(|s| s.motor_pos_degrees());
            let err = target - pos;
            let abs_err = err.abs();

            // Track stall
            if pos == prev_pos {
                stall_count += 1;
            } else {
                stall_count = 0;
            }
            prev_pos = pos;

            if abs_err <= brake_zone {
                // In brake zone — hold brake, start settle timer
                motor::brake(port_idx as u32);
                approach_done = true;
                nudging = 0;
                if settled_at.is_none() {
                    settled_at = Some(t_ms);
                }
                if let Some(sat) = settled_at {
                    if t_ms - sat >= settle_hold_ms as u32 {
                        break;
                    }
                }
            } else if !approach_done {
                // Phase 1: approach via decel ramp (omax → sc)
                settled_at = None;
                let out_mag = if abs_err >= decel_zone {
                    omax
                } else {
                    // Linear ramp: omax at decel_zone, sc at brake_zone
                    let range = decel_zone - brake_zone;
                    let frac = abs_err - brake_zone;
                    sc + (omax - sc) * frac / range.max(1)
                };
                let out = if err > 0 { out_mag } else { -out_mag };
                motor::set(port_idx as u32, out);
            } else if nudging > 0 {
                // Phase 2: currently nudging — drive at sc
                nudging -= 1;
                let out = if err > 0 { sc } else { -sc };
                motor::set(port_idx as u32, out);
                if nudging == 0 {
                    motor::brake(port_idx as u32);
                    stall_count = 0; // full wait before next nudge
                }
            } else if stall_count >= stall_wait && sc > 0 {
                // Phase 2: stalled outside brake zone → start nudge
                nudging = nudge_len;
                stall_count = 0;
                let out = if err > 0 { sc } else { -sc };
                motor::set(port_idx as u32, out);
            } else {
                // Phase 2: coast/brake while waiting
                motor::brake(port_idx as u32);
                settled_at = None;
            }

            // Print every 10 iters (200ms)
            if iter % 10 == 0 {
                let tag = if abs_err <= brake_zone {
                    "BRK"
                } else if nudging > 0 {
                    "NDG"
                } else if !approach_done {
                    "APP"
                } else {
                    "   "
                };
                cx.shared.shell.lock(|shell| {
                    let mut t = [0u8; 80];
                    let mut w = crate::shell::BufWriter::new(&mut t);
                    let _ = core::fmt::Write::write_fmt(
                        &mut w,
                        format_args!("  t={}ms pos={} err={} {}\r\n",
                            t_ms, pos, err, tag),
                    );
                    shell.push(w.written());
                });
            }

            Mono::delay(20.millis()).await;
        }

        motor::brake(port_idx as u32);

        let pos = cx.shared.sensor.lock(|s| s.motor_pos_degrees());
        let final_err = target - pos;
        cx.shared.shell.lock(|shell| {
            let mut t = [0u8; 64];
            let mut w = crate::shell::BufWriter::new(&mut t);
            let _ = core::fmt::Write::write_fmt(
                &mut w,
                format_args!("PID done. pos={}deg err={}deg\r\n", pos, final_err),
            );
            shell.push(w.written());
        });

        task_state::mark_idle(task_state::PID_RUN);
    }

    /// Pybricks-style servo position-hold using observer + PID.
    ///
    /// Runs at 5ms (100 Hz) with a Luenberger state observer that
    /// predicts between 50 Hz LUMP sensor updates, plus feedforward
    /// for friction and inertia. This replaces the old 50 Hz PID
    /// for high-quality motor control.
    #[task(shared = [shell, sensor], priority = 2)]
    async fn servo_run(
        mut cx: servo_run::Context,
        port_idx: u8,
        target_deg: i32,
    ) {
        task_state::mark_active(task_state::SERVO_RUN);

        // Determine motor model from sensor type_id
        let (type_id, init_pos) = cx.shared.sensor.lock(|s| {
            (s.type_id, s.motor_pos_degrees())
        });
        let (model, settings) = match type_id {
            sensor::TYPE_TECHNIC_L_ANGULAR | sensor::TYPE_SPIKE_L_MOTOR => {
                (&servo::MODEL_TECHNIC_L_ANGULAR, servo::ControlSettings::technic_l_angular())
            }
            sensor::TYPE_SPIKE_S_MOTOR => {
                (&servo::MODEL_TECHNIC_S_ANGULAR, servo::ControlSettings::technic_s_angular())
            }
            _ => {
                // Default: Technic M Angular (type 75, 48, 49)
                (&servo::MODEL_TECHNIC_M_ANGULAR, servo::ControlSettings::technic_m_angular())
            }
        };

        // Apply shell overrides if user tuned via 'stune'
        let settings = cx.shared.shell.lock(|shell| {
            shell.servo_config.unwrap_or(settings)
        });

        let target_mdeg = target_deg as i64 * 1000;
        let init_mdeg = init_pos as i64 * 1000;
        let time_ticks: u32 = 0;
        let mut srv = servo::Servo::new(model, settings, init_mdeg, time_ticks);
        // Apply observer gain override if set
        if settings.obs_gain > 0 {
            srv.observer.feedback_gain = settings.obs_gain;
        }
        // In raw mode, tell observer the actual loop time for correct speed calc
        if settings.obs_gain == -1 {
            srv.observer.actual_loop_ms = 20;
        }
        srv.start_position_hold(target_mdeg, time_ticks);

        // Report config
        cx.shared.shell.lock(|shell| {
            let mut t = [0u8; 96];
            let mut w = crate::shell::BufWriter::new(&mut t);
            let _ = core::fmt::Write::write_fmt(
                &mut w,
                format_args!("  servo hold: target={}deg Kp={} Kd={} model_gain={}\r\n",
                    target_deg, srv.settings.pid_kp, srv.settings.pid_kd, model.gain),
            );
            shell.push(w.written());
        });

        // Control loop timing: when obs==-1 (raw mode), run at 20ms to match
        // sensor rate. Otherwise run at 5ms with observer interpolating.
        let raw_mode = settings.obs_gain == -1;
        let loop_ms: u32 = if raw_mode { 20 } else { servo::LOOP_TIME_MS as u32 };
        let max_iters: u32 = 10_000 / loop_ms; // 10s total
        let print_interval: u32 = 200 / loop_ms; // every 200ms

        let mut prev_seq: u32 = cx.shared.sensor.lock(|s| s.data_seq);

        for iter in 0u32..max_iters {
            let t_ticks = iter * loop_ms * (servo::TICKS_PER_MS as u32);

            // Read latest position + sequence number from shared sensor state
            let (pos_deg, seq) = cx.shared.sensor.lock(|s| {
                (s.motor_pos_degrees(), s.data_seq)
            });
            let measured_mdeg = pos_deg as i64 * 1000;
            let fresh = seq != prev_seq;
            if fresh { prev_seq = seq; }

            // Run controller
            let actuation = srv.update(t_ticks, measured_mdeg, fresh);

            // Apply to motor
            let duty = match actuation {
                servo::Actuation::Coast => { motor::set(port_idx as u32, 0); 0 }
                servo::Actuation::Brake => { motor::brake(port_idx as u32); 0 }
                servo::Actuation::Torque(torque) => {
                    let d = srv.torque_to_duty(torque);
                    motor::set(port_idx as u32, d);
                    d
                }
            };

            // Print every ~200ms or on last iter
            if iter % print_interval == 0 || iter == max_iters - 1 {
                let t_ms = iter * loop_ms;
                let d = srv.diag();
                cx.shared.shell.lock(|shell| {
                    let mut t = [0u8; 120];
                    let mut w = crate::shell::BufWriter::new(&mut t);
                    let _ = core::fmt::Write::write_fmt(
                        &mut w,
                        format_args!("  t={} pos={} obs={} err={} spd={} I={} duty={}\r\n",
                            t_ms, pos_deg,
                            (d.est_angle_mdeg / 1000) as i32,
                            target_deg - (d.est_angle_mdeg / 1000) as i32,
                            d.est_speed / 1000,
                            d.integral / 1000,
                            duty),
                    );
                    shell.push(w.written());
                });
            }

            Mono::delay(loop_ms.millis()).await;
        }

        motor::brake(port_idx as u32);

        // Final position read
        let pos = cx.shared.sensor.lock(|s| s.motor_pos_degrees());
        cx.shared.shell.lock(|shell| {
            let mut t = [0u8; 48];
            let mut w = crate::shell::BufWriter::new(&mut t);
            let _ = core::fmt::Write::write_fmt(
                &mut w,
                format_args!("Servo done. final pos={}deg\r\n", pos),
            );
            shell.push(w.written());
        });

        task_state::mark_idle(task_state::SERVO_RUN);
    }

    /// Port E+F auto-probe: spawn sensor_poll + sensor_poll2 at boot.
    #[task(shared = [sensor], priority = 2)]
    async fn port_detect(mut cx: port_detect::Context) {
        loop {
            if task_state::is_auto_detect_paused() {
                Mono::delay(3000.millis()).await;
                continue;
            }

            // Port F → sensor_poll
            if !task_state::is_active(task_state::SENSOR_POLL) {
                let status = cx.shared.sensor.lock(|s| s.status);
                if status != sensor::Status::Data {
                    sensor_poll::spawn(5).ok();
                }
            }

            // Port E → sensor_poll2
            if !task_state::is_active(task_state::SENSOR_POLL2) {
                if sensor::port_status(4) != sensor::Status::Data {
                    sensor_poll2::spawn(4).ok();
                }
            }

            // Check every 3s — gives handshake time (~4s)
            Mono::delay(3000.millis()).await;
        }
    }

    /// Primary motor port (Port A by default).
    /// Syncs to cx.shared.motor_state for backward compat with PID/servo.
    #[task(shared = [motor_state], priority = 2)]
    async fn motor_poll(mut cx: motor_poll::Context, port_idx: u8) {
        task_state::mark_active(task_state::MOTOR_POLL);
        sensor::clear_port_abort(port_idx as usize);

        let mut init = sensor::SensorState::new();
        init.port_idx = port_idx;
        init.status = sensor::Status::Syncing;
        cx.shared.motor_state.lock(|s| *s = init);

        let state = lump_handshake(port_idx).await;
        cx.shared.motor_state.lock(|s| *s = state);

        if state.status != sensor::Status::Data {
            task_state::mark_idle(task_state::MOTOR_POLL);
            return;
        }

        let sp = sensor::PORTS[port_idx as usize];
        let pi = port_idx as usize;
        let mut tick: u32 = 0;
        loop {
            Mono::delay(20.millis()).await;
            tick += 1;

            let mut local_state = sensor::port_state(pi);
            if local_state.status != sensor::Status::Data { break; }

            sensor::lump_poll_data(&mut local_state);

            if tick % 5 == 0 {
                local_state.data_received = false;
                unsafe { sensor::lump_keepalive(sp) };
            }

            sensor::set_port_state(pi, &local_state);
            cx.shared.motor_state.lock(|s| *s = local_state);
        }
        task_state::mark_idle(task_state::MOTOR_POLL);
    }

    /// Secondary motor port (Port B by default).
    /// State stored in PORT_STATES only — no shared resource needed.
    #[task(priority = 2)]
    async fn motor_poll2(_cx: motor_poll2::Context, port_idx: u8) {
        task_state::mark_active(task_state::MOTOR_POLL2);
        sensor::clear_port_abort(port_idx as usize);

        let state = lump_handshake(port_idx).await;

        if state.status != sensor::Status::Data {
            task_state::mark_idle(task_state::MOTOR_POLL2);
            return;
        }

        let sp = sensor::PORTS[port_idx as usize];
        let pi = port_idx as usize;
        let mut tick: u32 = 0;
        loop {
            Mono::delay(20.millis()).await;
            tick += 1;

            let mut local_state = sensor::port_state(pi);
            if local_state.status != sensor::Status::Data { break; }

            sensor::lump_poll_data(&mut local_state);

            if tick % 5 == 0 {
                local_state.data_received = false;
                unsafe { sensor::lump_keepalive(sp) };
            }

            sensor::set_port_state(pi, &local_state);
        }
        task_state::mark_idle(task_state::MOTOR_POLL2);
    }

    /// Port A+B auto-probe: spawn motor_poll + motor_poll2 at boot.
    #[task(shared = [motor_state], priority = 2)]
    async fn motor_detect(mut cx: motor_detect::Context) {
        loop {
            // Port A → motor_poll
            if !task_state::is_active(task_state::MOTOR_POLL) {
                let status = cx.shared.motor_state.lock(|s| s.status);
                if status != sensor::Status::Data {
                    motor_poll::spawn(0).ok();
                }
            }

            // Port B → motor_poll2
            if !task_state::is_active(task_state::MOTOR_POLL2) {
                if sensor::port_status(1) != sensor::Status::Data {
                    motor_poll2::spawn(1).ok();
                }
            }

            Mono::delay(3000.millis()).await;
        }
    }

    /// Play a beep at given frequency for given duration.
    /// Spawned from shell commands (non-blocking via USB ISR).
    #[task(priority = 2)]
    async fn beep_tone(_cx: beep_tone::Context, freq: u32, dur_ms: u32) {
        task_state::mark_active(task_state::BEEP_TONE);
        sound::play(freq);
        Mono::delay(dur_ms.millis()).await;
        sound::stop();
        task_state::mark_idle(task_state::BEEP_TONE);
    }

    /// Transmit RTTY Baudot FSK audio on the speaker.
    /// 45.45 baud, ITA2 encoding, 170 Hz shift, 1.5 stop bits.
    /// Mark (lower tone) = 2760 Hz, Space (higher tone) = 2933 Hz.
    /// Preamble/postamble: LTRS diddles for receiver sync.
    ///
    /// Priority 2: preempts user-app tasks (priority 1) so RTTY plays
    /// truly in parallel with motor/sensor work.  Uses Mono::delay()
    /// await — zero busy-wait, yields between FSK tone switches.
    #[task(priority = 2)]
    async fn rtty_tx(_cx: rtty_tx::Context) {
        task_state::mark_active(task_state::RTTY_TX);
        rtty::mark_busy();

        let msg = rtty::get_message();
        let mut in_figures = false;

        // Helper: transmit one Baudot character (start + 5 data + 1.5 stop)
        async fn tx_char(code: u8) {
            // Start bit — space
            rtty::tx_start_bit();
            Mono::delay(rtty::BIT_MS.millis()).await;
            // 5 data bits, LSB first
            for i in 0..5u8 {
                rtty::tx_data_bit((code >> i) & 1 == 1);
                Mono::delay(rtty::BIT_MS.millis()).await;
            }
            // Stop bit — mark, 1.5 bit times
            rtty::tx_stop_bit();
            Mono::delay(rtty::STOP_MS.millis()).await;
        }

        // Preamble: LTRS diddles for receiver sync
        for _ in 0..rtty::DIDDLE_COUNT {
            tx_char(rtty::LTRS_CODE).await;
        }

        for &ch in msg.iter() {
            if let Some((code, needs_figs)) = rtty::ascii_to_baudot(ch) {
                if needs_figs && !in_figures {
                    tx_char(rtty::FIGS_CODE).await;
                    in_figures = true;
                } else if !needs_figs && in_figures
                    && ch != b'\r' && ch != b'\n' && ch != b' '
                {
                    tx_char(rtty::LTRS_CODE).await;
                    in_figures = false;
                }
                tx_char(code).await;
            }
        }

        // End with CR + LF
        if let Some((cr, _)) = rtty::ascii_to_baudot(b'\r') {
            tx_char(cr).await;
        }
        if let Some((lf, _)) = rtty::ascii_to_baudot(b'\n') {
            tx_char(lf).await;
        }

        // Postamble: LTRS diddles for receiver sync
        for _ in 0..rtty::DIDDLE_COUNT {
            tx_char(rtty::LTRS_CODE).await;
        }
        rtty::tx_silence();

        rtty::mark_idle();
        task_state::mark_idle(task_state::RTTY_TX);
    }

    /// LUMP handshake: disconnect, TYPE scan, INFO, ACK, baud switch.
    /// Shared by all port poll tasks.  All state stored in PORT_STATES.
    /// Returns the final SensorState (status = Data on success, Error on failure).
    ///
    /// Async state machine — every `.await` is a state boundary.
    /// After each wake the machine checks readiness / abort and transitions.
    async fn lump_handshake(port_idx: u8) -> sensor::SensorState {
        let sp = sensor::PORTS[port_idx as usize];
        let pi = port_idx as usize;
        let mut state = sensor::SensorState::new();
        state.port_idx = port_idx;
        state.status = sensor::Status::Syncing;
        sensor::set_port_state(pi, &state);

        macro_rules! rx_byte {
            ($timeout_ms:expr) => {{
                let mut _w = 0u32;
                loop {
                    if sensor::is_port_abort(pi) { break None; }
                    if let Some(b) = sensor::rx_read_byte(pi) {
                        break Some(b);
                    }
                    if _w >= $timeout_ms { break None; }
                    Mono::delay(1.millis()).await;
                    _w += 1;
                }
            }};
        }

        // ────────────────────────────────────────────────────
        // State: FastProbe — try 115200 before full handshake
        // ────────────────────────────────────────────────────
        let cached = sensor::cached_port_type(pi);
        if cached != 0 && !sensor::is_port_abort(pi) {
            unsafe {
                sensor::buffer_enable(sp);
                sensor::uart_init(sp, sensor::SPEED_LPF2);
                sensor::uart_start_irq_rx(sp);
            }
            sensor::rx_flush(pi);
            unsafe { sensor::lump_keepalive(sp) };
            let mut fast_ok = false;
            for _ in 0..20 {
                Mono::delay(10.millis()).await;
                if sensor::is_port_abort(pi) { break; }
                if sensor::lump_poll_data(&mut state) {
                    fast_ok = true;
                    break;
                }
            }
            if fast_ok && !sensor::is_port_abort(pi) {
                state.type_id = cached;
                state.mode = sensor::default_mode_for_type(cached);
                state.capabilities = sensor::cached_port_caps(pi);
                if state.mode != 0 {
                    unsafe { sensor::lump_select_mode(sp, state.mode) };
                    // ── StaleWait after mode select ──
                    Mono::delay(sensor::STALE_DATA_DELAY_MS.millis()).await;
                    sensor::rx_flush(pi);
                }
                // Provide motor-pin power if sensor requires it
                sensor::apply_pin_power(port_idx, state.capabilities);
                state.status = sensor::Status::Data;
                state.data_received = false;
                state.debug_step = 99;
                sensor::set_port_state(pi, &state);
                unsafe { sensor::lump_keepalive(sp) };
                return state;
            }
            unsafe { sensor::uart_deinit(sp) };
        }

        // ────────────────────────────────────────────
        // State: Disconnect — reset sensor LUMP watchdog
        // ────────────────────────────────────────────
        unsafe { sensor::disconnect(sp) };
        Mono::delay(500.millis()).await;
        if sensor::is_port_abort(pi) {
            unsafe { sensor::uart_deinit(sp) };
            state.status = sensor::Status::None;
            sensor::set_port_state(pi, &state);
            return state;
        }

        // ────────────────────────────────────────────
        // State: DisconnectWait — second pulse
        // ────────────────────────────────────────────
        unsafe { sensor::buffer_enable(sp) };
        Mono::delay(100.millis()).await;
        unsafe { sensor::disconnect(sp) };
        Mono::delay(1500.millis()).await;
        if sensor::is_port_abort(pi) {
            unsafe { sensor::uart_deinit(sp) };
            state.status = sensor::Status::None;
            sensor::set_port_state(pi, &state);
            return state;
        }

        // ────────────────────────────────────────────
        // State: SyncInit — UART at 2400, enable buffer
        // ────────────────────────────────────────────
        unsafe {
            sensor::uart_init(sp, 2400);
            sensor::uart_start_irq_rx(sp);
        }
        unsafe { sensor::buffer_enable(sp) };
        Mono::delay(2.millis()).await;
        sensor::rx_flush(pi);
        state.debug_step = 3;

        // ────────────────────────────────────────────
        // State: SyncType — scan for TYPE message
        // ────────────────────────────────────────────
        let mut buf = [0u8; sensor::MAX_MSG];
        let mut attempts = 0u32;
        loop {
            match rx_byte!(250) {
                Some(b) if b == sensor::TYPE_HEADER => {
                    let id = rx_byte!(50);
                    let chk = rx_byte!(50);
                    if let (Some(type_id), Some(checksum)) = (id, chk) {
                        let expected = 0xFF ^ sensor::TYPE_HEADER ^ type_id;
                        if checksum == expected && type_id >= 29 && type_id <= 101 {
                            state.type_id = type_id;
                            state.debug_step = 30;
                            break;
                        }
                    }
                    attempts += 1;
                }
                Some(b) => {
                    if state.debug_byte == 0xFF { state.debug_byte = b; }
                    if (state.debug_bytes_len as usize) < 8 {
                        state.debug_bytes[state.debug_bytes_len as usize] = b;
                        state.debug_bytes_len += 1;
                    }
                    attempts += 1;
                }
                None => { attempts += 1; }
            }
            if attempts > 500 || sensor::is_port_abort(pi) {
                state.status = sensor::Status::Error;
                state.debug_step = 10;
                sensor::set_port_state(pi, &state);
                unsafe { sensor::uart_deinit(sp) };
                return state;
            }
        }

        // ────────────────────────────────────────────
        // State: SyncInfo — read INFO until device ACK
        // ────────────────────────────────────────────
        let mut timeouts = 0u32;
        loop {
            match rx_byte!(250) {
                Some(b) if b == sensor::SYS_ACK => {
                    state.debug_step = 40;
                    break;
                }
                Some(hdr) => {
                    timeouts = 0;
                    let msize = sensor::msg_size(hdr);
                    if msize > 1 && msize <= sensor::MAX_MSG {
                        buf[0] = hdr;
                        let mut got = 0usize;
                        for i in 1..msize {
                            if let Some(b) = rx_byte!(50) {
                                buf[i] = b;
                                got += 1;
                            } else {
                                break;
                            }
                        }
                        if got == msize - 1 {
                            let mt = hdr & 0xC0;
                            let cmd = hdr & 0x07;
                            if mt == 0x40 && cmd == 1 {
                                state.num_modes = buf[1] + 1;
                            }
                            if mt == 0x40 && cmd == 2 && msize >= 6 {
                                state.new_baud = u32::from_le_bytes(
                                    [buf[1], buf[2], buf[3], buf[4]]);
                            }
                            // INFO_NAME (type=INFO=0x80, cmd=0): extract
                            // capability flags from FLAGS0 at buf[8].
                            // Pybricks: if name_len <= 5 && msg_size > 11,
                            // FLAGS0 contains NEEDS_SUPPLY_PIN1/PIN2 bits.
                            if mt == 0x80 && cmd == 0 && msize > 11 {
                                let name_len = (0..6usize)
                                    .take_while(|&i| buf[2 + i] != 0)
                                    .count();
                                if name_len <= 5 {
                                    state.capabilities |= buf[8];
                                }
                            }
                        }
                    }
                }
                None => {
                    timeouts += 1;
                    if timeouts > 5 || sensor::is_port_abort(pi) {
                        state.status = sensor::Status::Error;
                        state.debug_step = 31;
                        sensor::set_port_state(pi, &state);
                        unsafe { sensor::uart_deinit(sp) };
                        return state;
                    }
                }
            }
        }

        // ────────────────────────────────────────────
        // State: SyncAck — send ACK, switch baud
        // ────────────────────────────────────────────
        unsafe { sensor::uart_tx(sp, &[sensor::SYS_ACK]) };
        Mono::delay(10.millis()).await;
        unsafe { sensor::uart_set_baud(sp, state.new_baud) };
        sensor::rx_flush(pi);
        unsafe { sensor::uart_start_irq_rx(sp) };
        state.debug_step = 50;

        // ────────────────────────────────────────────
        // State: ModeSelect — select target mode
        // ────────────────────────────────────────────
        state.mode = sensor::default_mode_for_type(state.type_id);
        // Only send SELECT if mode is non-zero (mode 0 is the sensor's
        // power-on default; sending redundant SELECT(0) may confuse some
        // devices like the ultrasonic sensor).
        if state.mode != 0 {
            unsafe { sensor::lump_select_mode(sp, state.mode) };
            // ────────────────────────────────────────────
            // State: StaleWait — flush stale data after mode switch
            // ────────────────────────────────────────────
            Mono::delay(sensor::STALE_DATA_DELAY_MS.millis()).await;
            sensor::rx_flush(pi);
        }
        state.data_received = false;
        state.debug_step = 60;

        // Cache type for fast re-probe next time
        sensor::cache_port_type(pi, state.type_id);
        sensor::cache_port_caps(pi, state.capabilities);

        // Provide motor-pin power if sensor requires it (e.g. ultrasonic)
        sensor::apply_pin_power(port_idx, state.capabilities);

        state.status = sensor::Status::Data;
        sensor::set_port_state(pi, &state);

        // Initial keepalive
        unsafe { sensor::lump_keepalive(sp) };
        state
    }

    // ─────────────────────────────────────────────────────────────
    // lump_data_loop — the DATA-phase state machine with watchdog.
    //
    // Runs after a successful handshake.  Every 20 ms it:
    //   1. Polls ring buffer for DATA messages
    //   2. Every 5th tick (~100 ms): checks data watchdog + keepalive
    //   3. If watchdog fires → returns false (caller should re-sync)
    //   4. If abort requested → returns true (caller should stop)
    //
    // Returns true = clean exit (abort/status change),
    //         false = watchdog timeout (caller should re-sync).
    // ─────────────────────────────────────────────────────────────
    async fn lump_data_loop(pi: usize, sp: &sensor::SensorPort) -> bool {
        let mut tick: u32 = 0;
        let mut watchdog_misses: u8 = 0;

        loop {
            Mono::delay(20.millis()).await;
            tick += 1;

            let mut local_state = sensor::port_state(pi);
            if local_state.status != sensor::Status::Data {
                return true; // clean exit — status changed externally
            }
            if sensor::is_port_abort(pi) {
                return true; // clean exit — abort requested
            }

            // Poll ring buffer for DATA messages
            let got_data = sensor::lump_poll_data(&mut local_state);

            // Every 5th tick (~100 ms): keepalive + watchdog check
            if tick % 5 == 0 {
                if local_state.data_received || got_data {
                    // Sensor is alive — reset watchdog
                    watchdog_misses = 0;
                    local_state.data_received = false;
                } else {
                    // No DATA since last keepalive cycle
                    watchdog_misses += 1;
                    if watchdog_misses >= sensor::DATA_WATCHDOG_CYCLES {
                        // Sensor stopped responding — watchdog fires
                        local_state.status = sensor::Status::Error;
                        local_state.debug_step = 70; // watchdog marker
                        sensor::set_port_state(pi, &local_state);
                        return false; // → caller should re-sync
                    }
                }

                // Send pending light write if queued, otherwise keepalive
                if !sensor::send_pending_light(sp, pi) {
                    unsafe { sensor::lump_keepalive(sp) };
                }
            }

            sensor::set_port_state(pi, &local_state);
        }
    }

    /// Primary sensor port (Port F by default).
    /// Syncs to cx.shared.sensor for backward compat with shell/PID/servo.
    ///
    /// Runs as an async state machine:
    ///   Idle → Handshake → [StaleWait →] Data ←watchdog→ re-Handshake
    ///   Abort at any point → Idle
    #[task(shared = [sensor], priority = 2)]
    async fn sensor_poll(mut cx: sensor_poll::Context, port_idx: u8) {
        task_state::mark_active(task_state::SENSOR_POLL);
        sensor::clear_port_abort(port_idx as usize);
        let sp = sensor::PORTS[port_idx as usize];
        let pi = port_idx as usize;

        // Retry loop: handshake → data loop → watchdog re-sync
        let max_retries: u8 = 3;
        let mut retries: u8 = 0;

        loop {
            // Show Syncing in shared resource while handshake runs
            let mut init = sensor::SensorState::new();
            init.port_idx = port_idx;
            init.status = sensor::Status::Syncing;
            cx.shared.sensor.lock(|s| *s = init);

            let state = lump_handshake(port_idx).await;
            cx.shared.sensor.lock(|s| *s = state);

            if state.status != sensor::Status::Data {
                break;
            }

            // Cut motor-pin power so color sensor LEDs stay off.
            // Mode 5 (RGB_I) auto-lights LEDs when motor pins supply power.
            if state.type_id == sensor::TYPE_SPIKE_COLOR_SENSOR {
                motor::set(port_idx as u32, 0);
            }

            // ── DATA phase with watchdog (inlined for shared resource access) ──
            let mut tick: u32 = 0;
            let mut watchdog_misses: u8 = 0;
            let mut watchdog_fired = false;

            loop {
                Mono::delay(20.millis()).await;
                tick += 1;

                let mut local_state = sensor::port_state(pi);
                if local_state.status != sensor::Status::Data { break; }
                if sensor::is_port_abort(pi) { break; }

                let got_data = sensor::lump_poll_data(&mut local_state);

                if tick % 5 == 0 {
                    if local_state.data_received || got_data {
                        watchdog_misses = 0;
                        local_state.data_received = false;
                    } else {
                        watchdog_misses += 1;
                        if watchdog_misses >= sensor::DATA_WATCHDOG_CYCLES {
                            local_state.status = sensor::Status::Error;
                            local_state.debug_step = 70;
                            sensor::set_port_state(pi, &local_state);
                            cx.shared.sensor.lock(|s| *s = local_state);
                            watchdog_fired = true;
                            break;
                        }
                    }

                    if !sensor::send_pending_light(sp, pi) {
                        unsafe { sensor::lump_keepalive(sp) };
                    }
                }

                sensor::set_port_state(pi, &local_state);
                cx.shared.sensor.lock(|s| *s = local_state);
            }

            if !watchdog_fired {
                break; // clean exit — abort or status changed externally
            }

            // Watchdog fired — try to re-sync
            retries += 1;
            if retries > max_retries {
                let mut err = sensor::SensorState::new();
                err.port_idx = port_idx;
                err.status = sensor::Status::Error;
                err.debug_step = 71;
                sensor::set_port_state(pi, &err);
                cx.shared.sensor.lock(|s| *s = err);
                break;
            }

            unsafe { sensor::uart_deinit(sp) };
            Mono::delay(200.millis()).await;
        }

        task_state::mark_idle(task_state::SENSOR_POLL);
    }

    /// Secondary sensor port (Port E by default).
    /// State stored in PORT_STATES only — no shared resource needed.
    ///
    /// Same state machine as sensor_poll, without the shared resource mirror.
    #[task(priority = 2)]
    async fn sensor_poll2(_cx: sensor_poll2::Context, port_idx: u8) {
        task_state::mark_active(task_state::SENSOR_POLL2);
        sensor::clear_port_abort(port_idx as usize);
        let sp = sensor::PORTS[port_idx as usize];
        let pi = port_idx as usize;

        let max_retries: u8 = 3;
        let mut retries: u8 = 0;

        loop {
            let state = lump_handshake(port_idx).await;

            if state.status != sensor::Status::Data {
                break;
            }

            // Cut motor-pin power so color sensor LEDs stay off.
            if state.type_id == sensor::TYPE_SPIKE_COLOR_SENSOR {
                motor::set(port_idx as u32, 0);
            }

            // ── DATA phase with watchdog ──
            let clean = lump_data_loop(pi, sp).await;

            if clean {
                break;
            }

            // Watchdog fired — try to re-sync
            retries += 1;
            if retries > max_retries {
                let mut err = sensor::SensorState::new();
                err.port_idx = port_idx;
                err.status = sensor::Status::Error;
                err.debug_step = 71;
                sensor::set_port_state(pi, &err);
                break;
            }

            unsafe { sensor::uart_deinit(sp) };
            Mono::delay(200.millis()).await;
        }

        task_state::mark_idle(task_state::SENSOR_POLL2);
    }
    ///
    /// Runs at priority 1 (lowest) — all firmware tasks (priority 2+)
    /// preempt freely.  Sandboxed demos run in idle (Thread mode) for
    /// even lower execution priority; this task poll-waits for results.
    ///
    /// Two execution modes:
    ///   `go`  — sandboxed: delegate to idle, MPU + SVC in Thread mode
    ///   `go!` — privileged: direct fn pointers + MSP (firmware dev)
    #[task(shared = [sensor, motor_state], priority = 1)]
    async fn run_demo(mut cx: run_demo::Context, addr: u32) {
        task_state::mark_active(task_state::RUN_DEMO);
        user_app_io::reset();
        user_app_io::set_running(true);

        // Tell sensor module which port the demo can read from
        let port_idx = cx.shared.sensor.lock(|s| s.port_idx);
        sensor::demo_set_port(port_idx);

        // Set motor port for demos (motor_poll's port)
        let motor_port = cx.shared.motor_state.lock(|s| s.port_idx);
        if motor_port < 6 {
            sensor::demo_set_motor_port(motor_port);
        }

        // Keepalive bridge: send one keepalive before entering the demo
        // in case sensor_poll is between cycles.  sensor_poll (priority 2)
        // preempts during the demo, but this covers the handoff gap.
        if port_idx < 6 {
            sensor::rx_flush(port_idx as usize);
            unsafe { sensor::lump_keepalive(sensor::PORTS[port_idx as usize]); }
        }

        let sandboxed = user_app_io::is_sandbox_mode();
        let debug_mode = user_app_io::is_debug_mode();
        let result_val;

        if debug_mode {
            // ── Debug mode: delegate to idle (Thread mode, privileged) ──
            // Runs in Thread mode so DebugMonitor can halt for GDB breakpoints.
            // No MPU restrictions — demo has full peripheral access.
            // Status LED: yellow = debug
            led_matrix::set_status_rgb(pins::STATUS_TOP, 0x8000, 0x6000, 0);
            unsafe { led_matrix::update() };

            sandbox::request_debug_run(addr);
            while !sandbox::sandbox_done() {
                watchdog::feed();
                Mono::delay(1.millis()).await;
            }
            let (result, _faulted) = sandbox::sandbox_result();
            result_val = result;
        } else if sandboxed {
            // ── Sandboxed mode: delegate to idle (Thread mode) ──
            // SVC can only fire from Thread mode (SVCall priority 0xF0
            // is lower than any RTIC dispatcher in Handler mode).
            // Status LED: cyan = sandboxed
            led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0x6000, 0x8000);
            unsafe { led_matrix::update() };

            sandbox::request_sandbox(addr);
            // Poll-wait: yields at 1ms so firmware tasks (priority 2)
            // keep running — heartbeat, sensor, motor all preempt us.
            while !sandbox::sandbox_done() {
                watchdog::feed();
                Mono::delay(1.millis()).await;
            }
            let (result, faulted) = sandbox::sandbox_result();
            if faulted {
                let fault_addr = sandbox::last_fault_addr();
                let mut tmp = [0u8; 80];
                let mut w = shell::BufWriter::new(&mut tmp);
                let _ = core::fmt::Write::write_fmt(
                    &mut w,
                    format_args!("FAULT: MMFSR=0x{:02X} addr=0x{:08X}\r\n", result, fault_addr),
                );
                user_app_io::write(w.written());
                result_val = 0xDEAD_DEAD;
            } else {
                result_val = result;
            }
        } else {
            // ── Privileged mode: direct function pointers ──
            // Status LED: magenta = privileged
            led_matrix::set_status_rgb(pins::STATUS_TOP, 0x8000, 0, 0x8000);
            unsafe { led_matrix::update() };

            let api = user_app_io::build_api();
            let entry: extern "C" fn(*const spike_hub_api::MonitorApi) -> u32 =
                unsafe { core::mem::transmute(addr | 1) };

            // setjmp: save recovery context before calling demo.
            // If demo is killed (Ctrl-C / kill / button), any callback
            // longjmps here with jumped == 1, terminating the demo.
            let jumped = unsafe { user_app_io::abort_setjmp(user_app_io::jmpbuf_ptr()) };
            if jumped != 0 {
                result_val = 0xDEAD_0001;
            } else {
                user_app_io::set_jmpbuf_valid(true);
                result_val = entry(&api);
                user_app_io::set_jmpbuf_valid(false);
            }
        }

        // Report result through user_app_io buffer
        let mut tmp = [0u8; 80];
        let mut w = shell::BufWriter::new(&mut tmp);
        let _ = core::fmt::Write::write_fmt(
            &mut w,
            format_args!("=> {} (0x{:08X})\r\nspike> ", result_val, result_val),
        );
        user_app_io::write(w.written());

        // Pend USB ISR so it drains user_app_io to the host.
        cortex_m::peripheral::NVIC::pend(stm32f4::stm32f413::Interrupt::OTG_FS);

        user_app_io::set_running(false);

        // If user-app was killed, clean up hardware state.
        if user_app_io::is_aborted() {
            if is_paused() { toggle_pause(); }
            // Brake all motor ports and stop sound
            for p in 0..6 { motor::brake(p); }
            sound::stop();
        }

        sensor::demo_clear_port();
        sensor::demo_clear_motor_port();
        task_state::mark_idle(task_state::RUN_DEMO);

        // Status LED: green = idle
        led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0x6000, 0);
        unsafe { led_matrix::update() };
    }

    /// test_all: probe ports, exercise a ballet of matrix patterns,
    /// status LED colors, motors, sound, and react to color sensor.
    ///
    /// Respects the global pause/continue button toggle.
    #[task(shared = [sensor, shell], priority = 1)]
    async fn test_all(mut cx: test_all::Context) {
        task_state::mark_active(task_state::TEST_ALL);

        // Helper: wait while respecting pause
        macro_rules! ballet_delay {
            ($ms:expr) => {{
                let mut remaining = $ms as u32;
                while remaining > 0 {
                    while is_paused() {
                        // Yellow breathing while paused
                        led_matrix::set_status_rgb(pins::STATUS_BOT, 0x8000, 0x6000, 0);
                        unsafe { led_matrix::update() };
                        Mono::delay(100.millis()).await;
                    }
                    let chunk = core::cmp::min(remaining, 50);
                    Mono::delay(chunk.millis()).await;
                    remaining -= chunk;
                }
            }};
        }

        // ── Phase 1: LED matrix pattern parade ──
        cx.shared.shell.lock(|sh| sh.push(b"[test_all] matrix patterns\r\n"));

        let patterns: [(&[u8; 25], &str); 7] = [
            (&led_matrix::patterns::HEART, "heart"),
            (&led_matrix::patterns::CHECK, "check"),
            (&led_matrix::patterns::CROSS, "cross"),
            (&led_matrix::patterns::ARROW_UP, "arrow"),
            (&led_matrix::patterns::USB_ICON, "usb"),
            (&led_matrix::patterns::DFU_ICON, "dfu"),
            (&led_matrix::patterns::ALL_ON, "all"),
        ];
        for (pat, _name) in patterns.iter() {
            led_matrix::show_pattern(pat, 50);
            unsafe { led_matrix::update() };
            ballet_delay!(400);
        }

        // Digit sweep 0–9
        for d in 0..=9u8 {
            let pat = led_matrix::patterns::digit(d);
            led_matrix::show_pattern(&pat, 50);
            unsafe { led_matrix::update() };
            ballet_delay!(200);
        }

        // Clear matrix
        led_matrix::show_pattern(&[0u8; 25], 0);
        unsafe { led_matrix::update() };

        // ── Phase 2: pixel wave — sweep across 5×5 ──
        cx.shared.shell.lock(|sh| sh.push(b"[test_all] pixel wave\r\n"));
        for pass in 0..3u8 {
            let brightness = 30 + pass * 25;
            for i in 0..25usize {
                led_matrix::set_pixel(i, brightness as u16);
                unsafe { led_matrix::update() };
                ballet_delay!(40);
            }
            for i in (0..25usize).rev() {
                led_matrix::set_pixel(i, 0);
                unsafe { led_matrix::update() };
                ballet_delay!(40);
            }
        }

        // ── Phase 3: status LED color cycle ──
        cx.shared.shell.lock(|sh| sh.push(b"[test_all] status LED colors\r\n"));
        let colors: [(u16, u16, u16); 7] = [
            (0xFFFF, 0, 0),           // red
            (0, 0xFFFF, 0),           // green
            (0, 0, 0xFFFF),           // blue
            (0xFFFF, 0xFFFF, 0),      // yellow
            (0, 0xFFFF, 0xFFFF),      // cyan
            (0xFFFF, 0, 0xFFFF),      // magenta
            (0xFFFF, 0xFFFF, 0xFFFF), // white
        ];
        for &(r, g, b) in colors.iter() {
            led_matrix::set_status_rgb(pins::STATUS_TOP, r, g, b);
            led_matrix::set_status_rgb(pins::STATUS_BOT, r, g, b);
            unsafe { led_matrix::update() };
            ballet_delay!(300);
        }

        // ── Phase 4: sound scale ──
        cx.shared.shell.lock(|sh| sh.push(b"[test_all] sound scale\r\n"));
        let notes: [u32; 8] = [523, 587, 659, 698, 784, 880, 988, 1047]; // C5..C6
        for &freq in notes.iter() {
            sound::play(freq);
            ballet_delay!(150);
        }
        sound::stop();
        ballet_delay!(200);

        // ── Phase 5: motor exercise (ports A and B) ──
        cx.shared.shell.lock(|sh| sh.push(b"[test_all] motors A+B\r\n"));
        // Forward ramp
        for speed in (0..=60i32).step_by(10) {
            motor::set(0, speed);
            motor::set(1, speed);
            ballet_delay!(100);
        }
        ballet_delay!(300);
        // Reverse ramp
        for speed in (0..=60i32).step_by(10) {
            motor::set(0, -speed);
            motor::set(1, -speed);
            ballet_delay!(100);
        }
        ballet_delay!(300);
        // Stop
        motor::set(0, 0);
        motor::set(1, 0);

        // ── Phase 6: color sensor react (port F) — if sensor is active ──
        let sensor_active = cx.shared.sensor.lock(|s|
            s.status == sensor::Status::Data &&
            s.type_id == sensor::TYPE_SPIKE_COLOR_SENSOR
        );
        if sensor_active {
            cx.shared.shell.lock(|sh| sh.push(b"[test_all] color sensor react (5 s)\r\n"));
            let start_tick = 0u32;
            let mut tick = start_tick;
            while tick < 100 { // 100 × 50 ms = 5 s
                while is_paused() {
                    Mono::delay(100.millis()).await;
                }
                let (r, g, b, _i) = cx.shared.sensor.lock(|s| s.rgbi());
                // Map sensor RGBI (0–1024 typical) to 16-bit LED values
                let lr = ((r as u32).min(1024) * 64) as u16;
                let lg = ((g as u32).min(1024) * 64) as u16;
                let lb = ((b as u32).min(1024) * 64) as u16;
                led_matrix::set_status_rgb(pins::STATUS_TOP, lr, lg, lb);
                led_matrix::set_status_rgb(pins::STATUS_BOT, lr, lg, lb);
                // Also map to matrix brightness proportional to intensity
                let bri = (((r as u32 + g as u32 + b as u32) / 3).min(1024) * 100 / 1024) as u16;
                led_matrix::show_pattern(&led_matrix::patterns::ALL_ON, bri);
                unsafe { led_matrix::update() };
                Mono::delay(50.millis()).await;
                tick += 1;
            }
        } else {
            cx.shared.shell.lock(|sh| sh.push(b"[test_all] no color sensor -- skipping\r\n"));
        }

        // ── Finale: heart + chime ──
        led_matrix::show_pattern(&led_matrix::patterns::HEART, 60);
        led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0xFFFF, 0);
        led_matrix::set_status_rgb(pins::STATUS_BOT, 0, 0xFFFF, 0);
        unsafe { led_matrix::update() };
        sound::play(1047);
        ballet_delay!(150);
        sound::play(1319);
        ballet_delay!(150);
        sound::play(1568);
        ballet_delay!(300);
        sound::stop();

        ballet_delay!(1000);

        // Restore idle state
        led_matrix::show_pattern(&[0u8; 25], 0);
        led_matrix::set_pixel(12, 5); // heartbeat dot
        if is_paused() {
            led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0xA000, 0);
        } else {
            led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0x6000, 0);
        }
        led_matrix::set_status_rgb(pins::STATUS_BOT, 0, 0, 0);
        unsafe { led_matrix::update() };

        cx.shared.shell.lock(|sh| sh.push(b"[test_all] done\r\n"));
        task_state::mark_idle(task_state::TEST_ALL);
    }

    /// Disconnect USB serial for 2 seconds so the host terminal
    /// loses the device and can reconnect fresh.
    #[task(priority = 2)]
    async fn reconnect_ser(_cx: reconnect_ser::Context) {
        task_state::mark_active(task_state::RECONNECT_SER);

        // Force USB disconnect by pulling D+ low (disable USB peripheral)
        // OTG_FS GCCFG register (0x5000_0038): bit 16 = PWRDWN
        // Clearing PWRDWN disconnects the internal pull-up → host sees disconnect
        unsafe {
            let gccfg = core::ptr::read_volatile(0x5000_0038 as *const u32);
            core::ptr::write_volatile(0x5000_0038 as *mut u32, gccfg & !(1 << 16));
        }

        // Status LED: red blink during disconnect
        led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0, 0);
        unsafe { led_matrix::update() };

        Mono::delay(2000.millis()).await;

        // Re-enable USB pull-up
        unsafe {
            let gccfg = core::ptr::read_volatile(0x5000_0038 as *const u32);
            core::ptr::write_volatile(0x5000_0038 as *mut u32, gccfg | (1 << 16));
        }

        // Restore status LED
        if is_paused() {
            led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0xA000, 0);
        } else {
            led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0x6000, 0);
        }
        unsafe { led_matrix::update() };

        task_state::mark_idle(task_state::RECONNECT_SER);
    }

    /// Ring-button — three firmware-level actions by hold duration.
    ///
    /// Runs at **priority 2** so it always preempts user-apps (pri 1)
    /// and can never be blocked by a stuck demo.  The firmware — not
    /// the user-app — controls pause/resume/kill/power-off.
    ///
    ///   Short press (< 1 s)  →  **pause / resume** user-app
    ///     Paused:  yellow LED + double-beep.  Firmware holds the app
    ///     inside `demo_delay_ms`, maintaining sensor keepalive.
    ///     Resumed: cyan flash + high beep.
    ///
    ///   Medium press (1 – 3 s) released  →  **kill** user-app
    ///     Sets ABORT flag.  All MonitorApi callbacks return early,
    ///     motors are braked.  Red LED + descending beep.
    ///
    ///   Long press (3 – 5 s) released  →  **power off** (deep sleep)
    ///     Descending chime, PA13 LOW. Cold-boot on next center press.
    ///
    ///   Very long press (≥ 5 s)  →  **forced clean shutdown**
    ///
    /// Left button → DFU mode (unchanged).
    #[task(priority = 2)]
    async fn button_poll(_cx: button_poll::Context) {
        task_state::mark_active(task_state::BUTTON_POLL);
        let mut center_hold: u32 = 0;
        let mut release_count: u32 = 0;

        // 50 ms poll → 20 ticks per second
        const TICKS_1S: u32 = 20;
        const TICKS_3S: u32 = 3 * TICKS_1S;  // 60  — kill zone starts
        const TICKS_6S: u32 = 6 * TICKS_1S;  // 120 — poweroff zone starts
        const TICKS_8S: u32 = 8 * TICKS_1S;  // 160 — forced clean shutdown

        const DEBOUNCE_RELEASE: u32 = 3;  // 150 ms glitch tolerance
        const PULSE_BRIGHT: u32 = 4;      // 200 ms bright window

        loop {
            let btns = read_buttons();
            let center_pressed = btns & BTN_CENTER != 0;

            // Debounce: tolerate brief ADC glitches during a hold
            if center_pressed {
                release_count = 0;
            } else if center_hold > 0 {
                release_count += 1;
                if release_count < DEBOUNCE_RELEASE {
                    Mono::delay(50.millis()).await;
                    continue;
                }
            }

            if center_pressed {
                center_hold += 1;

                // ── ≥8 s: forced clean shutdown (safety net) ──
                if center_hold >= TICKS_8S {
                    for p in 0..6u32 { motor::brake(p); }
                    led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0, 0);
                    unsafe { led_matrix::update() };
                    sound::play(880);
                    power::delay_ms(100);
                    sound::play(440);
                    power::delay_ms(100);
                    sound::play(220);
                    power::delay_ms(100);
                    sound::stop();
                    power::clean_shutdown();
                }

                // Audible tick at each whole-second boundary
                if center_hold % TICKS_1S == 1 {
                    let tick_freq = if center_hold >= TICKS_6S {
                        220 // purple/poweroff zone: very low
                    } else if center_hold >= TICKS_3S {
                        440 // red/kill zone: low tone
                    } else if center_hold >= TICKS_1S {
                        660 // orange/approaching kill: medium tone
                    } else {
                        880 // white zone: high tone
                    };
                    sound::play(tick_freq);
                } else if center_hold % TICKS_1S == 3 {
                    sound::stop();
                }

                // ── Pulsing LED feedback by zone ──
                // Zones: white(0-1s) → orange(1-3s) → red(3-6s) → purple(6s+)
                let bright = (center_hold % TICKS_1S) < PULSE_BRIGHT;
                let (r, g, b) = if center_hold >= TICKS_6S {
                    // ≥6 s: purple pulse (release = power off)
                    if bright {
                        (0x8000_u16, 0x0000_u16, 0xFFFF_u16)
                    } else {
                        (0x2000, 0x0000, 0x3000)
                    }
                } else if center_hold >= TICKS_3S {
                    // 3–6 s: red pulse (kill zone — SIGKILL)
                    if bright {
                        (0xFFFF_u16, 0x1000_u16, 0x0000_u16)
                    } else {
                        (0x3000, 0x0400, 0x0000)
                    }
                } else if center_hold >= TICKS_1S {
                    // 1–3 s: orange pulse (approaching kill)
                    if bright {
                        (0xFFFF_u16, 0x6000_u16, 0x0000_u16)
                    } else {
                        (0x3000, 0x1800, 0x0000)
                    }
                } else {
                    // 0–1 s: white pulse (short press)
                    if bright {
                        (0xFFFF_u16, 0xFFFF_u16, 0xFFFF_u16)
                    } else {
                        (0x2000, 0x2000, 0x2000)
                    }
                };
                led_matrix::set_status_rgb(pins::STATUS_TOP, r, g, b);
                unsafe { led_matrix::update() };
            } else if center_hold > 0 {
                // ── Released (debounce passed) ──
                release_count = 0;
                sound::stop();

                if center_hold >= TICKS_6S {
                    // ── Power-off zone (≥6 s): deep sleep ──
                    for p in 0..6u32 { motor::brake(p); }
                    if user_app_io::is_running() {
                        user_app_io::request_abort();
                        if sandbox::is_sandboxed() {
                            unsafe { sandbox::force_kill_sandbox(); }
                        }
                    }
                    // Feedback: deep 3-tone descending
                    sound::play(440);
                    Mono::delay(80.millis()).await;
                    sound::play(220);
                    Mono::delay(80.millis()).await;
                    sound::play(110);
                    Mono::delay(80.millis()).await;
                    sound::stop();
                    power::deep_sleep();

                } else if center_hold >= TICKS_1S {
                    // ── Kill zone (1–6 s): SIGKILL running user-app ──
                    if user_app_io::is_running() {
                        user_app_io::request_abort();
                        // Hard-kill sandboxed demo (force redirect)
                        if sandbox::is_sandboxed() {
                            unsafe { sandbox::force_kill_sandbox(); }
                        }
                        // Stop motors + sound immediately
                        for p in 0..6u32 { motor::brake(p); }
                        sound::stop();
                        // Feedback: red flash + 3-tone descending
                        led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0, 0);
                        unsafe { led_matrix::update() };
                        sound::play(880);
                        Mono::delay(60.millis()).await;
                        sound::play(440);
                        Mono::delay(60.millis()).await;
                        sound::play(220);
                        Mono::delay(60.millis()).await;
                        sound::stop();
                    } else {
                        // Nothing to kill — short low beep
                        sound::play(440);
                        Mono::delay(40.millis()).await;
                        sound::stop();
                    }

                } else {
                    // ── Short press (< 1 s) ──
                    if user_app_io::is_running() {
                        // Toggle pause / resume
                        let now_paused = toggle_pause();
                        if now_paused {
                            // Paused: yellow LED + double beep 880→660
                            led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0xA000, 0);
                            unsafe { led_matrix::update() };
                            sound::play(880);
                            Mono::delay(60.millis()).await;
                            sound::stop();
                            Mono::delay(40.millis()).await;
                            sound::play(660);
                            Mono::delay(60.millis()).await;
                            sound::stop();
                        } else {
                            // Resumed: cyan flash + high beep 1760
                            led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0xFFFF, 0xFFFF);
                            unsafe { led_matrix::update() };
                            sound::play(1760);
                            Mono::delay(60.millis()).await;
                            sound::stop();
                        }
                    } else {
                        // Not running — re-run last demo or toggle pause
                        let addr = user_app_io::last_run_addr();
                        if addr != 0 {
                            // Clear pause state for fresh start
                            if is_paused() { toggle_pause(); }
                            // Green flash + ascending chirp 660→1320
                            led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0xFFFF, 0);
                            unsafe { led_matrix::update() };
                            sound::play(660);
                            Mono::delay(60.millis()).await;
                            sound::play(1320);
                            Mono::delay(60.millis()).await;
                            sound::stop();
                            // Re-launch (sandboxed)
                            user_app_io::request(addr, true);
                        } else {
                            // No demo ever run — toggle global pause
                            let now_paused = toggle_pause();
                            if now_paused {
                                led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0xA000, 0);
                                unsafe { led_matrix::update() };
                                sound::play(880);
                                Mono::delay(60.millis()).await;
                                sound::stop();
                                Mono::delay(40.millis()).await;
                                sound::play(660);
                                Mono::delay(60.millis()).await;
                                sound::stop();
                            } else {
                                led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0xFFFF, 0xFFFF);
                                unsafe { led_matrix::update() };
                                sound::play(1760);
                                Mono::delay(60.millis()).await;
                                sound::stop();
                            }
                        }
                    }
                    Mono::delay(40.millis()).await;
                }
                center_hold = 0;

                // Restore status LED based on current state
                if user_app_io::is_running() {
                    if is_paused() {
                        led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0xA000, 0);
                    } else {
                        led_matrix::set_status_rgb(pins::STATUS_TOP, 0x8000, 0, 0x8000);
                    }
                } else if is_paused() {
                    led_matrix::set_status_rgb(pins::STATUS_TOP, 0xFFFF, 0xA000, 0);
                } else {
                    led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0x6000, 0);
                }
                unsafe { led_matrix::update() };
            }

            // Left button → DFU mode
            if btns & BTN_LEFT != 0 {
                // Safety: stop motors before entering DFU
                for p in 0..6u32 { motor::brake(p); }
                led_matrix::show_pattern(&led_matrix::patterns::DFU_ICON, 60);
                unsafe { led_matrix::update() };
                power::delay_ms(300);
                power::enter_dfu();
            }

            // Right button → launch from SPI flash (slot 1), killing any running demo first.
            // Slot 0 is stomped by LEGO bootloader on every power cycle.
            if btns & BTN_RIGHT != 0 && center_hold == 0 {
                // ── Kill running demo first (SIGKILL) ──
                if user_app_io::is_running() {
                    user_app_io::request_abort();
                    if sandbox::is_sandboxed() {
                        unsafe { sandbox::force_kill_sandbox(); }
                    }
                    for p in 0..6u32 { motor::brake(p); }
                    sound::stop();
                    // Wait for demo task to actually finish (up to 500 ms)
                    for _ in 0..25u32 {
                        Mono::delay(20.millis()).await;
                        if !user_app_io::is_running() { break; }
                    }
                }

                // ── Launch from flash ──
                const FLASH_RUN_MAGIC: u32 = 0x464C_524E; // "FLRN"
                const SLOT1: u32 = 0x10000; // 64KB offset — slot 0 stomped by bootloader
                let mut hdr = [0u8; 8];
                ext_flash::read(SLOT1, &mut hdr);
                let magic = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
                let size = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]) as usize;
                if magic == FLASH_RUN_MAGIC && size > 0 && size <= upload::UPLOAD_BUF_SIZE {
                    // Blue flash = loading from flash
                    led_matrix::set_status_rgb(pins::STATUS_TOP, 0, 0, 0xFFFF);
                    unsafe { led_matrix::update() };
                    let buf = upload::upload_buf_mut();
                    ext_flash::read(SLOT1 + 8, &mut buf[..size]);
                    let ram_addr = upload::upload_buf_addr();
                    user_app_io::request(ram_addr, true);
                    // Rising chirp = launched
                    sound::play(880);
                    Mono::delay(60.millis()).await;
                    sound::play(1760);
                    Mono::delay(60.millis()).await;
                    sound::stop();
                } else {
                    // No valid binary — double low beep
                    sound::play(330);
                    Mono::delay(100.millis()).await;
                    sound::stop();
                    Mono::delay(50.millis()).await;
                    sound::play(220);
                    Mono::delay(100.millis()).await;
                    sound::stop();
                }
                // Debounce: wait for release
                loop {
                    Mono::delay(50.millis()).await;
                    if read_buttons() & BTN_RIGHT == 0 { break; }
                }
            }

            Mono::delay(50.millis()).await;
        }
    }

    // ── Sensor UART interrupt handlers ──
    // Each port's UART ISR pushes received bytes into sensor.rs ring buffer.
    // Priority 3: highest — ensures no byte loss from UART overrun.
    // Only the port with RXNEIE enabled will actually fire.

    #[task(binds = USART7, priority = 3)]
    fn uart7_isr(_cx: uart7_isr::Context) {
        // Port A — UART7
        unsafe { sensor::uart_isr_handler(0x4000_7800, 0); }
    }

    #[task(binds = USART4, priority = 3)]
    fn uart4_isr(_cx: uart4_isr::Context) {
        // Port B — UART4
        unsafe { sensor::uart_isr_handler(0x4000_4C00, 1); }
    }

    #[task(binds = USART8, priority = 3)]
    fn uart8_isr(_cx: uart8_isr::Context) {
        // Port C — UART8
        unsafe { sensor::uart_isr_handler(0x4000_7C00, 2); }
    }

    #[task(binds = UART5, priority = 3)]
    fn uart5_isr(_cx: uart5_isr::Context) {
        // Port D — UART5
        unsafe { sensor::uart_isr_handler(0x4000_5000, 3); }
    }

    #[task(binds = UART10, priority = 3)]
    fn uart10_isr(_cx: uart10_isr::Context) {
        // Port E — UART10
        unsafe { sensor::uart_isr_handler(0x4001_1C00, 4); }
    }

    #[task(binds = UART9, priority = 3)]
    fn uart9_isr(_cx: uart9_isr::Context) {
        // Port F — UART9
        unsafe { sensor::uart_isr_handler(0x4001_1800, 5); }
    }
}

// ════════════════════════════════════════════════════════════════
// Cortex-M exception handlers (outside RTIC app module)
// ════════════════════════════════════════════════════════════════

/// SVCall handler — dispatches SVC calls from sandboxed demos.
/// RTIC does not use SVCall, so we can claim this vector.
#[cortex_m_rt::exception]
unsafe fn SVCall() {
    sandbox::SVCall_handler();
}

/// MemManage fault handler — catches MPU violations from sandboxed demos.
/// Recovers cleanly instead of escalating to HardFault.
#[cortex_m_rt::exception]
unsafe fn MemoryManagement() {
    sandbox::MemManage_handler();
}

/// DebugMonitor exception — fires on DWT watchpoint hits.
/// Records which comparator matched and the faulting PC.
#[cortex_m_rt::exception]
unsafe fn DebugMonitor() {
    dwt::DebugMonitor_handler();
}
