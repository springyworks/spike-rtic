//! Event-driven demo -- "we hate polling"
//!
//! Four concurrent tasks, zero polling, pure event-driven.
//! Uses `wait_event` to sleep until firmware fires an event -- the RTIC way.
//!
//! ## Four tasks (multiplexed via single event loop)
//!
//! 1. **Input handler** (EVT_INPUT): host sends commands via `send` shell
//!    command.  Responds instantly to h/r/q/a single-char commands.
//!    q = quit.  Never missed -- event wakes the MCU immediately.
//! 2. **Button handler** (EVT_BUTTON): left=beep, right=IMU read,
//!    center=exit.  No polling -- firmware wakes us on state change.
//! 3. **Thermal + IMU heartbeat** (EVT_TIMEOUT, 2s): SoC die temp,
//!    battery voltage, IMU accel/gyro snapshot.  Hub LED by temp.
//! 4. **LED heartbeat** (blink pattern toggles each 2s tick).
//!
//! ## Event flow (no polling, no busy-wait)
//!
//! ```text
//!   Host terminal                User app              Firmware (RTIC)
//!   -------------                --------              ---------------
//!   spike> send h          -->   push_input()    -->   EVT_INPUT fires
//!                                wait_event wakes <--  immediate!
//!                                read_input -> "h"
//!                                print help
//!
//!   spike> send q          -->   push_input()    -->   EVT_INPUT fires
//!                                wait_event wakes <--  immediate!
//!                                read_input -> "q"
//!                                break -> clean exit
//!
//!   (2 seconds pass)             wait_event times out
//!                                EVT_TIMEOUT -> thermal read
//!
//!   The MCU halts between events (ARM WFI).
//! ```
//!
//! ## Usage
//!
//! ```text
//! spike> upload event_driven.bin
//! spike> go!
//! spike> send h        # help
//! spike> send a        # show all (thermal + IMU + battery)
//! spike> send r        # reset LEDs
//! spike> send q        # quit demo
//! ```
//!
//! Requires: API v12+, no external sensors needed.
//! Exit: center button OR `send q`.

#![no_std]
#![no_main]

use spike_hub_api::{
    MonitorApi, BTN_CENTER, BTN_LEFT, BTN_RIGHT,
    EVT_BUTTON, EVT_INPUT, EVT_TIMEOUT,
};

/// Main loop timeout -- 2 seconds between heartbeats.
const HEARTBEAT_MS: u32 = 2000;

/// ADC channel for SoC die temperature sensor (STM32F413).
const ADC_CH_TEMP: u32 = 18;
/// ADC channel for battery voltage divider.
const ADC_CH_VBAT: u32 = 11;
/// ADC channel for battery NTC thermistor.
const ADC_CH_NTC: u32 = 8;

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 12 {
        api.print(b"ERR: need API v12+ (read_input)\r\n");
        return 1;
    }

    api.print(b"=== Event-Driven: 4 Tasks, 0 Polling ===\r\n");
    api.print(b"Task 1: Input    -> spike> send {h|r|a|q}\r\n");
    api.print(b"Task 2: Buttons  -> left=beep right=IMU center=exit\r\n");
    api.print(b"Task 3: Thermal  -> hub LED (2s heartbeat)\r\n");
    api.print(b"Task 4: LED blink heartbeat\r\n");
    api.print(b"Exit: center btn OR send q\r\n\r\n");

    // Debounce: wait for center button release
    for _ in 0..20 {
        if (api.read_buttons)() & BTN_CENTER == 0 { break; }
        (api.delay_ms)(50);
    }

    // Initialize IMU (LSM6DS3 on-board, I2C2 bit-bang)
    let who = (api.imu_init)();
    let imu_ok = who == 0x6A;
    if imu_ok {
        api.print(b"IMU: LSM6DS3 OK (0x6A)\r\n");
    } else {
        api.print(b"IMU: init failed (who=");
        print_u32(api, who);
        api.print(b")\r\n");
    }

    // Initial thermal reading
    thermal_report(api);
    api.print(b"\r\nReady. Type: spike> send h  (for help)\r\n\r\n");

    // Clear matrix, show center dot as "alive" indicator
    for px in 0..25u32 { (api.set_pixel)(px, 0); }
    (api.set_pixel)(12, 40); // center dot
    (api.update_leds)();

    let mut tick: u32 = 0;
    let mut led_on = true;

    // ────────────────── EVENT LOOP ──────────────────
    //
    // Single wait_event covers ALL tasks.  The MCU sleeps (WFI)
    // between events.  EVT_INPUT and EVT_BUTTON wake instantly;
    // EVT_TIMEOUT fires every 2 seconds for the heartbeat.
    //
    // This is the RTIC pattern: declare what you listen for,
    // sleep, react.  No spinning, no polling, no wasted cycles.

    loop {
        let evt = (api.wait_event)(
            EVT_INPUT | EVT_BUTTON,
            HEARTBEAT_MS,
        );

        // ── Task 1: Input handler (immediate, never misses 'q') ──
        if evt & EVT_INPUT != 0 {
            let mut buf = [0u8; 128];
            let n = (api.read_input)(buf.as_mut_ptr(), buf.len() as u32);
            // Process each character in the input
            for i in 0..n as usize {
                match buf[i] {
                    b'q' | b'Q' => {
                        api.print(b"[INPUT] quit\r\n");
                        // Cleanup and exit
                        for px in 0..25u32 { (api.set_pixel)(px, 0); }
                        (api.update_leds)();
                        (api.set_hub_led)(0, 0, 0);
                        (api.sound_stop)();
                        api.print(b"\r\nEvent-driven demo done (quit), ");
                        print_u32(api, tick);
                        api.print(b" ticks.\r\n");
                        return tick;
                    }
                    b'h' | b'H' => {
                        api.print(b"[INPUT] help:\r\n");
                        api.print(b"  h = this help\r\n");
                        api.print(b"  a = show all (thermal + IMU + battery)\r\n");
                        api.print(b"  r = reset LEDs (clear matrix)\r\n");
                        api.print(b"  q = quit demo\r\n");
                    }
                    b'r' | b'R' => {
                        api.print(b"[INPUT] reset LEDs\r\n");
                        for px in 0..25u32 { (api.set_pixel)(px, 0); }
                        (api.set_pixel)(12, 40);
                        (api.update_leds)();
                        (api.set_hub_led)(0, 0, 0);
                        led_on = true;
                    }
                    b'a' | b'A' => {
                        api.print(b"[INPUT] all sensors:\r\n");
                        thermal_report(api);
                        imu_report(api, imu_ok);
                    }
                    b' ' | b'\r' | b'\n' => {} // whitespace: ignore
                    c => {
                        api.print(b"[INPUT] unknown '");
                        api.print(&[c]);
                        api.print(b"' (h=help)\r\n");
                    }
                }
            }
        }

        // ── Task 2: Button handler (pure event, zero latency) ──
        if evt & EVT_BUTTON != 0 {
            let btns = (api.read_buttons)();
            if btns & BTN_CENTER != 0 {
                api.print(b"[BTN] exit\r\n");
                break;
            }
            if btns & BTN_LEFT != 0 {
                (api.sound_play)(440);
                (api.delay_ms)(200);
                (api.sound_stop)();
                api.print(b"[BTN] beep 440Hz\r\n");
            }
            if btns & BTN_RIGHT != 0 {
                api.print(b"[BTN] IMU snapshot:\r\n");
                imu_report(api, imu_ok);
            }
        }

        // ── Task 3 + 4: Heartbeat (2-second timeout) ──
        if evt & EVT_TIMEOUT != 0 {
            tick += 1;

            // Thermal heartbeat
            thermal_report(api);

            // LED heartbeat — blink center pixel
            led_on = !led_on;
            (api.set_pixel)(12, if led_on { 40 } else { 0 });
            (api.update_leds)();
        }
    }

    // Cleanup: LEDs off, sound off
    for px in 0..25u32 { (api.set_pixel)(px, 0); }
    (api.update_leds)();
    (api.set_hub_led)(0, 0, 0);
    (api.sound_stop)();

    api.print(b"\r\nEvent-driven demo done, ");
    print_u32(api, tick);
    api.print(b" heartbeats.\r\n");
    tick
}

// ── Thermal report ────────────────────────────────────────

fn thermal_report(api: &MonitorApi) {
    let raw_t = (api.read_adc)(ADC_CH_TEMP);
    let raw_v = (api.read_adc)(ADC_CH_VBAT);
    let raw_ntc = (api.read_adc)(ADC_CH_NTC);

    // SoC temperature: V_sense = raw * 3.3/4096
    // T(C) = (V_sense - 0.76) / 0.0025 + 25
    let mv_x10 = raw_t * 330 / 4096;
    let temp_cdeg = if mv_x10 > 76 {
        ((mv_x10 - 76) * 100 / 25) + 2500
    } else {
        2500u32.saturating_sub((76 - mv_x10) * 100 / 25)
    };
    let temp_whole = temp_cdeg / 100;
    let temp_frac  = temp_cdeg % 100;

    // Hub LED: blue=cold(<30), green=normal(30-45), red=hot(>45)
    if temp_whole < 30 {
        (api.set_hub_led)(0, 0, 60);
    } else if temp_whole < 45 {
        (api.set_hub_led)(0, 60, 0);
    } else {
        (api.set_hub_led)(60, 0, 0);
    }

    api.print(b"[THERMAL] SoC=");
    print_u32(api, temp_whole);
    api.print(b".");
    if temp_frac < 10 { api.print(b"0"); }
    print_u32(api, temp_frac);
    api.print(b"C  Vbat=");
    print_u32(api, raw_v);
    api.print(b"  NTC=");
    print_u32(api, raw_ntc);
    api.print(b"\r\n");
}

// ── IMU report ────────────────────────────────────────────

fn imu_report(api: &MonitorApi, imu_ok: bool) {
    if !imu_ok {
        api.print(b"  IMU: not available\r\n");
        return;
    }
    let mut buf = [0u8; 12];
    let n = (api.imu_read)(buf.as_mut_ptr(), 12);
    if n != 12 {
        api.print(b"  IMU: read failed\r\n");
        return;
    }
    let ax = i16::from_le_bytes([buf[0], buf[1]]);
    let ay = i16::from_le_bytes([buf[2], buf[3]]);
    let az = i16::from_le_bytes([buf[4], buf[5]]);
    let gx = i16::from_le_bytes([buf[6], buf[7]]);
    let gy = i16::from_le_bytes([buf[8], buf[9]]);
    let gz = i16::from_le_bytes([buf[10], buf[11]]);

    api.print(b"  Accel: X=");
    print_i16(api, ax);
    api.print(b" Y=");
    print_i16(api, ay);
    api.print(b" Z=");
    print_i16(api, az);
    api.print(b"\r\n  Gyro:  X=");
    print_i16(api, gx);
    api.print(b" Y=");
    print_i16(api, gy);
    api.print(b" Z=");
    print_i16(api, gz);
    api.print(b"\r\n");
}

// ── Number printing (no alloc, no core::fmt) ──────────────

fn print_u32(api: &MonitorApi, val: u32) {
    if val == 0 {
        api.print(b"0");
        return;
    }
    let mut buf = [b'0'; 10];
    let mut v = val;
    let mut i = 9;
    while v > 0 {
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if i == 0 { break; }
        i -= 1;
    }
    if val > 0 { i += 1; }
    api.print(&buf[i..]);
}

fn print_i16(api: &MonitorApi, val: i16) {
    if val < 0 {
        api.print(b"-");
        print_u32(api, (-(val as i32)) as u32);
    } else {
        print_u32(api, val as u32);
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
