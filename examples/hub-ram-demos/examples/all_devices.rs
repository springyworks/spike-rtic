//! all_devices — Parallel multi-peripheral demo.
//!
//! Exercises sensor + dual motors + LED + sound + buttons simultaneously.
//! Firmware RTIC tasks (sensor_poll, motor_poll, USB) preempt at priority 2
//! while this demo runs at priority 1 — true preemptive multitasking.
//!
//! What runs in parallel (firmware, priority 2, preemptive):
//!   - sensor_poll:  drains ultrasonic LUMP data on port E every 20ms
//!   - motor_poll:   drains motor encoder on port A every 20ms
//!   - usb_interrupt: CDC serial I/O every 5ms
//!   - UART ISRs:    byte-level receive for sensor + motor (priority 3)
//!
//! What this demo does (user app, priority 1):
//!   - Reads ultrasonic distance via sensor_read()
//!   - Reads motor A encoder via motor_position()
//!   - Drives motor A + B via motor_set() (open-loop PWM, H-bridge)
//!   - Updates 5×5 LED matrix every cycle (distance bar + motor indicators)
//!   - Plays proximity tones via sound_play()
//!   - Polls buttons for speed/direction/exit control
//!   - delay_ms() keeps LUMP keepalive for both sensor + motor
//!
//! Requires:
//!   - Ultrasonic sensor on port E (start with: sensor e)
//!   - Motor on port A (auto-detected by motor_poll)
//!   - Optionally motor on port B
//!   - MonitorApi v7+
//!
//! Build:
//!   cd examples/hub-ram-demos
//!   cargo build --release --example all_devices

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

const PORT_A: u32 = 0;
const PORT_B: u32 = 1;
const LOOP_MS: u32 = 50; // 20 Hz control loop

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    api.print(b"=== All Devices: Preemptive Multitask ===\r\n");
    api.print(b"  Sensor: ultrasonic port E (RTIC prio 2)\r\n");
    api.print(b"  Motor:  A encoder + B drive (RTIC prio 2)\r\n");
    api.print(b"  Demo:   this loop (prio 1, preempted)\r\n");
    api.print(b"  Buttons: L=reverse R=speed Center=exit\r\n\r\n");

    clear_matrix(api);

    let mut speed: i32 = 40;
    let mut direction: i32 = 1;
    let mut prev_buttons: u8 = 0;
    let mut cycle: u32 = 0;
    let mut last_dist: i16 = -1;
    let mut motor_start_pos: i32 = (api.motor_position)();

    loop {
        cycle += 1;

        // ── Read ultrasonic distance (from sensor_poll ring buffer) ──
        let mut buf = [0u8; 8];
        let n = (api.sensor_read)(buf.as_mut_ptr(), 8);
        if n >= 2 {
            last_dist = i16::from_le_bytes([buf[0], buf[1]]);
        }

        // ── Read motor A position (from motor_poll) ──
        let motor_pos = (api.motor_position)();
        let degrees_moved = motor_pos - motor_start_pos;

        // ── Read buttons (edge-detect) ──
        let buttons = (api.read_buttons)();
        let pressed = buttons & !prev_buttons; // rising edges
        prev_buttons = buttons;

        if pressed & 0x02 != 0 {
            // Center button — exit
            (api.motor_set)(PORT_A, 0);
            (api.motor_set)(PORT_B, 0);
            (api.sound_stop)();
            clear_matrix(api);
            api.print(b"\r\n=== Exit (center button) ===\r\n");
            return 0;
        }
        if pressed & 0x01 != 0 {
            // Left button — reverse direction
            direction = -direction;
            motor_start_pos = motor_pos;
        }
        if pressed & 0x04 != 0 {
            // Right button — cycle speed
            speed = match speed {
                s if s <= 30 => 50,
                s if s <= 50 => 75,
                s if s <= 75 => 100,
                _ => 30,
            };
        }

        // ── Drive motors (open-loop PWM via H-bridge) ──
        let spd = speed * direction;
        (api.motor_set)(PORT_A, spd);
        (api.motor_set)(PORT_B, -spd); // opposite for spin

        // ── Proximity sound (closer = higher pitch) ──
        if last_dist >= 0 && last_dist < 2000 {
            let d = last_dist as u32;
            if d < 400 {
                // Map 0-400mm → 2000-200 Hz (inverse)
                let freq = 2000 - (d * 9 / 2);
                (api.sound_play)(freq.max(200));
            } else {
                (api.sound_stop)();
            }
        } else {
            (api.sound_stop)();
        }

        // ── LED matrix: distance bar (rows 0-2) + motor info (rows 3-4) ──
        update_display(api, last_dist, degrees_moved, speed, direction);

        // ── Print status every 10 cycles (every 500ms) ──
        if cycle % 10 == 0 {
            print_status(api, last_dist, motor_pos, degrees_moved, speed, direction);
        }

        // ── yield to firmware — maintains LUMP keepalive for BOTH ──
        // sensor_poll + motor_poll preempt during this wait
        (api.delay_ms)(LOOP_MS);

        // Auto-exit after ~60 seconds
        if cycle > 1200 {
            (api.motor_set)(PORT_A, 0);
            (api.motor_set)(PORT_B, 0);
            (api.sound_stop)();
            clear_matrix(api);
            api.print(b"\r\n=== Done (60s timeout) ===\r\n");
            return 0;
        }
    }
}

// ── LED Matrix Display ──
// Row 0-2 (pixels 0-14): distance bar graph (0mm = full bright, 2000mm = off)
// Row 3 (pixels 15-19): motor direction arrows
// Row 4 (pixels 20-24): speed indicator dots

fn update_display(api: &MonitorApi, dist: i16, degrees: i32, speed: i32, dir: i32) {
    // Distance bar: 15 pixels, brightness proportional to proximity
    let bar_level = if dist >= 0 && dist < 2000 {
        // 0mm = 15 pixels lit, 2000mm = 0 pixels lit
        15u32 - ((dist as u32).min(1999) * 15 / 2000)
    } else {
        0 // out of range = all off
    };

    for i in 0u32..15 {
        let brightness = if i < bar_level {
            // Closer pixels brighter (max 100)
            100 - (i * 5)
        } else {
            0
        };
        (api.set_pixel)(i, brightness);
    }

    // Direction arrows on row 3
    for i in 15u32..20 {
        (api.set_pixel)(i, 0);
    }
    if dir > 0 {
        // Right arrow: pixels 16, 17, 18
        (api.set_pixel)(16, 40);
        (api.set_pixel)(17, 70);
        (api.set_pixel)(18, 40);
    } else {
        // Left arrow: pixels 16, 17, 18
        (api.set_pixel)(18, 40);
        (api.set_pixel)(17, 70);
        (api.set_pixel)(16, 40);
    }
    // Show rotation with side indicators
    let rot = ((degrees.unsigned_abs() / 45) % 2) as u32;
    (api.set_pixel)(15, if rot == 0 { 50 } else { 0 });
    (api.set_pixel)(19, if rot == 1 { 50 } else { 0 });

    // Speed dots on row 4: more dots = faster
    let dots = (speed.unsigned_abs() / 25).min(4) as u32;
    for i in 0u32..5 {
        let px = 20 + i;
        (api.set_pixel)(px, if i < dots { 60 } else { 0 });
    }

    (api.update_leds)();
}

// ── Status line ──

fn print_status(api: &MonitorApi, dist: i16, pos: i32, moved: i32, speed: i32, dir: i32) {
    api.print(b"dist=");
    if dist < 0 || dist >= 2000 {
        api.print(b"---");
    } else {
        print_i32(api, dist as i32);
        api.print(b"mm");
    }
    api.print(b" pos=");
    print_i32(api, pos);
    api.print(b"deg moved=");
    print_i32(api, moved);
    api.print(b" spd=");
    print_i32(api, speed * dir);
    api.print(b"%\r\n");
}

fn clear_matrix(api: &MonitorApi) {
    for i in 0..25u32 { (api.set_pixel)(i, 0); }
    (api.update_leds)();
}

fn print_i32(api: &MonitorApi, v: i32) {
    if v < 0 {
        api.print(b"-");
        print_u32(api, (-(v as i64)) as u32);
    } else {
        print_u32(api, v as u32);
    }
}

fn print_u32(api: &MonitorApi, mut v: u32) {
    if v == 0 { api.print(b"0"); return; }
    let mut buf = [0u8; 10];
    let mut i = 10usize;
    while v > 0 { i -= 1; buf[i] = b'0' + (v % 10) as u8; v /= 10; }
    (api.write_fn)(api.context, buf[i..].as_ptr(), (10 - i) as u32);
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
