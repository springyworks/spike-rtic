//! quad_devices — Quad-port demo exercising 4 devices in parallel.
//!
//! Reads BOTH sensors (E=ultrasonic, F=color) while driving BOTH motors
//! (A+B) simultaneously.  All 4 firmware poll tasks run at priority 2,
//! preempting this demo at priority 1.
//!
//! Uses `port_read` (API v8) to read any port's latest data from
//! PORT_STATES[] — no more single-sensor bottleneck.
//!
//! Requires:
//!   - Motor on port A and port B
//!   - Ultrasonic sensor on port E
//!   - Color sensor on port F
//!   - MonitorApi v8+
//!
//! Build:
//!   cd examples/hub-ram-demos
//!   cargo build --release --example quad_devices

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

const PORT_A: u32 = 0;
const PORT_B: u32 = 1;
const PORT_E: u32 = 4;
const PORT_F: u32 = 5;
const LOOP_MS: u32 = 50; // 20 Hz

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 8 {
        api.print(b"ERROR: need API v8+, got v");
        print_u32(api, api.version);
        api.print(b"\r\n");
        return 1;
    }

    api.print(b"=== Quad Devices Demo (API v8) ===\r\n");
    api.print(b"  Motor A+B  |  Ultrasonic E  |  Color F\r\n");
    api.print(b"  L=reverse  R=speed  Center=exit\r\n\r\n");

    clear_matrix(api);

    let mut speed: i32 = 40;
    let mut direction: i32 = 1;
    let mut prev_buttons: u8 = 0;
    let mut cycle: u32 = 0;

    loop {
        cycle += 1;

        // ── Read ultrasonic distance (port E, mode 5: 2 bytes i16 mm) ──
        let mut ebuf = [0u8; 8];
        let en = (api.port_read)(PORT_E, ebuf.as_mut_ptr(), 8);
        let dist: i16 = if en >= 2 {
            i16::from_le_bytes([ebuf[0], ebuf[1]])
        } else {
            -1
        };

        // ── Read color sensor (port F, mode 5: 8 bytes RGBI×u16) ──
        let mut fbuf = [0u8; 8];
        let fn_ = (api.port_read)(PORT_F, fbuf.as_mut_ptr(), 8);
        let (r, g, b, i) = if fn_ >= 8 {
            (
                u16::from_le_bytes([fbuf[0], fbuf[1]]),
                u16::from_le_bytes([fbuf[2], fbuf[3]]),
                u16::from_le_bytes([fbuf[4], fbuf[5]]),
                u16::from_le_bytes([fbuf[6], fbuf[7]]),
            )
        } else {
            (0, 0, 0, 0)
        };

        // ── Read motor positions (ports A+B, mode 2: 4 bytes i32 deg) ──
        let mut abuf = [0u8; 8];
        let an = (api.port_read)(PORT_A, abuf.as_mut_ptr(), 8);
        let pos_a: i32 = if an >= 4 {
            i32::from_le_bytes([abuf[0], abuf[1], abuf[2], abuf[3]])
        } else {
            0
        };

        let mut bbuf = [0u8; 8];
        let bn = (api.port_read)(PORT_B, bbuf.as_mut_ptr(), 8);
        let pos_b: i32 = if bn >= 4 {
            i32::from_le_bytes([bbuf[0], bbuf[1], bbuf[2], bbuf[3]])
        } else {
            0
        };

        // ── Buttons ──
        let buttons = (api.read_buttons)();
        let pressed = buttons & !prev_buttons;
        prev_buttons = buttons;

        if pressed & 0x02 != 0 {
            // Center — exit
            (api.motor_set)(PORT_A, 0);
            (api.motor_set)(PORT_B, 0);
            (api.sound_stop)();
            clear_matrix(api);
            api.print(b"\r\n=== Exit ===\r\n");
            return 0;
        }
        if pressed & 0x01 != 0 {
            direction = -direction;
        }
        if pressed & 0x04 != 0 {
            speed = match speed {
                s if s <= 30 => 50,
                s if s <= 50 => 75,
                s if s <= 75 => 100,
                _ => 30,
            };
        }

        // ── Drive motors ──
        let spd = speed * direction;
        (api.motor_set)(PORT_A, spd);
        (api.motor_set)(PORT_B, -spd);

        // ── Proximity sound from ultrasonic ──
        if dist >= 0 && dist < 400 {
            let freq = 2000u32.saturating_sub(dist as u32 * 9 / 2).max(200);
            (api.sound_play)(freq);
        } else {
            (api.sound_stop)();
        }

        // ── LED matrix ──
        update_display(api, dist, r, g, b, i, speed, direction);

        // ── Status line every 500ms ──
        if cycle % 10 == 0 {
            api.print(b"E=");
            if dist >= 0 && dist < 2000 {
                print_i32(api, dist as i32);
                api.print(b"mm");
            } else {
                api.print(b"---");
            }
            api.print(b" F=R");
            print_u32(api, r as u32);
            api.print(b" G");
            print_u32(api, g as u32);
            api.print(b" B");
            print_u32(api, b as u32);
            api.print(b" I");
            print_u32(api, i as u32);
            api.print(b" A=");
            print_i32(api, pos_a);
            api.print(b"d B=");
            print_i32(api, pos_b);
            api.print(b"d spd=");
            print_i32(api, spd);
            api.print(b"%\r\n");
        }

        (api.delay_ms)(LOOP_MS);

        if cycle > 1200 {
            (api.motor_set)(PORT_A, 0);
            (api.motor_set)(PORT_B, 0);
            (api.sound_stop)();
            clear_matrix(api);
            api.print(b"\r\n=== Done (60s) ===\r\n");
            return 0;
        }
    }
}

// ── LED Display ──
// Row 0-1 (0-9):  ultrasonic distance bar
// Row 2 (10-14):  color sensor RGB indicator
// Row 3 (15-19):  motor direction
// Row 4 (20-24):  speed dots

fn update_display(api: &MonitorApi, dist: i16, r: u16, g: u16, b: u16, _i: u16, speed: i32, dir: i32) {
    // Distance bar: 10 pixels (rows 0-1)
    let bar = if dist >= 0 && dist < 2000 {
        10u32 - ((dist as u32).min(1999) * 10 / 2000)
    } else {
        0
    };
    for px in 0u32..10 {
        let bright = if px < bar { 80 - (px * 6) } else { 0 };
        (api.set_pixel)(px, bright);
    }

    // Color indicator: row 2.  Map RGBI→pixel brightness.
    // 5 pixels: [R] [RG] [G] [GB] [B]
    let scale = |v: u16| -> u32 { (v as u32).min(400) * 100 / 400 };
    (api.set_pixel)(10, scale(r));
    (api.set_pixel)(11, scale(r).max(scale(g)) / 2);
    (api.set_pixel)(12, scale(g));
    (api.set_pixel)(13, scale(g).max(scale(b)) / 2);
    (api.set_pixel)(14, scale(b));

    // Direction: row 3
    for px in 15u32..20 { (api.set_pixel)(px, 0); }
    if dir > 0 {
        (api.set_pixel)(17, 70);
        (api.set_pixel)(18, 40);
        (api.set_pixel)(19, 20);
    } else {
        (api.set_pixel)(17, 70);
        (api.set_pixel)(16, 40);
        (api.set_pixel)(15, 20);
    }

    // Speed dots: row 4
    let dots = (speed.unsigned_abs() / 25).min(4) as u32;
    for px in 0u32..5 {
        (api.set_pixel)(20 + px, if px < dots { 60 } else { 0 });
    }

    (api.update_leds)();
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
