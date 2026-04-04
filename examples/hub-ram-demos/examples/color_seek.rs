//! Color-seek — fast RGBI scan + dance on motor A.
//!
//! Stays in mode 5 (RGBI) the entire time — **zero mode switches**.
//! Classifies color from RGB in software (same approach as Pybricks).
//! This eliminates the 500-1000 ms mode-switch penalty per seek cycle.
//!
//! Motor A scans slowly, detects any non-black/non-none colour,
//! then dances: small oscillations around the found colour position,
//! increasing in amplitude with each discovery.
//!
//! Sensor: port F (color sensor, default).
//! Motor: port A only.
//!
//! Build:  cargo build --release --example color_seek
//! Objcopy: rust-objcopy -O binary target/thumbv7em-none-eabihf/release/examples/color_seek target/color-seek.bin

#![no_std]
#![no_main]

use spike_hub_api::{MonitorApi, BTN_CENTER};

const PORT_A: u32 = 0;

// Software colour IDs (derived from RGBI)
const COL_NONE: i8 = -1;
const COL_BLACK: i8 = 0;
const COL_RED: i8 = 9;
const COL_GREEN: i8 = 5;
const COL_YELLOW: i8 = 7;
const COL_BLUE: i8 = 3;
const COL_WHITE: i8 = 10;

const CRAWL: i32 = 30;      // slow scan speed
const DANCE_SPEED: i32 = 60; // dance oscillation speed
/// Sensor poll interval — 20 ms ≈ 50 Hz.
const POLL_MS: u32 = 20;

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };
    if api.version < 4 {
        api.print(b"ERR: need API v4+\r\n");
        return 1;
    }

    api.print(b"=== Color Seek + Dance (Motor A) ===\r\n");
    api.print(b"Scan=30  Dance=60  RGBI mode 5\r\n");
    api.print(b"Center btn = exit\r\n\r\n");

    // Wait for center button release
    for _ in 0..50 {
        if (api.read_buttons)() & BTN_CENTER == 0 { break; }
        (api.delay_ms)(50);
    }

    // Start in mode 5 (RGBI) — stay here for the entire demo.
    (api.sensor_mode)(5);
    let mut settled = false;
    for _ in 0..50 {
        (api.delay_ms)(20);
        let mut buf = [0u8; 8];
        let n = (api.sensor_read)(buf.as_mut_ptr(), 8);
        if n >= 8 { settled = true; break; }
    }
    if !settled {
        api.print(b"WARN: sensor not streaming RGBI\r\n");
    }

    // ── Phase 1: Scan forward, find any colour ──
    api.print(b"--- Scanning fwd ---\r\n");

    let mut found_colors: [ColorHit; 8] = [ColorHit::EMPTY; 8];
    let mut num_found: usize = 0;
    let mut prev: i8 = COL_NONE;

    (api.motor_set)(PORT_A, CRAWL);

    for tick in 0u32..600 {  // 600 × 20ms = 12s
        if (api.read_buttons)() & BTN_CENTER != 0 {
            (api.motor_brake)(PORT_A);
            api.print(b"Aborted\r\n");
            return 99;
        }

        let (c, r, g, b) = read_rgbi_color(api);

        // Log every 10th reading
        if tick % 10 == 0 {
            print_u32(api, tick);
            api.print(b": ");
            print_color_name(api, c);
            api.print(b" R"); print_u32(api, r as u32);
            api.print(b" G"); print_u32(api, g as u32);
            api.print(b" B"); print_u32(api, b as u32);
            api.print(b"\r\n");
        }

        // Detect colour transitions
        if c != prev && c != COL_NONE && c != COL_BLACK {
            api.print(b"  >> ");
            print_color_name(api, c);
            api.print(b" at t="); print_u32(api, tick);
            api.print(b"\r\n");

            if num_found < 8 {
                found_colors[num_found] = ColorHit { id: c, tick };
                num_found += 1;
            }
        }
        prev = c;

        // Stop after finding 3 colours (enough to dance with)
        if num_found >= 3 { break; }
        (api.delay_ms)(POLL_MS);
    }

    (api.motor_brake)(PORT_A);
    (api.delay_ms)(300);

    if num_found == 0 {
        api.print(b"\r\nNo colours found!\r\n");
        return 2;
    }

    // Print discoveries
    api.print(b"\r\nFound ");
    print_u32(api, num_found as u32);
    api.print(b" colour(s):\r\n");
    for i in 0..num_found {
        api.print(b"  ");
        print_color_name(api, found_colors[i].id);
        api.print(b" (t="); print_u32(api, found_colors[i].tick);
        api.print(b")\r\n");
    }

    // ── Phase 2: Dance! ──
    // Reverse back, find each colour again, do a little dance on it.
    api.print(b"\r\n--- Dance! ---\r\n");

    let mut dir: i32 = -1;  // start by reversing
    let mut dance_amp: u32 = 150; // dance duration ms, grows each time

    for cycle in 0u32..12 {
        if (api.read_buttons)() & BTN_CENTER != 0 { break; }

        // Pick target colour (cycle through found ones)
        let tgt = found_colors[(cycle as usize) % num_found].id;
        api.print(b"\r\nSeek ");
        print_color_name(api, tgt);

        (api.motor_set)(PORT_A, CRAWL * dir);

        // Search for the target colour
        let mut hit = false;
        for t in 0u32..500 {  // 500 × 20ms = 10s timeout
            if (api.read_buttons)() & BTN_CENTER != 0 { break; }
            let (c, _, _, _) = read_rgbi_color(api);
            if c == tgt {
                api.print(b" FOUND t="); print_u32(api, t);
                api.print(b"\r\n");
                hit = true;
                break;
            }
            (api.delay_ms)(POLL_MS);
        }

        (api.motor_brake)(PORT_A);

        if hit {
            // Dance: oscillate around the found position
            api.print(b"  Dance! amp=");
            print_u32(api, dance_amp);
            api.print(b"ms\r\n");

            // Play a celebration tone
            (api.sound_play)(440 + cycle * 100);

            for wiggle in 0u8..6 {
                let spd = if wiggle % 2 == 0 { DANCE_SPEED } else { -DANCE_SPEED };
                (api.motor_set)(PORT_A, spd);
                (api.delay_ms)(dance_amp);
            }

            (api.sound_stop)();
            (api.motor_brake)(PORT_A);
            (api.delay_ms)(300);

            // Increase dance amplitude each time
            dance_amp = (dance_amp + 50).min(500);
        } else {
            api.print(b" Timeout\r\n");
        }

        dir = -dir;
    }

    (api.motor_brake)(PORT_A);
    api.print(b"\r\n=== Done ===\r\n");
    0
}

// ── Data types ──

#[derive(Clone, Copy)]
struct ColorHit {
    id: i8,
    tick: u32,
}

impl ColorHit {
    const EMPTY: Self = Self { id: COL_NONE, tick: 0 };
}

// ── RGBI → colour classification ──

fn read_rgbi_color(api: &MonitorApi) -> (i8, u16, u16, u16) {
    let mut buf = [0u8; 8];
    let n = (api.sensor_read)(buf.as_mut_ptr(), 8);
    if n < 8 {
        return (COL_NONE, 0, 0, 0);
    }
    let r = u16::from_le_bytes([buf[0], buf[1]]);
    let g = u16::from_le_bytes([buf[2], buf[3]]);
    let b = u16::from_le_bytes([buf[4], buf[5]]);
    let c = classify_rgb(r, g, b);
    (c, r, g, b)
}

fn classify_rgb(r: u16, g: u16, b: u16) -> i8 {
    let r8 = if r >= 1024 { 255u16 } else { r >> 2 };
    let g8 = if g >= 1024 { 255u16 } else { g >> 2 };
    let b8 = if b >= 1024 { 255u16 } else { b >> 2 };

    let max = r8.max(g8).max(b8);
    let min = r8.min(g8).min(b8);
    let chroma = max - min;

    let val = (101 * max) / 256;
    if val < 4 { return COL_BLACK; }

    let sat = if max > 0 { 100 * chroma / max } else { 0 };
    if sat < 25 {
        return if val > 30 { COL_WHITE } else { COL_BLACK };
    }

    let hue = if chroma == 0 {
        0i16
    } else if max == r8 {
        let h = 60 * (g8 as i16 - b8 as i16) / chroma as i16;
        if h < 0 { h + 360 } else { h }
    } else if max == g8 {
        60 * (b8 as i16 - r8 as i16) / chroma as i16 + 120
    } else {
        60 * (r8 as i16 - g8 as i16) / chroma as i16 + 240
    };

    match hue {
        0..=30   => COL_RED,
        31..=70  => COL_YELLOW,
        71..=170 => COL_GREEN,
        171..=260 => COL_BLUE,
        _ => COL_RED,
    }
}

// ── Printing helpers ──

fn print_color_name(api: &MonitorApi, c: i8) {
    match c {
        COL_RED    => api.print(b"RED"),
        COL_GREEN  => api.print(b"GRN"),
        COL_YELLOW => api.print(b"YEL"),
        COL_BLUE   => api.print(b"BLU"),
        COL_BLACK  => api.print(b"BLK"),
        COL_WHITE  => api.print(b"WHT"),
        _          => api.print(b"---"),
    }
}

fn print_u32(api: &MonitorApi, mut v: u32) {
    if v == 0 { api.print(b"0"); return; }
    let mut buf = [0u8; 10];
    let mut i = 10usize;
    while v > 0 { i -= 1; buf[i] = b'0' + (v % 10) as u8; v /= 10; }
    (api.write_fn)(api.context, buf[i..].as_ptr(), (10 - i) as u32);
}

fn print_i32(api: &MonitorApi, v: i32) {
    if v < 0 { api.print(b"-"); print_u32(api, (-(v as i64)) as u32); }
    else { print_u32(api, v as u32); }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
