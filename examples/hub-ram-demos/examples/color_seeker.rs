//! color-seeker — rotational color scanner demo.
//!
//! Motor A rotates through positions in steps while the color sensor
//! on port F reads and classifies what it sees at each stop.  The
//! motor_goto API (v7+) provides closed-loop positioning so the
//! scan is repeatable.
//!
//! Cycle:
//!   1. Step motor A through 12 positions (0°, 30°, 60°, ... 330°)
//!   2. At each stop, settle briefly then read + classify color
//!   3. Display color on LED matrix, fire-and-forget RTTY for each color
//!   4. RTTY runs in parallel — scan never waits for broadcast to finish
//!   5. After full rotation, print summary and return to 0°
//!   6. Repeat until center button pressed or MAX_SWEEPS done
//!
//! Requires: motor on port A, color sensor on port F, API v7+.
//!
//! Build:
//!   cd examples/hub-ram-demos
//!   cargo build --release --example color_seeker
//!   arm-none-eabi-objcopy -O binary \
//!     target/thumbv7em-none-eabihf/release/examples/color_seeker \
//!     target/color-seeker.bin

#![no_std]
#![no_main]

use spike_hub_api::{MonitorApi, BTN_CENTER};

const PORT_A: u32 = 0;

// Software colour IDs
const COL_NONE: i8 = -1;
const COL_BLACK: i8 = 0;
const COL_RED: i8 = 9;
const COL_GREEN: i8 = 5;
const COL_YELLOW: i8 = 7;
const COL_BLUE: i8 = 3;
const COL_WHITE: i8 = 10;

// Scan parameters
const STEP_DEG: i32 = 30;           // degrees per step
const NUM_STEPS: usize = 12;        // 360 / 30 = 12 positions
const SETTLE_MS: u32 = 200;         // settle time after motor_goto
const MAX_SWEEPS: u32 = 3;          // stop after this many full rotations

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };
    if api.version < 7 {
        api.print(b"ERR: need API v7+ (motor_goto)\r\n");
        return 1;
    }

    api.print(b"=== COLOR SCANNER ===\r\n");
    api.print(b"Motor A rotates, sensor F reads color at each stop.\r\n");
    api.print(b"Center btn = stop\r\n\r\n");

    // Debounce: wait for center button release
    for _ in 0..50 {
        if (api.read_buttons)() & BTN_CENTER == 0 { break; }
        (api.delay_ms)(50);
    }

    // Set sensor to RGBI mode 5
    (api.sensor_mode)(5);
    // Wait for first data
    for _ in 0..100 {
        (api.delay_ms)(20);
        let mut buf = [0u8; 8];
        if (api.sensor_read)(buf.as_mut_ptr(), 8) >= 8 { break; }
    }

    // Read starting motor position
    let start_pos = (api.motor_position)();
    api.print(b"Start pos: ");
    print_i32(api, start_pos);
    api.print(b"deg\r\n");

    // RTTY announce — fire and forget, never block
    rtty_fire(api, b"SCAN ON");

    // ── Color map: stores detected color at each step ──
    let mut color_map: [i8; NUM_STEPS] = [COL_NONE; NUM_STEPS];
    let mut total_colors: u32 = 0;

    // ════════════════════════════════════════════════
    // SWEEP LOOP
    // ════════════════════════════════════════════════
    for sweep in 0..MAX_SWEEPS {
        api.print(b"\r\n--- Sweep ");
        print_u32(api, sweep + 1);
        api.print(b" ---\r\n");

        for step in 0..NUM_STEPS {
            // Exit check
            if (api.read_buttons)() & BTN_CENTER != 0 {
                api.print(b"\r\n[stopped by button]\r\n");
                goto_cleanup(api, start_pos, total_colors);
                return 0;
            }

            // Target position for this step
            let target = start_pos + (sweep as i32 * 360) + (step as i32 * STEP_DEG);

            // Move motor to position
            let err = (api.motor_goto)(PORT_A, target);

            // Settle time for sensor reading stability
            (api.delay_ms)(SETTLE_MS);

            // Read + classify color (average 3 reads for stability)
            let color = read_stable_color(api);
            color_map[step] = color;

            // LED: light up row pixel for this step (pixels 0-24 mapped)
            let px = step_to_pixel(step);
            let bri = if color != COL_NONE && color != COL_BLACK { 80u32 } else { 5 };
            (api.set_pixel)(px, bri);
            (api.update_leds)();

            // Telemetry
            api.print(b"  ");
            print_u32(api, (step as i32 * STEP_DEG) as u32);
            api.print(b"deg: ");
            print_color_name(api, color);
            let (r, g, b) = read_rgb(api);
            api.print(b" R"); print_u16(api, r);
            api.print(b" G"); print_u16(api, g);
            api.print(b" B"); print_u16(api, b);
            api.print(b" err="); print_i32(api, err);
            api.print(b"\r\n");

            // Count non-trivial colors
            if color != COL_NONE && color != COL_BLACK {
                total_colors += 1;
            }

            // Fire-and-forget RTTY for each color detected (non-blocking)
            // If RTTY is already busy, the new message is silently dropped
            rtty_step_announce(api, step, color);

            // Quick tone beep — non-blocking: start tone, it auto-stops
            // at next step or gets overridden
            if color != COL_NONE && color != COL_BLACK {
                (api.sound_play)(color_tone(color));
            } else {
                (api.sound_stop)();
            }
        }

        // End of sweep summary
        api.print(b"\r\nSweep ");
        print_u32(api, sweep + 1);
        api.print(b" map: ");
        for step in 0..NUM_STEPS {
            print_color_short(api, color_map[step]);
            api.print(b" ");
        }
        api.print(b"\r\n");

        // RTTY sweep summary — fire and forget
        rtty_fire(api, rtty_sweep_msg(sweep + 1, total_colors));
    }

    // Return to start
    goto_cleanup(api, start_pos, total_colors);
    0
}

fn goto_cleanup(api: &MonitorApi, start_pos: i32, total_colors: u32) {
    api.print(b"\r\nReturning to start...\r\n");
    (api.motor_goto)(PORT_A, start_pos);
    (api.motor_brake)(PORT_A);
    (api.sound_stop)();

    // Clear LEDs
    for i in 0u32..25 { (api.set_pixel)(i, 0); }
    (api.update_leds)();

    // Fire SCAN OFF — don't block even if previous RTTY still going
    rtty_fire(api, b"SCAN OFF");
    // Brief pause just to let the return-to-start motor_goto finish settling
    (api.delay_ms)(500);

    api.print(b"\r\n=== DONE. Colors detected: ");
    print_u32(api, total_colors);
    api.print(b" ===\r\n");
}

// ── Helpers ───────────────────────────────────────────────

/// Read color sensor 3 times with 50ms gaps, return most common.
fn read_stable_color(api: &MonitorApi) -> i8 {
    let mut colors = [COL_NONE; 3];
    for i in 0..3 {
        let (c, _, _, _) = read_rgbi_color(api);
        colors[i] = c;
        if i < 2 { (api.delay_ms)(50); }
    }
    // Majority vote
    if colors[0] == colors[1] || colors[0] == colors[2] { colors[0] }
    else if colors[1] == colors[2] { colors[1] }
    else { colors[0] } // no consensus, use first
}

fn read_rgbi_color(api: &MonitorApi) -> (i8, u16, u16, u16) {
    let mut buf = [0u8; 8];
    let n = (api.sensor_read)(buf.as_mut_ptr(), 8);
    if n < 8 { return (COL_NONE, 0, 0, 0); }
    let r = u16::from_le_bytes([buf[0], buf[1]]);
    let g = u16::from_le_bytes([buf[2], buf[3]]);
    let b = u16::from_le_bytes([buf[4], buf[5]]);
    (classify_rgb(r, g, b), r, g, b)
}

fn read_rgb(api: &MonitorApi) -> (u16, u16, u16) {
    let mut buf = [0u8; 8];
    let n = (api.sensor_read)(buf.as_mut_ptr(), 8);
    if n < 8 { return (0, 0, 0); }
    let r = u16::from_le_bytes([buf[0], buf[1]]);
    let g = u16::from_le_bytes([buf[2], buf[3]]);
    let b = u16::from_le_bytes([buf[4], buf[5]]);
    (r, g, b)
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
        0..=30 => COL_RED,
        31..=70 => COL_YELLOW,
        71..=170 => COL_GREEN,
        171..=260 => COL_BLUE,
        _ => COL_RED,
    }
}

/// Map step index (0-11) to a pixel on the 5x5 LED matrix.
/// Steps go around the edge like a clock face.
fn step_to_pixel(step: usize) -> u32 {
    // Top row L-R, right col top-down, bottom row R-L, left col bottom-up
    const MAP: [u32; 12] = [2, 3, 4, 9, 14, 19, 24, 23, 22, 21, 20, 15];
    MAP[step % 12]
}

fn color_tone(c: i8) -> u32 {
    match c {
        COL_RED => 523,
        COL_GREEN => 659,
        COL_BLUE => 784,
        COL_YELLOW => 880,
        COL_WHITE => 1047,
        _ => 440,
    }
}

/// Fire-and-forget RTTY: if channel is free, start broadcast.
/// If busy, silently drop — never block the scan.
fn rtty_fire(api: &MonitorApi, msg: &[u8]) {
    if (api.rtty_busy)() == 0 {
        (api.rtty_say)(msg.as_ptr(), msg.len() as u32);
    }
}

/// Announce a step's color over RTTY (fire-and-forget).
/// Format: "30R" or "120G" or "270K"
fn rtty_step_announce(api: &MonitorApi, step: usize, color: i8) {
    static mut BUF: [u8; 8] = [0; 8];
    unsafe {
        let deg = (step as u32) * (STEP_DEG as u32);
        let mut i = 0usize;
        i += fmt_u32(&mut BUF[i..], deg);
        BUF[i] = match color {
            COL_RED => b'R', COL_GREEN => b'G', COL_BLUE => b'B',
            COL_YELLOW => b'Y', COL_WHITE => b'W', COL_BLACK => b'K',
            _ => b'?',
        };
        i += 1;
        rtty_fire(api, &BUF[..i]);
    }
}

fn rtty_sweep_msg(sweep: u32, count: u32) -> &'static [u8] {
    static mut BUF: [u8; 32] = [0; 32];
    unsafe {
        let mut i = 0usize;
        BUF[i] = b'S'; i += 1;
        i += fmt_u32(&mut BUF[i..], sweep);
        BUF[i] = b' '; i += 1;
        BUF[i] = b'C'; i += 1;
        i += fmt_u32(&mut BUF[i..], count);
        &BUF[..i]
    }
}

fn print_color_name(api: &MonitorApi, c: i8) {
    match c {
        COL_RED => api.print(b"RED"),
        COL_GREEN => api.print(b"GRN"),
        COL_YELLOW => api.print(b"YEL"),
        COL_BLUE => api.print(b"BLU"),
        COL_BLACK => api.print(b"BLK"),
        COL_WHITE => api.print(b"WHT"),
        _ => api.print(b"---"),
    }
}

fn print_color_short(api: &MonitorApi, c: i8) {
    match c {
        COL_RED => api.print(b"R"),
        COL_GREEN => api.print(b"G"),
        COL_YELLOW => api.print(b"Y"),
        COL_BLUE => api.print(b"B"),
        COL_BLACK => api.print(b"K"),
        COL_WHITE => api.print(b"W"),
        _ => api.print(b"."),
    }
}

fn fmt_u32(buf: &mut [u8], mut v: u32) -> usize {
    if v == 0 { buf[0] = b'0'; return 1; }
    let mut tmp = [0u8; 10];
    let mut n = 0usize;
    while v > 0 { tmp[n] = b'0' + (v % 10) as u8; v /= 10; n += 1; }
    for j in 0..n { buf[j] = tmp[n - 1 - j]; }
    n
}

fn print_u32(api: &MonitorApi, mut v: u32) {
    if v == 0 { api.print(b"0"); return; }
    let mut buf = [0u8; 10];
    let mut i = 10usize;
    while v > 0 { i -= 1; buf[i] = b'0' + (v % 10) as u8; v /= 10; }
    (api.write_fn)(api.context, buf[i..].as_ptr(), (10 - i) as u32);
}

fn print_u16(api: &MonitorApi, v: u16) {
    print_u32(api, v as u32);
}

fn print_i32(api: &MonitorApi, v: i32) {
    if v < 0 {
        api.print(b"-");
        print_u32(api, (-v) as u32);
    } else {
        print_u32(api, v as u32);
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
