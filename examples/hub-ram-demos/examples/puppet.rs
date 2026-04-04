//! puppet — always-running differential color-reactive motor demo.
//!
//! Both motors A+B turn continuously at a slow baseline.
//! The color sensor (port F, RGBI mode 5) measures differential
//! change every loop iteration.  The MORE the color changes,
//! the MORE extra motor movement is layered on top of the baseline.
//!
//! Pipeline:  sensor RGBI → delta from prev → IIR smooth delta →
//!            scale to boost → baseline ± boost → IIR smooth → PWM
//!
//! Motor mapping (gives interesting asymmetric behavior):
//!   A = baseline + boost * red_fraction   (red-dominant change → A reacts)
//!   B = baseline + boost * blue_fraction  (blue-dominant change → B reacts)
//!   Both get a minimum boost from total delta so they always respond.
//!
//! LED matrix: bar graph showing current delta intensity (0-25 pixels).
//! Sound: pitch proportional to delta (silent when calm, rising with activity).
//!
//! Auto-terminates after 60 seconds.  Motors braked on exit.
//! Center button = early stop.
//!
//! Requires: motors on ports A+B, color sensor on port F, API v8+.
//!
//! Build:
//!   cd examples/hub-ram-demos
//!   cargo build --release --example puppet
//!   arm-none-eabi-objcopy -O binary \
//!     target/thumbv7em-none-eabihf/release/examples/puppet puppet.bin

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

const PORT_A: u32 = 0;
const PORT_B: u32 = 1;

// ── Timing ──
const LOOP_MS: u32 = 50;           // 20 Hz control loop

// ── Motor tuning ──
const BASELINE: i32 = 28;          // always-on slow forward PWM
const MAX_BOOST: i32 = 62;         // max extra speed from color delta
// Total possible = BASELINE + MAX_BOOST = 90

// ── DSP: IIR low-pass (fixed-point, alpha = 64/256 = 0.25) ──
const ALPHA: i32 = 64;
const ONE_MINUS_ALPHA: i32 = 192;

fn iir_smooth(prev: i32, target: i32) -> i32 {
    (ALPHA * target + ONE_MINUS_ALPHA * prev) >> 8
}

/// Absolute difference for u16, returned as u32 to avoid overflow in sums.
fn abs_diff(a: u16, b: u16) -> u32 {
    if a >= b { (a - b) as u32 } else { (b - a) as u32 }
}

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };
    if api.version < 8 {
        print(api, b"ERR: need API v8+ (port_read)\r\n");
        return 1;
    }

    print(api, b"=== PUPPET v2 ===\r\n");
    print(api, b"Motors always on + differential color.\r\n");
    print(api, b"Kill via right-button or shell.\r\n\r\n");

    // Init sensor to RGBI mode 5
    (api.sensor_mode)(5);
    (api.delay_ms)(500);

    // State
    let mut speed_a: i32 = 0;
    let mut speed_b: i32 = 0;
    let mut smooth_delta: i32 = 0;  // IIR-smoothed total delta
    let mut tick: u32 = 0;
    let mut dir_a: i32 = 1;  // +1 or -1, random per motor
    let mut dir_b: i32 = 1;

    // Previous RGBI reading for differential
    let (mut prev_r, mut prev_g, mut prev_b, mut prev_i) = read_rgbi(api);

    // Startup chirp — ascending
    (api.sound_play)(440);
    (api.delay_ms)(80);
    (api.sound_play)(880);
    (api.delay_ms)(80);
    (api.sound_play)(1320);
    (api.delay_ms)(80);
    (api.sound_stop)();

    // ════════════════════════════════════════════════
    // MAIN CONTROL LOOP — 20 Hz  (runs until firmware kills us)
    // ════════════════════════════════════════════════
    loop {

        // ── Sensor read ──
        let (r, g, b, i) = read_rgbi(api);

        // ── Differential: per-channel absolute deltas ──
        let dr = abs_diff(r, prev_r);
        let dg = abs_diff(g, prev_g);
        let db = abs_diff(b, prev_b);
        let di = abs_diff(i, prev_i);
        let raw_delta = (dr + dg + db + di) as i32; // 0..~4000 typically

        prev_r = r;
        prev_g = g;
        prev_b = b;
        prev_i = i;

        // ── IIR smooth the delta (avoids single-sample spikes) ──
        smooth_delta = iir_smooth(smooth_delta, raw_delta);

        // ── Map smoothed delta to boost (0..MAX_BOOST) ──
        // Scale: delta of ~400 = full boost.  Clamp at MAX_BOOST.
        let boost = core::cmp::min(smooth_delta * MAX_BOOST / 400, MAX_BOOST);

        // ── Per-channel fractions for asymmetric motor drive ──
        // Which channel changed most?  Drive motors differentially.
        let ch_total = dr + dg + db + 1; // +1 avoid div-by-zero
        let r_frac = (dr * 100 / ch_total) as i32; // 0-100
        let b_frac = (db * 100 / ch_total) as i32;

        // A gets more boost from red-dominant changes,
        // B gets more boost from blue-dominant changes.
        // Both always get at least half the boost (shared component).
        let boost_a = boost / 2 + boost * r_frac / 200;
        let boost_b = boost / 2 + boost * b_frac / 200;

        // ── Target speeds: baseline + boost ──
        // Pseudo-random wobble: two cheap LCG-style sequences with
        // different periods so A and B wander independently.
        let rng_a = ((tick.wrapping_mul(1103515245).wrapping_add(12345)) >> 16) % 17;
        let rng_b = ((tick.wrapping_mul(214013).wrapping_add(2531011)) >> 16) % 17;
        let wobble_a = rng_a as i32 - 8; // -8..+8
        let wobble_b = rng_b as i32 - 8; // -8..+8

        // Random direction flip ~every 2s (40 ticks), independent per motor
        if tick % 40 == 0 {
            dir_a = if (tick.wrapping_mul(1103515245).wrapping_add(12345) >> 24) & 1 == 0 { 1 } else { -1 };
        }
        if tick % 40 == 20 {
            dir_b = if (tick.wrapping_mul(214013).wrapping_add(2531011) >> 24) & 1 == 0 { 1 } else { -1 };
        }

        let target_a = dir_a * (BASELINE + boost_a + wobble_a);
        let target_b = dir_b * (BASELINE + boost_b + wobble_b);

        // ── IIR smooth motor commands ──
        speed_a = iir_smooth(speed_a, target_a);
        speed_b = iir_smooth(speed_b, target_b);

        // ── Apply ──
        (api.motor_set)(PORT_A, speed_a);
        (api.motor_set)(PORT_B, speed_b);

        // ── LED: bar graph of delta intensity ──
        show_delta(api, smooth_delta, tick);

        // ── Sound: pitch tracks delta ──
        delta_sound(api, smooth_delta, tick);

        // ── Telemetry: print delta every ~1s ──
        if tick % 20 == 0 {
            print_delta(api, smooth_delta, speed_a, speed_b);
        }

        (api.delay_ms)(LOOP_MS);
        tick = tick.wrapping_add(1);
    }
}

// ════════════════════════════════════════════════════════
// COLOR SENSOR — raw RGBI read
// ════════════════════════════════════════════════════════

fn read_rgbi(api: &MonitorApi) -> (u16, u16, u16, u16) {
    let mut buf = [0u8; 8];
    let n = (api.sensor_read)(buf.as_mut_ptr(), 8);
    if n < 8 { return (0, 0, 0, 0); }
    let r = u16::from_le_bytes([buf[0], buf[1]]);
    let g = u16::from_le_bytes([buf[2], buf[3]]);
    let b = u16::from_le_bytes([buf[4], buf[5]]);
    let i = u16::from_le_bytes([buf[6], buf[7]]);
    (r, g, b, i)
}

// ════════════════════════════════════════════════════════
// LED MATRIX — delta bar graph
// ════════════════════════════════════════════════════════

fn show_delta(api: &MonitorApi, delta: i32, tick: u32) {
    if tick % 4 != 0 { return; } // 5 Hz update

    // Map delta 0..500 → 0..25 lit pixels (bottom-up fill)
    let lit = core::cmp::min((delta * 25 / 500) as u32, 25);

    for idx in 0..25u32 {
        // Fill from bottom-left: row 4 first, then 3, 2, 1, 0
        // Pixel order: row*5+col.  We fill bottom-up, left-to-right.
        let row = 4 - idx / 5;
        let col = idx % 5;
        let pixel = row * 5 + col;
        if idx < lit {
            // Brightness ramps up for higher bars
            let bri = 20 + (idx as u32) * 3; // 20..95
            (api.set_pixel)(pixel, bri);
        } else {
            (api.set_pixel)(pixel, 0);
        }
    }
    (api.update_leds)();
}

// ════════════════════════════════════════════════════════
// SOUND — pitch tracks delta
// ════════════════════════════════════════════════════════

fn delta_sound(api: &MonitorApi, delta: i32, tick: u32) {
    if tick % 10 != 0 { return; } // 2 Hz update

    if delta < 20 {
        (api.sound_stop)(); // quiet when calm
    } else {
        // Map delta 20..500 → 200..1200 Hz
        let freq = 200 + (core::cmp::min(delta, 500) as u32) * 2;
        (api.sound_play)(freq);
    }
}

// ════════════════════════════════════════════════════════
// TELEMETRY
// ════════════════════════════════════════════════════════

fn print_delta(api: &MonitorApi, delta: i32, sa: i32, sb: i32) {
    // Print compact: "d=123 A=45 B=32\r\n"
    print(api, b"d=");
    print_num(api, delta);
    print(api, b" A=");
    print_num(api, sa);
    print(api, b" B=");
    print_num(api, sb);
    print(api, b"\r\n");
}

fn print_num(api: &MonitorApi, mut val: i32) {
    if val < 0 {
        print(api, b"-");
        val = -val;
    }
    let mut buf = [0u8; 6];
    let mut pos = buf.len();
    if val == 0 {
        pos -= 1;
        buf[pos] = b'0';
    } else {
        while val > 0 && pos > 0 {
            pos -= 1;
            buf[pos] = b'0' + (val % 10) as u8;
            val /= 10;
        }
    }
    (api.write_fn)(api.context, buf[pos..].as_ptr(), (buf.len() - pos) as u32);
}

fn print(api: &MonitorApi, msg: &[u8]) {
    (api.write_fn)(api.context, msg.as_ptr(), msg.len() as u32);
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
