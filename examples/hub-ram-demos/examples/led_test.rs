//! led_test — Test the hub status LED (glass light-pipe near USB port).
//!
//! Cycles through colors on the STATUS_TOP RGB LED using set_hub_led(),
//! with clear visual feedback for each color and smooth transitions.
//!
//! Sequence (2s per color):
//!   Red → Green → Blue → Cyan → Magenta → Yellow → White → Off
//!   then binary-delta blink (invert current), then fade breathing.
//!
//! Also exercises the 5×5 matrix with a matching color indicator.
//!
//! Requires: MonitorApi v10 (set_hub_led)
//!
//! Build:
//!   cd examples/hub-ram-demos
//!   cargo build --release --example led_test

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

#[derive(Copy, Clone)]
struct Color {
    r: u32,
    g: u32,
    b: u32,
    name: &'static [u8],
}

const COLORS: [Color; 8] = [
    Color { r: 100, g: 0, b: 0, name: b"RED" },
    Color { r: 0, g: 100, b: 0, name: b"GREEN" },
    Color { r: 0, g: 0, b: 100, name: b"BLUE" },
    Color { r: 0, g: 100, b: 100, name: b"CYAN" },
    Color { r: 100, g: 0, b: 100, name: b"MAGENTA" },
    Color { r: 100, g: 100, b: 0, name: b"YELLOW" },
    Color { r: 100, g: 100, b: 100, name: b"WHITE" },
    Color { r: 0, g: 0, b: 0, name: b"OFF" },
];

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 10 {
        api.print(b"ERROR: need API v10+, got v");
        print_u32(api, api.version);
        api.print(b"\r\n");
        return 1;
    }

    api.print(b"\r\n=== LED Test: Hub Status LED ===\r\n");
    api.print(b"Center button = exit\r\n\r\n");

    let mut prev_buttons: u8 = 0;

    // ── Phase 1: Solid colors (2s each) ──
    api.print(b"--- Phase 1: Solid colors ---\r\n");
    for color in &COLORS {
        api.print(b"  ");
        api.print(color.name);
        api.print(b" (");
        print_u32(api, color.r);
        api.print(b",");
        print_u32(api, color.g);
        api.print(b",");
        print_u32(api, color.b);
        api.print(b")\r\n");

        (api.set_hub_led)(color.r, color.g, color.b);

        // Matrix: show color index as lit pixels on row 0
        for px in 0..5u32 {
            (api.set_pixel)(px, 0);
        }
        let idx = unsafe { (color as *const Color).offset_from(COLORS.as_ptr()) } as u32;
        for px in 0..=idx.min(4) {
            (api.set_pixel)(px, 80);
        }
        (api.update_leds)();

        // Hold 2 seconds, check for exit
        for _ in 0..40 {
            let buttons = (api.read_buttons)();
            let pressed = buttons & !prev_buttons;
            prev_buttons = buttons;
            if pressed & 0x01 != 0 {
                (api.set_hub_led)(0, 0, 0);
                clear_matrix(api);
                api.print(b"Exit.\r\n");
                return 0;
            }
            (api.delay_ms)(50);
        }
    }

    // ── Phase 2: Binary-delta blink (rapid inversion) ──
    api.print(b"--- Phase 2: Binary-delta blink ---\r\n");
    for cycle in 0..30u32 {
        let base = &COLORS[(cycle as usize / 5) % 7]; // skip OFF
        let inverted = cycle % 2 == 1;
        if inverted {
            (api.set_hub_led)(
                100u32.saturating_sub(base.r),
                100u32.saturating_sub(base.g),
                100u32.saturating_sub(base.b),
            );
        } else {
            (api.set_hub_led)(base.r, base.g, base.b);
        }

        // Matrix blink
        let bright = if inverted { 20 } else { 90 };
        for px in 10..15u32 {
            (api.set_pixel)(px, bright);
        }
        (api.update_leds)();

        let buttons = (api.read_buttons)();
        let pressed = buttons & !prev_buttons;
        prev_buttons = buttons;
        if pressed & 0x01 != 0 {
            (api.set_hub_led)(0, 0, 0);
            clear_matrix(api);
            api.print(b"Exit.\r\n");
            return 0;
        }
        (api.delay_ms)(200);
    }

    // ── Phase 3: Breathing fade (white) ──
    api.print(b"--- Phase 3: Breathing fade ---\r\n");
    for cycle in 0..120u32 {
        // Triangle wave 0→100→0 over 60 cycles (3s per breath)
        let phase = cycle % 60;
        let brightness = if phase < 30 { phase * 100 / 30 } else { (60 - phase) * 100 / 30 };
        (api.set_hub_led)(brightness, brightness, brightness);

        // Matrix: bargraph on row 4
        for px in 20..25u32 {
            let thresh = (px - 20) * 20;
            (api.set_pixel)(px, if brightness > thresh { brightness } else { 0 });
        }
        (api.update_leds)();

        let buttons = (api.read_buttons)();
        let pressed = buttons & !prev_buttons;
        prev_buttons = buttons;
        if pressed & 0x01 != 0 {
            (api.set_hub_led)(0, 0, 0);
            clear_matrix(api);
            api.print(b"Exit.\r\n");
            return 0;
        }
        (api.delay_ms)(50);
    }

    // ── Phase 4: RGB rainbow sweep ──
    api.print(b"--- Phase 4: Rainbow sweep ---\r\n");
    for cycle in 0..180u32 {
        // Hue rotates through 0-360 in steps of 2
        let hue = (cycle * 2) % 360;
        let (r, g, b) = hue_to_rgb(hue);
        (api.set_hub_led)(r, g, b);

        // Matrix: rotating dot
        let dot = (cycle % 25) as u32;
        for px in 0..25u32 {
            (api.set_pixel)(px, if px == dot { 100 } else { 0 });
        }
        (api.update_leds)();

        let buttons = (api.read_buttons)();
        let pressed = buttons & !prev_buttons;
        prev_buttons = buttons;
        if pressed & 0x01 != 0 {
            (api.set_hub_led)(0, 0, 0);
            clear_matrix(api);
            api.print(b"Exit.\r\n");
            return 0;
        }
        (api.delay_ms)(50);
    }

    // Done
    (api.set_hub_led)(0, 0, 0);
    clear_matrix(api);
    api.print(b"\r\n=== LED test complete ===\r\n");
    0
}

/// Convert hue (0-359) to RGB (0-100).
fn hue_to_rgb(hue: u32) -> (u32, u32, u32) {
    let sector = hue / 60;
    let frac = (hue % 60) * 100 / 60; // 0-100 within sector
    match sector {
        0 => (100, frac, 0),             // red → yellow
        1 => (100 - frac, 100, 0),       // yellow → green
        2 => (0, 100, frac),             // green → cyan
        3 => (0, 100 - frac, 100),       // cyan → blue
        4 => (frac, 0, 100),             // blue → magenta
        _ => (100, 0, 100 - frac),       // magenta → red
    }
}

fn clear_matrix(api: &MonitorApi) {
    for i in 0..25u32 {
        (api.set_pixel)(i, 0);
    }
    (api.update_leds)();
}

fn print_u32(api: &MonitorApi, mut v: u32) {
    if v == 0 {
        api.print(b"0");
        return;
    }
    let mut buf = [0u8; 10];
    let mut i = 10usize;
    while v > 0 {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    (api.write_fn)(api.context, buf[i..].as_ptr(), (10 - i) as u32);
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
