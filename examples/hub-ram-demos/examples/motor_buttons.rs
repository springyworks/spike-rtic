//! Interactive button-controlled dual motors (Port A + Port B).
//!
//! Center button: toggle motors on/off
//! Left button:   reverse direction
//! Right button:  cycle speed (slow → medium → fast)
//!
//! The LED matrix shows the current state: speed bars and direction
//! arrows.  Release all buttons and press center to exit.
//!
//! Build & upload:
//!   cd hub-ram-demos && cargo build --release
//!   arm-none-eabi-objcopy -O binary \
//!       target/thumbv7em-none-eabihf/release/motor-buttons motor-buttons.bin

#![no_std]
#![no_main]

use spike_hub_api::{MonitorApi, BTN_CENTER, BTN_LEFT, BTN_RIGHT};

const PORT_A: u32 = 0;
const PORT_B: u32 = 1;

const SPEEDS: [i32; 3] = [30, 60, 90];

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 2 {
        api.print(b"ERR: need API v2+ (motor support)\r\n");
        return 1;
    }

    api.print(b"=== Motor Control (A+B) ===\r\n");
    api.print(b"  Center: toggle on/off\r\n");
    api.print(b"  Left:   reverse direction\r\n");
    api.print(b"  Right:  cycle speed\r\n");
    api.print(b"  Hold center 2s: exit\r\n\r\n");

    let mut running = false;
    let mut forward = true;
    let mut speed_idx: usize = 0;
    let mut center_hold: u32 = 0;

    // Debounce state: track previous button state to detect edges
    let mut prev_btns: u8 = 0;

    show_state(api, running, forward, speed_idx);

    loop {
        let btns = (api.read_buttons)();
        let pressed = btns & !prev_btns; // rising edges
        prev_btns = btns;

        // Hold center for ~2 s (40 × 50 ms) to exit
        if btns & BTN_CENTER != 0 {
            center_hold += 1;
            if center_hold >= 40 {
                (api.motor_set)(PORT_A, 0);
                (api.motor_set)(PORT_B, 0);
                clear_matrix(api);
                api.print(b"Exiting.\r\n");
                return 0;
            }
        } else {
            center_hold = 0;
        }

        // Toggle on/off on center click (rising edge)
        if pressed & BTN_CENTER != 0 {
            running = !running;
            if running {
                api.print(b"Motors ON\r\n");
            } else {
                api.print(b"Motors OFF\r\n");
            }
        }

        // Reverse on left click
        if pressed & BTN_LEFT != 0 {
            forward = !forward;
            if forward {
                api.print(b"Direction: FORWARD\r\n");
            } else {
                api.print(b"Direction: REVERSE\r\n");
            }
        }

        // Cycle speed on right click
        if pressed & BTN_RIGHT != 0 {
            speed_idx = (speed_idx + 1) % SPEEDS.len();
            api.print(b"Speed: ");
            let label = match speed_idx {
                0 => b"SLOW (30%)\r\n" as &[u8],
                1 => b"MEDIUM (60%)\r\n",
                _ => b"FAST (90%)\r\n",
            };
            api.print(label);
        }

        // Apply motor state
        if running {
            let spd = if forward {
                SPEEDS[speed_idx]
            } else {
                -SPEEDS[speed_idx]
            };
            (api.motor_set)(PORT_A, spd);
            (api.motor_set)(PORT_B, spd);
        } else {
            (api.motor_set)(PORT_A, 0);
            (api.motor_set)(PORT_B, 0);
        }

        show_state(api, running, forward, speed_idx);

        (api.delay_ms)(50);
    }
}

/// Display current state on LED matrix.
fn show_state(api: &MonitorApi, running: bool, forward: bool, speed_idx: usize) {
    // Clear
    for i in 0..25u32 {
        (api.set_pixel)(i, 0);
    }

    if !running {
        // Paused: show two vertical bars (pause icon)
        // Col 1: pixels 1, 6, 11, 16, 21
        // Col 3: pixels 3, 8, 13, 18, 23
        for &p in &[1u32, 6, 11, 16, 21, 3, 8, 13, 18, 23] {
            (api.set_pixel)(p, 40);
        }
    } else {
        // Direction arrow at top
        if forward {
            // Up arrow top 2 rows
            (api.set_pixel)(2, 70);
            (api.set_pixel)(6, 50);
            (api.set_pixel)(8, 50);
        } else {
            // Down arrow
            (api.set_pixel)(22, 70);
            (api.set_pixel)(16, 50);
            (api.set_pixel)(18, 50);
        }

        // Speed bars (bottom rows): more bars = higher speed
        let bar_pixels: &[&[u32]] = &[
            &[20, 21, 22, 23, 24],      // row 4 (bottom)
            &[15, 16, 17, 18, 19],      // row 3
            &[10, 11, 12, 13, 14],      // row 2
        ];
        let bars = speed_idx + 1; // 1, 2, or 3 bars
        for row in 0..bars {
            if let Some(pixels) = bar_pixels.get(row) {
                for &p in *pixels {
                    (api.set_pixel)(p, 30 + (row as u32 * 20));
                }
            }
        }
    }

    (api.update_leds)();
}

fn clear_matrix(api: &MonitorApi) {
    for i in 0..25u32 {
        (api.set_pixel)(i, 0);
    }
    (api.update_leds)();
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
