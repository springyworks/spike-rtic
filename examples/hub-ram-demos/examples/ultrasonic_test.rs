//! Ultrasonic sensor distance demo (port E).
//!
//! Reads distance every 100ms, displays on LED matrix as a bar graph,
//! and prints distance in mm.  Press center button to exit.
//!
//! Build & upload:
//!   cd hub-ram-demos && cargo build --release --example ultrasonic_test
//!   arm-none-eabi-objcopy -O binary \
//!       target/thumbv7em-none-eabihf/release/examples/ultrasonic_test \
//!       ultrasonic_test.bin
//!   # then in spike shell: upload <size>, send COBS, go

#![no_std]
#![no_main]

use spike_hub_api::{MonitorApi, BTN_CENTER};

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 8 {
        api.print(b"ERR: need API v8+\r\n");
        return 1;
    }

    api.print(b"=== Ultrasonic Distance port E (center btn = stop) ===\r\n");

    // Port E = index 4. Firmware already selects MODE_US_DISTL during handshake.
    // Wait for sensor to start streaming data.
    let mut buf = [0u8; 32];
    for _ in 0..10 {
        (api.delay_ms)(100);
        let _ = (api.port_read)(4, buf.as_mut_ptr(), 32);
    }

    let mut frame: u32 = 0;

    loop {
        if (api.read_buttons)() & BTN_CENTER != 0 {
            break;
        }

        let n = (api.port_read)(4, buf.as_mut_ptr(), 32);

        // Print every 5th frame (~500ms at 100ms loop)
        if frame % 5 == 0 && n >= 2 {
            let dist = i16::from_le_bytes([buf[0], buf[1]]);

            api.print(b"dist=");
            print_i16(api, dist);
            api.print(b"mm\r\n");

            // LED bar graph: 5 rows, each row = 400mm
            // Row 0 (top) = 0-400mm, row 4 (bottom) = 1600-2000mm
            // Fill pixels in the row proportional to distance
            clear_matrix(api);
            if dist >= 0 {
                let d = dist as u32;
                // Number of full LEDs to fill (25 LEDs, 80mm each)
                let filled = (d / 80).min(25) as usize;
                for i in 0..filled {
                    (api.set_pixel)(i as u32, 60);
                }
            }
            (api.update_leds)();
        }

        frame = frame.wrapping_add(1);
        (api.delay_ms)(100);
    }

    clear_matrix(api);
    // Show checkmark
    let check: [u32; 5] = [24, 18, 12, 11, 5];
    for &px in check.iter() {
        (api.set_pixel)(px, 60);
    }
    (api.update_leds)();

    api.print(b"Done. frames=");
    print_u32(api, frame);
    api.print(b"\r\n");
    0
}

fn clear_matrix(api: &MonitorApi) {
    for i in 0..25u32 {
        (api.set_pixel)(i, 0);
    }
}

fn print_i16(api: &MonitorApi, val: i16) {
    if val < 0 {
        api.print(b"-");
        print_u32(api, (-val) as u32);
    } else {
        print_u32(api, val as u32);
    }
}

fn print_u32(api: &MonitorApi, mut val: u32) {
    if val == 0 {
        api.print(b"0");
        return;
    }
    let mut buf = [0u8; 10];
    let mut pos = 10;
    while val > 0 && pos > 0 {
        pos -= 1;
        buf[pos] = b'0' + (val % 10) as u8;
        val /= 10;
    }
    api.print(&buf[pos..10]);
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
