//! sensor-dump — raw RGBI dump from color sensor until center button.
//!
//! Continuously calls `sensor_read` every 100 ms and prints:
//! - Byte count returned by the ring buffer
//! - Raw hex dump of the data bytes
//! - Parsed RGBI values (Red, Green, Blue, Intensity as u16)
//!
//! Useful for debugging LUMP keepalive timing, ring buffer health,
//! and verifying that mode 5 (RGBI × u16 × 4 = 8 bytes) is active.
//!
//! Requires: color sensor on port F, MonitorApi v3+.
//! Exit: press center button.
//!
//! Build:  `cargo build --release --example sensor_dump`

#![no_std]
#![no_main]

use spike_hub_api::{MonitorApi, BTN_CENTER};

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 3 {
        api.print(b"ERR: need API v3+\r\n");
        return 1;
    }

    api.print(b"=== Sensor Dump (center btn = stop) ===\r\n");

    // Request mode 5 (RGBI)
    (api.sensor_mode)(5);
    api.print(b"Requested mode 5 (RGBI)\r\n");
    let mut buf = [0u8; 32];
    // Short delays with sensor_read calls to maintain keepalives
    for _ in 0..4 {
        (api.delay_ms)(50);
        let _ = (api.sensor_read)(buf.as_mut_ptr(), 32);
    }

    let mut frame: u32 = 0;

    loop {
        if (api.read_buttons)() & BTN_CENTER != 0 {
            break;
        }

        let n = (api.sensor_read)(buf.as_mut_ptr(), 32);

        // Print every 10th frame (~100 ms at 10 ms loop)
        if frame % 10 == 0 {
            api.print(b"[");
            print_num(api, frame);
            api.print(b"] n=");
            print_num(api, n);

            if n > 0 {
                api.print(b" hex:");
                for i in 0..(n as usize).min(16) {
                    api.print(b" ");
                    print_hex8(api, buf[i]);
                }
            }

            if n >= 8 {
                let r = i16::from_le_bytes([buf[0], buf[1]]);
                let g = i16::from_le_bytes([buf[2], buf[3]]);
                let b = i16::from_le_bytes([buf[4], buf[5]]);
                let i_val = i16::from_le_bytes([buf[6], buf[7]]);
                api.print(b"  R=");
                print_i16(api, r);
                api.print(b" G=");
                print_i16(api, g);
                api.print(b" B=");
                print_i16(api, b);
                api.print(b" I=");
                print_i16(api, i_val);
            } else if n == 0 {
                api.print(b" (empty)");
            }
            api.print(b"\r\n");
        }

        frame = frame.wrapping_add(1);
        (api.delay_ms)(10);
    }

    api.print(b"\r\nDone. frames=");
    print_num(api, frame);
    api.print(b"\r\n");
    0
}

// ── Helpers ──

fn print_num(api: &MonitorApi, val: u32) {
    let mut b = [0u8; 10];
    let s = fmt_u32(val, &mut b);
    api.print(s);
}

fn print_i16(api: &MonitorApi, val: i16) {
    if val < 0 {
        api.print(b"-");
        print_num(api, (-(val as i32)) as u32);
    } else {
        print_num(api, val as u32);
    }
}

fn print_hex8(api: &MonitorApi, val: u8) {
    const HEX: &[u8] = b"0123456789ABCDEF";
    let b = [HEX[(val >> 4) as usize], HEX[(val & 0xF) as usize]];
    api.print(&b);
}

fn fmt_u32(mut val: u32, buf: &mut [u8; 10]) -> &[u8] {
    if val == 0 {
        buf[0] = b'0';
        return &buf[..1];
    }
    let mut pos = 10;
    while val > 0 && pos > 0 {
        pos -= 1;
        buf[pos] = b'0' + (val % 10) as u8;
        val /= 10;
    }
    &buf[pos..]
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
