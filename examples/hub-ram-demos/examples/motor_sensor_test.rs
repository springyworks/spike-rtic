//! Minimal motor + sensor integration test — verify MonitorApi callbacks.
//!
//! Exercises the core API in a fixed sequence:
//!   1. Motor A forward at 50% PWM for 2 s
//!   2. Motor A electromagnetic brake, hold 0.5 s
//!   3. Read color sensor 20 × at 100 ms intervals (mode 5 RGBI)
//!   4. Motor A reverse at 50% PWM for 2 s
//!   5. Motor A coast (free-spin)
//!
//! Serves as a smoke test to confirm that `motor_set`, `motor_brake`,
//! `sensor_read`, and `delay_ms` callbacks respond correctly under
//! realistic timing.  No closed-loop control — pure open-loop sequence.
//!
//! Requires: motor on port A, color sensor on port F, MonitorApi v3+.
//!
//! Build:  `cargo build --release --example motor_sensor_test`

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

const PORT_A: u32 = 0;

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    api.print(b"=== Motor+Sensor Test ===\r\n");

    // Test 1: Motor A forward
    api.print(b"Motor A fwd 50...\r\n");
    (api.motor_set)(PORT_A, 50);
    (api.delay_ms)(2000);
    (api.motor_brake)(PORT_A);
    api.print(b"Motor A brake\r\n");
    (api.delay_ms)(500);

    // Test 2: Read sensor 20 times
    api.print(b"Sensor reads (20x 100ms):\r\n");
    for i in 0u32..20 {
        let mut buf = [0u8; 16];
        let n = (api.sensor_read)(buf.as_mut_ptr(), 16);

        // Print tick, byte count, and raw hex
        print_u32(api, i);
        api.print(b": n=");
        print_u32(api, n);
        api.print(b" [");
        for j in 0..(n as usize).min(8) {
            print_hex_byte(api, buf[j]);
            if j < (n as usize).min(8) - 1 { api.print(b" "); }
        }
        api.print(b"]");

        // Parse RGBI if we have 8 bytes
        if n >= 8 {
            let r = u16::from_le_bytes([buf[0], buf[1]]);
            let g = u16::from_le_bytes([buf[2], buf[3]]);
            let b = u16::from_le_bytes([buf[4], buf[5]]);
            let i_val = u16::from_le_bytes([buf[6], buf[7]]);
            api.print(b" R="); print_u32(api, r as u32);
            api.print(b" G="); print_u32(api, g as u32);
            api.print(b" B="); print_u32(api, b as u32);
            api.print(b" I="); print_u32(api, i_val as u32);
        }
        api.print(b"\r\n");

        (api.delay_ms)(100);
    }

    // Test 3: Motor A reverse
    api.print(b"Motor A rev -50...\r\n");
    (api.motor_set)(PORT_A, -50);
    (api.delay_ms)(2000);
    (api.motor_set)(PORT_A, 0);
    api.print(b"Motor A coast\r\n");

    api.print(b"=== Done ===\r\n");
    0
}

fn print_u32(api: &MonitorApi, mut v: u32) {
    if v == 0 { api.print(b"0"); return; }
    let mut buf = [0u8; 10];
    let mut i = 10usize;
    while v > 0 { i -= 1; buf[i] = b'0' + (v % 10) as u8; v /= 10; }
    (api.write_fn)(api.context, buf[i..].as_ptr(), (10 - i) as u32);
}

fn print_hex_byte(api: &MonitorApi, b: u8) {
    let hi = b >> 4;
    let lo = b & 0x0F;
    let buf = [
        if hi < 10 { b'0' + hi } else { b'A' + hi - 10 },
        if lo < 10 { b'0' + lo } else { b'A' + lo - 10 },
    ];
    (api.write_fn)(api.context, buf.as_ptr(), 2);
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
