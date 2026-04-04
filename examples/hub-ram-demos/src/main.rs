//! Default RAM demo binary — motor ramp test on Port B.
//!
//! This is the `[[bin]]` target of the `hub-ram-demos` crate (the binary
//! produced by `cargo build --release` without `--example`).  Individual
//! demos live in the `examples/` directory; this binary serves as a
//! quick smoke-test that exercises motor_set, motor_brake, read_adc,
//! and LED pixel APIs to confirm the MonitorApi callback table works.
//!
//! Uses MonitorApi callbacks (motor_set, motor_brake, set_pixel,
//! update_leds, read_adc, delay_ms) — no direct register access.
//!
//! Build & upload:
//! ```sh
//! cd hub-ram-demos && cargo build --release
//! arm-none-eabi-objcopy -O binary \
//!     target/thumbv7em-none-eabihf/release/spike-demo spike-demo.bin
//! python3 ../../helper-tools/upload_demo.py spike-demo.bin
//! ```

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

/// Motor port B index (0=A, 1=B, 2=C, 3=D, 4=E, 5=F).
const PORT_B: u32 = 1;

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    // Check API version
    if api.version < 2 {
        api.print(b"ERR: need API v2+ (motor support)\r\n");
        return 1;
    }

    api.print(b"=== Motor Test via API (Port B) ===\r\n");

    // Forward ramp
    api.print(b"  Forward 30%...\r\n");
    (api.set_pixel)(2, 80);
    (api.update_leds)();
    (api.motor_set)(PORT_B, 30);
    (api.delay_ms)(1000);

    api.print(b"  Forward 60%...\r\n");
    (api.motor_set)(PORT_B, 60);
    (api.delay_ms)(1000);

    api.print(b"  Forward 90%...\r\n");
    (api.motor_set)(PORT_B, 90);
    (api.delay_ms)(1000);

    // Coast
    api.print(b"  Coast...\r\n");
    (api.set_pixel)(2, 0);
    (api.set_pixel)(12, 50);
    (api.update_leds)();
    (api.motor_set)(PORT_B, 0);
    (api.delay_ms)(500);

    // Reverse ramp
    api.print(b"  Reverse 30%...\r\n");
    (api.set_pixel)(12, 0);
    (api.set_pixel)(22, 80);
    (api.update_leds)();
    (api.motor_set)(PORT_B, -30);
    (api.delay_ms)(1000);

    api.print(b"  Reverse 60%...\r\n");
    (api.motor_set)(PORT_B, -60);
    (api.delay_ms)(1000);

    api.print(b"  Reverse 90%...\r\n");
    (api.motor_set)(PORT_B, -90);
    (api.delay_ms)(1000);

    // Brake
    api.print(b"  Brake...\r\n");
    (api.set_pixel)(22, 0);
    (api.set_pixel)(6, 50);
    (api.set_pixel)(8, 50);
    (api.update_leds)();
    (api.motor_brake)(PORT_B);
    (api.delay_ms)(1000);

    // Stop
    api.print(b"  Stop.\r\n");
    (api.motor_set)(PORT_B, 0);

    // Battery check
    let bat = (api.read_adc)(11);
    api.print(b"bat_raw=");
    let mut buf = [0u8; 8];
    let s = fmt_u32(bat, &mut buf);
    api.print(s);
    api.print(b"\r\n");

    // Checkmark
    for i in 0..25u32 {
        (api.set_pixel)(i, 0);
    }
    (api.set_pixel)(19, 60);
    (api.set_pixel)(17, 60);
    (api.set_pixel)(11, 60);
    (api.set_pixel)(10, 60);
    (api.update_leds)();

    api.print(b"=== Motor test complete (API v2) ===\r\n");
    0
}

fn fmt_u32(mut val: u32, buf: &mut [u8; 8]) -> &[u8] {
    if val == 0 {
        buf[0] = b'0';
        return &buf[..1];
    }
    let mut pos = 8;
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
