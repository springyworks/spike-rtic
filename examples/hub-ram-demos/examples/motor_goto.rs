//! Motor position demo — drive motor to target positions using motor_goto.
//!
//! Requires: motor on Port A (default motor_poll port).
//! The firmware auto-syncs Port A in POS mode at boot.
//!
//! Sequence: goto 90°, goto 0°, goto 360°, goto 0°.
//! Prints position and error after each move.
//!
//! Build & upload:
//!   cd hub-ram-demos && cargo build --release --example motor_goto
//!   arm-none-eabi-objcopy -O binary \
//!       target/thumbv7em-none-eabihf/release/examples/motor_goto motor_goto.bin

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 7 {
        api.print(b"ERR: need API v7+ (motor_goto)\r\n");
        return 1;
    }

    api.print(b"=== Motor Goto Demo ===\r\n");

    // Read initial position
    let pos0 = (api.motor_position)();
    print_i32(api, b"Start pos: ", pos0, b"deg\r\n");

    // Move sequence
    let targets: [i32; 4] = [90, 0, 360, 0];
    for &target in targets.iter() {
        print_i32(api, b"Goto ", target, b"deg...\r\n");
        let err = (api.motor_goto)(0, target);
        let pos = (api.motor_position)();
        print_i32(api, b"  pos=", pos, b"");
        print_i32(api, b" err=", err, b"deg\r\n");
        (api.delay_ms)(500);
    }

    api.print(b"Done.\r\n");
    0
}

/// Print a message with an i32 value (no alloc, no format!).
fn print_i32(api: &MonitorApi, prefix: &[u8], val: i32, suffix: &[u8]) {
    api.print(prefix);
    let mut buf = [0u8; 12];
    let s = i32_to_bytes(val, &mut buf);
    api.print(s);
    api.print(suffix);
}

fn i32_to_bytes(mut val: i32, buf: &mut [u8; 12]) -> &[u8] {
    if val == 0 {
        buf[0] = b'0';
        return &buf[..1];
    }
    let neg = val < 0;
    if neg { val = -val; }
    let mut i = buf.len();
    while val > 0 {
        i -= 1;
        buf[i] = b'0' + (val % 10) as u8;
        val /= 10;
    }
    if neg {
        i -= 1;
        buf[i] = b'-';
    }
    &buf[i..]
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
