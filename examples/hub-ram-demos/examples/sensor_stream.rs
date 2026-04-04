//! sensor_stream — Continuous 4-port sensor data stream.
//!
//! Reads all 4 active ports (A+B motors, E ultrasonic, F color) via
//! port_read API and prints a clean status line every 200ms.
//! Motors nudge forward briefly at startup so you can see them move,
//! then coast while the stream runs.
//!
//! Ctrl-C (or center button) to stop.
//!
//! Requires: MonitorApi v8+, sensors started with `sensor e` + `sensor f`
//!
//! Build:
//!   cd examples/hub-ram-demos
//!   cargo build --release --example sensor_stream

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

const PORT_A: u32 = 0;
const PORT_B: u32 = 1;
const PORT_E: u32 = 4;
const PORT_F: u32 = 5;

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 8 {
        api.print(b"Need API v8+\r\n");
        return 1;
    }

    api.print(b"=== Sensor Stream (v8) ===\r\n");
    api.print(b"Center button = exit\r\n\r\n");

    // Nudge motors briefly so user sees them move
    api.print(b"Motor nudge...");
    (api.motor_set)(PORT_A, 30);
    (api.motor_set)(PORT_B, -30);
    (api.delay_ms)(400);
    (api.motor_set)(PORT_A, 0);
    (api.motor_set)(PORT_B, 0);
    api.print(b" done\r\n\r\n");

    // Header
    api.print(b"  Ultra(E)   Color(F) R/G/B/I         MotorA  MotorB\r\n");
    api.print(b"  --------   --------------------      ------  ------\r\n");

    let mut prev_buttons: u8 = 0;
    let mut cycle: u32 = 0;

    loop {
        cycle += 1;

        // ── Read all 4 ports ──
        let mut buf = [0u8; 8];

        // Ultrasonic (E) — mode 0: 2 bytes i16 distance mm
        let en = (api.port_read)(PORT_E, buf.as_mut_ptr(), 8);
        let dist: i16 = if en >= 2 {
            i16::from_le_bytes([buf[0], buf[1]])
        } else {
            -1
        };

        // Color (F) — mode 5: 8 bytes RGBI × u16
        let mut cbuf = [0u8; 8];
        let cn = (api.port_read)(PORT_F, cbuf.as_mut_ptr(), 8);
        let (cr, cg, cb, ci) = if cn >= 8 {
            (
                u16::from_le_bytes([cbuf[0], cbuf[1]]),
                u16::from_le_bytes([cbuf[2], cbuf[3]]),
                u16::from_le_bytes([cbuf[4], cbuf[5]]),
                u16::from_le_bytes([cbuf[6], cbuf[7]]),
            )
        } else {
            (0, 0, 0, 0)
        };

        // Motor A — mode 2: 4 bytes i32 degrees
        let mut abuf = [0u8; 8];
        let an = (api.port_read)(PORT_A, abuf.as_mut_ptr(), 8);
        let pos_a: i32 = if an >= 4 {
            i32::from_le_bytes([abuf[0], abuf[1], abuf[2], abuf[3]])
        } else {
            0
        };

        // Motor B
        let mut bbuf = [0u8; 8];
        let bn = (api.port_read)(PORT_B, bbuf.as_mut_ptr(), 8);
        let pos_b: i32 = if bn >= 4 {
            i32::from_le_bytes([bbuf[0], bbuf[1], bbuf[2], bbuf[3]])
        } else {
            0
        };

        // ── Buttons ──
        let buttons = (api.read_buttons)();
        let pressed = buttons & !prev_buttons;
        prev_buttons = buttons;
        if pressed & 0x02 != 0 {
            api.print(b"\r\n=== Exit ===\r\n");
            return 0;
        }

        // ── Print line ──
        api.print(b"  ");
        // Distance
        if dist >= 0 && dist < 2000 {
            print_i32_pad(api, dist as i32, 4);
            api.print(b"mm  ");
        } else {
            api.print(b" ---mm  ");
        }

        // Color RGBI
        api.print(b" R");
        print_u16_pad(api, cr, 3);
        api.print(b" G");
        print_u16_pad(api, cg, 3);
        api.print(b" B");
        print_u16_pad(api, cb, 3);
        api.print(b" I");
        print_u16_pad(api, ci, 3);

        // Motor positions
        api.print(b"     ");
        print_i32_pad(api, pos_a, 6);
        api.print(b"d ");
        print_i32_pad(api, pos_b, 6);
        api.print(b"d\r\n");

        (api.delay_ms)(200);

        // Auto-exit after 5 minutes
        if cycle > 1500 {
            api.print(b"\r\n=== Timeout ===\r\n");
            return 0;
        }
    }
}

fn print_i32_pad(api: &MonitorApi, v: i32, width: usize) {
    let mut buf = [b' '; 12];
    let neg = v < 0;
    let mut abs = if neg { (-(v as i64)) as u32 } else { v as u32 };
    let mut i = 11usize;
    if abs == 0 {
        buf[i] = b'0';
        i -= 1;
    }
    while abs > 0 {
        buf[i] = b'0' + (abs % 10) as u8;
        abs /= 10;
        if i == 0 { break; }
        i -= 1;
    }
    if neg {
        buf[i] = b'-';
        if i > 0 { i -= 1; }
    }
    let digits = 12 - (i + 1);
    let pad = if width > digits { width - digits } else { 0 };
    let start = if pad > 0 && (i + 1) > pad { i + 1 - pad } else { i + 1 };
    (api.write_fn)(api.context, buf[start..].as_ptr(), (12 - start) as u32);
}

fn print_u16_pad(api: &MonitorApi, v: u16, width: usize) {
    print_i32_pad(api, v as i32, width);
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
