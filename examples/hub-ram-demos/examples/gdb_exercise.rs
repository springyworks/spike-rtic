//! gdb_exercise — A demo designed for exercising VS Code GDB debugging.
//!
//! This demo has multiple functions, local variables, loops, and
//! interesting data structures — ideal for setting breakpoints,
//! watching variables, and single-stepping through code.
//!
//! Build with debug info:
//!   cd examples/hub-ram-demos
//!   cargo build --example gdb_exercise --release
//!
//! Then objcopy forcontinue
//!  upload:
//!   arm-none-eabi-objcopy -O binary \
//!     target/thumbv7em-none-eabihf/release/examples/gdb_exercise \
//!     target/spike-usr_bins/gdb_exercise.bin
//!
//! Upload + enter GDB:
//!   python3 helper-tools/upload_demo.py target/spike-usr_bins/gdb_exercise.bin
//!   # then in shell: `gdb`  to enter RSP mode
//!   # then F5 in VS Code to attach

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

// ── Data structures for watchpoints and inspection ──

struct SensorSample {
    r: u16,
    g: u16,
    b: u16,
    intensity: u16,
}

struct MotorState {
    port: u32,
    speed: i32,
    position: i32,
    direction: bool,
}

// ── Breakpoint-friendly functions ──

/// Classify a color from RGBI values — good stepping target.
fn classify_color(sample: &SensorSample) -> &'static [u8] {
    // Set a breakpoint here to inspect sample fields
    let max = max3(sample.r, sample.g, sample.b);

    if sample.intensity < 20 {
        b"black"
    } else if sample.intensity > 200 && sample.r > 150 && sample.g > 150 && sample.b > 150 {
        b"white"
    } else if max == sample.r && sample.r > sample.g * 2 {
        b"red"
    } else if max == sample.g && sample.g > sample.r * 2 {
        b"green"
    } else if max == sample.b && sample.b > sample.r * 2 {
        b"blue"
    } else if sample.r > 100 && sample.g > 80 && sample.b < 50 {
        b"yellow"
    } else {
        b"unknown"
    }
}

fn max3(a: u16, b: u16, c: u16) -> u16 {
    let mut m = a;
    if b > m { m = b; }
    if c > m { m = c; }
    m
}

/// Fibonacci — good for stepping through a simple loop.
fn fibonacci(n: u32) -> u32 {
    let mut a: u32 = 0;
    let mut b: u32 = 1;
    for _ in 0..n {
        let tmp = b;
        b = a.wrapping_add(b);
        a = tmp;
    }
    a
}

/// Compute a simple checksum over a byte slice.
fn checksum(data: &[u8]) -> u32 {
    let mut sum: u32 = 0;
    for &byte in data {
        sum = sum.wrapping_add(byte as u32);
        sum = sum.wrapping_mul(31);
    }
    sum
}

/// Motor ramp — drives a motor through speed steps.
/// Set a watchpoint on `state.speed` to catch changes.
fn motor_ramp(api: &MonitorApi, state: &mut MotorState) {
    let speeds: [i32; 6] = [0, 30, 50, 70, 50, 0];

    for &spd in speeds.iter() {
        // Breakpoint here: inspect spd and state each iteration
        state.speed = spd;
        let actual = if state.direction { spd } else { -spd };
        (api.motor_set)(state.port, actual);
        api.print(b"  motor speed=");
        print_i32(api, actual);
        api.print(b"\r\n");
        (api.delay_ms)(300);
    }

    (api.motor_brake)(state.port);
}

/// LED pattern — lights pixels in a cross pattern.
fn show_cross(api: &MonitorApi) {
    // row 2 (middle row)
    for col in 0..5u32 {
        (api.set_pixel)(2 * 5 + col, 60);
    }
    // col 2 (middle column)
    for row in 0..5u32 {
        (api.set_pixel)(row * 5 + 2, 60);
    }
    (api.update_leds)();
}

fn clear_leds(api: &MonitorApi) {
    for i in 0..25u32 {
        (api.set_pixel)(i, 0);
    }
    (api.update_leds)();
}

// ── Printing helpers ──

fn print_u32(api: &MonitorApi, val: u32) {
    let mut buf = [b'0'; 10];
    let mut v = val;
    let mut i = 9;
    if v == 0 {
        api.print(b"0");
        return;
    }
    while v > 0 {
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if i == 0 { break; }
        i -= 1;
    }
    api.print(&buf[i..]);
}

fn print_i32(api: &MonitorApi, val: i32) {
    if val < 0 {
        api.print(b"-");
        print_u32(api, (-(val as i64)) as u32);
    } else {
        print_u32(api, val as u32);
    }
}

fn print_hex_u32(api: &MonitorApi, val: u32) {
    let hex = b"0123456789abcdef";
    let mut buf = [b'0'; 8];
    for i in 0..8 {
        buf[7 - i] = hex[((val >> (i * 4)) & 0xF) as usize];
    }
    api.print(b"0x");
    api.print(&buf);
}

// ════════════════════════════════════════════════════════
//  Entry point
// ════════════════════════════════════════════════════════

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    // ── Version gate ──
    if api.version < 2 {
        api.print(b"ERROR: need API v2+\r\n");
        return 1;
    }

    api.print(b"\r\n=== GDB Exercise Demo ===\r\n");
    api.print(b"Set breakpoints, watchpoints, and step through.\r\n\r\n");

    // ── Phase 1: Fibonacci (step through the loop) ──
    api.print(b"Phase 1: Fibonacci sequence\r\n");
    for n in 0..12u32 {
        let result = fibonacci(n);  // ← BREAKPOINT: inspect n, result
        api.print(b"  fib(");
        print_u32(api, n);
        api.print(b") = ");
        print_u32(api, result);
        api.print(b"\r\n");
    }

    (api.delay_ms)(500);

    // ── Phase 2: Fake sensor data + classification ──
    api.print(b"\r\nPhase 2: Color classification\r\n");
    let samples = [
        SensorSample { r: 200, g: 30, b: 25, intensity: 180 },   // red
        SensorSample { r: 20, g: 180, b: 15, intensity: 160 },   // green
        SensorSample { r: 15, g: 20, b: 210, intensity: 170 },   // blue
        SensorSample { r: 220, g: 200, b: 190, intensity: 240 }, // white
        SensorSample { r: 5, g: 3, b: 2, intensity: 8 },         // black
        SensorSample { r: 180, g: 140, b: 20, intensity: 200 },  // yellow
    ];

    for (i, sample) in samples.iter().enumerate() {
        let name = classify_color(sample);  // ← BREAKPOINT: step into
        api.print(b"  sample[");
        print_u32(api, i as u32);
        api.print(b"] RGBI=(");
        print_u32(api, sample.r as u32);
        api.print(b",");
        print_u32(api, sample.g as u32);
        api.print(b",");
        print_u32(api, sample.b as u32);
        api.print(b",");
        print_u32(api, sample.intensity as u32);
        api.print(b") -> ");
        api.print(name);
        api.print(b"\r\n");
    }

    (api.delay_ms)(500);

    // ── Phase 3: Checksum computation ──
    api.print(b"\r\nPhase 3: Checksums\r\n");
    let messages: [&[u8]; 3] = [
        b"hello spike",
        b"GDB exercise",
        b"breakpoint here!",
    ];
    for msg in messages.iter() {
        let csum = checksum(msg);  // ← BREAKPOINT: watch csum
        api.print(b"  crc32(\"");
        api.print(msg);
        api.print(b"\") = ");
        print_hex_u32(api, csum);
        api.print(b"\r\n");
    }

    (api.delay_ms)(500);

    // ── Phase 4: LED cross pattern ──
    api.print(b"\r\nPhase 4: LED cross\r\n");
    show_cross(api);  // ← BREAKPOINT: step into pixel-setting
    (api.delay_ms)(1000);
    clear_leds(api);

    // ── Phase 5: Motor ramp (if API v2+) ──
    api.print(b"\r\nPhase 5: Motor ramp on port B\r\n");
    let mut motor = MotorState {
        port: 1,       // port B
        speed: 0,
        position: 0,
        direction: true,
    };
    // Set a DWT watchpoint on motor.speed to catch every change:
    //   dwt set 0 <addr_of_motor.speed> w
    // Or in GDB: watch motor.speed
    motor_ramp(api, &mut motor);

    (api.delay_ms)(500);

    // ── Phase 6: Button poll loop ──
    api.print(b"\r\nPhase 6: Press CENTER to finish (or wait 5s)\r\n");
    let mut ticks: u32 = 0;
    loop {
        let btns = (api.read_buttons)();
        if btns & 0x01 != 0 {
            // Center button pressed — exit
            api.print(b"  CENTER pressed, exiting.\r\n");
            break;
        }
        ticks += 1;
        if ticks >= 100 {
            // Timeout after ~5s (100 × 50ms)
            api.print(b"  Timeout, exiting.\r\n");
            break;
        }
        (api.delay_ms)(50);
    }

    api.print(b"\r\n=== GDB Exercise complete ===\r\n");
    api.print(b"Exit code: 0\r\n");

    0
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
