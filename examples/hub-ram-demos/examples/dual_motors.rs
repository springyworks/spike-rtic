//! Dual-motor choreography (Port A + Port B).
//!
//! Drives motors A and B through a synchronized dance pattern:
//! forward ramp, spin (opposite directions), brake, reverse,
//! and coast — with LED matrix feedback at each stage.
//!
//! Build & upload:
//!   cd hub-ram-demos && cargo build --release
//!   arm-none-eabi-objcopy -O binary \
//!       target/thumbv7em-none-eabihf/release/dual-motors dual-motors.bin
//!   # then in spike shell: upload <size>, send COBS, go

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

const PORT_A: u32 = 0;
const PORT_B: u32 = 1;

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 2 {
        api.print(b"ERR: need API v2+ (motor support)\r\n");
        return 1;
    }

    api.print(b"=== Dual Motor Dance (A + B) ===\r\n");
    clear_matrix(api);

    // ── Stage 1: Forward ramp (both motors, same direction) ──
    api.print(b"[1] Forward ramp...\r\n");
    show_arrow_up(api);
    for speed in [20i32, 40, 60, 80] {
        (api.motor_set)(PORT_A, speed);
        (api.motor_set)(PORT_B, speed);
        (api.delay_ms)(400);
    }

    // ── Stage 2: Spin — motors opposite directions ──
    api.print(b"[2] Spin (A fwd, B rev)...\r\n");
    show_spin(api);
    (api.motor_set)(PORT_A, 70);
    (api.motor_set)(PORT_B, -70);
    (api.delay_ms)(1500);

    // ── Stage 3: Reverse spin ──
    api.print(b"[3] Reverse spin (A rev, B fwd)...\r\n");
    (api.motor_set)(PORT_A, -70);
    (api.motor_set)(PORT_B, 70);
    (api.delay_ms)(1500);

    // ── Stage 4: Brake both ──
    api.print(b"[4] Brake...\r\n");
    show_stop(api);
    (api.motor_brake)(PORT_A);
    (api.motor_brake)(PORT_B);
    (api.delay_ms)(800);

    // ── Stage 5: Reverse ramp ──
    api.print(b"[5] Reverse ramp...\r\n");
    show_arrow_down(api);
    for speed in [20i32, 40, 60, 80] {
        (api.motor_set)(PORT_A, -speed);
        (api.motor_set)(PORT_B, -speed);
        (api.delay_ms)(400);
    }

    // ── Stage 6: Alternating pulses ──
    api.print(b"[6] Alternating pulses...\r\n");
    for _ in 0..4 {
        show_left(api);
        (api.motor_set)(PORT_A, 90);
        (api.motor_set)(PORT_B, 0);
        (api.delay_ms)(300);
        show_right(api);
        (api.motor_set)(PORT_A, 0);
        (api.motor_set)(PORT_B, 90);
        (api.delay_ms)(300);
    }

    // ── Done: coast + checkmark ──
    (api.motor_set)(PORT_A, 0);
    (api.motor_set)(PORT_B, 0);
    show_check(api);
    api.print(b"=== Dual motor dance complete ===\r\n");
    0
}

// ── LED helpers (5×5 matrix, pixels 0–24, row-major) ──

fn clear_matrix(api: &MonitorApi) {
    for i in 0..25u32 {
        (api.set_pixel)(i, 0);
    }
    (api.update_leds)();
}

fn show_arrow_up(api: &MonitorApi) {
    clear_matrix(api);
    // Up arrow: pixel 2 (top center), 6,8 (second row sides), 12 (center)
    for &p in &[2u32, 6, 8, 12, 17, 22] {
        (api.set_pixel)(p, 60);
    }
    (api.update_leds)();
}

fn show_arrow_down(api: &MonitorApi) {
    clear_matrix(api);
    for &p in &[2u32, 7, 12, 16, 18, 22] {
        (api.set_pixel)(p, 60);
    }
    (api.update_leds)();
}

fn show_spin(api: &MonitorApi) {
    clear_matrix(api);
    // Circular pattern
    for &p in &[1u32, 2, 3, 9, 14, 23, 22, 21, 15, 10] {
        (api.set_pixel)(p, 50);
    }
    (api.update_leds)();
}

fn show_stop(api: &MonitorApi) {
    clear_matrix(api);
    // X pattern
    for &p in &[0u32, 4, 6, 8, 12, 16, 18, 20, 24] {
        (api.set_pixel)(p, 80);
    }
    (api.update_leds)();
}

fn show_left(api: &MonitorApi) {
    clear_matrix(api);
    for &p in &[2u32, 6, 10, 11, 12, 16, 22] {
        (api.set_pixel)(p, 60);
    }
    (api.update_leds)();
}

fn show_right(api: &MonitorApi) {
    clear_matrix(api);
    for &p in &[2u32, 8, 12, 13, 14, 18, 22] {
        (api.set_pixel)(p, 60);
    }
    (api.update_leds)();
}

fn show_check(api: &MonitorApi) {
    clear_matrix(api);
    for &p in &[19u32, 17, 11, 10] {
        (api.set_pixel)(p, 60);
    }
    (api.update_leds)();
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
