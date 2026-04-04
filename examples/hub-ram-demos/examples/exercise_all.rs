//! exercise_all — Device discovery + sensor-reactive parallel exercise.
//!
//! Probes all 6 ports (A–F), then runs a reactive control loop:
//!   - Sensors (ultrasonic, color) are read every cycle at 20Hz
//!   - Sensor CHANGES drive motor energy: big delta → kick pulse
//!   - Motors use 70% base PWM (above stiction), 100% kick on events
//!   - Hub status LED (USB glass pipe) cycles colors with activity
//!   - LED matrix shows live device status + change intensity
//!   - Sound pitch tracks sensor excitement level
//!
//! Anti-stiction strategy:
//!   - Base PWM 70% (above ~25% stiction threshold)
//!   - Sensor change → 100% kick for 3 cycles (150ms), then resume
//!   - Alternate direction every 4s to keep gears moving
//!   - motor_goto() for position jumps (has ramp+nudge controller)
//!
//! Status LED behavior:
//!   - Idle: dim white breathing
//!   - Sensor delta: color encodes which sensor changed
//!     (ultrasonic=cyan, color=magenta, IMU=yellow)
//!   - Proximity alarm: red flash
//!   - Button press: green flash
//!   - Binary-delta blink: invert current color every 250ms on events
//!
//! Port probe heuristic:
//!   port_read(port) returns >0 bytes → device connected.
//!     4 bytes = motor (i32 degrees), 2 = ultrasonic (i16 mm),
//!     8 = color RGBI (4×i16), 1 = color ID
//!
//! Requires: MonitorApi v10, sensors started with `sensor e/f`
//!
//! Build:
//!   cd examples/hub-ram-demos
//!   cargo build --release --example exercise_all

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

const NUM_PORTS: usize = 6;
const PORT_NAMES: [u8; 6] = [b'A', b'B', b'C', b'D', b'E', b'F'];
const LOOP_MS: u32 = 50; // 20 Hz main loop
const BASE_SPEED: i32 = 70; // above stiction threshold
const KICK_SPEED: i32 = 100; // full power burst on sensor events
const KICK_CYCLES: u32 = 3; // kick lasts 150ms at 20Hz

#[derive(Copy, Clone, PartialEq)]
enum DevType {
    None,
    Motor,
    Ultrasonic,
    ColorRGBI,
    ColorID,
    Unknown,
}

#[derive(Copy, Clone)]
struct PortInfo {
    dev: DevType,
    data: [u8; 8],
    prev: [u8; 8],
    len: u32,
}

// Hub LED target color
#[derive(Copy, Clone)]
struct Rgb {
    r: u32,
    g: u32,
    b: u32,
}

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

    api.print(b"\r\n=== Exercise All v2: Sensor-Reactive ===\r\n");
    api.print(b"Buttons: L=reverse  R=speed  Center=exit\r\n\r\n");

    // ── Phase 1: Probe all ports ──
    api.print(b"--- Probing ports ---\r\n");
    let mut ports = [PortInfo {
        dev: DevType::None,
        data: [0u8; 8],
        prev: [0u8; 8],
        len: 0,
    }; NUM_PORTS];

    let mut motor_count: u32 = 0;
    let mut sensor_count: u32 = 0;

    // Flash hub LED blue during probe
    (api.set_hub_led)(0, 0, 80);

    (api.delay_ms)(200);
    for port in 0..NUM_PORTS {
        let mut buf = [0u8; 8];
        let n = (api.port_read)(port as u32, buf.as_mut_ptr(), 8);
        ports[port].len = n;
        ports[port].data = buf;
        ports[port].prev = buf; // initialize prev = current
        ports[port].dev = classify(n);

        api.print(b"  Port ");
        api.print(&[PORT_NAMES[port]]);
        api.print(b": ");
        match ports[port].dev {
            DevType::None => api.print(b"(empty)\r\n"),
            DevType::Motor => {
                let pos = i32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                api.print(b"MOTOR pos=");
                print_i32(api, pos);
                api.print(b"deg\r\n");
                motor_count += 1;
            }
            DevType::Ultrasonic => {
                let dist = i16::from_le_bytes([buf[0], buf[1]]);
                api.print(b"ULTRASONIC dist=");
                print_i32(api, dist as i32);
                api.print(b"mm\r\n");
                sensor_count += 1;
            }
            DevType::ColorRGBI => {
                let r = u16::from_le_bytes([buf[0], buf[1]]);
                let g = u16::from_le_bytes([buf[2], buf[3]]);
                let b_val = u16::from_le_bytes([buf[4], buf[5]]);
                let i = u16::from_le_bytes([buf[6], buf[7]]);
                api.print(b"COLOR R");
                print_u32(api, r as u32);
                api.print(b" G");
                print_u32(api, g as u32);
                api.print(b" B");
                print_u32(api, b_val as u32);
                api.print(b" I");
                print_u32(api, i as u32);
                api.print(b"\r\n");
                sensor_count += 1;
            }
            DevType::ColorID => {
                api.print(b"COLOR-ID val=");
                print_u32(api, buf[0] as u32);
                api.print(b"\r\n");
                sensor_count += 1;
            }
            DevType::Unknown => {
                api.print(b"UNKNOWN (");
                print_u32(api, n);
                api.print(b" bytes)\r\n");
            }
        }
    }

    // ── Phase 1b: Probe IMU ──
    let imu_ok = (api.imu_init)();
    api.print(b"  IMU: ");
    if imu_ok == 0x6A {
        api.print(b"LSM6DS3TR-C OK\r\n");
    } else {
        api.print(b"not detected\r\n");
    }

    api.print(b"\r\nFound: ");
    print_u32(api, motor_count);
    api.print(b" motor(s), ");
    print_u32(api, sensor_count);
    api.print(b" sensor(s)");
    if imu_ok == 0x6A {
        api.print(b", IMU");
    }
    api.print(b"\r\n\r\n");

    if motor_count == 0 && sensor_count == 0 && imu_ok != 0x6A {
        api.print(b"No devices found. Start sensors with `sensor <port>` first.\r\n");
        (api.set_hub_led)(100, 0, 0); // red = error
        return 1;
    }

    // ── Phase 2: Reactive exercise loop ──
    api.print(b"--- Exercising (60s, Center=exit) ---\r\n");
    clear_matrix(api);

    // Hub LED green = running
    (api.set_hub_led)(0, 60, 0);

    let mut direction: i32 = 1;
    let mut prev_buttons: u8 = 0;
    let mut cycle: u32 = 0;
    let mut kick_remaining: u32 = 0; // cycles of 100% kick left
    let mut excitement: u32 = 0; // decaying sensor activity counter
    let mut led_color = Rgb { r: 0, g: 20, b: 0 }; // current hub LED
    let mut led_inverted = false; // binary-delta blink state
    let mut prev_imu = [0u8; 12];

    loop {
        cycle += 1;

        // ── Read ALL ports, track deltas ──
        let mut total_delta: u32 = 0;

        for port in 0..NUM_PORTS {
            if ports[port].dev == DevType::None {
                continue;
            }
            ports[port].prev = ports[port].data;
            let mut buf = [0u8; 8];
            let n = (api.port_read)(port as u32, buf.as_mut_ptr(), 8);
            ports[port].data = buf;
            ports[port].len = n;

            // Compute delta for this port
            let d = port_delta(&ports[port]);
            total_delta += d;
        }

        // ── Read IMU + delta ──
        let mut imu_buf = [0u8; 12];
        let imu_n = if imu_ok == 0x6A {
            (api.imu_read)(imu_buf.as_mut_ptr(), 12)
        } else {
            0
        };
        let imu_delta = if imu_n >= 6 {
            let ax = i16::from_le_bytes([imu_buf[0], imu_buf[1]]);
            let pay = i16::from_le_bytes([prev_imu[0], prev_imu[1]]);
            let d = (ax as i32 - pay as i32).unsigned_abs() / 100;
            prev_imu = imu_buf;
            d
        } else {
            0
        };
        total_delta += imu_delta;

        // ── Update excitement (decaying accumulator) ──
        excitement = excitement.saturating_sub(1) + total_delta.min(20);
        let excited = excitement > 5;

        // ── Kick detection: big sensor change triggers motor burst ──
        if total_delta > 10 && kick_remaining == 0 {
            kick_remaining = KICK_CYCLES;
        }
        if kick_remaining > 0 {
            kick_remaining -= 1;
        }

        // ── Buttons ──
        let buttons = (api.read_buttons)();
        let pressed = buttons & !prev_buttons;
        prev_buttons = buttons;

        if pressed & 0x01 != 0 {
            // Center — exit
            stop_all_motors(api, &ports);
            (api.sound_stop)();
            clear_matrix(api);
            (api.set_hub_led)(0, 0, 0);
            api.print(b"\r\n=== Exit (center button) ===\r\n");
            return 0;
        }
        if pressed & 0x02 != 0 {
            direction = -direction;
            led_color = Rgb { r: 0, g: 80, b: 0 }; // green flash
        }
        if pressed & 0x04 != 0 {
            direction = -direction; // right also flips
            led_color = Rgb { r: 0, g: 80, b: 0 };
        }

        // ── Auto-reverse every 4s (80 cycles) ──
        if cycle % 80 == 0 {
            direction = -direction;
        }

        // ── Drive motors: base 70%, kick to 100% on events ──
        let motor_speed = if kick_remaining > 0 {
            KICK_SPEED
        } else {
            BASE_SPEED
        };

        // ── Proximity check BEFORE driving ──
        let mut alarm = false;
        for port in 0..NUM_PORTS {
            if ports[port].dev == DevType::Ultrasonic && ports[port].len >= 2 {
                let dist = i16::from_le_bytes([ports[port].data[0], ports[port].data[1]]);
                if dist >= 0 && dist < 100 {
                    alarm = true;
                }
            }
        }

        if alarm {
            // Proximity alarm: brake + red LED + alarm tone
            stop_all_motors(api, &ports);
            (api.sound_play)(1500);
            led_color = Rgb { r: 100, g: 0, b: 0 };
        } else {
            // Drive all motors
            for port in 0..NUM_PORTS {
                if ports[port].dev == DevType::Motor {
                    let spd = motor_speed * direction;
                    let motor_spd = if port % 2 == 0 { spd } else { -spd };
                    (api.motor_set)(port as u32, motor_spd);
                }
            }

            // Sound: pitch tracks excitement
            if excited {
                let freq = 300 + excitement.min(40) * 15; // 300-900 Hz
                (api.sound_play)(freq);
            } else if cycle % 40 == 0 {
                // Quiet heartbeat every 2s
                (api.sound_play)(220);
            } else if cycle % 40 == 2 {
                (api.sound_stop)();
            }
        }

        // ── Hub status LED: encode what's happening ──
        if !alarm {
            if total_delta > 10 {
                // Big change: pick color by source type
                led_color = delta_color(&ports);
                led_inverted = !led_inverted; // binary-delta toggle
            } else if excitement < 3 {
                // Idle: dim white breathing
                let breath = breath_val(cycle);
                led_color = Rgb {
                    r: breath,
                    g: breath,
                    b: breath,
                };
            }
            // Gradual decay toward idle
            led_color.r = led_color.r.saturating_sub(1);
            led_color.g = led_color.g.saturating_sub(1);
            led_color.b = led_color.b.saturating_sub(1);
        }

        // Apply (with binary-delta inversion on excitement)
        if led_inverted && excited {
            (api.set_hub_led)(
                100u32.saturating_sub(led_color.r),
                100u32.saturating_sub(led_color.g),
                100u32.saturating_sub(led_color.b),
            );
        } else {
            (api.set_hub_led)(led_color.r, led_color.g, led_color.b);
        }

        // ── LED matrix ──
        update_display(api, &ports, imu_n, &imu_buf, motor_speed, direction, excitement);

        // ── Status line every 500ms ──
        if cycle % 10 == 0 {
            print_status(api, &ports, imu_n, &imu_buf, motor_speed, direction, cycle, excitement);
        }

        // ── Yield ──
        (api.delay_ms)(LOOP_MS);

        // Auto-exit after 60s
        if cycle > 1200 {
            stop_all_motors(api, &ports);
            (api.sound_stop)();
            clear_matrix(api);
            (api.set_hub_led)(0, 0, 0);
            api.print(b"\r\n=== Done (60s timeout) ===\r\n");
            return 0;
        }
    }
}

// ── Device classification by byte count ──

fn classify(n: u32) -> DevType {
    match n {
        0 => DevType::None,
        1 => DevType::ColorID,
        2 => DevType::Ultrasonic,
        4 => DevType::Motor,
        8 => DevType::ColorRGBI,
        _ => DevType::Unknown,
    }
}

// ── Delta computation for a single port ──
fn port_delta(p: &PortInfo) -> u32 {
    match p.dev {
        DevType::Motor if p.len >= 4 => {
            let cur = i32::from_le_bytes([p.data[0], p.data[1], p.data[2], p.data[3]]);
            let prev = i32::from_le_bytes([p.prev[0], p.prev[1], p.prev[2], p.prev[3]]);
            (cur - prev).unsigned_abs().min(1000) / 10
        }
        DevType::Ultrasonic if p.len >= 2 => {
            let cur = i16::from_le_bytes([p.data[0], p.data[1]]) as i32;
            let prev = i16::from_le_bytes([p.prev[0], p.prev[1]]) as i32;
            (cur - prev).unsigned_abs().min(500) / 5
        }
        DevType::ColorRGBI if p.len >= 8 => {
            let mut d: u32 = 0;
            for ch in 0..4 {
                let off = ch * 2;
                let cur = u16::from_le_bytes([p.data[off], p.data[off + 1]]) as i32;
                let prev = u16::from_le_bytes([p.prev[off], p.prev[off + 1]]) as i32;
                d += (cur - prev).unsigned_abs().min(200);
            }
            d / 10
        }
        _ => 0,
    }
}

// ── Pick LED color based on which sensor type had biggest delta ──
fn delta_color(ports: &[PortInfo; NUM_PORTS]) -> Rgb {
    let mut best_d: u32 = 0;
    let mut best_type = DevType::None;
    for port in 0..NUM_PORTS {
        if ports[port].dev == DevType::None {
            continue;
        }
        let d = port_delta(&ports[port]);
        if d > best_d {
            best_d = d;
            best_type = ports[port].dev;
        }
    }
    match best_type {
        DevType::Ultrasonic => Rgb { r: 0, g: 80, b: 100 }, // cyan
        DevType::ColorRGBI | DevType::ColorID => Rgb { r: 80, g: 0, b: 100 }, // magenta
        DevType::Motor => Rgb { r: 100, g: 60, b: 0 }, // orange
        _ => Rgb { r: 50, g: 50, b: 0 }, // yellow = IMU/other
    }
}

// ── Sine-ish breathing 0-30 brightness, period ~3s ──
fn breath_val(cycle: u32) -> u32 {
    // Triangle wave: period 60 cycles = 3s at 20Hz
    let phase = cycle % 60;
    if phase < 30 {
        phase
    } else {
        60 - phase
    }
}

// ── Motor helpers ──

fn stop_all_motors(api: &MonitorApi, ports: &[PortInfo; NUM_PORTS]) {
    for port in 0..NUM_PORTS {
        if ports[port].dev == DevType::Motor {
            (api.motor_brake)(port as u32);
        }
    }
}

// ── LED matrix ──

fn update_display(
    api: &MonitorApi,
    ports: &[PortInfo; NUM_PORTS],
    imu_n: u32,
    imu_buf: &[u8; 12],
    speed: i32,
    direction: i32,
    excitement: u32,
) {
    let port_to_row: [usize; 6] = [0, 0, 1, 1, 2, 3];
    let port_to_col: [usize; 6] = [0, 3, 0, 3, 0, 0];

    for px in 0u32..20 {
        (api.set_pixel)(px, 0);
    }

    for port in 0..NUM_PORTS {
        let row = port_to_row[port];
        let col = port_to_col[port];
        let base_px = (row * 5 + col) as u32;

        // Per-port delta drives brightness boost
        let delta = port_delta(&ports[port]);
        let boost = (delta * 5).min(50);

        match ports[port].dev {
            DevType::Motor if ports[port].len >= 4 => {
                let pos = i32::from_le_bytes([
                    ports[port].data[0],
                    ports[port].data[1],
                    ports[port].data[2],
                    ports[port].data[3],
                ]);
                (api.set_pixel)(base_px, (60 + boost).min(100));
                let flicker = ((pos.unsigned_abs() / 30) % 2) as u32;
                (api.set_pixel)(base_px + 1, if flicker == 0 { 50 + boost } else { 10 });
            }
            DevType::Ultrasonic if ports[port].len >= 2 => {
                let dist = i16::from_le_bytes([ports[port].data[0], ports[port].data[1]]);
                (api.set_pixel)(base_px, (40 + boost).min(100));
                let bright = if dist >= 0 && dist < 2000 {
                    100u32.saturating_sub((dist as u32).min(1999) * 100 / 2000)
                } else {
                    0
                };
                (api.set_pixel)(base_px + 1, bright);
            }
            DevType::ColorRGBI if ports[port].len >= 8 => {
                let r = u16::from_le_bytes([ports[port].data[0], ports[port].data[1]]);
                let g = u16::from_le_bytes([ports[port].data[2], ports[port].data[3]]);
                let b_val = u16::from_le_bytes([ports[port].data[4], ports[port].data[5]]);
                (api.set_pixel)(base_px, (30 + boost).min(100));
                let scale = |v: u16| -> u32 { (v as u32).min(400) * 100 / 400 };
                (api.set_pixel)(base_px + 1, scale(r));
                (api.set_pixel)(base_px + 2, scale(g));
                if col + 3 < 5 {
                    (api.set_pixel)(base_px + 3, scale(b_val));
                }
            }
            DevType::ColorID if ports[port].len >= 1 => {
                (api.set_pixel)(base_px, (30 + boost).min(100));
                (api.set_pixel)(base_px + 1, (ports[port].data[0] as u32) * 10);
            }
            _ => {}
        }
    }

    // Row 4: IMU + excitement bar
    if imu_n >= 12 {
        let ax = i16::from_le_bytes([imu_buf[0], imu_buf[1]]);
        let ay = i16::from_le_bytes([imu_buf[2], imu_buf[3]]);
        let tilt_x = (ax.unsigned_abs() as u32).min(8000) * 100 / 8000;
        let tilt_y = (ay.unsigned_abs() as u32).min(8000) * 100 / 8000;
        (api.set_pixel)(20, tilt_x);
        (api.set_pixel)(21, tilt_y);
    } else {
        (api.set_pixel)(20, 0);
        (api.set_pixel)(21, 0);
    }

    // Excitement bar: px 22-24
    let ex_level = excitement.min(30);
    (api.set_pixel)(22, if ex_level >= 5 { 50 } else { 0 });
    (api.set_pixel)(23, if ex_level >= 15 { 70 } else { 0 });
    (api.set_pixel)(24, if ex_level >= 25 { 100 } else { 0 });

    (api.update_leds)();
}

// ── Status line ──

fn print_status(
    api: &MonitorApi,
    ports: &[PortInfo; NUM_PORTS],
    imu_n: u32,
    imu_buf: &[u8; 12],
    speed: i32,
    direction: i32,
    cycle: u32,
    excitement: u32,
) {
    api.print(b"t=");
    print_u32(api, cycle * LOOP_MS / 1000);
    api.print(b"s ");

    for port in 0..NUM_PORTS {
        match ports[port].dev {
            DevType::Motor if ports[port].len >= 4 => {
                let pos = i32::from_le_bytes([
                    ports[port].data[0],
                    ports[port].data[1],
                    ports[port].data[2],
                    ports[port].data[3],
                ]);
                api.print(&[PORT_NAMES[port]]);
                api.print(b"=");
                print_i32(api, pos);
                api.print(b"d ");
            }
            DevType::Ultrasonic if ports[port].len >= 2 => {
                let dist = i16::from_le_bytes([ports[port].data[0], ports[port].data[1]]);
                api.print(&[PORT_NAMES[port]]);
                api.print(b"=");
                print_i32(api, dist as i32);
                api.print(b"mm ");
            }
            DevType::ColorRGBI if ports[port].len >= 8 => {
                let r = u16::from_le_bytes([ports[port].data[0], ports[port].data[1]]);
                let g = u16::from_le_bytes([ports[port].data[2], ports[port].data[3]]);
                let b_val = u16::from_le_bytes([ports[port].data[4], ports[port].data[5]]);
                api.print(&[PORT_NAMES[port]]);
                api.print(b"=R");
                print_u32(api, r as u32);
                api.print(b"G");
                print_u32(api, g as u32);
                api.print(b"B");
                print_u32(api, b_val as u32);
                api.print(b" ");
            }
            DevType::ColorID if ports[port].len >= 1 => {
                api.print(&[PORT_NAMES[port]]);
                api.print(b"=c");
                print_u32(api, ports[port].data[0] as u32);
                api.print(b" ");
            }
            _ => {}
        }
    }

    if imu_n >= 12 {
        let ax = i16::from_le_bytes([imu_buf[0], imu_buf[1]]);
        let ay = i16::from_le_bytes([imu_buf[2], imu_buf[3]]);
        let az = i16::from_le_bytes([imu_buf[4], imu_buf[5]]);
        api.print(b"IMU=");
        print_i32(api, ax as i32);
        api.print(b",");
        print_i32(api, ay as i32);
        api.print(b",");
        print_i32(api, az as i32);
        api.print(b" ");
    }

    api.print(b"spd=");
    print_i32(api, speed * direction);
    api.print(b"% ex=");
    print_u32(api, excitement);
    api.print(b"\r\n");
}

// ── Helpers ──

fn clear_matrix(api: &MonitorApi) {
    for i in 0..25u32 {
        (api.set_pixel)(i, 0);
    }
    (api.update_leds)();
}

fn print_i32(api: &MonitorApi, v: i32) {
    if v < 0 {
        api.print(b"-");
        print_u32(api, (-(v as i64)) as u32);
    } else {
        print_u32(api, v as u32);
    }
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
