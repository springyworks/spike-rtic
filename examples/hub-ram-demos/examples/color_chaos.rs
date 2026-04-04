//! color-chaos -- derivative-driven motor madness from RGBI color sensor
//! with concurrent RTTY FSK audio on the hub speaker.
//!
//! Uses MonitorApi v4: `sensor_read` for LUMP data, `sound_play`/`sound_stop`
//! for hardware-driven FSK tones.  The firmware handles LUMP handshake and
//! keepalive; TIM6+DAC play tones with zero CPU cost.
//!
//! Three concurrent streams — all running in one 10 ms loop:
//!   1. **Sensor** — reads RGBI (mode 5) via firmware ring buffer
//!   2. **Motors** — derivative-driven cross-coupled impulses on A+B
//!   3. **RTTY audio** — ITA2 Baudot FSK on the speaker (mark/space tones)
//!
//! RTTY encoding runs as a state machine: each 10 ms frame advances
//! the FSK modulator by one step.  At ~2 frames per bit (20 ms ≈ 50 baud),
//! event keywords like "RED" "GRN" "TWIST" warble out of the speaker
//! while the motors twitch and the LED matrix dances.
//!
//! Motors: port A, port B.  Color sensor: port F.
//! Center button = exit.

#![no_std]
#![no_main]

use spike_hub_api::{MonitorApi, BTN_CENTER};

// Motor ports
const PORT_A: u32 = 0;
const PORT_B: u32 = 1;

// ── FSK parameters (standard RTTY) ──
// 2125 Hz mark / 2295 Hz space — 170 Hz shift, harmonics above 3 kHz radio cutoff.
const MARK_HZ: u32 = 2125;   // logical 1 (mark)
const SPACE_HZ: u32 = 2295;  // logical 0 (space)
// Frames per bit: 10 ms frame × 2 = 20 ms ≈ 50 baud (close to 45.45)
const FRAMES_PER_BIT: u8 = 2;

// ── ITA2 Baudot lookup ──
// Index = ASCII 'A'..'Z' → 5-bit Baudot code.  0 = unmapped.
const BAUDOT_LETTERS: [u8; 26] = [
    3,  // A
    25, // B
    14, // C
    9,  // D
    1,  // E
    13, // F
    26, // G
    20, // H
    6,  // I
    11, // J
    15, // K
    18, // L
    28, // M
    12, // N
    24, // O
    22, // P
    23, // Q
    10, // R
    5,  // S
    16, // T
    7,  // U
    30, // V
    19, // W
    29, // X
    21, // Y
    17, // Z
];
const BAUDOT_SPACE: u8 = 4;
const BAUDOT_LTRS: u8 = 31;

// ── RTTY state machine ──
// Transmits queued bytes as ITA2 Baudot FSK, one bit per FRAMES_PER_BIT.

struct RttyFsk {
    queue: [u8; 32],    // message queue (ASCII uppercase)
    q_len: u8,
    q_pos: u8,          // current character index in queue
    bit_phase: u8,      // 0..6: start(0), data(1-5), stop(6)
    frame_ctr: u8,      // counts frames within current bit
    current_code: u8,   // 5-bit Baudot code being transmitted
    idle_frames: u8,    // frames of silence after message
    active: bool,       // currently transmitting
}

impl RttyFsk {
    const fn new() -> Self {
        RttyFsk {
            queue: [0; 32],
            q_len: 0,
            q_pos: 0,
            bit_phase: 0,
            frame_ctr: 0,
            current_code: 0,
            idle_frames: 0,
            active: false,
        }
    }

    /// Queue a message for RTTY transmission.
    /// If already transmitting, the new message replaces remaining queue.
    fn say(&mut self, msg: &[u8]) {
        let n = msg.len().min(31);
        self.queue[..n].copy_from_slice(&msg[..n]);
        self.q_len = n as u8;
        self.q_pos = 0;
        self.bit_phase = 0;
        self.frame_ctr = 0;
        self.active = true;
        self.idle_frames = 0;
    }

    /// Advance one frame (10 ms).  Returns the FSK frequency to play,
    /// or 0 to silence the speaker.
    fn tick(&mut self) -> u32 {
        if !self.active {
            return 0;
        }

        // Count frames within current bit
        self.frame_ctr += 1;
        if self.frame_ctr < FRAMES_PER_BIT {
            // Hold current tone
            return self.current_tone();
        }
        self.frame_ctr = 0;

        // Advance to next bit
        match self.bit_phase {
            0 => {
                // Start bit = space
                self.load_next_char();
                if !self.active { return 0; }
                self.bit_phase = 1;
                return SPACE_HZ;
            }
            1..=5 => {
                // Data bits, LSB first
                let bit_idx = self.bit_phase - 1;
                let bit = (self.current_code >> bit_idx) & 1;
                self.bit_phase += 1;
                if bit == 1 { MARK_HZ } else { SPACE_HZ }
            }
            6 => {
                // Stop bit = mark (1.5 bit times, we do 2 frames)
                self.bit_phase = 0; // ready for next character
                MARK_HZ
            }
            _ => { self.active = false; 0 }
        }
    }

    fn current_tone(&self) -> u32 {
        if !self.active { return 0; }
        match self.bit_phase {
            0 => SPACE_HZ,
            1..=5 => {
                let bit_idx = self.bit_phase - 1;
                if (self.current_code >> bit_idx) & 1 == 1 { MARK_HZ } else { SPACE_HZ }
            }
            _ => MARK_HZ,
        }
    }

    fn load_next_char(&mut self) {
        if self.q_pos >= self.q_len {
            // Tail: silence after a short mark
            self.idle_frames += 1;
            if self.idle_frames > 4 {
                self.active = false;
            }
            self.current_code = BAUDOT_LTRS; // idle = LTRS
            return;
        }
        let ch = self.queue[self.q_pos as usize];
        self.q_pos += 1;
        self.current_code = match ch {
            b'A'..=b'Z' => BAUDOT_LETTERS[(ch - b'A') as usize],
            b'a'..=b'z' => BAUDOT_LETTERS[(ch - b'a') as usize],
            b' ' => BAUDOT_SPACE,
            _ => BAUDOT_SPACE,
        };
    }
}

// ── RGBI state ──

struct Rgbi {
    r: i16,
    g: i16,
    b: i16,
    i: i16,
}

impl Rgbi {
    const fn zero() -> Self { Rgbi { r: 0, g: 0, b: 0, i: 0 } }

    fn from_buf(buf: &[u8]) -> Self {
        if buf.len() >= 8 {
            Rgbi {
                r: i16::from_le_bytes([buf[0], buf[1]]),
                g: i16::from_le_bytes([buf[2], buf[3]]),
                b: i16::from_le_bytes([buf[4], buf[5]]),
                i: i16::from_le_bytes([buf[6], buf[7]]),
            }
        } else {
            Rgbi::zero()
        }
    }
}

// ══════════════════════════════════════════════════════════
// Entry
// ══════════════════════════════════════════════════════════

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 4 {
        api.print(b"ERR: need API v4+ (sensor + sound)\r\n");
        return 1;
    }

    api.print(b"=== Color Chaos v3 (RTTY + motors + sensor) ===\r\n");
    api.print(b"  Sensor: firmware LUMP  |  Speaker: FSK Baudot\r\n");
    api.print(b"  Motors: A + B          |  Exit: center button\r\n\r\n");

    // Request mode 5 (RGBI) through the API
    (api.sensor_mode)(5);

    // Startup confirmation: mark then space so user hears both RTTY tones
    (api.sound_play)(MARK_HZ);
    (api.delay_ms)(150);
    (api.sound_stop)();
    (api.delay_ms)(50);
    (api.sound_play)(SPACE_HZ);
    (api.delay_ms)(150);
    (api.sound_stop)();
    (api.delay_ms)(50);

    // Lead-in: mark tone for ~200 ms (sync preamble)
    // Interleave sensor_read calls to maintain LUMP keepalives
    (api.sound_play)(MARK_HZ);
    let mut warmup = [0u8; 32];
    for _ in 0..4 {
        (api.delay_ms)(50);
        let _ = (api.sensor_read)(warmup.as_mut_ptr(), 32);
    }

    api.print(b"Running -- hold color near sensor!\r\n\r\n");

    let mut prev = Rgbi::zero();
    let mut speed_a: i32 = 0;
    let mut speed_b: i32 = 0;
    let mut prev_speed_a: i32 = 0;
    let mut prev_speed_b: i32 = 0;
    let mut frame: u32 = 0;
    let mut idle_frames: u32 = 0;
    let mut sensor_buf = [0u8; 32];
    let mut rtty = RttyFsk::new();

    // Initial RTTY announcement
    rtty.say(b"COLOR CHAOS");

    loop {
        // ── Exit: firmware injects BTN_CENTER on abort ──
        let btns = (api.read_buttons)();
        if btns & BTN_CENTER != 0 {
            api.print(b"\r\nAbort -- stopping.\r\n");
            break;
        }

        // ── Read sensor data via firmware API ──
        let n = (api.sensor_read)(sensor_buf.as_mut_ptr(), 32);
        let cur = if n >= 8 {
            Rgbi::from_buf(&sensor_buf[..n as usize])
        } else {
            Rgbi { r: prev.r, g: prev.g, b: prev.b, i: prev.i }
        };

        // Diagnostic: first frame shows raw byte count
        if frame == 0 {
            api.print(b"  sensor_read returned ");
            print_num(api, n);
            api.print(b" bytes: [");
            for i in 0..(n as usize).min(12) {
                if i > 0 { api.print(b" "); }
                print_hex8(api, sensor_buf[i]);
            }
            api.print(b"]\r\n");
        }

        // ── Compute derivatives ──
        let dr = (cur.r as i32) - (prev.r as i32);
        let dg = (cur.g as i32) - (prev.g as i32);
        let db = (cur.b as i32) - (prev.b as i32);
        let di = (cur.i as i32) - (prev.i as i32);

        prev.r = cur.r;
        prev.g = cur.g;
        prev.b = cur.b;
        prev.i = cur.i;

        // ── Map derivatives to motor impulses ──
        speed_a += dr * 2;
        speed_b += dg * 2;
        speed_a -= db * 2;
        speed_a += di;
        speed_b -= di;

        // ── Decay 15% each frame ──
        speed_a = speed_a * 85 / 100;
        speed_b = speed_b * 85 / 100;
        speed_a = clamp(speed_a, -100, 100);
        speed_b = clamp(speed_b, -100, 100);

        // ── Baseline heartbeat oscillation so motors always move ──
        // Slow sine-like wobble: ±15 on A, ±15 on B (phase-shifted)
        let wobble_a = match (frame / 25) % 4 {
            0 => 15i32,
            1 => 0,
            2 => -15,
            _ => 0,
        };
        let wobble_b = match ((frame + 12) / 25) % 4 {
            0 => 15i32,
            1 => 0,
            2 => -15,
            _ => 0,
        };
        let drive_a = clamp(speed_a + wobble_a, -100, 100);
        let drive_b = clamp(speed_b + wobble_b, -100, 100);

        // ── Drive motors ──
        (api.motor_set)(PORT_A, drive_a);
        (api.motor_set)(PORT_B, drive_b);

        // ── LED matrix ──
        show_bars(api, &cur, speed_a, speed_b);

        // ── Event detection + CDC narration + RTTY queueing ──
        let abs_dr = abs_i32(dr);
        let abs_dg = abs_i32(dg);
        let abs_db = abs_i32(db);
        let abs_di = abs_i32(di);
        let mut spoke = false;

        if abs_dr > 1 {
            if dr > 0 { api.print(b"RED SPIKE +"); } else { api.print(b"RED DROP  -"); }
            print_num(api, abs_dr as u32);
            api.print(b"  A JOLT-> ");
            print_i32(api, speed_a);
            api.print(b"\r\n");
            if !rtty.active { rtty.say(b"RED"); }
            spoke = true;
        }

        if abs_dg > 1 {
            if dg > 0 { api.print(b"GRN SPIKE +"); } else { api.print(b"GRN DROP  -"); }
            print_num(api, abs_dg as u32);
            api.print(b"  B JOLT-> ");
            print_i32(api, speed_b);
            api.print(b"\r\n");
            if !rtty.active { rtty.say(b"GRN"); }
            spoke = true;
        }

        if abs_db > 1 {
            if db > 0 { api.print(b"BLU SURGE +"); } else { api.print(b"BLU FADE  -"); }
            print_num(api, abs_db as u32);
            api.print(b"  A LURCH<- ");
            print_i32(api, speed_a);
            api.print(b"\r\n");
            if !rtty.active { rtty.say(b"BLU"); }
            spoke = true;
        }

        if abs_di > 2 {
            api.print(b"TWIST dI=");
            print_i32(api, di);
            api.print(b"  A<>B ");
            print_i32(api, speed_a);
            api.print(b"/");
            print_i32(api, speed_b);
            api.print(b"\r\n");
            if !rtty.active { rtty.say(b"TWIST"); }
            spoke = true;
        }

        if (prev_speed_a > 5 && speed_a < -5) || (prev_speed_a < -5 && speed_a > 5) {
            api.print(b"~~ A FLIP ");
            print_i32(api, prev_speed_a);
            api.print(b"->");
            print_i32(api, speed_a);
            api.print(b"\r\n");
            if !rtty.active { rtty.say(b"FLIP"); }
            spoke = true;
        }
        if (prev_speed_b > 5 && speed_b < -5) || (prev_speed_b < -5 && speed_b > 5) {
            api.print(b"~~ B FLIP ");
            print_i32(api, prev_speed_b);
            api.print(b"->");
            print_i32(api, speed_b);
            api.print(b"\r\n");
            spoke = true;
        }

        prev_speed_a = speed_a;
        prev_speed_b = speed_b;

        if spoke { idle_frames = 0; } else { idle_frames += 1; }

        // Heartbeat every ~1 second of quiet
        if idle_frames > 0 && idle_frames % 100 == 0 {
            api.print(b". n=");
            print_num(api, n);
            api.print(b" R=");
            print_i16(api, cur.r);
            api.print(b" G=");
            print_i16(api, cur.g);
            api.print(b" B=");
            print_i16(api, cur.b);
            api.print(b" I=");
            print_i16(api, cur.i);
            api.print(b"  A=");
            print_i32(api, speed_a);
            api.print(b" B=");
            print_i32(api, speed_b);
            api.print(b"\r\n");
            if !rtty.active { rtty.say(b"QRV"); } // ham radio "ready"
        }

        // ── RTTY FSK audio — advance one frame ──
        let tone = rtty.tick();
        if tone > 0 {
            (api.sound_play)(tone);
        } else {
            (api.sound_stop)();
        }

        frame = frame.wrapping_add(1);
        (api.delay_ms)(10);
    }

    // ── Cleanup ──
    (api.sound_stop)();
    (api.motor_set)(PORT_A, 0);
    (api.motor_set)(PORT_B, 0);
    matrix_fill(api, 0);

    api.print(b"Done. frames=");
    print_num(api, frame);
    api.print(b"\r\n");
    0
}

// ══════════════════════════════════════════════════════════
// LED display
// ══════════════════════════════════════════════════════════

/// Show RGBI as 4 horizontal bar graphs + motor bar on row 5.
fn show_bars(api: &MonitorApi, c: &Rgbi, sa: i32, sb: i32) {
    for p in 0..25u32 { (api.set_pixel)(p, 0); }

    let vals = [c.r, c.g, c.b, c.i];
    for (row, &v) in vals.iter().enumerate() {
        let lit = if v <= 0 { 0 } else { ((v as u32) * 5 / 1024).min(5) };
        let bri: u32 = match row {
            0 => 80,
            1 => 50,
            2 => 70,
            _ => 40,
        };
        for col in 0..lit {
            (api.set_pixel)((row as u32) * 5 + col, bri);
        }
    }

    let a_pix = abs_i32(sa) as u32 / 40;
    let b_pix = abs_i32(sb) as u32 / 40;
    (api.set_pixel)(22, 20);
    if a_pix >= 1 { (api.set_pixel)(21, 60); }
    if a_pix >= 2 { (api.set_pixel)(20, 90); }
    if b_pix >= 1 { (api.set_pixel)(23, 60); }
    if b_pix >= 2 { (api.set_pixel)(24, 90); }

    (api.update_leds)();
}

fn matrix_fill(api: &MonitorApi, bri: u32) {
    for p in 0..25u32 { (api.set_pixel)(p, bri); }
    (api.update_leds)();
}

// ══════════════════════════════════════════════════════════
// Helpers
// ══════════════════════════════════════════════════════════

fn clamp(v: i32, lo: i32, hi: i32) -> i32 {
    if v < lo { lo } else if v > hi { hi } else { v }
}

fn abs_i32(v: i32) -> i32 {
    if v < 0 { -v } else { v }
}

fn print_num(api: &MonitorApi, val: u32) {
    let mut buf = [0u8; 10];
    let s = fmt_u32(val, &mut buf);
    api.print(s);
}

fn print_i32(api: &MonitorApi, val: i32) {
    if val < 0 {
        api.print(b"-");
        print_num(api, (-(val as i64)) as u32);
    } else {
        print_num(api, val as u32);
    }
}

fn print_i16(api: &MonitorApi, val: i16) {
    print_i32(api, val as i32);
}

fn print_hex8(api: &MonitorApi, val: u8) {
    const HEX: &[u8] = b"0123456789ABCDEF";
    let buf = [HEX[(val >> 4) as usize], HEX[(val & 0xF) as usize]];
    api.print(&buf);
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
