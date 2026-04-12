//! **Real RTIC-sync primitives** — Channel, Signal, Watch on 5×5 LEDs.
//!
//! This privileged demo uses the **actual `rtic-sync` crate** — the same
//! channel, signal, and watch types used inside real RTIC v2 applications.
//! No MCU register access: only MonitorApi callbacks for LEDs and timing.
//!
//! ## Architecture
//!
//! A cooperative scheduler drives 20 "tasks" (LEDs 0-24 minus heartbeat),
//! each backed by real `rtic_sync::channel::Channel` instances with
//! `try_send` / `try_recv`, plus `rtic_sync::signal::Signal` for broadcast.
//!
//! | RTIC primitive used  | What it does here                            |
//! |----------------------|----------------------------------------------|
//! | `Channel<u8, 4>`     | MPSC message passing — producers push, one   |
//! |                      | consumer per LED drains and lights up         |
//! | `Signal<u8>`         | Broadcast "priority event" to all watchers    |
//! | `try_send` / `try_recv` | non-blocking channel ops (real rtic-sync) |
//! | `SignalWriter::write` / `SignalReader::try_read` | real signal ops |
//! | `Channel::is_empty/full` | queue introspection                      |
//!
//! ## LED layout (5×5, pixel 12 = heartbeat reserved)
//!
//! ```text
//!  Row 0:  [P0  P1  P2  P3  P4 ]   ← Pipeline: producer→consumer chain
//!  Row 1:  [G0  G1  G2  G3  G4 ]   ← Generators: random msg sources
//!  Row 2:  [F0  F1  ♥   F3  F4 ]   ← Fan-out: signal broadcast targets
//!  Row 3:  [R0  R1  R2  R3  R4 ]   ← Ring: messages travel in a circle
//!  Row 4:  [S0  S1  S2  S3  S4 ]   ← Stats: queue fill levels
//! ```
//!
//! ## Scenarios playing simultaneously
//!
//! 1. **Pipeline** (row 0): 5-stage producer→consumer chain via channels.
//!    P0 generates, sends to P1's channel, P1 forwards to P2, etc.
//!    Each LED brightness = message value received.
//!
//! 2. **Random generators** (row 1): Each LED produces random messages
//!    into the pipeline or ring at random intervals.
//!
//! 3. **Signal fan-out** (row 2): A Signal broadcasts a value; all
//!    targets light up simultaneously when a new value arrives.
//!
//! 4. **Ring buffer** (row 3): Messages circulate through a 5-node ring
//!    of channels. LED brightness tracks the message value as it orbits.
//!
//! 5. **Queue stats** (row 4): LED brightness = fill level of each
//!    pipeline channel (bright = full, dim = empty).
//!
//! ## Controls
//!
//! | Input      | Action                                    |
//! |------------|-------------------------------------------|
//! | center btn | exit                                      |
//! | left btn   | inject burst of messages into pipeline    |
//! | right btn  | trigger signal broadcast                  |
//! | `send q`   | quit                                      |
//! | `send b`   | burst: flood all channels                 |
//! | `send s`   | trigger signal event                      |
//! | `send d`   | drain: empty all channels                 |
//! | `send h`   | help                                      |
//!
//! Requires: API v12+, privileged (`go!`).

#![no_std]
#![no_main]

use spike_hub_api::{
    MonitorApi, BTN_CENTER, BTN_LEFT, BTN_RIGHT, PRIV_MAGIC,
    EVT_BUTTON, EVT_INPUT, EVT_TIMEOUT,
};
use rtic_sync::channel::Channel;
use rtic_sync::signal::Signal;

// Force-link cortex-m's critical-section implementation
use cortex_m as _;

// ── Privileged marker ──────────────────────────────────────

#[link_section = ".demo_header"]
#[used]
static _PRIV_MARKER: u32 = PRIV_MAGIC;

// ── Xorshift32 RNG ────────────────────────────────────────

struct Rng(u32);

impl Rng {
    fn new(seed: u32) -> Self {
        Self(if seed == 0 { 0xDEAD_BEEF } else { seed })
    }
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }
    fn range(&mut self, lo: u32, hi: u32) -> u32 {
        lo + (self.next() % (hi - lo + 1))
    }
}

// ── LED pixel map ──────────────────────────────────────────
// We use 24 pixels (0-24 except 12 which is heartbeat).

const HEARTBEAT: u32 = 12;
const PIPELINE: [u32; 5] = [0, 1, 2, 3, 4];       // row 0
const GENERATORS: [u32; 5] = [5, 6, 7, 8, 9];      // row 1
const FANOUT: [u32; 4] = [10, 11, 13, 14];          // row 2 minus heartbeat
const RING: [u32; 5] = [15, 16, 17, 18, 19];        // row 3
const STATS: [u32; 5] = [20, 21, 22, 23, 24];       // row 4

// ── Number formatting ──────────────────────────────────────

fn fmt_u32(buf: &mut [u8], mut val: u32) -> usize {
    if val == 0 {
        if !buf.is_empty() { buf[0] = b'0'; }
        return 1;
    }
    let mut tmp = [0u8; 10];
    let mut i = 0;
    while val > 0 {
        tmp[i] = b'0' + (val % 10) as u8;
        val /= 10;
        i += 1;
    }
    let len = i.min(buf.len());
    for j in 0..len {
        buf[j] = tmp[i - 1 - j];
    }
    len
}

// ── Main entry ─────────────────────────────────────────────

#[no_mangle]
pub extern "C" fn _start(api: *const MonitorApi) -> i32 {
    let api = unsafe { &*api };

    // API version check
    if api.version < 12 {
        api.print(b"Need API v12+\r\n");
        return -1;
    }

    api.print(b"\r\n=== RTIC-sync Channels Demo ===\r\n");
    api.print(b"Real rtic_sync::channel::Channel + Signal\r\n");
    api.print(b"send h = help, send q = quit\r\n\r\n");

    // ── Create RTIC-sync channels (the real deal!) ─────────
    //
    // Pipeline: 4 channels connecting 5 stages (P0→P1→P2→P3→P4)
    let mut pipe0 = Channel::<u8, 4>::new();
    let mut pipe1 = Channel::<u8, 4>::new();
    let mut pipe2 = Channel::<u8, 4>::new();
    let mut pipe3 = Channel::<u8, 4>::new();

    // Ring: 5 channels forming a circular buffer
    let mut ring0 = Channel::<u8, 4>::new();
    let mut ring1 = Channel::<u8, 4>::new();
    let mut ring2 = Channel::<u8, 4>::new();
    let mut ring3 = Channel::<u8, 4>::new();
    let mut ring4 = Channel::<u8, 4>::new();

    // Signal: broadcast to fan-out row
    let signal = Signal::<u8>::new();

    // Split into senders/receivers
    let (mut pipe0_tx, mut pipe0_rx) = pipe0.split();
    let (mut pipe1_tx, mut pipe1_rx) = pipe1.split();
    let (mut pipe2_tx, mut pipe2_rx) = pipe2.split();
    let (mut pipe3_tx, mut pipe3_rx) = pipe3.split();

    let (mut ring0_tx, mut ring0_rx) = ring0.split();
    let (mut ring1_tx, mut ring1_rx) = ring1.split();
    let (mut ring2_tx, mut ring2_rx) = ring2.split();
    let (mut ring3_tx, mut ring3_rx) = ring3.split();
    let (mut ring4_tx, mut ring4_rx) = ring4.split();

    let (mut sig_writer, mut sig_reader) = signal.split();

    let mut rng = Rng::new(0xCAFE_1337);

    // Pipeline generator value (what P0 produces)
    let mut pipe_gen: u8 = 10;
    // Ring seed value
    let mut ring_val: u8 = 50;

    // Per-LED brightness state
    let mut led_bright = [0u32; 25];

    // Tick counter
    let mut tick: u32 = 0;

    // Generator timers (row 1) — each fires at a different rate
    let mut gen_timers: [u32; 5] = [0; 5];
    let mut gen_periods: [u32; 5] = [
        rng.range(3, 8),
        rng.range(5, 12),
        rng.range(4, 10),
        rng.range(6, 15),
        rng.range(3, 7),
    ];

    // Signal auto-fire period
    let mut sig_period = rng.range(20, 40);
    let mut sig_timer: u32 = 0;

    // Stats for status line
    let mut total_sent: u32 = 0;
    let mut total_recv: u32 = 0;
    let mut total_signals: u32 = 0;

    // ── Cooperative scheduler loop ─────────────────────────
    loop {
        tick += 1;

        // ── 1. Pipeline stage P0: generate ─────────────────
        if tick % 4 == 0 {
            pipe_gen = pipe_gen.wrapping_add(7);
            if pipe0_tx.try_send(pipe_gen).is_ok() {
                led_bright[PIPELINE[0] as usize] = (pipe_gen as u32 * 100) / 255;
                total_sent += 1;
            }
        }

        // ── 2. Pipeline forwarding P0→P1→P2→P3→P4 ─────────
        if let Ok(v) = pipe0_rx.try_recv() {
            led_bright[PIPELINE[1] as usize] = (v as u32 * 100) / 255;
            let _ = pipe1_tx.try_send(v.wrapping_add(3));
            total_recv += 1;
            total_sent += 1;
        }
        if let Ok(v) = pipe1_rx.try_recv() {
            led_bright[PIPELINE[2] as usize] = (v as u32 * 100) / 255;
            let _ = pipe2_tx.try_send(v.wrapping_add(3));
            total_recv += 1;
            total_sent += 1;
        }
        if let Ok(v) = pipe2_rx.try_recv() {
            led_bright[PIPELINE[3] as usize] = (v as u32 * 100) / 255;
            let _ = pipe3_tx.try_send(v.wrapping_add(3));
            total_recv += 1;
            total_sent += 1;
        }
        if let Ok(v) = pipe3_rx.try_recv() {
            led_bright[PIPELINE[4] as usize] = (v as u32 * 100) / 255;
            total_recv += 1;
        }

        // ── 3. Ring circulation ────────────────────────────
        // Inject into ring periodically
        if tick % 6 == 0 {
            ring_val = ring_val.wrapping_add(13);
            let _ = ring0_tx.try_send(ring_val);
            total_sent += 1;
        }

        // Forward around the ring: 0→1→2→3→4→0
        if let Ok(v) = ring0_rx.try_recv() {
            led_bright[RING[0] as usize] = (v as u32 * 100) / 255;
            let _ = ring1_tx.try_send(v.wrapping_sub(5));
            total_recv += 1;
            total_sent += 1;
        }
        if let Ok(v) = ring1_rx.try_recv() {
            led_bright[RING[1] as usize] = (v as u32 * 100) / 255;
            let _ = ring2_tx.try_send(v.wrapping_sub(5));
            total_recv += 1;
            total_sent += 1;
        }
        if let Ok(v) = ring2_rx.try_recv() {
            led_bright[RING[2] as usize] = (v as u32 * 100) / 255;
            let _ = ring3_tx.try_send(v.wrapping_sub(5));
            total_recv += 1;
            total_sent += 1;
        }
        if let Ok(v) = ring3_rx.try_recv() {
            led_bright[RING[3] as usize] = (v as u32 * 100) / 255;
            let _ = ring4_tx.try_send(v.wrapping_sub(5));
            total_recv += 1;
            total_sent += 1;
        }
        if let Ok(v) = ring4_rx.try_recv() {
            led_bright[RING[4] as usize] = (v as u32 * 100) / 255;
            // Complete the ring: send back to ring0
            let _ = ring0_tx.try_send(v.wrapping_sub(5));
            total_recv += 1;
            total_sent += 1;
        }

        // ── 4. Random generators (row 1) ───────────────────
        for i in 0..5 {
            gen_timers[i] += 1;
            if gen_timers[i] >= gen_periods[i] {
                gen_timers[i] = 0;
                gen_periods[i] = rng.range(3, 12);
                let val = rng.range(20, 100) as u8;
                led_bright[GENERATORS[i] as usize] = val as u32;

                // Generators inject into pipeline or ring randomly
                match i {
                    0 | 1 => { let _ = pipe0_tx.try_send(val); total_sent += 1; }
                    2 => { let _ = ring0_tx.try_send(val); total_sent += 1; }
                    3 | 4 => { let _ = ring0_tx.try_send(val); total_sent += 1; }
                    _ => {}
                }
            }
        }

        // ── 5. Signal broadcast (row 2 fan-out) ────────────
        sig_timer += 1;
        if sig_timer >= sig_period {
            sig_timer = 0;
            sig_period = rng.range(15, 35);
            let val = rng.range(40, 100) as u8;
            sig_writer.write(val);
            total_signals += 1;
        }

        // All fan-out LEDs read the signal
        if let Some(v) = sig_reader.try_read() {
            for &px in &FANOUT {
                led_bright[px as usize] = v as u32;
            }
        }

        // ── 6. Stats row (row 4) — channel fill levels ────
        // Show whether each pipeline channel is empty/full
        led_bright[STATS[0] as usize] = if pipe0_tx.is_full() { 100 } else if pipe0_tx.is_empty() { 0 } else { 50 };
        led_bright[STATS[1] as usize] = if pipe1_tx.is_full() { 100 } else if pipe1_tx.is_empty() { 0 } else { 50 };
        led_bright[STATS[2] as usize] = if pipe2_tx.is_full() { 100 } else if pipe2_tx.is_empty() { 0 } else { 50 };
        led_bright[STATS[3] as usize] = if pipe3_tx.is_full() { 100 } else if pipe3_tx.is_empty() { 0 } else { 50 };
        // Last stat LED: total message flow indicator
        led_bright[STATS[4] as usize] = ((total_sent + total_recv) % 101) as u32;

        // ── 7. Decay all LEDs slightly ─────────────────────
        for i in 0..25 {
            if i == HEARTBEAT as usize { continue; }
            if led_bright[i] > 2 {
                led_bright[i] -= 2;
            } else {
                led_bright[i] = 0;
            }
        }

        // ── 8. Update physical LEDs ────────────────────────
        for i in 0..25u32 {
            if i == HEARTBEAT { continue; }
            (api.set_pixel)(i, led_bright[i as usize]);
        }
        (api.update_leds)();

        // ── 9. Status output every 200 ticks ───────────────
        if tick % 200 == 0 {
            let mut buf = [0u8; 80];
            let mut pos = 0;

            // "[rtic-sync] t=NNN sent=NNN recv=NNN sig=NNN"
            let prefix = b"[rtic-sync] t=";
            buf[pos..pos + prefix.len()].copy_from_slice(prefix);
            pos += prefix.len();
            pos += fmt_u32(&mut buf[pos..], tick);
            buf[pos..pos + 6].copy_from_slice(b" sent=");
            pos += 6;
            pos += fmt_u32(&mut buf[pos..], total_sent);
            buf[pos..pos + 6].copy_from_slice(b" recv=");
            pos += 6;
            pos += fmt_u32(&mut buf[pos..], total_recv);
            buf[pos..pos + 5].copy_from_slice(b" sig=");
            pos += 5;
            pos += fmt_u32(&mut buf[pos..], total_signals);
            buf[pos..pos + 2].copy_from_slice(b"\r\n");
            pos += 2;

            api.print(&buf[..pos]);
        }

        // ── 10. Input handling ─────────────────────────────
        let evt = (api.wait_event)(EVT_BUTTON | EVT_INPUT | EVT_TIMEOUT, 30);

        // Check buttons
        if evt & EVT_BUTTON != 0 {
            let btns = (api.read_buttons)();
            if btns & BTN_CENTER != 0 {
                api.print(b"[rtic-sync] center -> quit\r\n");
                (api.sound_stop)();
                return 0;
            }
            if btns & BTN_LEFT != 0 {
                // Burst: flood pipeline
                api.print(b"[rtic-sync] burst into pipeline\r\n");
                for v in 0..16u8 {
                    let _ = pipe0_tx.try_send(v.wrapping_mul(17));
                    total_sent += 1;
                }
                (api.sound_play)(880);
                (api.delay_ms)(50);
                (api.sound_stop)();
            }
            if btns & BTN_RIGHT != 0 {
                // Signal broadcast
                api.print(b"[rtic-sync] signal broadcast\r\n");
                sig_writer.write(100);
                total_signals += 1;
                (api.sound_play)(660);
                (api.delay_ms)(50);
                (api.sound_stop)();
            }
        }

        // Check serial input
        let mut inbuf = [0u8; 16];
        let n = (api.read_input)(inbuf.as_mut_ptr(), inbuf.len() as u32);
        if n > 0 {
            match inbuf[0] {
                b'q' | b'Q' => {
                    api.print(b"[rtic-sync] quit\r\n");
                    (api.sound_stop)();
                    return 0;
                }
                b'h' | b'H' => {
                    api.print(b"  b=burst  s=signal  d=drain  q=quit\r\n");
                }
                b'b' | b'B' => {
                    // Burst all channels
                    api.print(b"[rtic-sync] burst\r\n");
                    for v in 0..16u8 {
                        let _ = pipe0_tx.try_send(v.wrapping_mul(11));
                        let _ = ring0_tx.try_send(v.wrapping_mul(13));
                        total_sent += 2;
                    }
                }
                b's' | b'S' => {
                    // Trigger signal
                    api.print(b"[rtic-sync] signal!\r\n");
                    sig_writer.write(100);
                    total_signals += 1;
                }
                b'd' | b'D' => {
                    // Drain all channels
                    api.print(b"[rtic-sync] drain\r\n");
                    while pipe0_rx.try_recv().is_ok() { total_recv += 1; }
                    while pipe1_rx.try_recv().is_ok() { total_recv += 1; }
                    while pipe2_rx.try_recv().is_ok() { total_recv += 1; }
                    while pipe3_rx.try_recv().is_ok() { total_recv += 1; }
                    while ring0_rx.try_recv().is_ok() { total_recv += 1; }
                    while ring1_rx.try_recv().is_ok() { total_recv += 1; }
                    while ring2_rx.try_recv().is_ok() { total_recv += 1; }
                    while ring3_rx.try_recv().is_ok() { total_recv += 1; }
                    while ring4_rx.try_recv().is_ok() { total_recv += 1; }
                    // Clear all LEDs
                    for i in 0..25u32 {
                        if i == HEARTBEAT { continue; }
                        led_bright[i as usize] = 0;
                        (api.set_pixel)(i, 0);
                    }
                    (api.update_leds)();
                }
                _ => {}
            }
        }
    }
}

// ── Panic handler ──────────────────────────────────────────

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
