//! RTIC Async Tasks Visualizer — 24 concurrent "tasks" on 5×5 LEDs.
//!
//! Each of the 24 LEDs (pixel 12 = heartbeat, reserved) represents a
//! simulated RTIC async task.  Tasks have random periods, can spawn
//! other tasks, send messages through channels, and preempt each other
//! — all visualized as LED brightness pulses and cascading flashes.
//!
//! ## What it demonstrates
//!
//! This demo visualizes **core RTIC v2 async concepts** on bare LEDs:
//!
//! | RTIC concept         | Visualization                                   |
//! |----------------------|-------------------------------------------------|
//! | **async task**       | Each LED = one task, independent random period   |
//! | **spawn()**          | Task fires → connected neighbor flashes (chain)  |
//! | **Mono::delay**      | Each task sleeps its own random interval          |
//! | **channel send/recv**| "Messages" ripple outward from sender LED         |
//! | **priority preempt** | High-pri flash (full brightness) overrides others |
//! | **shared resource**  | Center-cross LEDs synchronize brightness          |
//! | **lock()**           | Brief all-dim moment = critical section           |
//!
//! ## LED behaviors
//!
//! - **Pulse**: task wakes, does work (bright), then sleeps (dim→off)
//! - **Cascade**: one task spawns neighbors → ripple effect
//! - **Message wave**: random "channel message" radiates from a pixel
//! - **Priority flash**: every ~5s, all LEDs do a priority-inversion
//!   visualization: low-pri dims, high-pri blazes, then resolves
//! - **Lock flash**: periodic brief all-off = simulated critical section
//!
//! ## Controls
//!
//! | Key/Button | Action                                  |
//! |------------|-----------------------------------------|
//! | center btn | exit                                    |
//! | left btn   | trigger spawn cascade from left column  |
//! | right btn  | trigger spawn cascade from right column |
//! | `send h`   | help                                    |
//! | `send s`   | trigger spawn cascade from random pixel |
//! | `send m`   | trigger message wave from center        |
//! | `send p`   | trigger priority event                  |
//! | `send f`   | toggle fast/slow speed                  |
//! | `send q`   | quit                                    |
//!
//! Requires: API v12+, no external devices.  Non-privileged (`go`).

#![no_std]
#![no_main]

use spike_hub_api::{
    MonitorApi, BTN_CENTER, BTN_LEFT, BTN_RIGHT,
    EVT_BUTTON, EVT_INPUT, EVT_TIMEOUT,
};

// ── Xorshift32 RNG ────────────────────────────────────────

struct Rng(u32);

impl Rng {
    fn new(seed: u32) -> Self {
        Rng(if seed == 0 { 0xDEAD_BEEF } else { seed })
    }
    fn next(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }
    fn range(&mut self, n: u32) -> u32 {
        self.next() % n
    }
}

// ── Constants ──────────────────────────────────────────────

const HEARTBEAT: usize = 12; // firmware heartbeat — never touch
const N_TASKS: usize = 24;   // one per LED (skip pixel 12)
const TICK_MS: u32 = 50;     // simulation tick

/// Map task index (0–23) to pixel index (0–24, skipping 12)
const fn task_to_pixel(t: usize) -> u32 {
    if t < 12 { t as u32 } else { (t + 1) as u32 }
}

/// Map pixel index to task index (inverse)
const fn pixel_to_task(p: usize) -> usize {
    if p < 12 { p } else if p > 12 { p - 1 } else { 0 /* shouldn't happen */ }
}

/// Neighbors for cascade/spawn (indices are task indices, not pixels)
/// Each task can "spawn" up to 4 neighbors (N, S, E, W on the grid)
fn neighbors(t: usize) -> [Option<usize>; 4] {
    let p = task_to_pixel(t) as usize;
    let row = p / 5;
    let col = p % 5;
    let mut n = [None; 4];
    // North
    if row > 0 {
        let np = (row - 1) * 5 + col;
        if np != HEARTBEAT { n[0] = Some(pixel_to_task(np)); }
    }
    // South
    if row < 4 {
        let np = (row + 1) * 5 + col;
        if np != HEARTBEAT { n[1] = Some(pixel_to_task(np)); }
    }
    // West
    if col > 0 {
        let np = row * 5 + col - 1;
        if np != HEARTBEAT { n[2] = Some(pixel_to_task(np)); }
    }
    // East
    if col < 4 {
        let np = row * 5 + col + 1;
        if np != HEARTBEAT { n[3] = Some(pixel_to_task(np)); }
    }
    n
}

// ── Task states ────────────────────────────────────────────

/// Simulated async task state
#[derive(Clone, Copy)]
struct Task {
    /// Ticks remaining until this task "wakes"
    sleep_remain: u16,
    /// Base period for this task (random, set at init)
    period: u16,
    /// Current brightness (fades down each tick)
    brightness: u8,
    /// Priority level 1–3 (affects brightness and preemption)
    priority: u8,
    /// "Channel message" propagation: >0 means this pixel is
    /// part of an active ripple wave, counting down
    ripple: u8,
    /// Cascade spawn pending (ticks until neighbors fire)
    cascade: u8,
}

impl Task {
    fn new(rng: &mut Rng) -> Self {
        Task {
            sleep_remain: rng.range(40) as u16 + 5,
            period: rng.range(30) as u16 + 8, // 8–37 ticks = 400ms–1.85s
            brightness: 0,
            priority: (rng.range(3) as u8) + 1, // 1, 2, or 3
            ripple: 0,
            cascade: 0,
        }
    }
}

// ── Event types for the visualizer ─────────────────────────

/// Global effects
#[derive(Clone, Copy, PartialEq)]
enum GlobalEffect {
    None,
    /// Priority inversion visualization (ticks remaining)
    PriorityEvent(u8),
    /// Critical section / lock visualization (ticks remaining)
    LockFlash(u8),
    /// Message wave radiating from pixel (center tick, radius)
    MessageWave { origin: usize, radius: u8, ticks: u8 },
}

// ── Main ───────────────────────────────────────────────────

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 12 {
        api.print(b"ERR: need API v12+\r\n");
        return 1;
    }

    // Seed from ADC noise
    let seed = (api.read_adc)(8) ^ ((api.read_adc)(11) << 12)
             ^ ((api.read_adc)(1) << 4);
    let mut rng = Rng::new(seed);

    // Initialize 24 tasks with random periods
    let mut tasks = [Task::new(&mut Rng::new(0)); N_TASKS];
    for i in 0..N_TASKS {
        tasks[i] = Task::new(&mut rng);
        // Stagger start times so they don't all fire at once
        tasks[i].sleep_remain = rng.range(tasks[i].period as u32) as u16 + 1;
    }

    let mut fast_mode = false;
    let mut global = GlobalEffect::None;
    let mut tick_count: u32 = 0;
    let mut auto_cascade_timer: u16 = 0; // auto-trigger spawn cascades
    let mut auto_msg_timer: u16 = 0;     // auto-trigger message waves
    let mut auto_pri_timer: u16 = 0;     // auto-trigger priority events
    let mut stats_spawns: u32 = 0;
    let mut stats_messages: u32 = 0;

    // Print banner
    api.print(b"\r\n");
    api.print(b"=========================================\r\n");
    api.print(b"  RTIC ASYNC TASK VISUALIZER  -- 24 LEDs\r\n");
    api.print(b"=========================================\r\n");
    api.print(b"  Each LED = one async task\r\n");
    api.print(b"  Pulse = task wakes & runs\r\n");
    api.print(b"  Cascade = spawn() chain\r\n");
    api.print(b"  Wave = channel message\r\n");
    api.print(b"  Flash = priority preemption\r\n");
    api.print(b"\r\n");
    api.print(b"  send h = help  center btn = exit\r\n");
    api.print(b"=========================================\r\n\r\n");

    // Startup sound
    (api.sound_play)(880);
    (api.delay_ms)(80);
    (api.sound_play)(1320);
    (api.delay_ms)(80);
    (api.sound_play)(1760);
    (api.delay_ms)(80);
    (api.sound_stop)();

    loop {
        let tick_ms = if fast_mode { TICK_MS / 2 } else { TICK_MS };
        let evt = (api.wait_event)(EVT_BUTTON | EVT_INPUT | EVT_TIMEOUT, tick_ms);

        // ── Handle input ────────────────────────────────
        if evt & EVT_BUTTON != 0 {
            let btns = (api.read_buttons)();
            if btns & BTN_CENTER != 0 {
                // Clear LEDs and stop sound
                (api.sound_stop)();
                for i in 0u32..25 {
                    if i as usize != HEARTBEAT {
                        (api.set_pixel)(i, 0);
                    }
                }
                (api.update_leds)();
                return 0;
            }
            if btns & BTN_LEFT != 0 {
                // Spawn cascade from left column
                trigger_cascade(&mut tasks, &[0, 5, 10, 15, 20], &mut rng);
                stats_spawns += 1;
            }
            if btns & BTN_RIGHT != 0 {
                // Spawn cascade from right column (pixels 4,9,14,19,24 → tasks)
                trigger_cascade(&mut tasks, &[4, 9, 13, 18, 23], &mut rng);
                stats_spawns += 1;
            }
        }

        // Always poll input — EVT_INPUT detection can miss in sandboxed mode
        {
            let mut buf = [0u8; 8];
            let n = (api.read_input)(buf.as_mut_ptr(), buf.len() as u32);
            if n > 0 {
                match buf[0] {
                    b'q' => {
                        (api.sound_stop)();
                        for i in 0u32..25 {
                            if i as usize != HEARTBEAT { (api.set_pixel)(i, 0); }
                        }
                        (api.update_leds)();
                        return 0;
                    }
                    b'h' => {
                        api.print(b"\r\n--- RTIC Async Visualizer ---\r\n");
                        api.print(b"  24 LEDs = 24 async tasks\r\n");
                        api.print(b"  Each pulse = task wakeup\r\n");
                        api.print(b"  Cascading flash = spawn() chain\r\n");
                        api.print(b"  Ripple wave = channel::send/recv\r\n");
                        api.print(b"  Priority flash = preemption demo\r\n");
                        api.print(b"  s=spawn m=message p=priority f=fast q=quit\r\n");
                    }
                    b's' => {
                        // Spawn cascade from random pixel
                        let t = rng.range(N_TASKS as u32) as usize;
                        tasks[t].cascade = 3;
                        tasks[t].brightness = 100;
                        stats_spawns += 1;
                        api.print(b"[spawn cascade]\r\n");
                    }
                    b'm' => {
                        // Message wave from center area
                        let origin = rng.range(N_TASKS as u32) as usize;
                        global = GlobalEffect::MessageWave {
                            origin,
                            radius: 0,
                            ticks: 20,
                        };
                        stats_messages += 1;
                        api.print(b"[message wave]\r\n");
                    }
                    b'p' => {
                        global = GlobalEffect::PriorityEvent(10);
                        api.print(b"[priority preemption]\r\n");
                    }
                    b'f' => {
                        fast_mode = !fast_mode;
                        if fast_mode {
                            api.print(b"[fast mode ON]\r\n");
                        } else {
                            api.print(b"[fast mode OFF]\r\n");
                        }
                    }
                    _ => {}
                }
            }
        }

        // Ensure sound is stopped unless we explicitly want it
        if !matches!(global, GlobalEffect::PriorityEvent(_)) {
            (api.sound_stop)();
        }

        // ── Update simulation ───────────────────────────

        tick_count += 1;

        // Auto-triggers for autonomous animation
        auto_cascade_timer += 1;
        auto_msg_timer += 1;
        auto_pri_timer += 1;

        // Auto spawn cascade every 2–4 seconds
        if auto_cascade_timer > rng.range(40) as u16 + 40 {
            auto_cascade_timer = 0;
            let t = rng.range(N_TASKS as u32) as usize;
            tasks[t].cascade = 3;
            tasks[t].brightness = 100;
            stats_spawns += 1;
        }

        // Auto message wave every 4–8 seconds
        if auto_msg_timer > rng.range(80) as u16 + 80 {
            auto_msg_timer = 0;
            let origin = rng.range(N_TASKS as u32) as usize;
            if matches!(global, GlobalEffect::None) {
                global = GlobalEffect::MessageWave {
                    origin,
                    radius: 0,
                    ticks: 16,
                };
                stats_messages += 1;
            }
        }

        // Auto priority event every 8–12 seconds
        if auto_pri_timer > rng.range(80) as u16 + 160 {
            auto_pri_timer = 0;
            if matches!(global, GlobalEffect::None) {
                global = GlobalEffect::PriorityEvent(10);
            }
        }

        // Lock flash every ~200 ticks (10s) — brief all-dim to show critical section
        if tick_count % 200 == 0 && matches!(global, GlobalEffect::None) {
            global = GlobalEffect::LockFlash(3);
        }

        // ── Tick each task ──────────────────────────────

        for i in 0..N_TASKS {
            // Countdown sleep
            if tasks[i].sleep_remain > 0 {
                tasks[i].sleep_remain -= 1;
            }

            // Task wakes up!
            if tasks[i].sleep_remain == 0 {
                // "Run" the task — flash bright based on priority
                let max_bri = match tasks[i].priority {
                    3 => 100,
                    2 => 60,
                    _ => 30,
                };
                tasks[i].brightness = max_bri;

                // "Delay().await" — go back to sleep
                tasks[i].sleep_remain = tasks[i].period;

                // Small chance to spawn a cascade (simulates spawn())
                if rng.range(20) == 0 {
                    tasks[i].cascade = 2;
                    stats_spawns += 1;
                }
            }

            // Handle cascade spawning
            if tasks[i].cascade > 0 {
                tasks[i].cascade -= 1;
                if tasks[i].cascade == 0 {
                    // Spawn neighbors (like task::spawn())
                    let nb = neighbors(i);
                    for slot in &nb {
                        if let Some(n) = *slot {
                            if tasks[n].brightness < 40 {
                                tasks[n].brightness = 80;
                                tasks[n].sleep_remain = tasks[n].period;
                                // Chain: small chance to cascade further
                                if rng.range(4) == 0 {
                                    tasks[n].cascade = 2;
                                }
                            }
                        }
                    }
                }
            }

            // Handle ripple (channel message visualization)
            if tasks[i].ripple > 0 {
                tasks[i].ripple -= 1;
                let ripple_bri = tasks[i].ripple as u8 * 15;
                if ripple_bri > tasks[i].brightness {
                    tasks[i].brightness = ripple_bri;
                }
            }

            // Decay brightness (fade down each tick)
            if tasks[i].brightness > 0 {
                let decay = match tasks[i].priority {
                    3 => 8, // high-pri fades fast (short burst)
                    2 => 5,
                    _ => 3, // low-pri glows longer
                };
                if tasks[i].brightness > decay {
                    tasks[i].brightness -= decay;
                } else {
                    tasks[i].brightness = 0;
                }
            }
        }

        // ── Global effects ──────────────────────────────

        match &mut global {
            GlobalEffect::PriorityEvent(ref mut ticks) => {
                // Priority preemption visualization:
                // High-pri tasks blaze, low-pri tasks dim out
                for i in 0..N_TASKS {
                    match tasks[i].priority {
                        3 => tasks[i].brightness = 100,
                        2 => {
                            if tasks[i].brightness > 20 {
                                tasks[i].brightness = 20;
                            }
                        }
                        _ => tasks[i].brightness = 0,
                    }
                }
                // Short beep while priority event active
                if *ticks == 8 {
                    (api.sound_play)(660);
                }
                if *ticks == 5 {
                    (api.sound_stop)();
                }
                *ticks -= 1;
                if *ticks == 0 {
                    // Resolution: all tasks resume normally
                    global = GlobalEffect::None;
                }
            }
            GlobalEffect::LockFlash(ref mut ticks) => {
                // All LEDs dim briefly = critical section lock
                for i in 0..N_TASKS {
                    tasks[i].brightness = 1;
                }
                *ticks -= 1;
                if *ticks == 0 {
                    global = GlobalEffect::None;
                }
            }
            GlobalEffect::MessageWave {
                origin,
                ref mut radius,
                ref mut ticks,
            } => {
                // Ripple outward from origin — Manhattan distance
                let op = task_to_pixel(*origin) as usize;
                let o_row = op / 5;
                let o_col = op % 5;
                let r = *radius as usize;

                for i in 0..N_TASKS {
                    let p = task_to_pixel(i) as usize;
                    let row = p / 5;
                    let col = p % 5;
                    let dist = abs_diff(row, o_row) + abs_diff(col, o_col);
                    if dist == r {
                        tasks[i].ripple = 5;
                        tasks[i].brightness = 80;
                    }
                }

                // Expand radius every 2 ticks
                if *ticks % 2 == 0 && *radius < 8 {
                    *radius += 1;
                }
                *ticks -= 1;
                if *ticks == 0 {
                    global = GlobalEffect::None;
                }
            }
            GlobalEffect::None => {}
        }

        // ── Render LEDs ─────────────────────────────────

        for i in 0..N_TASKS {
            let pixel = task_to_pixel(i);
            let bri = tasks[i].brightness as u32;
            (api.set_pixel)(pixel, bri);
        }
        (api.update_leds)();

        // ── Periodic status line ────────────────────────

        if tick_count % 100 == 0 {
            // Count active (non-zero brightness) tasks
            let active = tasks.iter().filter(|t| t.brightness > 0).count();
            let _ = fmt_status(api, tick_count, active as u32, stats_spawns, stats_messages);
        }
    }
}

// ── Helpers ────────────────────────────────────────────────

fn abs_diff(a: usize, b: usize) -> usize {
    if a > b { a - b } else { b - a }
}

fn trigger_cascade(tasks: &mut [Task; N_TASKS], task_ids: &[usize], rng: &mut Rng) {
    for &t in task_ids {
        if t < N_TASKS {
            tasks[t].cascade = (rng.range(3) as u8) + 1;
            tasks[t].brightness = 100;
        }
    }
}

fn fmt_status(api: &MonitorApi, ticks: u32, active: u32, spawns: u32, msgs: u32) {
    // Manual number formatting (no alloc)
    let mut buf = [0u8; 80];
    let mut pos = 0;

    let prefix = b"[rtic] t=";
    buf[pos..pos + prefix.len()].copy_from_slice(prefix);
    pos += prefix.len();
    pos += fmt_u32(&mut buf[pos..], ticks);

    let mid = b" active=";
    buf[pos..pos + mid.len()].copy_from_slice(mid);
    pos += mid.len();
    pos += fmt_u32(&mut buf[pos..], active);

    let sp = b" spawns=";
    buf[pos..pos + sp.len()].copy_from_slice(sp);
    pos += sp.len();
    pos += fmt_u32(&mut buf[pos..], spawns);

    let ms = b" msgs=";
    buf[pos..pos + ms.len()].copy_from_slice(ms);
    pos += ms.len();
    pos += fmt_u32(&mut buf[pos..], msgs);

    buf[pos] = b'\r';
    pos += 1;
    buf[pos] = b'\n';
    pos += 1;

    api.print(&buf[..pos]);
}

fn fmt_u32(buf: &mut [u8], val: u32) -> usize {
    if val == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut tmp = [0u8; 10];
    let mut n = val;
    let mut i = 0;
    while n > 0 {
        tmp[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    // Reverse into buf
    for j in 0..i {
        buf[j] = tmp[i - 1 - j];
    }
    i
}

// ── Panic handler ──────────────────────────────────────────

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
