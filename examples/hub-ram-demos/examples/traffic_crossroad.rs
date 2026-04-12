//! Traffic crossroad simulation on the 5x5 LED matrix.
//!
//! A miniature intersection with four traffic lights and cars making
//! random turns (straight, left, right).  Cars obey the traffic signals.
//! Pure MonitorApi usage -- no privilege required, works with `go`.
//!
//! ## LED layout
//!
//! ```text
//!  Row 0:  [ B][ B][ R ][ B][ B]     B=building  R=road(vertical)
//!  Row 1:  [ B][TL][ R ][TR][ B]     TL=NS light TR=EW light
//!  Row 2:  [ R][ R][ ♥ ][ R][ R]     Road(horiz) ♥=heartbeat(skip)
//!  Row 3:  [ B][BL][ R ][BR][ B]     BL=EW light BR=NS light
//!  Row 4:  [ B][ B][ R ][ B][ B]
//! ```
//!
//! ## Traffic lights
//!
//! - NS pair: pixels 6 (NW) and 18 (SE) -- controls north/south traffic
//! - EW pair: pixels 8 (NE) and 16 (SW) -- controls east/west traffic
//! - Green = bright (9), Yellow = medium (5), Red = dim (1)
//!
//! ## Cars
//!
//! - Enter from edges: pixel 2 (north), 22 (south), 10 (west), 14 (east)
//! - Wait at intersection approach if red light
//! - Random turn decision: 50% straight, 25% left, 25% right
//! - Max 4 cars simultaneously
//!
//! ## Interactive commands (`spike> send <char>`)
//!
//! | Key | Action                              |
//! |-----|-------------------------------------|
//! | `h` | Help                                |
//! | `c` | Spawn car at random entry           |
//! | `g` | Force traffic light change          |
//! | `f` | Toggle fast/normal speed            |
//! | `q` | Quit demo                           |
//!
//! ## Button controls
//!
//! - Center: exit demo
//! - Left: spawn car from west
//! - Right: spawn car from east
//!
//! Requires: API v12+, no external devices.  Non-privileged (`go`).

#![no_std]
#![no_main]

use spike_hub_api::{
    MonitorApi, BTN_CENTER, BTN_LEFT, BTN_RIGHT,
    EVT_BUTTON, EVT_INPUT, EVT_TIMEOUT,
};

// ── Timing ─────────────────────────────────────────────────────

const TICK_MS: u32 = 300;          // base simulation tick
const TICK_FAST_MS: u32 = 150;     // fast mode tick

const GREEN_TICKS: u16 = 14;      // ~4.2s green phase
const YELLOW_TICKS: u16 = 3;      // ~0.9s yellow phase

// ── Pixel assignments ──────────────────────────────────────────

// Roads
const ROAD_N0: u8 = 2;   // north entry (top of vertical road)
const ROAD_N1: u8 = 7;   // north approach (wait position for southbound)
const ROAD_S1: u8 = 17;  // south approach (wait position for northbound)
const ROAD_S0: u8 = 22;  // south entry (bottom of vertical road)
const ROAD_W0: u8 = 10;  // west entry
const ROAD_W1: u8 = 11;  // west approach (wait position for eastbound)
const ROAD_E1: u8 = 13;  // east approach (wait position for westbound)
const ROAD_E0: u8 = 14;  // east entry

// Traffic lights
const TL_NS_NW: u8 = 6;  // NS traffic light (northwest corner)
const TL_NS_SE: u8 = 18; // NS traffic light (southeast corner)
const TL_EW_NE: u8 = 8;  // EW traffic light (northeast corner)
const TL_EW_SW: u8 = 16; // EW traffic light (southwest corner)

// Buildings (decorative, dark or dim)
const CORNERS: [u8; 4] = [20, 21, 23, 24];

// Heartbeat pixel -- NEVER TOUCH
const HEARTBEAT: u8 = 12;

// ── Brightness levels ──────────────────────────────────────────

const BRI_OFF: u32 = 0;
const BRI_ROAD: u32 = 1;      // dim road surface
const BRI_RED: u32 = 2;       // traffic light red
const BRI_YELLOW: u32 = 5;    // traffic light yellow
const BRI_GREEN: u32 = 8;     // traffic light green
const BRI_CAR: u32 = 15;      // car = brightest

// ── Directions ─────────────────────────────────────────────────

const DIR_SOUTH: u8 = 0;  // driving top→bottom
const DIR_NORTH: u8 = 1;  // driving bottom→top
const DIR_EAST: u8 = 2;   // driving left→right
const DIR_WEST: u8 = 3;   // driving right→left

// ── Turn decisions ─────────────────────────────────────────────

const TURN_STRAIGHT: u8 = 0;
const TURN_RIGHT: u8 = 1;
const TURN_LEFT: u8 = 2;

// ── Car state ──────────────────────────────────────────────────

const MAX_CARS: usize = 6;

#[derive(Clone, Copy)]
struct Car {
    active: bool,
    pixel: u8,         // current pixel position
    direction: u8,     // DIR_SOUTH/NORTH/EAST/WEST (initial travel direction)
    exiting: bool,     // true = past intersection, heading to exit
    exit_pixel: u8,    // next pixel after intersection crossing
    final_pixel: u8,   // edge pixel where car despawns
}

impl Car {
    const fn empty() -> Self {
        Car {
            active: false,
            pixel: 0,
            direction: 0,
            exiting: false,
            exit_pixel: 0,
            final_pixel: 0,
        }
    }
}

// ── Traffic light phases ───────────────────────────────────────

const PHASE_NS_GREEN: u8 = 0;
const PHASE_NS_YELLOW: u8 = 1;
const PHASE_EW_GREEN: u8 = 2;
const PHASE_EW_YELLOW: u8 = 3;

// ── LFSR random ────────────────────────────────────────────────

struct Rng {
    state: u32,
}

impl Rng {
    fn new(seed: u32) -> Self {
        Rng { state: if seed == 0 { 0xCAFE_BABE } else { seed } }
    }

    fn next(&mut self) -> u32 {
        // xorshift32
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        x
    }

    /// Random value in 0..n (biased but fine for a toy demo)
    fn range(&mut self, n: u32) -> u32 {
        self.next() % n
    }
}

// ── Entry point ────────────────────────────────────────────────

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 12 {
        api.print(b"ERR: need API v12+ (wait_event)\r\n");
        return 1;
    }

    api.print(b"\r\n");
    api.print(b"==============================================\r\n");
    api.print(b"  TRAFFIC CROSSROAD -- 5x5 LED simulation\r\n");
    api.print(b"==============================================\r\n");
    api.print(b"  Roads: + shape    Lights: 4 corners\r\n");
    api.print(b"  Cars spawn randomly, obey traffic lights,\r\n");
    api.print(b"  and make random turns at intersection.\r\n");
    api.print(b"\r\n");
    api.print(b"  send h = help   center btn = exit\r\n");
    api.print(b"==============================================\r\n\r\n");

    // Debounce center button
    for _ in 0..20 {
        if (api.read_buttons)() & BTN_CENTER == 0 { break; }
        (api.delay_ms)(50);
    }

    // Seed RNG from ADC noise
    let seed = (api.read_adc)(8) ^ ((api.read_adc)(11) << 12);
    let mut rng = Rng::new(seed);

    // State
    let mut cars = [Car::empty(); MAX_CARS];
    let mut phase: u8 = PHASE_NS_GREEN;
    let mut phase_timer: u16 = GREEN_TICKS;
    let mut fast_mode = false;
    let mut tick_count: u32 = 0;
    let mut spawn_cooldown: u8 = 0;

    // Initial LED state
    render(api, &cars, phase, phase_timer);

    // Sound: startup beep
    (api.sound_play)(880);
    (api.delay_ms)(80);
    (api.sound_stop)();

    api.print(b"Traffic running... send h for help\r\n");

    loop {
        let tick = if fast_mode { TICK_FAST_MS } else { TICK_MS };
        let evt = (api.wait_event)(EVT_BUTTON | EVT_INPUT | EVT_TIMEOUT, tick);

        // ── Handle input ───────────────────────────────────
        if evt & EVT_INPUT != 0 {
            let mut buf = [0u8; 8];
            let n = (api.read_input)(buf.as_mut_ptr(), buf.len() as u32);
            if n > 0 {
                match buf[0] {
                    b'h' => print_help(api),
                    b'q' => {
                        api.print(b"Exiting traffic sim.\r\n");
                        clear_leds(api);
                        return 0;
                    }
                    b'c' => {
                        let dir = rng.range(4) as u8;
                        if spawn_car(&mut cars, dir) {
                            api.print(b"Car spawned!\r\n");
                        } else {
                            api.print(b"No room for car.\r\n");
                        }
                    }
                    b'g' => {
                        advance_phase(&mut phase, &mut phase_timer);
                        api.print(b"Light changed!\r\n");
                    }
                    b'f' => {
                        fast_mode = !fast_mode;
                        if fast_mode {
                            api.print(b"FAST mode ON\r\n");
                        } else {
                            api.print(b"Normal speed\r\n");
                        }
                    }
                    _ => {
                        api.print(b"Unknown cmd. send h for help\r\n");
                    }
                }
            }
        }

        // ── Handle buttons ─────────────────────────────────
        if evt & EVT_BUTTON != 0 {
            let btns = (api.read_buttons)();
            if btns & BTN_CENTER != 0 {
                api.print(b"Center btn -- exit.\r\n");
                clear_leds(api);
                // debounce
                while (api.read_buttons)() & BTN_CENTER != 0 {
                    (api.delay_ms)(50);
                }
                return 0;
            }
            if btns & BTN_LEFT != 0 {
                spawn_car(&mut cars, DIR_EAST);  // car from west going east
                (api.sound_play)(440);
                (api.delay_ms)(40);
                (api.sound_stop)();
            }
            if btns & BTN_RIGHT != 0 {
                spawn_car(&mut cars, DIR_WEST);  // car from east going west
                (api.sound_play)(660);
                (api.delay_ms)(40);
                (api.sound_stop)();
            }
        }

        // ── Simulation tick ────────────────────────────────
        tick_count += 1;

        // Update traffic lights
        if phase_timer > 0 {
            phase_timer -= 1;
        }
        if phase_timer == 0 {
            advance_phase(&mut phase, &mut phase_timer);
        }

        // Determine which direction has green
        let ns_go = phase == PHASE_NS_GREEN || phase == PHASE_NS_YELLOW;
        let ew_go = phase == PHASE_EW_GREEN || phase == PHASE_EW_YELLOW;

        // Move cars
        move_cars(&mut cars, ns_go, ew_go, &mut rng);

        // Auto-spawn cars periodically
        if spawn_cooldown > 0 {
            spawn_cooldown -= 1;
        }
        if spawn_cooldown == 0 && count_active(&cars) < 3 {
            let dir = rng.range(4) as u8;
            if spawn_car(&mut cars, dir) {
                spawn_cooldown = 4 + (rng.range(6) as u8); // 4-9 ticks between spawns
            }
        }

        // Render
        render(api, &cars, phase, phase_timer);

        // Periodic status
        if tick_count % 20 == 0 {
            api.print(b"[traffic] cars=");
            print_u32(api, count_active(&cars) as u32);
            api.print(b" phase=");
            let ph_name = match phase {
                PHASE_NS_GREEN  => b"NS-grn" as &[u8],
                PHASE_NS_YELLOW => b"NS-yel" as &[u8],
                PHASE_EW_GREEN  => b"EW-grn" as &[u8],
                PHASE_EW_YELLOW => b"EW-yel" as &[u8],
                _ => b"?" as &[u8],
            };
            api.print(ph_name);
            api.print(b"\r\n");
        }
    }
}

// ── Traffic light phase machine ────────────────────────────────

fn advance_phase(phase: &mut u8, timer: &mut u16) {
    *phase = (*phase + 1) % 4;
    *timer = match *phase {
        PHASE_NS_GREEN | PHASE_EW_GREEN => GREEN_TICKS,
        PHASE_NS_YELLOW | PHASE_EW_YELLOW => YELLOW_TICKS,
        _ => GREEN_TICKS,
    };
}

// ── Car management ─────────────────────────────────────────────

fn count_active(cars: &[Car; MAX_CARS]) -> usize {
    cars.iter().filter(|c| c.active).count()
}

/// Entry pixels for each direction
fn entry_pixel(dir: u8) -> u8 {
    match dir {
        DIR_SOUTH => ROAD_N0,   // enters from north edge
        DIR_NORTH => ROAD_S0,   // enters from south edge
        DIR_EAST  => ROAD_W0,   // enters from west edge
        DIR_WEST  => ROAD_E0,   // enters from east edge
        _ => 0,
    }
}

/// Wait pixel (just before intersection) for each direction
fn wait_pixel(dir: u8) -> u8 {
    match dir {
        DIR_SOUTH => ROAD_N1,
        DIR_NORTH => ROAD_S1,
        DIR_EAST  => ROAD_W1,
        DIR_WEST  => ROAD_E1,
        _ => 0,
    }
}

fn spawn_car(cars: &mut [Car; MAX_CARS], dir: u8) -> bool {
    let ep = entry_pixel(dir);
    // Don't spawn if entry pixel is occupied
    if pixel_occupied(cars, ep) {
        return false;
    }
    // Find free slot
    for car in cars.iter_mut() {
        if !car.active {
            *car = Car {
                active: true,
                pixel: ep,
                direction: dir,
                exiting: false,
                exit_pixel: 0,
                final_pixel: 0,
            };
            return true;
        }
    }
    false
}

fn pixel_occupied(cars: &[Car; MAX_CARS], px: u8) -> bool {
    cars.iter().any(|c| c.active && c.pixel == px)
}

/// Compute the exit path (post-intersection pixel and final edge pixel)
/// based on initial direction and chosen turn.
fn exit_path(dir: u8, turn: u8) -> (u8, u8) {
    match (dir, turn) {
        // Southbound (top → bottom)
        (DIR_SOUTH, TURN_STRAIGHT) => (ROAD_S1, ROAD_S0),  // → 17 → 22
        (DIR_SOUTH, TURN_RIGHT)    => (ROAD_W1, ROAD_W0),  // → 11 → 10 (turn west)
        (DIR_SOUTH, TURN_LEFT)     => (ROAD_E1, ROAD_E0),  // → 13 → 14 (turn east)

        // Northbound (bottom → top)
        (DIR_NORTH, TURN_STRAIGHT) => (ROAD_N1, ROAD_N0),  // → 7 → 2
        (DIR_NORTH, TURN_RIGHT)    => (ROAD_E1, ROAD_E0),  // → 13 → 14 (turn east)
        (DIR_NORTH, TURN_LEFT)     => (ROAD_W1, ROAD_W0),  // → 11 → 10 (turn west)

        // Eastbound (left → right)
        (DIR_EAST, TURN_STRAIGHT)  => (ROAD_E1, ROAD_E0),  // → 13 → 14
        (DIR_EAST, TURN_RIGHT)     => (ROAD_S1, ROAD_S0),  // → 17 → 22 (turn south)
        (DIR_EAST, TURN_LEFT)      => (ROAD_N1, ROAD_N0),  // → 7 → 2 (turn north)

        // Westbound (right → left)
        (DIR_WEST, TURN_STRAIGHT)  => (ROAD_W1, ROAD_W0),  // → 11 → 10
        (DIR_WEST, TURN_RIGHT)     => (ROAD_N1, ROAD_N0),  // → 7 → 2 (turn north)
        (DIR_WEST, TURN_LEFT)      => (ROAD_S1, ROAD_S0),  // → 17 → 22 (turn south)

        _ => (ROAD_S1, ROAD_S0),
    }
}

fn move_cars(cars: &mut [Car; MAX_CARS], ns_go: bool, ew_go: bool, rng: &mut Rng) {
    // Process each car independently.
    // We iterate by index to allow checking other cars for collisions.
    for i in 0..MAX_CARS {
        if !cars[i].active {
            continue;
        }

        let car = cars[i];

        if car.exiting {
            // Car is past intersection, heading to final_pixel
            if car.pixel == car.exit_pixel {
                // Move to final pixel (edge) if not occupied
                if !pixel_occupied_except(cars, car.final_pixel, i) {
                    cars[i].pixel = car.final_pixel;
                } // else wait
            } else if car.pixel == car.final_pixel {
                // Reached edge -- despawn
                cars[i].active = false;
            }
        } else {
            // Car is approaching intersection
            let wp = wait_pixel(car.direction);

            if car.pixel == entry_pixel(car.direction) {
                // At entry pixel, move to wait pixel if free
                if !pixel_occupied_except(cars, wp, i) {
                    cars[i].pixel = wp;
                }
            } else if car.pixel == wp {
                // At wait pixel -- check traffic light
                let can_go = match car.direction {
                    DIR_SOUTH | DIR_NORTH => ns_go,
                    DIR_EAST | DIR_WEST => ew_go,
                    _ => false,
                };

                if can_go {
                    // Choose random turn: 50% straight, 25% right, 25% left
                    let r = rng.range(4);
                    let turn = match r {
                        0 | 1 => TURN_STRAIGHT,
                        2 => TURN_RIGHT,
                        3 => TURN_LEFT,
                        _ => TURN_STRAIGHT,
                    };

                    let (exit_px, final_px) = exit_path(car.direction, turn);

                    // Cross intersection (skip pixel 12) -- move directly
                    // to exit pixel if not occupied
                    if !pixel_occupied_except(cars, exit_px, i) {
                        cars[i].pixel = exit_px;
                        cars[i].exiting = true;
                        cars[i].exit_pixel = exit_px;
                        cars[i].final_pixel = final_px;
                    }
                }
                // else: red light, wait
            }
        }
    }
}

/// Check if pixel is occupied by any car except car at index `except`
fn pixel_occupied_except(cars: &[Car; MAX_CARS], px: u8, except: usize) -> bool {
    for j in 0..MAX_CARS {
        if j != except && cars[j].active && cars[j].pixel == px {
            return true;
        }
    }
    false
}

// ── Rendering ──────────────────────────────────────────────────

fn render(api: &MonitorApi, cars: &[Car; MAX_CARS], phase: u8, _timer: u16) {
    // 1. Clear all pixels except heartbeat
    for i in 0u32..25 {
        if i as u8 == HEARTBEAT { continue; }
        (api.set_pixel)(i, BRI_OFF);
    }

    // 2. Draw road surface (dim)
    let road_pixels = [
        ROAD_N0, ROAD_N1, ROAD_S1, ROAD_S0,
        ROAD_W0, ROAD_W1, ROAD_E1, ROAD_E0,
    ];
    for &px in road_pixels.iter() {
        (api.set_pixel)(px as u32, BRI_ROAD);
    }

    // 3. Draw buildings (corner decorations -- subtle)
    for &px in CORNERS.iter() {
        // Dim corner markers to frame the intersection
        (api.set_pixel)(px as u32, 1);
    }

    // 4. Draw traffic lights
    let (ns_bri, ew_bri) = match phase {
        PHASE_NS_GREEN  => (BRI_GREEN, BRI_RED),
        PHASE_NS_YELLOW => (BRI_YELLOW, BRI_RED),
        PHASE_EW_GREEN  => (BRI_RED, BRI_GREEN),
        PHASE_EW_YELLOW => (BRI_RED, BRI_YELLOW),
        _ => (BRI_RED, BRI_RED),
    };

    (api.set_pixel)(TL_NS_NW as u32, ns_bri);
    (api.set_pixel)(TL_NS_SE as u32, ns_bri);
    (api.set_pixel)(TL_EW_NE as u32, ew_bri);
    (api.set_pixel)(TL_EW_SW as u32, ew_bri);

    // 5. Draw cars (brightest, overwrites road surface)
    for car in cars.iter() {
        if car.active && car.pixel != HEARTBEAT {
            (api.set_pixel)(car.pixel as u32, BRI_CAR);
        }
    }

    // 6. Push to display
    (api.update_leds)();
}

fn clear_leds(api: &MonitorApi) {
    for i in 0u32..25 {
        if i as u8 == HEARTBEAT { continue; }
        (api.set_pixel)(i, BRI_OFF);
    }
    (api.update_leds)();
}

// ── Help ───────────────────────────────────────────────────────

fn print_help(api: &MonitorApi) {
    api.print(b"\r\n--- Traffic Crossroad Help ---\r\n");
    api.print(b"  send c = spawn car\r\n");
    api.print(b"  send g = force light change\r\n");
    api.print(b"  send f = toggle fast mode\r\n");
    api.print(b"  send h = this help\r\n");
    api.print(b"  send q = quit\r\n");
    api.print(b"  left btn  = car from west\r\n");
    api.print(b"  right btn = car from east\r\n");
    api.print(b"  center    = exit\r\n");
    api.print(b"------------------------------\r\n\r\n");
}

// ── Number printing ────────────────────────────────────────────

fn print_u32(api: &MonitorApi, val: u32) {
    if val == 0 {
        api.print(b"0");
        return;
    }
    let mut buf = [b'0'; 10];
    let mut n = val;
    let mut i = 9;
    while n > 0 {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        if i == 0 { break; }
        i -= 1;
    }
    if val > 0 { i += 1; }
    api.print(&buf[i..]);
}

// ── Panic handler ──────────────────────────────────────────────

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
