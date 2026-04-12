//! PDP-11 front-panel dashboard — privileged RTIC-inspection demo.
//!
//! **Requires `go!` (privileged mode)** — reads Cortex-M4 system registers
//! (NVIC, SCB, MPU, SysTick, timers) that are inaccessible from unprivileged
//! Thread mode.  The demo header contains `PRIV_MAGIC` so firmware will
//! refuse `go` (sandboxed) with a clear error message.
//!
//! ## What it does
//!
//! 1. **PDP-11 dashboard lights** — the 5×5 LED matrix blinks like a 1970s
//!    minicomputer front panel, driven by actual hardware register values.
//!    Each row shows different system state as binary/activity lights.
//!    Pixel 12 (center) is NEVER touched — that's the firmware heartbeat.
//!
//! 2. **Interactive register inspection** — type `send` commands to dump
//!    RTIC's interrupt priorities, SCB state, MPU config, clock tree, and
//!    timer registers to the serial console.
//!
//! ## LED layout (PDP-11 style)
//!
//! ```text
//!  Row 0:  [ 0][ 1][ 2][ 3][ 4]  "ADDR" — 5-bit address counter
//!  Row 1:  [ 5][ 6][ 7][ 8][ 9]  "DATA HI" — register data bits 15-11
//!  Row 2:  [10][11][ ♥][13][14]   "DATA LO" — bits 10-8, 7-6  (12=heartbeat)
//!  Row 3:  [15][16][17][18][19]  "BUS" — shifting activity pattern
//!  Row 4:  [20][21][22][23][24]  "STATUS" — button state + run indicator
//! ```
//!
//! ## Interactive commands (`spike> send <char>`)
//!
//! | Key | Action                                              |
//! |-----|-----------------------------------------------------|
//! | `h` | Help                                                |
//! | `n` | Dump NVIC interrupt priorities (RTIC task config)    |
//! | `s` | Dump SCB registers (system/fault state)              |
//! | `m` | Dump MPU configuration                               |
//! | `c` | Dump RCC clock tree                                  |
//! | `t` | Dump SysTick + TIM1/TIM2 (RTIC monotonics/motors)   |
//! | `p` | Cycle LED pattern mode (pdp11/scan/random)           |
//! | `q` | Quit demo                                            |
//!
//! ## Usage
//!
//! ```text
//! spike> upload pdp11_lights.bin
//! spike> go!
//! spike> send h
//! spike> send n       # see RTIC interrupt priorities
//! spike> send q       # exit
//! ```
//!
//! Requires: API v12+, no external devices.  Privileged mode only.

#![no_std]
#![no_main]

use spike_hub_api::{
    MonitorApi, BTN_CENTER, BTN_LEFT, BTN_RIGHT, PRIV_MAGIC,
    EVT_BUTTON, EVT_INPUT,
};

// ── Privileged demo header ─────────────────────────────────
//
// Placed in .demo_header (before .text._start in the linker script).
// Firmware reads the first word of the binary; if it matches
// PRIV_MAGIC, `go` (sandboxed) is refused.

#[link_section = ".demo_header"]
#[used]
static _PRIV_MARKER: u32 = PRIV_MAGIC;

// ── Cortex-M4 system register addresses (PPB) ─────────────

const NVIC_ISER0: u32 = 0xE000_E100; // Interrupt Set Enable
const NVIC_IPR0: u32 = 0xE000_E400;  // Interrupt Priority (byte-accessible)

const SCB_CPUID: u32 = 0xE000_ED00;
const SCB_ICSR: u32 = 0xE000_ED04;
const SCB_VTOR: u32 = 0xE000_ED08;
const SCB_AIRCR: u32 = 0xE000_ED0C;
const SCB_SCR: u32 = 0xE000_ED10;
const SCB_CCR: u32 = 0xE000_ED14;
const SCB_SHPR1: u32 = 0xE000_ED18;
const SCB_SHPR2: u32 = 0xE000_ED1C;
const SCB_SHPR3: u32 = 0xE000_ED20;
const SCB_SHCSR: u32 = 0xE000_ED24;

const MPU_TYPE: u32 = 0xE000_ED90;
const MPU_CTRL: u32 = 0xE000_ED94;
const MPU_RNR: u32 = 0xE000_ED98;
const MPU_RBAR: u32 = 0xE000_ED9C;
const MPU_RASR: u32 = 0xE000_EDA0;

const SYSTICK_CSR: u32 = 0xE000_E010;
const SYSTICK_RVR: u32 = 0xE000_E014;
const SYSTICK_CVR: u32 = 0xE000_E018;

// STM32F413 peripherals
const RCC_CR: u32 = 0x4002_3800;
const RCC_CFGR: u32 = 0x4002_3808;
const RCC_AHB1ENR: u32 = 0x4002_3830;
const RCC_APB1ENR: u32 = 0x4002_3840;
const RCC_APB2ENR: u32 = 0x4002_3844;

const TIM1_BASE: u32 = 0x4001_0000;
const TIM2_BASE: u32 = 0x4000_0000;
const TIM_CR1: u32 = 0x00;
const TIM_PSC: u32 = 0x28;
const TIM_ARR: u32 = 0x2C;
const TIM_CNT: u32 = 0x24;

// ── Display state ──────────────────────────────────────────

const BLINK_MS: u32 = 250; // >200ms as requested

/// LED pattern modes
const MODE_PDP11: u8 = 0;
const MODE_SCAN: u8 = 1;
const MODE_RANDOM: u8 = 2;
const MODE_COUNT: u8 = 3;

// ── Register read helper ───────────────────────────────────

#[inline(always)]
unsafe fn rd(addr: u32) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

// ── Hex formatting ─────────────────────────────────────────

fn hex(v: u32, buf: &mut [u8; 10]) -> &[u8] {
    buf[0] = b'0';
    buf[1] = b'x';
    let h = b"0123456789ABCDEF";
    for i in 0..8 {
        buf[2 + i] = h[((v >> (28 - i * 4)) & 0xF) as usize];
    }
    &buf[..]
}

// ── Number printing ────────────────────────────────────────

fn print_u32(api: &MonitorApi, val: u32) {
    if val == 0 {
        api.print(b"0");
        return;
    }
    let mut buf = [b'0'; 10];
    let mut v = val;
    let mut i = 9;
    while v > 0 {
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if i == 0 { break; }
        i -= 1;
    }
    if val > 0 { i += 1; }
    api.print(&buf[i..]);
}

// ── LFSR pseudo-random (for PDP-11 data bus effect) ────────

fn lfsr_step(state: u32) -> u32 {
    if state == 0 { return 1; }
    let bit = ((state >> 31) ^ (state >> 21) ^ (state >> 1) ^ state) & 1;
    (state << 1) | bit
}

// ── LED patterns ───────────────────────────────────────────

/// PDP-11 mode: register data drives LEDs
fn pattern_pdp11(api: &MonitorApi, tick: u32, reg_data: u32) {
    // Row 0 (px 0-4): 5-bit address counter (tick-driven)
    let addr_bits = tick & 0x1F;
    for bit in 0..5u32 {
        let on = (addr_bits >> bit) & 1 != 0;
        (api.set_pixel)(bit, if on { 80 } else { 5 });
    }

    // Row 1 (px 5-9): register data bits 15-11
    for bit in 0..5u32 {
        let on = (reg_data >> (15 - bit)) & 1 != 0;
        (api.set_pixel)(5 + bit, if on { 70 } else { 3 });
    }

    // Row 2 (px 10,11,13,14): data bits 10-8, 7  -- skip 12 (heartbeat)!
    (api.set_pixel)(10, if (reg_data >> 10) & 1 != 0 { 60 } else { 3 });
    (api.set_pixel)(11, if (reg_data >> 9) & 1 != 0 { 60 } else { 3 });
    // pixel 12 = heartbeat — NEVER TOUCH
    (api.set_pixel)(13, if (reg_data >> 8) & 1 != 0 { 60 } else { 3 });
    (api.set_pixel)(14, if (reg_data >> 7) & 1 != 0 { 60 } else { 3 });

    // Row 3 (px 15-19): shifting activity — data bits 6-2
    for bit in 0..5u32 {
        let on = (reg_data >> (6 - bit)) & 1 != 0;
        (api.set_pixel)(15 + bit, if on { 50 } else { 3 });
    }

    // Row 4 (px 20-24): status indicators
    let phase = (tick / 2) % 5;
    for i in 0..5u32 {
        let bright = if i == phase { 90 } else { 5 };
        (api.set_pixel)(20 + i, bright);
    }
}

/// Scanner mode: Knight Rider sweep across all rows
fn pattern_scan(api: &MonitorApi, tick: u32) {
    let total = 24; // 24 pixels (skip 12)
    let pos = (tick % (total * 2)) as i32;
    let pos = if pos >= total as i32 { total as i32 * 2 - pos - 1 } else { pos };

    for px in 0..25u32 {
        if px == 12 { continue; } // heartbeat
        // Map pixel to sequential index (skip 12)
        let idx = if px < 12 { px } else { px - 1 };
        let dist = (idx as i32 - pos).unsigned_abs();
        let bright = if dist == 0 {
            90u32
        } else if dist <= 2 {
            40 / dist
        } else {
            3
        };
        (api.set_pixel)(px, bright);
    }
}

/// Random register data mode: reads different registers each tick
fn pattern_random(api: &MonitorApi, tick: u32, lfsr: u32) {
    for px in 0..25u32 {
        if px == 12 { continue; }
        // Combine LFSR bits with tick to make a busy pattern
        let bit_idx = px + (tick * 7);
        let on = (lfsr >> (bit_idx % 32)) & 1 != 0;
        let flicker = ((tick + px * 3) % 4) == 0;
        let bright = match (on, flicker) {
            (true, false) => 80,
            (true, true) => 40,
            (false, false) => 5,
            (false, true) => 15,
        };
        (api.set_pixel)(px, bright);
    }
}

// ── Register dump commands ─────────────────────────────────

fn dump_nvic(api: &MonitorApi) {
    let mut h = [0u8; 10];

    api.print(b"\r\n=== NVIC -- RTIC interrupt priorities ===\r\n");
    api.print(b"Enabled interrupts (ISER0..2):\r\n");

    // STM32F413 has up to 102 interrupts (ISER0..ISER3)
    for reg in 0..4u32 {
        let iser = unsafe { rd(NVIC_ISER0 + reg * 4) };
        if iser != 0 {
            api.print(b"  ISER");
            print_u32(api, reg);
            api.print(b"=");
            api.print(hex(iser, &mut h));
            api.print(b"\r\n");

            // Show individual interrupt priorities for enabled ones
            for bit in 0..32u32 {
                if iser & (1 << bit) != 0 {
                    let irq = reg * 32 + bit;
                    let prio = unsafe {
                        core::ptr::read_volatile((NVIC_IPR0 + irq) as *const u8)
                    };
                    api.print(b"    IRQ ");
                    print_u32(api, irq);
                    api.print(b": prio=");
                    print_u32(api, prio as u32);
                    // STM32F4 uses top 4 bits → logical priority
                    api.print(b" (logical ");
                    print_u32(api, (prio >> 4) as u32);
                    api.print(b")\r\n");
                }
            }
        }
    }
    api.print(b"Lower number = higher priority (RTIC convention)\r\n");
}

fn dump_scb(api: &MonitorApi) {
    let mut h = [0u8; 10];

    api.print(b"\r\n=== SCB -- system control block ===\r\n");
    unsafe {
        api.print(b"CPUID ="); api.print(hex(rd(SCB_CPUID), &mut h)); api.print(b"\r\n");
        api.print(b"ICSR  ="); api.print(hex(rd(SCB_ICSR), &mut h)); api.print(b"\r\n");
        api.print(b"VTOR  ="); api.print(hex(rd(SCB_VTOR), &mut h)); api.print(b"\r\n");
        api.print(b"AIRCR ="); api.print(hex(rd(SCB_AIRCR), &mut h)); api.print(b"\r\n");
        api.print(b"SCR   ="); api.print(hex(rd(SCB_SCR), &mut h)); api.print(b"\r\n");
        api.print(b"CCR   ="); api.print(hex(rd(SCB_CCR), &mut h)); api.print(b"\r\n");
        api.print(b"SHPR1 ="); api.print(hex(rd(SCB_SHPR1), &mut h));
        api.print(b"  (MemManage/BusFault/UsageFault)\r\n");
        api.print(b"SHPR2 ="); api.print(hex(rd(SCB_SHPR2), &mut h));
        api.print(b"  (SVCall)\r\n");
        api.print(b"SHPR3 ="); api.print(hex(rd(SCB_SHPR3), &mut h));
        api.print(b"  (PendSV/SysTick)\r\n");
        api.print(b"SHCSR ="); api.print(hex(rd(SCB_SHCSR), &mut h));
        api.print(b"  (fault enables)\r\n");
    }
}

fn dump_mpu(api: &MonitorApi) {
    let mut h = [0u8; 10];

    api.print(b"\r\n=== MPU -- memory protection ===\r\n");
    unsafe {
        let mtype = rd(MPU_TYPE);
        api.print(b"TYPE="); api.print(hex(mtype, &mut h));
        let regions = (mtype >> 8) & 0xFF;
        api.print(b"  ("); print_u32(api, regions); api.print(b" regions)\r\n");
        api.print(b"CTRL="); api.print(hex(rd(MPU_CTRL), &mut h)); api.print(b"\r\n");

        for rgn in 0..regions.min(8) {
            core::ptr::write_volatile(MPU_RNR as *mut u32, rgn);
            let rbar = rd(MPU_RBAR);
            let rasr = rd(MPU_RASR);
            if rasr & 1 != 0 { // enabled
                api.print(b"  R"); print_u32(api, rgn);
                api.print(b": base="); api.print(hex(rbar & 0xFFFF_FFE0, &mut h));
                let size_n = (rasr >> 1) & 0x1F;
                let size_bytes: u32 = 1 << (size_n + 1);
                api.print(b" size=");
                if size_bytes >= 1024 * 1024 {
                    print_u32(api, size_bytes / (1024 * 1024));
                    api.print(b"MB");
                } else if size_bytes >= 1024 {
                    print_u32(api, size_bytes / 1024);
                    api.print(b"KB");
                } else {
                    print_u32(api, size_bytes);
                    api.print(b"B");
                }
                let ap = (rasr >> 24) & 7;
                api.print(match ap {
                    0 => b" AP=none",
                    1 => b" AP=priv-RW",
                    2 => b" AP=priv-RW/unpriv-RO",
                    3 => b" AP=full-RW",
                    5 => b" AP=priv-RO",
                    6 => b" AP=RO-all",
                    _ => b" AP=???",
                });
                if rasr & (1 << 28) != 0 { api.print(b" XN"); }
                api.print(b"\r\n");
            }
        }
    }
}

fn dump_clocks(api: &MonitorApi) {
    let mut h = [0u8; 10];

    api.print(b"\r\n=== RCC -- clock tree ===\r\n");
    unsafe {
        api.print(b"CR      ="); api.print(hex(rd(RCC_CR), &mut h)); api.print(b"\r\n");
        api.print(b"CFGR    ="); api.print(hex(rd(RCC_CFGR), &mut h)); api.print(b"\r\n");
        api.print(b"AHB1ENR ="); api.print(hex(rd(RCC_AHB1ENR), &mut h)); api.print(b"\r\n");
        api.print(b"APB1ENR ="); api.print(hex(rd(RCC_APB1ENR), &mut h)); api.print(b"\r\n");
        api.print(b"APB2ENR ="); api.print(hex(rd(RCC_APB2ENR), &mut h)); api.print(b"\r\n");

        // Decode SWS (system clock source)
        let cfgr = rd(RCC_CFGR);
        let sws = (cfgr >> 2) & 3;
        api.print(b"Clock source: ");
        api.print(match sws {
            0 => b"HSI (16 MHz)\r\n",
            1 => b"HSE\r\n",
            2 => b"PLL\r\n",
            3 => b"PLL-R\r\n",
            _ => b"???\r\n",
        });
    }
}

fn dump_timers(api: &MonitorApi) {
    let mut h = [0u8; 10];

    api.print(b"\r\n=== SysTick -- RTIC Mono clock ===\r\n");
    unsafe {
        api.print(b"CSR="); api.print(hex(rd(SYSTICK_CSR), &mut h));
        api.print(b" RVR="); api.print(hex(rd(SYSTICK_RVR), &mut h));
        api.print(b" CVR="); api.print(hex(rd(SYSTICK_CVR), &mut h));
        api.print(b"\r\n");
    }

    api.print(b"\r\n=== TIM1 -- motor PWM ===\r\n");
    unsafe {
        api.print(b"CR1="); api.print(hex(rd(TIM1_BASE + TIM_CR1), &mut h));
        api.print(b" PSC="); api.print(hex(rd(TIM1_BASE + TIM_PSC), &mut h));
        api.print(b" ARR="); api.print(hex(rd(TIM1_BASE + TIM_ARR), &mut h));
        api.print(b" CNT="); api.print(hex(rd(TIM1_BASE + TIM_CNT), &mut h));
        api.print(b"\r\n");
    }

    api.print(b"\r\n=== TIM2 -- motor encoder ===\r\n");
    unsafe {
        api.print(b"CR1="); api.print(hex(rd(TIM2_BASE + TIM_CR1), &mut h));
        api.print(b" PSC="); api.print(hex(rd(TIM2_BASE + TIM_PSC), &mut h));
        api.print(b" ARR="); api.print(hex(rd(TIM2_BASE + TIM_ARR), &mut h));
        api.print(b" CNT="); api.print(hex(rd(TIM2_BASE + TIM_CNT), &mut h));
        api.print(b"\r\n");
    }
}

// ── Entry point ────────────────────────────────────────────

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 12 {
        api.print(b"ERR: need API v12+\r\n");
        return 1;
    }

    api.print(b"\r\n");
    api.print(b"  ____  ____  ____     _ _   \r\n");
    api.print(b" |  _ \\|  _ \\|  _ \\  / / |  \r\n");
    api.print(b" | |_) | | | | |_) |/ /| |  \r\n");
    api.print(b" |  __/| |_| |  __// / | |  \r\n");
    api.print(b" |_|   |____/|_|  /_/  |_|  lights\r\n");
    api.print(b"\r\n");
    api.print(b"=== PDP-11 Dashboard -- privileged register inspector ===\r\n");
    api.print(b"Commands: h=help  n=NVIC  s=SCB  m=MPU  c=clocks  t=timers\r\n");
    api.print(b"          p=pattern  q=quit\r\n");
    api.print(b"Exit: center button OR send q\r\n\r\n");

    // Verify we actually have privileged access by reading SCB
    let cpuid = unsafe { rd(SCB_CPUID) };
    if cpuid == 0 {
        api.print(b"WARN: cannot read CPUID -- may not be privileged!\r\n");
    } else {
        api.print(b"CPUID=");
        let mut h = [0u8; 10];
        api.print(hex(cpuid, &mut h));
        api.print(b" (Cortex-M4");
        let rev = cpuid & 0xF;
        let patch = (cpuid >> 20) & 0xF;
        api.print(b" r");
        print_u32(api, patch);
        api.print(b"p");
        print_u32(api, rev);
        api.print(b")\r\n");
    }

    // Debounce center button
    for _ in 0..20 {
        if (api.read_buttons)() & BTN_CENTER == 0 { break; }
        (api.delay_ms)(50);
    }

    // Initial LED state: all dim except heartbeat (untouched)
    for px in 0..25u32 {
        if px == 12 { continue; }
        (api.set_pixel)(px, 3);
    }
    (api.update_leds)();

    let mut tick: u32 = 0;
    let mut lfsr: u32 = 0xDEAD_BEEF;
    let mut mode: u8 = MODE_PDP11;

    // ── Main event loop ──────────────────────────────────
    loop {
        let evt = (api.wait_event)(
            EVT_INPUT | EVT_BUTTON,
            BLINK_MS,
        );

        // ── Input handler ──
        if evt & EVT_INPUT != 0 {
            let mut buf = [0u8; 128];
            let n = (api.read_input)(buf.as_mut_ptr(), buf.len() as u32);
            for i in 0..n as usize {
                match buf[i] {
                    b'q' | b'Q' => {
                        api.print(b"[PDP11] quit\r\n");
                        cleanup(api);
                        return tick;
                    }
                    b'h' | b'H' => {
                        api.print(b"[PDP11] commands:\r\n");
                        api.print(b"  h = help\r\n");
                        api.print(b"  n = NVIC priorities (RTIC tasks)\r\n");
                        api.print(b"  s = SCB system state\r\n");
                        api.print(b"  m = MPU sandbox config\r\n");
                        api.print(b"  c = RCC clock tree\r\n");
                        api.print(b"  t = SysTick + timer regs\r\n");
                        api.print(b"  p = cycle LED pattern\r\n");
                        api.print(b"  q = quit\r\n");
                    }
                    b'n' | b'N' => dump_nvic(api),
                    b's' | b'S' => dump_scb(api),
                    b'm' | b'M' => dump_mpu(api),
                    b'c' | b'C' => dump_clocks(api),
                    b't' | b'T' => dump_timers(api),
                    b'p' | b'P' => {
                        mode = (mode + 1) % MODE_COUNT;
                        api.print(b"[PDP11] pattern: ");
                        api.print(match mode {
                            MODE_PDP11 => b"pdp11 (register data)\r\n",
                            MODE_SCAN  => b"scanner (Knight Rider)\r\n",
                            MODE_RANDOM => b"random (LFSR chaos)\r\n",
                            _ => b"???\r\n",
                        });
                    }
                    b' ' | b'\r' | b'\n' => {}
                    c => {
                        api.print(b"[PDP11] unknown '");
                        api.print(&[c]);
                        api.print(b"' (h=help)\r\n");
                    }
                }
            }
        }

        // ── Button handler ──
        if evt & EVT_BUTTON != 0 {
            let btns = (api.read_buttons)();
            if btns & BTN_CENTER != 0 {
                api.print(b"[PDP11] center button -- exit\r\n");
                break;
            }
            if btns & BTN_LEFT != 0 {
                // Left button: beep + dump NVIC
                (api.sound_play)(880);
                (api.delay_ms)(100);
                (api.sound_stop)();
                dump_nvic(api);
            }
            if btns & BTN_RIGHT != 0 {
                // Right button: beep + dump SCB
                (api.sound_play)(660);
                (api.delay_ms)(100);
                (api.sound_stop)();
                dump_scb(api);
            }
        }

        // ── Heartbeat tick: update LED display ──
        // This fires every BLINK_MS (250ms) via EVT_TIMEOUT
        // or after any event — keeps LEDs alive either way.
        tick += 1;

        // Read a register to drive the display — cycle through sources
        let reg_source = (tick / 8) % 6;
        let reg_data = unsafe {
            match reg_source {
                0 => rd(SCB_ICSR),       // pending interrupts
                1 => rd(SYSTICK_CVR),    // SysTick counter (fast-changing)
                2 => rd(RCC_CR),         // clock control
                3 => rd(NVIC_ISER0),     // enabled interrupt mask
                4 => rd(TIM1_BASE + TIM_CNT), // TIM1 counter
                _ => rd(TIM2_BASE + TIM_CNT), // TIM2 counter
            }
        };

        // Evolve LFSR with register data for randomness
        lfsr = lfsr_step(lfsr ^ reg_data);

        // Update LED pattern
        match mode {
            MODE_PDP11 => pattern_pdp11(api, tick, reg_data),
            MODE_SCAN => pattern_scan(api, tick),
            MODE_RANDOM => pattern_random(api, tick, lfsr),
            _ => pattern_pdp11(api, tick, reg_data),
        }

        (api.update_leds)();

        // Row 4 status: show button state on px 20-21
        let btns = (api.read_buttons)();
        (api.set_pixel)(20, if btns & BTN_LEFT != 0 { 90 } else { 5 });
        (api.set_pixel)(21, if btns & BTN_CENTER != 0 { 90 } else { 5 });
        // px 22 = always off (spacer to heartbeat contrast)
        (api.set_pixel)(23, if btns & BTN_RIGHT != 0 { 90 } else { 5 });
        // px 24 = run indicator (blinks)
        (api.set_pixel)(24, if tick % 2 == 0 { 70 } else { 10 });
        (api.update_leds)();
    }

    cleanup(api);
    api.print(b"\r\nPDP-11 done, ");
    print_u32(api, tick);
    api.print(b" ticks.\r\n");
    tick
}

fn cleanup(api: &MonitorApi) {
    for px in 0..25u32 {
        if px == 12 { continue; }
        (api.set_pixel)(px, 0);
    }
    (api.update_leds)();
    (api.set_hub_led)(0, 0, 0);
    (api.sound_stop)();
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
