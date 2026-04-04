//! Register dump + motor forensics — hardware state inspection tool.
//!
//! **MUST run with `go!` (privileged mode)** — reads timer/GPIO registers
//! directly.  Compares firmware-configured peripheral state against
//! manual initialization to diagnose PWM, polarity, and pin-mux issues.
//!
//! Sequence:
//!   1. Dump ALL TIM1 and GPIOE registers (firmware's current state)
//!   2. Set motor A to 20% using the same logic as `motor::set()`
//!   3. Dump again — compare what changed
//!   4. Wait 5 s (user observes motor behavior)
//!   5. Re-init TIM1 from scratch and repeat at 20%
//!
//! Ports: motor A (TIM1 CH1/CH2 on PE9/PE11).
//! If serial becomes unresponsive after `go!`, hard-reset (lift battery).

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use spike_hub_api::MonitorApi;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! { loop {} }

const GPIOE: u32 = 0x4002_1000;
const MODER: u32 = 0x00;
const OTYPER: u32 = 0x04;
const OSPEEDR: u32 = 0x08;
const PUPDR: u32 = 0x0C;
const IDR: u32 = 0x10;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;
const AFRL: u32 = 0x20;
const AFRH: u32 = 0x24;

const TIM1: u32 = 0x4001_0000;
const CR1: u32 = 0x00;
const CR2: u32 = 0x04;
const SMCR: u32 = 0x08;
const DIER: u32 = 0x0C;
const SR: u32 = 0x10;
const EGR: u32 = 0x14;
const CCMR1: u32 = 0x18;
const CCMR2: u32 = 0x1C;
const CCER: u32 = 0x20;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const RCR: u32 = 0x30;
const CCR1: u32 = 0x34;
const CCR2: u32 = 0x38;
const CCR3: u32 = 0x3C;
const CCR4: u32 = 0x40;
const BDTR: u32 = 0x44;

#[inline(always)]
unsafe fn rd(base: u32, off: u32) -> u32 {
    core::ptr::read_volatile((base + off) as *const u32)
}

#[inline(always)]
unsafe fn wr(base: u32, off: u32, val: u32) {
    core::ptr::write_volatile((base + off) as *mut u32, val);
}

#[inline(always)]
unsafe fn rmw(base: u32, off: u32, clear: u32, set: u32) {
    let v = rd(base, off);
    wr(base, off, (v & !clear) | set);
}

fn hex(v: u32, buf: &mut [u8; 10]) -> &[u8] {
    buf[0] = b'0'; buf[1] = b'x';
    let h = b"0123456789ABCDEF";
    for i in 0..8 { buf[2+i] = h[((v >> (28-i*4)) & 0xF) as usize]; }
    &buf[..]
}

fn dump_tim1(api: &MonitorApi, label: &[u8]) {
    let mut h = [0u8; 10];
    api.print(b"\r\n=== TIM1 ");
    api.print(label);
    api.print(b" ===\r\n");
    unsafe {
        api.print(b"CR1=");   api.print(hex(rd(TIM1, CR1), &mut h));
        api.print(b" CR2=");  api.print(hex(rd(TIM1, CR2), &mut h));
        api.print(b" SMCR="); api.print(hex(rd(TIM1, SMCR), &mut h));
        api.print(b"\r\n");
        api.print(b"DIER=");  api.print(hex(rd(TIM1, DIER), &mut h));
        api.print(b" SR=");   api.print(hex(rd(TIM1, SR), &mut h));
        api.print(b"\r\n");
        api.print(b"CCMR1="); api.print(hex(rd(TIM1, CCMR1), &mut h));
        api.print(b" CCMR2="); api.print(hex(rd(TIM1, CCMR2), &mut h));
        api.print(b"\r\n");
        api.print(b"CCER=");  api.print(hex(rd(TIM1, CCER), &mut h));
        api.print(b" CNT=");  api.print(hex(rd(TIM1, CNT), &mut h));
        api.print(b"\r\n");
        api.print(b"PSC=");   api.print(hex(rd(TIM1, PSC), &mut h));
        api.print(b" ARR=");  api.print(hex(rd(TIM1, ARR), &mut h));
        api.print(b" RCR=");  api.print(hex(rd(TIM1, RCR), &mut h));
        api.print(b"\r\n");
        api.print(b"CCR1=");  api.print(hex(rd(TIM1, CCR1), &mut h));
        api.print(b" CCR2="); api.print(hex(rd(TIM1, CCR2), &mut h));
        api.print(b" CCR3="); api.print(hex(rd(TIM1, CCR3), &mut h));
        api.print(b" CCR4="); api.print(hex(rd(TIM1, CCR4), &mut h));
        api.print(b"\r\n");
        api.print(b"BDTR=");  api.print(hex(rd(TIM1, BDTR), &mut h));
        api.print(b"\r\n");
    }
}

fn dump_gpioe(api: &MonitorApi, label: &[u8]) {
    let mut h = [0u8; 10];
    api.print(b"\r\n=== GPIOE ");
    api.print(label);
    api.print(b" ===\r\n");
    unsafe {
        api.print(b"MODER=");   api.print(hex(rd(GPIOE, MODER), &mut h));
        api.print(b" OTYPER="); api.print(hex(rd(GPIOE, OTYPER), &mut h));
        api.print(b"\r\n");
        api.print(b"OSPEEDR="); api.print(hex(rd(GPIOE, OSPEEDR), &mut h));
        api.print(b" PUPDR=");  api.print(hex(rd(GPIOE, PUPDR), &mut h));
        api.print(b"\r\n");
        api.print(b"IDR=");     api.print(hex(rd(GPIOE, IDR), &mut h));
        api.print(b" ODR=");    api.print(hex(rd(GPIOE, ODR), &mut h));
        api.print(b"\r\n");
        api.print(b"AFRL=");    api.print(hex(rd(GPIOE, AFRL), &mut h));
        api.print(b" AFRH=");   api.print(hex(rd(GPIOE, AFRH), &mut h));
        api.print(b"\r\n");
    }
}

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };
    if api.version < 5 { api.print(b"ERR: need API v5+\r\n"); return 1; }

    api.print(b"\r\n========================================\r\n");
    api.print(b"  REG DUMP + MOTOR TEST\r\n");
    api.print(b"========================================\r\n");

    // ── PHASE 1: Dump current state (firmware left it) ──
    dump_tim1(api, b"CURRENT");
    dump_gpioe(api, b"CURRENT");

    // ── PHASE 2: Do exactly what motor::set(0, 20) does ──
    api.print(b"\r\n--- Applying motor::set(A, 20) logic ---\r\n");
    unsafe {
        // pin_gpio_low(GPIOE, 11) - pin2 = output LOW
        rmw(GPIOE, MODER, 3 << 22, 1 << 22);  // PE11 = output
        wr(GPIOE, BSRR, 1 << (11 + 16));       // PE11 = LOW
        // reg_write(TIM1, CCR1, 200)
        wr(TIM1, CCR1, 200);
        // pin_af_mode(GPIOE, 9, AF1)
        rmw(GPIOE, AFRH, 0xF << 4, 1 << 4);    // AFR9 = AF1
        rmw(GPIOE, MODER, 3 << 18, 2 << 18);   // PE9 = AF
    }
    api.print(b"Set: PE11=OUT/LOW, CCR1=200, PE9=AF1\r\n");

    dump_tim1(api, b"AFTER SET");
    dump_gpioe(api, b"AFTER SET");

    api.print(b"\r\n>>> MOTOR A SHOULD BE AT 20%% <<<\r\n");
    api.print(b"Waiting 5 seconds...\r\n");
    (api.delay_ms)(5000);

    // Coast
    unsafe {
        rmw(GPIOE, MODER, 3 << 18, 1 << 18);  // PE9 = output
        wr(GPIOE, BSRR, 1 << (9 + 16));        // PE9 = LOW
        rmw(GPIOE, MODER, 3 << 22, 1 << 22);  // PE11 = output
        wr(GPIOE, BSRR, 1 << (11 + 16));       // PE11 = LOW
    }
    api.print(b"Coast.\r\n");
    (api.delay_ms)(1000);

    // ── PHASE 3: Full re-init of timer then set 20% ──
    api.print(b"\r\n--- FULL TIMER REINIT + set 20%% ---\r\n");
    unsafe {
        // Stop timer
        wr(TIM1, CR1, 0);
        // Set PSC, ARR (matching firmware: PSC=7, ARR=999)
        wr(TIM1, PSC, 7);
        wr(TIM1, ARR, 999);
        // CCMR1: OC1M=PWM mode 1 (110), OC1PE=1, OC2M=PWM mode 1, OC2PE=1
        wr(TIM1, CCMR1, 0x6868);
        // CCMR2: same for CH3/CH4
        wr(TIM1, CCMR2, 0x6868);
        // CCER: enable all 4 channels, normal polarity
        wr(TIM1, CCER, 0x1111);
        // CCR all to 0
        wr(TIM1, CCR1, 0);
        wr(TIM1, CCR2, 0);
        wr(TIM1, CCR3, 0);
        wr(TIM1, CCR4, 0);
        // BDTR: MOE=1 only
        wr(TIM1, BDTR, 1 << 15);
        // Enable counter
        wr(TIM1, CR1, 1);
        // Force update
        wr(TIM1, EGR, 1);
    }
    api.print(b"Timer reinit done.\r\n");

    // Now set motor to 20%
    unsafe {
        rmw(GPIOE, MODER, 3 << 22, 1 << 22);  // PE11 = output
        wr(GPIOE, BSRR, 1 << (11 + 16));       // PE11 = LOW
        wr(TIM1, CCR1, 200);
        rmw(GPIOE, AFRH, 0xF << 4, 1 << 4);    // AFR9 = AF1
        rmw(GPIOE, MODER, 3 << 18, 2 << 18);   // PE9 = AF
    }

    dump_tim1(api, b"REINIT+SET");
    dump_gpioe(api, b"REINIT+SET");

    api.print(b"\r\n>>> MOTOR A AT 20%% (REINIT) <<<\r\n");
    api.print(b"Waiting 5 seconds...\r\n");
    (api.delay_ms)(5000);

    // Test at 5%
    unsafe { wr(TIM1, CCR1, 50); }
    api.print(b"\r\n>>> MOTOR A AT 5%% <<<\r\n");
    (api.delay_ms)(3000);

    // Test at 50%
    unsafe { wr(TIM1, CCR1, 500); }
    api.print(b"\r\n>>> MOTOR A AT 50%% <<<\r\n");
    (api.delay_ms)(3000);

    // Coast
    unsafe {
        rmw(GPIOE, MODER, 3 << 18, 1 << 18);
        wr(GPIOE, BSRR, 1 << (9 + 16));
        rmw(GPIOE, MODER, 3 << 22, 1 << 22);
        wr(GPIOE, BSRR, 1 << (11 + 16));
    }

    api.print(b"\r\n========================================\r\n");
    api.print(b"DONE\r\n");
    api.print(b"========================================\r\n");
    0
}
