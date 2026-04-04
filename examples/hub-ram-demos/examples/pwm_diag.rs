//! PWM diagnostic test - compares fast-decay vs slow-decay (Pybricks-style).
//!
//! **MUST run with `go!` (privileged mode)** - accesses timer/GPIO registers
//! directly.  If serial becomes unresponsive, hard-reset the hub (lift battery).
//!
//! Tests Port A and Port B (motors on TIM1 CH1/CH2 and CH3/CH4).
//! Sweeps duty cycles from 5% to 100% in both decay modes.
//! Reports each step clearly; user observes motor behavior.
//!
//! Build:
//!   cd hub-ram-demos && cargo build --release --example pwm_diag
//!   arm-none-eabi-objcopy -O binary \
//!       target/thumbv7em-none-eabihf/release/examples/pwm_diag pwm_diag.bin
//!
//! Upload & run:
//!   python3 ../../helper-tools/upload_demo.py pwm_diag.bin
//!   # or: python3 ../../helper-tools/spike_hub_controller.py upload pwm_diag.bin

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use spike_hub_api::MonitorApi;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {}
}

// -- Register addresses --

const GPIOE: u32 = 0x4002_1000;
const MODER: u32 = 0x00;
const BSRR: u32 = 0x18;
const AFRL: u32 = 0x20;
const AFRH: u32 = 0x24;

const TIM1: u32 = 0x4001_0000;
const CR1: u32 = 0x00;
const EGR: u32 = 0x14;
const CCMR1: u32 = 0x18;
const CCMR2: u32 = 0x1C;
const CCER: u32 = 0x20;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const CCR1: u32 = 0x34;
const CCR2: u32 = 0x38;
const CCR3: u32 = 0x3C;
const CCR4: u32 = 0x40;
const BDTR: u32 = 0x44;

const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_APB2ENR: u32 = 0x44;

const PWM_PERIOD: u32 = 1000;

// -- Register helpers --

#[inline(always)]
unsafe fn wr(base: u32, off: u32, val: u32) {
    core::ptr::write_volatile((base + off) as *mut u32, val);
}

#[inline(always)]
unsafe fn rd(base: u32, off: u32) -> u32 {
    core::ptr::read_volatile((base + off) as *const u32)
}

#[inline(always)]
unsafe fn rmw(base: u32, off: u32, clear: u32, set: u32) {
    let v = rd(base, off);
    wr(base, off, (v & !clear) | set);
}

// -- GPIO helpers --

unsafe fn gpio_low(port: u32, pin: u32) {
    rmw(port, MODER, 3 << (pin * 2), 1 << (pin * 2));
    wr(port, BSRR, 1 << (pin + 16));
}

unsafe fn gpio_high(port: u32, pin: u32) {
    rmw(port, MODER, 3 << (pin * 2), 1 << (pin * 2));
    wr(port, BSRR, 1 << pin);
}

unsafe fn gpio_af(port: u32, pin: u32, af: u32) {
    if pin < 8 {
        let s = pin * 4;
        rmw(port, AFRL, 0xF << s, af << s);
    } else {
        let s = (pin - 8) * 4;
        rmw(port, AFRH, 0xF << s, af << s);
    }
    rmw(port, MODER, 3 << (pin * 2), 2 << (pin * 2));
}

// -- Timer setup --

unsafe fn init_tim1() {
    rmw(RCC, RCC_AHB1ENR, 0, 1 << 4);
    rmw(RCC, RCC_APB2ENR, 0, 1 << 0);
    let _ = rd(RCC, RCC_APB2ENR);

    wr(TIM1, CR1, 0);
    wr(TIM1, PSC, 7);
    wr(TIM1, ARR, PWM_PERIOD);

    wr(TIM1, CCMR1, (0b110 << 4) | (1 << 3) | (0b110 << 12) | (1 << 11));
    wr(TIM1, CCMR2, (0b110 << 4) | (1 << 3) | (0b110 << 12) | (1 << 11));

    wr(TIM1, CCER, (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12));

    wr(TIM1, CCR1, 0);
    wr(TIM1, CCR2, 0);
    wr(TIM1, CCR3, 0);
    wr(TIM1, CCR4, 0);

    rmw(TIM1, BDTR, 0, 1 << 15);
    rmw(TIM1, CR1, 0, 1 << 0);
    wr(TIM1, EGR, 1 << 0);
}

// -- Coast --

unsafe fn coast_a() {
    gpio_low(GPIOE, 9);
    gpio_low(GPIOE, 11);
}

unsafe fn coast_b() {
    gpio_low(GPIOE, 13);
    gpio_low(GPIOE, 14);
}

// -- Formatting --

fn fmt_u32(v: u32, buf: &mut [u8; 12]) -> &[u8] {
    if v == 0 {
        buf[0] = b'0';
        return &buf[..1];
    }
    let mut i = 12usize;
    let mut n = v;
    while n > 0 && i > 0 {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    &buf[i..]
}

fn fmt_hex(v: u32, buf: &mut [u8; 10]) -> &[u8] {
    buf[0] = b'0';
    buf[1] = b'x';
    let hex = b"0123456789ABCDEF";
    for i in 0..8 {
        buf[2 + i] = hex[((v >> (28 - i * 4)) & 0xF) as usize];
    }
    &buf[..]
}

// -- Dump timer --

fn dump_regs(api: &MonitorApi) {
    let mut h = [0u8; 10];
    unsafe {
        api.print(b"  TIM1: CR1=");
        api.print(fmt_hex(rd(TIM1, CR1), &mut h));
        api.print(b" PSC=");
        api.print(fmt_hex(rd(TIM1, PSC), &mut h));
        api.print(b" ARR=");
        api.print(fmt_hex(rd(TIM1, ARR), &mut h));
        api.print(b"\r\n  CCER=");
        api.print(fmt_hex(rd(TIM1, CCER), &mut h));
        api.print(b" CCMR1=");
        api.print(fmt_hex(rd(TIM1, CCMR1), &mut h));
        api.print(b" CCMR2=");
        api.print(fmt_hex(rd(TIM1, CCMR2), &mut h));
        api.print(b"\r\n  CCR1=");
        api.print(fmt_hex(rd(TIM1, CCR1), &mut h));
        api.print(b" CCR2=");
        api.print(fmt_hex(rd(TIM1, CCR2), &mut h));
        api.print(b" CCR3=");
        api.print(fmt_hex(rd(TIM1, CCR3), &mut h));
        api.print(b" CCR4=");
        api.print(fmt_hex(rd(TIM1, CCR4), &mut h));
        api.print(b"\r\n  BDTR=");
        api.print(fmt_hex(rd(TIM1, BDTR), &mut h));
        api.print(b" GPIOE_MODER=");
        api.print(fmt_hex(rd(GPIOE, MODER), &mut h));
        api.print(b"\r\n");
    }
}

// -- Polarity helper --

unsafe fn set_ccxp(ccer_bit: u32, invert: bool) {
    let p = ccer_bit + 1;
    if invert {
        rmw(TIM1, CCER, 0, 1 << p);
    } else {
        rmw(TIM1, CCER, 1 << p, 0);
    }
}

// -- ENTRY --

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    if api.version < 5 {
        api.print(b"ERR: need API v5+\r\n");
        return 1;
    }

    api.print(b"\r\n");
    api.print(b"================================================\r\n");
    api.print(b"  PWM DIAGNOSTIC - Fast-Decay vs Slow-Decay\r\n");
    api.print(b"  Port A: PE9/PE11 (TIM1 CH1/CH2)\r\n");
    api.print(b"  Port B: PE13/PE14 (TIM1 CH3/CH4)\r\n");
    api.print(b"  ** RUN WITH go! (privileged) **\r\n");
    api.print(b"================================================\r\n\r\n");

    api.print(b"[INIT] Reinitializing TIM1...\r\n");
    unsafe { init_tim1(); coast_a(); coast_b(); }

    api.print(b"[INIT] Timer state:\r\n");
    dump_regs(api);
    api.print(b"\r\n");

    let hold: u32 = 2000;
    let mut nb = [0u8; 12];

    // ====== TEST 1: GPIO direct ======
    api.print(b"------------------------------------------------\r\n");
    api.print(b"TEST 1: GPIO DIRECT (no PWM) - full voltage\r\n");

    api.print(b"  A fwd (PE9=H PE11=L)\r\n");
    unsafe { gpio_high(GPIOE, 9); gpio_low(GPIOE, 11); }
    (api.delay_ms)(hold);
    unsafe { coast_a(); }
    (api.delay_ms)(500);

    api.print(b"  A rev (PE9=L PE11=H)\r\n");
    unsafe { gpio_low(GPIOE, 9); gpio_high(GPIOE, 11); }
    (api.delay_ms)(hold);
    unsafe { coast_a(); }
    (api.delay_ms)(500);

    api.print(b"  B fwd (PE13=H PE14=L)\r\n");
    unsafe { gpio_high(GPIOE, 13); gpio_low(GPIOE, 14); }
    (api.delay_ms)(hold);
    unsafe { coast_b(); }
    (api.delay_ms)(500);

    api.print(b"  B rev (PE13=L PE14=H)\r\n");
    unsafe { gpio_low(GPIOE, 13); gpio_high(GPIOE, 14); }
    (api.delay_ms)(hold);
    unsafe { coast_b(); }
    (api.delay_ms)(1000);

    // ====== TEST 2: FAST-DECAY ======
    api.print(b"------------------------------------------------\r\n");
    api.print(b"TEST 2: FAST-DECAY (pin1=PWM norm, pin2=LOW)\r\n");
    api.print(b"  Off: both LOW = coast (BAD at low duty)\r\n\r\n");

    unsafe {
        set_ccxp(0, false);
        set_ccxp(4, false);
        set_ccxp(8, false);
        set_ccxp(12, false);
    }

    for &duty in &[5u32, 10, 20, 30, 50, 75, 100] {
        let ccr = duty * PWM_PERIOD / 100;
        api.print(b"  [FAST] A fwd ");
        api.print(fmt_u32(duty, &mut nb));
        api.print(b"% CCR=");
        api.print(fmt_u32(ccr, &mut nb));
        api.print(b" CCxP=0 pin2=LOW\r\n");

        unsafe {
            gpio_low(GPIOE, 11);
            wr(TIM1, CCR1, ccr);
            gpio_af(GPIOE, 9, 1);
        }
        (api.delay_ms)(hold);
        unsafe { coast_a(); }
        (api.delay_ms)(500);
    }
    api.print(b"\r\n");

    // ====== TEST 3: SLOW-DECAY (Pybricks) ======
    api.print(b"------------------------------------------------\r\n");
    api.print(b"TEST 3: SLOW-DECAY (pin1=PWM inv, pin2=HIGH)\r\n");
    api.print(b"  Off: both HIGH = brake (GOOD at low duty)\r\n\r\n");

    for &duty in &[5u32, 10, 15, 20, 25, 30, 50, 75, 100] {
        let ccr = duty * PWM_PERIOD / 100;
        api.print(b"  [SLOW] A fwd ");
        api.print(fmt_u32(duty, &mut nb));
        api.print(b"% CCR=");
        api.print(fmt_u32(ccr, &mut nb));
        api.print(b" CCxP=1 pin2=HIGH\r\n");

        unsafe {
            set_ccxp(0, true);
            gpio_high(GPIOE, 11);
            wr(TIM1, CCR1, ccr);
            gpio_af(GPIOE, 9, 1);
        }
        if duty == 5 { dump_regs(api); }
        (api.delay_ms)(hold);
        unsafe { coast_a(); set_ccxp(0, false); }
        (api.delay_ms)(500);
    }
    api.print(b"\r\n");

    // ====== TEST 4: SLOW-DECAY Port B ======
    api.print(b"------------------------------------------------\r\n");
    api.print(b"TEST 4: SLOW-DECAY Port B (CH3/CH4)\r\n\r\n");

    for &duty in &[10u32, 20, 50, 100] {
        let ccr = duty * PWM_PERIOD / 100;
        api.print(b"  [SLOW-B] B fwd ");
        api.print(fmt_u32(duty, &mut nb));
        api.print(b"% CCR=");
        api.print(fmt_u32(ccr, &mut nb));
        api.print(b"\r\n");

        unsafe {
            set_ccxp(8, true);
            gpio_high(GPIOE, 14);
            wr(TIM1, CCR3, ccr);
            gpio_af(GPIOE, 13, 1);
        }
        (api.delay_ms)(hold);
        unsafe { coast_b(); set_ccxp(8, false); }
        (api.delay_ms)(500);
    }
    api.print(b"\r\n");

    // ====== TEST 5: SLOW-DECAY reverse ======
    api.print(b"------------------------------------------------\r\n");
    api.print(b"TEST 5: SLOW-DECAY REVERSE Port A\r\n\r\n");

    for &duty in &[10u32, 25, 50, 75] {
        let ccr = duty * PWM_PERIOD / 100;
        api.print(b"  [SLOW-R] A rev ");
        api.print(fmt_u32(duty, &mut nb));
        api.print(b"% CCR=");
        api.print(fmt_u32(ccr, &mut nb));
        api.print(b"\r\n");

        unsafe {
            set_ccxp(4, true);
            gpio_high(GPIOE, 9);
            wr(TIM1, CCR2, ccr);
            gpio_af(GPIOE, 11, 1);
        }
        (api.delay_ms)(hold);
        unsafe { coast_a(); set_ccxp(4, false); }
        (api.delay_ms)(500);
    }
    api.print(b"\r\n");

    // ====== TEST 6: Low duty sweep ======
    api.print(b"------------------------------------------------\r\n");
    api.print(b"TEST 6: LOW SWEEP slow-decay 1-15% Port A\r\n\r\n");

    for duty in [1u32, 2, 3, 5, 7, 10, 12, 15] {
        let ccr = duty * PWM_PERIOD / 100;
        api.print(b"  [LOW] ");
        api.print(fmt_u32(duty, &mut nb));
        api.print(b"% CCR=");
        api.print(fmt_u32(ccr, &mut nb));
        api.print(b"\r\n");

        unsafe {
            set_ccxp(0, true);
            gpio_high(GPIOE, 11);
            wr(TIM1, CCR1, ccr);
            gpio_af(GPIOE, 9, 1);
        }
        (api.delay_ms)(1500);
        unsafe { coast_a(); set_ccxp(0, false); }
        (api.delay_ms)(500);
    }
    api.print(b"\r\n");

    // ====== DONE ======
    unsafe { coast_a(); coast_b(); }

    api.print(b"================================================\r\n");
    api.print(b"ALL TESTS COMPLETE\r\n\r\n");
    api.print(b"EXPECTED:\r\n");
    api.print(b" TEST 1 (GPIO):  Strong spin both dirs\r\n");
    api.print(b" TEST 2 (FAST):  Dead below ~50%%\r\n");
    api.print(b" TEST 3 (SLOW):  Spins at 5-10%%\r\n");
    api.print(b" TEST 4 (SLOW-B):Same on Port B\r\n");
    api.print(b" TEST 5 (REV):   Reverse at 10%%\r\n");
    api.print(b" TEST 6 (LOW):   Min working duty\r\n\r\n");
    api.print(b"If SLOW works -> fix motor.rs:\r\n");
    api.print(b"  CCxP=1 (inverted) + pin2=HIGH\r\n");
    api.print(b"================================================\r\n");

    0
}
