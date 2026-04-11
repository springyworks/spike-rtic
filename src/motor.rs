#![allow(dead_code)]
//! Motor driver for LEGO SPIKE Prime Hub.
//!
//! Supports all 6 motor ports (A–F) using H-bridge PWM via TIM1/TIM3/TIM4.
//! Uses Pybricks-style **slow-decay** drive (matches lib/pbio/drv/motor_driver):
//!   - PWM with inverted polarity + other pin HIGH
//!   - Drive phase: PWM output LOW, other HIGH → motor runs
//!   - Brake phase: PWM output HIGH, other HIGH → electromagnetic braking
//!   - Coast: both pins LOW
//!
//! ## Timer mapping
//!   - TIM1 (APB2): Port A (CH1/CH2), Port B (CH3/CH4)
//!   - TIM3 (APB1): Port E (CH1/CH2), Port F (CH3/CH4)
//!   - TIM4 (APB1): Port C (CH1/CH2), Port D (CH3/CH4)
//!
//! PWM frequency: 12 kHz (PSC=7, ARR=999 at 96 MHz).

use crate::pins::{self, MotorPort};
use crate::reg::{reg_modify, reg_read, reg_write};

// RCC
const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_APB1ENR: u32 = 0x40;
const RCC_APB2ENR: u32 = 0x44;

// Timer register offsets
const CR1: u32 = 0x00;
const CCMR1: u32 = 0x18;
const CCMR2: u32 = 0x1C;
const CCER: u32 = 0x20;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const BDTR: u32 = 0x44;
const EGR: u32 = 0x14;

const PWM_ARR: u32 = 999;

/// All motor port definitions, indexed 0=A through 5=F.
const PORTS: [&MotorPort; 6] = [
    &pins::MOTOR_PORT_A,
    &pins::MOTOR_PORT_B,
    &pins::MOTOR_PORT_C,
    &pins::MOTOR_PORT_D,
    &pins::MOTOR_PORT_E,
    &pins::MOTOR_PORT_F,
];

/// Convert port letter ('a'–'f' or 'A'–'F') to port index 0–5.
pub fn port_index(ch: u8) -> Option<usize> {
    match ch {
        b'a'..=b'f' => Some((ch - b'a') as usize),
        b'A'..=b'F' => Some((ch - b'A') as usize),
        _ => None,
    }
}

/// Get the AF number for a timer.
/// TIM1 = AF1, TIM3 = AF2, TIM4 = AF2.
fn timer_af(timer_base: u32) -> u32 {
    if timer_base == pins::TIM1_BASE { 1 } else { 2 }
}

/// Configure one GPIO pin as alternate-function for its timer.
unsafe fn configure_pin_af(port: u32, pin: u32, af: u32) {
    // MODER: set pin to AF mode (0b10)
    reg_modify(port, pins::MODER, 3 << (pin * 2), 2 << (pin * 2));

    // AFR: set alternate function
    if pin < 8 {
        let shift = pin * 4;
        reg_modify(port, pins::AFRL, 0xF << shift, af << shift);
    } else {
        let shift = (pin - 8) * 4;
        reg_modify(port, pins::AFRH, 0xF << shift, af << shift);
    }
}

/// Determine which CCMR register and bit offsets to use based on the
/// CCR offset (0x34=CH1, 0x38=CH2, 0x3C=CH3, 0x40=CH4).
fn channel_config(ccr_offset: u32) -> (u32, u32, u32) {
    // Returns (ccmr_offset, ocm_shift, ocpe_bit, ccer_bit)
    match ccr_offset {
        0x34 => (CCMR1, 4, 0),   // CH1: OC1M[6:4], OC1PE=bit3, CC1E=bit0
        0x38 => (CCMR1, 12, 4),  // CH2: OC2M[14:12], OC2PE=bit11, CC2E=bit4
        0x3C => (CCMR2, 4, 8),   // CH3: OC3M[6:4], OC3PE=bit3, CC3E=bit8
        0x40 => (CCMR2, 12, 12), // CH4: OC4M[14:12], OC4PE=bit11, CC4E=bit12
        _ => (CCMR1, 4, 0),      // fallback
    }
}

/// Initialize a single timer for PWM. Safe to call multiple times
/// (TIM1 shared by ports A+B, TIM3 by E+F, TIM4 by C+D).
unsafe fn init_timer(timer: u32) {
    // Enable timer clock
    if timer == pins::TIM1_BASE {
        reg_modify(RCC, RCC_APB2ENR, 0, 1 << 0); // TIM1 on APB2
    } else if timer == pins::TIM3_BASE {
        reg_modify(RCC, RCC_APB1ENR, 0, 1 << 1); // TIM3 on APB1
    } else if timer == pins::TIM4_BASE {
        reg_modify(RCC, RCC_APB1ENR, 0, 1 << 2); // TIM4 on APB1
    }
    let _ = reg_read(RCC, RCC_APB2ENR); // settle

    // PSC: APB2 timers (TIM1) run at 96 MHz, APB1 timers (TIM3/4) at
    // 48 MHz but with x2 multiplier = 96 MHz.
    // PSC=7 → 12 MHz timer clock. With ARR=999 → 12 kHz PWM.
    // (Matches Pybricks; above audible range, eliminates motor whine.)
    reg_write(timer, PSC, 7);
    reg_write(timer, ARR, PWM_ARR);

    // For TIM1 (advanced): enable Main Output Enable (MOE)
    if timer == pins::TIM1_BASE {
        reg_modify(timer, BDTR, 0, 1 << 15);
    }

    // Enable counter
    reg_modify(timer, CR1, 0, 1 << 0);

    // Force update event so PSC/ARR/CCRx preloads take effect
    reg_write(timer, EGR, 1 << 0); // UG bit
}

/// Configure one PWM channel on a timer: set PWM mode 1, enable preload,
/// enable output with inverted polarity, and set initial CCR to 0.
///
/// Pybricks-style slow-decay drive:
///   - PWM mode 1 + inverted polarity (CCxP=1):
///     CNT < CCR → output LOW  (drive phase)
///     CNT >= CCR → output HIGH (brake phase, with other pin also HIGH)
///   - Paired with GPIO HIGH on the other H-bridge leg:
///     Drive phase: PWM=LOW, other=HIGH → current flows → motor drives
///     Brake phase: PWM=HIGH, other=HIGH → both HIGH → slow-decay brake
///   - This matches the Pybricks motor driver exactly.
unsafe fn init_channel(timer: u32, ccr_offset: u32) {
    let (ccmr, ocm_shift, ccer_bit) = channel_config(ccr_offset);

    // OCxM = PWM mode 1 (0b110), OCxPE = 1 (preload)
    let ocpe_bit = ocm_shift - 1;
    reg_modify(timer, ccmr, 7 << ocm_shift, (0b110 << ocm_shift) | (1 << ocpe_bit));

    // CCxE = 1 (enable output), CCxP = 1 (inverted polarity = Pybricks slow-decay).
    reg_modify(timer, CCER, 3 << ccer_bit, (1 << ccer_bit) | (1 << (ccer_bit + 1)));

    // CCRx = 0 (stopped — with inverted polarity, CCR=0 means always HIGH = idle)
    reg_write(timer, ccr_offset, 0);
}

/// Initialize all motor port hardware.
///
/// Enables GPIO clocks, configures pins as alternate-function,
/// sets up TIM1/TIM3/TIM4 for PWM at 1 kHz, all channels at 0 (stopped).
pub unsafe fn init() {
    // Enable all needed GPIO clocks: A, B, C, E
    reg_modify(RCC, RCC_AHB1ENR, 0, (1 << 0) | (1 << 1) | (1 << 2) | (1 << 4));

    // Initialize timers (each called once despite sharing)
    init_timer(pins::TIM1_BASE);
    init_timer(pins::TIM3_BASE);
    init_timer(pins::TIM4_BASE);

    // Configure each port's channels and start in coast (both pins LOW)
    for port in PORTS.iter() {
        init_channel(port.timer, port.ccr1_offset);
        init_channel(port.timer, port.ccr2_offset);
        // Coast: both motor pins as GPIO output LOW
        pin_gpio_low(port.pwm1_port, port.pwm1_pin);
        pin_gpio_low(port.pwm2_port, port.pwm2_pin);
    }
}

/// Set a GPIO pin to output mode and drive it HIGH.
unsafe fn pin_gpio_high(port: u32, pin: u32) {
    // MODER: set to output (0b01)
    reg_modify(port, pins::MODER, 3 << (pin * 2), 1 << (pin * 2));
    // BSRR: set pin high
    reg_write(port, pins::BSRR, 1 << pin);
}

/// Set a GPIO pin to output mode and drive it LOW.
unsafe fn pin_gpio_low(port: u32, pin: u32) {
    // MODER: set to output (0b01)
    reg_modify(port, pins::MODER, 3 << (pin * 2), 1 << (pin * 2));
    // BSRR: reset pin (set high half)
    reg_write(port, pins::BSRR, 1 << (pin + 16));
}

/// Set a GPIO pin to alternate-function mode for PWM.
unsafe fn pin_af_mode(port: u32, pin: u32, af: u32) {
    // AFR
    if pin < 8 {
        let shift = pin * 4;
        reg_modify(port, pins::AFRL, 0xF << shift, af << shift);
    } else {
        let shift = (pin - 8) * 4;
        reg_modify(port, pins::AFRH, 0xF << shift, af << shift);
    }
    // MODER: set to AF (0b10)
    reg_modify(port, pins::MODER, 3 << (pin * 2), 2 << (pin * 2));
}

/// Set motor speed on a port.
///
/// Pybricks-style slow-decay H-bridge drive:
///   - Forward: pin1=AF(PWM, inverted polarity), pin2=GPIO HIGH
///   - Reverse: pin1=GPIO HIGH, pin2=AF(PWM, inverted polarity)
///   - Coast (speed=0): both pins GPIO LOW
///
/// PWM mode 1 + inverted polarity:
///   CNT < CCR → output LOW  (drive: PWM=LOW, other=HIGH → motor runs)
///   CNT >= CCR → output HIGH (brake: PWM=HIGH, other=HIGH → slow-decay)
///
/// This matches the Pybricks motor driver exactly (lib/pbio/drv/motor_driver
/// /motor_driver_hbridge_pwm.c).  Slow-decay provides strong holding torque
/// at low duty cycles — the motor brakes during the off-phase instead of
/// coasting, delivering smooth controlled motion at all speeds.
///
/// - `port_idx`: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F
/// - `speed`: -100 to +100 (positive=forward, negative=reverse, 0=coast)
pub fn set(port_idx: u32, speed: i32) {
    if port_idx >= 6 { return; }
    let mp = PORTS[port_idx as usize];
    let af = timer_af(mp.timer);

    let duty = speed.unsigned_abs().min(100);
    let ccr = duty * (PWM_ARR + 1) / 100;

    unsafe {
        if speed > 0 {
            // Forward: pin1 = PWM (inverted), pin2 = GPIO HIGH
            pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
            reg_write(mp.timer, mp.ccr1_offset, ccr);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
        } else if speed < 0 {
            // Reverse: pin1 = GPIO HIGH, pin2 = PWM (inverted)
            pin_gpio_high(mp.pwm1_port, mp.pwm1_pin);
            reg_write(mp.timer, mp.ccr2_offset, ccr);
            pin_af_mode(mp.pwm2_port, mp.pwm2_pin, af);
        } else {
            // Coast: both pins LOW
            pin_gpio_low(mp.pwm1_port, mp.pwm1_pin);
            pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        }
    }
}

/// Read motor diagnostic info: timer registers + GPIO states.
/// Returns (PSC, ARR, CCR1, CCR2, CCER, CR1, pin1_moder, pin2_moder).
pub fn diag(port_idx: u32) -> (u32, u32, u32, u32, u32, u32, u32, u32) {
    if port_idx >= 6 { return (0,0,0,0,0,0,0,0); }
    let mp = PORTS[port_idx as usize];
    unsafe {
        let psc = reg_read(mp.timer, PSC);
        let arr = reg_read(mp.timer, ARR);
        let ccr1 = reg_read(mp.timer, mp.ccr1_offset);
        let ccr2 = reg_read(mp.timer, mp.ccr2_offset);
        let ccer = reg_read(mp.timer, CCER);
        let cr1 = reg_read(mp.timer, CR1);
        let moder1 = (reg_read(mp.pwm1_port, pins::MODER) >> (mp.pwm1_pin * 2)) & 3;
        let moder2 = (reg_read(mp.pwm2_port, pins::MODER) >> (mp.pwm2_pin * 2)) & 3;
        (psc, arr, ccr1, ccr2, ccer, cr1, moder1, moder2)
    }
}

/// Brake motor on a port (both H-bridge pins driven HIGH).
///
/// - `port_idx`: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F
pub fn brake(port_idx: u32) {
    if port_idx >= 6 { return; }
    let mp = PORTS[port_idx as usize];
    unsafe {
        pin_gpio_high(mp.pwm1_port, mp.pwm1_pin);
        pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
    }
}

// ── Trial test infrastructure ──────────────────────────────
//
// Systematic PWM configuration trials.  Each trial configures the
// H-bridge differently and reports what was set.  The user observes
// the motor and reports results.

/// Output callback type for trial messages.
pub type TrialPrint = fn(&[u8]);

/// Run a diagnostic trial sequence on one motor port.
///
/// Goes through 14 stages, each lasting `hold_ms`.  Between each stage
/// the motor is coasted for 500 ms.  The `print` callback receives
/// ASCII status lines.
///
/// Returns after all stages complete.
pub fn trial(port_idx: u32, hold_ms: u32, print: TrialPrint) {
    if port_idx >= 6 { return; }
    let mp = PORTS[port_idx as usize];
    let af = timer_af(mp.timer);
    let timer = mp.timer;

    // Helper: coast (both LOW), wait
    let coast = || unsafe {
        pin_gpio_low(mp.pwm1_port, mp.pwm1_pin);
        pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        crate::power::delay_ms(500);
    };

    // Helper: set CCxP bit (inverted polarity) on a channel
    let set_ccxp = |ccr_off: u32, invert: bool| unsafe {
        let (_, _, ccer_bit) = channel_config(ccr_off);
        if invert {
            // Set CCxP (bit ccer_bit+1)
            reg_modify(timer, CCER, 0, 1 << (ccer_bit + 1));
        } else {
            // Clear CCxP
            reg_modify(timer, CCER, 1 << (ccer_bit + 1), 0);
        }
    };

    let _buf = [0u8; 80];

    // ── Stage 1: GPIO direct drive (no PWM) — forward ──
    {
        let msg = b"[1/14] GPIO direct: pin1=HIGH pin2=LOW (full voltage)\r\n";
        print(msg);
        unsafe {
            pin_gpio_high(mp.pwm1_port, mp.pwm1_pin);
            pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
    }

    // ── Stage 2: GPIO direct drive — reverse ──
    {
        let msg = b"[2/14] GPIO direct: pin1=LOW pin2=HIGH (reverse)\r\n";
        print(msg);
        unsafe {
            pin_gpio_low(mp.pwm1_port, mp.pwm1_pin);
            pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
    }

    // ── Stage 3: Pybricks-style slow-decay 100% (inverted PWM + pin2=HIGH) ──
    {
        let msg = b"[3/14] PYBRICKS: CCxP=1 pin2=HIGH CCR=1000/1000 (100%)\r\n";
        print(msg);
        unsafe {
            set_ccxp(mp.ccr1_offset, true); // invert polarity
            reg_write(timer, mp.ccr1_offset, 1000);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
        set_ccxp(mp.ccr1_offset, false); // restore
    }

    // ── Stage 4: Pybricks-style slow-decay 50% ──
    {
        let msg = b"[4/14] PYBRICKS: CCxP=1 pin2=HIGH CCR=500/1000 (50%)\r\n";
        print(msg);
        unsafe {
            set_ccxp(mp.ccr1_offset, true);
            reg_write(timer, mp.ccr1_offset, 500);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
        set_ccxp(mp.ccr1_offset, false);
    }

    // ── Stage 5: Pybricks-style slow-decay 25% ──
    {
        let msg = b"[5/14] PYBRICKS: CCxP=1 pin2=HIGH CCR=250/1000 (25%)\r\n";
        print(msg);
        unsafe {
            set_ccxp(mp.ccr1_offset, true);
            reg_write(timer, mp.ccr1_offset, 250);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
        set_ccxp(mp.ccr1_offset, false);
    }

    // ── Stage 6: Pybricks-style slow-decay 10% ──
    {
        let msg = b"[6/14] PYBRICKS: CCxP=1 pin2=HIGH CCR=100/1000 (10%)\r\n";
        print(msg);
        unsafe {
            set_ccxp(mp.ccr1_offset, true);
            reg_write(timer, mp.ccr1_offset, 100);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
        set_ccxp(mp.ccr1_offset, false);
    }

    // ── Stage 7: Fast-decay (our current) 100% (normal PWM + pin2=LOW) ──
    {
        let msg = b"[7/14] FAST-DECAY: CCxP=0 pin2=LOW CCR=1000/1000 (100%)\r\n";
        print(msg);
        unsafe {
            reg_write(timer, mp.ccr1_offset, 1000);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
    }

    // ── Stage 8: Fast-decay 50% ──
    {
        let msg = b"[8/14] FAST-DECAY: CCxP=0 pin2=LOW CCR=500/1000 (50%)\r\n";
        print(msg);
        unsafe {
            reg_write(timer, mp.ccr1_offset, 500);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
    }

    // ── Stage 9: Fast-decay 25% ──
    {
        let msg = b"[9/14] FAST-DECAY: CCxP=0 pin2=LOW CCR=250/1000 (25%)\r\n";
        print(msg);
        unsafe {
            reg_write(timer, mp.ccr1_offset, 250);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
    }

    // ── Stage 10: Fast-decay 10% ──
    {
        let msg = b"[10/14] FAST-DECAY: CCxP=0 pin2=LOW CCR=100/1000 (10%)\r\n";
        print(msg);
        unsafe {
            reg_write(timer, mp.ccr1_offset, 100);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
    }

    // ── Stage 11: Slow-decay with normal polarity (pin2=HIGH, CCxP=0) ──
    // If motor drives OPPOSITE to Pybricks stages, polarity logic differs.
    {
        let msg = b"[11/14] SLOW-NORM: CCxP=0 pin2=HIGH CCR=500/1000 (50%?)\r\n";
        print(msg);
        unsafe {
            reg_write(timer, mp.ccr1_offset, 500);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
    }

    // ── Stage 12: Low freq test — PSC=95 (1 kHz PWM), 50% ──
    {
        let msg = b"[12/14] LOW-FREQ: PSC=95 ARR=999 (1kHz) CCR=500 pin2=LOW\r\n";
        print(msg);
        unsafe {
            reg_write(timer, PSC, 95);
            reg_write(timer, EGR, 1); // force update
            reg_write(timer, mp.ccr1_offset, 500);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
        // Restore PSC
        unsafe {
            reg_write(timer, PSC, 7);
            reg_write(timer, EGR, 1);
        }
    }

    // ── Stage 13: Very low freq — PSC=959 (100 Hz PWM), 50% ──
    {
        let msg = b"[13/14] VLOW-FREQ: PSC=959 ARR=999 (100Hz) CCR=500 pin2=LOW\r\n";
        print(msg);
        unsafe {
            reg_write(timer, PSC, 959);
            reg_write(timer, EGR, 1);
            reg_write(timer, mp.ccr1_offset, 500);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_low(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
        unsafe {
            reg_write(timer, PSC, 7);
            reg_write(timer, EGR, 1);
        }
    }

    // ── Stage 14: Pybricks exact clone — PSC=7, ARR=1000, CCxP=1 ──
    // period=1000 (not 999!), prescalar=8 stored as PSC=7
    {
        let msg = b"[14/14] PYBRICKS-EXACT: PSC=7 ARR=1000 CCxP=1 CCR=100 (10%)\r\n";
        print(msg);
        unsafe {
            reg_write(timer, ARR, 1000);
            reg_write(timer, EGR, 1);
            set_ccxp(mp.ccr1_offset, true);
            reg_write(timer, mp.ccr1_offset, 100);
            pin_af_mode(mp.pwm1_port, mp.pwm1_pin, af);
            pin_gpio_high(mp.pwm2_port, mp.pwm2_pin);
        }
        crate::power::delay_ms(hold_ms);
        coast();
        // Restore
        unsafe {
            set_ccxp(mp.ccr1_offset, false);
            reg_write(timer, ARR, PWM_ARR);
            reg_write(timer, EGR, 1);
        }
    }

    print(b"[DONE] All 14 stages complete. Motor stopped.\r\n");
}

// ── Trajectory generator ───────────────────────────────────

/// Integer square root (Newton's method).
fn isqrt(n: u64) -> u64 {
    if n == 0 { return 0; }
    let mut x = n;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}

/// Trapezoidal trajectory generator for smooth position moves.
///
/// Instead of jumping to a target position (step input), this generates
/// a smooth reference that accelerates, cruises, and decelerates.
/// The PID controller tracks this reference, keeping errors small.
///
/// This matches Pybricks' trajectory approach (lib/pbio/src/trajectory.c).
#[derive(Clone, Copy)]
pub struct Trajectory {
    start_pos: i32,
    target_pos: i32,
    direction: i32,      // +1 or -1
    accel: i32,          // deg/s² (always positive)
    cruise_speed: i32,   // deg/s (always positive, actual peak speed)
    t1: i32,             // end of accel phase (ms)
    t2: i32,             // end of cruise phase (ms)
    t3: i32,             // end of decel phase = total time (ms)
    pos1: i32,           // position offset at end of accel (degrees, positive)
    pos2: i32,           // position offset at end of cruise (degrees, positive)
}

impl Trajectory {
    /// Create a new trapezoidal trajectory from start to target position.
    ///
    /// - `accel`: acceleration in deg/s² (Pybricks default: 2000)
    /// - `max_speed`: maximum speed in deg/s (Technic M rated: 1000)
    pub fn new(start: i32, target: i32, accel: i32, max_speed: i32) -> Self {
        let distance = (target - start).abs();
        let direction = if target >= start { 1 } else { -1 };

        if distance == 0 {
            return Self {
                start_pos: start, target_pos: target, direction: 1,
                accel, cruise_speed: 0,
                t1: 0, t2: 0, t3: 0, pos1: 0, pos2: 0,
            };
        }

        // Distance to accelerate to max_speed: v²/(2a)
        let dist_full_accel = max_speed as i64 * max_speed as i64 / (2 * accel as i64);

        if dist_full_accel * 2 >= distance as i64 {
            // Triangle profile: can't reach max speed
            // peak_speed = sqrt(accel * distance)
            let peak_speed = isqrt((accel as i64 * distance as i64) as u64) as i32;
            let t_accel = (peak_speed as i64 * 1000 / accel as i64) as i32;
            let pos1 = distance / 2;

            Self {
                start_pos: start, target_pos: target, direction,
                accel, cruise_speed: peak_speed,
                t1: t_accel,
                t2: t_accel, // no cruise phase
                t3: t_accel * 2,
                pos1,
                pos2: pos1,
            }
        } else {
            // Trapezoidal profile: accel → cruise → decel
            let t_accel = (max_speed as i64 * 1000 / accel as i64) as i32;
            let pos_accel = dist_full_accel as i32;
            let dist_cruise = distance - 2 * pos_accel;
            let t_cruise = (dist_cruise as i64 * 1000 / max_speed as i64) as i32;

            Self {
                start_pos: start, target_pos: target, direction,
                accel, cruise_speed: max_speed,
                t1: t_accel,
                t2: t_accel + t_cruise,
                t3: t_accel * 2 + t_cruise,
                pos1: pos_accel,
                pos2: pos_accel + dist_cruise,
            }
        }
    }

    /// Get reference position (degrees) and speed (deg/s) at time `t_ms`
    /// milliseconds since trajectory start.
    pub fn reference(&self, t_ms: i32) -> (i32, i32) {
        if t_ms <= 0 {
            return (self.start_pos, 0);
        }
        if t_ms >= self.t3 {
            return (self.target_pos, 0);
        }

        let (pos_offset, speed) = if t_ms < self.t1 {
            // Acceleration phase: pos = a*t²/2, speed = a*t
            let speed = (self.accel as i64 * t_ms as i64 / 1000) as i32;
            let pos = (self.accel as i64 * t_ms as i64 * t_ms as i64 / 2_000_000) as i32;
            (pos, speed)
        } else if t_ms < self.t2 {
            // Cruise phase: constant speed
            let dt = t_ms - self.t1;
            let pos = self.pos1 as i64 + self.cruise_speed as i64 * dt as i64 / 1000;
            (pos as i32, self.cruise_speed)
        } else {
            // Deceleration phase
            let dt = t_ms - self.t2;
            let t_remaining = self.t3 - t_ms;
            let speed = (self.accel as i64 * t_remaining as i64 / 1000) as i32;
            let pos = self.pos2 as i64
                + self.cruise_speed as i64 * dt as i64 / 1000
                - self.accel as i64 * dt as i64 * dt as i64 / 2_000_000;
            (pos as i32, speed)
        };

        (self.start_pos + self.direction * pos_offset, self.direction * speed)
    }

    /// Total duration in milliseconds.
    pub fn duration_ms(&self) -> i32 {
        self.t3
    }
}

// ── PID controller ─────────────────────────────────────────

/// PID controller for motor position/speed control.
///
/// Runs at 20ms (50 Hz) matching the LUMP sensor update rate.
/// Pybricks runs at 5ms but has a state observer to predict between
/// measurements — we don't, so running faster than the sensor just
/// repeats stale corrections and causes overshoot.
///
/// Key features:
///   - **Trajectory tracking**: P term tracks smooth reference, not step target
///   - **Speed-error derivative**: D = Kd × (ref_speed - measured_speed)
///   - **Adaptive Kp**: halved at low speed + small error
///   - **Integral anti-windup**: rate-limited, deadzone, range-limited
///
/// Reference: pybricks lib/pbio/src/control.c, servo.c, trajectory.c
#[derive(Clone, Copy)]
pub struct Pid {
    pub kp: i32,       // proportional gain ×1024 (on position error, degrees)
    pub ki: i32,       // integral gain ×1024
    pub kd: i32,       // derivative gain ×1024 (on speed error, deg/s)
    integral: i32,
    pub prev_error: i32,
    prev_pos: i32,          // last measured position (for speed estimation)
    pub estimated_speed: i32,   // estimated motor speed in deg/s
    pub out_min: i32,       // output clamp (default -100)
    pub out_max: i32,       // output clamp (default +100)
    pub speed_threshold: i32,    // adaptive Kp: halve below this speed (deg/s)
    pub error_threshold: i32,    // adaptive Kp: halve below this error (deg)
    pub integral_deadzone: i32,  // don't accumulate Ki inside this (degrees)
    pub stiction_comp: i32,      // minimum output to overcome friction
    pub integral_change_max: i32, // max error contribution to integral per tick (deg)
    pub accel: i32,              // trajectory acceleration (deg/s²)
    pub max_speed: i32,          // trajectory max speed (deg/s)
}

impl Pid {
    /// Create a new PID for position-hold at 50 Hz (20ms loop).
    ///
    /// Tuned empirically on Technic M Angular motor (type_id 75).
    /// Strategy: trajectory-based approach at 80% omax, then hold at 40%
    /// with a settle deadband of ~5° (brake instead of hunting).
    /// No stiction compensation — it causes limit-cycle oscillation.
    ///
    /// Tunable via shell: pid kp|ki|kd|sc|omax|... <value>
    pub const fn new() -> Self {
        Self {
            kp: 800,
            ki: 16,
            kd: 400,      // strong speed damping — key to stability
            integral: 0,
            prev_error: 0,
            prev_pos: 0,
            estimated_speed: 0,
            out_min: -40,
            out_max: 40,
            speed_threshold: 150,
            error_threshold: 15,
            integral_deadzone: 3,   // prevents integral windup during transients
            stiction_comp: 35,      // min PWM to overcome motor friction
            integral_change_max: 10,
            accel: 800,
            max_speed: 400,
        }
    }

    /// Reset integrator and speed estimation state.
    pub fn reset(&mut self) {
        self.integral = 0;
        self.prev_error = 0;
        self.estimated_speed = 0;
    }

    /// Seed the previous position for speed estimation.
    /// Call once before the first update with the initial position.
    pub fn seed_position(&mut self, pos: i32) {
        self.prev_pos = pos;
    }

    /// Create a trajectory from the given start to target position,
    /// using this PID's configured accel and max_speed.
    pub fn make_trajectory(&self, start: i32, target: i32) -> Trajectory {
        Trajectory::new(start, target, self.accel, self.max_speed)
    }

    /// Compute PID output tracking a trajectory reference.
    ///
    /// Called every 20ms (matching ~50 Hz LUMP sensor rate).
    /// Each call gets fresh sensor data — no stale-correction problem.
    ///
    /// - `ref_pos`: trajectory reference position (degrees)
    /// - `ref_speed`: trajectory reference speed (deg/s)
    /// - `measured_pos`: latest sensor position (degrees)
    ///
    /// Returns clamped motor duty (-100..+100).
    pub fn update_trajectory(&mut self, ref_pos: i32, ref_speed: i32, measured_pos: i32) -> i32 {
        // ── Speed estimation ──
        // 20ms loop → multiply position delta by 50 to get deg/s
        self.estimated_speed = (measured_pos - self.prev_pos) * 50;
        self.prev_pos = measured_pos;

        let position_error = ref_pos - measured_pos;
        let speed_error = ref_speed - self.estimated_speed;

        // ── Proportional ──
        // With high Kp + output cap, P is saturated for large errors
        // and proportional for small errors near target. No adaptive Kp needed.
        let p = ((self.kp as i64 * position_error as i64) >> 10) as i32;

        // ── Integral ──
        // Pybricks-style: only accumulate when near target (within 2× saturation
        // range of P) and outside deadzone. Rate-limited by integral_change_max.
        let p_saturation = if self.kp > 0 { (self.out_max << 10) / self.kp * 2 } else { 100 };
        let in_integral_range = position_error.abs() >= self.integral_deadzone
                             && position_error.abs() <= p_saturation;
        let would_decrease = (self.integral + position_error).abs() < self.integral.abs();
        if in_integral_range || would_decrease {
            let bounded = if would_decrease {
                position_error
            } else {
                position_error.clamp(-self.integral_change_max, self.integral_change_max)
            };
            self.integral += bounded;
            let i_max = if self.ki > 0 { (self.out_max << 10) / self.ki } else { 100_000 };
            self.integral = self.integral.clamp(-i_max, i_max);
        }
        let i = ((self.ki as i64 * self.integral as i64) >> 10) as i32;

        // ── Derivative on speed error ──
        // Pybricks-style: D = Kd × (ref_speed - measured_speed).
        // This damps oscillation without fighting the approach (unlike
        // derivative-on-error which produces negative D during approach).
        let d = ((self.kd as i64 * speed_error as i64) >> 10) as i32;

        self.prev_error = position_error;

        // ── Sum and clamp ──
        let mut out = (p + i + d).clamp(self.out_min, self.out_max);

        // ── Stiction compensation ──
        if self.stiction_comp > 0
            && position_error.abs() > 1
            && self.estimated_speed.abs() < 20
            && out != 0
            && out.abs() < self.stiction_comp
        {
            out = if position_error > 0 { self.stiction_comp } else { -self.stiction_comp };
        }

        out
    }

    /// Legacy update with position (no trajectory, derivative-on-error).
    /// Kept for backward compatibility.
    pub fn update_with_pos(&mut self, error: i32, pos: i32) -> i32 {
        let speed = (pos - self.prev_pos) * 50; // 20ms assumed
        self.prev_pos = pos;

        let effective_kp = if speed.abs() < self.speed_threshold
                           && error.abs() <= self.error_threshold {
            self.kp / 2
        } else {
            self.kp
        };

        let p = ((effective_kp as i64 * error as i64) >> 10) as i32;

        if error.abs() > self.integral_deadzone {
            self.integral += error;
            let i_max = (self.out_max << 10) / self.ki.max(1);
            self.integral = self.integral.clamp(-i_max, i_max);
        }
        let i = ((self.ki as i64 * self.integral as i64) >> 10) as i32;

        let d = ((self.kd as i64 * (error - self.prev_error) as i64) >> 10) as i32;
        self.prev_error = error;

        (p + i + d).clamp(self.out_min, self.out_max)
    }

    /// Simple update (no adaptive gain, for backward compat).
    pub fn update(&mut self, error: i32) -> i32 {
        let p = ((self.kp as i64 * error as i64) >> 10) as i32;

        if error.abs() > self.integral_deadzone {
            self.integral += error;
            let i_max = (self.out_max << 10) / self.ki.max(1);
            self.integral = self.integral.clamp(-i_max, i_max);
        }
        let i = ((self.ki as i64 * self.integral as i64) >> 10) as i32;

        let d = ((self.kd as i64 * (error - self.prev_error) as i64) >> 10) as i32;
        self.prev_error = error;

        (p + i + d).clamp(self.out_min, self.out_max)
    }
}
