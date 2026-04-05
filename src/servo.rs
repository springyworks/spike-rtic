#![allow(dead_code)]
//! Pybricks-style servo control: observer + PID + trajectory.
//!
//! Port of libpybricks control.c, observer.c, integrator.c, servo_settings.c
//! to Rust for RTIC v2.
//!
//! ## Architecture
//!
//! ```text
//! ┌──────────┐    ┌───────────┐    ┌─────────┐    ┌─────────┐
//! │Trajectory│───▸│   PID     │───▸│Observer │───▸│Motor PWM│
//! │ Planner  │    │Controller │    │(predict)│    │  set()  │
//! └──────────┘    └───────────┘    └─────────┘    └─────────┘
//!       ▲               ▲               ▲
//!       │               │               │
//!   target_pos    position_error   LUMP encoder
//! ```
//!
//! Unlike our old PID (50 Hz, raw encoder diff for speed), this runs
//! at 100 Hz with a Luenberger state observer that predicts angle and
//! speed between sensor updates, plus feedforward for friction/inertia.
//!
//! ## Units (matching Pybricks internal)
//!
//! - **Position**: millidegrees (mdeg)
//! - **Speed**: mdeg/s
//! - **Torque**: micro-Newton-meters (uNm)
//! - **Time**: control ticks (100 µs each, 10 per ms)
//! - **Gains**: Kp in uNm/deg, Ki in uNm/(deg·s), Kd in uNm/(deg/s)
//!   (multiplied by value_in_mdeg and divided by 1000)
//!
//! ## Key difference from old PID
//!
//! The old PID outputs a % duty cycle (−100..+100). This module outputs
//! a **torque in uNm** which gets converted to voltage via the observer
//! model, then to a duty cycle. This decouples the control gains from
//! the battery voltage and motor characteristics.

/// Control loop period in ms. Pybricks uses 5ms = 100 Hz.
/// We use 5ms too, even though our sensor is ~50 Hz — the observer
/// predicts between measurements.
pub const LOOP_TIME_MS: i32 = 5;

/// Ticks per millisecond (Pybricks uses 10, i.e. 100 µs ticks).
pub const TICKS_PER_MS: i32 = 10;

// ── Integer math helpers ───────────────────────────────────

/// (a × b) / c with 64-bit intermediate to avoid overflow.
/// Matches pbio_int_math_mult_then_div.
fn mult_then_div(a: i32, b: i32, c: i32) -> i32 {
    if c == 0 { return 0; }
    ((a as i64 * b as i64) / c as i64) as i32
}

/// Multiply value by gain and scale: (gain × value) / 1000.
/// Matches pbio_control_settings_mul_by_gain.
fn mul_by_gain(value: i32, gain: i32) -> i32 {
    mult_then_div(gain, value, 1000)
}

/// Divide value by gain with scale: (value × 1000) / gain.
fn div_by_gain(value: i32, gain: i32) -> i32 {
    if gain < 1 { return 0; }
    mult_then_div(value, 1000, gain)
}

/// Multiply by loop time in seconds: input / (1000 / LOOP_TIME_MS).
fn mul_by_loop_time(input: i32) -> i32 {
    input / (1000 / LOOP_TIME_MS)
}

fn sign(x: i32) -> i32 {
    if x > 0 { 1 } else if x < 0 { -1 } else { 0 }
}

/// Integer square root (Newton's method).
fn isqrt(n: u64) -> u64 {
    if n == 0 { return 0; }
    let mut x = n;
    let mut y = (x + 1) / 2;
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}

// ── Trapezoidal Trajectory (millidegrees) ──────────────────
//
// Generates a smooth acceleration → cruise → deceleration profile
// so the PID only sees small tracking errors, not a 180° step.
// ref_speed and ref_accel feed forward into the torque model.

/// Trapezoidal trajectory in millidegrees.
/// Time measured in milliseconds relative to trajectory start.
#[derive(Clone, Copy)]
pub struct Trajectory {
    start_mdeg: i64,
    target_mdeg: i64,
    direction: i64,         // +1 or -1
    accel: i64,             // mdeg/s² (always positive)
    cruise_speed: i64,      // mdeg/s (peak, always positive)
    t1_ms: i32,             // end of accel phase (ms)
    t2_ms: i32,             // end of cruise phase (ms)
    t3_ms: i32,             // end of decel = total duration (ms)
    pos1_mdeg: i64,         // position offset at end of accel
    pos2_mdeg: i64,         // position offset at end of cruise
}

impl Trajectory {
    /// Plan a move from `start` to `target` (both in millidegrees).
    ///
    /// `accel_mdeg_s2`: acceleration/deceleration in mdeg/s²
    /// `max_speed_mdeg_s`: maximum cruise speed in mdeg/s
    pub fn new(start: i64, target: i64, accel_mdeg_s2: i32, max_speed_mdeg_s: i32) -> Self {
        let distance = (target - start).abs();
        let direction: i64 = if target >= start { 1 } else { -1 };
        let accel = accel_mdeg_s2 as i64;
        let max_speed = max_speed_mdeg_s as i64;

        if distance == 0 || accel == 0 {
            return Self {
                start_mdeg: start, target_mdeg: target, direction,
                accel, cruise_speed: 0,
                t1_ms: 0, t2_ms: 0, t3_ms: 0,
                pos1_mdeg: 0, pos2_mdeg: 0,
            };
        }

        // Distance to accelerate to max_speed: v² / (2a)
        let dist_full_accel = max_speed * max_speed / (2 * accel);

        if dist_full_accel * 2 >= distance {
            // Triangle profile — can't reach max speed
            // peak_speed = sqrt(accel * distance)
            let peak = isqrt((accel as u64) * (distance as u64)) as i64;
            let t_accel = if accel > 0 { (peak * 1000 / accel) as i32 } else { 0 };
            let pos1 = distance / 2;

            Self {
                start_mdeg: start, target_mdeg: target, direction,
                accel, cruise_speed: peak,
                t1_ms: t_accel, t2_ms: t_accel, t3_ms: t_accel * 2,
                pos1_mdeg: pos1, pos2_mdeg: pos1,
            }
        } else {
            // Trapezoidal profile — accel → cruise → decel
            let t_accel = (max_speed * 1000 / accel) as i32;
            let pos_accel = dist_full_accel;
            let dist_cruise = distance - 2 * pos_accel;
            let t_cruise = (dist_cruise * 1000 / max_speed) as i32;

            Self {
                start_mdeg: start, target_mdeg: target, direction,
                accel, cruise_speed: max_speed,
                t1_ms: t_accel,
                t2_ms: t_accel + t_cruise,
                t3_ms: t_accel * 2 + t_cruise,
                pos1_mdeg: pos_accel,
                pos2_mdeg: pos_accel + dist_cruise,
            }
        }
    }

    /// Create a zero-length trajectory (already at target).
    pub fn hold(pos_mdeg: i64) -> Self {
        Self {
            start_mdeg: pos_mdeg, target_mdeg: pos_mdeg, direction: 1,
            accel: 0, cruise_speed: 0,
            t1_ms: 0, t2_ms: 0, t3_ms: 0,
            pos1_mdeg: 0, pos2_mdeg: 0,
        }
    }

    /// Get reference at elapsed time `t_ms`:
    /// returns (position_mdeg, speed_mdeg_s, accel_mdeg_s2).
    pub fn reference(&self, t_ms: i32) -> (i64, i32, i32) {
        if t_ms <= 0 {
            // Before or at start: at start pos, about to accelerate
            let a_out = if self.t3_ms > 0 { self.direction * self.accel } else { 0 };
            return (self.start_mdeg, 0, a_out as i32);
        }
        if t_ms >= self.t3_ms {
            // After trajectory: at target, stopped
            return (self.target_mdeg, 0, 0);
        }

        let t = t_ms as i64;
        let a = self.accel;
        let dir = self.direction;

        if t_ms < self.t1_ms {
            // Acceleration phase
            let speed = a * t / 1000;
            let pos = a * t * t / 2_000_000;
            (self.start_mdeg + dir * pos, (dir * speed) as i32, (dir * a) as i32)
        } else if t_ms < self.t2_ms {
            // Cruise phase
            let dt = (t_ms - self.t1_ms) as i64;
            let pos = self.pos1_mdeg + self.cruise_speed * dt / 1000;
            (self.start_mdeg + dir * pos, (dir * self.cruise_speed) as i32, 0)
        } else {
            // Deceleration phase
            let dt = (t_ms - self.t2_ms) as i64;
            let pos = self.pos2_mdeg
                + self.cruise_speed * dt / 1000
                - a * dt * dt / 2_000_000;
            let t_rem = (self.t3_ms - t_ms) as i64;
            let speed = a * t_rem / 1000;
            (self.start_mdeg + dir * pos, (dir * speed) as i32, (dir * (-a)) as i32)
        }
    }

    /// Whether the trajectory is complete at elapsed time `t_ms`.
    pub fn is_done(&self, t_ms: i32) -> bool {
        t_ms >= self.t3_ms
    }

    /// Total duration in milliseconds.
    pub fn duration_ms(&self) -> i32 {
        self.t3_ms
    }
}

// ── Observer model constants ───────────────────────────────
//
// Auto-generated from system identification (Pybricks motor_data.py).
// These describe the discrete-time state-space model of each motor.

/// Prescale constants for fixed-point observer math.
/// Generated by pbio/doc/control/model.py
const PRESCALE_SPEED: i32 = 858;
const PRESCALE_ACCELERATION: i32 = 858;
const PRESCALE_CURRENT: i32 = 71582;
const PRESCALE_VOLTAGE: i32 = 178956;
const PRESCALE_TORQUE: i32 = 2147;

const MAX_NUM_SPEED: i32 = 2_500_000;
const MAX_NUM_ACCELERATION: i32 = 2_500_000;
const MAX_NUM_VOLTAGE: i32 = 12_000;
const MAX_NUM_TORQUE: i32 = 1_000_000;
const MAX_NUM_CURRENT: i32 = 30_000;

/// Motor model constants for the Luenberger observer.
/// Each motor type has a unique set of these.
#[derive(Clone, Copy)]
pub struct ObserverModel {
    pub d_angle_d_speed: i32,
    pub d_speed_d_speed: i32,
    pub d_current_d_speed: i32,
    pub d_angle_d_current: i32,
    pub d_speed_d_current: i32,
    pub d_current_d_current: i32,
    pub d_angle_d_voltage: i32,
    pub d_speed_d_voltage: i32,
    pub d_current_d_voltage: i32,
    pub d_angle_d_torque: i32,
    pub d_speed_d_torque: i32,
    pub d_current_d_torque: i32,
    pub d_voltage_d_torque: i32,
    pub d_torque_d_voltage: i32,
    pub d_torque_d_speed: i32,
    pub d_torque_d_acceleration: i32,
    pub torque_friction: i32,
    /// Observer correction gain.
    pub gain: i32,
}

/// Technic M Angular Motor (type_id 75, SPIKE_M_MOTOR).
/// This is the motor on ports A/B in our default setup.
pub const MODEL_TECHNIC_M_ANGULAR: ObserverModel = ObserverModel {
    d_angle_d_speed: 177194,
    d_speed_d_speed: 934,
    d_current_d_speed: -165023,
    d_angle_d_current: 2407354,
    d_speed_d_current: 8311,
    d_current_d_current: 1058029,
    d_angle_d_voltage: 7431528,
    d_speed_d_voltage: 14444,
    d_current_d_voltage: 225610,
    d_angle_d_torque: -919183,
    d_speed_d_torque: -2332,
    d_current_d_torque: 629020,
    d_voltage_d_torque: 47606,
    d_torque_d_voltage: 8071,
    d_torque_d_speed: 5903,
    d_torque_d_acceleration: 163151,
    torque_friction: 21413,
    gain: 2000,
};

/// Technic L Angular Motor (type_id 76, SPIKE_L_MOTOR).
pub const MODEL_TECHNIC_L_ANGULAR: ObserverModel = ObserverModel {
    d_angle_d_speed: 174943,
    d_speed_d_speed: 904,
    d_current_d_speed: -58045,
    d_angle_d_current: 8368268,
    d_speed_d_current: 26508,
    d_current_d_current: 396164,
    d_angle_d_voltage: 13442903,
    d_speed_d_voltage: 25105,
    d_current_d_voltage: 86900,
    d_angle_d_torque: -3690545,
    d_speed_d_torque: -9310,
    d_current_d_torque: 975141,
    d_voltage_d_torque: 133763,
    d_torque_d_voltage: 2872,
    d_torque_d_speed: 1919,
    d_torque_d_acceleration: 40344,
    torque_friction: 23239,
    gain: 4000,
};

/// Technic S Angular Motor (SPIKE_S_MOTOR, type_id 65).
pub const MODEL_TECHNIC_S_ANGULAR: ObserverModel = ObserverModel {
    d_angle_d_speed: 179217,
    d_speed_d_speed: 956,
    d_current_d_speed: -249247,
    d_angle_d_current: 1950303,
    d_speed_d_current: 7666,
    d_current_d_current: -9356019,
    d_angle_d_voltage: 5654927,
    d_speed_d_voltage: 11702,
    d_current_d_voltage: 349105,
    d_angle_d_torque: -425928,
    d_speed_d_torque: -1085,
    d_current_d_torque: 383927,
    d_voltage_d_torque: 22334,
    d_torque_d_voltage: 17203,
    d_torque_d_speed: 12282,
    d_torque_d_acceleration: 354592,
    torque_friction: 9182,
    gain: 500,
};

/// Interactive Motor (type_id 49).
pub const MODEL_INTERACTIVE: ObserverModel = ObserverModel {
    d_angle_d_speed: 179110,
    d_speed_d_speed: 941,
    d_current_d_speed: -316164,
    d_angle_d_current: 7311289,
    d_speed_d_current: 35750,
    d_current_d_current: -12014584,
    d_angle_d_voltage: 4603893,
    d_speed_d_voltage: 10967,
    d_current_d_voltage: 355664,
    d_angle_d_torque: -728461,
    d_speed_d_torque: -1850,
    d_current_d_torque: 668004,
    d_voltage_d_torque: 32225,
    d_torque_d_voltage: 11923,
    d_torque_d_speed: 10599,
    d_torque_d_acceleration: 207820,
    torque_friction: 11227,
    gain: 2000,
};

impl ObserverModel {
    /// Convert torque (uNm) to voltage (mV).
    pub fn torque_to_voltage(&self, torque: i32) -> i32 {
        PRESCALE_TORQUE * torque.clamp(-MAX_NUM_TORQUE, MAX_NUM_TORQUE) / self.d_voltage_d_torque
    }

    /// Convert voltage (mV) to torque (uNm).
    pub fn voltage_to_torque(&self, voltage: i32) -> i32 {
        PRESCALE_VOLTAGE * voltage.clamp(-MAX_NUM_VOLTAGE, MAX_NUM_VOLTAGE) / self.d_torque_d_voltage
    }

    /// Feedforward torque for a given reference speed and acceleration.
    /// Compensates friction, back-EMF, and inertia.
    ///
    /// When speed_ref is zero but accel_ref is non-zero (start of a move),
    /// friction compensation uses the acceleration direction.  This ensures
    /// the motor gets enough torque to break through static friction on
    /// the very first control tick instead of stalling.
    pub fn feedforward_torque(&self, speed_ref: i32, accel_ref: i32) -> i32 {
        // Use accel direction for friction when starting from rest
        let friction_dir = if speed_ref != 0 { sign(speed_ref) } else { sign(accel_ref) };
        let friction = (self.torque_friction / 2) * friction_dir;
        let back_emf = PRESCALE_SPEED
            * speed_ref.clamp(-MAX_NUM_SPEED, MAX_NUM_SPEED)
            / self.d_torque_d_speed;
        let accel = PRESCALE_ACCELERATION
            * accel_ref.clamp(-MAX_NUM_ACCELERATION, MAX_NUM_ACCELERATION)
            / self.d_torque_d_acceleration;
        (friction + back_emf + accel).clamp(-MAX_NUM_TORQUE, MAX_NUM_TORQUE)
    }

    /// Maximum voltage for this motor type.
    /// S motor is rated for 6V, others for 9V.
    pub fn max_voltage(&self) -> i32 {
        // Heuristic: S motor has small gain (500) — use 6000 mV.
        // All others: 9000 mV.
        if self.gain <= 500 { 6000 } else { 9000 }
    }
}

// ── Luenberger State Observer ──────────────────────────────
//
// 3-state model: angle (mdeg), speed (mdeg/s), current (0.1 mA).
// Predicts next state from current state + applied voltage.
// Corrected by measurement: feedback_voltage ∝ (measured − predicted).
//
// This is what lets us run PID at 100 Hz even though the encoder
// only updates at ~50 Hz — the observer interpolates between samples.

/// Observer state.
#[derive(Clone, Copy)]
pub struct Observer {
    /// Estimated angle in millidegrees (accumulated).
    pub angle_mdeg: i64,
    /// Estimated speed in mdeg/s.
    pub speed: i32,
    /// Estimated current in 0.1 mA units.
    pub current: i32,
    /// Previous measured angle for numeric differentiation.
    prev_measured_mdeg: i64,
    /// Numeric speed (backup / sanity check).
    pub speed_numeric: i32,
    /// Model parameters.
    model: &'static ObserverModel,
    /// Tunable feedback gain (overrides model.gain if set).
    pub feedback_gain: i32,
    /// Actual loop time in ms (may differ from LOOP_TIME_MS in raw mode).
    pub actual_loop_ms: i32,
}

impl Observer {
    pub fn new(model: &'static ObserverModel, initial_angle_mdeg: i64) -> Self {
        Self {
            angle_mdeg: initial_angle_mdeg,
            speed: 0,
            current: 0,
            prev_measured_mdeg: initial_angle_mdeg,
            speed_numeric: 0,
            model,
            feedback_gain: model.gain,
            actual_loop_ms: LOOP_TIME_MS,
        }
    }

    /// Reset observer to a new angle. Speed and current → 0.
    pub fn reset(&mut self, angle_mdeg: i64) {
        self.angle_mdeg = angle_mdeg;
        self.speed = 0;
        self.current = 0;
        self.prev_measured_mdeg = angle_mdeg;
        self.speed_numeric = 0;
    }

    /// Get observer feedback voltage to keep observer in sync with reality.
    /// error = measured − estimated, corrected by observer gain.
    fn feedback_voltage(&self, measured_mdeg: i64) -> i32 {
        let error = (measured_mdeg - self.angle_mdeg) as i32;
        PRESCALE_TORQUE * self.feedback_gain / self.model.d_voltage_d_torque * error / 1000
    }

    /// Predict next state and correct with measurement.
    ///
    /// Called every LOOP_TIME_MS (5ms).
    /// - `measured_mdeg`: latest encoder reading in millidegrees
    /// - `voltage`: voltage currently applied to motor (mV), or 0 if coasting
    /// - `fresh`: true if `measured_mdeg` is a NEW sample (sensor updated).
    ///   When false (stale), the observer runs prediction-only with no
    ///   measurement correction.  This is the key to upsampling 50 Hz
    ///   sensor data to 200 Hz: predict between samples, correct only when
    ///   a new measurement arrives.
    pub fn update(&mut self, measured_mdeg: i64, mut voltage: i32, fresh: bool) {
        let m = self.model;

        // Numeric speed: only meaningful when measurement is fresh
        if fresh {
            let delta = (measured_mdeg - self.prev_measured_mdeg) as i32;
            // Time since last fresh sample depends on how many stale ticks
            // elapsed.  For correct scaling we'd need to track inter-sample
            // time, but at 50 Hz the interval is ~20 ms.  We use actual_loop_ms
            // as a reasonable approximation (caller can adjust).
            self.speed_numeric = delta * (1000 / self.actual_loop_ms);
            self.prev_measured_mdeg = measured_mdeg;
        }

        // Observer error feedback as voltage correction (only on fresh data)
        let feedback_v = if fresh {
            self.feedback_voltage(measured_mdeg)
        } else {
            0
        };

        // Add feedback to applied voltage
        voltage += feedback_v;

        // Static friction torque model
        let torque = if self.speed > 0 {
            m.torque_friction / 2
        } else {
            -(m.torque_friction / 2)
        };

        // State update: x(k+1) = A·x(k) + B·[voltage, torque]
        let d_angle = PRESCALE_SPEED * self.speed / m.d_angle_d_speed
            + PRESCALE_CURRENT * self.current / m.d_angle_d_current
            + PRESCALE_VOLTAGE * voltage / m.d_angle_d_voltage
            + PRESCALE_TORQUE * torque / m.d_angle_d_torque;
        self.angle_mdeg += d_angle as i64;

        let speed_next = (PRESCALE_SPEED as i64 * self.speed as i64 / m.d_speed_d_speed as i64
            + PRESCALE_CURRENT as i64 * self.current as i64 / m.d_speed_d_current as i64
            + PRESCALE_VOLTAGE as i64 * voltage as i64 / m.d_speed_d_voltage as i64
            + PRESCALE_TORQUE as i64 * torque as i64 / m.d_speed_d_torque as i64) as i32;
        let speed_next = speed_next.clamp(-MAX_NUM_SPEED, MAX_NUM_SPEED);

        let current_next = (PRESCALE_SPEED as i64 * self.speed as i64 / m.d_current_d_speed as i64
            + PRESCALE_CURRENT as i64 * self.current as i64 / m.d_current_d_current as i64
            + PRESCALE_VOLTAGE as i64 * voltage as i64 / m.d_current_d_voltage as i64
            + PRESCALE_TORQUE as i64 * torque as i64 / m.d_current_d_torque as i64) as i32;
        let current_next = current_next.clamp(-MAX_NUM_CURRENT, MAX_NUM_CURRENT);

        // Friction zero-crossing: if torque would have flipped speed sign, clamp to 0
        let speed_without_friction = (PRESCALE_SPEED as i64 * self.speed as i64 / m.d_speed_d_speed as i64
            + PRESCALE_CURRENT as i64 * self.current as i64 / m.d_speed_d_current as i64
            + PRESCALE_VOLTAGE as i64 * voltage as i64 / m.d_speed_d_voltage as i64) as i32;
        let speed_next = if (speed_next < 0) != (speed_without_friction < 0) {
            0
        } else {
            speed_next
        };

        self.speed = speed_next;
        self.current = current_next;
    }

    /// Get estimated state for use by the PID controller.
    pub fn estimated_state(&self) -> (i64, i32) {
        (self.angle_mdeg, self.speed)
    }
}

// ── Position Integrator (anti-windup) ──────────────────────
//
// Pybricks-style: only accumulates error when near target,
// rate-limited, magnitude-clamped, pausable when stalled.

/// Integrator for position-based control.
#[derive(Clone, Copy)]
pub struct PositionIntegrator {
    /// Accumulated error (mdeg·ticks → scaled to mdeg·s internally).
    pub count_err_integral: i32,
    /// Previous position error for rate evaluation.
    count_err_prev: i32,
    /// Whether trajectory is running (not paused due to stall).
    trajectory_running: bool,
    /// Time when integration was last paused.
    time_pause_begin: u32,
    /// Total accumulated pause time.
    time_paused_total: u32,
}

impl PositionIntegrator {
    pub const fn new() -> Self {
        Self {
            count_err_integral: 0,
            count_err_prev: 0,
            trajectory_running: true,
            time_pause_begin: 0,
            time_paused_total: 0,
        }
    }

    pub fn reset(&mut self, time_now: u32) {
        self.count_err_integral = 0;
        self.count_err_prev = 0;
        self.time_paused_total = 0;
        self.time_pause_begin = time_now;
        self.trajectory_running = true;
    }

    /// Get reference time compensated for pauses (for trajectory evaluation).
    pub fn ref_time(&self, time_now: u32) -> u32 {
        let real_time = if self.trajectory_running {
            time_now
        } else {
            self.time_pause_begin
        };
        real_time.wrapping_sub(self.time_paused_total)
    }

    /// Pause integration (motor stalled / at torque limit).
    pub fn pause(&mut self, time_now: u32) {
        if !self.trajectory_running {
            return;
        }
        self.trajectory_running = false;
        self.time_pause_begin = time_now;
    }

    /// Resume integration.
    pub fn resume(&mut self, time_now: u32) {
        if self.trajectory_running {
            return;
        }
        self.trajectory_running = true;
        self.time_paused_total += time_now.wrapping_sub(self.time_pause_begin);
    }

    /// Update integrator and return current integral value.
    ///
    /// Pybricks-style anti-windup:
    /// 1. Only integrate when near target (within 2× P-saturation range)
    /// 2. Rate-limit growth by integral_change_max
    /// 3. Always allow magnitude decrease
    /// 4. Clamp to actuation_max / Ki
    pub fn update(
        &mut self,
        position_error: i32,
        position_remaining: i32,
        settings: &ControlSettings,
        loop_time_ms: i32,
    ) -> i32 {
        // Region where integral should be active: 2× the P saturation region
        let integral_range = div_by_gain(settings.actuation_max, settings.pid_kp) * 2;

        // Max integral value that would produce actuation_max torque
        let integral_max = div_by_gain(settings.actuation_max, settings.pid_ki);

        // Previous error (to be integrated)
        let mut cerr = self.count_err_prev;

        // Check if adding this error would decrease integral magnitude
        let mul_lt = |v: i32| -> i32 { v / (1000 / loop_time_ms) };
        let would_decrease = (self.count_err_integral + mul_lt(cerr)).abs()
            < self.count_err_integral.abs();

        // Integrate if trajectory is running OR if it would decrease magnitude
        if self.trajectory_running || would_decrease {
            // Rate-limit growth (not decrease)
            if !would_decrease {
                cerr = cerr.clamp(-settings.integral_change_max, settings.integral_change_max);
                // Re-check after clamping
            }

            let still_decrease = (self.count_err_integral + mul_lt(cerr)).abs()
                < self.count_err_integral.abs();

            // Only add if near target or decreasing
            if position_remaining.abs() <= integral_range || still_decrease || would_decrease {
                self.count_err_integral += mul_lt(cerr);
            }

            // Clamp to bounds
            self.count_err_integral = self.count_err_integral.clamp(-integral_max, integral_max);
        }

        // Save error for next iteration
        self.count_err_prev = position_error;

        self.count_err_integral
    }

    /// Check if motor is stalled (paused for longer than stall_time).
    pub fn stalled(&self, time_now: u32, speed: i32, speed_ref: i32, settings: &ControlSettings) -> bool {
        if self.trajectory_running {
            return false;
        }

        let (speed, speed_ref) = if speed_ref < 0 {
            (-speed, -speed_ref)
        } else {
            (speed, speed_ref)
        };

        if speed_ref != 0 && speed > settings.stall_speed_limit {
            return false;
        }

        if time_now.wrapping_sub(self.time_pause_begin) < settings.stall_time {
            return false;
        }

        true
    }
}

// ── Control Settings ───────────────────────────────────────

/// PID gains and control limits in Pybricks internal units.
#[derive(Clone, Copy)]
pub struct ControlSettings {
    /// Position error feedback (uNm/deg). Multiplied by error_mdeg / 1000.
    pub pid_kp: i32,
    /// Integral feedback (uNm/(deg·s)).
    pub pid_ki: i32,
    /// Speed error feedback (uNm/(deg/s)).
    pub pid_kd: i32,
    /// Max rate of integral accumulation (mdeg per tick).
    pub integral_change_max: i32,
    /// Maximum actuation torque (uNm).
    pub actuation_max: i32,
    /// Speed tolerance for "on target" (mdeg/s).
    pub speed_tolerance: i32,
    /// Position tolerance for "on target" (mdeg).
    pub position_tolerance: i32,
    /// Speed below which motor is considered stalled (mdeg/s).
    pub stall_speed_limit: i32,
    /// Time before stall flag asserts (ticks).
    pub stall_time: u32,
    /// Max speed for trajectory (mdeg/s).
    pub speed_max: i32,
    /// Acceleration for trajectory (mdeg/s²).
    pub acceleration: i32,
    /// Deceleration for trajectory (mdeg/s²).
    pub deceleration: i32,
    /// Observer feedback gain override (0 = use model default).
    pub obs_gain: i32,
}

impl ControlSettings {
    /// Default settings for Technic M Angular motor.
    /// Matches Pybricks servo_settings.c for type_id 75/76.
    pub fn technic_m_angular() -> Self {
        let model = &MODEL_TECHNIC_M_ANGULAR;
        let actuation_max = model.voltage_to_torque(model.max_voltage());
        let position_tolerance = 10_000; // 10 deg in mdeg
        // Ki: saturate integral in ~2s if stuck at position_tolerance
        let pid_ki = actuation_max / (position_tolerance / 1000) / 2;

        Self {
            pid_kp: 15000,
            pid_ki,
            pid_kd: 1800,
            integral_change_max: 15_000,    // 15 deg in mdeg
            actuation_max,
            speed_tolerance: 50_000,        // 50 deg/s
            position_tolerance,
            stall_speed_limit: 20_000,      // 20 deg/s
            stall_time: 200 * TICKS_PER_MS as u32,  // 200 ms
            speed_max: 1_080_000,           // 1080 deg/s
            acceleration: 2_000_000,        // 2000 deg/s²
            deceleration: 2_000_000,
            obs_gain: 0,
        }
    }

    /// Default settings for Technic L Angular motor.
    pub fn technic_l_angular() -> Self {
        let model = &MODEL_TECHNIC_L_ANGULAR;
        let actuation_max = model.voltage_to_torque(model.max_voltage());
        let position_tolerance = 10_000;
        let pid_ki = actuation_max / (position_tolerance / 1000) / 2;

        Self {
            pid_kp: 35000,
            pid_ki,
            pid_kd: 6000,
            integral_change_max: 15_000,
            actuation_max,
            speed_tolerance: 50_000,
            position_tolerance,
            stall_speed_limit: 20_000,
            stall_time: 200 * TICKS_PER_MS as u32,
            speed_max: 970_000,
            acceleration: 1_500_000,
            deceleration: 1_500_000,
            obs_gain: 0,
        }
    }

    /// Default settings for S motor.
    pub fn technic_s_angular() -> Self {
        let model = &MODEL_TECHNIC_S_ANGULAR;
        let actuation_max = model.voltage_to_torque(model.max_voltage());
        let position_tolerance = 10_000;
        let pid_ki = actuation_max / (position_tolerance / 1000) / 2;

        Self {
            pid_kp: 7500,
            pid_ki,
            pid_kd: 1000,
            integral_change_max: 15_000,
            actuation_max,
            speed_tolerance: 50_000,
            position_tolerance,
            stall_speed_limit: 20_000,
            stall_time: 200 * TICKS_PER_MS as u32,
            speed_max: 620_000,
            acceleration: 2_000_000,
            deceleration: 2_000_000,
            obs_gain: 0,
        }
    }
}

// ── Servo Controller ───────────────────────────────────────
//
// Ties together: trajectory + PID + observer + motor output.
// This is the main entry point for motor control.

/// Control mode.
#[derive(Clone, Copy, PartialEq)]
pub enum ControlType {
    /// No active control.
    None,
    /// Position control: go to a specific angle and hold.
    Position,
    /// Timed control: run at speed for a duration.
    Timed,
}

/// What to do when control command completes.
#[derive(Clone, Copy, PartialEq)]
pub enum OnCompletion {
    Coast,
    Brake,
    Hold,
}

/// Actuation command output from the controller.
#[derive(Clone, Copy, PartialEq)]
pub enum Actuation {
    /// Coast the motor (no drive).
    Coast,
    /// Passively brake (short terminals).
    Brake,
    /// Apply torque (uNm). Converted to duty by caller.
    Torque(i32),
}

/// Complete servo controller.
pub struct Servo {
    pub observer: Observer,
    pub settings: ControlSettings,
    pub integrator: PositionIntegrator,
    pub model: &'static ObserverModel,

    // Control state
    pub control_type: ControlType,
    pub on_completion: OnCompletion,
    pub stalled: bool,
    pub on_target: bool,
    /// Low-pass filtered PID output (load diagnostic).
    pub pid_average: i32,

    // Trajectory planner (trapezoidal accel→cruise→decel)
    pub trajectory: Trajectory,
    traj_target_pos_mdeg: i64,
    traj_start_ticks: u32,

    // Time tracking
    time_ticks: u32,

    // Last fresh measured position (held between sensor updates)
    last_measured_mdeg: i64,
}

impl Servo {
    /// Create a new servo controller for a specific motor type.
    pub fn new(
        model: &'static ObserverModel,
        settings: ControlSettings,
        initial_angle_mdeg: i64,
        time_ticks: u32,
    ) -> Self {
        Self {
            observer: Observer::new(model, initial_angle_mdeg),
            settings,
            integrator: PositionIntegrator::new(),
            model,
            control_type: ControlType::None,
            on_completion: OnCompletion::Hold,
            stalled: false,
            on_target: true,
            pid_average: 0,
            trajectory: Trajectory::hold(initial_angle_mdeg),
            traj_target_pos_mdeg: initial_angle_mdeg,
            traj_start_ticks: time_ticks,
            time_ticks,
            last_measured_mdeg: initial_angle_mdeg,
        }
    }

    /// Start a position hold command.
    /// Target in millidegrees.
    pub fn start_position_hold(&mut self, target_mdeg: i64, time_ticks: u32) {
        self.control_type = ControlType::Position;
        self.on_completion = OnCompletion::Hold;
        self.on_target = false;
        self.stalled = false;
        self.traj_target_pos_mdeg = target_mdeg;
        self.traj_start_ticks = time_ticks;
        self.integrator.reset(time_ticks);

        // Plan trapezoidal trajectory from current observer position to target
        let start_mdeg = self.observer.angle_mdeg;
        self.trajectory = Trajectory::new(
            start_mdeg,
            target_mdeg,
            self.settings.acceleration,
            self.settings.speed_max,
        );
    }

    /// Stop control and coast.
    pub fn stop(&mut self) {
        self.control_type = ControlType::None;
        self.on_target = true;
        self.stalled = false;
        self.pid_average = 0;
    }

    /// Core control update. Call every LOOP_TIME_MS (5ms).
    ///
    /// Returns the actuation command.
    ///
    /// - `time_ticks`: current time in 100µs ticks
    /// - `measured_mdeg`: latest encoder position in millidegrees
    /// - `fresh`: true if measured_mdeg is a new sample from the sensor
    pub fn update(&mut self, time_ticks: u32, measured_mdeg: i64, fresh: bool) -> Actuation {
        self.time_ticks = time_ticks;

        if self.control_type == ControlType::None {
            // Not controlling — still update observer for tracking
            self.observer.update(measured_mdeg, 0, fresh);
            return Actuation::Coast;
        }

        // ── Get reference from trajectory ──
        // Evaluate trajectory at pause-compensated elapsed time
        let ref_time = self.integrator.ref_time(time_ticks);
        let elapsed_ms = ref_time.wrapping_sub(self.traj_start_ticks) as i32 / TICKS_PER_MS;
        let (ref_pos_mdeg, ref_speed, ref_accel) = self.trajectory.reference(elapsed_ms);
        let traj_done = self.trajectory.is_done(elapsed_ms);

        // ── Get observer estimated state ──
        let (est_angle, est_speed) = self.observer.estimated_state();

        // ── Update last-known measured position on fresh samples ──
        if fresh {
            self.last_measured_mdeg = measured_mdeg;
        }

        // ── Compute errors ──
        // During trajectory: use observer for both position and speed.
        //   The observer interpolates smoothly between 50Hz samples, giving
        //   stable 200 Hz control. Model drift is acceptable over short moves.
        // After trajectory (holding): use measured position for P error.
        //   This eliminates observer drift at steady state. The D term still
        //   comes from the observer for smooth damping.
        let speed_for_error = if self.settings.obs_gain == -1 {
            self.observer.speed_numeric
        } else {
            est_speed
        };
        let pos_for_error = if traj_done {
            // Hold phase: use measured position to avoid observer drift
            self.last_measured_mdeg
        } else {
            // Trajectory phase: use observer for smooth tracking
            est_angle
        };
        let position_error = (ref_pos_mdeg - pos_for_error) as i32; // mdeg
        let speed_error = ref_speed - speed_for_error; // mdeg/s

        // ── Position remaining (for integral range gating) ──
        // Distance from current trajectory reference to final target.
        // Integral only kicks in when near final destination.
        let position_remaining = (self.traj_target_pos_mdeg - ref_pos_mdeg) as i32;

        // ── Integral ──
        let loop_ms = self.observer.actual_loop_ms;
        let integral_error = self.integrator.update(
            position_error,
            position_remaining,
            &self.settings,
            loop_ms,
        );

        // ── PID torque computation ──
        let torque_p = mul_by_gain(position_error, self.settings.pid_kp);
        let torque_i = mul_by_gain(integral_error, self.settings.pid_ki);
        let torque_d = mul_by_gain(speed_error, self.settings.pid_kd);

        let feedback_torque = (torque_p + torque_i + torque_d)
            .clamp(-self.settings.actuation_max, self.settings.actuation_max);

        // ── Anti-windup: pause integration if at torque limit ──
        let windup_margin =
            (est_speed.abs() / (1000 / loop_ms)) * 2;
        let max_windup_torque =
            self.settings.actuation_max + mul_by_gain(windup_margin, self.settings.pid_kp);

        let pause_integration = torque_p.abs() >= max_windup_torque
            && sign(torque_p) != -sign(speed_error)
            && sign(torque_p) != -sign(ref_accel);

        if pause_integration {
            self.integrator.pause(time_ticks);
        } else {
            self.integrator.resume(time_ticks);
        }

        // ── Feedforward torque ──
        let feedforward_torque = self.model.feedforward_torque(ref_speed, ref_accel);

        let total_torque = feedback_torque + feedforward_torque;

        // ── Stall detection ──
        self.stalled = self.integrator.stalled(
            time_ticks,
            est_speed,
            ref_speed,
            &self.settings,
        );

        // ── On-target check ──
        self.on_target = self.check_completion(measured_mdeg, traj_done);

        // ── Low-pass filtered load ──
        self.pid_average = (self.pid_average * (100 - loop_ms)
            + feedback_torque * loop_ms) / 100;

        // ── Convert torque to voltage, then to duty cycle ──
        let voltage = self.model.torque_to_voltage(total_torque);

        // Update observer with the voltage we're applying
        self.observer.update(measured_mdeg, voltage, fresh);

        // ── Decide actuation ──
        if !self.on_target {
            Actuation::Torque(total_torque)
        } else {
            match self.on_completion {
                OnCompletion::Coast => {
                    self.stop();
                    Actuation::Coast
                }
                OnCompletion::Brake => {
                    self.stop();
                    Actuation::Brake
                }
                OnCompletion::Hold => {
                    // Keep actuating to hold position
                    Actuation::Torque(total_torque)
                }
            }
        }
    }

    /// Check if position control is complete.
    fn check_completion(&self, measured_mdeg: i64, traj_done: bool) -> bool {
        if self.control_type == ControlType::None {
            return true;
        }

        // Trajectory must be finished first
        if !traj_done {
            return false;
        }

        let pos_remaining = (self.traj_target_pos_mdeg - measured_mdeg) as i32;

        // Done if standing still and close enough to target
        if self.observer.speed.abs() > self.settings.speed_tolerance {
            return false;
        }
        pos_remaining.abs() <= self.settings.position_tolerance
    }

    /// Convert torque actuation to motor duty cycle (-100..+100).
    ///
    /// Uses the observer model to convert torque → voltage → duty.
    /// Battery voltage is assumed 9000 mV (could be measured for accuracy).
    pub fn torque_to_duty(&self, torque: i32) -> i32 {
        let voltage = self.model.torque_to_voltage(torque);
        // Duty = voltage / battery_voltage × 100
        // Using 9000 mV (nominal SPIKE battery voltage)
        let duty = voltage * 100 / 9000;
        duty.clamp(-100, 100)
    }

    /// Get current PID state for diagnostics.
    pub fn diag(&self) -> ServoDiag {
        ServoDiag {
            est_angle_mdeg: self.observer.angle_mdeg,
            est_speed: self.observer.speed,
            speed_numeric: self.observer.speed_numeric,
            integral: self.integrator.count_err_integral,
            pid_average: self.pid_average,
            stalled: self.stalled,
            on_target: self.on_target,
        }
    }
}

/// Diagnostic snapshot for shell display.
pub struct ServoDiag {
    pub est_angle_mdeg: i64,
    pub est_speed: i32,
    pub speed_numeric: i32,
    pub integral: i32,
    pub pid_average: i32,
    pub stalled: bool,
    pub on_target: bool,
}
