# Pybricks Low-Speed Motor Driving Analysis

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---


## Summary
Pybricks handles low-speed motor control through a sophisticated **piece-wise affine (PWA) feedback controller** that reduces proportional gain (Kp) at low speeds and small position errors to prevent deadband/stiction issues. The system uses **12 kHz PWM** with **H-bridge fast decay** (conventional PWM mode).

---

## 1. PWM CONFIGURATION

### Motor PWM Frequency
**12 kHz** for motors (reduced from 1.2 kHz for noise reduction)

**File:** [lib/pbio/platform/move_hub/platform.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/platform/move_hub/platform.c#L240-L275)
```c
// Motor drivers (ports A, B, C, D)
{
    .platform_init = pwm_dev_0_platform_init,
    .TIMx = TIM1,
    .prescalar = 4,        // 48 MHz / 4 = 12 MHz clock
    .period = 1000,        // 12 MHz / 1000 = 12 kHz PWM frequency
    .id = PWM_DEV_0,
    .channels = /* all 4 motor channels */
},
```

**Comment in code:**
> "Official LEGO firmware uses 1.2 kHz PWM for motors. We have changed to 12 kHz to reduce the unpleasant noise (similar to the frequency used by the official EV3 firmware)."

### PWM Mode
**PWM Mode 1 (0x6 in STM32 CCMR register)** = Standard non-inverted PWM output

**File:** [lib/pbio/drv/pwm/pwm_stm32_tim.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/drv/pwm/pwm_stm32_tim.c#L61-L108)
```c
ccmr1 |= (0x6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
// 0x6 = PWM mode 1 (output low when counter < CCR, high when counter >= CCR)
```

### H-Bridge Control
**Type:** Standard H-bridge with PWM via one FET, logic high on opposite FET = **Fast Decay**

**File:** [lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c#L53-L68)
```c
static void pbdrv_motor_driver_run_fwd(const pbdrv_motor_driver_hbridge_pwm_platform_data_t *data, 
                                        int16_t duty_cycle) {
    pbdrv_pwm_dev_t *pwm_dev;
    if (pbdrv_pwm_get_dev(data->pin1_pwm_id, &pwm_dev) == PBIO_SUCCESS) {
        pbdrv_pwm_set_duty(pwm_dev, data->pin1_pwm_ch, duty_cycle);
    }
    pbdrv_gpio_out_high(&data->pin2_gpio);  // ← Opposite pin held HIGH
}
```

**Decay Mode Explanation:**
- When PWM pin goes LOW (off): opposite pin HIGH → motor freewheels through body diode (fast decay)
- When PWM pin goes HIGH (on): opposite pin already HIGH → quick transition
- This is **fast decay** - allows rapid deceleration without inductor energy buildup

---

## 2. LOW-SPEED ANTI-STICTION CONTROL

### Adaptive Proportional Gain (Kp) at Low Speeds

Pybricks reduces the proportional feedback gain at:
- **Low command speeds** (below threshold)
- **Small position errors** (below threshold)

**File:** [lib/pbio/src/control.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/control.c#L170-L220)
```c
static int32_t pbio_control_get_pid_kp(const pbio_control_settings_t *settings, 
                                        int32_t position_error, 
                                        int32_t target_error, 
                                        int32_t abs_command_speed) {
    
    // Use reduced kp if BOTH: command speed is low AND position error is non-zero
    if (abs_command_speed >= settings->pid_kp_low_speed_threshold || position_error == 0) {
        return settings->pid_kp;  // Use normal (high) Kp
    }
    
    // Reduce Kp for low-speed, small-error region
    const int32_t kp_low = settings->pid_kp * settings->pid_kp_low_pct / 100;
    
    // Piece-wise affine (PWA) feedback:
    // - Small errors: linear feedback with kp_low
    // - Larger errors: gradual transition to full kp
    const int32_t kp_pwa = position_error <= settings->pid_kp_low_error_threshold ?
        kp_low :
        settings->pid_kp - settings->pid_kp_low_error_threshold * (settings->pid_kp - kp_low) / position_error;
    
    // Ensure proportional feedback can always reach tolerance → apply saturation constraints
    const int32_t saturation_lower = pbio_control_settings_div_by_gain(settings->actuation_max, 
                                                                        settings->pid_kp);
    const int32_t saturation_upper = saturation_lower * 100 / settings->pid_kp_low_pct;
    
    // Gradually shift towards higher Kp as we approach final target
    int32_t kp_target;
    if (target_error < saturation_lower) {
        kp_target = settings->pid_kp;  // Full gain near target
    } else if (target_error > saturation_upper) {
        kp_target = kp_low;  // Reduced gain far from target
    } else {
        // Smooth transition in between
        kp_target = kp_low + settings->pid_kp * 
            (100 - settings->pid_kp_low_pct) * (saturation_upper - target_error) /
            (saturation_upper - saturation_lower) / 100;
    }
    
    return pbio_int_math_max(kp_pwa, kp_target);
}
```

### Control Settings Parameters

**File:** [lib/pbio/src/servo.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/servo.c#L260-L300)
```c
srv->control.settings = (pbio_control_settings_t) {
    .pid_kp_low_pct = 50,                        // Use 50% of normal Kp at low speeds
    .pid_kp_low_error_threshold = 5000,          // 5 degrees (mdeg)
    .pid_kp_low_speed_threshold = /* motor-specific */,  // See motor table below
    
    .integral_deadzone = 8000,                   // 8 degrees - zone where Ki doesn't accumulate
    .integral_change_max = 15000,                // 15 degrees/tick - max integral rate
    
    .stall_speed_limit = 20000,                  // 20 deg/s - if speed < 20 and max torque, stalled
    .stall_time = 200,                           // 200 ms - how long to be stuck before flagging stall
    
    .speed_tolerance = 50000,                    // 50 deg/s - tolerance near target
    .position_tolerance = precision_profile,    // Depends on precision (e.g., 5-20 degrees)
};
```

### Motor-Specific Low-Speed Thresholds

**File:** [lib/pbio/src/motor/servo_settings.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/motor/servo_settings.c#L214-L330)

| Motor Type | `pid_kp_low_speed_threshold` (deg/s) | Notes |
|---|---|---|
| **SPIKE M Motor** (TECHNIC_M) | 0 | Never reduces Kp for speed (always uses reduced Kp when error small) |
| **SPIKE L Motor** (TECHNIC_L) | 250 | Reduces Kp only below 250 deg/s |
| **SPIKE S Motor** | 250 | Reduces Kp only below 250 deg/s |
| **Move Hub Motor** | 250 | Same as SPIKE motors |
| **Interactive Motor** (PUP) | 0 | Never reduces Kp for speed |
| **Technic L Motor (PUP)** | 250 | Reduces Kp below 250 deg/s |
| **Technic XL Motor (PUP)** | 250 | Reduces Kp below 250 deg/s |
| **Medium Motor (EV3)** | N/A (uses override) | After base settings, Kp /= 3 |
| **Large Motor (EV3)** | 100 | Even lower threshold for aggressive EV3 |
| **NXT Motor** | 100 | Very restrictive |

Example from code:
```c
{
    .id = LEGO_DEVICE_TYPE_ID_TECHNIC_L_MOTOR,
    .rated_max_speed = 1500,
    .feedback_gain_low = 45,
    .precision_profile = 20,
    .pid_kp_low_speed_threshold = 250,  // ← Below 250 deg/s, use reduced Kp
},
```

---

## 3. NO EXPLICIT "KICK" OR STARTUP BOOST

The codebase contains **no special startup kick logic** or stiction compensation.

Instead, the system relies on:

1. **Integral feedback** (Ki) that accumulates error over time
2. **Full Kp when error is zero** (even at low speeds)
3. **Feedforward torque** from observer model prediction
4. **Gradual PWA gain reduction** that prevents oscillation

**File:** [lib/pbio/src/servo.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/servo.c#L260-L290)
```c
// Ki accumulates position error with restrictions
.pid_ki = nominal_torque / precision_profile / 2,

// Feedforward torque provided by observer (applied regardless of speed)
int32_t feedforward_torque = pbio_observer_get_feedforward_torque(
    srv->observer.model, ref.speed, ref.acceleration);

// Total torque combines both
int32_t total_torque = feedback_torque + feedforward_torque;
```

---

## 4. INTEGRAL DEADZONE & ANTI-WINDUP

### Integral Deadzone
In the **8-degree deadzone** around the target, the integrator doesn't accumulate error growth:

**File:** [lib/pbio/src/integrator.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/integrator.c#L180-L230)
```c
int32_t pbio_position_integrator_update(pbio_position_integrator_t *itg, 
                                         int32_t position_error, 
                                         int32_t target_error) {
    // ...
    
    // Specify integral activation zone
    int32_t integral_range_upper = pbio_control_settings_div_by_gain(
        itg->settings->actuation_max, itg->settings->pid_kp) * 2;
    
    // Only integrate if target_error is outside deadzone OR if integral is decreasing
    if ((pbio_int_math_abs(target_error) >= itg->settings->integral_deadzone &&
         pbio_int_math_abs(target_error) <= integral_range_upper) || decrease) {
        itg->count_err_integral += pbio_control_settings_mul_by_loop_time(error_now);
    }
    
    // Clamp to prevent saturation
    itg->count_err_integral = pbio_int_math_clamp(itg->count_err_integral,
        pbio_control_settings_div_by_gain(itg->settings->actuation_max, 
                                          itg->settings->pid_ki));
}
```

---

## 5. LOW-SPEED BEHAVIOR SUMMARY

### What Happens at Low Speeds (< threshold):

1. **Small error region (< 5 deg):**
   - Proportional gain reduced to **50% of normal**
   - Smooth feedback, less aggressive
   - Avoids oscillation/chatter

2. **Medium error region (5-50 deg at target):**
   - Gain interpolated from 50% up to full value
   - Piecewise affine transition
   - Ensures smooth approach

3. **Integral Control:**
   - Accumulates slowly (saturates in ~2 seconds if stuck at tolerance edge)
   - Doesn't accumulate in 8-degree deadzone
   - Provides ultimate overcoming of friction

4. **Feedforward:**
   - Observer model predicts required torque based on speed/acceleration
   - Applied independently of position error
   - Compensates for friction and inertia

### Result:
✅ Motor smoothly overcomes static friction at low speeds without harsh "kicks"
✅ Reduced oscillation near target
✅ Integrator provides steady approach to target even with Kp reduced
✅ No minimum duty limitation — system relies on control law structure instead

---

## 6. CONTROL LOOP TIMING

**File:** [lib/pbio/src/motor_process.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/motor_process.c)
```c
static pbio_error_t pbio_motor_process_thread(pbio_os_state_t *state, void *context) {
    static pbio_os_timer_t timer;
    
    PBIO_OS_ASYNC_BEGIN(state);
    
    timer.start = pbdrv_clock_get_ms() - PBIO_CONFIG_CONTROL_LOOP_TIME_MS;
    timer.duration = PBIO_CONFIG_CONTROL_LOOP_TIME_MS;  // Typically 10 ms
    
    for (;;) {
        pbio_drivebase_update_all();
        pbio_servo_update_all();  // ← Runs control update
        
        timer.start += PBIO_CONFIG_CONTROL_LOOP_TIME_MS;
        
        PBIO_OS_AWAIT_UNTIL(state, pbio_os_timer_is_expired(&timer));
    }
}
```

**Control loop time:** ~10 ms (100 Hz servo update frequency)

---

## KEY FILES

| File | Purpose |
|---|---|
| [lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c) | H-bridge PWM duty cycle setting |
| [lib/pbio/src/control.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/control.c) | PID controller with adaptive low-speed Kp |
| [lib/pbio/src/servo.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/servo.c#L260-L300) | Servo control settings initialization |
| [lib/pbio/src/integrator.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/integrator.c) | Anti-windup deadzone logic |
| [lib/pbio/src/motor/servo_settings.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/motor/servo_settings.c) | Motor-specific parameters table |
| [lib/pbio/drv/pwm/pwm_stm32_tim.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/drv/pwm/pwm_stm32_tim.c) | PWM timer configuration (12 kHz) |
| [lib/pbio/platform/move_hub/platform.c](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/platform/move_hub/platform.c) | Platform-specific PWM setup |
