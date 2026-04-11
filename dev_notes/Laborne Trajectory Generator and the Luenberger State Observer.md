[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---

This is a great exercise. Looking closely at your `servo.rs` implementation specifically through the lens of a Rust/RTIC environment, there are a few architectural nuances, potential glitches, and areas for improvement that go beyond the high-level theory.

Here is a re-evaluation of how your specific codebase implements the Laborne + Luenberger combo, highlighting a critical timing glitch and some RTIC-specific improvements.

### 1. ⚠️ The Critical Glitch: The 100Hz vs 200Hz Math Mismatch
There is a fundamental arithmetic contradiction in your code’s comments and constants that will severely impact your PID math and Luenberger predictions if your RTIC timers are configured incorrectly.

In `servo.rs`, you have:
```rust
/// Control loop period in ms. Pybricks uses 5ms = 100 Hz.
pub const LOOP_TIME_MS: i32 = 5;
```
**The Glitch:** 5ms is **200 Hz**, not 100 Hz. (1000ms / 5ms = 200). 
* **Why this matters for RTIC:** The Luenberger observer's state matrix constants (`PRESCALE_SPEED`, `d_angle_d_speed`, etc.) are pre-calculated for a *very specific* discrete-time step (`dt`). If your RTIC timer task (`#[task(binds = TIMx_UP...)]` or software `spawn_after`) is firing at 100 Hz (10ms), but `LOOP_TIME_MS` is hardcoded to 5ms, your observer will predict only half the movement it should per tick. 
* **The Fix:** Verify your RTIC timer interval. If it is firing every 10ms, change `LOOP_TIME_MS` to `10`. If it's firing every 5ms, keep it `5` but update the documentation so you don't confuse yourself later!

### 2. The `obs_gain == -1` Override (The "Escape Hatch")
In my previous explanation, I stated that the PID controller always compares the Laborne reference to the Luenberger estimate during a move. However, your code includes a specific escape hatch:
```rust
let speed_for_error = if self.settings.obs_gain == -1 {
    self.observer.speed_numeric
} else {
    est_speed
};
```
**The Nuance:** If you set the observer gain to `-1`, you completely bypass the Luenberger speed estimate and fall back to raw numeric differentiation (`speed_numeric`). 
* **When to use this:** This is highly useful for debugging. If a motor model is drastically wrong (e.g., you swapped a Technic M motor for an L motor but didn't change the `ObserverModel`), the Luenberger estimate will diverge wildly from reality. Setting `obs_gain = -1` allows you to test the PID and Laborne trajectory independently of the observer's physics model.

### 3. Friction Zero-Crossing: The Hidden Magic
A major issue with combining mathematical trajectories (Laborne) with state estimators (Luenberger) is that at the very end of a move, the predicted speed approaches zero. Friction models get very unstable around zero (chattering back and forth between + and - speed).

Your code handles this beautifully in `Observer::update`:
```rust
let speed_without_friction = (PRESCALE_SPEED as i64 * self.speed as i64 / m.d_speed_d_speed as i64 + ...) as i32;
let speed_next = if (speed_next < 0) != (speed_without_friction < 0) {
    0
} else {
    speed_next
};
```
**The Improvement:** This prevents the Luenberger observer from endlessly vibrating at standstill. It states that if the modeled friction torque was strong enough to completely flip the direction of the motor's speed, the motor actually just **stopped**. This is what allows your `check_completion` function to accurately report `on_target = true` without the PID constantly fighting phantom observer velocities.

### 4. RTIC-Specific Improvement: Task Jitter & `actual_loop_ms`
Your code attempts to handle timing variations:
```rust
// In Observer::update
self.speed_numeric = delta * (1000 / self.actual_loop_ms);
```
Right now, `actual_loop_ms` defaults to `LOOP_TIME_MS`. In RTIC, hardware timer tasks (e.g., bound to a TIM interrupt) have near-zero jitter. However, if your `update()` function is called from a *software* task scheduled via RTIC's `Monotonic` timer, it might experience jitter depending on higher-priority tasks.

**Recommended RTIC Architecture:**
1.  **Strict Hardware Timing:** Run the `Servo::update` call inside a high-priority hardware interrupt task (e.g., `TIM2_IRQHandler`) to guarantee `dt` is perfectly consistent.
2.  **Pass Monotonic Time:** Instead of relying on `LOOP_TIME_MS`, calculate the exact delta between RTIC `Monotonic::now()` calls and pass that exact `dt` into the update functions. This makes the numeric differentiation (`speed_numeric`) flawlessly accurate even if the task is delayed by a few microseconds.

Would you like to look at how to hook this `Servo` struct up to an RTIC `Monotonic` timer to ensure the observer model gets perfectly spaced execution ticks?12:08:27.001  Mon 30Mar2026kosmod