# Motor Position Controller — Two-Phase Ramp+Nudge

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---


## Overview

Open-loop position controller for LEGO Technic motors (type 75 = Technic M Angular).  
No PID — uses deceleration ramp for approach, then stiction-comp nudge pulses for  
fine convergence. Works within the constraints of 50 Hz LUMP sensor data.

## Architecture

```
Phase 1: APPROACH (first time reaching target)
┌─────────────────────────────────────────────┐
│  far zone (|err| >= decel_zone)             │
│    → drive at omax%                         │
├─────────────────────────────────────────────┤
│  decel zone (brake_zone < |err| < decel)    │
│    → linear ramp: omax → sc over distance   │
├─────────────────────────────────────────────┤
│  brake zone (|err| <= brake_zone)           │
│    → electromagnetic brake, start settle    │
└─────────────────────────────────────────────┘

Phase 2: RECOVERY (after first brake, if overshot)
┌─────────────────────────────────────────────┐
│  Stalled for 200ms outside brake zone?      │
│    → nudge: drive at sc% for 100ms          │
│    → brake, wait 200ms, check again         │
│  Each nudge moves shaft ~2-5°               │
│  Repeat until in brake zone                 │
├─────────────────────────────────────────────┤
│  In brake zone for 2s → done                │
└─────────────────────────────────────────────┘
```

## Tunable Parameters (via shell: `pid <param> <val>`)

| Param | Shell | Default | Purpose |
| --- | --- | --- | --- |
| omax | `pid omax 40` | 40 | Max PWM during approach (lower = less overshoot, slower) |
| sc | `pid sc 35` | 35 | Stiction comp / nudge power (must be > motor stiction ~25%) |
| et | `pid et 15` | 15 | Brake zone radius in degrees (wider = easier settle) |
| decel\_zone | (= et × 5) | 75 | Ramp-down distance, derived from et |

## Test Results (Technic M Angular on Port A, 2026-03-30)

| Move | Final pos | Error | Overshoot | Recovery |
| --- | --- | --- | --- | --- |
| 0→90° | 92° | \-2° | 2° | none needed |
| 92→0° | \-3° | +3° | 41° | 2 nudges |
| 0→90° (repeat) | 92° | \-2° | 0° | none |
| 2→360° | 356° | +4° | 33° | 2 nudges |

Typical accuracy: **±2–5°** with no oscillation.

## Key Findings

**Motor stiction is ~25% PWM** — any output below this won't move the shaft.  
The sc (stiction comp) must be above this threshold.

**LUMP sensor updates at ~50ms (50 Hz)** — position glitches happen  
(100663295, 51922 etc). The controller naturally ignores them because  
one bad reading only affects one 20ms iteration.

**Speed estimation is too noisy for D-term** — sampling 50ms data at 20ms  
creates wild spikes (0 to ±800 deg/s). Classical PID with D-term oscillates  
badly. P-only or ramp-based control is more robust.

**Nudge pulses prevent limit-cycle oscillation** — constant drive at sc%  
overshoots both ways creating bang-bang oscillation. Short 100ms pulses  
with 200ms brake between them converge without building momentum.

**omax controls overshoot vs speed tradeoff**:

*   omax=80: fast approach, 60-80° overshoot → many nudges
*   omax=40: moderate speed, 20-40° overshoot → 1-2 nudges
*   omax=25: slow but minimal overshoot

## What Didn't Work

*   **Full PID (kp+ki+kd)**: D-term destabilized due to noisy speed estimation
*   **P-only with stiction floor**: constant sc output when stuck → bang-bang oscillation
*   **Ramp to 0% at brake edge**: motor stalls outside brake zone, sc nudge at 20ms too short
*   **kp=400 with >>10 scaling**: at err=14°, output = 5% — far below stiction

## Code Location

*   Controller: `src/main.rs` → `pid_run` task
*   Motor PWM: `src/motor.rs` → `set()`, `brake()`
*   Sensor feedback: `src/sensor.rs` → `SensorState::motor_pos_degrees()`
*   Shell tuning: `src/shell.rs` → `pid` command block
*   Defaults: `src/motor.rs` → `Pid::default()`

## Future Improvements

*   Filter sensor glitches (reject jumps > 500°/iter)
*   Gate PID on data\_seq changes (only act on fresh sensor data)
*   Expose motor\_goto to MonitorApi for user apps (currently shell-only)
*   Adaptive omax: start high, reduce on overshoot
*   Integral term with anti-windup for reducing steady-state error