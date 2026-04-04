# dev_notes/ — Index & Status (2026-04-03)

Guide to all documentation in this folder. Files categorized by role and freshness.

## Legend

- **CORE** — Active reference, directly used during development
- **BACKGROUND** — Pybricks/external analysis, useful context but not our implementation
- **STALE** — Outdated or superseded; kept for history

---

## Core Reference (keep, actively maintained)

| File | Topic | Notes |
|------|-------|-------|
| [sensor-fixes.md](sensor-fixes.md) | LUMP keepalive, chunked delay, mode-switch arch, flash, motor PWM confirmed | **Golden source** — validated findings and working fixes |
| [sensor-power-requirements.md](sensor-power-requirements.md) | Extra PWM-like power for some peripherals | **NEW** — some LEGO parts need power pin activation |
| [motor-encoder-mechanism.md](motor-encoder-mechanism.md) | Motor position = LUMP UART, not quadrature | Short, factual, important for anyone asking "where does encoder data come from?" |
| [motor-position-controller.md](motor-position-controller.md) | Ramp+nudge controller design, tunable parameters, test results | Documents our implemented `motor_goto` controller |
| [pybricks-lump-protocol-reference.md](pybricks-lump-protocol-reference.md) | Complete LUMP protocol spec: headers, checksums, handshake, modes, keepalive | **Most valuable file** — essential for firmware maintenance |
| [rtic-async-patterns.md](rtic-async-patterns.md) | RTIC v2 async/await, spawning, channels, completion patterns | How to do concurrent async in firmware and user apps |
| [Laborne Trajectory Generator...md](Laborne%20Trajectory%20Generator%20and%20the%20Luenberger%20State%20Observer.md) | Servo code review, timing glitches, observer gain analysis | Technical critique of servo architecture |

## Background / External Analysis (useful context, not our implementation)

| File | Topic | Notes |
|------|-------|-------|
| [pybricks_lowspeed_motor_analysis.md](pybricks_lowspeed_motor_analysis.md) | Pybricks PID: adaptive Kp, integral deadzone, anti-windup | How Pybricks does closed-loop. We do open-loop ramp+nudge. Not directly applicable but explains the gap. |
| [motor-pwm-findings.md](motor-pwm-findings.md) | Pybricks PWM: 12kHz, duty range, slow-decay | Background from Pybricks source. Our motor.rs is the implementation. |
| [build-hat-reference.md](build-hat-reference.md) | Raspberry Pi Build HAT protocol, device IDs, PID params | External reference (RP2040 Build HAT), not SPIKE RTIC hardware. Useful for stiction compensation ideas. |
| [AI-talks to hub-example-chat.md](AI-talks%20to%20hub-example-chat.md) | Sensor probing transcripts, color sensor type 61, mode switching tests | Development log / working transcript. Historical value. |

## Stale / Superseded

| File | Topic | Notes |
|------|-------|-------|
| [uart-protocol.md](uart-protocol.md) | Single URL to external GitHub repo | **Empty** — content superseded by `pybricks-lump-protocol-reference.md`. Can be deleted. |
| [copilot-self-rules.md](copilot-self-rules.md) | AI agent rules (10 rules) | **Duplicate** of `.github/copilot-instructions.md`. Canonical source is that file + `AIcoder-thinkings/spike-rules.md`. Can be deleted. |

---

## Merge Opportunities

1. **Motor docs** — `motor-pwm-findings.md` + `pybricks_lowspeed_motor_analysis.md` could merge into one background file (`pybricks-motor-analysis.md`) since both are "how Pybricks does it"
2. **Our motor docs** — `motor-position-controller.md` + `motor-encoder-mechanism.md` could merge into `motor-control.md` since both describe our implementation
3. **Rules** — Delete `copilot-self-rules.md`, keep `.github/copilot-instructions.md` as canonical

*Note: Given the complexity of this project, prefer keeping files separate and using this index. Wrong merges are worse than extra files.*
