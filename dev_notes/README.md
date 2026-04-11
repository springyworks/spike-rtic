# dev\_notes/ — Index & Status (2026-04-03)

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md)

---

Guide to all documentation in this folder. Files categorized by role and freshness.

## Legend

*   **CORE** — Active reference, directly used during development
*   **BACKGROUND** — Pybricks/external analysis, useful context but not our implementation
*   **STALE** — Outdated or superseded; kept for history

---

## Core Reference (keep, actively maintained)

| File | Topic | Notes |
| --- | --- | --- |
| [sensor-fixes.md](sensor-fixes.md) | LUMP keepalive, chunked delay, mode-switch arch, flash, motor PWM confirmed | **Golden source** — see also [User Manual §2.5](../USER_MANUAL.md#25-sensor-lpf2-lump), [LUMP protocol](pybricks-lump-protocol-reference.md) |
| [gdb-debugging.md](gdb-debugging.md) | GDB/LLDB RSP architecture, DebugMonitor priority, trampoline, GDB\_ACTIVE guard | See also [User Manual §2.8](../USER_MANUAL.md#28-gdb-remote-debug), [Ref Manual §3.7](../REFERENCE_MANUAL.md#37-gdb-rsp-stub), [gdb-vs-codelldb](gdb-vs-codelldb.md) |
| [sensor-power-requirements.md](sensor-power-requirements.md) | Extra PWM-like power for some peripherals | **NEW** — some LEGO parts need power pin activation |
| [motor-encoder-mechanism.md](motor-encoder-mechanism.md) | Motor position = LUMP UART, not quadrature | See also [motor-position-controller](motor-position-controller.md), [User Manual §2.4](../USER_MANUAL.md#24-motor-control) |
| [motor-position-controller.md](motor-position-controller.md) | Ramp+nudge controller design, tunable parameters, test results | See also [motor-encoder-mechanism](motor-encoder-mechanism.md), [API motor_goto](../spike-hub-api/README.md#2-api-fields) |
| [pybricks-lump-protocol-reference.md](pybricks-lump-protocol-reference.md) | Complete LUMP protocol spec: headers, checksums, handshake, modes, keepalive | **Most valuable file** — see also [sensor-fixes](sensor-fixes.md), [build-hat-reference](build-hat-reference.md) |
| [rtic-async-patterns.md](rtic-async-patterns.md) | RTIC v2 async/await, spawning, channels, completion patterns | See also [Ref Manual §3.2](../REFERENCE_MANUAL.md#32-rtic-conventions) |
| [Laborne Trajectory Generator...md](Laborne%20Trajectory%20Generator%20and%20the%20Luenberger%20State%20Observer.md) | Servo code review, timing glitches, observer gain analysis | See also [motor-position-controller](motor-position-controller.md) |

## Background / External Analysis (useful context, not our implementation)

| File | Topic | Notes |
| --- | --- | --- |
| [pybricks\_lowspeed\_motor\_analysis.md](pybricks_lowspeed_motor_analysis.md) | Pybricks PID: adaptive Kp, integral deadzone, anti-windup | See also [motor-pwm-findings](motor-pwm-findings.md), [User Manual §7.1](../USER_MANUAL.md#71-motor-control) |
| [motor-pwm-findings.md](motor-pwm-findings.md) | Pybricks PWM: 12kHz, duty range, slow-decay | See also [pybricks analysis](pybricks_lowspeed_motor_analysis.md) |
| [self-hosted-DWT-use-address-triggering.md](self-hosted-DWT-use-address-triggering.md) | DWT self-hosted watchpoint concept, DHCSR/FPB/DebugMonitor overview | See also [Ref Manual §3.6](../REFERENCE_MANUAL.md#36-dwt-self-hosted-watchpoints), [User Manual §2.7](../USER_MANUAL.md#27-dwt-watchpoints) |
| [GDB remote seriela protocol.md](GDB%20remote%20seriela%20protocol.md) | Link to GDB RSP docs | See also [gdb-debugging](gdb-debugging.md), [Ref Manual §3.7](../REFERENCE_MANUAL.md#37-gdb-rsp-stub) |
| [stack-corruption-and-guard-bands.md](stack-corruption-and-guard-bands.md) | MPU guard bands, stack painting, fault marker | See also [Ref Manual §5](../REFERENCE_MANUAL.md#5-safety--sandbox), [Ref Manual §2.2](../REFERENCE_MANUAL.md#22-sram-320-kb) |
| [AI-talks to hub-example-chat.md](AI-talks%20to%20hub-example-chat.md) | Sensor probing transcripts, color sensor type 61, mode switching tests | Historical transcript — see [sensor-fixes](sensor-fixes.md) for validated findings |

---

## Merge Opportunities

1.  **Motor docs** — `motor-pwm-findings.md` + `pybricks_lowspeed_motor_analysis.md` could merge into one background file (`pybricks-motor-analysis.md`) since both are "how Pybricks does it"
2.  **Our motor docs** — `motor-position-controller.md` + `motor-encoder-mechanism.md` could merge into `motor-control.md` since both describe our implementation
3.  **Rules** — Delete `copilot-self-rules.md`, keep `.github/copilot-instructions.md` as canonical

_Note: Given the complexity of this project, prefer keeping files separate and using this index. Wrong merges are worse than extra files._