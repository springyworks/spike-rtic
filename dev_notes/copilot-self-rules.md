# Copilot Self-Rules — SPIKE RTIC Project

These rules are derived from the project's copilot-instructions.md, dev_notes/,
and hard lessons from past sessions.  Copilot must consult this file before
making any change.

## 1. Read the docs first
all the README.MD files ;; `dev_notes/` and `.github/copilot-instructions.md` are the authority.
Do not invent solutions that contradict them.  When unsure, read the
Pybricks source (`lib/pbio/`).

## 2. No busy-waits in firmware
Every wait is `Mono::delay().await`.  The only exception is user apps
(SRAM2 demos) which call `delay_ms` through SVC — and even that does
chunked 20 ms waits with sensor keepalive internally.

## 3. Firmware is infrastructure, not application
User apps (color_seeker, etc.) are hub-ram-demos built separately,
uploaded via COBS, run with `go`.  Never add application logic to
firmware tasks.  Firmware provides APIs (MonitorApi) — user apps
consume them.

## 4. Don't break what works
Before changing any file, understand what it currently does.  Never
remove working code as a "quick fix."  If a fix for X breaks Y, revert.
Motor PWM, sensor LUMP, USB flushing, RTTY, and sandbox are stable —
leave them alone unless explicitly asked.

## 5. Motor: open-loop PWM has physics limits
Stiction below ~25 % is real.  Pybricks solves it with 100 Hz PID +
integral accumulation — NOT with kicks or hacks.  Our firmware has no
closed-loop motor control yet.  Don't pretend open-loop tricks fix this.
See `dev_notes/pybricks_lowspeed_motor_analysis.md`.

## 6. Sensor: stay in mode 5, classify in software
Mode switches cost 500–1000 ms.  RGBI at 50 Hz in mode 5 is the
architecture.  Software HSV classification.  One mode switch at startup,
zero during operation.  See `dev_notes/sensor-fixes.md`.

## 7. LUMP keepalive < 250 ms or sensor dies
`demo_maintain()` handles this during SVC delay_ms.  `sensor_poll`
handles it during normal operation.  Never block the keepalive path.

## 8. Shell uses `\r` not `\r\n`
Every shell command ends with `\r`.  `\r\n` causes
"ERR: unknown command."

## 9. Flash with dfu-util only
```
dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D target/spike-rtic.bin
```
The user enters DFU mode themselves.  Check `lsusb | grep 0694` before
flashing.

## 10. Don't over-engineer
No speculative features.  No "improvements" beyond what was asked.
No adding code to files that aren't directly related.  Ask if the
scope is unclear.  Keep all documentation in `dev_notes/` so it is
version-controlled and committable.

## 11. Every demo must check API version
All RAM demo `_start` functions must check `api.version >= N` before
using any API fields added in version N.  This prevents silent crashes
when a demo is uploaded to firmware with a different API version.
The check is cheap (one compare + early return) and the error message
tells the user exactly what's wrong.

## 12. No stale .bin files — binaries live in target/ only
Never leave `.bin` files outside the proper build output directories:
- Firmware: `target/thumbv7em-none-eabihf/release/spike-rtic`
- Demo bins: `examples/hub-ram-demos/target/spike-usr_bins/`

Stale bins (e.g. `target/firmware.bin`, `exercise_all.bin` in the wrong
directory) are **dangerous** — you might flash old firmware or upload
an old demo and spend hours debugging a "sandbox fault" that's really
just a version mismatch.  After `objcopy`, place the `.bin` in
`target/spike-usr_bins/` and nowhere else.  Clean up any strays.
