# SPIKE RTIC — 10 Rules (never violate these)

1. **Read the docs first.** README's  and copilot-instructions.md are the authority. Do not invent solutions that contradict them. When unsure, read Pybricks source.

2. **No busy-waits in firmware.** Every wait is `Mono::delay().await`. The only exception is user apps (SRAM2 demos) which call `delay_ms` through SVC — and even that does chunked 20ms waits with sensor keepalive internally.

3. **Firmware is infrastructure, not application.** User apps (color_seeker etc.) are hub-ram-demos built separately, uploaded via COBS, run with `go`. Never add application logic to firmware tasks. Firmware provides APIs (MonitorApi) — user apps consume them.

4. **Don't break what works.** Before changing any file, understand what it currently does. Never remove working code as a "quick fix." If a fix for X breaks Y, revert. The user has spent weeks getting motor PWM, sensor LUMP, USB flushing, RTTY, sandbox working.

5. **Motor: open-loop PWM has physics limits.** Stiction below ~25% is real. Pybricks solves it with 100Hz PID + integral accumulation — NOT with kicks or hacks. Our firmware has no closed-loop motor control yet. Don't pretend open-loop tricks fix this.

6. **Sensor: stay in mode 5, classify in software.** Mode switches cost 500-1000ms. RGBI at 50Hz in mode 5 is the architecture. Software HSV classification. One mode switch at startup, zero during operation.

7. **LUMP keepalive < 250ms or sensor dies.** demo_maintain() handles this during SVC delay_ms. sensor_poll handles it during normal operation. Never block the keepalive path.

8. **Shell accepts CR, LF, or CRLF.** Send commands with `\r\n` for Linux standard. The shell has CRLF suppression (saw_cr flag) so all three line endings work. All output uses CRLF (`\r\n`).

9. **Flash with dfu-util only.** `dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D target/spike-rtic.bin`. The user enters DFU mode themselves. Check `lsusb | grep 0694` before flashing.

10. **Don't over-engineer.** No speculative features. No "improvements" beyond what was asked. No adding code to files that aren't directly related. Ask if the scope is unclear.

11. **SVC only works from Thread mode.** SVCall (0xF0, priority 15) is lower than ANY RTIC dispatcher. SVC from Handler mode → HardFault. Sandbox demos run from `#[idle]` (Thread mode). run_demo delegates via atomics.

12. **Firmware priority 2, user apps priority 1.** All firmware RTIC tasks (heartbeat, sensor, motor, shell) at priority 2. User-facing tasks (run_demo, test_all) at priority 1. USB ISR at 2, UART ISRs at 3. Firmware NEVER blocked by user apps.

13. **Hub status LED via `set_hub_led(r, g, b)`.** Controls STATUS_TOP RGB LED (TLC5955 channels 5,4,3) through the glass light-pipe near USB. Values 0-100. Added in API v10 (SVC #21). Use it in demos for state indication: blue=probing, green=running, red=error, color cycling for sensor events, binary-delta inversion for blink.

14. **Sensor-reactive demos track deltas.** Keep `prev` data per-port, compute per-cycle deltas. Accumulate into a decaying excitement counter. Big delta → motor kick pulse (100% for 150ms), LED color flash, sound pitch shift. This is the pattern for sensor-driven behavior in exercise_all and similar demos.

---

## Project Layout (canonical paths)

- **Firmware:** `src/`
- **Shared API:** `spike-hub-api/`
- **User Apps:** `examples/hub-ram-demos/`
- **Pre-built bins:** `examples/hub-ram-demos/target/spike-usr_bins/` — objcopy'd .bin files for upload
- **Helper Tools:** `helper-tools/` — ALL Python (.py) and shell (.sh) scripts
- **AI Memory:** `AIcoder-thinkings/` — AI agent memory stores and reasoning logs
- **Dev Notes:** `dev_notes/`
