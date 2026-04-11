# SPIKE RTIC Sensor Fixes (2025-03-26)

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---


## Root Cause: SVCall blocks sensor_poll
- SVCall priority 0xF0 = NVIC 15 = same as RTIC priority 1 tasks
- During SVC #1 (delay_ms), sensor_poll cannot preempt → no keepalives → LUMP 250ms timeout → sensor disconnect

## Fixes Applied
1. **Chunked delay_ms**: SVC #1 splits delay into 20ms chunks, calls `sensor::demo_maintain()` between (drains ring buf + keepalive)
2. **Keepalive frequency**: `demo_sensor_read` keepalive every 3rd call (was 10th)
3. **Keepalive suppression**: After `sensor_mode(N)` SELECT, suppress keepalives for 5 × 20ms ≈ 100ms to let sensor process mode change
4. **State clearing**: `demo_sensor_mode` clears data_len=0, data_received=false on mode switch
5. **RX ring buffer**: Enlarged 256 → 1024 bytes
6. **Adaptive mode-switch wait**: color_seek polls until 8-byte RGBI data arrives (up to 1500ms), exits early on success
7. **Mode-0 restore**: Waits until 1-byte mode-0 data arrives instead of fixed delay

## LUMP Key Facts
- Keepalive NACK must arrive every <250ms or sensor disconnects
- Mode switch: sensor needs ~100ms silence after SELECT (no NACK during this window)
- Color Sensor (type 61) mode 0 = 1-byte color ID, mode 5 = 8-byte RGBI (4×u16)
- Mode switch from 0→5 can take 200-1000ms before valid data flows

## Performance: Single-Mode Architecture
- Color Sensor streams RGBI at ~50 Hz in mode 5 — plenty for real-time control
- Mode switches cost 500-1000ms EACH → the killer for real-time apps
- Solution: stay in mode 5, classify RGB→HSV→color in software (same as Pybricks)
- Pybricks NEVER uses mode 0 for color detection; always mode 5 + software classify
- color_seek v3: 25s total runtime vs ~120s with mode switching (5× faster)
- One mode switch at startup is fine; zero during operation is the goal

## trace.rs
- 256-entry ring buffer (2KB), 8-byte entries (tag, val, arg, tick)
- Shell: `trace on/off/clear/legend/dump`
- The 3000ms holds flood the 256-entry buffer with DLY_KA entries

## W25Q256JV External SPI Flash (2025-03-28)
- CRITICAL: write_enable() needs 6 NOPs (~62ns) after cs_high() before next cs_low()
- W25Q256JV tSHSL spec = 50ns minimum CS# high time
- Without this delay, page_program() and sector_erase() silently fail
- LEGO factory firmware leaves BP0-BP3 set in SR1 → must call unlock() during init
- SPI2 pins: PB12(CS), PB13(SCK), PC2(MISO), PC3(MOSI), AF5
- APB1=48MHz, BR=/4 → 12 MHz SPI clock (conservative, chip supports 133 MHz)
- 4-byte addressing mode required (CMD 0xB7) for full 32 MB access
- flash_store_run.py: host script for upload→store→run flow

## Motor PWM — CONFIRMED WORKING (2026-03-28)
- **Slow-decay (Pybricks-style) is the correct approach** — fast-decay did NOT work
- Inverted PWM polarity (CCxP=1) + other pin GPIO HIGH
- Forward: pin1=PWM(inverted), pin2=GPIO HIGH → slow-decay braking on off-phase
- Reverse: pin2=PWM(inverted), pin1=GPIO HIGH
- Coast: both pins GPIO LOW
- Brake: both pins GPIO HIGH
- TIM1: PSC=7, ARR=999 → 12 kHz PWM (APB2=96MHz, timer=96MHz, 96M/8/1000=12kHz)
- Port A: PE9 (TIM1_CH1, pin1), PE11 (TIM1_CH2, pin2)
- Port B: PE13 (TIM1_CH3, pin1), PE14 (TIM1_CH4, pin2)
- `motor a <duty>` shell command: duty -100..+100, maps to CCR 0..999
- `motor a ramp` test: steps 5→10→20→40→60→80→100→brake→coast, 400ms each
- User confirmed 8+ distinct motor movements heard/seen on real hub+motor
- Motor encoder position: comes via LUMP UART (separate pins), NOT hardware quadrature
  - Port A motor UART: PE8(TX)/PE7(RX) on UART7 — independent from PWM pins

## Color Seeker Demo — CONFIRMED WORKING (2026-03-28)
- Motor A scans, Color Sensor F detects RGBI at 50Hz
- Ctrl-C (0x03) abort: shell.feed() intercepts before demo-running check
- demo_delay_ms() calls sensor::demo_maintain() every 20ms for keepalives
- Found 12+ colors in test run, motor dances on detection
