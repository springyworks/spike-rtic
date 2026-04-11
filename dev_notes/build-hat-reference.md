# Raspberry Pi Build HAT — LPF2 Protocol Reference

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---


Research date: 2026-03-28  
Source: https://www.raspberrypi.com/documentation/accessories/build-hat.html  
Python lib: https://github.com/RaspberryPiFoundation/python-build-hat (MIT, v0.9.0)  
.NET lib: https://github.com/dotnet/iot/tree/main/src/devices/BuildHat  
Serial protocol spec: https://pip.raspberrypi.com/documents/RP-008140-DS

## Architecture

The Build HAT has an **RP2040 microcontroller** that handles ALL low-level LUMP  
protocol (sync, keepalives, mode switching, data parsing). The Raspberry Pi talks  
to the RP2040 via a **high-level text serial protocol** at 115200 baud on  
GPIO14(TX)/GPIO15(RX). The Python/C#/.NET libraries are just serial console wrappers.

The RP2040 firmware is a **proprietary binary blob** loaded from `firmware.bin`  
at initialization. It is NOT open source.

**We (spike\_rtic) handle LUMP directly on the STM32** — no intermediary MCU.  
The Build HAT approach is fundamentally different from ours.

## Serial Protocol Commands (Pi → RP2040)

```
port <n>                     # Select port 0-3 (A-D)
port <n> ; select <mode>     # Set LUMP mode for data streaming
port <n> ; selrate <ms>      # Set data update interval (ms)
port <n> ; combi <n> <m1> <d1> <m2> <d2> ...  # Combi mode
port <n> ; pwm ; set <val>   # Direct PWM (-1.0 to +1.0)
port <n> ; coast             # Coast motor (float)
port <n> ; off               # Turn off port
port <n> ; on                # Turn on port
port <n> ; port_plimit <val> # Power limit (0.0-1.0, default 0.7)
port <n> ; write1 <hex>      # Raw LUMP write

# PID position control:
pid <port> 0 1 s4 0.0027777778 0 5 0 .1 3 0.01
set ramp <pos_from> <pos_to> <duration> 0

# PID speed control:
pid <port> 0 0 s1 1 0 0.003 0.01 0 100 0.01
set pulse <speed> 0.0 <seconds> 0

# PWM params:
pwmparams <pwmthresh> <minpwm>
  # pwmthresh: 0-1, threshold below which switches from fast to slow PWM
  # minpwm: 0-1, threshold below which drive is cut off (stiction compensation)
  # Default: pwmthresh=0.65, minpwm=0.01
```

## Device Type IDs

| Type | Name | LEGO Part |
| --- | --- | --- |
| 1 | Passive Motor (system medium motor) | 45303 |
| 2 | Passive Motor (train motor) | 88011 |
| 8 | Light | 88005 |
| 34 | WeDo 2.0 Tilt Sensor | 45305 |
| 35 | WeDo 2.0 Motion Sensor | 45304 |
| 37 | Color & Distance Sensor | 88007 |
| 38 | Medium Linear Motor | 88008 |
| 46 | Large Motor | 88013 |
| 47 | XL Motor | 88014 |
| 48 | Medium Angular Motor (Cyan) | 45603 |
| 49 | Large Angular Motor (Cyan) | 45602 |
| 61 | Color Sensor | 45605 |
| 62 | Distance Sensor | 45604 |
| 63 | Force Sensor | 45606 |
| 64 | 3x3 Color Light Matrix | 45608 |
| 65 | Small Angular Motor | 45607 |
| 75 | Medium Angular Motor (Grey) | 88018 |
| 76 | Large Angular Motor (Grey) | 88017 |

## Motor Combi Modes (Active Motors)

Active motors use combi mode for simultaneous speed + position + absolute position:

```python
# Motors with absolute position (type 48, 49, 65, 75, 76):
self.mode([(1, 0), (2, 0), (3, 0)])  # speed, position, abs_position

# Motors WITHOUT absolute position (type 38 = Medium Linear):
self.mode([(1, 0), (2, 0)])  # speed, position only
```

Mode mapping for SPIKE motors:

*   Mode 0: Power (PWM duty cycle)
*   Mode 1: Speed (degrees/sec)
*   Mode 2: Position (cumulative degrees, i32)
*   Mode 3: Absolute position (-180 to 180)
*   Mode 4: CALIB (combo calibration mode)

## PID Parameters from Python Library

Speed hold PID:

```
pid {port} 0 0 s1 1 0 0.003 0.01 0 100 0.01
```

Positional ramp PID:

```
pid {port} 0 1 s4 0.0027777778 0 5 0 .1 3 0.01
set ramp {current_pos} {target_pos} {duration_seconds} 0
```

*   Position values in decimal rotations (pos/360.0), not degrees
*   Speed range collapsed: speed \* 0.05 for percentage mode, speed/60 for RPM mode
*   Power limit default 0.7

## PWM Parameters

```python
motor.pwmparams(0.65, 0.01)  # Default values
# pwmthresh=0.65: below this ratio, use slow-decay PWM (more holding torque)
# minpwm=0.01: below this ratio, cut drive entirely (dead-zone for stiction)
```

This is relevant to our stiction problem — the Build HAT firmware has built-in  
dead-zone and fast/slow PWM switching that we should implement.

## Key Insight: We Don't Need Legacy Support

User note: "Pybricks assumes immediately that there is a highspeed device  
connected to the LPF2 port, I personally do not use legacy LEGO stuff."

All modern SPIKE devices (types 48, 49, 61, 62, 63, 64, 65, 75, 76) are  
high-speed 115200 baud. The 2400 baud LUMP sync is only needed for the initial  
handshake — the sensor always starts at 2400, then negotiates up. But we could  
potentially speed up the sync by reducing timeouts and being more aggressive.