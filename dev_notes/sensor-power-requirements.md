# Sensor & Peripheral Extra Power Requirements (2026-04-03)

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---


## Finding: Some LEGO Parts Need Extra PWM-Like Power

After days of testing, we discovered that certain LEGO peripherals (sensors, actuators)  
require extra power delivery beyond what the LUMP protocol's data lines provide.

### Observed Behavior

*   Some sensor/actuator combinations draw more current than the port's default supply
*   Without the extra PWM-like power signal, the device may:
    *   Fail to respond to handshake
    *   Drop out mid-operation (simulating keepalive timeout)
    *   Return corrupted or zero data intermittently
    *   Work for a few seconds, then stop

### Root Cause

LEGO SPIKE Prime ports have TWO power-related signals per port:

1.  **LUMP UART** (data lines) — 3.3V logic, low current
2.  **Motor/sensor power pins** — Higher voltage for motor H-bridge, but ALSO  
    used to supply power to sensors that need it (e.g., ultrasonic transducer,  
    color sensor LEDs at high brightness)

The power pins may need a PWM-like signal to regulate current delivery to  
the peripheral, especially for sensors with active emitters (ultrasonic  
transducer, color sensor illumination LEDs at full brightness).

### Implications for Firmware

*   `motor_set(port, 0)` = coast (pins LOW) — may cut power to sensors on that port
*   A port with a sensor may need a low-duty PWM on the power pins to keep  
    the sensor's active elements powered
*   This is separate from LUMP keepalive — keepalive keeps protocol alive,  
    power keeps the _hardware_ alive

### What Pybricks Does

Pybricks `pbdrv_motor_driver` configures port power even for sensor-only ports.  
The `DCMotor` driver's `coast` vs `brake` distinction matters: brake (both high)  
keeps the sensor powered, coast (both low) may not.

### TODO

*   Investigate which ports/devices need extra power
*   Determine if a fixed low-duty PWM (~5-10%) on motor pins is sufficient
*   Check if `motor_brake(port)` (both HIGH) keeps sensor power vs `motor_set(port, 0)` (coast)
*   Look at Pybricks `pbdrv_legodev` port power initialization
*   Consider adding `port_power(port, bool)` API if needed