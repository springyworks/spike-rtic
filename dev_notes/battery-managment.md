The **MPS MP2639A** is a "hardware-configured" chip. Unlike many modern PMICs (Power Management Integrated Circuits) that use $I^2C$ to set parameters via software, the MP2639A relies almost entirely on the **physical values of resistors and capacitors** connected to its pins to determine its behavior.

Because it lacks internal non-volatile memory or registers, its "defaults" are effectively defined by the manufacturer's recommended external component values.

---

### 1. Mode Selection (Boost vs. Buck)
The most critical default is determined by the **MODE** pin. This tells the chip whether to act as a charger (Buck) or a power bank (Boost).
* **High (Logic 1):** Discharge Mode (Boost). It takes battery voltage and boosts it to 5V on the USB/VIN pin.
* **Low (Logic 0 / Ground):** Charge Mode (Buck). It takes 5V input and charges the 2-cell battery.

### 2. Charging Parameters (Typical Defaults)
In the SPIKE Prime and most standard implementations, the following "default" behaviors are set by specific pins:

| Parameter | Controlling Pin | Typical Implementation Value |
| :--- | :--- | :--- |
| **Charge Voltage** | **VSET** | Typically connected to **GND** for a 2-cell (8.4V) configuration. |
| **Charge Current** | **ISET** | Determined by a resistor ($R_{ISET}$). Default max is usually **2.5A**. |
| **Input Current Limit** | **ILIM** | Determined by a resistor ($R_{ILIM}$). Typically set to **3.0A**. |
| **Pre-charge Current** | Internal | Typically **10%** of the programmed full-charge current. |
| **Termination Current** | Internal | Typically **10%** of the programmed full-charge current. |



### 3. Protection Thresholds
The chip has built-in safety "defaults" that are hard-coded into the silicon:
* **Battery UVLO (Under Voltage Lock Out):** The chip will stop discharging if the battery per cell drops below approximately **2.4V**.
* **Input Over-Voltage Protection:** If the input voltage on the VIN pin exceeds **approx. 6V**, the chip stops charging to protect itself.
* **Thermal Regulation:** If the internal junction temperature hits **120°C**, the chip automatically begins reducing the charge current to cool down.

### 4. Logic & Indication (LED Pins)
The chip has two pins, **STAT** and **OLP**, that provide the default status signaling:
* **Charging:** STAT pin is pulled Low (LED On).
* **Charge Full:** STAT pin is High Impedance (LED Off).
* **Fault (NTC Error/Timer out):** STAT pin blinks at **0.5Hz**.

### 5. The NTC (Temperature) Default
The **NTC pin** expects a 10kΩ Thermistor. If the voltage on this pin falls outside the range of **30% to 74% of VCC**, the chip assumes the battery is too hot or too cold and will default to **suspending all charging** until the temperature stabilizes.

> **Note for Pybricks users:** Because these defaults are set by resistors on the SPIKE Prime PCB, the Pybricks firmware cannot change the maximum charge speed or the target voltage via code. These are "baked" into the hardware design by LEGO.

---

## Firmware Charger Control (spike-rtic)

### ISET PWM — The Missing Piece

The ISET pin is **not** hardwired to a fixed resistor on the SPIKE Prime hub.
Instead, it is driven by **TIM5 Channel 1 on PA0** (AF2) with a PWM signal
that acts as a variable current-limit control.

Without firmware driving ISET, the pin floats and the MP2639A delivers
**zero charge current** — the battery only charges when the LEGO bootloader
is running (because the bootloader configures TIM5).

**Pybricks TIM5 config** (from `platform.c` and `pwm_stm32_tim.c`):

| Register | Value | Effect |
|:---------|:------|:-------|
| PSC | 9 | Divide 96 MHz timer clock by 10 → 9.6 MHz tick |
| ARR | 100 | Period → 96 kHz PWM |
| CCR1 | duty | 0–100 maps directly to 0–100% duty cycle |

Note: Pybricks stores `prescalar = 10` in platform data and writes
`PSC = prescalar - 1 = 9` in the timer init code.

**Duty cycle values** (from `charger_mp2639a.c`):

| Duty | Current limit | Use case |
|:-----|:-------------|:---------|
| 100 | Maximum (hardware-set by $R_{ISET}$) | DCP / charging dock |
| 15 | ~500 mA | USB Standard Downstream Port |
| 2 | ~100 mA | USB suspend / minimal |
| 0 | Disabled | Charger off |

Our firmware currently uses duty=100 (max) whenever charging is enabled.
A future improvement could use USB Battery Charging Detection (BCD) to
select the appropriate limit.

### MODE Pin Control

MODE is routed through **TLC5955 channel 14** (not a GPIO).

- ch14 = 0x0000 → TLC output off → external pull-down pulls MODE **LOW** → charge (buck) mode
- ch14 = 0xFFFF → TLC output sinks through pull-up → MODE **HIGH** → discharge (boost) mode

The MODE state latches on the next `led_matrix::update()` call (SPI1 shift-out).

### /CHG (CHGOK/STAT) Detection

The /CHG signal shares a resistor ladder with the center button on **ADC ch14 (PC4)**.
Three inputs create 8 voltage levels decoded as a 3-bit flags word:

- bit 0 (CH_0) = center button pressed
- bit 1 (CH_1) = unused
- bit 2 (CH_2) = /CHG active (LOW = charging)

Decode thresholds from Pybricks `RESISTOR_LADDER_DEV_0`:
```
[3642, 3142, 2879, 2634, 2449, 2209, 2072, 1800]
```

**Known issue**: Our hub's ADC reads ~3650 for "no flags" baseline, only 8 counts
above the 3642 threshold. This means /CHG detection is marginal — the charger may
be actively charging but the ladder never drops below 3642 to set the CH_2 flag.
Charging can be verified independently by monitoring battery voltage trend and
USB current draw (ADC ch3).

### Charger State Machine

The firmware runs a state machine in `power::ChargerState::tick()` called every
500 ms from the heartbeat task:

1. **USB present** → enable charger (MODE LOW + ISET 100%)
2. **Sample /CHG** via resistor ladder, maintain 7-sample circular buffer
3. **Detect fault** — >2 transitions in the window = 0.5 Hz blink = fault
4. **Declare complete** — only when battery ≥ 8300 mV and /CHG not active
5. **Default to charging** — USB present + ISET active → assume charging
   (empirically confirmed by voltage-rise data; /CHG ladder is marginal)
6. **Charge timeout** — after 60 min continuous charging, pause 30 s then restart
   (prevents MP2639A safety-timer expiry from causing a permanent fault state)
7. **USB removed** → disable charger, reset state

### Battery LED — Morse Status Indicator

The battery LED blinks a repeating two-letter Morse code to show charger state.
Timing uses Farnsworth spacing: 125 ms per element (dit/dah), 875 ms letter gap,
3 s repeat pause. This gives clear, readable flashes even at a glance.

| Morse | Letters | Meaning | LED Colour |
|:------|:--------|:--------|:-----------|
| `-.-. ....` | **CH** | Charging (USB present, ISET active) | Green (bright flash / dim glow) |
| `.... ..` | **HI** | Complete (battery ≥ 8300 mV) | Green (bright flash / dim glow) |
| `-. --.` | **NG** | Fault (NTC error or safety timer) | Amber (bright flash / dim glow) |
| `-... -` | **BT** | Battery (no USB, discharging) | Off (no LED) |

Additionally, the **STATUS_TOP LED** (near USB port) shows a brief dim cyan pulse
every 5 seconds as a "system alive" heartbeat overlay, independent of the Morse
pattern. During normal operation (not charging idle), the blue data-activity pulses
you see on STATUS_TOP indicate USB serial traffic.

### Boot Sequence

`init_charger_iset()` runs during RTIC init (after LED matrix and ADC):

1. Enable TIM5 clock (RCC APB1ENR bit 3)
2. Configure PA0 as AF2 (TIM5_CH1)
3. Set PSC=9, ARR=100, CCR1=100 (96 kHz, 100% duty)
4. PWM mode 1, output enable, counter enable
5. **MODE kick**: toggle MODE HIGH for 500 ms → back to LOW
   - Forces the MP2639A to restart its charge cycle
   - Needed because the bootloader may have left the charger in sleep/complete state

---

## Voltage Measurement

**Voltage divider**: 3:1 (200kΩ / 100kΩ), confirmed from Pybricks `pbdrvconfig.h`:
```
VOLTAGE_RAW_MAX  = 4096
VOLTAGE_SCALED_MAX = 9900
```
Formula: `battery_mV = raw * 9900 / 4096`

**ADC channel**: ch11 (PC1)

### Battery Thresholds

| Threshold | Value | Rationale |
|:----------|:------|:----------|
| Full charge | 8400 mV | 2S × 4.2V/cell |
| Auto-recharge | 8000 mV | MP2639A restarts charge below this |
| Good | >7600 mV | Green battery LED |
| Low | 7000–7600 mV | Amber battery LED |
| Warning | <6200 mV | Shell warning message |
| Critical (shutdown) | 5800 mV | Above MP2639A UV cutoff (5.75V falling) |
| Trickle threshold | 5900 mV | MP2639A enters trickle pre-charge |
| UV cutoff | 5750 mV | MP2639A shuts down charging |

### Other ADC Channels

| Channel | Pin | Signal |
|:--------|:----|:-------|
| ch3 | PA3 | USB charger input current sense |
| ch8 | PB0 | Battery NTC thermistor |
| ch10 | PC0 | Battery current |
| ch11 | PC1 | Battery voltage |
| ch14 | PC4 | Center button + /CHG resistor ladder |

### NTC Thresholds

MP2639A suspends charging when NTC voltage leaves 30–74% of VCC (3.3V):
- Cold threshold: 69.9% of VVNTC → ~0°C
- Hot threshold: 47.4% of VVNTC → ~50°C
- ADC thresholds: 1229 (30% of 4095) to 3030 (74% of 4095)

---

## Shell Commands

- `bat` — voltage, current, NTC, USB current, /CHG status, ISET duty, ladder raw value
- `load` — same as bat plus button state and critical-low warning
- `adc <ch>` — read any raw ADC channel
- `md 0x40000C34 1` — read TIM5 CCR1 (ISET duty) directly
- `mw 0x40000C34 <duty>` — override ISET duty for testing (0–100)

## Empirical Observations

- Battery voltage rose 6516 → 7229 mV (~713 mV) in ~30 min after enabling ISET PWM
- USB current sensor (ch3) responds to ISET duty: ~400 at duty=0, ~1050 at duty=15, ~1350 at duty=100
- /CHG ladder reading is consistently ~3650 (just above 3642 threshold) regardless of ISET state
- Charging works; /CHG open-drain detection is marginal on this hub unit