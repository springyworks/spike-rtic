# Pybricks Motor PWM Control Implementation Summary

## Key Architecture

### 1. **PWM Frequency & Timer Configuration (SPIKE Prime)**

**File:** lib/pbio/platform/prime_hub/platform.c

- **Motor PWM Frequency:** 12 kHz (12,000 Hz)
- **Rationale:** Changed from official LEGO firmware's 1.2 kHz to 12 kHz to reduce unpleasant noise
- **Motor Timers:** TIM1, TIM3, TIM4 (all running at 12 kHz)
  - TIM1 (PWM_DEV_0): Ports A & B motors
  - TIM3 (PWM_DEV_1): Ports E & F motors
  - TIM4 (PWM_DEV_2): Ports C & D motors
- **Timer Configuration:**
  - Prescalar: 8 (results in 12 MHz clock from 96 MHz system clock)
  - Period: 1000 (12MHz ÷ 1000 = 12 kHz PWM)
  - All channels inverted (PWM output is active-low)

### 2. **Duty Cycle Range**

- **Max Duty:** `PBDRV_MOTOR_DRIVER_MAX_DUTY = 1000`
- **Range:** -1000 to +1000 (signed 16-bit)
- **Duty vs Voltage Conversion:** Done in lib/pbio/src/battery.c
  - Formula: `duty_cycle = voltage_mV × 1000 / avg_battery_voltage_mV`

### 3. **Motor Control Modes at PWM Level**

**File:** lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c

#### **COAST Mode**
- Both pin1_gpio and pin2_gpio set LOW
- Motor floats freely with no torque

#### **BRAKE Mode (duty_cycle = 0)**
- Both GPIO pins set HIGH
- Shorts motor terminals together, creating electromagnetic braking

#### **FORWARD (positive duty_cycle)**
- pin1 PWM enabled on TIMx_CCRn output (inverted polarity)
- pin2 GPIO set HIGH
- Slow-decay: off-phase = both HIGH = brake

#### **REVERSE (negative duty_cycle)**
- pin2 PWM enabled on TIMx_CCRn output (inverted polarity)
- pin1 GPIO set HIGH

### 4. **Motor Port Pin Mapping (SPIKE Prime)**

| Port | pin1 GPIO    | pin1 PWM   | pin2 GPIO    | pin2 PWM   |
|------|--------------|------------|--------------|------------|
| A    | PE9, TIM1 CH1| TIM1 CH1   | PE11, TIM1 CH2| TIM1 CH2  |
| B    | PE13, TIM1 CH3| TIM1 CH3  | PE14, TIM1 CH4| TIM1 CH4  |
| C    | PB6, TIM4 CH1| TIM4 CH1   | PB7, TIM4 CH2| TIM4 CH2  |
| D    | PB8, TIM4 CH3| TIM4 CH3   | PB9, TIM4 CH4| TIM4 CH4  |
| E    | PC6, TIM3 CH1| TIM3 CH1   | PC7, TIM3 CH2| TIM3 CH2  |
| F    | PC8, TIM3 CH3| TIM3 CH3   | PB1, TIM3 CH4| TIM3 CH4  |

### 5. **High-Level Control Flow**

1. User sets motor voltage via servo/dcmotor API
2. Converts voltage (mV) → duty_cycle via battery module
3. Applies direction (flips sign if COUNTERCLOCKWISE)
4. H-bridge driver: forward/reverse/brake/coast based on sign
5. PWM driver sets TIMx_CCRn register
