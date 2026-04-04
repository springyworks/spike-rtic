# Motor Encoder on SPIKE Prime Hub - Findings

## Key Result: ALL via UART/LUMP, NO hardware quadrature encoder

- `PBDRV_CONFIG_COUNTER (0)` — counter driver is DISABLED on prime_hub
- All 6 ports have `counter_driver_index = PBDRV_IOPORT_INDEX_NOT_AVAILABLE`
- Motor position is read entirely via LUMP protocol data messages over UART

## Two motor types:
1. **Absolute motors** (Technic L/M Angular): CALIB mode (mode 4), reports absolute angle in decidegrees (0-3600) as int16_t at bin_data[2..3]
2. **Relative motors** (Interactive Motor, type 38): POS mode (mode 2), reports incremental angle in degrees as int32_t at bin_data[0..3]

## LUMP modes for position:
- `LEGO_DEVICE_MODE_PUP_ABS_MOTOR__CALIB = 4` — absolute motors default to this
- `LEGO_DEVICE_MODE_PUP_REL_MOTOR__POS = 2` — relative motors default to this
- Pybricks selects default mode right after LUMP ACK + baud rate change

## Port A pin assignments:
- UART TX: PE8, AF8 (UART7)
- UART RX: PE7, AF8 (UART7)  
- UART buf: PA10
- p5: PD7, p6: PD8 (for DCM/ID)
- Motor H-bridge via motor_driver_index 0 (separate PWM)
