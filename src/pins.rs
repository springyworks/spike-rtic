//! Pin mapping for the LEGO SPIKE Prime Hub (STM32F413VGT6).
//!
//! Derived from pybricks-micropython platform data and the LegoSpikeRust project.
//! See: pybricks-micropython/lib/pbio/platform/prime_hub/platform.c
//!
//! Many constants are documented here for future peripheral drivers and are
//! intentionally kept even if not yet used by the current firmware.

#![allow(dead_code)]

// ── GPIO base addresses ──
pub const GPIOA: u32 = 0x4002_0000;
pub const GPIOB: u32 = 0x4002_0400;
pub const GPIOC: u32 = 0x4002_0800;
pub const GPIOD: u32 = 0x4002_0C00;
pub const GPIOE: u32 = 0x4002_1000;

// ── GPIO register offsets ──
pub const MODER: u32 = 0x00;
pub const OTYPER: u32 = 0x04;
pub const OSPEEDR: u32 = 0x08;
pub const PUPDR: u32 = 0x0C;
pub const IDR: u32 = 0x10;
pub const ODR: u32 = 0x14;
pub const BSRR: u32 = 0x18;
pub const AFRL: u32 = 0x20;
pub const AFRH: u32 = 0x24;

// ── Power control ──
pub const POWER_KEEP_ALIVE_PORT: u32 = GPIOA;
pub const POWER_KEEP_ALIVE_PIN: u32 = 13; // PA13 — HIGH = stay on, LOW = shutdown

pub const PORT_VCC_EN_PORT: u32 = GPIOA;
pub const PORT_VCC_EN_PIN: u32 = 14; // PA14 — I/O port 3.3V

// ── USB OTG FS ──
pub const USB_DM_PORT: u32 = GPIOA;
pub const USB_DM_PIN: u32 = 11; // PA11
pub const USB_DP_PORT: u32 = GPIOA;
pub const USB_DP_PIN: u32 = 12; // PA12
pub const USB_VBUS_PORT: u32 = GPIOA;
pub const USB_VBUS_PIN: u32 = 9; // PA9

// ── TLC5955 LED matrix (SPI1) ──
pub const LED_SCK_PORT: u32 = GPIOA;
pub const LED_SCK_PIN: u32 = 5; // PA5  — SPI1_SCK
pub const LED_MISO_PORT: u32 = GPIOA;
pub const LED_MISO_PIN: u32 = 6; // PA6  — SPI1_MISO
pub const LED_MOSI_PORT: u32 = GPIOA;
pub const LED_MOSI_PIN: u32 = 7; // PA7  — SPI1_MOSI
pub const LED_LAT_PORT: u32 = GPIOA;
pub const LED_LAT_PIN: u32 = 15; // PA15 — Latch
pub const LED_GSCLK_PORT: u32 = GPIOB;
pub const LED_GSCLK_PIN: u32 = 15; // PB15 — TIM12_CH2 (9.6 MHz PWM)

// ── External SPI flash (SPI2, W25Q256) ──
pub const FLASH_CS_PORT: u32 = GPIOB;
pub const FLASH_CS_PIN: u32 = 12; // PB12
pub const FLASH_SCK_PORT: u32 = GPIOB;
pub const FLASH_SCK_PIN: u32 = 13; // PB13
pub const FLASH_MISO_PORT: u32 = GPIOC;
pub const FLASH_MISO_PIN: u32 = 2; // PC2
pub const FLASH_MOSI_PORT: u32 = GPIOC;
pub const FLASH_MOSI_PIN: u32 = 3; // PC3

// ── IMU (LSM6DS3TR-C on I2C2) ──
pub const IMU_SDA_PORT: u32 = GPIOB;
pub const IMU_SDA_PIN: u32 = 3; // PB3
pub const IMU_SCL_PORT: u32 = GPIOB;
pub const IMU_SCL_PIN: u32 = 10; // PB10
pub const IMU_INT1_PORT: u32 = GPIOB;
pub const IMU_INT1_PIN: u32 = 4; // PB4

// ── Audio ──
pub const DAC_OUT_PORT: u32 = GPIOA;
pub const DAC_OUT_PIN: u32 = 4; // PA4 — DAC1_CH1
pub const AMP_EN_PORT: u32 = GPIOC;
pub const AMP_EN_PIN: u32 = 10; // PC10

// ── Bluetooth (CC2564C via USART2) ──
pub const BT_EN_PORT: u32 = GPIOA;
pub const BT_EN_PIN: u32 = 2; // PA2
pub const BT_TX_PORT: u32 = GPIOD;
pub const BT_TX_PIN: u32 = 5; // PD5
pub const BT_RX_PORT: u32 = GPIOD;
pub const BT_RX_PIN: u32 = 6; // PD6
pub const BT_CTS_PORT: u32 = GPIOD;
pub const BT_CTS_PIN: u32 = 3; // PD3
pub const BT_RTS_PORT: u32 = GPIOD;
pub const BT_RTS_PIN: u32 = 4; // PD4

// ── Battery / ADC ──
pub const BAT_CURRENT_PORT: u32 = GPIOC;
pub const BAT_CURRENT_PIN: u32 = 0; // PC0 — ADC1_CH10
pub const BAT_VOLTAGE_PORT: u32 = GPIOC;
pub const BAT_VOLTAGE_PIN: u32 = 1; // PC1 — ADC1_CH11
pub const BAT_NTC_PORT: u32 = GPIOB;
pub const BAT_NTC_PIN: u32 = 0; // PB0 — ADC1_CH8
pub const USB_CHARGER_CURRENT_PORT: u32 = GPIOA;
pub const USB_CHARGER_CURRENT_PIN: u32 = 3; // PA3 — ADC1_CH3

// ── Buttons (resistor ladder ADC) ──
pub const BUTTON_CENTER_PORT: u32 = GPIOC;
pub const BUTTON_CENTER_PIN: u32 = 4; // PC4 — ADC1_CH14
pub const BUTTON_LR_PORT: u32 = GPIOA;
pub const BUTTON_LR_PIN: u32 = 1; // PA1 — ADC1_CH1

// ── Motor ports (H-bridge PWM) ──
// Port A: PE9 (TIM1_CH1), PE11 (TIM1_CH2)
// Port B: PE13 (TIM1_CH3), PE14 (TIM1_CH4)
// Port C: PB6 (TIM4_CH1), PB7 (TIM4_CH2)
// Port D: PB8 (TIM4_CH3), PB9 (TIM4_CH4)
// Port E: PC6 (TIM3_CH1), PC7 (TIM3_CH2)
// Port F: PC8 (TIM3_CH3), PB1 (TIM3_CH4)

pub struct MotorPort {
    pub pwm1_port: u32,
    pub pwm1_pin: u32,
    pub pwm2_port: u32,
    pub pwm2_pin: u32,
    pub timer: u32,
    pub ccr1_offset: u32,
    pub ccr2_offset: u32,
}

pub const TIM1_BASE: u32 = 0x4001_0000;
pub const TIM3_BASE: u32 = 0x4000_0400;
pub const TIM4_BASE: u32 = 0x4000_0800;

pub const MOTOR_PORT_A: MotorPort = MotorPort {
    pwm1_port: GPIOE, pwm1_pin: 9,
    pwm2_port: GPIOE, pwm2_pin: 11,
    timer: TIM1_BASE, ccr1_offset: 0x34, ccr2_offset: 0x38,
};

pub const MOTOR_PORT_B: MotorPort = MotorPort {
    pwm1_port: GPIOE, pwm1_pin: 13,
    pwm2_port: GPIOE, pwm2_pin: 14,
    timer: TIM1_BASE, ccr1_offset: 0x3C, ccr2_offset: 0x40,
};

pub const MOTOR_PORT_C: MotorPort = MotorPort {
    pwm1_port: GPIOB, pwm1_pin: 6,
    pwm2_port: GPIOB, pwm2_pin: 7,
    timer: TIM4_BASE, ccr1_offset: 0x34, ccr2_offset: 0x38,
};

pub const MOTOR_PORT_D: MotorPort = MotorPort {
    pwm1_port: GPIOB, pwm1_pin: 8,
    pwm2_port: GPIOB, pwm2_pin: 9,
    timer: TIM4_BASE, ccr1_offset: 0x3C, ccr2_offset: 0x40,
};

pub const MOTOR_PORT_E: MotorPort = MotorPort {
    pwm1_port: GPIOC, pwm1_pin: 6,
    pwm2_port: GPIOC, pwm2_pin: 7,
    timer: TIM3_BASE, ccr1_offset: 0x34, ccr2_offset: 0x38,
};

pub const MOTOR_PORT_F: MotorPort = MotorPort {
    pwm1_port: GPIOC, pwm1_pin: 8,
    pwm2_port: GPIOB, pwm2_pin: 1,
    timer: TIM3_BASE, ccr1_offset: 0x3C, ccr2_offset: 0x40,
};

// ── I/O port UARTs + device-ID sense pins ──
// Each port has a UART for smart-device comms and two sense/ID pins
// used by the LEGO protocol to detect what is plugged in (motor vs
// sensor vs nothing).  These are **digital GPIO**, NOT ADC-connected
// current-sense pins.  Per-port motor current is not measurable via
// ADC on this board; use battery current (ADC ch10) for total load.
//
// Port A: UART7  (TX=PE8,  RX=PE7,  AF8)   ID: PD7,PD8
// Port B: UART4  (TX=PD1,  RX=PD0,  AF11)  ID: PD9,PD10
// Port C: UART8  (TX=PE1,  RX=PE0,  AF8)   ID: PD11,PE4
// Port D: UART5  (TX=PC12, RX=PD2,  AF8)   ID: PC15,PC14
// Port E: UART10 (TX=PE3,  RX=PE2,  AF11)  ID: PC13,PE12
// Port F: UART9  (TX=PD15, RX=PD14, AF11)  ID: PC11,PE6

// ── TLC5955 LED channel → 5×5 matrix pixel index mapping ──
// Row-major pixel order (top-left=0, bottom-right=24)
// Channels sourced from pybricks/LegoSpikeRust testing.
pub const MATRIX_CHANNELS: [u8; 25] = [
    38, 36, 41, 46, 33,
    37, 28, 39, 47, 21,
    24, 29, 31, 45, 23,
    26, 27, 32, 34, 22,
    25, 40, 30, 35,  9,
];

// Status LED channels (RGB triplets)
pub const STATUS_TOP: (u8, u8, u8) = (5, 4, 3);       // R, G, B
pub const STATUS_BOT: (u8, u8, u8) = (8, 7, 6);       // R, G, B
pub const BATTERY_LED: (u8, u8, u8) = (2, 1, 0);      // R, G, B
pub const BLUETOOTH_LED: (u8, u8, u8) = (20, 19, 18); // R, G, B

// ADC thresholds for button resistor ladder
pub const BUTTON_CENTER_THRESHOLD: u32 = 3200;
pub const LR_LEVELS: [u32; 8] = [3872, 3394, 3009, 2755, 2538, 2327, 2141, 1969];

// ── Battery charger (MP2639A) ──
//
// The charger MODE pin is routed through TLC5955 channel 14 (not a
// regular GPIO).  When the GS value is 0 the TLC5955 channel is off
// and the MODE pin is pulled LOW by its external pull-down → charge
// mode (buck).  Setting ch14 to max sinks current through the pull-up
// circuit, pulling MODE HIGH → discharge mode (boost).
//
// The charger /CHG (STAT) signal shares a resistor ladder with the
// center button on ADC ch14 (PC4).  Three inputs create 8 discrete
// voltage levels decoded as a 3-bit flags word:
//   bit 0 (CH_0) = center button pressed
//   bit 1 (CH_1) = unused input
//   bit 2 (CH_2) = /CHG active (LOW = battery is charging)
//
// Thresholds from Pybricks RESISTOR_LADDER_DEV_0:
pub const CHARGER_MODE_CH: u8 = 14;
pub const RLAD_CENTER: [u32; 8] = [3642, 3142, 2879, 2634, 2449, 2209, 2072, 1800];

// NTC healthy range: 30%–74% of VCC (3.3 V).
// ADC is 12-bit (0–4095), so thresholds are 0.30*4095 ≈ 1229 and 0.74*4095 ≈ 3030.
pub const NTC_LOW_THRESHOLD: u32 = 1229;
pub const NTC_HIGH_THRESHOLD: u32 = 3030;
