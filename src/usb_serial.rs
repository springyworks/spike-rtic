//! USB CDC-ACM serial driver for LEGO SPIKE Prime Hub.
//!
//! Uses the Synopsys USB OTG FS peripheral (PA11 D−, PA12 D+) at
//! `0x5000_0000`.  Provides a virtual COM port (`/dev/ttyACM0` on Linux)
//! for the interactive shell, binary upload (COBS), and streaming output.
//!
//! # Contents
//!
//! - [`SpikeUsb`] — implements `synopsys_usb_otg::UsbPeripheral` for OTG_FS
//! - [`init()`]   — enables clocks, configures GPIO alternate functions,
//!   allocates the `UsbBus`, and builds the `SerialPort` + `UsbDevice`
//! - [`poll_usb()`] — called from the OTG_FS ISR (priority 2) to drain
//!   IN/OUT endpoints and detect enumeration events
//!
//! # Hardware
//!
//! | Signal | Pin  | AF  | Notes               |
//! |--------|------|-----|---------------------|
//! | D−     | PA11 | AF10| OTG_FS_DM           |
//! | D+     | PA12 | AF10| OTG_FS_DP           |
//! | VBUS   | PA9  | —   | sensed but not used |
//!
//! Clocks: AHB2 `OTG_FS` clock enabled; 48 MHz from PLL Q divider.

use synopsys_usb_otg::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

use crate::pins;
use crate::reg::{reg_modify, reg_read};

const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_AHB2ENR: u32 = 0x34;

/// OTG_FS peripheral descriptor for synopsys-usb-otg.
pub struct SpikeUsbOtg;

/// # Safety
/// The REGISTERS pointer points to the STM32F413 OTG_FS register block.
/// GPIO and clock configuration is done in `init()` before use.
unsafe impl Sync for SpikeUsbOtg {}
unsafe impl Send for SpikeUsbOtg {}

unsafe impl synopsys_usb_otg::UsbPeripheral for SpikeUsbOtg {
    const REGISTERS: *const () = 0x5000_0000 as *const ();
    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 320; // 1.25 KB FIFO for OTG_FS
    const ENDPOINT_COUNT: usize = 6;

    fn enable() {
        // Clock is already enabled in init(), but ensure it here too
        unsafe {
            reg_modify(RCC, RCC_AHB2ENR, 0, 1 << 7); // OTG_FS clock
        }
    }

    fn ahb_frequency_hz(&self) -> u32 {
        96_000_000
    }
}

/// Static USB bus allocator — must live forever.
static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static mut USB_BUS: Option<UsbBusAllocator<UsbBus<SpikeUsbOtg>>> = None;

/// Initialize USB OTG FS GPIO and clocks.
///
/// # Safety
/// Must be called once. Returns the bus allocator reference for creating devices.
pub unsafe fn init() -> &'static UsbBusAllocator<UsbBus<SpikeUsbOtg>> {
    // Enable GPIOA clock
    reg_modify(RCC, RCC_AHB1ENR, 0, 1 << 0);
    // Enable USB OTG FS clock
    reg_modify(RCC, RCC_AHB2ENR, 0, 1 << 7);
    let _ = reg_read(RCC, RCC_AHB2ENR);

    // PA11 (DM), PA12 (DP) → AF10 (OTG_FS), very high speed
    reg_modify(
        pins::GPIOA,
        pins::MODER,
        (3 << 22) | (3 << 24),
        (2 << 22) | (2 << 24),
    );
    reg_modify(
        pins::GPIOA,
        pins::OSPEEDR,
        (3 << 22) | (3 << 24),
        (3 << 22) | (3 << 24),
    );
    reg_modify(
        pins::GPIOA,
        pins::AFRH,
        (0xF << 12) | (0xF << 16),
        (10 << 12) | (10 << 16),
    );

    // PA9 (VBUS) → input (no pull, external sensing)
    reg_modify(pins::GPIOA, pins::MODER, 3 << 18, 0);

    let usb = SpikeUsbOtg;

    let ep_mem = &mut *core::ptr::addr_of_mut!(EP_MEMORY);
    *core::ptr::addr_of_mut!(USB_BUS) = Some(UsbBus::new(usb, ep_mem));
    (*core::ptr::addr_of!(USB_BUS)).as_ref().unwrap()
}

/// Create a CDC serial port and USB device from the bus allocator.
pub fn create_device(
    bus: &'static UsbBusAllocator<UsbBus<SpikeUsbOtg>>,
) -> (
    SerialPort<'static, UsbBus<SpikeUsbOtg>>,
    UsbDevice<'static, UsbBus<SpikeUsbOtg>>,
) {
    let serial = SerialPort::new(bus);
    let usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(0x0694, 0x0042))
        .strings(&[StringDescriptors::default()
            .manufacturer("LEGO-RTIC")
            .product("Spike Prime RTIC Shell")
            .serial_number("RTIC001")])
        .expect("string descriptor error")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();
    (serial, usb_dev)
}
