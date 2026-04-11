//! Typed peripheral tokens for compile-time register ownership.
//!
//! Each token represents exclusive ownership of a memory-mapped peripheral.
//! Tokens are created once during init via `take()` and stored in module-level
//! [`OncePeripheral`] cells.  After init, modules access registers through
//! safe methods on the token — no `unsafe` needed at call sites.
//!
//! ## Single-owner guarantee
//!
//! `take()` uses an atomic flag; calling it twice panics immediately.
//! This prevents two modules from accidentally accessing the same peripheral.
//!
//! ## Zero runtime cost
//!
//! Tokens are zero-sized types.  `read()`, `write()`, `modify()`
//! inline to the same `volatile` loads/stores as the raw `reg_*()` functions.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

// ── Write-once cell for peripheral token storage ──────────────────────

/// A write-once cell for storing a peripheral token at module level.
///
/// Written exactly once during `init()` (single-threaded, interrupts
/// disabled), then read-only from RTIC tasks.
pub struct OncePeripheral<T> {
    done: AtomicBool,
    val: UnsafeCell<MaybeUninit<T>>,
}

// SAFETY: init writes once before interrupts are enabled;
// subsequent reads are never concurrent with the write.
unsafe impl<T: Send> Sync for OncePeripheral<T> {}

impl<T> OncePeripheral<T> {
    pub const fn new() -> Self {
        Self {
            done: AtomicBool::new(false),
            val: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// Store the token.  Panics if already initialised.
    pub fn set(&self, val: T) {
        assert!(
            !self.done.swap(true, Ordering::SeqCst),
            "OncePeripheral set twice"
        );
        unsafe { (*self.val.get()).write(val); }
    }

    /// Borrow the stored token.
    ///
    /// In debug builds, panics if `set()` was not called.
    /// In release builds the check is elided for zero overhead.
    #[inline(always)]
    pub fn get(&self) -> &T {
        debug_assert!(self.done.load(Ordering::Relaxed), "OncePeripheral not set");
        unsafe { &*(*self.val.get()).as_ptr() }
    }
}

// ── Peripheral token definition macro ─────────────────────────────────

macro_rules! peripheral {
    ($doc:expr, $name:ident, $base:expr, $guard:ident) => {
        static $guard: AtomicBool = AtomicBool::new(false);

#[allow(dead_code)]
        #[doc = $doc]
        pub struct $name(());

        #[allow(dead_code)]
        impl $name {
            /// Take exclusive ownership.  Panics if already taken.
            pub fn take() -> Self {
                assert!(
                    !$guard.swap(true, Ordering::SeqCst),
                    concat!(stringify!($name), " already taken")
                );
                Self(())
            }

            #[inline(always)]
            pub fn write(&self, offset: u32, val: u32) {
                unsafe { crate::reg::reg_write($base, offset, val) }
            }

            #[inline(always)]
            pub fn read(&self, offset: u32) -> u32 {
                unsafe { crate::reg::reg_read($base, offset) }
            }

            #[inline(always)]
            pub fn modify(&self, offset: u32, clear: u32, set: u32) {
                unsafe { crate::reg::reg_modify($base, offset, clear, set) }
            }

            /// Write a single byte to a register (for 8-bit SPI DR access).
            #[inline(always)]
            pub fn write_byte(&self, offset: u32, val: u8) {
                unsafe {
                    core::ptr::write_volatile(($base + offset) as *mut u8, val)
                }
            }

            /// Read a single byte from a register (for 8-bit SPI DR access).
            #[inline(always)]
            pub fn read_byte(&self, offset: u32) -> u8 {
                unsafe {
                    core::ptr::read_volatile(($base + offset) as *const u8)
                }
            }
        }
    };
}

// ── Exclusively-owned peripheral tokens ───────────────────────────────

peripheral!("TIM6 basic timer — DAC trigger for speaker tone generation.",
    Tim6, 0x4000_1000, TIM6_TAKEN);

peripheral!("DAC1 — triangle wave output to speaker amplifier (PA4).",
    Dac1, 0x4000_7400, DAC1_TAKEN);

peripheral!("SPI1 — TLC5955 LED matrix driver.",
    Spi1, 0x4001_3000, SPI1_TAKEN);

peripheral!("TIM12 — GSCLK for TLC5955 LED driver.",
    Tim12, 0x4000_1800, TIM12_TAKEN);

peripheral!("SPI2 — W25Q256JV external SPI flash.",
    Spi2, 0x4000_3800, SPI2_TAKEN);

peripheral!("TIM5 — ISET PWM for MP2639A battery charger.",
    Tim5, 0x4000_0C00, TIM5_TAKEN);

peripheral!("IWDG — Independent Watchdog.",
    Iwdg, 0x4000_3000, IWDG_TAKEN);

peripheral!("ADC1 — button and battery readings.",
    Adc1, 0x4001_2000, ADC1_TAKEN);
