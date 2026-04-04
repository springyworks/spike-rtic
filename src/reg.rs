//! Low-level volatile register access helpers for STM32F413.
//!
//! Provides `reg_read`, `reg_write`, and `reg_modify` for direct memory-mapped
//! peripheral I/O.  All operations use `core::ptr::{read,write}_volatile` to
//! prevent the compiler from reordering, caching, or eliminating hardware
//! register accesses.
//!
//! # Functions
//!
//! - [`reg_read`]  — read a 32-bit register at `base + offset`
//! - [`reg_write`] — write a 32-bit value to `base + offset`
//! - [`reg_modify`] — atomic read-modify-write: clear `clear` bits, set `set` bits
//!
//! # Safety
//!
//! All functions are `unsafe` — the caller must ensure addresses point to valid
//! memory-mapped peripherals and that concurrent access is properly synchronized.
//! In RTIC this is guaranteed by priority-based critical sections.

/// Write a 32-bit value to a memory-mapped register.
#[inline(always)]
pub unsafe fn reg_write(base: u32, offset: u32, val: u32) {
    core::ptr::write_volatile((base + offset) as *mut u32, val);
}

/// Read a 32-bit value from a memory-mapped register.
#[inline(always)]
pub unsafe fn reg_read(base: u32, offset: u32) -> u32 {
    core::ptr::read_volatile((base + offset) as *const u32)
}

/// Modify a register: clear bits in `clear` mask, then set bits in `set` mask.
#[inline(always)]
pub unsafe fn reg_modify(base: u32, offset: u32, clear: u32, set: u32) {
    let v = reg_read(base, offset);
    reg_write(base, offset, (v & !clear) | set);
}
