//! Internal SRAM test for STM32F413VGT6.
//!
//! Memory map:
//!   SRAM1  256 KB  0x2000_0000 – 0x2003_FFFF
//!   SRAM2   64 KB  0x2004_0000 – 0x2004_FFFF
//!
//! This module provides:
//!   - `raminfo`: report memory map and usage (stack pointer, linker symbols)
//!   - `ramtest`: non-destructive March C- test on a caller-supplied buffer,
//!     plus a safe walk of the upper SRAM2 region that the firmware doesn't use.

use core::fmt::{self, Write};

// ── SRAM region constants ──

pub const SRAM1_BASE: u32 = 0x2000_0000;
pub const SRAM1_SIZE: u32 = 256 * 1024;
pub const SRAM1_END: u32 = SRAM1_BASE + SRAM1_SIZE;

pub const SRAM2_BASE: u32 = 0x2004_0000;
pub const SRAM2_SIZE: u32 = 64 * 1024;
pub const SRAM2_END: u32 = SRAM2_BASE + SRAM2_SIZE;

pub const TOTAL_SRAM: u32 = SRAM1_SIZE + SRAM2_SIZE; // 320 KB

/// Get the current stack pointer.
#[inline(always)]
fn read_sp() -> u32 {
    let sp: u32;
    unsafe { core::arch::asm!("mov {}, sp", out(reg) sp) };
    sp
}

// Linker-provided symbols (cortex-m-rt).
extern "C" {
    static __sdata: u8;
    static __edata: u8;
    static __sbss: u8;
    static __ebss: u8;
}

fn sym_addr(s: &u8) -> u32 {
    (s as *const u8) as u32
}

// ── RAM info ──

/// Minimal buffer writer (no alloc).
struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}
impl<'a> BufWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn written(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}
impl<'a> Write for BufWriter<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let n = core::cmp::min(self.buf.len() - self.pos, bytes.len());
        self.buf[self.pos..self.pos + n].copy_from_slice(&bytes[..n]);
        self.pos += n;
        Ok(())
    }
}

/// Print memory map info via the provided write function.
pub fn cmd_raminfo<F: FnMut(&[u8])>(write_fn: &mut F) {
    let sp = read_sp();
    let data_start = unsafe { sym_addr(&__sdata) };
    let data_end = unsafe { sym_addr(&__edata) };
    let bss_start = unsafe { sym_addr(&__sbss) };
    let bss_end = unsafe { sym_addr(&__ebss) };
    // Stack top = end of RAM (grows downward)
    let stack_top = SRAM1_END;

    write_fn(b"=== STM32F413 SRAM Map ===\r\n");

    let mut tmp = [0u8; 80];

    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, "SRAM1: 0x{:08X}..0x{:08X} ({}K)\r\n", SRAM1_BASE, SRAM1_END, SRAM1_SIZE / 1024);
    write_fn(w.written());

    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, "SRAM2: 0x{:08X}..0x{:08X} ({}K)\r\n", SRAM2_BASE, SRAM2_END, SRAM2_SIZE / 1024);
    write_fn(w.written());

    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, "Total: {}K\r\n", TOTAL_SRAM / 1024);
    write_fn(w.written());

    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, ".data:  0x{:08X}..0x{:08X}\r\n", data_start, data_end);
    write_fn(w.written());

    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, ".bss:   0x{:08X}..0x{:08X}\r\n", bss_start, bss_end);
    write_fn(w.written());

    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, "SP:     0x{:08X}  (top 0x{:08X})\r\n", sp, stack_top);
    write_fn(w.written());

    // Free RAM = between end of BSS and current SP (stack grows down)
    let free = sp.saturating_sub(bss_end);
    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, "Free (heap area): {} bytes\r\n", free);
    write_fn(w.written());
}

// ── March C- Memory Test ──
//
// A simplified March C- test sequence (6 phases):
//   ⇑ w0  — ascending write 0
//   ⇑ r0,w1 — ascending read 0, write 1
//   ⇑ r1,w0 — ascending read 1, write 0
//   ⇓ r0,w1 — descending read 0, write 1
//   ⇓ r1,w0 — descending read 1, write 0
//   ⇑ r0 — ascending verify 0
//
// Uses word-granularity (u32) with patterns 0x0000_0000 / 0xFFFF_FFFF.

/// Result of a RAM test run.
pub struct RamTestResult {
    /// Number of u32 words tested.
    pub words_tested: u32,
    /// Address of first failure, or 0 if PASS.
    pub first_fail_addr: u32,
    /// Expected value at first failure.
    pub expected: u32,
    /// Actual value read at first failure.
    pub actual: u32,
    /// true if test passed.
    pub pass: bool,
}

/// Run March C- test on the given address range (must be word-aligned).
///
/// # Safety
/// The caller must guarantee that `base..base+size` is valid, mapped SRAM
/// that is not in use (not overlapping stack, .data, .bss, or DMA buffers).
pub unsafe fn march_c_test(base: u32, size: u32) -> RamTestResult {
    let words = size / 4;
    let ptr = base as *mut u32;

    // Phase 1: ⇑ w0
    for i in 0..words {
        core::ptr::write_volatile(ptr.add(i as usize), 0x0000_0000);
    }

    // Phase 2: ⇑ r0, w1
    for i in 0..words {
        let addr = ptr.add(i as usize);
        let val = core::ptr::read_volatile(addr);
        if val != 0x0000_0000 {
            return RamTestResult {
                words_tested: i,
                first_fail_addr: addr as u32,
                expected: 0x0000_0000,
                actual: val,
                pass: false,
            };
        }
        core::ptr::write_volatile(addr, 0xFFFF_FFFF);
    }

    // Phase 3: ⇑ r1, w0
    for i in 0..words {
        let addr = ptr.add(i as usize);
        let val = core::ptr::read_volatile(addr);
        if val != 0xFFFF_FFFF {
            return RamTestResult {
                words_tested: i,
                first_fail_addr: addr as u32,
                expected: 0xFFFF_FFFF,
                actual: val,
                pass: false,
            };
        }
        core::ptr::write_volatile(addr, 0x0000_0000);
    }

    // Phase 4: ⇓ r0, w1
    for i in (0..words).rev() {
        let addr = ptr.add(i as usize);
        let val = core::ptr::read_volatile(addr);
        if val != 0x0000_0000 {
            return RamTestResult {
                words_tested: words - i,
                first_fail_addr: addr as u32,
                expected: 0x0000_0000,
                actual: val,
                pass: false,
            };
        }
        core::ptr::write_volatile(addr, 0xFFFF_FFFF);
    }

    // Phase 5: ⇓ r1, w0
    for i in (0..words).rev() {
        let addr = ptr.add(i as usize);
        let val = core::ptr::read_volatile(addr);
        if val != 0xFFFF_FFFF {
            return RamTestResult {
                words_tested: words - i,
                first_fail_addr: addr as u32,
                expected: 0xFFFF_FFFF,
                actual: val,
                pass: false,
            };
        }
        core::ptr::write_volatile(addr, 0x0000_0000);
    }

    // Phase 6: ⇑ r0 (final verify)
    for i in 0..words {
        let addr = ptr.add(i as usize);
        let val = core::ptr::read_volatile(addr);
        if val != 0x0000_0000 {
            return RamTestResult {
                words_tested: i,
                first_fail_addr: addr as u32,
                expected: 0x0000_0000,
                actual: val,
                pass: false,
            };
        }
    }

    RamTestResult {
        words_tested: words,
        first_fail_addr: 0,
        expected: 0,
        actual: 0,
        pass: true,
    }
}

/// Run March C- on a static test buffer (safe, non-destructive).
/// Tests 4 KB of dedicated buffer to prove basic RAM health.
pub fn cmd_ramtest_safe<F: FnMut(&[u8])>(write_fn: &mut F) {
    // 4 KB static test buffer — placed in BSS, only used during this test
    static mut TEST_BUF: [u32; 1024] = [0u32; 1024];

    let base = core::ptr::addr_of!(TEST_BUF) as u32;
    let size = (1024 * core::mem::size_of::<u32>()) as u32;

    let mut tmp = [0u8; 80];

    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, "Testing 0x{:08X}..0x{:08X} ({}B)...\r\n", base, base + size, size);
    write_fn(w.written());

    let result = unsafe { march_c_test(base, size) };
    report_result(write_fn, &result, "safe-buf");
}

/// Run extended test on SRAM2 (0x2004_0000 – 0x2004_FFFF, 64 KB).
///
/// The firmware's linker script uses one contiguous 320K block starting at
/// 0x2000_0000, but the stack, .data, and .bss typically fit within SRAM1.
/// SRAM2 may be partially/fully unused — this tests the top 32 KB of SRAM2
/// which is almost certainly free.
pub fn cmd_ramtest_sram2<F: FnMut(&[u8])>(write_fn: &mut F) {
    let bss_end = unsafe { sym_addr(&__ebss) };

    // If BSS ends before SRAM2, the whole of SRAM2 is free for testing
    // Otherwise test from bss_end (aligned up to 4) to SRAM2_END
    let test_base = if bss_end < SRAM2_BASE {
        SRAM2_BASE
    } else if bss_end < SRAM2_END {
        (bss_end + 3) & !3 // align up to 4
    } else {
        write_fn(b"SRAM2 fully used by firmware, skipping.\r\n");
        return;
    };

    // Leave a 256-byte guard above the current SP in case we're in SRAM2
    let sp = read_sp();
    let test_end = if sp > test_base && sp < SRAM2_END {
        // Stack is in SRAM2 — don't touch below SP minus guard
        let safe_end = sp.saturating_sub(256) & !3;
        if safe_end <= test_base {
            write_fn(b"SRAM2 too close to stack, skipping.\r\n");
            return;
        }
        safe_end
    } else {
        SRAM2_END
    };

    let size = test_end - test_base;

    let mut tmp = [0u8; 80];
    let mut w = BufWriter::new(&mut tmp);
    let _ = write!(w, "Testing SRAM2 0x{:08X}..0x{:08X} ({}K)...\r\n",
                   test_base, test_end, size / 1024);
    write_fn(w.written());

    let result = unsafe { march_c_test(test_base, size) };
    report_result(write_fn, &result, "SRAM2");
}

/// Run the full test suite: safe buffer + SRAM2.
pub fn cmd_ramtest_all<F: FnMut(&[u8])>(write_fn: &mut F) {
    write_fn(b"=== RAM Test Suite ===\r\n");
    write_fn(b"\r\n[1/2] Safe buffer (4KB March C-):\r\n");
    cmd_ramtest_safe(write_fn);
    write_fn(b"\r\n[2/2] SRAM2 region:\r\n");
    cmd_ramtest_sram2(write_fn);
    write_fn(b"\r\n=== Done ===\r\n");
}

/// Format and emit a test result.
fn report_result<F: FnMut(&[u8])>(write_fn: &mut F, r: &RamTestResult, label: &str) {
    let mut tmp = [0u8; 96];

    if r.pass {
        let mut w = BufWriter::new(&mut tmp);
        let _ = write!(w, "PASS [{}]: {} words OK\r\n", label, r.words_tested);
        write_fn(w.written());
    } else {
        let mut w = BufWriter::new(&mut tmp);
        let _ = write!(w, "FAIL [{}] @ 0x{:08X}: exp 0x{:08X} got 0x{:08X}\r\n",
                       label, r.first_fail_addr, r.expected, r.actual);
        write_fn(w.written());
    }
}

/// Quick address-bus test: write unique pattern to power-of-two offsets,
/// then verify. Catches address line faults.
pub fn cmd_addrtest<F: FnMut(&[u8])>(write_fn: &mut F) {
    static mut ADDR_BUF: [u32; 1024] = [0u32; 1024]; // 4 KB
    let base = core::ptr::addr_of_mut!(ADDR_BUF) as *mut u32;
    let len: u32 = 1024;

    // Write address-unique patterns at power-of-2 offsets
    let mut offset: u32 = 1;
    unsafe {
        core::ptr::write_volatile(base, 0xDEAD_0000);
        while offset < len {
            core::ptr::write_volatile(base.add(offset as usize), 0xDEAD_0000 | offset);
            offset <<= 1;
        }

        // Verify
        let v = core::ptr::read_volatile(base);
        if v != 0xDEAD_0000 {
            let mut tmp = [0u8; 80];
            let mut w = BufWriter::new(&mut tmp);
            let _ = write!(w, "FAIL addr[0]: exp 0xDEAD0000, got 0x{:08X}\r\n", v);
            write_fn(w.written());
            return;
        }

        offset = 1;
        while offset < len {
            let expected = 0xDEAD_0000 | offset;
            let v = core::ptr::read_volatile(base.add(offset as usize));
            if v != expected {
                let mut tmp = [0u8; 80];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "FAIL addr[{}]: exp 0x{:08X}, got 0x{:08X}\r\n",
                               offset, expected, v);
                write_fn(w.written());
                return;
            }
            offset <<= 1;
        }
    }

    write_fn(b"PASS address-bus test\r\n");
}
