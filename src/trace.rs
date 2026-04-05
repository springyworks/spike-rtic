#![allow(dead_code)]
//! Lightweight RAM trace buffer for post-mortem debugging.
//!
//! Records timestamped trace entries in a fixed-size ring buffer
//! in SRAM1.  Entries are written by firmware code (SVC handler,
//! sensor module) and dumped via the `trace` shell command.
//!
//! Each entry is 8 bytes:
//!   - tag:  u8  — what happened (see TAG_* constants)
//!   - val:  u8  — secondary value (SVC return, byte count, etc.)
//!   - arg:  u16 — primary argument (SVC number, port, etc.)
//!   - tick: u32 — monotonic counter (increments per entry)
//!
//! Ring buffer: 256 entries × 8 bytes = 2 KB of SRAM1.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

// ── Trace tags ─────────────────────────────────────────────

/// SVC dispatch entry (arg=svc_num, val=r0 low byte)
pub const TAG_SVC: u8 = 0x01;
/// SVC return (arg=svc_num, val=ret low byte)
pub const TAG_SVC_RET: u8 = 0x02;
/// sensor_read called (arg=ring_buf_avail, val=data_len)
pub const TAG_SENSOR_READ: u8 = 0x10;
/// sensor_read data result (arg=first 2 data bytes, val=total_len)
pub const TAG_SENSOR_DATA: u8 = 0x11;
/// keepalive sent (arg=port_idx, val=source: 0=sensor_poll, 1=demo, 2=delay)
pub const TAG_KEEPALIVE: u8 = 0x12;
/// lump_poll_data message (arg=hdr_byte, val=data_len)
pub const TAG_LUMP_MSG: u8 = 0x13;
/// Ring buffer state (arg=head, val=tail low byte)
pub const TAG_RING_STATE: u8 = 0x14;
/// delay_ms start (arg=ms low 16, val=0)
pub const TAG_DELAY_START: u8 = 0x20;
/// delay_ms chunk keepalive (arg=remaining_ms, val=0)
pub const TAG_DELAY_KA: u8 = 0x21;
/// mode switch (arg=mode, val=port)
pub const TAG_MODE_SWITCH: u8 = 0x30;
/// demo start (arg=addr low 16, val=0)
pub const TAG_DEMO_START: u8 = 0x40;
/// demo end (arg=result low 16, val=faulted)
pub const TAG_DEMO_END: u8 = 0x41;
/// Custom demo trace (arg=user_val, val=user_tag)
pub const TAG_USER: u8 = 0x80;

// ── Trace buffer ───────────────────────────────────────────

const TRACE_ENTRIES: usize = 256;

#[repr(C)]
#[derive(Clone, Copy)]
struct TraceEntry {
    tag: u8,
    val: u8,
    arg: u16,
    tick: u32,
}

const EMPTY_ENTRY: TraceEntry = TraceEntry { tag: 0, val: 0, arg: 0, tick: 0 };

static mut TRACE_BUF: [TraceEntry; TRACE_ENTRIES] = [EMPTY_ENTRY; TRACE_ENTRIES];
static TRACE_POS: AtomicU32 = AtomicU32::new(0);
static TRACE_TICK: AtomicU32 = AtomicU32::new(0);
static TRACE_ENABLED: AtomicBool = AtomicBool::new(false);

// ── Public API ─────────────────────────────────────────────

/// Enable or disable tracing.
pub fn set_enabled(en: bool) {
    TRACE_ENABLED.store(en, Ordering::Release);
}

/// Check if tracing is enabled.
pub fn is_enabled() -> bool {
    TRACE_ENABLED.load(Ordering::Relaxed)
}

/// Clear the trace buffer.
pub fn clear() {
    TRACE_POS.store(0, Ordering::Release);
    TRACE_TICK.store(0, Ordering::Release);
}

/// Record a trace entry.  No-op if tracing is disabled.
///
/// Safe to call from any priority level — uses atomic position counter.
#[inline]
pub fn record(tag: u8, val: u8, arg: u16) {
    if !TRACE_ENABLED.load(Ordering::Relaxed) {
        return;
    }
    let tick = TRACE_TICK.fetch_add(1, Ordering::Relaxed);
    let pos = TRACE_POS.fetch_add(1, Ordering::Relaxed) as usize % TRACE_ENTRIES;
    unsafe {
        TRACE_BUF[pos] = TraceEntry { tag, val, arg, tick };
    }
}

/// Number of entries recorded (may exceed TRACE_ENTRIES if wrapped).
pub fn count() -> u32 {
    TRACE_POS.load(Ordering::Relaxed)
}

/// Dump the trace buffer via a callback.
///
/// Iterates from oldest to newest entry, calling `emit` for each.
/// Format: "TICK TAG VAL ARG\r\n"
pub fn dump(mut emit: impl FnMut(&[u8])) {
    let total = TRACE_POS.load(Ordering::Relaxed);
    if total == 0 {
        emit(b"(trace empty)\r\n");
        return;
    }

    let count = total.min(TRACE_ENTRIES as u32);
    let start = if total > TRACE_ENTRIES as u32 {
        (total as usize) % TRACE_ENTRIES
    } else {
        0
    };

    let mut tmp = [0u8; 48];
    for i in 0..count as usize {
        let idx = (start + i) % TRACE_ENTRIES;
        let e = unsafe { TRACE_BUF[idx] };
        if e.tag == 0 && e.tick == 0 {
            continue;
        }
        let mut w = crate::shell::BufWriter::new(&mut tmp);
        let _ = core::fmt::Write::write_fmt(
            &mut w,
            format_args!(
                "{:6} {:02X} {:02X} {:04X}\r\n",
                e.tick, e.tag, e.val, e.arg
            ),
        );
        emit(w.written());
    }

    let mut tmp2 = [0u8; 32];
    let mut w2 = crate::shell::BufWriter::new(&mut tmp2);
    let _ = core::fmt::Write::write_fmt(
        &mut w2,
        format_args!("({} entries, {} total)\r\n", count, total),
    );
    emit(w2.written());
}

/// Dump trace tags legend.
pub fn dump_legend(mut emit: impl FnMut(&[u8])) {
    emit(b"Trace tags:\r\n");
    emit(b"  01=SVC 02=SVC_RET 10=SENS_RD 11=SENS_DAT\r\n");
    emit(b"  12=KEEPALIVE 13=LUMP_MSG 14=RING_ST\r\n");
    emit(b"  20=DLY_START 21=DLY_KA 30=MODE_SW\r\n");
    emit(b"  40=DEMO_START 41=DEMO_END 80=USER\r\n");
    emit(b"Format: TICK TAG VAL ARG\r\n");
}
