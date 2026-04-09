#![allow(dead_code)]
#![allow(static_mut_refs)]
//! GDB Remote Serial Protocol (RSP) stub — "Demon mode".
//!
//! Implements a minimal GDB stub that runs over the same USB CDC serial
//! as the interactive shell.  The `gdb` shell command switches the hub
//! into this mode; GDB on the PC connects with:
//!
//! ```text
//! arm-none-eabi-gdb -ex "target remote /dev/ttyACM0"
//! ```
//!
//! ## Supported RSP commands (minimal viable set)
//!
//!   `?`           — stop reason (returns T05 = SIGTRAP)
//!   `g`           — read all registers (R0–R15 + xPSR, 17×32-bit)
//!   `G`           — write all registers
//!   `m addr,len`  — read memory (hex encoded)
//!   `M addr,len:` — write memory (hex encoded)
//!   `Z2/3/4,a,l`  — set hardware watchpoint (write/read/access)
//!   `z2/3/4,a,l`  — remove hardware watchpoint
//!   `c`           — continue execution
//!   `s`           — single step
//!   `D`           — detach (return to shell)
//!   `k`           — kill (same as detach for us)
//!   `qSupported`  — feature negotiation
//!   `Hg0` / `Hc0` — thread select (stub: OK)
//!
//! ## Packet format
//!
//!   `$<data>#<checksum>`  where checksum = sum of data bytes & 0xFF
//!   ACK = `+`, NAK = `-`
//!
//! ## Architecture
//!
//! The stub does NOT run in an interrupt or exception — it runs in the
//! shell's `feed()` path.  When GDB mode is active, incoming USB bytes
//! are routed to [`GdbStub::feed()`] instead of the normal shell parser.
//! Output is pushed to the same Shell `push()` buffer and drained by
//! the USB interrupt as usual.
//!
//! When a DebugMonitor exception fires (DWT watchpoint hit), the handler
//! in `dwt.rs` records the hit.  The stub notices this on the next
//! `feed()` call and sends a stop-reply packet to GDB.

use crate::dwt;
use crate::fpb;
use crate::user_app_io;

// ── Packet buffer ──
// GDB RSP packets can be large (register dump = 17×8 = 136 hex chars + framing).
// Memory reads: m addr,len → up to 256 bytes = 512 hex chars.
// We keep it modest since we are on a microcontroller.
const PKT_BUF_SIZE: usize = 1024;
const RESP_BUF_SIZE: usize = 1024;

// ── Software breakpoint table ──
// For SRAM2 demos (0x2004_xxxx), FPB can't set breakpoints (Flash-only).
// Instead, we save the original Thumb instruction and write BKPT #0 (0xBE00).
const MAX_SW_BREAKPOINTS: usize = 16;
struct SwBreakpoint {
    addr: u32,
    orig: u16, // saved original Thumb instruction
    active: bool,
}
static mut SW_BKPTS: [SwBreakpoint; MAX_SW_BREAKPOINTS] = {
    const EMPTY: SwBreakpoint = SwBreakpoint { addr: 0, orig: 0, active: false };
    [EMPTY; MAX_SW_BREAKPOINTS]
};

/// GDB RSP protocol state.
#[derive(Clone, Copy, PartialEq)]
enum RxState {
    /// Waiting for `$` start byte.
    Idle,
    /// Accumulating packet data bytes.
    Data,
    /// Got `#`, waiting for first checksum hex digit.
    Checksum1,
    /// Got first checksum digit, waiting for second.
    Checksum2(u8),
}

/// A minimal GDB RSP stub.
pub struct GdbStub {
    /// Incoming packet assembly buffer.
    pkt: [u8; PKT_BUF_SIZE],
    /// Current write position in pkt.
    pkt_len: usize,
    /// Running checksum of data bytes.
    rx_sum: u8,
    /// parser state.
    state: RxState,
    /// True when the current packet has overflowed PKT_BUF_SIZE.
    /// Overflow packets are NAK'd even if the checksum is correct,
    /// because the data is truncated and dispatch would be wrong.
    pkt_overflow: bool,
    /// True while we are in GDB mode (shell routes bytes here).
    pub active: bool,
    /// Set true when a DebugMonitor hit is detected but not yet reported.
    pending_stop: bool,
    /// Last DebugMonitor count we saw — to detect new hits.
    last_debugmon: u32,
    /// True if the target is "running" (GDB sent `c`/`s` and we haven't
    /// responded with a stop-reply yet).  While running, we poll for
    /// DWT hits and Ctrl-C (0x03) interrupts.
    target_running: bool,
    /// Set when continuing past a software breakpoint: the single-step
    /// halt should re-patch the BKPT and auto-continue (not report to GDB).
    step_then_continue: bool,
    /// No-ack mode (QStartNoAckMode): suppress +/- ACK/NAK.
    no_ack: bool,
    /// Entry breakpoint address planted by enter_debug(); auto-removed
    /// the first time the target halts at it.  0 = none.
    entry_bkpt: u32,
}

impl GdbStub {
    pub const fn new() -> Self {
        Self {
            pkt: [0u8; PKT_BUF_SIZE],
            pkt_len: 0,
            rx_sum: 0,
            state: RxState::Idle,
            pkt_overflow: false,
            active: false,
            pending_stop: false,
            last_debugmon: 0,
            target_running: false,
            step_then_continue: false,
            no_ack: false,
            entry_bkpt: 0,
        }
    }

    /// Enter GDB mode.  Called from the `gdb` shell command.
    /// Returns a greeting message to push to the shell output.
    ///
    /// Pends DebugMonitor so the demo halts in Thread mode before GDB
    /// connects.  By the time GDB sends `?`, registers are valid and
    /// the target is genuinely stopped.
    pub fn enter(&mut self) -> &'static [u8] {
        self.active = true;
        self.state = RxState::Idle;
        self.pkt_len = 0;
        self.pkt_overflow = false;
        self.pending_stop = true; // report halt to first `?`
        self.last_debugmon = dwt::debugmon_count();
        self.target_running = false;
        self.step_then_continue = false;
        self.no_ack = false;
        self.entry_bkpt = 0;
        // Halt the demo: pend DebugMonitor so it fires on return to
        // Thread mode (where sandboxed demos run).
        dwt::set_gdb_active(true);
        dwt::pend_debugmon();
        b"GDB RSP active. Connect with:\r\n  arm-none-eabi-gdb -ex \"target remote /dev/ttyACM0\"\r\nType Ctrl-C three times rapidly OR type this string  \"$D#44\" to exit GDB mode.\r\n"
    }

    /// Enter GDB mode for debug sessions — used by `debug` shell command.
    ///
    /// Unlike `enter()`, does NOT pend DebugMonitor (the BKPT at the
    /// demo entry point will halt the target naturally).  Also plants
    /// a software breakpoint at the demo's _start address so the demo
    /// halts on its first instruction.
    ///
    /// Sets `target_running = true` so the first halt (from hitting the
    /// entry BKPT) is reported as a T05 stop-reply to GDB.  The entry
    /// BKPT is auto-removed on first hit so `continue` runs straight
    /// to the user's breakpoints.
    pub fn enter_debug(&mut self, entry_addr: u32) -> &'static [u8] {
        self.active = true;
        self.state = RxState::Idle;
        self.pkt_len = 0;
        self.pkt_overflow = false;
        self.pending_stop = false; // wait for real BKPT halt
        self.last_debugmon = dwt::debugmon_count();
        self.target_running = true; // demo will start and hit BKPT
        self.step_then_continue = false;
        self.no_ack = false;
        self.entry_bkpt = entry_addr;
        dwt::set_gdb_active(true);
        // Plant BKPT at demo entry so it halts on first instruction.
        // The demo hasn't started yet (idle picks it up after we return).
        unsafe { set_sw_breakpoint(entry_addr); }
        b"GDB debug active - demo will halt at entry.\r\nType Ctrl-C three times rapidly OR type this string  \"$D#44\" to exit GDB mode.\r\n"
    }

    /// Silent detach — called when DTR drops (host closed port) while
    /// GDB RSP is active.  Cleans up all debug state and resumes the
    /// target without writing any response (nobody is listening).
    pub fn auto_detach(&mut self) {
        unsafe {
            dwt::clear_all();
            crate::fpb::clear_all();
            clear_all_sw_breakpoints();
            dwt::cleanup_for_detach();
        }
        if dwt::is_halted() {
            dwt::resume_target();
        }
        self.active = false;
        self.target_running = false;
        self.entry_bkpt = 0;
    }

    /// Flush buffered demo output as O-packets into the response writer.
    ///
    /// Called from halt detection (just before T05) while GDB is still
    /// in "waiting for stop-reply" mode, where O-packets are valid.
    /// Output accumulated in `user_app_io::BUF` while the demo ran.
    ///
    /// Writes as many O-packets as fit in `rw`, leaving room for T05.
    fn flush_o_packets(&self, rw: &mut RespWriter) {
        loop {
            let data = user_app_io::pending();
            if data.is_empty() {
                break;
            }
            // Reserve 10 bytes for the T05 packet ($T05#b9 = 8 + margin)
            let avail = rw.buf.len().saturating_sub(rw.pos + 10);
            // Each O-packet: $O<hex>#xx = 5 framing + 2 per input byte
            if avail < 7 {
                break; // not enough room for even 1 byte
            }
            let max_input = (avail - 5) / 2;
            let n = core::cmp::min(data.len(), max_input);
            if n == 0 {
                break;
            }

            // Build $O<hex>#<checksum> directly into rw
            rw.push(b'$');
            let mut sum: u8 = b'O';
            rw.push(b'O');
            for &b in &data[..n] {
                let hi = HEX_CHARS[(b >> 4) as usize];
                let lo = HEX_CHARS[(b & 0xF) as usize];
                sum = sum.wrapping_add(hi).wrapping_add(lo);
                rw.push(hi);
                rw.push(lo);
            }
            rw.push(b'#');
            rw.push(HEX_CHARS[(sum >> 4) as usize]);
            rw.push(HEX_CHARS[(sum & 0xF) as usize]);

            user_app_io::advance(n);
        }
    }

    /// Feed incoming USB bytes while in GDB mode.
    /// Writes response bytes into `resp` and returns how many bytes were written.
    /// If the stub decides to exit (detach/kill), sets `self.active = false`.
    pub fn feed(&mut self, data: &[u8], resp: &mut [u8; RESP_BUF_SIZE]) -> usize {
        let mut rw = RespWriter::new(resp);

        // Check if target has halted inside DebugMonitor handler.
        //
        // Race guard: after `c`/`s` calls resume_target(), the DebugMonitor
        // handler (priority 0xF0) hasn't cleared TARGET_HALTED yet because
        // we're in the USB ISR (priority 0xE0, higher).  The next poll_gdb()
        // would see is_halted()=true with stale registers and send a bogus
        // T05.  We prevent this by checking debugmon_count() — if it hasn't
        // incremented since the last halt we processed, this is a stale
        // observation of the same halt, not a new one.
        let new_halt = dwt::debugmon_count() != self.last_debugmon;
        if dwt::is_halted() && self.target_running && new_halt {
            if self.step_then_continue {
                // We single-stepped past a removed BKPT — re-patch and resume
                self.step_then_continue = false;
                self.last_debugmon = dwt::debugmon_count();
                unsafe { repatch_all_sw_breakpoints(); }
                dwt::clear_single_step();
                dwt::resume_target();
                // target_running stays true — we don't report this halt to GDB
            } else {
                // Check if this is the initial entry halt (planted by `debug`)
                let is_entry_halt = if self.entry_bkpt != 0 {
                    let pc = dwt::saved_reg(15);
                    if pc == self.entry_bkpt {
                        unsafe { clear_sw_breakpoint(self.entry_bkpt); }
                        self.entry_bkpt = 0;
                        true
                    } else {
                        false
                    }
                } else {
                    false
                };

                self.target_running = false;
                self.last_debugmon = dwt::debugmon_count();

                if is_entry_halt {
                    // Entry halt — GDB hasn't connected yet, so T05 now
                    // would buffer in serial/socat and create a stale
                    // stop-reply.  Defer to `?` handler via pending_stop.
                    self.pending_stop = true;
                } else if data.is_empty() {
                    // Post-continue halt (polled, no incoming command).
                    // GDB is waiting for a stop-reply after `c`/`s`.
                    self.flush_o_packets(&mut rw);
                    write_packet(&mut rw, b"T05");
                } else {
                    // Post-continue halt with incoming data — defer.
                    self.pending_stop = true;
                }
            }
        }

        for &b in data {
            // Ctrl-C (0x03) while target is "running" → interrupt
            if b == 0x03 && self.target_running {
                self.target_running = false;
                write_packet(&mut rw, b"T02"); // SIGINT
                continue;
            }

            match self.state {
                RxState::Idle => {
                    if b == b'$' {
                        self.pkt_len = 0;
                        self.rx_sum = 0;
                        self.pkt_overflow = false;
                        self.state = RxState::Data;
                    }
                    // GDB may send '+' or '-' ACK/NAK — ignore them
                }
                RxState::Data => {
                    if b == b'#' {
                        self.state = RxState::Checksum1;
                    } else if b == b'$' {
                        // Re-sync: new packet start
                        self.pkt_len = 0;
                        self.rx_sum = 0;
                        self.pkt_overflow = false;
                    } else {
                        self.rx_sum = self.rx_sum.wrapping_add(b);
                        if self.pkt_len < PKT_BUF_SIZE {
                            self.pkt[self.pkt_len] = b;
                            self.pkt_len += 1;
                        } else {
                            self.pkt_overflow = true;
                        }
                    }
                }
                RxState::Checksum1 => {
                    if let Some(d) = hex_digit(b) {
                        self.state = RxState::Checksum2(d);
                    } else {
                        self.state = RxState::Idle; // malformed
                    }
                }
                RxState::Checksum2(hi) => {
                    self.state = RxState::Idle;
                    if let Some(lo) = hex_digit(b) {
                        let expected = (hi << 4) | lo;
                        if expected == self.rx_sum && !self.pkt_overflow {
                            // Valid packet — ACK and dispatch
                            if !self.no_ack { rw.push(b'+'); }
                            self.dispatch(&mut rw);
                        } else {
                            // Checksum mismatch or overflow — NAK
                            if !self.no_ack { rw.push(b'-'); }
                        }
                    } else {
                        if !self.no_ack { rw.push(b'-'); } // malformed checksum
                    }
                }
            }
        }

        rw.pos
    }

    /// Dispatch a complete, valid RSP packet.
    fn dispatch(&mut self, rw: &mut RespWriter) {
        if self.pkt_len == 0 {
            write_packet(rw, b"");
            return;
        }

        let cmd = self.pkt[0];
        let args = &self.pkt[1..self.pkt_len];

        match cmd {
            // ── Stop reason ──
            b'?' => {
                if self.pending_stop {
                    self.pending_stop = false;
                    self.flush_o_packets(rw);
                    write_packet(rw, b"T05"); // SIGTRAP (breakpoint/watchpoint hit)
                } else if dwt::is_halted() {
                    // Target genuinely halted (BKPT, watchpoint, single-step).
                    // Clean up entry breakpoint if we halted there.
                    if self.entry_bkpt != 0 {
                        let pc = dwt::saved_reg(15);
                        if pc == self.entry_bkpt {
                            unsafe { clear_sw_breakpoint(self.entry_bkpt); }
                            self.entry_bkpt = 0;
                        }
                    }
                    if self.target_running {
                        self.target_running = false;
                    }
                    self.last_debugmon = dwt::debugmon_count();
                    write_packet(rw, b"T05");
                } else if self.target_running {
                    // Target not halted but running — report T05 anyway.
                    // This happens in debug mode before the demo hits the
                    // entry BKPT.  GDB needs an initial stop to proceed.
                    // Don't pend DebugMonitor — it would halt idle, not the demo.
                    write_packet(rw, b"T05");
                } else {
                    write_packet(rw, b"S05"); // SIGTRAP — initial stop
                }
            }

            // ── Read all registers ──
            // With target XML (arm-core.xml), GDB expects 16 core regs + cpsr = 17 regs.
            //   R0-R15  (16 × 8 hex = 128)
            //   cpsr    (1 × 8 hex  = 8)
            // Total = 136 hex chars
            b'g' => {
                let mut buf = [b'0'; 136]; // pre-fill with '0' (zeros)
                if dwt::regs_valid() {
                    // Use saved registers from DebugMonitor exception
                    for i in 0..16usize {
                        hex_encode_u32_le(dwt::saved_reg(i), &mut buf[i * 8..]);
                    }
                    // cpsr = xPSR (index 16)
                    hex_encode_u32_le(dwt::saved_reg(16), &mut buf[128..]);
                } else {
                    // No DebugMon context — read what we can live
                    // R0-R12: zeros (can't access caller's regs)
                    // SP (R13) — current MSP
                    let sp: u32;
                    unsafe { core::arch::asm!("mrs {}, MSP", out(reg) sp) };
                    hex_encode_u32_le(sp, &mut buf[13 * 8..]);
                    // LR (R14)
                    let lr: u32;
                    unsafe { core::arch::asm!("mov {}, lr", out(reg) lr) };
                    hex_encode_u32_le(lr, &mut buf[14 * 8..]);
                    // PC (R15) — approximate with LR
                    hex_encode_u32_le(lr, &mut buf[15 * 8..]);
                    // cpsr
                    let xpsr: u32;
                    unsafe { core::arch::asm!("mrs {}, xPSR", out(reg) xpsr) };
                    hex_encode_u32_le(xpsr, &mut buf[128..]);
                }
                write_packet(rw, &buf);
            }

            // ── Read single register: p n ──
            b'p' => {
                if let Some(n) = parse_hex_str(args) {
                    let mut buf = [b'0'; 8];
                    if n <= 15 {
                        if dwt::regs_valid() {
                            hex_encode_u32_le(dwt::saved_reg(n as usize), &mut buf);
                        } else if n == 13 {
                            let sp: u32;
                            unsafe { core::arch::asm!("mrs {}, MSP", out(reg) sp) };
                            hex_encode_u32_le(sp, &mut buf);
                        } else if n == 14 || n == 15 {
                            let lr: u32;
                            unsafe { core::arch::asm!("mov {}, lr", out(reg) lr) };
                            hex_encode_u32_le(lr, &mut buf);
                        }
                        write_packet(rw, &buf);
                    } else if n == 16 || n == 25 {
                        // cpsr/xpsr: register 16 sequential (LLDB) or 25 (GDB legacy)
                        if dwt::regs_valid() {
                            hex_encode_u32_le(dwt::saved_reg(16), &mut buf);
                        } else {
                            let xpsr: u32;
                            unsafe { core::arch::asm!("mrs {}, xPSR", out(reg) xpsr) };
                            hex_encode_u32_le(xpsr, &mut buf);
                        }
                        write_packet(rw, &buf);
                    } else {
                        // Unknown register
                        write_packet(rw, b"E45");
                    }
                } else {
                    write_packet(rw, b"E01");
                }
            }

            // ── Write all registers (stub: accept but ignore) ──
            b'G' => {
                write_packet(rw, b"OK");
            }

            // ── Read memory: m addr,length ──
            b'm' => {
                if let Some((addr, len)) = parse_m_args(args) {
                    // Cap at what fits in BOTH the hex buffer and the response packet
                    let mut buf = [0u8; 520]; // 520 hex chars = 260 bytes of data
                    let max_bytes = buf.len() / 2; // 2 hex chars per byte
                    let len = core::cmp::min(len as usize, max_bytes) as u32;
                    let mut pos = 0;
                    for i in 0..len {
                        let byte = unsafe {
                            core::ptr::read_volatile((addr + i) as *const u8)
                        };
                        buf[pos] = HEX_CHARS[(byte >> 4) as usize];
                        buf[pos + 1] = HEX_CHARS[(byte & 0xF) as usize];
                        pos += 2;
                    }
                    write_packet(rw, &buf[..pos]);
                } else {
                    write_packet(rw, b"E01"); // parse error
                }
            }

            // ── Write memory: M addr,length:hex ──
            b'M' => {
                if let Some((addr, _len, hex_data)) = parse_M_args(args) {
                    let mut i = 0;
                    let mut offset = 0u32;
                    while i + 1 < hex_data.len() {
                        if let (Some(hi), Some(lo)) = (hex_digit(hex_data[i]), hex_digit(hex_data[i+1])) {
                            let byte = (hi << 4) | lo;
                            unsafe {
                                core::ptr::write_volatile((addr + offset) as *mut u8, byte);
                            }
                            offset += 1;
                        }
                        i += 2;
                    }
                    write_packet(rw, b"OK");
                } else {
                    write_packet(rw, b"E01");
                }
            }

            // ── Set breakpoint/watchpoint: Z type,addr,kind ──
            b'Z' => {
                if let Some((wp_type, addr, kind)) = parse_z_args(args) {
                    match wp_type {
                        // Z0 = software breakpoint (BKPT instruction patching)
                        0 => {
                            let ok = unsafe { set_sw_breakpoint(addr) };
                            if ok {
                                write_packet(rw, b"OK");
                            } else {
                                write_packet(rw, b"E29");
                            }
                        }
                        // Z1 = hardware breakpoint (FPB — Flash only)
                        1 => {
                            let ok = unsafe { fpb::set_breakpoint(addr) };
                            if ok.is_some() {
                                write_packet(rw, b"OK");
                            } else {
                                // FPB can't reach SRAM2; suggest swbreak to GDB
                                write_packet(rw, b"E29");
                            }
                        }
                        // Z2 = write watchpoint, Z3 = read, Z4 = access
                        2 | 3 | 4 => {
                            let func = match wp_type {
                                2 => dwt::WatchFunc::DataWrite,
                                3 => dwt::WatchFunc::DataRead,
                                _ => dwt::WatchFunc::DataRW,
                            };
                            let mut set = false;
                            for n in 0..4u32 {
                                let (_, _, f, _) = dwt::read_watchpoint(n);
                                if f == 0 {
                                    let mask = dwt_mask_for_size(kind);
                                    unsafe { dwt::set_watchpoint(n, addr, mask, func); }
                                    set = true;
                                    break;
                                }
                            }
                            if set {
                                write_packet(rw, b"OK");
                            } else {
                                write_packet(rw, b"E29");
                            }
                        }
                        _ => write_packet(rw, b""),
                    }
                } else {
                    write_packet(rw, b"E01");
                }
            }

            // ── Remove breakpoint/watchpoint: z type,addr,kind ──
            b'z' => {
                if let Some((wp_type, addr, _kind)) = parse_z_args(args) {
                    match wp_type {
                        // z0 = remove software breakpoint
                        0 => {
                            unsafe { clear_sw_breakpoint(addr); }
                            write_packet(rw, b"OK");
                        }
                        // z1 = remove hardware breakpoint
                        1 => {
                            unsafe { fpb::clear_breakpoint(addr); }
                            write_packet(rw, b"OK");
                        }
                        2 | 3 | 4 => {
                            for n in 0..4u32 {
                                let (comp_addr, _, _, _) = dwt::read_watchpoint(n);
                                if comp_addr == addr {
                                    unsafe { dwt::clear_watchpoint(n); }
                                    break;
                                }
                            }
                            write_packet(rw, b"OK");
                        }
                        _ => write_packet(rw, b""),
                    }
                } else {
                    write_packet(rw, b"E01");
                }
            }

            // ── Continue ──
            b'c' => {
                dwt::clear_single_step();
                // If stopped at a software breakpoint, temporarily remove it
                // and single-step past, then re-patch.
                let at_bkpt = unsafe { unpatch_bkpt_at_pc() };
                if at_bkpt {
                    // Step one instruction past the restored original,
                    // then re-patch and continue.  The step will trigger
                    // DebugMonitor → halt.  We handle re-patch + resume
                    // by setting a flag so the next halt auto-continues.
                    self.step_then_continue = true;
                    dwt::request_single_step();
                    self.target_running = true;
                    if dwt::is_halted() {
                        dwt::resume_target();
                    }
                } else {
                    self.target_running = true;
                    if dwt::is_halted() {
                        dwt::resume_target();
                    }
                }
                // Don't send a response now — response comes when
                // the target stops (watchpoint hit or Ctrl-C).
            }

            // ── Single step ──
            b's' => {
                // Request single-step: DebugMon handler will set MON_STEP in DEMCR.
                dwt::request_single_step();
                self.target_running = true;
                if dwt::is_halted() {
                    dwt::resume_target();
                }
            }

            // ── Detach ──
            b'D' => {
                write_packet(rw, b"OK");
                unsafe {
                    dwt::clear_all();
                    fpb::clear_all();
                    clear_all_sw_breakpoints();
                    dwt::cleanup_for_detach();
                }
                // Resume target if halted so the demo doesn't hang forever
                if dwt::is_halted() {
                    dwt::resume_target();
                }
                self.active = false;
                self.target_running = false;
                self.entry_bkpt = 0;
            }

            // ── Kill ──
            b'k' => {
                write_packet(rw, b"OK");
                unsafe {
                    dwt::clear_all();
                    fpb::clear_all();
                    clear_all_sw_breakpoints();
                    dwt::cleanup_for_detach();
                }
                if dwt::is_halted() {
                    dwt::resume_target();
                }
                self.active = false;
                self.target_running = false;
                self.entry_bkpt = 0;
            }

            // ── Query packets ──
            b'q' => {
                self.handle_q(args, rw);
            }

            // ── Set packets (Q) ──
            b'Q' => {
                if starts_with(args, b"StartNoAckMode") {
                    self.no_ack = true;
                    write_packet(rw, b"OK");
                } else {
                    write_packet(rw, b"");
                }
            }

            // ── Thread ops (stub: always OK) ──
            b'H' => {
                write_packet(rw, b"OK");
            }

            // ── Thread alive ──
            b'T' => {
                write_packet(rw, b"OK"); // thread 1 always alive
            }

            // ── vCont and other v-packets ──
            b'v' => {
                if starts_with(args, b"Cont?") {
                    // Tell client we support vCont with continue, step, stop
                    write_packet(rw, b"vCont;c;s;t");
                } else if starts_with(args, b"Cont;c") {
                    // vCont;c  — continue (same as bare 'c')
                    dwt::clear_single_step();
                    let at_bkpt = unsafe { unpatch_bkpt_at_pc() };
                    if at_bkpt {
                        self.step_then_continue = true;
                        dwt::request_single_step();
                        self.target_running = true;
                        if dwt::is_halted() { dwt::resume_target(); }
                    } else {
                        self.target_running = true;
                        if dwt::is_halted() { dwt::resume_target(); }
                    }
                } else if starts_with(args, b"Cont;s") {
                    // vCont;s  — single step (same as bare 's')
                    dwt::request_single_step();
                    self.target_running = true;
                    if dwt::is_halted() { dwt::resume_target(); }
                } else if starts_with(args, b"Cont;t") {
                    // vCont;t  — stop/interrupt
                    if self.target_running {
                        self.target_running = false;
                        write_packet(rw, b"T02"); // SIGINT
                    }
                } else if starts_with(args, b"MustReplyEmpty") {
                    write_packet(rw, b"");
                } else {
                    write_packet(rw, b"");
                }
            }

            // ── Unknown command — empty reply means "unsupported" ──
            _ => {
                write_packet(rw, b"");
            }
        }
    }

    /// Handle q (query) packets.
    fn handle_q(&self, args: &[u8], rw: &mut RespWriter) {
        if starts_with(args, b"Supported") {
            // Tell GDB what we support
            // swbreak+ = we support Z0 software breakpoints
            // hwbreak+ = we support Z1 hardware breakpoints
            // qXfer:features:read+ = we provide target description XML
            write_packet(rw, b"PacketSize=1024;swbreak+;hwbreak+;qXfer:features:read+;QStartNoAckMode+;vContSupported+");
        } else if starts_with(args, b"Xfer:features:read:target.xml:") {
            let after = &args[b"Xfer:features:read:target.xml:".len()..];
            self.serve_xfer(after, TARGET_XML, rw);
        } else if starts_with(args, b"Attached") {
            // 1 = attached to existing process (don't kill on detach)
            write_packet(rw, b"1");
        } else if starts_with(args, b"TStatus") {
            write_packet(rw, b"");
        } else if starts_with(args, b"fThreadInfo") {
            write_packet(rw, b"m1");
        } else if starts_with(args, b"sThreadInfo") {
            write_packet(rw, b"l");
        } else if starts_with(args, b"C") {
            write_packet(rw, b"QC1");
        } else {
            write_packet(rw, b"");
        }
    }

    /// Serve a qXfer chunk: parse "offset,length" and return 'l' (last) or 'm' (more).
    fn serve_xfer(&self, offset_len: &[u8], data: &[u8], rw: &mut RespWriter) {
        if let Some((offset, length)) = parse_m_args(offset_len) {
            let off = offset as usize;
            let len = length as usize;
            if off >= data.len() {
                write_packet(rw, b"l"); // past end
            } else {
                let end = core::cmp::min(off + len, data.len());
                let chunk = &data[off..end];
                let is_last = end >= data.len();
                // Build response: 'l' or 'm' prefix + data
                let mut buf = [0u8; 1000];
                buf[0] = if is_last { b'l' } else { b'm' };
                let clen = core::cmp::min(chunk.len(), buf.len() - 1);
                buf[1..1 + clen].copy_from_slice(&chunk[..clen]);
                write_packet(rw, &buf[..1 + clen]);
            }
        } else {
            write_packet(rw, b"E01");
        }
    }
}

// ── Response writer ──

struct RespWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> RespWriter<'a> {
    fn new(buf: &'a mut [u8; RESP_BUF_SIZE]) -> Self {
        Self { buf, pos: 0 }
    }

    fn push(&mut self, b: u8) {
        if self.pos < self.buf.len() {
            self.buf[self.pos] = b;
            self.pos += 1;
        }
    }

    fn push_slice(&mut self, data: &[u8]) {
        for &b in data {
            self.push(b);
        }
    }

    fn written(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}

// ── Packet framing ──

const HEX_CHARS: &[u8; 16] = b"0123456789abcdef";

/// Write a complete RSP packet: $data#checksum
fn write_packet(rw: &mut RespWriter, data: &[u8]) {
    rw.push(b'$');
    let mut sum: u8 = 0;
    for &b in data {
        sum = sum.wrapping_add(b);
        rw.push(b);
    }
    rw.push(b'#');
    rw.push(HEX_CHARS[(sum >> 4) as usize]);
    rw.push(HEX_CHARS[(sum & 0xF) as usize]);
}

// ── Hex encoding helpers ──

/// Encode a u32 as 8 hex chars in little-endian byte order (GDB ARM convention).
fn hex_encode_u32_le(val: u32, buf: &mut [u8]) {
    let bytes = val.to_le_bytes();
    for (i, &b) in bytes.iter().enumerate() {
        buf[i * 2] = HEX_CHARS[(b >> 4) as usize];
        buf[i * 2 + 1] = HEX_CHARS[(b & 0xF) as usize];
    }
}

/// Decode a single hex digit.
fn hex_digit(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Parse hex string to u32.
fn parse_hex_str(s: &[u8]) -> Option<u32> {
    if s.is_empty() || s.len() > 8 { return None; }
    let mut result: u32 = 0;
    for &b in s {
        let d = hex_digit(b)?;
        result = result.checked_mul(16)?.checked_add(d as u32)?;
    }
    Some(result)
}

/// Parse `m addr,length` arguments.
fn parse_m_args(args: &[u8]) -> Option<(u32, u32)> {
    let comma = args.iter().position(|&b| b == b',')?;
    let addr = parse_hex_str(&args[..comma])?;
    let len = parse_hex_str(&args[comma + 1..])?;
    Some((addr, len))
}

/// Parse `M addr,length:hexdata` arguments.
#[allow(non_snake_case)]
fn parse_M_args(args: &[u8]) -> Option<(u32, u32, &[u8])> {
    let comma = args.iter().position(|&b| b == b',')?;
    let colon = args.iter().position(|&b| b == b':')?;
    if colon <= comma { return None; }
    let addr = parse_hex_str(&args[..comma])?;
    let len = parse_hex_str(&args[comma + 1..colon])?;
    Some((addr, len, &args[colon + 1..]))
}

/// Parse `Z type,addr,kind` or `z type,addr,kind` arguments.
fn parse_z_args(args: &[u8]) -> Option<(u8, u32, u32)> {
    // args = "type,addr,kind"  (first byte after Z/z already consumed)
    if args.is_empty() { return None; }
    let tp = hex_digit(args[0])?;
    if args.len() < 3 || args[1] != b',' { return None; }
    let rest = &args[2..];
    let comma = rest.iter().position(|&b| b == b',')?;
    let addr = parse_hex_str(&rest[..comma])?;
    let kind = parse_hex_str(&rest[comma + 1..])?;
    Some((tp, addr, kind))
}

/// Check if `data` starts with `prefix`.
fn starts_with(data: &[u8], prefix: &[u8]) -> bool {
    data.len() >= prefix.len() && &data[..prefix.len()] == prefix
}

/// Compute DWT mask value from watchpoint size.
/// Mask of N means ignore the low N bits of the address.
/// Size must be power of 2 (1, 2, 4, 8, ...).
fn dwt_mask_for_size(size: u32) -> u32 {
    if size <= 1 { return 0; }
    let mut mask = 0u32;
    let mut s = size;
    while s > 1 {
        mask += 1;
        s >>= 1;
    }
    mask
}

// ── Software breakpoint helpers ──

/// BKPT #0 encoding for Thumb (T1 encoding): 0xBE00
const THUMB_BKPT: u16 = 0xBE00;

/// Set a software breakpoint by patching memory with BKPT #0.
/// Returns true on success.
unsafe fn set_sw_breakpoint(addr: u32) -> bool {
    // Check if already set
    for bp in SW_BKPTS.iter() {
        if bp.active && bp.addr == addr {
            return true; // already set
        }
    }
    // Find a free slot
    for bp in SW_BKPTS.iter_mut() {
        if !bp.active {
            // Save original instruction (Thumb = 16-bit)
            bp.orig = core::ptr::read_volatile(addr as *const u16);
            bp.addr = addr;
            bp.active = true;
            // Write BKPT #0
            core::ptr::write_volatile(addr as *mut u16, THUMB_BKPT);
            // ISB to flush instruction pipeline after patching code
            core::arch::asm!("isb");
            return true;
        }
    }
    false // no free slot
}

/// Remove a software breakpoint, restoring the original instruction.
unsafe fn clear_sw_breakpoint(addr: u32) {
    for bp in SW_BKPTS.iter_mut() {
        if bp.active && bp.addr == addr {
            // Restore original instruction
            core::ptr::write_volatile(addr as *mut u16, bp.orig);
            core::arch::asm!("isb");
            bp.active = false;
            return;
        }
    }
}

/// Remove all software breakpoints.
unsafe fn clear_all_sw_breakpoints() {
    for bp in SW_BKPTS.iter_mut() {
        if bp.active {
            core::ptr::write_volatile(bp.addr as *mut u16, bp.orig);
            bp.active = false;
        }
    }
    core::arch::asm!("isb");
}

/// Check if the target is stopped at a software breakpoint address.
/// If so, temporarily restore the original instruction so the CPU can
/// execute it before re-patching.  Returns true if a BKPT was unpatched
/// (caller should single-step then re-patch).
unsafe fn unpatch_bkpt_at_pc() -> bool {
    let pc = dwt::saved_reg(15); // R15 = PC
    for bp in SW_BKPTS.iter_mut() {
        if bp.active && bp.addr == pc {
            // Restore original instruction temporarily
            core::ptr::write_volatile(bp.addr as *mut u16, bp.orig);
            core::arch::asm!("isb");
            return true;
        }
    }
    false
}

/// Re-patch all active software breakpoints (after stepping past one).
unsafe fn repatch_all_sw_breakpoints() {
    for bp in SW_BKPTS.iter() {
        if bp.active {
            core::ptr::write_volatile(bp.addr as *mut u16, THUMB_BKPT);
        }
    }
    core::arch::asm!("isb");
}

// ── Target description XML ──
// This tells GDB/LLDB we are an ARM Cortex-M target with 16 GP regs + cpsr.
// Without this, GDB defaults to "i386" or legacy "arm" with FPA regs.

// Single inline target description — no xi:include, everything in one fetch.
// Registers are numbered sequentially 0–16 (no gaps).  Both GDB and LLDB
// are happy with this layout; the 'p' handler accepts both p10 (reg 16)
// and p19 (reg 25 legacy) for cpsr.
const TARGET_XML: &[u8] = b"<?xml version=\"1.0\"?>\
<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\
<target version=\"1.0\">\
<architecture>arm</architecture>\
<feature name=\"org.gnu.gdb.arm.m-profile\">\
<reg name=\"r0\" bitsize=\"32\"/>\
<reg name=\"r1\" bitsize=\"32\"/>\
<reg name=\"r2\" bitsize=\"32\"/>\
<reg name=\"r3\" bitsize=\"32\"/>\
<reg name=\"r4\" bitsize=\"32\"/>\
<reg name=\"r5\" bitsize=\"32\"/>\
<reg name=\"r6\" bitsize=\"32\"/>\
<reg name=\"r7\" bitsize=\"32\"/>\
<reg name=\"r8\" bitsize=\"32\"/>\
<reg name=\"r9\" bitsize=\"32\"/>\
<reg name=\"r10\" bitsize=\"32\"/>\
<reg name=\"r11\" bitsize=\"32\"/>\
<reg name=\"r12\" bitsize=\"32\"/>\
<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>\
<reg name=\"lr\" bitsize=\"32\"/>\
<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>\
<reg name=\"xpsr\" bitsize=\"32\"/>\
</feature>\
</target>";
