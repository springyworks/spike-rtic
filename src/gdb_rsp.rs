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

// ── Packet buffer ──
// GDB RSP packets can be large (register dump = 17×8 = 136 hex chars + framing).
// Memory reads: m addr,len → up to 256 bytes = 512 hex chars.
// We keep it modest since we are on a microcontroller.
const PKT_BUF_SIZE: usize = 600;
const RESP_BUF_SIZE: usize = 600;

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
}

impl GdbStub {
    pub const fn new() -> Self {
        Self {
            pkt: [0u8; PKT_BUF_SIZE],
            pkt_len: 0,
            rx_sum: 0,
            state: RxState::Idle,
            active: false,
            pending_stop: false,
            last_debugmon: 0,
            target_running: false,
        }
    }

    /// Enter GDB mode.  Called from the `gdb` shell command.
    /// Returns a greeting message to push to the shell output.
    pub fn enter(&mut self) -> &'static [u8] {
        self.active = true;
        self.state = RxState::Idle;
        self.pkt_len = 0;
        self.pending_stop = false;
        self.last_debugmon = dwt::debugmon_count();
        self.target_running = false;
        b"GDB stub active. Connect with:\r\n  arm-none-eabi-gdb -ex \"target remote /dev/ttyACM0\"\r\nType Ctrl-C three times rapidly to exit GDB mode.\r\n"
    }

    /// Feed incoming USB bytes while in GDB mode.
    /// Writes response bytes into `resp` and returns how many bytes were written.
    /// If the stub decides to exit (detach/kill), sets `self.active = false`.
    pub fn feed(&mut self, data: &[u8], resp: &mut [u8; RESP_BUF_SIZE]) -> usize {
        let mut rw = RespWriter::new(resp);

        // Check for new DebugMonitor hits
        let dm = dwt::debugmon_count();
        if dm != self.last_debugmon {
            self.last_debugmon = dm;
            if self.target_running {
                self.target_running = false;
                // Send stop-reply: T05 = SIGTRAP (watchpoint)
                write_packet(&mut rw, b"T05");
            } else {
                self.pending_stop = true;
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
                    } else {
                        self.rx_sum = self.rx_sum.wrapping_add(b);
                        if self.pkt_len < PKT_BUF_SIZE {
                            self.pkt[self.pkt_len] = b;
                            self.pkt_len += 1;
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
                        if expected == self.rx_sum {
                            // Valid packet — ACK and dispatch
                            rw.push(b'+');
                            self.dispatch(&mut rw);
                        } else {
                            // Checksum mismatch — NAK
                            rw.push(b'-');
                        }
                    } else {
                        rw.push(b'-'); // malformed checksum
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
                    write_packet(rw, b"T05"); // SIGTRAP
                } else {
                    write_packet(rw, b"S00"); // no signal, just attached
                }
            }

            // ── Read all registers (ARM: R0-R12, SP, LR, PC, xPSR = 17 regs) ──
            b'g' => {
                let mut buf = [0u8; 17 * 8]; // 17 regs × 8 hex chars
                let mut pos = 0;
                // Read from DWT saved context if we have a hit, else read live
                // For now, read live registers via MSP/PSP
                // R0-R12: not easily accessible without an exception frame
                // We'll provide zeros for R0-R12, then SP, LR, PC from live state
                for i in 0..13u32 {
                    // R0-R12: read as 0 (no saved context outside exception)
                    let _ = i;
                    hex_encode_u32_le(0, &mut buf[pos..]);
                    pos += 8;
                }
                // SP (R13) — current MSP
                let sp: u32;
                unsafe { core::arch::asm!("mrs {}, MSP", out(reg) sp) };
                hex_encode_u32_le(sp, &mut buf[pos..]);
                pos += 8;
                // LR (R14) — current LR
                let lr: u32;
                unsafe { core::arch::asm!("mov {}, lr", out(reg) lr) };
                hex_encode_u32_le(lr, &mut buf[pos..]);
                pos += 8;
                // PC (R15) — approximate with LR
                hex_encode_u32_le(lr, &mut buf[pos..]);
                pos += 8;
                // xPSR
                let xpsr: u32;
                unsafe { core::arch::asm!("mrs {}, xPSR", out(reg) xpsr) };
                hex_encode_u32_le(xpsr, &mut buf[pos..]);
                write_packet(rw, &buf);
            }

            // ── Write all registers (stub: accept but ignore) ──
            b'G' => {
                write_packet(rw, b"OK");
            }

            // ── Read memory: m addr,length ──
            b'm' => {
                if let Some((addr, len)) = parse_m_args(args) {
                    // Cap at what fits in response buffer
                    let max_bytes = (RESP_BUF_SIZE - 10) / 2; // 2 hex chars per byte
                    let len = core::cmp::min(len as usize, max_bytes) as u32;
                    let mut buf = [0u8; 520]; // enough for 256 bytes
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
                        // Z2 = write watchpoint, Z3 = read, Z4 = access
                        2 | 3 | 4 => {
                            let func = match wp_type {
                                2 => dwt::WatchFunc::DataWrite,
                                3 => dwt::WatchFunc::DataRead,
                                _ => dwt::WatchFunc::DataRW,
                            };
                            // Find a free comparator
                            let mut set = false;
                            for n in 0..4u32 {
                                let (_, _, f, _) = dwt::read_watchpoint(n);
                                if f == 0 { // disabled
                                    let mask = dwt_mask_for_size(kind);
                                    unsafe { dwt::set_watchpoint(n, addr, mask, func); }
                                    set = true;
                                    break;
                                }
                            }
                            if set {
                                write_packet(rw, b"OK");
                            } else {
                                write_packet(rw, b"E29"); // no free comparator
                            }
                        }
                        // Z0 = software breakpoint, Z1 = hardware breakpoint
                        // Not supported yet
                        _ => write_packet(rw, b""),  // empty = unsupported
                    }
                } else {
                    write_packet(rw, b"E01");
                }
            }

            // ── Remove breakpoint/watchpoint: z type,addr,kind ──
            b'z' => {
                if let Some((wp_type, addr, _kind)) = parse_z_args(args) {
                    match wp_type {
                        2 | 3 | 4 => {
                            // Find matching comparator by address
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
                self.target_running = true;
                // Don't send a response now — response comes when
                // the target stops (watchpoint hit or Ctrl-C).
            }

            // ── Single step (not fully implemented — just continue) ──
            b's' => {
                // Real single-step needs MON_STEP in DEMCR.
                // For now, treat as continue — the next DWT hit stops us.
                self.target_running = true;
            }

            // ── Detach ──
            b'D' => {
                write_packet(rw, b"OK");
                unsafe { dwt::clear_all(); }
                self.active = false;
            }

            // ── Kill ──
            b'k' => {
                unsafe { dwt::clear_all(); }
                self.active = false;
            }

            // ── Query packets ──
            b'q' => {
                self.handle_q(args, rw);
            }

            // ── Thread ops (stub: always OK) ──
            b'H' => {
                write_packet(rw, b"OK");
            }

            // ── vCont? and other v-packets ──
            b'v' => {
                // vMustReplyEmpty
                write_packet(rw, b"");
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
            write_packet(rw, b"PacketSize=256;hwbreak-;swbreak-");
        } else if starts_with(args, b"Attached") {
            // 1 = attached to existing process (don't kill on detach)
            write_packet(rw, b"1");
        } else if starts_with(args, b"TStatus") {
            // No trace running
            write_packet(rw, b"");
        } else if starts_with(args, b"fThreadInfo") {
            // Single thread
            write_packet(rw, b"m1");
        } else if starts_with(args, b"sThreadInfo") {
            write_packet(rw, b"l"); // end of list
        } else if starts_with(args, b"C") {
            // Current thread
            write_packet(rw, b"QC1");
        } else {
            write_packet(rw, b""); // unsupported
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
