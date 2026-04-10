#![allow(dead_code)]
//! RTTY (Radio Teletype) Baudot/ITA2 encoder for the SPIKE Prime speaker.
//!
//! Generates classic amateur-radio FSK tones via DAC triangle wave.
//! Protocol: ITA2 (International Telegraph Alphabet No. 2), 45.45 baud,
//! 5-bit code with LTRS/FIGS shift, 1 start bit + 5 data bits + 1.5 stop bits.
//! Compatible with fldigi RTTY-45 defaults (170 Hz shift, 1.5 stop bits).
//!
//! FSK tones chosen for ~170 Hz shift after DAC timer rounding:
//!   Mark  (1) = 1500 Hz req → actual 1515.4 Hz  (lower tone)
//!   Space (0) = 1670 Hz req → actual 1677.3 Hz  (higher tone)
//!   Actual shift ≈ 162 Hz  (within decoder filter bandwidth)
//!   (MAMP=8 / play_quiet, CYCLE=1022, 96 MHz timer clock)
//!
//! Standard convention: Mark = lower frequency, Space = Mark + shift.
//! Idle line state = Mark (continuous lower tone).
//! Diddle = LTRS shift char (11111) transmitted during idle to keep
//! the receiver synchronized (the classic bouncy RTTY sound).
//!
//! At 45.45 baud each bit = 22.0 ms → ~6 characters per second.
//!
//! This module provides encoding + message buffer.  The actual async
//! FSK modulation runs inside the `rtty_tx` RTIC task in main.rs.

use crate::sound;
use core::sync::atomic::{AtomicBool, Ordering};

// ── Busy flag (atomic, safe across priorities) ──
static BUSY: AtomicBool = AtomicBool::new(false);

/// Mark RTTY transmission as active.  Called by rtty_tx task on entry.
pub fn mark_busy() { BUSY.store(true, Ordering::Release); }

/// Mark RTTY transmission as idle.  Called by rtty_tx task on exit.
pub fn mark_idle() { BUSY.store(false, Ordering::Release); }

/// Check if RTTY is currently transmitting.  Safe from any priority.
pub fn is_busy() -> bool { BUSY.load(Ordering::Acquire) }

// ── FSK parameters (pub for main.rs task) ──
pub const MARK_HZ: u32 = 1500;  // logical 1 → actual 1515.4 Hz (quiet, ARR=61)
pub const SPACE_HZ: u32 = 1670; // logical 0 → actual 1677.3 Hz (quiet, ARR=55)

/// Bit duration at 45.45 baud: 22 ms
pub const BIT_MS: u32 = 22;
/// Stop bit duration: 1.5 × 22 = 33 ms  (fldigi default)
pub const STOP_MS: u32 = 33;
/// Number of LTRS diddle characters for preamble and postamble sync.
pub const DIDDLE_COUNT: u32 = 4;

// ── ITA2 / Baudot encoding ──

/// LTRS shift character (Baudot code 31)
pub const LTRS_CODE: u8 = 31;
/// FIGS shift character (Baudot code 27)
pub const FIGS_CODE: u8 = 27;

// ITA2 table: index = Baudot code, value = (letter, figure)
const ITA2: [(u8, u8); 32] = [
    (0x00, 0x00), // 00 NUL
    (b'E',  b'3'), // 01
    (b'\n', b'\n'),// 02 LF
    (b'A',  b'-'), // 03
    (b' ',  b' '), // 04 SPACE
    (b'S',  b'\''),// 05
    (b'I',  b'8'), // 06
    (b'U',  b'7'), // 07
    (b'\r', b'\r'),// 08 CR
    (b'D',  b'$'), // 09
    (b'R',  b'4'), // 10
    (b'J',  b'\x07'),// 11 BELL
    (b'N',  b','), // 12
    (b'F',  b'!'), // 13
    (b'C',  b':'), // 14
    (b'K',  b'('), // 15
    (b'T',  b'5'), // 16
    (b'Z',  b'"'), // 17
    (b'L',  b')'), // 18
    (b'W',  b'2'), // 19
    (b'H',  b'#'), // 20
    (b'Y',  b'6'), // 21
    (b'P',  b'0'), // 22
    (b'Q',  b'1'), // 23
    (b'O',  b'9'), // 24
    (b'B',  b'?'), // 25
    (b'G',  b'&'), // 26
    (0xFF,  0xFF), // 27 FIGS shift
    (b'M',  b'.'), // 28
    (b'X',  b'/'), // 29
    (b'V',  b';'), // 30
    (0xFF,  0xFF), // 31 LTRS shift
];

/// Look up the Baudot code for an ASCII character.
/// Returns (baudot_code, needs_figs_shift).
/// Returns None for characters that have no Baudot representation.
pub fn ascii_to_baudot(ch: u8) -> Option<(u8, bool)> {
    let upper = if ch >= b'a' && ch <= b'z' { ch - 32 } else { ch };

    // Search letters column first
    for (code, &(letter, _)) in ITA2.iter().enumerate() {
        if letter == upper && letter != 0x00 && letter != 0xFF {
            return Some((code as u8, false));
        }
    }
    // Search figures column
    for (code, &(_, figure)) in ITA2.iter().enumerate() {
        if figure == upper && figure != 0x00 && figure != 0xFF {
            return Some((code as u8, true));
        }
    }
    None
}

// ── Message buffer ──
// Shell writes under RTIC lock, rtty_tx task reads after spawn.

const MSG_MAX: usize = 80;
static mut MSG_BUF: [u8; MSG_MAX] = [0u8; MSG_MAX];
static mut MSG_LEN: usize = 0;

/// Store a message for the RTTY task to transmit.
/// Call this from shell command handler (under RTIC lock).
pub fn set_message(msg: &[u8]) {
    let n = if msg.len() > MSG_MAX { MSG_MAX } else { msg.len() };
    unsafe {
        MSG_BUF[..n].copy_from_slice(&msg[..n]);
        MSG_LEN = n;
    }
}

/// Get the stored message.  Call from rtty_tx task only.
pub fn get_message() -> &'static [u8] {
    unsafe { &MSG_BUF[..MSG_LEN] }
}

/// Transmit one Baudot character as FSK tones (blocking, sync).
/// Start bit (space) + 5 data bits (LSB first) + 1.5 stop bits (mark).
/// Caller must provide async delay between calls.
///
/// The tone is LEFT PLAYING after this returns (mark tone for stop bit).
/// The caller's delay provides the stop bit duration.
pub fn tx_start_bit() {
    sound::play_quiet(SPACE_HZ);
}

pub fn tx_data_bit(bit: bool) {
    if bit {
        sound::play_quiet(MARK_HZ);
    } else {
        sound::play_quiet(SPACE_HZ);
    }
}

pub fn tx_stop_bit() {
    sound::play_quiet(MARK_HZ);
}

pub fn tx_idle() {
    sound::play_quiet(MARK_HZ);
}

pub fn tx_silence() {
    sound::stop();
}
