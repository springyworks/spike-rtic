#![allow(dead_code)]
//! Minimal COBS receiver for binary upload over USB CDC.
//!
//! Protocol:
//!   Host sends: `upload <size>\r\n`  (shell enters upload mode)
//!   Hub replies: `READY <size>\r\n`
//!   Host sends: COBS-encoded binary data terminated by 0x00
//!   Hub replies: `OK <received_size> <crc32>\r\n`  or  `ERR ...\r\n`
//!   Hub returns to shell mode.
//!
//! COBS framing: data is encoded so that 0x00 never appears in the payload,
//! then a single 0x00 byte marks end-of-frame. We decode in-place.

/// Maximum upload size: 64 KB (fits in SRAM2 comfortably)
pub const UPLOAD_BUF_SIZE: usize = 64 * 1024;

/// Upload receive buffer — pinned to SRAM2 (0x2004_0000) so demos
/// can be linked to a known fixed address.  NOLOAD section: not zeroed
/// at boot (content is undefined until the first upload).
///
/// This is one of the few places we use `static mut` — it needs a
/// `#[link_section]` attribute that RTIC's resource system doesn't
/// support.  Access is safe because the shell state machine ensures
/// mutual exclusion (all callers run under the RTIC shell lock).
#[link_section = ".sram2"]
static mut UPLOAD_BUF: [u8; UPLOAD_BUF_SIZE] = [0u8; UPLOAD_BUF_SIZE];

/// State of an in-progress upload.
pub struct UploadReceiver {
    /// Write index into UPLOAD_BUF (COBS-encoded bytes received so far).
    idx: usize,
    /// Expected decoded size (from the `upload <size>` command, 0 = any).
    _expected: u32,
}

impl UploadReceiver {
    pub fn new(expected: u32) -> Self {
        Self { idx: 0, _expected: expected }
    }

    /// Feed a chunk of bytes from USB. Returns:
    ///   - `None` if still receiving
    ///   - `Some(Ok(len))` if frame complete (decoded data in upload_buf()[..len])
    ///   - `Some(Err(msg))` on error
    pub fn feed(&mut self, data: &[u8]) -> Option<Result<usize, &'static str>> {
        for &b in data {
            if b == 0x00 {
                // End of COBS frame — decode in place
                let encoded = unsafe { &mut UPLOAD_BUF[..self.idx] };
                match cobs_decode_in_place(encoded) {
                    Some(decoded_len) => {
                        self.idx = 0;
                        return Some(Ok(decoded_len));
                    }
                    None => {
                        self.idx = 0;
                        return Some(Err("COBS decode error"));
                    }
                }
            } else if self.idx < UPLOAD_BUF_SIZE {
                unsafe { UPLOAD_BUF[self.idx] = b };
                self.idx += 1;
            } else {
                self.idx = 0;
                return Some(Err("overflow"));
            }
        }
        None // still receiving
    }

    /// How many encoded bytes received so far.
    pub fn bytes_received(&self) -> usize {
        self.idx
    }

    /// Abort the upload, reset state.
    pub fn abort(&mut self) {
        self.idx = 0;
    }
}

/// Access the upload buffer (decoded data lives here after successful decode).
pub fn upload_buf() -> &'static [u8] {
    // SAFETY: we only hand out a shared reference when no upload is in progress
    // (caller ensures mutual exclusion via shell state machine).
    unsafe { core::slice::from_raw_parts(core::ptr::addr_of!(UPLOAD_BUF) as *const u8, UPLOAD_BUF_SIZE) }
}

/// Mutable access to the upload buffer (for loading data from external flash).
pub fn upload_buf_mut() -> &'static mut [u8] {
    unsafe { core::slice::from_raw_parts_mut(core::ptr::addr_of_mut!(UPLOAD_BUF) as *mut u8, UPLOAD_BUF_SIZE) }
}

/// Base address of the upload buffer in RAM.
/// Demos are linked to this address (0x2004_0000 on SPIKE Prime).
pub fn upload_buf_addr() -> u32 {
    core::ptr::addr_of!(UPLOAD_BUF) as u32
}

/// CRC-32 (ISO 3309 / ITU-T V.42 / "standard" CRC-32) of a byte slice.
/// No lookup table — bit-by-bit, small code size for embedded.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

/// COBS decode in-place. Returns decoded length, or None on error.
///
/// Input: COBS-encoded bytes (without the trailing 0x00 delimiter).
/// Output: decoded bytes written to the beginning of the same buffer.
fn cobs_decode_in_place(buf: &mut [u8]) -> Option<usize> {
    let len = buf.len();
    if len == 0 {
        return Some(0);
    }

    let mut read = 0usize;
    let mut write = 0usize;

    while read < len {
        let code = buf[read] as usize;
        read += 1;

        if code == 0 {
            // Unexpected zero in COBS stream
            return None;
        }

        let data_bytes = code - 1;
        if read + data_bytes > len {
            return None; // truncated
        }

        // Copy data bytes (may overlap, but read >= write always holds)
        for _ in 0..data_bytes {
            buf[write] = buf[read];
            write += 1;
            read += 1;
        }

        // If code < 0xFF, emit a zero (unless we're at the end)
        if code < 0xFF && read < len {
            buf[write] = 0x00;
            write += 1;
        }
    }

    Some(write)
}
