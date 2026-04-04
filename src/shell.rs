//! Hub Monitor CLI — interactive command shell over USB CDC serial.
//!
//! Provides a VT100-style line-editing prompt (`spike>`) for real-time
//! debug interaction with the LEGO SPIKE Prime Hub.  A Demon-style
//! resident debug monitor: bare-metal register dumps, memory peek/poke,
//! CRC, fill, DWT hardware watchpoints, and GDB RSP remote debugging —
//! everything you need to interrogate hardware without a JTAG probe.
//!
//! ## Architecture
//!
//! The shell accumulates output in an internal ring-free **`OutBuf`**
//! (2 KB).  The USB interrupt drains it to the CDC endpoint across
//! multiple polls, solving the classic CDC truncation problem where
//! `serial.write()` silently drops data when the 64-byte USB FS
//! bulk-IN FIFO is full.
//!
//! Flow:  shell.feed(input) → push to OutBuf → USB ISR drains → host
//!
//! ## Commands
//!
//! ### General
//!   help          — list commands
//!   info          — show MCU/firmware info
//!   led <pattern> — display pattern (heart/check/cross/arrow/clear/all)
//!   led <n>       — display digit 0–9
//!   px <i> <b>    — set pixel i (0–24) to brightness b (0–100)
//!   status <color>— status LED (red/green/blue/off)
//!   btn           — read button state
//!   uptime        — show ticks since boot
//!   dfu           — print DFU instructions
//!   off           — power off the hub
//!   reset         — soft reset (SYSRESETREQ)
//!   upload [size] — receive COBS binary upload
//!   bininfo       — info on last uploaded binary
//!   go [addr]     — execute RAM demo (Demon-style "go")
//!   raminfo       — SRAM map & usage stats
//!   ramtest [arg] — RAM test (safe|sram2|addr|all)
//!
//! ### Demon debug monitor
//!   md <addr> [n] — hex memory dump (n words, default 16)
//!   mw <addr> <v> — write 32-bit word to address
//!   regs          — Cortex-M4 system register dump
//!   clocks        — RCC / PLL / bus clock configuration
//!   gpio <port>   — GPIO port state (a–e)
//!   uid           — STM32 96-bit unique device ID
//!   flash         — flash size & option bytes / RDP level
//!   adc <ch>      — read raw ADC channel (0–15)
//!   bat           — battery voltage, current, NTC, USB charger
//!   crc <a> <len> — CRC-32 of memory range
//!   fill <a><n><v>— fill memory with word pattern

use core::fmt::{self, Write};

use crate::ext_flash;
use crate::imu;
use crate::led_matrix;
use crate::user_app_io;
use crate::led_matrix::patterns;
use crate::motor;
use crate::dwt;
use crate::gdb_rsp;
use crate::power;
use crate::ram_test;
use crate::sensor;
use crate::servo;
use crate::sound;
use crate::task_state;
use crate::sandbox;
use crate::upload;

/// Maximum input line length (bytes).  Longer lines are silently truncated.
const MAX_LINE: usize = 80;

/// Size of the outgoing CDC buffer for interactive shell commands.
///
/// Demo output now goes through [`user_app_io`]'s separate 32 KB buffer,
/// so this only needs to hold the largest interactive response
/// (help ≈ 1 KB, clocks ≈ 700 B).  4 KB is generous.
const OUT_SIZE: usize = 4 * 1024;

// ── Helpers ────────────────────────────────────────────────────

/// Minimal stack-allocated buffer writer for `core::fmt::Write`.
/// Used to format numbers/hex into a small `[u8]` without allocation.
pub struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}
impl<'a> BufWriter<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    pub fn written(&self) -> &[u8] {
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

// ── Shell ──────────────────────────────────────────────────────

/// Interactive command shell with built-in output buffer.
///
/// The shell accumulates all output (echo, prompts, command responses)
/// in `out[..out_len]`.  The caller (USB interrupt) drains it via
/// [`pending()`] / [`advance()`] across multiple USB polls, which
/// solves the CDC truncation problem that plagued direct `serial.write()`
/// calls.
pub struct Shell {
    /// Input line buffer — characters typed by the user.
    buf: [u8; MAX_LINE],
    /// Current input cursor position.
    pos: usize,
    /// Uptime tick counter (seconds since boot).
    ticks: u32,
    /// Active COBS upload receiver, if any.
    upload_rx: Option<upload::UploadReceiver>,
    /// Decoded size from the last successful upload (0 = none).
    last_upload_len: usize,
    /// Outgoing data buffer — filled during feed/dispatch.
    out: [u8; OUT_SIZE],
    /// Number of valid bytes in `out`.
    out_len: usize,
    /// Number of bytes already sent to the USB CDC endpoint.
    out_sent: usize,
    /// Pending sensor probe request (port index 0–5), consumed by USB ISR.
    sensor_request: Option<u8>,
    /// Pending beep request (freq_hz, duration_ms), consumed by USB ISR.
    beep_request: Option<(u32, u32)>,
    /// Pending RTTY transmission request, consumed by USB ISR.
    rtty_request: bool,
    /// Pending test_all request, consumed by USB ISR.
    test_all_request: bool,
    /// Pending USB reconnect request, consumed by USB ISR.
    reconnect_request: bool,
    /// After sensor_poll stops (0xFF request), probe this port.
    sensor_stop_then_probe: Option<u8>,
    /// Persistent PID gains — tunable at runtime via 'pid' command.
    pub pid_config: motor::Pid,
    /// Pending PID run request: (port_idx, target_deg).
    pid_request: Option<(u8, i32)>,
    /// Pending servo run request: (port_idx, target_deg).
    servo_request: Option<(u8, i32)>,
    /// Persistent servo gains — tunable at runtime via 'stune' command.
    pub servo_config: Option<servo::ControlSettings>,
    /// True once the welcome banner has been pushed into the output buffer.
    /// Input is discarded until this is set, preventing garbled commands.
    banner_sent: bool,
    /// CRLF state: true if the last byte was CR.  If the next byte is LF,
    /// it is eaten (part of the same CRLF sequence, not a second newline).
    /// This lets the shell accept CR, LF, or CRLF — all standard line endings.
    saw_cr: bool,
    /// True once the first host input has been received after banner_sent.
    /// On the first input, any stale output (banner queued before the host
    /// connected) is cleared so the host gets a clean response.
    /// Reset to false on DTR rising edge or USB disconnect.
    host_synced: bool,
    /// Previous DTR state for edge detection.  When DTR transitions
    /// false→true the host opened the serial port — reset host_synced
    /// so stale output is cleared before the first command.
    last_dtr: bool,
    /// GDB RSP stub — active when shell is in "Demon mode".
    gdb: gdb_rsp::GdbStub,
}

impl Shell {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_LINE],
            pos: 0,
            ticks: 0,
            upload_rx: None,
            last_upload_len: 0,
            out: [0u8; OUT_SIZE],
            out_len: 0,
            out_sent: 0,
            sensor_request: None,
            beep_request: None,
            rtty_request: false,
            test_all_request: false,
            reconnect_request: false,
            sensor_stop_then_probe: None,
            pid_config: motor::Pid::new(),
            pid_request: None,
            servo_request: None,
            servo_config: None,
            banner_sent: false,
            saw_cr: false,
            host_synced: false,
            last_dtr: false,
            gdb: gdb_rsp::GdbStub::new(),
        }
    }

    /// Push the welcome banner into the output buffer and mark banner as sent.
    /// Called once from send_banner task instead of writing directly to serial.
    pub fn send_banner(&mut self) {
        self.push(Shell::prompt());
        self.banner_sent = true;
    }

    /// Detect DTR rising edge (host opened the serial port).
    /// Call from the USB ISR on every poll.  When the host opens a new
    /// `serial.Serial()` connection, DTR transitions false→true.  We
    /// reset `host_synced` so the next `feed()` clears stale output.
    pub fn check_dtr(&mut self, dtr: bool) {
        if dtr && !self.last_dtr {
            // DTR rising edge: host (re)opened the port
            self.host_synced = false;
        }
        self.last_dtr = dtr;
    }

    /// Mark the host as disconnected (USB no longer Configured).
    /// Called from the USB ISR when `usb_dev.state() != Configured`.
    /// Ensures the next connection clears any stale output.
    pub fn usb_disconnected(&mut self) {
        self.host_synced = false;
    }

    /// Whether the host is synced (first input received after connect).
    /// The USB ISR gates output drain on this: no bytes are sent to the
    /// USB hardware until the host has synced, preventing stale banner
    /// data from contaminating the first command's response.
    pub fn is_synced(&self) -> bool {
        self.host_synced
    }

    /// Take a pending sensor-probe request (port index), if any.
    /// The USB ISR calls this after feed() and spawns sensor_poll.
    pub fn take_sensor_request(&mut self) -> Option<u8> {
        self.sensor_request.take()
    }

    /// Take a pending stop-then-probe request, if any.
    pub fn take_stop_then_probe(&mut self) -> Option<u8> {
        self.sensor_stop_then_probe.take()
    }

    /// Re-enqueue a deferred probe (USB ISR retries until sensor_poll idle).
    pub fn set_stop_then_probe(&mut self, port: u8) {
        self.sensor_stop_then_probe = Some(port);
    }

    /// Take a pending PID request (port_idx, target_deg), if any.
    pub fn take_pid_request(&mut self) -> Option<(u8, i32)> {
        self.pid_request.take()
    }

    /// Take a pending servo request (port_idx, target_deg), if any.
    pub fn take_servo_request(&mut self) -> Option<(u8, i32)> {
        self.servo_request.take()
    }

    /// Take a pending beep request (freq, duration_ms), if any.
    /// The USB ISR calls this after feed() and spawns beep_tone.
    pub fn take_beep_request(&mut self) -> Option<(u32, u32)> {
        self.beep_request.take()
    }

    /// Take a pending RTTY request, if any.
    /// The USB ISR calls this after feed() and spawns rtty_tx.
    pub fn take_rtty_request(&mut self) -> bool {
        let r = self.rtty_request;
        self.rtty_request = false;
        r
    }

    /// Take a pending test_all request, if any.
    pub fn take_test_all_request(&mut self) -> bool {
        let r = self.test_all_request;
        self.test_all_request = false;
        r
    }

    /// Take a pending reconnect request, if any.
    pub fn take_reconnect_request(&mut self) -> bool {
        let r = self.reconnect_request;
        self.reconnect_request = false;
        r
    }

    // ── Output buffer management ──────────────────────────────

    /// Append bytes to the outgoing buffer.  If the buffer is full,
    /// excess data is silently dropped (better than blocking the ISR).
    pub fn push(&mut self, data: &[u8]) {
        let space = OUT_SIZE - self.out_len;
        let n = core::cmp::min(data.len(), space);
        self.out[self.out_len..self.out_len + n].copy_from_slice(&data[..n]);
        self.out_len += n;
    }

    /// Return a slice of bytes waiting to be sent over USB CDC.
    /// Call from the USB interrupt after each `usb_dev.poll()`.
    pub fn pending(&self) -> &[u8] {
        &self.out[self.out_sent..self.out_len]
    }

    /// Mark `n` bytes as successfully written to the CDC endpoint.
    /// Resets the buffer when everything has been drained.
    pub fn advance(&mut self, n: usize) {
        self.out_sent += n;
        if self.out_sent >= self.out_len {
            self.out_sent = 0;
            self.out_len = 0;
        }
    }

    // ── Tick / uptime ────────────────────────────────────────

    /// Increment uptime counter (call once per second from a periodic task).
    pub fn tick(&mut self) {
        self.ticks = self.ticks.wrapping_add(1);
    }

    // ── Input processing ─────────────────────────────────────

    /// Feed incoming bytes from USB CDC.
    ///
    /// All output (echo, prompts, command results) is pushed into the
    /// internal `out` buffer.  The caller must drain it via
    /// [`pending()`] / [`advance()`].
    pub fn feed(&mut self, data: &[u8], sensor: &sensor::SensorState, motor: &sensor::SensorState) {
        // ── Banner not yet sent: discard all input to prevent garbling ──
        if !self.banner_sent {
            return;
        }

        // ── Host sync: clear stale output on first input after (re)connect ──
        // The banner may have been queued while no host was connected.
        // When the host finally opens the port, this stale data would mix
        // with the first command's response.  Clear it here so the host
        // always gets a clean response starting from scratch.
        if !self.host_synced {
            self.host_synced = true;
            self.out_len = 0;
            self.out_sent = 0;
        }

        // ── Demo running: only accept Ctrl-C to abort ──
        // (removed — shell stays interactive while demo runs in background)

        // ── Upload mode: feed bytes to COBS receiver, no shell echo ──
        if self.upload_rx.is_some() {
            // Note: Ctrl-C (0x03) is NOT checked here because COBS-encoded
            // binary data can legitimately contain any byte value 0x01..0xFF.
            // Abort is handled by timeout in UploadReceiver instead.
            let result = if let Some(rx) = self.upload_rx.as_mut() {
                let feed_result = rx.feed(data);
                // Progress dot every 4 KB
                if feed_result.is_none() {
                    let n = rx.bytes_received();
                    if n > 0 && n % 4096 < 64 {
                        self.push(b".");
                    }
                }
                feed_result
            } else {
                None
            };
            match result {
                None => return, // still receiving
                Some(Ok(decoded_len)) => {
                    self.last_upload_len = decoded_len;
                    let buf = upload::upload_buf();
                    let crc = upload::crc32(&buf[..decoded_len]);
                    let mut tmp = [0u8; 64];
                    let mut w = BufWriter::new(&mut tmp);
                    let _ = write!(w, "\r\nOK {} 0x{:08X}\r\n", decoded_len, crc);
                    self.push(w.written());
                    led_matrix::show_pattern(&patterns::CHECK, 50);
                    unsafe { led_matrix::update() };
                    self.upload_rx = None;
                    self.push(b"spike> ");
                    return;
                }
                Some(Err(msg)) => {
                    self.push(b"\r\nERR upload: ");
                    self.push(msg.as_bytes());
                    self.push(b"\r\n");
                    led_matrix::show_pattern(&patterns::CROSS, 50);
                    unsafe { led_matrix::update() };
                    self.upload_rx = None;
                    self.push(b"spike> ");
                    return;
                }
            }
        }

        // ── GDB RSP mode ("Demon mode"): route bytes to GDB stub ──
        if self.gdb.active {
            let mut gdb_tmp = [0u8; 600];
            let n = self.gdb.feed(data, &mut gdb_tmp);
            if n > 0 {
                self.push(&gdb_tmp[..n]);
            }
            // If stub deactivated (detach/kill), print return message
            if !self.gdb.active {
                self.push(b"\r\nGDB detached. Back to shell.\r\nspike> ");
            }
            return;
        }

        // ── Normal shell mode ──
        for &b in data {
            // ── CRLF suppression: if last char was CR, eat the LF ──
            if self.saw_cr && b == b'\n' {
                self.saw_cr = false;
                continue;
            }
            self.saw_cr = false;

            match b {
                // Backspace / DEL — erase last character
                0x08 | 0x7F => {
                    if self.pos > 0 {
                        self.pos -= 1;
                        self.push(b"\x08 \x08");
                    }
                }
                // Enter (CR or LF) — execute command
                b'\r' | b'\n' => {
                    if b == b'\r' {
                        self.saw_cr = true; // remember, so we eat a following LF
                    }
                    self.push(b"\r\n");
                    // Copy input line to stack so self.buf is free for push()
                    let mut line_copy = [0u8; MAX_LINE];
                    let line_len = self.pos;
                    line_copy[..line_len].copy_from_slice(&self.buf[..line_len]);
                    self.pos = 0;
                    self.dispatch(&line_copy[..line_len], sensor, motor);
                    self.push(b"spike> ");
                }
                // Printable ASCII — echo and buffer
                0x20..=0x7E => {
                    if self.pos < MAX_LINE - 1 {
                        self.buf[self.pos] = b;
                        self.pos += 1;
                        self.push(&[b]);
                    }
                }
                // Ctrl-C → cancel current line
                0x03 => {
                    self.push(b"^C\r\n");
                    self.pos = 0;
                    self.push(b"spike> ");
                }
                _ => {}
            }
        }
    }

    /// Parse and execute the command line.
    ///
    /// Takes a borrowed slice of the already-copied input line so that
    /// `self.buf` / `self.out` are free for writing.
    fn dispatch(&mut self, line: &[u8], sensor: &sensor::SensorState, motor: &sensor::SensorState) {
        let line = core::str::from_utf8(line).unwrap_or("");
        let mut parts = line.split_ascii_whitespace();
        let cmd = match parts.next() {
            Some(c) => c,
            None => return, // empty line
        };

        match cmd {
            "help" | "?" => {
                // Help text is ~900 bytes.  With the OutBuf, this is
                // no longer truncated — it drains across USB polls.
                self.push(b"=== Hub Monitor CLI (Demon) ===\r\n");
                self.push(b"  help          - this message\r\n");
                self.push(b"  info          - MCU info\r\n");
                self.push(b"  led <pattern> - heart/check/cross/arrow/usb/clear/all\r\n");
                self.push(b"  led <0-9>     - show digit\r\n");
                self.push(b"  px <i> <b>    - set pixel i brightness b (0-100)\r\n");
                self.push(b"  status <clr>  - red/green/blue/white/off\r\n");
                self.push(b"  btn           - read buttons\r\n");
                self.push(b"  adc <ch>      - read raw ADC channel (0-15)\r\n");
                self.push(b"  bat           - battery voltage/current/temp\r\n");
                self.push(b"  dfu           - print DFU flash instructions\r\n");
                self.push(b"  off           - clean power off\r\n");
                self.push(b"  sleep         - low-power stop mode\r\n");
                self.push(b"  reset         - soft reset (SYSRESETREQ)\r\n");
                self.push(b"  uptime        - ticks since boot\r\n");
                self.push(b"  beep [f] [ms] - play tone (default 1000 Hz 200 ms)\r\n");
                self.push(b"  load          - battery + system load\r\n");
                self.push(b"  upload [size] - receive COBS binary upload\r\n");
                self.push(b"  bininfo       - info on last upload in RAM\r\n");
                self.push(b"  go [addr]     - run RAM demo sandboxed (MPU protected)\r\n");
                self.push(b"  go! [addr]    - run RAM demo privileged (firmware dev)\r\n");
                self.push(b"  kill [-2|-9|-15] - signal running demo (default: -15/SIGTERM)\r\n");
                self.push(b"  raminfo       - SRAM map & usage\r\n");
                self.push(b"  ramtest [arg] - RAM test (safe|sram2|addr|all)\r\n");
                self.push(b"  trace [on|off|clear|legend] - SVC/sensor trace buffer\r\n");
                self.push(b"--- Demon debug monitor ---\r\n");
                self.push(b"  md <addr> [n] - hex dump n words (default 16)\r\n");
                self.push(b"  mw <addr> <v> - write 32-bit word\r\n");
                self.push(b"  regs          - Cortex-M4 system regs\r\n");
                self.push(b"  clocks        - RCC / PLL config\r\n");
                self.push(b"  gpio <a-e>    - GPIO port state\r\n");
                self.push(b"  uid           - STM32 unique device ID\r\n");
                self.push(b"  flash         - flash size & option bytes\r\n");
                self.push(b"  crc <addr> <n>- CRC-32 of n bytes at addr\r\n");
                self.push(b"  fill <a><n><v>- fill n words at addr with val\r\n");
                self.push(b"--- Motor control ---\r\n");
                self.push(b"  motor pwm <port> <duty> - set motor PWM (-100 to 100)\r\n");
                self.push(b"  motor <a-f> <speed> - set motor (-100..100, 0=stop)\r\n");
                self.push(b"  motor <a-f> brake   - brake motor\r\n");
                self.push(b"  motor <a-f> ramp    - ramp 5..100% fwd+rev, 1.5s/step\r\n");
                self.push(b"  motor <a-f> diag    - timer/GPIO register dump\r\n");
                self.push(b"  motor <a-f> trial   - 14-stage PWM config trial\r\n");
                self.push(b"  motor <a-f> pos     - read position (needs 'sensor <port>' first)\r\n");
                self.push(b"  motor <a-f> pid [d] - PID hold position d deg (10s, needs sensor)\r\n");
                self.push(b"  pid              - show PID gains\r\n");
                self.push(b"  pid <kp|ki|kd|sc|dz> <val> - set PID param live\r\n");
                self.push(b"  servo <a-f> hold [d]- Pybricks-style observer+PID hold d deg\r\n");
                self.push(b"  servo <a-f> stop    - stop servo, brake motor\r\n");
                self.push(b"  stune               - show servo PID gains\r\n");
                self.push(b"  stune <kp|ki|kd|icm|amax|ptol> <val> - set servo param\r\n");
                self.push(b"  stune reset         - revert to motor-type defaults\r\n");
                self.push(b"--- Sensor (LPF2 LUMP) ---\r\n");
                self.push(b"  ports               - show all 6 port states\r\n");
                self.push(b"  sensor <a-f>        - probe port (LUMP sync)\r\n");
                self.push(b"  sensor              - show current sensor data\r\n");
                self.push(b"  sensor mode <0-9>   - switch sensor mode\r\n");
                self.push(b"  sensor diag <a-f>   - dump port GPIO/UART state\r\n");
                self.push(b"  sensor raw <a-f> [b] - raw byte sniff at baud b\r\n");
                self.push(b"  sensor stop         - stop sensor polling\r\n");
                self.push(b"  light <r> <g> <b>   - color sensor LED (0-100 each)\r\n");
                self.push(b"--- IMU (LSM6DS3TR-C, I2C2) ---\r\n");
                self.push(b"  imu [init]          - init IMU, show WHO_AM_I\r\n");
                self.push(b"  imu read            - read accel + gyro XYZ\r\n");
                self.push(b"--- Audio ---\r\n");
                self.push(b"  rtty <text>     - send Baudot RTTY (45.45 baud FSK)\r\n");
                self.push(b"--- System ---\r\n");
                self.push(b"  ps              - RTIC task tree (who spawned whom)\r\n");
                self.push(b"  ls              - ROM/RAM memory map\r\n");
                self.push(b"  version         - firmware version + API version\r\n");
                self.push(b"  time            - human-readable uptime (Hh MMm SSs)\r\n");
                self.push(b"  unixtime        - pseudo unix timestamp from boot\r\n");
                self.push(b"  test_all        - ballet: probe ports, exercise everything\r\n");
                self.push(b"  reconnect-ser   - drop USB serial 2 s (terminal reconnect)\r\n");
                self.push(b"--- External Flash (SPI2, 32 MB) ---\r\n");
                self.push(b"  spiflash        - JEDEC ID + status\r\n");
                self.push(b"  spiflash read <a> [n] - hex dump (max 256 bytes)\r\n");
                self.push(b"  spiflash erase <a>    - erase 4KB sector\r\n");
                self.push(b"  spiflash write <a><v> - write 4 bytes\r\n");
                self.push(b"  spiflash store <a>[sz]- RAM upload buf -> flash\r\n");
                self.push(b"  spiflash load <a><sz> - flash -> RAM buf, then go\r\n");
                self.push(b"  spiflash run  <a><sz> - load + execute (sandboxed)\r\n");
                self.push(b"  spiflash dir          - list stored binaries\r\n");
                self.push(b"--- DWT watchpoints (self-hosted, no JTAG) ---\r\n");
                self.push(b"  dwt status          - show 4 DWT comparators\r\n");
                self.push(b"  dwt set <n> <addr> [r|w|rw|pc] - arm watchpoint\r\n");
                self.push(b"  dwt clear [n]       - disarm watchpoint (all if no n)\r\n");
                self.push(b"  dwt init            - re-init DWT + DebugMonitor\r\n");
                self.push(b"--- GDB remote debug (Demon mode) ---\r\n");
                self.push(b"  gdb               - enter GDB RSP stub mode\r\n");
                self.push(b"  bye|quit|exit  - end session (= reconnect-ser)\r\n");
                self.push(b"  dfu            - print DFU flash instructions\r\n");
                self.push(b"  off            - clean power off (release PA13)\r\n");
                self.push(b"  sleep          - power off, center button re-boots\r\n");
            }
            "info" => {
                self.push(b"LEGO SPIKE Prime Hub\r\n");
                self.push(b"MCU: STM32F413VGT6 (Cortex-M4F @ 96MHz)\r\n");
                self.push(b"Flash: 1MB, RAM: 320KB\r\n");
                self.push(b"FW: RTIC v2 spike-prime-rtic\r\n");

                // Stack high-water mark
                let (used, total) = unsafe { sandbox::stack_high_water() };
                let mut tmp = [0u8; 80];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "Stack: {} / {} bytes used\r\n", used, total);
                self.push(w.written());

                // Previous-boot stack fault
                let (mmfsr, addr) = sandbox::last_stack_fault();
                if mmfsr != 0 {
                    let mut tmp2 = [0u8; 80];
                    let mut w2 = BufWriter::new(&mut tmp2);
                    let _ = write!(w2, "PREV FAULT: MMFSR=0x{:02X} addr=0x{:08X}\r\n", mmfsr, addr);
                    self.push(w2.written());
                }
            }
            "led" => {
                if let Some(arg) = parts.next() {
                    match arg {
                        "heart" => {
                            led_matrix::show_pattern(&patterns::HEART, 50);
                            self.push(b"OK heart\r\n");
                        }
                        "check" => {
                            led_matrix::show_pattern(&patterns::CHECK, 50);
                            self.push(b"OK check\r\n");
                        }
                        "cross" => {
                            led_matrix::show_pattern(&patterns::CROSS, 50);
                            self.push(b"OK cross\r\n");
                        }
                        "arrow" => {
                            led_matrix::show_pattern(&patterns::ARROW_UP, 50);
                            self.push(b"OK arrow\r\n");
                        }
                        "usb" => {
                            led_matrix::show_pattern(&patterns::USB_ICON, 50);
                            self.push(b"OK usb\r\n");
                        }
                        "clear" => {
                            led_matrix::show_pattern(&[0u8; 25], 0);
                            self.push(b"OK clear\r\n");
                        }
                        "all" => {
                            led_matrix::show_pattern(&patterns::ALL_ON, 40);
                            self.push(b"OK all\r\n");
                        }
                        digit if digit.len() == 1 => {
                            if let Some(n) = digit.as_bytes()[0].checked_sub(b'0') {
                                if n <= 9 {
                                    let pat = patterns::digit(n);
                                    led_matrix::show_pattern(&pat, 50);
                                    self.push(b"OK digit\r\n");
                                } else {
                                    self.push(b"ERR: digit 0-9\r\n");
                                }
                            } else {
                                self.push(b"ERR: unknown pattern\r\n");
                            }
                        }
                        _ => {
                            self.push(b"ERR: unknown pattern. Try: heart check cross arrow usb clear all 0-9\r\n");
                        }
                    }
                    unsafe { led_matrix::update() };
                } else {
                    self.push(b"Usage: led <pattern|digit>\r\n");
                }
            }
            "px" => {
                let idx = parts.next().and_then(|s| parse_u32(s));
                let bri = parts.next().and_then(|s| parse_u32(s));
                match (idx, bri) {
                    (Some(i), Some(b)) if i < 25 && b <= 100 => {
                        led_matrix::set_pixel(i as usize, b as u16);
                        unsafe { led_matrix::update() };
                        self.push(b"OK\r\n");
                    }
                    _ => self.push(b"Usage: px <0-24> <0-100>\r\n"),
                }
            }
            "status" => {
                if let Some(color) = parts.next() {
                    let (r, g, b) = match color {
                        "red" => (0x8000, 0, 0),
                        "green" => (0, 0x8000, 0),
                        "blue" => (0, 0, 0x8000),
                        "white" => (0x4000, 0x4000, 0x4000),
                        "off" => (0, 0, 0),
                        _ => {
                            self.push(b"ERR: red/green/blue/white/off\r\n");
                            return;
                        }
                    };
                    led_matrix::set_status_rgb(crate::pins::STATUS_TOP, r, g, b);
                    unsafe { led_matrix::update() };
                    self.push(b"OK\r\n");
                } else {
                    self.push(b"Usage: status <red|green|blue|white|off>\r\n");
                }
            }
            "btn" => {
                let buttons = crate::read_buttons();
                if buttons == 0 {
                    self.push(b"none\r\n");
                } else {
                    if buttons & crate::BTN_CENTER != 0 {
                        self.push(b"CENTER ");
                    }
                    if buttons & crate::BTN_LEFT != 0 {
                        self.push(b"LEFT ");
                    }
                    if buttons & crate::BTN_RIGHT != 0 {
                        self.push(b"RIGHT ");
                    }
                    self.push(b"\r\n");
                }
            }
            "dfu" => {
                // DFU requires a physical button combo — cannot be entered from software.
                self.push(b"=== DFU Flash Instructions ===\r\n");
                self.push(b"DFU requires a physical button combo:\r\n");
                self.push(b"  1. Disconnect USB cable\r\n");
                self.push(b"  2. Hold CENTER + LEFT buttons together\r\n");
                self.push(b"  3. While holding, reconnect USB cable\r\n");
                self.push(b"  4. Keep holding until LED blinks pink/magenta\r\n");
                self.push(b"  5. Release buttons\r\n");
                self.push(b"Verify: lsusb | grep 0694:0011\r\n");
                self.push(b"Flash:  dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D firmware.bin\r\n");
                self.push(b"\r\nNote: 'off' to power down first, or just unplug.\r\n");
            }
            "off" => {
                self.push(b"Powering off (clean shutdown)...\r\n");
                power::delay_ms(100);
                power::clean_shutdown();
            }
            "sleep" => {
                self.push(b"Powering off... press center button to boot\r\n");
                power::delay_ms(200); // let USB flush
                power::deep_sleep();
            }
            "uptime" => {
                let t = self.ticks;
                let mut num_buf = [0u8; 12];
                let s = format_u32(t, &mut num_buf);
                self.push(s);
                self.push(b" ticks\r\n");
            }
            "beep" => {
                let freq = parts.next().and_then(|s| parse_u32(s)).unwrap_or(1000);
                let dur = parts.next().and_then(|s| parse_u32(s)).unwrap_or(200);
                let freq = if freq < 100 { 100 } else if freq > 10000 { 10000 } else { freq };
                let dur = if dur > 5000 { 5000 } else { dur };
                // Non-blocking: USB ISR will spawn the async beep_tone task
                self.beep_request = Some((freq, dur));
                let mut tmp = [0u8; 40];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "beep {} Hz {} ms\r\n", freq, dur);
                self.push(w.written());
            }
            "rtty" => {
                // Collect the rest of the line as the message.
                // remainder() is unstable, so we find the offset manually.
                let rest = if line.len() > 5 { &line[5..] } else { "" };
                let rest = rest.trim();
                let msg = if rest.is_empty() { b"CQ CQ CQ DE SPIKE" as &[u8] } else { rest.as_bytes() };
                crate::rtty::set_message(msg);
                let mut tmp = [0u8; 100];
                let mut w = BufWriter::new(&mut tmp);
                let n = if msg.len() > 80 { 80 } else { msg.len() };
                let _ = write!(w, "RTTY 45.45 baud: \"");
                self.push(w.written());
                self.push(&msg[..n]);
                self.push(b"\"\r\n");
                self.rtty_request = true;
            }
            "load" => {
                // Battery info
                let v_raw = crate::read_adc(11);
                let v_mv = v_raw * 6600 / 4095;
                let i_raw = crate::read_adc(10);
                let ntc_raw = crate::read_adc(8);
                let usb_raw = crate::read_adc(3);

                let mut tmp = [0u8; 200];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "Battery:  {} mV (raw {})\r\n", v_mv, v_raw);
                let _ = write!(w, "Current:  raw {} (ch10)\r\n", i_raw);
                let _ = write!(w, "NTC:      raw {} (ch8)\r\n", ntc_raw);
                let _ = write!(w, "USB chg:  raw {} (ch3)\r\n", usb_raw);
                if v_mv < power::BATTERY_CRITICAL_MV {
                    let _ = write!(w, "WARNING: battery critically low!\r\n");
                }
                self.push(w.written());
            }
            "upload" => {
                let expected = parts.next().and_then(|s| parse_u32(s)).unwrap_or(0);
                if expected as usize > upload::UPLOAD_BUF_SIZE {
                    self.push(b"ERR: too large (max 64K)\r\n");
                    return;
                }
                self.upload_rx = Some(upload::UploadReceiver::new(expected));
                let mut tmp = [0u8; 40];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "READY {}\r\n", expected);
                self.push(w.written());
                led_matrix::show_pattern(&patterns::USB_ICON, 40);
                unsafe { led_matrix::update() };
                return;
            }
            "bininfo" => {
                let addr = upload::upload_buf_addr();
                let size = self.last_upload_len;
                let mut tmp = [0u8; 64];
                if size == 0 {
                    let mut w = BufWriter::new(&mut tmp);
                    let _ = write!(w, "No upload. buf @ 0x{:08X} ({}K)\r\n",
                        addr, upload::UPLOAD_BUF_SIZE / 1024);
                    self.push(w.written());
                } else {
                    let buf = upload::upload_buf();
                    let crc = upload::crc32(&buf[..size]);
                    let mut w = BufWriter::new(&mut tmp);
                    let _ = write!(w, "RAM bin: {} B @ 0x{:08X} CRC 0x{:08X}\r\n",
                        size, addr, crc);
                    self.push(w.written());
                }
            }
            "go" | "go!" => {
                // All RAM demos now run as preemptible RTIC tasks (never idle loop).
                // "go"  = sandboxed (MPU + SVC, unprivileged)
                // "go!" = privileged (direct fn pointers, no MPU)
                let sandboxed = cmd != "go!";

                if user_app_io::is_running() {
                    self.push(b"ERR: demo already running\r\n");
                    return;
                }

                let addr = parts.next().and_then(|s| parse_num(s))
                    .unwrap_or(upload::upload_buf_addr());

                // Sanity: must be in SRAM range
                if addr < 0x2000_0000 || addr >= 0x2005_0000 {
                    self.push(b"ERR: addr outside SRAM (0x20000000..0x20050000)\r\n");
                    return;
                }

                let mut tmp = [0u8; 80];
                let mut w = BufWriter::new(&mut tmp);
                if sandboxed {
                    let _ = write!(w, "go 0x{:08X} [sandboxed]\r\n", addr);
                } else {
                    let _ = write!(w, "go! 0x{:08X} [PRIVILEGED]\r\n", addr);
                }
                self.push(w.written());

                // Request demo launch — always handled by RTIC run_demo task
                user_app_io::request(addr, sandboxed);
            }

            "kill" => {
                // Unix-style kill: -15 (SIGTERM, default), -2 (SIGINT), -9 (SIGKILL)
                if !user_app_io::is_running() {
                    self.push(b"No demo running.\r\n");
                    return;
                }
                let sig = parts.next().unwrap_or("-15");
                match sig {
                    "-2" | "-15" => {
                        // SIGINT / SIGTERM — cooperative: set abort flag,
                        // demo exits at next API call (write, delay, motor, etc.)
                        user_app_io::request_abort();
                        self.push(b"Abort requested (SIGTERM).\r\n");
                    }
                    "-9" => {
                        // SIGKILL — immediate: force-kill + brake motors
                        user_app_io::request_abort();
                        if crate::sandbox::is_sandboxed() {
                            unsafe { crate::sandbox::force_kill_sandbox(); }
                        }
                        for p in 0..6u32 { motor::brake(p); }
                        sound::stop();
                        self.push(b"Killed (SIGKILL).\r\n");
                    }
                    _ => {
                        self.push(b"Usage: kill [-2|-9|-15] (default: -15)\r\n");
                    }
                }
            }

            "raminfo" => {
                ram_test::cmd_raminfo(&mut |d| self.push(d));
            }
            "ramtest" => {
                if let Some(arg) = parts.next() {
                    match arg {
                        "safe" => ram_test::cmd_ramtest_safe(&mut |d| self.push(d)),
                        "sram2" => ram_test::cmd_ramtest_sram2(&mut |d| self.push(d)),
                        "addr" => ram_test::cmd_addrtest(&mut |d| self.push(d)),
                        "all" => ram_test::cmd_ramtest_all(&mut |d| self.push(d)),
                        _ => self.push(b"Usage: ramtest [safe|sram2|addr|all]\r\n"),
                    }
                } else {
                    ram_test::cmd_ramtest_all(&mut |d| self.push(d));
                }
            }
            "trace" => {
                match parts.next() {
                    Some("on") => {
                        crate::trace::clear();
                        crate::trace::set_enabled(true);
                        self.push(b"Trace ON (buffer cleared)\r\n");
                    }
                    Some("off") => {
                        crate::trace::set_enabled(false);
                        self.push(b"Trace OFF\r\n");
                    }
                    Some("clear") => {
                        crate::trace::clear();
                        self.push(b"Trace buffer cleared\r\n");
                    }
                    Some("legend") => {
                        crate::trace::dump_legend(|d| self.push(d));
                    }
                    _ => {
                        // Default: dump the trace buffer
                        crate::trace::dump(|d| self.push(d));
                    }
                }
            }

            // ── Demon debug monitor commands ──

            "md" => {
                // Memory dump: md <addr> [count]  (count = number of 32-bit words, default 16)
                let addr = parts.next().and_then(|s| parse_num(s));
                let count = parts.next().and_then(|s| parse_num(s)).unwrap_or(16);
                match addr {
                    Some(a) if a & 3 == 0 => {
                        let a = a as u32;
                        let n = if count > 256 { 256 } else { count };
                        for i in 0..n {
                            let offset = i * 4;
                            if i % 4 == 0 {
                                let mut tmp = [0u8; 16];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "{:08X}: ", a.wrapping_add(offset));
                                self.push(w.written());
                            }
                            let val = unsafe {
                                core::ptr::read_volatile((a.wrapping_add(offset)) as *const u32)
                            };
                            let mut tmp = [0u8; 12];
                            let mut w = BufWriter::new(&mut tmp);
                            let _ = write!(w, "{:08X} ", val);
                            self.push(w.written());
                            if i % 4 == 3 || i == n - 1 {
                                self.push(b"\r\n");
                            }
                        }
                    }
                    Some(_) => self.push(b"ERR: addr must be 4-byte aligned\r\n"),
                    None => self.push(b"Usage: md <addr> [count]\r\n"),
                }
            }
            "mw" => {
                // Memory write: mw <addr> <value>
                let addr = parts.next().and_then(|s| parse_num(s));
                let val = parts.next().and_then(|s| parse_num(s));
                match (addr, val) {
                    (Some(a), Some(v)) if a & 3 == 0 => {
                        unsafe {
                            core::ptr::write_volatile(a as *mut u32, v);
                        }
                        let mut tmp = [0u8; 32];
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "OK [{:08X}] <- {:08X}\r\n", a, v);
                        self.push(w.written());
                    }
                    (Some(_), Some(_)) => self.push(b"ERR: addr must be 4-byte aligned\r\n"),
                    _ => self.push(b"Usage: mw <addr> <value>\r\n"),
                }
            }
            "regs" => {
                // Cortex-M4 system registers (Demon-style register dump)
                self.push(b"=== Cortex-M4 System Registers ===\r\n");
                let mut tmp = [0u8; 48];

                // CPUID (SCB 0xE000_ED00)
                let cpuid = unsafe { core::ptr::read_volatile(0xE000_ED00 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "CPUID:    0x{:08X}\r\n", cpuid);
                self.push(w.written());

                // Decode CPUID
                let implementer = (cpuid >> 24) & 0xFF;
                let variant = (cpuid >> 20) & 0xF;
                let partno = (cpuid >> 4) & 0xFFF;
                let revision = cpuid & 0xF;
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  impl=0x{:02X} var={} part=0x{:03X} rev={}\r\n",
                    implementer, variant, partno, revision);
                self.push(w.written());

                // ICSR (0xE000_ED04)
                let icsr = unsafe { core::ptr::read_volatile(0xE000_ED04 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "ICSR:     0x{:08X}\r\n", icsr);
                self.push(w.written());

                // VTOR (0xE000_ED08)
                let vtor = unsafe { core::ptr::read_volatile(0xE000_ED08 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "VTOR:     0x{:08X}\r\n", vtor);
                self.push(w.written());

                // AIRCR (0xE000_ED0C)
                let aircr = unsafe { core::ptr::read_volatile(0xE000_ED0C as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "AIRCR:    0x{:08X}\r\n", aircr);
                self.push(w.written());

                // SCR (0xE000_ED10)
                let scr = unsafe { core::ptr::read_volatile(0xE000_ED10 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "SCR:      0x{:08X}\r\n", scr);
                self.push(w.written());

                // CCR (0xE000_ED14)
                let ccr = unsafe { core::ptr::read_volatile(0xE000_ED14 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "CCR:      0x{:08X}\r\n", ccr);
                self.push(w.written());

                // SHCSR (0xE000_ED24)
                let shcsr = unsafe { core::ptr::read_volatile(0xE000_ED24 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "SHCSR:    0x{:08X}\r\n", shcsr);
                self.push(w.written());

                // DFSR (0xE000_ED30) — Debug Fault Status
                let dfsr = unsafe { core::ptr::read_volatile(0xE000_ED30 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "DFSR:     0x{:08X}\r\n", dfsr);
                self.push(w.written());

                // DEMCR (0xE000_EDFC) — Debug Exception & Monitor Control
                let demcr = unsafe { core::ptr::read_volatile(0xE000_EDFC as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "DEMCR:    0x{:08X}\r\n", demcr);
                self.push(w.written());

                // DWT CYCCNT (0xE0001004) — cycle counter
                let cyccnt = unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "CYCCNT:   0x{:08X}\r\n", cyccnt);
                self.push(w.written());

                // SP
                let sp: u32;
                unsafe { core::arch::asm!("mov {}, sp", out(reg) sp) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "SP:       0x{:08X}\r\n", sp);
                self.push(w.written());

                // PRIMASK
                let primask: u32;
                unsafe { core::arch::asm!("mrs {}, PRIMASK", out(reg) primask) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "PRIMASK:  0x{:08X}\r\n", primask);
                self.push(w.written());

                // BASEPRI
                let basepri: u32;
                unsafe { core::arch::asm!("mrs {}, BASEPRI", out(reg) basepri) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "BASEPRI:  0x{:08X}\r\n", basepri);
                self.push(w.written());

                // CONTROL
                let control: u32;
                unsafe { core::arch::asm!("mrs {}, CONTROL", out(reg) control) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "CONTROL:  0x{:08X}\r\n", control);
                self.push(w.written());
            }
            "clocks" => {
                // RCC register dump — show clock tree config
                self.push(b"=== RCC Clock Config ===\r\n");
                let mut tmp = [0u8; 56];

                let rcc = 0x4002_3800u32;
                let cr = unsafe { core::ptr::read_volatile((rcc + 0x00) as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "CR:      0x{:08X}\r\n", cr);
                self.push(w.written());
                // Decode oscillator status
                if cr & (1 << 1) != 0 { self.push(b"  HSI: ON\r\n"); }
                if cr & (1 << 17) != 0 { self.push(b"  HSE: ON\r\n"); }
                if cr & (1 << 25) != 0 { self.push(b"  PLL: ON\r\n"); }

                let pllcfgr = unsafe { core::ptr::read_volatile((rcc + 0x04) as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "PLLCFGR: 0x{:08X}\r\n", pllcfgr);
                self.push(w.written());

                let pllm = pllcfgr & 0x3F;
                let plln = (pllcfgr >> 6) & 0x1FF;
                let pllp_bits = (pllcfgr >> 16) & 0x3;
                let pllp = (pllp_bits + 1) * 2;
                let pllq = (pllcfgr >> 24) & 0xF;
                let pllsrc = if pllcfgr & (1 << 22) != 0 { "HSE" } else { "HSI" };

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  M={} N={} P={} Q={} src={}\r\n",
                    pllm, plln, pllp, pllq, pllsrc);
                self.push(w.written());

                // Compute frequencies
                let src_freq: u32 = if pllcfgr & (1 << 22) != 0 { 16_000_000 } else { 16_000_000 };
                let vco_in = src_freq / pllm;
                let vco_out = vco_in * plln;
                let sysclk = vco_out / pllp;
                let usbclk = vco_out / pllq;

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  SYSCLK: {} MHz\r\n", sysclk / 1_000_000);
                self.push(w.written());

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  USB:    {} MHz\r\n", usbclk / 1_000_000);
                self.push(w.written());

                let cfgr = unsafe { core::ptr::read_volatile((rcc + 0x08) as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "CFGR:    0x{:08X}\r\n", cfgr);
                self.push(w.written());

                let sws = (cfgr >> 2) & 0x3;
                let swsrc = match sws {
                    0 => "HSI",
                    1 => "HSE",
                    2 => "PLL",
                    _ => "???",
                };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  SWS: {} ({})\r\n", sws, swsrc);
                self.push(w.written());

                let ppre1 = (cfgr >> 10) & 0x7;
                let ppre2 = (cfgr >> 13) & 0x7;
                let apb1_div = if ppre1 & 4 != 0 { 1 << ((ppre1 & 3) + 1) } else { 1 };
                let apb2_div = if ppre2 & 4 != 0 { 1 << ((ppre2 & 3) + 1) } else { 1 };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  APB1: /{} ({}MHz)  APB2: /{} ({}MHz)\r\n",
                    apb1_div, sysclk / apb1_div / 1_000_000,
                    apb2_div, sysclk / apb2_div / 1_000_000);
                self.push(w.written());

                // AHB/APB enable registers
                let ahb1enr = unsafe { core::ptr::read_volatile((rcc + 0x30) as *const u32) };
                let ahb2enr = unsafe { core::ptr::read_volatile((rcc + 0x34) as *const u32) };
                let apb1enr = unsafe { core::ptr::read_volatile((rcc + 0x40) as *const u32) };
                let apb2enr = unsafe { core::ptr::read_volatile((rcc + 0x44) as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "AHB1ENR: 0x{:08X}\r\n", ahb1enr);
                self.push(w.written());
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "AHB2ENR: 0x{:08X}\r\n", ahb2enr);
                self.push(w.written());
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "APB1ENR: 0x{:08X}\r\n", apb1enr);
                self.push(w.written());
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "APB2ENR: 0x{:08X}\r\n", apb2enr);
                self.push(w.written());
            }
            "gpio" => {
                // GPIO port state dump — gpio <a|b|c|d|e>
                if let Some(port_name) = parts.next() {
                    let base = match port_name {
                        "a" | "A" => Some(crate::pins::GPIOA),
                        "b" | "B" => Some(crate::pins::GPIOB),
                        "c" | "C" => Some(crate::pins::GPIOC),
                        "d" | "D" => Some(crate::pins::GPIOD),
                        "e" | "E" => Some(crate::pins::GPIOE),
                        _ => None,
                    };
                    if let Some(base) = base {
                        let mut tmp = [0u8; 48];

                        let moder = unsafe { core::ptr::read_volatile((base + 0x00) as *const u32) };
                        let port_ch = port_name.as_bytes()[0].to_ascii_uppercase() as char;
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "GPIO{} MODER:   0x{:08X}\r\n", port_ch, moder);
                        self.push(w.written());

                        let otyper = unsafe { core::ptr::read_volatile((base + 0x04) as *const u32) };
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "  OTYPER:  0x{:08X}\r\n", otyper);
                        self.push(w.written());

                        let ospeedr = unsafe { core::ptr::read_volatile((base + 0x08) as *const u32) };
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "  OSPEEDR: 0x{:08X}\r\n", ospeedr);
                        self.push(w.written());

                        let pupdr = unsafe { core::ptr::read_volatile((base + 0x0C) as *const u32) };
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "  PUPDR:   0x{:08X}\r\n", pupdr);
                        self.push(w.written());

                        let idr = unsafe { core::ptr::read_volatile((base + 0x10) as *const u32) };
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "  IDR:     0x{:04X}\r\n", idr & 0xFFFF);
                        self.push(w.written());

                        let odr = unsafe { core::ptr::read_volatile((base + 0x14) as *const u32) };
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "  ODR:     0x{:04X}\r\n", odr & 0xFFFF);
                        self.push(w.written());

                        // Decode pin modes
                        self.push(b"  Pins: ");
                        for pin in 0..16u32 {
                            let mode = (moder >> (pin * 2)) & 0x3;
                            let ch = match mode {
                                0 => b'I', // Input
                                1 => b'O', // Output
                                2 => b'A', // Alternate function
                                3 => b'N', // aNalog
                                _ => b'?',
                            };
                            self.push(&[ch]);
                        }
                        self.push(b"\r\n");
                        self.push(b"  (I=in O=out A=AF N=analog)\r\n");
                    } else {
                        self.push(b"ERR: port a-e\r\n");
                    }
                } else {
                    self.push(b"Usage: gpio <a|b|c|d|e>\r\n");
                }
            }
            "uid" => {
                // STM32F4 96-bit unique ID at 0x1FFF_7A10
                let uid0 = unsafe { core::ptr::read_volatile(0x1FFF_7A10 as *const u32) };
                let uid1 = unsafe { core::ptr::read_volatile(0x1FFF_7A14 as *const u32) };
                let uid2 = unsafe { core::ptr::read_volatile(0x1FFF_7A18 as *const u32) };
                let mut tmp = [0u8; 48];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "UID: {:08X}-{:08X}-{:08X}\r\n", uid0, uid1, uid2);
                self.push(w.written());
            }
            "flash" => {
                // Flash size register (in KB) at 0x1FFF_7A22
                let flash_kb = unsafe {
                    core::ptr::read_volatile(0x1FFF_7A22 as *const u16)
                } as u32;
                let mut tmp = [0u8; 48];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "Flash: {} KB ({} bytes)\r\n", flash_kb, flash_kb * 1024);
                self.push(w.written());

                // Flash option bytes (FLASH_OPTCR 0x4002_3C14)
                let optcr = unsafe { core::ptr::read_volatile(0x4002_3C14 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "OPTCR:  0x{:08X}\r\n", optcr);
                self.push(w.written());

                // Flash access control register
                let acr = unsafe { core::ptr::read_volatile(0x4002_3C00 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "ACR:    0x{:08X} (ws={})\r\n", acr, acr & 0xF);
                self.push(w.written());

                // Read protection level
                let rdp = (optcr >> 8) & 0xFF;
                let rdp_str = match rdp {
                    0xAA => "Level 0 (no protection)",
                    0xCC => "Level 2 (permanent!)",
                    _ => "Level 1 (read protected)",
                };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "RDP:    0x{:02X} {}\r\n", rdp, rdp_str);
                self.push(w.written());
            }
            "adc" => {
                // Raw ADC channel read: adc <ch>
                if let Some(ch) = parts.next().and_then(|s| parse_u32(s)) {
                    if ch <= 15 {
                        let val = crate::read_adc(ch);
                        let mut tmp = [0u8; 40];
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "ADC ch{}: {} (0x{:03X})\r\n", ch, val, val);
                        self.push(w.written());
                    } else {
                        self.push(b"ERR: channel 0-15\r\n");
                    }
                } else {
                    self.push(b"Usage: adc <0-15>\r\n");
                }
            }
            "bat" => {
                // Battery readings via ADC
                // ch10 = BAT_CURRENT (PC0), ch11 = BAT_VOLTAGE (PC1),
                // ch8 = BAT_NTC (PB0), ch3 = USB charger current (PA3)
                let v_raw = crate::read_adc(11);
                let i_raw = crate::read_adc(10);
                let ntc_raw = crate::read_adc(8);
                let usb_raw = crate::read_adc(3);
                let mut tmp = [0u8; 48];

                // Voltage: ADC has 3.3V ref, 12-bit; voltage divider ~2:1
                // so V_bat ≈ raw * 3.3 * 2 / 4095 ≈ raw * 6600 / 4095 mV
                let mv = (v_raw * 6600) / 4095;
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "Voltage:  {} mV (raw {})\r\n", mv, v_raw);
                self.push(w.written());

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "Current:  raw {} (ch10)\r\n", i_raw);
                self.push(w.written());

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "NTC temp: raw {} (ch8)\r\n", ntc_raw);
                self.push(w.written());

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "USB chg:  raw {} (ch3)\r\n", usb_raw);
                self.push(w.written());
            }
            "reset" => {
                self.push(b"Resetting... (SYSRESETREQ)\r\n");
                power::delay_ms(100);
                cortex_m::peripheral::SCB::sys_reset();
            }
            "crc" => {
                // CRC-32 of memory range: crc <addr> <len_bytes>
                let addr = parts.next().and_then(|s| parse_num(s));
                let len = parts.next().and_then(|s| parse_num(s));
                match (addr, len) {
                    (Some(a), Some(n)) => {
                        if n > 1024 * 1024 {
                            self.push(b"ERR: max 1MB\r\n");
                        } else {
                            let slice = unsafe {
                                core::slice::from_raw_parts(a as *const u8, n as usize)
                            };
                            let crc = upload::crc32(slice);
                            let mut tmp = [0u8; 48];
                            let mut w = BufWriter::new(&mut tmp);
                            let _ = write!(w, "CRC32 0x{:08X}..+{}: 0x{:08X}\r\n", a, n, crc);
                            self.push(w.written());
                        }
                    }
                    _ => self.push(b"Usage: crc <addr> <len>\r\n"),
                }
            }
            "fill" => {
                // Fill memory with word pattern: fill <addr> <count_words> <value>
                let addr = parts.next().and_then(|s| parse_num(s));
                let count = parts.next().and_then(|s| parse_num(s));
                let val = parts.next().and_then(|s| parse_num(s));
                match (addr, count, val) {
                    (Some(a), Some(n), Some(v)) if a & 3 == 0 => {
                        if n > 256 * 1024 {
                            self.push(b"ERR: max 256K words\r\n");
                        } else {
                            for i in 0..n {
                                unsafe {
                                    core::ptr::write_volatile(
                                        a.wrapping_add(i * 4) as *mut u32,
                                        v,
                                    );
                                }
                            }
                            let mut tmp = [0u8; 48];
                            let mut w = BufWriter::new(&mut tmp);
                            let _ = write!(w, "OK filled {} words @ 0x{:08X}\r\n", n, a);
                            self.push(w.written());
                        }
                    }
                    (Some(_), Some(_), Some(_)) => {
                        self.push(b"ERR: addr must be 4-byte aligned\r\n");
                    }
                    _ => self.push(b"Usage: fill <addr> <count> <value>\r\n"),
                }
            }

            "motor" => {
                match (parts.next(), parts.next()) {
                    (Some("pwm"), Some(port_str)) => {
                        // motor pwm <port> <duty>
                        let port_ch = port_str.as_bytes().first().copied().unwrap_or(0);
                        if let Some(idx) = motor::port_index(port_ch) {
                            if let Some(duty_str) = parts.next() {
                                let negative = duty_str.starts_with('-');
                                let num_str = if negative { &duty_str[1..] } else { duty_str };
                                if let Some(v) = parse_u32(num_str) {
                                    let duty = if negative { -(v as i32) } else { v as i32 };
                                    if duty >= -100 && duty <= 100 {
                                        motor::set(idx as u32, duty);
                                        let mut tmp = [0u8; 64];
                                        let mut w = BufWriter::new(&mut tmp);
                                        let _ = write!(w, "OK motor {} pwm {}\r\n", port_str, duty);
                                        self.push(w.written());
                                    } else {
                                        self.push(b"ERR: duty must be -100..100\r\n");
                                    }
                                } else {
                                    self.push(b"ERR: invalid duty value\r\n");
                                }
                            } else {
                                self.push(b"Usage: motor pwm <a-f> <-100..100>\r\n");
                            }
                        } else {
                            self.push(b"ERR: port must be a-f\r\n");
                        }
                    }
                    (Some(port_str), Some(arg)) => {
                        let port_ch = port_str.as_bytes().first().copied().unwrap_or(0);
                        if let Some(idx) = motor::port_index(port_ch) {
                            if arg == "brake" {
                                motor::brake(idx as u32);
                                let mut tmp = [0u8; 32];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "motor {} brake\r\n", port_str);
                                self.push(w.written());
                            } else if arg == "diag" {
                                let (psc, arr, ccr1, ccr2, ccer, cr1, m1, m2) =
                                    motor::diag(idx as u32);
                                let mut tmp = [0u8; 192];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "motor {} diag:\r\n", port_str);
                                let _ = write!(w, "  PSC={} ARR={} CR1=0x{:04X}\r\n", psc, arr, cr1);
                                let _ = write!(w, "  CCR1={} CCR2={} CCER=0x{:04X}\r\n", ccr1, ccr2, ccer);
                                let _ = write!(w, "  pin1_mode={} pin2_mode={}\r\n", m1, m2);
                                let _ = write!(w, "  (mode: 0=in 1=out 2=AF 3=analog)\r\n");
                                self.push(w.written());
                            } else if arg == "ramp" {
                                // Ramp test: slow to fast with 1.5s per step, with diag
                                self.push(b"Ramp motor ");
                                self.push(&[port_ch]);
                                self.push(b": 5..100% fwd, 1.5s each step\r\n");
                                let steps: [i32; 12] = [5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100];
                                for &s in &steps {
                                    motor::set(idx as u32, s);
                                    let (psc, arr, ccr1, ccr2, ccer, cr1, m1, m2) =
                                        motor::diag(idx as u32);
                                    let mut tmp = [0u8; 96];
                                    let mut w = BufWriter::new(&mut tmp);
                                    let _ = write!(w, "  {:>3}%  CCR1={:<4} CCR2={:<4} CCER=0x{:04X} m1={} m2={}\r\n",
                                        s, ccr1, ccr2, ccer, m1, m2);
                                    self.push(w.written());
                                    crate::power::delay_ms(1500);
                                }
                                // Reverse ramp at 50% and 100%
                                self.push(b"  reverse:\r\n");
                                for &s in &[-50i32, -100] {
                                    motor::set(idx as u32, s);
                                    let (_, _, ccr1, ccr2, ccer, _, m1, m2) =
                                        motor::diag(idx as u32);
                                    let mut tmp = [0u8; 96];
                                    let mut w = BufWriter::new(&mut tmp);
                                    let _ = write!(w, "  {:>4}%  CCR1={:<4} CCR2={:<4} CCER=0x{:04X} m1={} m2={}\r\n",
                                        s, ccr1, ccr2, ccer, m1, m2);
                                    self.push(w.written());
                                    crate::power::delay_ms(1500);
                                }
                                motor::brake(idx as u32);
                                self.push(b"  brake. done.\r\n");
                            } else if arg == "trial" {
                                // Systematic PWM trial: 14 stages, 3s each
                                self.push(b"Motor trial: 14 stages, 3s each on port ");
                                self.push(&[port_ch]);
                                self.push(b"\r\nWatch the motor and note which stages spin!\r\n");
                                // Flush output before blocking trial
                                // Use a static cell for the shell pointer
                                static mut TRIAL_SHELL: Option<*mut Shell> = None;
                                unsafe { TRIAL_SHELL = Some(self as *mut Shell); }
                                fn trial_print(data: &[u8]) {
                                    unsafe {
                                        if let Some(ptr) = TRIAL_SHELL {
                                            (*ptr).push(data);
                                        }
                                    }
                                }
                                motor::trial(idx as u32, 3000, trial_print);
                            } else if arg == "pos" {
                                // Read motor position from LUMP sensor data.
                                // Check sensor_poll first, then motor_poll's motor_state.
                                let src = if sensor.is_motor() && sensor.port_idx == idx as u8 {
                                    Some(sensor)
                                } else if motor.is_motor() && motor.port_idx == idx as u8 {
                                    Some(motor)
                                } else {
                                    None
                                };
                                if let Some(st) = src {
                                    let mut tmp = [0u8; 192];
                                    let mut w = BufWriter::new(&mut tmp);
                                    let _ = write!(w, "{} mode={} len={}\r\n",
                                        st.type_name(), st.mode, st.data_len);
                                    self.push(w.written());
                                    // Hex dump of raw data
                                    let mut w2 = BufWriter::new(&mut tmp);
                                    let _ = write!(w2, "  raw:");
                                    for i in 0..st.data_len.min(16) {
                                        let _ = write!(w2, " {:02X}", st.data[i]);
                                    }
                                    let _ = write!(w2, "\r\n");
                                    self.push(w2.written());
                                    // Interpret based on mode
                                    if st.mode == sensor::MODE_MOTOR_POS || st.mode == sensor::MODE_ABS_MOTOR_POS {
                                        let pos = st.motor_pos_degrees();
                                        let mut w3 = BufWriter::new(&mut tmp);
                                        let _ = write!(w3, "  pos={}deg\r\n", pos);
                                        self.push(w3.written());
                                    } else if st.mode == sensor::MODE_ABS_MOTOR_CALIB {
                                        let (pos, apos, speed) = st.motor_calib_data();
                                        let mut w3 = BufWriter::new(&mut tmp);
                                        let _ = write!(w3, "  pos={}deg apos={}ddeg spd={}%\r\n", pos, apos, speed);
                                        self.push(w3.written());
                                    } else {
                                        let mut w3 = BufWriter::new(&mut tmp);
                                        let _ = write!(w3, "  (use 'sensor mode 2' for position)\r\n");
                                        self.push(w3.written());
                                    }
                                } else {
                                    self.push(b"ERR: no motor synced on port ");
                                    self.push(&[port_ch]);
                                    self.push(b"\r\n");
                                }
                            } else if arg == "pid" {
                                // PID position hold: motor <port> pid [target_deg]
                                // Requires 'sensor <port>' first for LUMP encoder data.
                                // Runs blocking in USB ISR context — manually drains
                                // LUMP ring buffer + sends keepalives since sensor_poll
                                // is preempted (lower priority).
                                if !sensor.is_motor() {
                                    self.push(b"ERR: no motor synced (run 'sensor ");
                                    self.push(&[port_ch]);
                                    self.push(b"' first)\r\n");
                                } else if sensor.mode != sensor::MODE_MOTOR_POS
                                       && sensor.mode != sensor::MODE_ABS_MOTOR_POS
                                       && sensor.mode != sensor::MODE_ABS_MOTOR_CALIB {
                                    self.push(b"ERR: sensor not in POS mode (run 'sensor mode 2')\r\n");
                                } else {
                                    let target = parts.next()
                                        .and_then(|s| {
                                            let neg = s.starts_with('-');
                                            let num = if neg { &s[1..] } else { s };
                                            parse_u32(num).map(|v| if neg { -(v as i32) } else { v as i32 })
                                        })
                                        .unwrap_or(0);
                                    let mut tmp = [0u8; 80];
                                    let mut w = BufWriter::new(&mut tmp);
                                    let _ = write!(w, "PID hold {}deg on motor {} (async)\r\n", target, port_str);
                                    self.push(w.written());
                                    self.pid_request = Some((idx as u8, target));
                                }
                            } else {
                                // parse speed
                                let negative = arg.starts_with('-');
                                let num_str = if negative { &arg[1..] } else { arg };
                                if let Some(v) = parse_u32(num_str) {
                                    let speed = if negative { -(v as i32) } else { v as i32 };
                                    motor::set(idx as u32, speed);
                                    let mut tmp = [0u8; 32];
                                    let mut w = BufWriter::new(&mut tmp);
                                    let _ = write!(w, "motor {} = {}\r\n", port_str, speed);
                                    self.push(w.written());
                                } else {
                                    self.push(b"ERR: invalid speed\r\n");
                                }
                            }
                        } else {
                            self.push(b"ERR: port must be a-f\r\n");
                        }
                    }
                    _ => self.push(b"Usage: motor <a-f> <speed|-100..100|brake|diag|trial|pos|pid|ramp>\r\n"),
                }
            }

            "servo" => {
                // Pybricks-style servo control: observer + PID + feedforward.
                // servo <port> hold <deg>  — position hold with full control stack
                // servo <port> diag        — show observer state
                // servo <port> stop        — stop control, coast motor
                match parts.next() {
                    Some(port_str) => {
                        let port_ch = port_str.as_bytes().first().copied().unwrap_or(0);
                        if let Some(idx) = motor::port_index(port_ch) {
                            match parts.next() {
                                Some("hold") => {
                                    if !sensor.is_motor() {
                                        self.push(b"ERR: no motor synced (run 'sensor ");
                                        self.push(&[port_ch]);
                                        self.push(b"' first)\r\n");
                                    } else if sensor.mode != sensor::MODE_MOTOR_POS
                                           && sensor.mode != sensor::MODE_ABS_MOTOR_POS
                                           && sensor.mode != sensor::MODE_ABS_MOTOR_CALIB {
                                        self.push(b"ERR: sensor not in POS mode\r\n");
                                    } else {
                                        let target = parts.next()
                                            .and_then(|s| {
                                                let neg = s.starts_with('-');
                                                let num = if neg { &s[1..] } else { s };
                                                parse_u32(num).map(|v| if neg { -(v as i32) } else { v as i32 })
                                            })
                                            .unwrap_or(0);
                                        let mut tmp = [0u8; 80];
                                        let mut w = BufWriter::new(&mut tmp);
                                        let _ = write!(w, "Servo hold {}deg on port {} (100Hz)\r\n",
                                            target, port_str);
                                        self.push(w.written());
                                        self.servo_request = Some((idx as u8, target));
                                    }
                                }
                                Some("stop") => {
                                    motor::brake(idx as u32);
                                    self.push(b"Servo stopped, motor braked\r\n");
                                }
                                _ => {
                                    self.push(b"Usage: servo <a-f> hold <deg>\r\n");
                                    self.push(b"       servo <a-f> stop\r\n");
                                }
                            }
                        } else {
                            self.push(b"ERR: port must be a-f\r\n");
                        }
                    }
                    _ => {
                        self.push(b"Usage: servo <a-f> hold <deg>\r\n");
                    }
                }
            }

            "stune" => {
                // Show or set servo (Pybricks-style) gains.
                // stune         -> show all
                // stune kp 20000 -> set Kp
                // stune reset   -> revert to defaults
                if self.servo_config.is_none() {
                    self.servo_config = Some(servo::ControlSettings::technic_m_angular());
                }
                match parts.next() {
                    None => {
                        let c = self.servo_config.unwrap();
                        let mut t = [0u8; 96];
                        let mut w = BufWriter::new(&mut t);
                        let _ = write!(w, "kp={} ki={} kd={} obs={}\r\n", c.pid_kp, c.pid_ki, c.pid_kd, c.obs_gain);
                        self.push(w.written());
                        let mut w2 = BufWriter::new(&mut t);
                        let _ = write!(w2, "icm={} amax={} ptol={}\r\n",
                            c.integral_change_max, c.actuation_max, c.position_tolerance);
                        self.push(w2.written());
                    }
                    Some("reset") => {
                        self.servo_config = None;
                        self.push(b"Servo config reset to defaults\r\n");
                    }
                    Some(param) => {
                        if let Some(val_s) = parts.next() {
                            let neg = val_s.starts_with('-');
                            let num_s = if neg { &val_s[1..] } else { val_s };
                            if let Some(v) = parse_u32(num_s) {
                                let val = if neg { -(v as i32) } else { v as i32 };
                                let cfg = self.servo_config.as_mut().unwrap();
                                match param {
                                    "kp" => { cfg.pid_kp = val; }
                                    "ki" => { cfg.pid_ki = val; }
                                    "kd" => { cfg.pid_kd = val; }
                                    "icm" => { cfg.integral_change_max = val; }
                                    "amax" => { cfg.actuation_max = val; }
                                    "ptol" => { cfg.position_tolerance = val; }
                                    "obs" => { cfg.obs_gain = val; }
                                    _ => { self.push(b"ERR: kp|ki|kd|icm|amax|ptol|obs|reset\r\n"); return; }
                                }
                                let mut t = [0u8; 48];
                                let mut w = BufWriter::new(&mut t);
                                let _ = write!(w, "{}={}\r\n", param, val);
                                self.push(w.written());
                            } else {
                                self.push(b"ERR: bad number\r\n");
                            }
                        } else {
                            self.push(b"Usage: stune <kp|ki|kd|icm|amax|ptol> <val>\r\n");
                        }
                    }
                }
            }

            "ports" => {
                // Show status of all 6 ports from PORT_STATES
                for i in 0..6usize {
                    let st = sensor::port_state(i);
                    let port_ch = (b'A' + i as u8) as char;
                    let mut tmp = [0u8; 128];
                    let mut w = BufWriter::new(&mut tmp);
                    match st.status {
                        sensor::Status::None => {
                            let _ = write!(w, "  {} --\r\n", port_ch);
                        }
                        sensor::Status::Syncing => {
                            let _ = write!(w, "  {} syncing...\r\n", port_ch);
                        }
                        sensor::Status::Error => {
                            let _ = write!(w, "  {} error step={}\r\n", port_ch, st.debug_step);
                        }
                        sensor::Status::Data => {
                            let _ = write!(w, "  {} {} type={} mode={}", port_ch,
                                st.type_name(), st.type_id, st.mode);
                            if st.is_ultrasonic() && st.data_len >= 2 {
                                let d = st.distance_mm();
                                if d < 0 || d >= 2000 {
                                    let _ = write!(w, " dist=---");
                                } else {
                                    let _ = write!(w, " dist={}mm", d);
                                }
                                let _ = write!(w, " [{:02X} {:02X}] s{} dl{} p{} q{} dm{}",
                                    st.data[0], st.data[1],
                                    st.debug_step, st.data_len, st.diag_polls,
                                    st.data_seq, st.diag_data_msgs);
                            } else if st.mode == sensor::MODE_RGB_I && st.data_len >= 8 {
                                let (r, g, b_val, intensity) = st.rgbi();
                                let _ = write!(w, " R={} G={} B={} I={}", r, g, b_val, intensity);
                            } else if st.is_motor() && st.data_len >= 4 {
                                let _ = write!(w, " pos={}deg", st.motor_pos_degrees());
                            } else if st.data_len >= 1 {
                                let _ = write!(w, " val={}", st.value_i8());
                            }
                            let _ = write!(w, "\r\n");
                        }
                    }
                    self.push(w.written());
                }
            }

            "sensor" => {
                match parts.next() {
                    Some("raw") => {
                        // Raw byte sniff: power-cycle, listen, dump hex
                        if let Some(p) = parts.next() {
                            let ch = p.as_bytes().first().copied().unwrap_or(0);
                            if let Some(idx) = sensor::port_index(ch) {
                                let sp = sensor::PORTS[idx];
                                let baud_str = parts.next().unwrap_or("115200");
                                let baud = parse_u32(baud_str).unwrap_or(115200);
                                let mut tmp = [0u8; 64];
                                let mut out = [0u8; 64];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Sniff port {} @{} for 2s...\r\n", p, baud);
                                self.push(w.written());
                                let n = unsafe { sensor::raw_sniff(sp, baud, 2000, &mut out) };
                                let mut w2 = BufWriter::new(&mut tmp);
                                let _ = write!(w2, "Got {} bytes:", n);
                                self.push(w2.written());
                                // Print hex bytes in groups of 16
                                for i in 0..n {
                                    if i % 16 == 0 {
                                        self.push(b"\r\n ");
                                    }
                                    let mut hx = [0u8; 4];
                                    let mut hw = BufWriter::new(&mut hx);
                                    let _ = write!(hw, " {:02X}", out[i]);
                                    self.push(hw.written());
                                }
                                self.push(b"\r\n");
                                return;
                            }
                        }
                        self.push(b"Usage: sensor raw <a-f> [baud]\r\n");
                    }
                    Some("diag") => {
                        // Dump GPIO state of a sensor port for debugging
                        if let Some(p) = parts.next() {
                            let ch = p.as_bytes().first().copied().unwrap_or(0);
                            if let Some(idx) = sensor::port_index(ch) {
                                let sp = sensor::PORTS[idx];
                                let mut tmp = [0u8; 256];
                                let mut w = BufWriter::new(&mut tmp);
                                unsafe {
                                    use crate::reg::reg_read;
                                    let buf_moder = reg_read(sp.buf_port, crate::pins::MODER);
                                    let buf_odr = reg_read(sp.buf_port, crate::pins::ODR);
                                    let buf_idr = reg_read(sp.buf_port, crate::pins::IDR);
                                    let tx_moder = reg_read(sp.tx_port, crate::pins::MODER);
                                    let rx_moder = reg_read(sp.rx_port, crate::pins::MODER);
                                    let tx_idr = reg_read(sp.tx_port, crate::pins::IDR);
                                    let rx_idr = reg_read(sp.rx_port, crate::pins::IDR);
                                    let uart_sr = reg_read(sp.uart_base, 0x00);
                                    let uart_cr1 = reg_read(sp.uart_base, 0x0C);
                                    let uart_brr = reg_read(sp.uart_base, 0x08);
                                    let _ = write!(w, "Port {} diag:\r\n", p);
                                    self.push(w.written());
                                    let mut w2 = BufWriter::new(&mut tmp);
                                    let _ = write!(w2, "  BUF pin{} MODER=0x{:08X} ODR=0x{:08X} IDR=0x{:08X}\r\n",
                                        sp.buf_pin, buf_moder, buf_odr, buf_idr);
                                    self.push(w2.written());
                                    let mut w3 = BufWriter::new(&mut tmp);
                                    let _ = write!(w3, "  TX pin{} MODER=0x{:08X} IDR=0x{:08X}\r\n",
                                        sp.tx_pin, tx_moder, tx_idr);
                                    self.push(w3.written());
                                    let mut w4 = BufWriter::new(&mut tmp);
                                    let _ = write!(w4, "  RX pin{} MODER=0x{:08X} IDR=0x{:08X}\r\n",
                                        sp.rx_pin, rx_moder, rx_idr);
                                    self.push(w4.written());
                                    let mut w5 = BufWriter::new(&mut tmp);
                                    let _ = write!(w5, "  UART SR=0x{:08X} CR1=0x{:08X} BRR=0x{:08X}\r\n",
                                        uart_sr, uart_cr1, uart_brr);
                                    self.push(w5.written());
                                    // ISR counters
                                    let mut w6 = BufWriter::new(&mut tmp);
                                    let _ = write!(w6, "  ISR rx={} ore={}\r\n",
                                        sensor::isr_rx_count(idx), sensor::isr_ore_count(idx));
                                    self.push(w6.written());
                                    // Ring buffer state
                                    let mut w7 = BufWriter::new(&mut tmp);
                                    let _ = write!(w7, "  RingBuf avail={}\r\n",
                                        sensor::rx_available_pub(idx));
                                    self.push(w7.written());
                                }
                                return;
                            }
                        }
                        self.push(b"Usage: sensor diag <a-f>\r\n");
                    }
                    Some("dump") => {
                        // Comprehensive sensor data dump for debugging
                        if let Some(p) = parts.next() {
                            let ch = p.as_bytes().first().copied().unwrap_or(0);
                            if let Some(idx) = sensor::port_index(ch) {
                                let st = sensor::port_state(idx);
                                let mut tmp = [0u8; 160];

                                // Line 1: identity + status
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "=== Port {} dump ===\r\n", (b'A' + idx as u8) as char);
                                self.push(w.written());

                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  status={} type={} ({}) mode={}\r\n",
                                    match st.status {
                                        sensor::Status::None => "None",
                                        sensor::Status::Syncing => "Syncing",
                                        sensor::Status::Data => "Data",
                                        sensor::Status::Error => "Error",
                                    },
                                    st.type_id, st.type_name(), st.mode);
                                self.push(w.written());

                                // Line 2: debug info
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  debug_step={} debug_byte=0x{:02X} first_hdr=0x{:02X} ext_mode={} caps=0x{:02X}\r\n",
                                    st.debug_step, st.debug_byte, st.diag_first_hdr, st.ext_mode, st.capabilities);
                                self.push(w.written());

                                // Line 3: data counters
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  polls={} data_msgs={} sys={} bad_sz={} bad_chk={}\r\n",
                                    st.diag_polls, st.diag_data_msgs, st.diag_sys,
                                    st.diag_bad_size, st.diag_bad_chk);
                                self.push(w.written());

                                // Line 4: data state
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  data_len={} data_seq={} data_received={}\r\n",
                                    st.data_len, st.data_seq, st.data_received);
                                self.push(w.written());

                                // Line 5: ISR counters
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  ISR rx={} ore={}\r\n",
                                    sensor::isr_rx_count(idx), sensor::isr_ore_count(idx));
                                self.push(w.written());

                                // Line 6: ring buffer state
                                let avail = sensor::rx_available_pub(idx);
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  ring_avail={}\r\n", avail);
                                self.push(w.written());

                                // Line 7: full data buffer (up to 16 bytes)
                                let show = if st.data_len > 16 { 16 } else { st.data_len };
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  data[0..{}]:", st.data_len);
                                for i in 0..show {
                                    let _ = write!(w, " {:02X}", st.data[i]);
                                }
                                let _ = write!(w, "\r\n");
                                self.push(w.written());

                                // Line 8: debug_bytes from handshake
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  hs_bytes[{}]:", st.debug_bytes_len);
                                for i in 0..st.debug_bytes_len as usize {
                                    let _ = write!(w, " {:02X}", st.debug_bytes[i]);
                                }
                                let _ = write!(w, "\r\n");
                                self.push(w.written());

                                // Line 9: last 8 message headers seen
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  last_hdrs[{}]:", st.diag_hdr_idx);
                                for i in 0..8u8 {
                                    let _ = write!(w, " {:02X}", st.diag_last_hdrs[i as usize]);
                                }
                                let _ = write!(w, "  cmd_msgs={}\r\n", st.diag_cmd_msgs);
                                self.push(w.written());

                                // Line 10: last raw DATA message
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  last_raw[{}]:", st.diag_last_raw_len);
                                for i in 0..st.diag_last_raw_len as usize {
                                    let _ = write!(w, " {:02X}", st.diag_last_raw[i]);
                                }
                                let _ = write!(w, "\r\n");
                                self.push(w.written());

                                // Line 9+: ring buffer peek (first 48 bytes waiting)
                                let peek_n = if avail > 48 { 48 } else { avail };
                                if peek_n > 0 {
                                    self.push(b"  ring peek:");
                                    for i in 0..peek_n {
                                        if i % 16 == 0 {
                                            self.push(b"\r\n   ");
                                        }
                                        let b = sensor::rx_peek_at_pub(idx, i);
                                        let mut hx = [0u8; 4];
                                        let mut hw = BufWriter::new(&mut hx);
                                        let _ = write!(hw, " {:02X}", b);
                                        self.push(hw.written());
                                    }
                                    self.push(b"\r\n");
                                }

                                return;
                            }
                        }
                        self.push(b"Usage: sensor dump <a-f>\r\n");
                    }
                    Some("stop") => {
                        // Request sensor_poll to stop by setting status to None
                        // Also pause auto-detect so port_detect doesn't re-spawn.
                        self.push(b"Sensor stopped\r\n");
                        self.sensor_request = Some(0xFF);
                        task_state::pause_auto_detect();
                    }
                    Some("mode") => {
                        if let Some(m) = parts.next().and_then(|s| parse_u32(s)) {
                            if m <= 9 {
                                let mut tmp = [0u8; 40];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Mode switch to {} (next poll)\r\n", m);
                                self.push(w.written());
                                // Request mode switch: encode as 0x80 | mode
                                self.sensor_request = Some(0x80 | m as u8);
                            } else {
                                self.push(b"ERR: mode 0-9\r\n");
                            }
                        } else {
                            self.push(b"Usage: sensor mode <0-9>\r\n");
                        }
                    }
                    Some(port_str) => {
                        let ch = port_str.as_bytes().first().copied().unwrap_or(0);
                        if let Some(idx) = sensor::port_index(ch) {
                            // If this port is already active with data, show it
                            if sensor.status == sensor::Status::Data && sensor.port_idx == idx as u8 {
                                let mut tmp = [0u8; 128];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "{} (type {}) port {} mode {}\r\n",
                                    sensor.type_name(), sensor.type_id,
                                    (b'A' + sensor.port_idx) as char, sensor.mode);
                                self.push(w.written());
                                let mut w2 = BufWriter::new(&mut tmp);
                                if sensor.mode == sensor::MODE_RGB_I {
                                    let (r, g, b, i) = sensor.rgbi();
                                    let _ = write!(w2, "  R={} G={} B={} I={}\r\n", r, g, b, i);
                                } else if sensor.mode == sensor::MODE_HSV {
                                    let (h, s, v) = sensor.hsv();
                                    let _ = write!(w2, "  H={} S={} V={}\r\n", h, s, v);
                                } else if sensor.is_ultrasonic() && sensor.data_len >= 2 {
                                    let d = sensor.distance_mm();
                                    let _ = write!(w2, "  dist={}mm\r\n", d);
                                } else {
                                    let v = sensor.value_i8();
                                    let _ = write!(w2, "  value={}\r\n", v);
                                }
                                self.push(w2.written());
                            } else if sensor.status == sensor::Status::Syncing && sensor.port_idx == idx as u8 {
                                let mut tmp = [0u8; 48];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Syncing port {}...\r\n",
                                    (b'A' + idx as u8) as char);
                                self.push(w.written());
                            } else {
                                // Not active on this port — probe it
                                // Pause auto-detect if switching away from port F
                                if idx != 5 {
                                    task_state::pause_auto_detect();
                                } else {
                                    task_state::resume_auto_detect();
                                }
                                // Stop current sensor_poll first (0xFF), then probe
                                // The USB ISR processes stop first, then spawns new probe.
                                self.sensor_stop_then_probe = Some(idx as u8);
                                self.sensor_request = Some(0xFF);
                                let mut tmp = [0u8; 32];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Probing port {}...\r\n", port_str);
                                self.push(w.written());
                            }
                        } else {
                            self.push(b"ERR: port a-f\r\n");
                        }
                    }
                    None => {
                        // No port given — show status or hint
                        if sensor.status == sensor::Status::Data {
                            let mut tmp = [0u8; 128];
                            let mut w = BufWriter::new(&mut tmp);
                            let _ = write!(w, "{} (type {}) port {} mode {}\r\n",
                                sensor.type_name(), sensor.type_id,
                                (b'A' + sensor.port_idx) as char, sensor.mode);
                            self.push(w.written());
                            let mut w2 = BufWriter::new(&mut tmp);
                            if sensor.mode == sensor::MODE_RGB_I {
                                let (r, g, b, i) = sensor.rgbi();
                                let _ = write!(w2, "  R={} G={} B={} I={}\r\n", r, g, b, i);
                            } else if sensor.mode == sensor::MODE_HSV {
                                let (h, s, v) = sensor.hsv();
                                let _ = write!(w2, "  H={} S={} V={}\r\n", h, s, v);
                            } else if sensor.is_ultrasonic() && sensor.data_len >= 2 {
                                let d = sensor.distance_mm();
                                if d < 0 || d >= 2000 {
                                    let _ = write!(w2, "  dist=out-of-range (raw={})\r\n", d);
                                } else {
                                    let _ = write!(w2, "  dist={}mm\r\n", d);
                                }
                            } else {
                                let v = sensor.value_i8();
                                let _ = write!(w2, "  value={}\r\n", v);
                            }
                            self.push(w2.written());
                            // LUMP poll diagnostics
                            let mut w3 = BufWriter::new(&mut tmp);
                            let _ = write!(w3, "  poll={} sys={} data={} bchk={} bsz={} hdr=0x{:02X}\r\n",
                                sensor.diag_polls, sensor.diag_sys, sensor.diag_data_msgs,
                                sensor.diag_bad_chk, sensor.diag_bad_size, sensor.diag_first_hdr);
                            self.push(w3.written());
                            let mut w4 = BufWriter::new(&mut tmp);
                            let _ = write!(w4, "  isr_rx={} isr_ore={} baud={} seq={}\r\n",
                                sensor::isr_rx_count(sensor.port_idx as usize),
                                sensor::isr_ore_count(sensor.port_idx as usize),
                                sensor.new_baud, sensor.data_seq);
                            self.push(w4.written());
                            // Raw data bytes
                            let mut w5 = BufWriter::new(&mut tmp);
                            let dl = sensor.data_len;
                            let show = dl.min(8);
                            let _ = write!(w5, "  data_len={} raw=[", dl);
                            for i in 0..show {
                                if i > 0 { let _ = write!(w5, ","); }
                                let _ = write!(w5, "{:02X}", sensor.data[i]);
                            }
                            let _ = write!(w5, "]\r\n");
                            self.push(w5.written());
                        } else if sensor.status == sensor::Status::Syncing {
                            let mut tmp = [0u8; 48];
                            let mut w = BufWriter::new(&mut tmp);
                            let _ = write!(w, "Syncing port {}...\r\n",
                                (b'A' + sensor.port_idx) as char);
                            self.push(w.written());
                        } else if sensor.status == sensor::Status::Error {
                            let mut tmp = [0u8; 128];
                            let mut w = BufWriter::new(&mut tmp);
                            let _ = write!(w, "Sensor error on port {} step={} byte=0x{:02X}\r\n",
                                (b'A' + sensor.port_idx) as char,
                                sensor.debug_step, sensor.debug_byte);
                            self.push(w.written());
                        } else {
                            self.push(b"Usage: sensor <a-f>\r\n");
                        }
                        return;
                    }
                }
            }

            "light" => {
                // Set color sensor LED brightness: light <r> <g> <b> (0-100 each)
                let r = parts.next().and_then(|s| parse_u32(s)).unwrap_or(0);
                let g = parts.next().and_then(|s| parse_u32(s)).unwrap_or(0);
                let b = parts.next().and_then(|s| parse_u32(s)).unwrap_or(0);
                let r = r.min(100) as u8;
                let g = g.min(100) as u8;
                let b = b.min(100) as u8;
                sensor::queue_sensor_light(r, g, b);
                let mut tmp = [0u8; 48];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "Light: R={} G={} B={}\r\n", r, g, b);
                self.push(w.written());
            }

            "pid" => {
                // Show or set PID gains interactively.
                // pid          -> show all
                // pid kp 400   -> set Kp
                // pid ki 5     -> set Ki
                // pid kd 3000  -> set Kd
                // pid sc 20    -> stiction_comp
                // pid dz 3     -> integral_deadzone
                // pid et 5     -> error_threshold
                // pid st 250   -> speed_threshold
                match parts.next() {
                    None => {
                        let p = self.pid_config;
                        let mut t = [0u8; 80];
                        let mut w = BufWriter::new(&mut t);
                        let _ = write!(w, "kp={} ki={} kd={}\r\n", p.kp, p.ki, p.kd);
                        self.push(w.written());
                        let mut w2 = BufWriter::new(&mut t);
                        let _ = write!(w2, "sc={} dz={} et={} st={}\r\n",
                            p.stiction_comp, p.integral_deadzone,
                            p.error_threshold, p.speed_threshold);
                        self.push(w2.written());
                        let mut w3 = BufWriter::new(&mut t);
                        let _ = write!(w3, "accel={} maxspd={} icm={} omax={}\r\n",
                            p.accel, p.max_speed, p.integral_change_max, p.out_max);
                        self.push(w3.written());
                    }
                    Some(param) => {
                        if let Some(val_s) = parts.next() {
                            let neg = val_s.starts_with('-');
                            let num_s = if neg { &val_s[1..] } else { val_s };
                            if let Some(v) = parse_u32(num_s) {
                                let val = if neg { -(v as i32) } else { v as i32 };
                                let mut t = [0u8; 48];
                                let mut w = BufWriter::new(&mut t);
                                match param {
                                    "kp" => { self.pid_config.kp = val; let _ = write!(w, "kp={}", val); }
                                    "ki" => { self.pid_config.ki = val; let _ = write!(w, "ki={}", val); }
                                    "kd" => { self.pid_config.kd = val; let _ = write!(w, "kd={}", val); }
                                    "sc" => { self.pid_config.stiction_comp = val; let _ = write!(w, "sc={}", val); }
                                    "dz" => { self.pid_config.integral_deadzone = val; let _ = write!(w, "dz={}", val); }
                                    "et" => { self.pid_config.error_threshold = val; let _ = write!(w, "et={}", val); }
                                    "st" => { self.pid_config.speed_threshold = val; let _ = write!(w, "st={}", val); }
                                    "accel" => { self.pid_config.accel = val; let _ = write!(w, "accel={}", val); }
                                    "maxspd" => { self.pid_config.max_speed = val; let _ = write!(w, "maxspd={}", val); }
                                    "icm" => { self.pid_config.integral_change_max = val; let _ = write!(w, "icm={}", val); }
                                    "omax" => { self.pid_config.out_max = val; self.pid_config.out_min = -val; let _ = write!(w, "omax={}", val); }
                                    _ => { self.push(b"ERR: kp|ki|kd|sc|dz|et|st|accel|maxspd|icm|omax\r\n"); return; }
                                }
                                self.push(w.written());
                                self.push(b"\r\n");
                            } else {
                                self.push(b"ERR: bad number\r\n");
                            }
                        } else {
                            self.push(b"Usage: pid <kp|ki|kd|sc|dz|et|st|accel|maxspd|icm|omax> <value>\r\n");
                        }
                    }
                }
            }

            "ps" => {
                self.push(b"=== RTIC Tasks ===\r\n");
                self.push(b"init (boot)\r\n");

                // init-spawned tasks
                self.push(b"  heartbeat        loop  pri=1 ");
                self.push(if task_state::is_active(task_state::HEARTBEAT) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"    send_banner    once  pri=1 ");
                self.push(if task_state::is_active(task_state::SEND_BANNER) { b"ACTIVE\r\n" } else { b"done\r\n" });

                self.push(b"  shell_tick       loop  pri=1 ");
                self.push(if task_state::is_active(task_state::SHELL_TICK) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"  button_poll      loop  pri=1 ");
                self.push(if task_state::is_active(task_state::BUTTON_POLL) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"  boot_beep        once  pri=1 ");
                self.push(if task_state::is_active(task_state::BOOT_BEEP) { b"ACTIVE\r\n" } else { b"done\r\n" });

                // USB ISR children
                self.push(b"usb_interrupt       ISR  pri=2 (OTG_FS)\r\n");

                self.push(b"  sensor_poll      async pri=1 ");
                if task_state::is_active(task_state::SENSOR_POLL) {
                    let mut tmp = [0u8; 32];
                    let mut w = BufWriter::new(&mut tmp);
                    let _ = write!(w, "ACTIVE port={}\r\n",
                        (b'A' + sensor.port_idx) as char);
                    self.push(w.written());
                } else {
                    self.push(b"-\r\n");
                }

                self.push(b"  run_demo         async pri=1 ");
                self.push(if task_state::is_active(task_state::RUN_DEMO) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"  beep_tone        async pri=1 ");
                self.push(if task_state::is_active(task_state::BEEP_TONE) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"  rtty_tx          async pri=1 ");
                self.push(if task_state::is_active(task_state::RTTY_TX) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"  test_all         async pri=1 ");
                self.push(if task_state::is_active(task_state::TEST_ALL) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"  reconnect_ser    async pri=1 ");
                self.push(if task_state::is_active(task_state::RECONNECT_SER) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"  servo_run        async pri=1 ");
                self.push(if task_state::is_active(task_state::SERVO_RUN) { b"ACTIVE\r\n" } else { b"-\r\n" });

                self.push(b"  motor_poll       async pri=1 ");
                if task_state::is_active(task_state::MOTOR_POLL) {
                    let mut tmp = [0u8; 48];
                    let mut w = BufWriter::new(&mut tmp);
                    let _ = write!(w, "ACTIVE port={} type={}\r\n",
                        (b'A' + motor.port_idx) as char, motor.type_id);
                    self.push(w.written());
                } else {
                    self.push(b"-\r\n");
                }

                self.push(b"  motor_detect     loop  pri=1 ACTIVE\r\n");

                // UART ISRs
                self.push(b"uart ISRs           ISR  pri=3 (sensor RXNE)\r\n");
                self.push(b"  A=UART7 B=UART4 C=UART8 D=UART5 E=UART10 F=UART9\r\n");
            }

            "ls" => {
                // Linker symbols
                extern "C" {
                    static __sidata: u8;
                    static __sdata: u8;
                    static __edata: u8;
                    static __sbss: u8;
                    static __ebss: u8;
                }

                let sidata = unsafe { &__sidata as *const u8 as u32 };
                let sdata = unsafe { &__sdata as *const u8 as u32 };
                let edata = unsafe { &__edata as *const u8 as u32 };
                let sbss = unsafe { &__sbss as *const u8 as u32 };
                let ebss = unsafe { &__ebss as *const u8 as u32 };
                let sp: u32;
                unsafe { core::arch::asm!("mov {}, sp", out(reg) sp) };

                let data_size = edata - sdata;
                let fw_end = sidata + data_size;
                let fw_size = fw_end - 0x0800_8000;
                let flash_free = 0x0810_0000 - fw_end;

                // .bss may extend into SRAM2 due to linker INSERT; clamp
                const SRAM1_END: u32 = 0x2004_0000;
                let bss_end = if ebss > SRAM1_END { SRAM1_END } else { ebss };
                let bss_size = bss_end - sbss;
                let stack_free = if sp > bss_end { sp - bss_end } else { 0 };

                let mut tmp = [0u8; 64];

                self.push(b"=== Flash (ROM) ===\r\n");

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  0x08000000  Bootloader    32 KB\r\n");
                self.push(w.written());

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  0x08008000  Firmware   {:5} B  ({} KB)\r\n",
                    fw_size, fw_size / 1024);
                self.push(w.written());

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  0x{:08X}  Free      {:5} KB\r\n",
                    fw_end, flash_free / 1024);
                self.push(w.written());

                self.push(b"=== SRAM1 (256 KB) ===\r\n");

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  0x{:08X}  .data     {:5} B\r\n", sdata, data_size);
                self.push(w.written());

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  0x{:08X}  .bss      {:5} B\r\n", sbss, bss_size);
                self.push(w.written());

                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "  0x{:08X}  Free+Stack {:4} KB  (SP=0x{:08X})\r\n",
                    bss_end, stack_free / 1024, sp);
                self.push(w.written());

                self.push(b"=== SRAM2 (64 KB) ===\r\n");
                self.push(b"  0x20040000  Apps/Upload   64 KB\r\n");
                self.push(b"  0x2004FFF0  DFU magic     16 B\r\n");

                self.push(b"=== External SPI Flash (W25Q256) ===\r\n");
                let jedec = ext_flash::read_jedec_id();
                if jedec == ext_flash::EXPECTED_JEDEC {
                    self.push(b"  0x00000000  W25Q256JV    32 MB  (SPI2)\r\n");
                } else {
                    let mut w = BufWriter::new(&mut tmp);
                    let _ = write!(w, "  JEDEC 0x{:06X} (not W25Q256?)\r\n", jedec);
                    self.push(w.written());
                }

                if self.last_upload_len > 0 {
                    let addr = upload::upload_buf_addr();
                    let mut w = BufWriter::new(&mut tmp);
                    let _ = write!(w, "Upload: {} B @ 0x{:08X}\r\n",
                        self.last_upload_len, addr);
                    self.push(w.written());
                }
            }

            "test_all" => {
                self.push(b"Starting test_all ballet...\r\n");
                self.test_all_request = true;
            }

            "version" => {
                self.push(b"spike-rtic 0.1.0\r\n");
                self.push(b"RTIC v2, Cortex-M4F, STM32F413VGT6\r\n");
                let mut tmp = [0u8; 48];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "API version: {}\r\n", spike_hub_api::API_VERSION);
                self.push(w.written());

                let uid0 = unsafe { core::ptr::read_volatile(0x1FFF_7A10 as *const u32) };
                let uid1 = unsafe { core::ptr::read_volatile(0x1FFF_7A14 as *const u32) };
                let uid2 = unsafe { core::ptr::read_volatile(0x1FFF_7A18 as *const u32) };
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "UID: {:08X}-{:08X}-{:08X}\r\n", uid0, uid1, uid2);
                self.push(w.written());
            }

            "imu" => {
                match parts.next() {
                    Some("init") | None => {
                        let who = imu::init();
                        let mut tmp = [0u8; 60];
                        let mut w = BufWriter::new(&mut tmp);
                        if who != 0 {
                            let _ = write!(w, "IMU OK: WHO_AM_I=0x{:02X} (LSM6DS3TR-C)\r\n", who);
                        } else {
                            let _ = write!(w, "IMU FAIL: no response on I2C2\r\n");
                        }
                        self.push(w.written());
                    }
                    Some("read") => {
                        if !imu::is_ready() {
                            let who = imu::init();
                            if who == 0 {
                                self.push(b"IMU not responding\r\n");
                                return;
                            }
                        }
                        let mut buf = [0u8; 12];
                        let n = imu::read(&mut buf);
                        if n == 12 {
                            let ax = i16::from_le_bytes([buf[0], buf[1]]);
                            let ay = i16::from_le_bytes([buf[2], buf[3]]);
                            let az = i16::from_le_bytes([buf[4], buf[5]]);
                            let gx = i16::from_le_bytes([buf[6], buf[7]]);
                            let gy = i16::from_le_bytes([buf[8], buf[9]]);
                            let gz = i16::from_le_bytes([buf[10], buf[11]]);
                            let mut tmp = [0u8; 80];
                            let mut w = BufWriter::new(&mut tmp);
                            let _ = write!(w, "Accel: X={} Y={} Z={}\r\n", ax, ay, az);
                            self.push(w.written());
                            let mut w = BufWriter::new(&mut tmp);
                            let _ = write!(w, "Gyro:  X={} Y={} Z={}\r\n", gx, gy, gz);
                            self.push(w.written());
                        } else {
                            self.push(b"IMU read failed\r\n");
                        }
                    }
                    _ => self.push(b"Usage: imu [init|read]\r\n"),
                }
            }

            "time" => {
                let t = self.ticks;
                let secs = t % 60;
                let mins = (t / 60) % 60;
                let hours = t / 3600;
                let mut tmp = [0u8; 40];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "{}h {:02}m {:02}s ({} ticks)\r\n",
                    hours, mins, secs, t);
                self.push(w.written());
            }

            "unixtime" => {
                // No RTC battery backup — uptime-based pseudo-timestamp.
                // The base epoch is arbitrary (firmware build, not wall-clock).
                // Useful for relative timing, not absolute date.
                let t = self.ticks;
                // Use a fixed base: 2026-01-01 00:00:00 UTC = 1767225600
                let pseudo_unix = 1_767_225_600u32.wrapping_add(t);
                let mut tmp = [0u8; 40];
                let mut w = BufWriter::new(&mut tmp);
                let _ = write!(w, "{} (base + {} s uptime)\r\n", pseudo_unix, t);
                self.push(w.written());
                self.push(b"Note: no RTC -- pseudo epoch from boot\r\n");
            }

            "spiflash" | "extflash" => {
                match parts.next() {
                    Some("id") | None => {
                        let jedec = ext_flash::read_jedec_id();
                        let mfr = (jedec >> 16) & 0xFF;
                        let dev = jedec & 0xFFFF;
                        let mut tmp = [0u8; 80];
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "JEDEC ID: 0x{:06X} (mfr=0x{:02X} dev=0x{:04X})\r\n",
                            jedec, mfr, dev);
                        self.push(w.written());
                        if jedec == ext_flash::EXPECTED_JEDEC {
                            self.push(b"  Winbond W25Q256JV (32 MB)\r\n");
                        } else if jedec == 0 || jedec == 0xFFFFFF {
                            self.push(b"  ERR: no flash detected (check SPI2)\r\n");
                        } else {
                            self.push(b"  Unknown flash chip\r\n");
                        }
                        let sr1 = ext_flash::read_sr1();
                        let sr2 = ext_flash::read_sr2();
                        let sr3 = ext_flash::read_sr3();
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "SR1=0x{:02X} SR2=0x{:02X} SR3=0x{:02X} busy={}\r\n",
                            sr1, sr2, sr3, ext_flash::is_busy());
                        self.push(w.written());
                        // Decode protection
                        if sr1 & 0x7C != 0 {
                            self.push(b"  WARN: BP bits set (write-protected)\r\n");
                            self.push(b"  Use 'spiflash unlock' to clear.\r\n");
                        } else {
                            self.push(b"  Protection: none (all sectors writable)\r\n");
                        }
                    }
                    Some("read") => {
                        let addr = parts.next().and_then(|s| parse_num(s));
                        let count = parts.next().and_then(|s| parse_num(s)).unwrap_or(64);
                        match addr {
                            Some(a) if a < ext_flash::FLASH_SIZE => {
                                let n = core::cmp::min(count, 256) as usize;
                                let mut buf = [0u8; 256];
                                ext_flash::read(a, &mut buf[..n]);
                                let mut tmp = [0u8; 16];
                                for i in 0..n {
                                    if i % 16 == 0 {
                                        let mut w = BufWriter::new(&mut tmp);
                                        let _ = write!(w, "{:08X}: ", a + i as u32);
                                        self.push(w.written());
                                    }
                                    let mut w = BufWriter::new(&mut tmp);
                                    let _ = write!(w, "{:02X} ", buf[i]);
                                    self.push(w.written());
                                    if i % 16 == 15 || i == n - 1 {
                                        self.push(b"\r\n");
                                    }
                                }
                            }
                            Some(_) => self.push(b"ERR: addr >= 32 MB\r\n"),
                            None => self.push(b"Usage: spiflash read <addr> [count]\r\n"),
                        }
                    }
                    Some("erase") => {
                        let addr = parts.next().and_then(|s| parse_num(s));
                        match addr {
                            Some(a) if a < ext_flash::FLASH_SIZE => {
                                let mut tmp = [0u8; 48];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Erasing 4KB sector @ 0x{:08X}...\r\n", a);
                                self.push(w.written());
                                ext_flash::sector_erase(a);
                                self.push(b"OK\r\n");
                            }
                            Some(_) => self.push(b"ERR: addr >= 32 MB\r\n"),
                            None => self.push(b"Usage: spiflash erase <addr>\r\n"),
                        }
                    }
                    Some("write") => {
                        let addr = parts.next().and_then(|s| parse_num(s));
                        let val = parts.next().and_then(|s| parse_num(s));
                        match (addr, val) {
                            (Some(a), Some(v)) if a < ext_flash::FLASH_SIZE => {
                                let bytes = [
                                    (v >> 24) as u8, (v >> 16) as u8,
                                    (v >> 8) as u8, v as u8,
                                ];
                                ext_flash::page_program(a, &bytes);
                                let mut tmp = [0u8; 48];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "OK wrote 4 bytes @ 0x{:08X}\r\n", a);
                                self.push(w.written());
                            }
                            _ => self.push(b"Usage: spiflash write <addr> <u32_value>\r\n"),
                        }
                    }
                    Some("store") => {
                        // Store upload buffer contents to SPI flash.
                        // Usage: spiflash store <flash_addr> [size]
                        // If size is omitted, uses last_upload_len.
                        let addr = parts.next().and_then(|s| parse_num(s));
                        let size = parts.next().and_then(|s| parse_num(s))
                            .unwrap_or(self.last_upload_len as u32);
                        match addr {
                            Some(a) if a < ext_flash::FLASH_SIZE && size > 0 => {
                                let sz = core::cmp::min(size as usize, upload::UPLOAD_BUF_SIZE);
                                let buf = upload::upload_buf();
                                // Show first 8 bytes of upload buffer (sanity check)
                                let mut tmp = [0u8; 96];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "RAM buf[0..8]: {:02X}{:02X}{:02X}{:02X} {:02X}{:02X}{:02X}{:02X}\r\n",
                                    buf[0], buf[1], buf[2], buf[3],
                                    buf[4], buf[5], buf[6], buf[7]);
                                self.push(w.written());
                                // Erase enough sectors to fit
                                let sectors = (sz + ext_flash::SECTOR_SIZE as usize - 1)
                                    / ext_flash::SECTOR_SIZE as usize;
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Erasing {} sectors @ 0x{:08X}...\r\n", sectors, a);
                                self.push(w.written());
                                for i in 0..sectors {
                                    ext_flash::sector_erase(a + (i as u32) * ext_flash::SECTOR_SIZE);
                                }
                                // Verify erase
                                let mut verify = [0u8; 4];
                                ext_flash::read(a, &mut verify);
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Post-erase: {:02X}{:02X}{:02X}{:02X}\r\n",
                                    verify[0], verify[1], verify[2], verify[3]);
                                self.push(w.written());
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Writing {} bytes ({} pages)...\r\n",
                                    sz, (sz + 255) / 256);
                                self.push(w.written());
                                ext_flash::write(a, &buf[..sz]);
                                // Verify first 8 bytes
                                let mut verify8 = [0u8; 8];
                                ext_flash::read(a, &mut verify8);
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Verify[0..8]: {:02X}{:02X}{:02X}{:02X} {:02X}{:02X}{:02X}{:02X}\r\n",
                                    verify8[0], verify8[1], verify8[2], verify8[3],
                                    verify8[4], verify8[5], verify8[6], verify8[7]);
                                self.push(w.written());
                            }
                            Some(_) => self.push(b"ERR: bad addr or size=0\r\n"),
                            None => self.push(b"Usage: spiflash store <flash_addr> [size]\r\n"),
                        }
                    }
                    Some("load") => {
                        // Load from SPI flash to upload buffer.
                        let addr = parts.next().and_then(|s| parse_num(s));
                        let size = parts.next().and_then(|s| parse_num(s));
                        match (addr, size) {
                            (Some(a), Some(sz)) if a < ext_flash::FLASH_SIZE && sz > 0 => {
                                let n = core::cmp::min(sz as usize, upload::UPLOAD_BUF_SIZE);
                                let buf = upload::upload_buf_mut();
                                ext_flash::read(a, &mut buf[..n]);
                                self.last_upload_len = n;
                                let crc = upload::crc32(&buf[..n]);
                                let ram_addr = upload::upload_buf_addr();
                                let mut tmp = [0u8; 80];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Loaded {} B -> 0x{:08X} CRC=0x{:08X}\r\n",
                                    n, ram_addr, crc);
                                self.push(w.written());
                                self.push(b"Use 'go' to execute.\r\n");
                            }
                            _ => self.push(b"Usage: spiflash load <flash_addr> <size>\r\n"),
                        }
                    }
                    Some("run") => {
                        // Load from SPI flash and immediately execute (sandboxed).
                        let addr = parts.next().and_then(|s| parse_num(s));
                        let size = parts.next().and_then(|s| parse_num(s));
                        match (addr, size) {
                            (Some(a), Some(sz)) if a < ext_flash::FLASH_SIZE && sz > 0 => {
                                if user_app_io::is_running() {
                                    self.push(b"ERR: demo already running\r\n");
                                    return;
                                }
                                let n = core::cmp::min(sz as usize, upload::UPLOAD_BUF_SIZE);
                                let buf = upload::upload_buf_mut();
                                ext_flash::read(a, &mut buf[..n]);
                                self.last_upload_len = n;
                                let ram_addr = upload::upload_buf_addr();
                                let mut tmp = [0u8; 80];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "Loaded {} B from flash 0x{:08X}\r\n", n, a);
                                self.push(w.written());
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "go 0x{:08X} [sandboxed]\r\n", ram_addr);
                                self.push(w.written());
                                user_app_io::request(ram_addr, true);
                            }
                            _ => self.push(b"Usage: spiflash run <flash_addr> <size>\r\n"),
                        }
                    }
                    Some("dir") => {
                        // Scan for stored binaries at known slots.
                        // Convention: demos stored at 64KB-aligned addresses.
                        // A slot is "occupied" if first 4 bytes != 0xFFFFFFFF.
                        self.push(b"Ext flash directory (64KB slots):\r\n");
                        let mut found = 0u32;
                        let mut tmp = [0u8; 160];
                        for slot in 0..512u32 {
                            let base = slot * ext_flash::BLOCK_SIZE;
                            let mut peek = [0xFFu8; 32];
                            ext_flash::read(base, &mut peek);
                            if peek[..4] != [0xFF, 0xFF, 0xFF, 0xFF] {
                                // Hex line
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "  [{}] 0x{:08X}: ", slot, base);
                                for i in 0..32 {
                                    let _ = write!(w, "{:02X}", peek[i]);
                                    if i == 3 || i == 7 || i == 15 { let _ = write!(w, " "); }
                                }
                                let _ = write!(w, "\r\n");
                                self.push(w.written());
                                // ASCII line with non-printable labels
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "        ASCII: ");
                                for &b in &peek {
                                    match b {
                                        0x00 => { let _ = write!(w, "[NUL]"); }
                                        0x07 => { let _ = write!(w, "[BEL]"); }
                                        0x08 => { let _ = write!(w, "[BS]"); }
                                        0x09 => { let _ = write!(w, "[TAB]"); }
                                        0x0A => { let _ = write!(w, "[LF]"); }
                                        0x0D => { let _ = write!(w, "[CR]"); }
                                        0x1B => { let _ = write!(w, "[ESC]"); }
                                        0xFF => { let _ = write!(w, "."); }
                                        0x20..=0x7E => { let _ = write!(w, "{}", b as char); }
                                        _ => { let _ = write!(w, "[{:02X}]", b); }
                                    }
                                }
                                let _ = write!(w, "\r\n");
                                self.push(w.written());
                                found += 1;
                            }
                        }
                        if found == 0 {
                            self.push(b"  (empty -- all 0xFF)\r\n");
                        }
                    }
                    Some("unlock") => {
                        self.push(b"Clearing flash write protection...\r\n");
                        ext_flash::unlock();
                        let sr1 = ext_flash::read_sr1();
                        let sr3 = ext_flash::read_sr3();
                        let mut tmp = [0u8; 64];
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "SR1=0x{:02X} SR3=0x{:02X}\r\n", sr1, sr3);
                        self.push(w.written());
                        if sr1 & 0x7C == 0 {
                            self.push(b"OK: all sectors unlocked.\r\n");
                        } else {
                            self.push(b"WARN: BP bits still set! (HW WP pin?)\r\n");
                        }
                    }
                    Some("test") => {
                        // Diagnostic: test write_enable WEL, then erase+write+read
                        self.push(b"--- WEL test ---\r\n");
                        let (wel_b, wel_a, sr1) = ext_flash::diag_write_enable();
                        let mut tmp = [0u8; 80];
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "WEL before={} after={} SR1=0x{:02X}\r\n",
                            wel_b, wel_a, sr1);
                        self.push(w.written());
                        if !wel_a {
                            self.push(b"FAIL: WEL not set! Check SPI wiring.\r\n");
                            return;
                        }
                        self.push(b"--- erase+write @ 0x00000000 (inline) ---\r\n");
                        let (erase_ok, write_ok, sr1_wel, readback) =
                            ext_flash::diag_write_test();
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "Erase={} WEL_SR1=0x{:02X} Write={}\r\n",
                            erase_ok, sr1_wel, write_ok);
                        self.push(w.written());
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "Read: {:02X} {:02X} {:02X} {:02X}\r\n",
                            readback[0], readback[1], readback[2], readback[3]);
                        self.push(w.written());
                        if write_ok {
                            self.push(b"PASS: inline write works!\r\n");
                        } else {
                            self.push(b"FAIL: inline write mismatch.\r\n");
                        }

                        // Test page_program() function
                        self.push(b"--- page_program() test ---\r\n");
                        let (pe, pw, pr) = ext_flash::diag_page_program_test();
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "Erase={} Write={} Read: {:02X}{:02X}{:02X}{:02X}\r\n",
                            pe, pw, pr[0], pr[1], pr[2], pr[3]);
                        self.push(w.written());
                        if pw { self.push(b"PASS: page_program()\r\n"); }
                        else { self.push(b"FAIL: page_program()\r\n"); }

                        // Test multi-page write()
                        self.push(b"--- write() multi-page test ---\r\n");
                        let (me, mw, mr) = ext_flash::diag_write_multi_test();
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "Erase={} Match={} Read: {:02X}{:02X}{:02X}{:02X} {:02X}{:02X}{:02X}{:02X}\r\n",
                            me, mw, mr[0], mr[1], mr[2], mr[3], mr[4], mr[5], mr[6], mr[7]);
                        self.push(w.written());
                        if mw { self.push(b"PASS: multi-page write()\r\n"); }
                        else { self.push(b"FAIL: multi-page write()\r\n"); }
                    }
                    Some("install") => {
                        // Install current upload buffer as the "right-button" binary.
                        // Slot 1 (0x10000) — slot 0 is stomped by LEGO bootloader on boot.
                        // Format at flash 0x00010000: [MAGIC:4][SIZE:4][binary...]
                        const MAGIC: u32 = 0x464C_524E; // "FLRN"
                        let sz = self.last_upload_len;
                        if sz == 0 {
                            self.push(b"ERR: nothing uploaded (upload first)\r\n");
                            return;
                        }
                        if sz > upload::UPLOAD_BUF_SIZE - 8 {
                            self.push(b"ERR: binary too large\r\n");
                            return;
                        }
                        let total = sz + 8; // header + binary
                        let sectors = (total + ext_flash::SECTOR_SIZE as usize - 1)
                            / ext_flash::SECTOR_SIZE as usize;
                        let mut tmp = [0u8; 80];
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "Installing {} B (erase {} sectors)...\r\n", sz, sectors);
                        self.push(w.written());
                        const SLOT1: u32 = 0x10000; // 64KB — avoid slot 0 (bootloader stomps it)
                        for i in 0..sectors {
                            ext_flash::sector_erase(SLOT1 + (i as u32) * ext_flash::SECTOR_SIZE);
                        }
                        // Write 8-byte header: MAGIC + SIZE
                        let mut hdr = [0u8; 8];
                        hdr[0..4].copy_from_slice(&MAGIC.to_le_bytes());
                        hdr[4..8].copy_from_slice(&(sz as u32).to_le_bytes());
                        ext_flash::write(SLOT1, &hdr);
                        // Write binary data starting at offset 8
                        let buf = upload::upload_buf();
                        ext_flash::write(SLOT1 + 8, &buf[..sz]);
                        // Verify header readback
                        let mut verify = [0u8; 8];
                        ext_flash::read(SLOT1, &mut verify);
                        let got_magic = u32::from_le_bytes([verify[0], verify[1], verify[2], verify[3]]);
                        let got_size = u32::from_le_bytes([verify[4], verify[5], verify[6], verify[7]]);
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "Verify: magic=0x{:08X} size={}\r\n", got_magic, got_size);
                        self.push(w.written());
                        if got_magic == MAGIC && got_size == sz as u32 {
                            self.push(b"OK: right-button binary installed.\r\n");
                        } else {
                            self.push(b"WARN: verify mismatch!\r\n");
                        }
                    }
                    _ => {
                        self.push(b"Usage: spiflash [id|read|erase|write|store|load|run|dir|unlock|test|install]\r\n");
                        self.push(b"  spiflash             - show JEDEC ID + status\r\n");
                        self.push(b"  spiflash read <a> [n]- hex dump n bytes (max 256)\r\n");
                        self.push(b"  spiflash erase <a>   - erase 4KB sector\r\n");
                        self.push(b"  spiflash write <a><v>- write 4 bytes (BE)\r\n");
                        self.push(b"  spiflash store <a>[s]- upload buf -> flash\r\n");
                        self.push(b"  spiflash load <a><sz> - flash -> RAM buf, then go\r\n");
                        self.push(b"  spiflash run <a><sz>  - load + go (sandboxed)\r\n");
                        self.push(b"  spiflash dir         - list occupied 64KB slots\r\n");
                        self.push(b"  spiflash install     - upload buf -> flash slot 0 (right btn)\r\n");
                        self.push(b"  spiflash unlock      - clear write protection\r\n");
                        self.push(b"  spiflash test        - run all write diagnostics\r\n");
                    }
                }
            }

            // ── DWT hardware watchpoints (self-hosted, no JTAG) ──
            "dwt" => {
                let sub = parts.next().unwrap_or("status");
                match sub {
                    "set" => {
                        // dwt set <n> <addr> [r|w|rw|pc]
                        let n = parts.next().and_then(parse_u32);
                        let addr = parts.next().and_then(|s| parse_hex(s));
                        let func_str = parts.next().unwrap_or("rw");
                        match (n, addr) {
                            (Some(n), Some(addr)) if n < 4 => {
                                if let Some(func) = dwt::WatchFunc::from_str(func_str) {
                                    unsafe { dwt::set_watchpoint(n, addr, 0, func); }
                                    let mut tmp = [0u8; 64];
                                    let mut w = BufWriter::new(&mut tmp);
                                    let _ = write!(w, "DWT#{} = 0x{:08X} {}\r\n", n, addr, func.as_str());
                                    self.push(w.written());
                                } else {
                                    self.push(b"ERR: func must be r|w|rw|pc|off\r\n");
                                }
                            }
                            _ => self.push(b"Usage: dwt set <0-3> <addr> [r|w|rw|pc]\r\n"),
                        }
                    }
                    "clear" => {
                        if let Some(n) = parts.next().and_then(parse_u32) {
                            if n < 4 {
                                unsafe { dwt::clear_watchpoint(n); }
                                let mut tmp = [0u8; 32];
                                let mut w = BufWriter::new(&mut tmp);
                                let _ = write!(w, "DWT#{} cleared\r\n", n);
                                self.push(w.written());
                            } else {
                                self.push(b"ERR: n must be 0-3\r\n");
                            }
                        } else {
                            unsafe { dwt::clear_all(); }
                            self.push(b"All DWT watchpoints cleared\r\n");
                        }
                    }
                    "status" | "info" => {
                        let ncomp = dwt::num_comparators();
                        let mut tmp = [0u8; 80];
                        let mut w = BufWriter::new(&mut tmp);
                        let _ = write!(w, "DWT: {} comparators, {} DebugMon hits\r\n",
                            ncomp, dwt::debugmon_count());
                        self.push(w.written());

                        for n in 0..4u32 {
                            let (addr, mask, func, matched) = dwt::read_watchpoint(n);
                            let (hits, pc) = dwt::hit_info(n as usize);
                            let func_name = match func {
                                0 => "off",
                                4 => "pc",
                                5 => "read",
                                6 => "write",
                                7 => "rw",
                                _ => "???",
                            };
                            let mut tmp2 = [0u8; 96];
                            let mut w2 = BufWriter::new(&mut tmp2);
                            let _ = write!(w2, "  #{}: addr=0x{:08X} mask={} func={} match={} hits={} lastPC=0x{:08X}\r\n",
                                n, addr, mask, func_name, matched, hits, pc);
                            self.push(w2.written());
                        }
                    }
                    "init" => {
                        unsafe { dwt::init(); }
                        self.push(b"DWT re-initialized\r\n");
                    }
                    _ => {
                        self.push(b"Usage: dwt [set|clear|status|init]\r\n");
                        self.push(b"  dwt set <0-3> <addr> [r|w|rw|pc] - arm watchpoint\r\n");
                        self.push(b"  dwt clear [0-3]                  - disarm (all if no arg)\r\n");
                        self.push(b"  dwt status                       - show all comparators\r\n");
                        self.push(b"  dwt init                         - re-init DWT+DebugMon\r\n");
                    }
                }
            }

            // ── GDB RSP stub ("Demon mode") ──
            "gdb" | "demon" => {
                let msg = self.gdb.enter();
                self.push(msg);
            }

            "reconnect-ser" | "reconnect" | "bye" | "quit" | "exit" | "logout" | "disconnect" => {
                self.push(b"Goodbye. USB serial drops for 2 s...\r\n");
                self.reconnect_request = true;
            }

            _ => {
                self.push(b"ERR: unknown command. Type 'help'\r\n");
            }
        }
    }

    /// Return the prompt string for initial display.
    pub fn prompt() -> &'static [u8] {
        b"\r\n=== LEGO SPIKE Prime RTIC Shell ===\r\nType 'help' for commands.\r\n\r\nspike> "
    }
}

/// Parse a decimal string to u32.
fn parse_u32(s: &str) -> Option<u32> {
    let mut result: u32 = 0;
    if s.is_empty() {
        return None;
    }
    for &b in s.as_bytes() {
        let d = b.wrapping_sub(b'0');
        if d > 9 {
            return None;
        }
        result = result.checked_mul(10)?.checked_add(d as u32)?;
    }
    Some(result)
}

/// Parse a hex string (with optional 0x prefix) to u32.
fn parse_hex(s: &str) -> Option<u32> {
    let s = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")).unwrap_or(s);
    if s.is_empty() || s.len() > 8 {
        return None;
    }
    let mut result: u32 = 0;
    for &b in s.as_bytes() {
        let d = match b {
            b'0'..=b'9' => b - b'0',
            b'a'..=b'f' => b - b'a' + 10,
            b'A'..=b'F' => b - b'A' + 10,
            _ => return None,
        };
        result = result.checked_mul(16)?.checked_add(d as u32)?;
    }
    Some(result)
}

/// Parse a number: hex (0x prefix) or decimal.
fn parse_num(s: &str) -> Option<u32> {
    if s.starts_with("0x") || s.starts_with("0X") {
        parse_hex(s)
    } else {
        parse_u32(s)
    }
}

/// Format a u32 as decimal ASCII into a buffer, return the used slice.
fn format_u32(mut val: u32, buf: &mut [u8; 12]) -> &[u8] {
    if val == 0 {
        buf[0] = b'0';
        return &buf[..1];
    }
    let mut pos = 12;
    while val > 0 {
        pos -= 1;
        buf[pos] = b'0' + (val % 10) as u8;
        val /= 10;
    }
    &buf[pos..]
}
