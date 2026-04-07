//! Demo xtask — build, upload, and debug SPIKE hub RAM demos.
//!
//! Pure Rust replacement for the Python debug pipeline. Handles serial
//! port detection, COBS upload, RSP mode entry, and symlink creation.
//!
//! ## Robustness features
//!
//! - **Auto port-free:** If another process (stale picocom, screen, Python
//!   script) holds the serial port, we detect it via `fuser`/`lsof` and
//!   kill it automatically before proceeding.
//! - **State probing:** Before assuming what mode the hub is in, we send
//!   safe probe bytes and classify the response as Shell, RSP/GDB, or
//!   Unknown — then adapt the recovery sequence accordingly.
//!
//! # Usage
//!
//! ```text
//! cargo xtask debug <name>     Build + upload + go + enter RSP + symlink
//! cargo xtask build <name>     Build + objcopy only
//! cargo xtask upload <name>    Build + upload + go (no RSP)
//! cargo xtask list             List available demos
//! cargo xtask port             Show detected hub serial port
//! cargo xtask free-port        Kill any process blocking the hub serial port
//! ```
//!
//! The `debug` command is designed as a VS Code preLaunchTask — it exits
//! after entering RSP mode so GDB/LLDB can connect to the released port.

use serialport::{SerialPort, SerialPortType};
use std::env;
use std::fs;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{exit, Command, Stdio};
use std::time::{Duration, Instant};

const LEGO_VID: u16 = 0x0694;
const RUNTIME_PID: u16 = 0x0042;
const BAUD: u32 = 115_200;
const SYMLINK: &str = "/tmp/spike-hub";

// ── ANSI ────────────────────────────────────────────────────
const RED: &str = "\x1b[91m";
const GREEN: &str = "\x1b[92m";
const YELLOW: &str = "\x1b[93m";
const CYAN: &str = "\x1b[96m";
const BOLD: &str = "\x1b[1m";
const DIM: &str = "\x1b[2m";
const RESET: &str = "\x1b[0m";

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();

    match args.first().map(|s| s.as_str()) {
        Some("debug") => cmd_debug(&args[1..]),
        Some("build") => cmd_build(&args[1..]),
        Some("upload") => cmd_upload(&args[1..]),
        Some("list") => cmd_list(),
        Some("port") => cmd_port(),
        Some("free-port") => cmd_free_port(),
        Some("-h") | Some("--help") => usage(),
        _ => usage(),
    }
}

fn usage() {
    eprintln!("{BOLD}demo-xtask{RESET} — SPIKE hub RAM demo manager");
    eprintln!();
    eprintln!("Usage: cargo xtask <command> [demo_name]");
    eprintln!();
    eprintln!("  {CYAN}debug{RESET}  <name>   Build → objcopy → upload → go → GDB RSP mode");
    eprintln!("  {CYAN}build{RESET}  <name>   Build → objcopy (.bin output)");
    eprintln!("  {CYAN}upload{RESET} <name>   Build → objcopy → upload → go (run demo)");
    eprintln!("  {CYAN}list{RESET}            List available demo names");
    eprintln!("  {CYAN}port{RESET}            Show detected hub serial port");
    eprintln!("  {CYAN}free-port{RESET}       Kill processes blocking the hub serial port");
    eprintln!();
    eprintln!("The debug command creates {SYMLINK} → serial port for GDB/LLDB.");
    eprintln!("Port locks are resolved automatically (fuser/lsof + SIGTERM/SIGKILL).");
}

// ── Port-conflict resolution ────────────────────────────────

/// Find PIDs that have `port` open, via fuser then lsof fallback.
fn find_port_users(port: &str) -> Vec<(u32, String)> {
    let my_pid = std::process::id();
    let mut users = Vec::new();

    // Try fuser first
    if let Ok(out) = Command::new("fuser")
        .arg(port)
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
    {
        let text = String::from_utf8_lossy(&out.stdout);
        for tok in text.split_whitespace() {
            let clean = tok.trim_end_matches('m').trim_end_matches('e');
            if let Ok(pid) = clean.parse::<u32>() {
                if pid != my_pid {
                    users.push((pid, pid_cmdline(pid)));
                }
            }
        }
    }

    if !users.is_empty() {
        return users;
    }

    // Fallback: lsof
    if let Ok(out) = Command::new("lsof")
        .args(["-t", port])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
    {
        let text = String::from_utf8_lossy(&out.stdout);
        for line in text.lines() {
            if let Ok(pid) = line.trim().parse::<u32>() {
                if pid != my_pid {
                    users.push((pid, pid_cmdline(pid)));
                }
            }
        }
    }

    users
}

/// Read /proc/<pid>/cmdline for display.
fn pid_cmdline(pid: u32) -> String {
    fs::read(format!("/proc/{pid}/cmdline"))
        .map(|raw| {
            raw.iter()
                .map(|&b| if b == 0 { b' ' } else { b })
                .collect::<Vec<u8>>()
        })
        .map(|v| String::from_utf8_lossy(&v).trim().to_string())
        .unwrap_or_else(|_| "?".into())
}

/// Kill processes blocking `port`. Returns true if port is now free.
fn kill_port_users(port: &str) -> bool {
    let users = find_port_users(port);
    if users.is_empty() {
        return true;
    }

    eprintln!("{YELLOW}Port {port} is locked by:{RESET}");
    for (pid, cmd) in &users {
        let short: String = cmd.chars().take(70).collect();
        eprintln!("  PID {pid}: {short}");
    }

    // SIGTERM first
    for (pid, _) in &users {
        eprintln!("  {DIM}Sending SIGTERM to PID {pid}{RESET}");
        let _ = Command::new("kill")
            .args(["-TERM", &pid.to_string()])
            .status();
    }
    std::thread::sleep(Duration::from_millis(800));

    // Check survivors → SIGKILL
    let remaining = find_port_users(port);
    if !remaining.is_empty() {
        for (pid, _) in &remaining {
            eprintln!("  {DIM}Sending SIGKILL to PID {pid}{RESET}");
            let _ = Command::new("kill")
                .args(["-KILL", &pid.to_string()])
                .status();
        }
        std::thread::sleep(Duration::from_millis(500));
    }

    let still = find_port_users(port);
    if still.is_empty() {
        eprintln!("{GREEN}✓{RESET} Port {port} freed");
        true
    } else {
        eprintln!("{RED}✗{RESET} Could not free {port}");
        false
    }
}

/// Ensure port is not locked. Auto-kills blockers. Dies on failure.
fn ensure_port_free(port: &str) {
    if find_port_users(port).is_empty() {
        return;
    }
    if !kill_port_users(port) {
        die(&format!("Cannot free {port} — close the blocking process manually"));
    }
    // Small settle time after kill
    std::thread::sleep(Duration::from_millis(300));
}

// ── Hub mode probing ────────────────────────────────────────

#[derive(Debug, PartialEq)]
enum HubMode {
    Shell,   // spike> prompt is responding
    Rsp,     // GDB RSP mode (responds to $? packets)
    Unknown, // port open but can't classify
}

/// Probe what mode the hub is in by sending safe bytes and reading response.
fn probe_hub_mode(serial: &mut Box<dyn SerialPort>) -> HubMode {
    // Drain any stale data first
    drain(serial);

    // 1. Try a safe RSP packet: $?#3f  (status query)
    let _ = serial.write_all(b"$?#3f");
    let _ = serial.flush();
    let resp = read_until_timeout(serial, Duration::from_millis(600));
    if resp.contains('$') || resp.contains('+') {
        // RSP mode — got a GDB RSP response
        return HubMode::Rsp;
    }

    // 2. Drain leftover, try shell: send empty line, look for spike>
    drain(serial);
    send_line(serial, "");
    let resp = read_until_timeout(serial, Duration::from_millis(800));
    if resp.contains("spike>") {
        return HubMode::Shell;
    }

    // 3. Try once more with slight delay (hub might be mid-boot)
    std::thread::sleep(Duration::from_millis(500));
    send_line(serial, "");
    let resp = read_until_timeout(serial, Duration::from_secs(3));
    if resp.contains("spike>") {
        return HubMode::Shell;
    }

    HubMode::Unknown
}

/// Read whatever comes back within `timeout`, return as string.
/// Returns early once data has been received and an idle gap (no new
/// data for 80ms) is detected — avoids waiting the full timeout when
/// the hub has already finished sending.
fn read_until_timeout(serial: &mut Box<dyn SerialPort>, timeout: Duration) -> String {
    let deadline = Instant::now() + timeout;
    let mut buf = [0u8; 512];
    let mut acc = Vec::new();
    let mut idle_since: Option<Instant> = None;
    while Instant::now() < deadline {
        match serial.read(&mut buf) {
            Ok(n) if n > 0 => {
                acc.extend_from_slice(&buf[..n]);
                idle_since = None; // reset idle timer
            }
            _ => {
                if !acc.is_empty() {
                    // Data was received before — start/continue idle timer
                    let now = Instant::now();
                    match idle_since {
                        None => idle_since = Some(now),
                        Some(t) if now.duration_since(t) > Duration::from_millis(80) => break,
                        _ => {}
                    }
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        }
    }
    String::from_utf8_lossy(&acc).into_owned()
}

/// Transition hub to shell mode from whatever state it's in.
fn ensure_shell_mode(serial: &mut Box<dyn SerialPort>) {
    let mode = probe_hub_mode(serial);
    match mode {
        HubMode::Shell => {
            eprintln!("  {GREEN}✓{RESET} Hub is in shell mode");
        }
        HubMode::Rsp => {
            eprintln!("  {YELLOW}Hub is in RSP/GDB mode — sending detach...{RESET}");
            let _ = serial.write_all(b"$D#44");
            let _ = serial.flush();
            std::thread::sleep(Duration::from_millis(200));
            drain(serial);
            // Now try shell
            send_line(serial, "");
            let resp = read_until(serial, "spike>", Duration::from_secs(5));
            if !resp.contains("spike>") {
                die("Hub did not return to shell after RSP detach");
            }
            eprintln!("  {GREEN}✓{RESET} Recovered to shell mode");
        }
        HubMode::Unknown => {
            eprintln!("  {YELLOW}Hub mode unknown — trying full recovery...{RESET}");
            // Shotgun approach: Ctrl-C, RSP detach, stop, empty line
            let _ = serial.write_all(b"\x03");
            std::thread::sleep(Duration::from_millis(100));
            let _ = serial.write_all(b"$D#44");
            let _ = serial.flush();
            std::thread::sleep(Duration::from_millis(500));
            drain(serial);
            send_line(serial, "stop");
            drain_for(serial, Duration::from_millis(300));
            send_line(serial, "");
            let resp = read_until(serial, "spike>", Duration::from_secs(8));
            if !resp.contains("spike>") {
                // Last resort: wait for boot
                eprintln!("  {YELLOW}Waiting for hub boot...{RESET}");
                std::thread::sleep(Duration::from_secs(3));
                drain(serial);
                send_line(serial, "");
                let resp2 = read_until(serial, "spike>", Duration::from_secs(8));
                if !resp2.contains("spike>") {
                    die(&format!("Cannot reach shell mode. Got: {:?}", resp2));
                }
            }
            eprintln!("  {GREEN}✓{RESET} Recovered to shell mode");
        }
    }
}

// ── Commands ────────────────────────────────────────────────

fn cmd_debug(args: &[String]) {
    let name = require_demo_name(args);
    let root = demo_root();

    // 1. Build first (no port needed yet)
    let bin_path = build_and_objcopy(&root, &name);

    // 2. Find hub port
    let port = find_hub_port_or_die();

    // 3. Ensure port is free (auto-kill blockers)
    ensure_port_free(&port);

    // 4. Open serial
    eprintln!("{BOLD}Uploading & entering RSP mode...{RESET}");
    let mut serial = open_serial_robust(&port);

    // 5. Probe hub state and get to shell mode
    ensure_shell_mode(&mut serial);

    // 6. Stop any running demo
    send_line(&mut serial, "stop");
    drain_for(&mut serial, Duration::from_millis(300));

    // 7. Upload
    let bin_data = fs::read(&bin_path).unwrap_or_else(|e| {
        die(&format!("Cannot read {}: {e}", bin_path.display()));
    });
    // Bump timeout for upload protocol (needs longer reads)
    let _ = serial.set_timeout(Duration::from_secs(2));
    upload_binary(&mut serial, &bin_data);
    // Restore fast timeout
    let _ = serial.set_timeout(Duration::from_millis(100));

    // 8. Start demo
    send_line(&mut serial, "go");
    drain_for(&mut serial, Duration::from_millis(300));

    // 9. Enter GDB RSP mode
    send_line(&mut serial, "gdb");
    // Wait for GDB greeting — confirms the stub is active.
    let greeting = read_until(&mut serial, "exit GDB mode", Duration::from_secs(2));
    if !greeting.contains("GDB stub active") {
        eprintln!("{YELLOW}Warning: GDB greeting not seen — proceeding anyway{RESET}");
    }
    // Final drain: consume any trailing bytes after the greeting
    drain_for(&mut serial, Duration::from_millis(200));
    // Flush kernel serial buffers so GDB starts with a clean slate
    let _ = serial.clear(serialport::ClearBuffer::All);

    // 10. Release port for GDB/LLDB
    drop(serial);

    // 11. Create stable symlink
    create_symlink(&port);

    let elf = elf_path(&root, &name);
    eprintln!("{GREEN}✓{RESET} RSP mode active on {CYAN}{port}{RESET}");
    eprintln!("{GREEN}✓{RESET} Symlink {CYAN}{SYMLINK}{RESET} → {port}");
    eprintln!();
    eprintln!("{DIM}Connect with:{RESET}");
    eprintln!("  gdb-multiarch {} \\", elf.display());
    eprintln!("    -ex \"set architecture arm\" \\");
    eprintln!("    -ex \"target remote {SYMLINK}\"");
}

fn cmd_build(args: &[String]) {
    let name = require_demo_name(args);
    let root = demo_root();
    let bin = build_and_objcopy(&root, &name);
    let sz = fs::metadata(&bin).map(|m| m.len()).unwrap_or(0);
    eprintln!("{GREEN}✓{RESET} {}: {} bytes", bin.display(), sz);
}

fn cmd_upload(args: &[String]) {
    let name = require_demo_name(args);
    let root = demo_root();

    let bin_path = build_and_objcopy(&root, &name);
    let port = find_hub_port_or_die();

    // Auto-free port
    ensure_port_free(&port);

    let mut serial = open_serial_robust(&port);
    ensure_shell_mode(&mut serial);
    send_line(&mut serial, "stop");
    drain_for(&mut serial, Duration::from_millis(300));

    let bin_data = fs::read(&bin_path).unwrap_or_else(|e| {
        die(&format!("Cannot read {}: {e}", bin_path.display()));
    });
    upload_binary(&mut serial, &bin_data);

    send_line(&mut serial, "go");
    eprintln!("{GREEN}✓{RESET} Demo '{name}' running on {port}");
}

fn cmd_free_port() {
    match find_hub_port() {
        Some(port) => {
            let users = find_port_users(&port);
            if users.is_empty() {
                eprintln!("{GREEN}✓{RESET} {port} is free (no blocking processes)");
            } else {
                kill_port_users(&port);
            }
        }
        None => {
            // No hub found via VID:PID — try all ttyACM ports
            let mut found_any = false;
            if let Ok(entries) = fs::read_dir("/dev") {
                let mut ports: Vec<String> = entries
                    .flatten()
                    .filter_map(|e| {
                        let n = e.file_name().to_string_lossy().to_string();
                        if n.starts_with("ttyACM") {
                            Some(format!("/dev/{n}"))
                        } else {
                            None
                        }
                    })
                    .collect();
                ports.sort();
                for port in &ports {
                    let users = find_port_users(port);
                    if !users.is_empty() {
                        found_any = true;
                        kill_port_users(port);
                    }
                }
                if !found_any {
                    if ports.is_empty() {
                        eprintln!("{YELLOW}No ttyACM ports found{RESET}");
                    } else {
                        eprintln!("{GREEN}✓{RESET} All ttyACM ports are free");
                    }
                }
            }
        }
    }
}

fn cmd_list() {
    let root = demo_root();
    let examples_dir = root.join("examples");

    let mut demos: Vec<String> = fs::read_dir(&examples_dir)
        .unwrap_or_else(|e| die(&format!("Cannot read {}: {e}", examples_dir.display())))
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            name.strip_suffix(".rs").map(|s| s.to_string())
        })
        .collect();
    demos.sort();

    eprintln!("{BOLD}Available demos:{RESET}");
    for d in &demos {
        eprintln!("  {d}");
    }
    eprintln!("\n{DIM}{} demos total{RESET}", demos.len());
}

fn cmd_port() {
    match find_hub_port() {
        Some(port) => {
            eprintln!("{GREEN}✓{RESET} Hub found: {CYAN}{port}{RESET}");
            println!("{port}"); // machine-readable on stdout
        }
        None => {
            eprintln!("{RED}✗{RESET} LEGO SPIKE hub not found (VID:PID {:04x}:{:04x})", LEGO_VID, RUNTIME_PID);
            exit(1);
        }
    }
}

// ── Build pipeline ──────────────────────────────────────────

fn build_and_objcopy(root: &Path, name: &str) -> PathBuf {
    eprintln!("{BOLD}Building {CYAN}{name}{RESET}{BOLD} (release)...{RESET}");

    let status = Command::new("cargo")
        .args(["build", "--example", name, "--release"])
        .current_dir(root)
        .status()
        .unwrap_or_else(|e| die(&format!("cargo: {e}")));

    if !status.success() {
        die("Build failed");
    }
    eprintln!("{GREEN}✓{RESET} Build OK");

    let elf = elf_path(root, name);
    if !elf.exists() {
        die(&format!("ELF not found: {}", elf.display()));
    }

    let bin_dir = root.join("target/spike-usr_bins");
    fs::create_dir_all(&bin_dir).ok();
    let bin = bin_dir.join(format!("{name}.bin"));

    eprintln!("  objcopy → {}", bin.display());
    let status = Command::new("arm-none-eabi-objcopy")
        .args(["-O", "binary"])
        .arg(&elf)
        .arg(&bin)
        .status()
        .unwrap_or_else(|e| die(&format!("objcopy: {e}")));

    if !status.success() {
        die("objcopy failed");
    }

    let sz = fs::metadata(&bin).map(|m| m.len()).unwrap_or(0);
    eprintln!("{GREEN}✓{RESET} {name}.bin: {sz} bytes");
    bin
}

fn elf_path(root: &Path, name: &str) -> PathBuf {
    root.join(format!(
        "target/thumbv7em-none-eabihf/release/examples/{name}"
    ))
}

fn demo_root() -> PathBuf {
    // Walk up from CWD looking for Cargo.toml with [workspace] that
    // has hub-ram-demo.x (our linker script).
    // Or use environment variable.
    if let Ok(root) = env::var("DEMO_ROOT") {
        return PathBuf::from(root);
    }

    // Try the xtask's own location (xtask is at hub-ram-demos/xtask/)
    let xtask_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let parent = xtask_dir.parent().unwrap_or(Path::new("."));
    if parent.join("hub-ram-demo.x").exists() {
        return parent.to_path_buf();
    }

    // Walk up from CWD
    if let Ok(cwd) = env::current_dir() {
        let mut dir = cwd.as_path();
        loop {
            if dir.join("hub-ram-demo.x").exists() {
                return dir.to_path_buf();
            }
            match dir.parent() {
                Some(p) => dir = p,
                None => break,
            }
        }
    }

    die("Cannot find hub-ram-demos root (no hub-ram-demo.x found)");
}

// ── Serial port detection ───────────────────────────────────

fn find_hub_port() -> Option<String> {
    // Use serialport crate to enumerate USB serial ports by VID:PID
    if let Ok(ports) = serialport::available_ports() {
        for p in &ports {
            if let SerialPortType::UsbPort(usb) = &p.port_type {
                if usb.vid == LEGO_VID && usb.pid == RUNTIME_PID {
                    return Some(p.port_name.clone());
                }
            }
        }
    }

    // Fallback: scan sysfs
    if let Ok(entries) = fs::read_dir("/sys/class/tty") {
        let mut acm_ports: Vec<String> = entries
            .flatten()
            .filter_map(|e| {
                let name = e.file_name().to_string_lossy().to_string();
                if !name.starts_with("ttyACM") {
                    return None;
                }
                let vid_path = format!("/sys/class/tty/{name}/device/../idVendor");
                let pid_path = format!("/sys/class/tty/{name}/device/../idProduct");
                let vid = fs::read_to_string(&vid_path).ok()?;
                let pid = fs::read_to_string(&pid_path).ok()?;
                if vid.trim() == "0694" && pid.trim() == "0042" {
                    Some(format!("/dev/{name}"))
                } else {
                    None
                }
            })
            .collect();
        acm_ports.sort();
        if let Some(port) = acm_ports.into_iter().next() {
            return Some(port);
        }
    }

    None
}

fn find_hub_port_or_die() -> String {
    find_hub_port().unwrap_or_else(|| {
        die("LEGO SPIKE hub not found. Is it plugged in and running firmware?");
    })
}

// ── Serial communication ────────────────────────────────────

/// Open serial port with automatic port-free retry.
/// Uses 100ms timeout — fast for probing and draining.  Callers that
/// need longer reads (upload protocol) bump it temporarily.
fn open_serial_robust(port: &str) -> Box<dyn SerialPort> {
    let timeout = Duration::from_millis(100);
    // First attempt
    match serialport::new(port, BAUD)
        .timeout(timeout)
        .open()
    {
        Ok(s) => return s,
        Err(e) => {
            let msg = format!("{e}");
            if msg.contains("lock") || msg.contains("busy") || msg.contains("denied") {
                eprintln!("{YELLOW}Port locked — auto-freeing {port}...{RESET}");
                if !kill_port_users(port) {
                    die(&format!("Cannot free {port}"));
                }
                std::thread::sleep(Duration::from_millis(500));
                // Retry
                return serialport::new(port, BAUD)
                    .timeout(timeout)
                    .open()
                    .unwrap_or_else(|e2| {
                        die(&format!("Cannot open {port} after freeing: {e2}"));
                    });
            }
            die(&format!("Cannot open {port}: {e}"));
        }
    }
}

fn send_line(serial: &mut Box<dyn SerialPort>, cmd: &str) {
    let mut data = cmd.as_bytes().to_vec();
    data.extend_from_slice(b"\r\n");
    let _ = serial.write_all(&data);
    let _ = serial.flush();
}

fn drain(serial: &mut Box<dyn SerialPort>) {
    let mut buf = [0u8; 512];
    loop {
        match serial.read(&mut buf) {
            Ok(0) => break,
            Ok(_) => continue,
            Err(_) => break,
        }
    }
}

fn drain_for(serial: &mut Box<dyn SerialPort>, dur: Duration) {
    let deadline = Instant::now() + dur;
    let mut buf = [0u8; 512];
    while Instant::now() < deadline {
        match serial.read(&mut buf) {
            Ok(_) => {}
            Err(_) => std::thread::sleep(Duration::from_millis(20)),
        }
    }
}

/// Read until `needle` appears or timeout.  Returns accumulated text.
fn read_until(serial: &mut Box<dyn SerialPort>, needle: &str, timeout: Duration) -> String {
    let deadline = Instant::now() + timeout;
    let mut buf = [0u8; 512];
    let mut acc = Vec::new();

    while Instant::now() < deadline {
        match serial.read(&mut buf) {
            Ok(n) if n > 0 => {
                acc.extend_from_slice(&buf[..n]);
                let text = String::from_utf8_lossy(&acc);
                if text.contains(needle) {
                    return text.into_owned();
                }
            }
            _ => std::thread::sleep(Duration::from_millis(20)),
        }
    }
    String::from_utf8_lossy(&acc).into_owned()
}



// ── COBS upload protocol ────────────────────────────────────

fn upload_binary(serial: &mut Box<dyn SerialPort>, data: &[u8]) {
    eprintln!("  Uploading {} bytes...", data.len());

    // Send upload command
    send_line(serial, &format!("upload {}", data.len()));
    let resp = read_until(serial, "READY", Duration::from_secs(3));
    if !resp.contains("READY") {
        die(&format!("Upload not ready: {resp:?}"));
    }

    // COBS-encode and send
    let encoded = cobs_encode(data);
    serial.write_all(&encoded).unwrap_or_else(|e| {
        die(&format!("Write failed: {e}"));
    });
    let _ = serial.flush();

    // Wait for OK
    let resp = read_until(serial, "OK", Duration::from_secs(5));
    if !resp.contains("OK") {
        die(&format!("Upload failed: {resp:?}"));
    }
    eprintln!("{GREEN}✓{RESET} Upload OK");
}

/// COBS-encode a byte slice (Consistent Overhead Byte Stuffing).
fn cobs_encode(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + data.len() / 254 + 2);
    let mut idx = 0;
    while idx < data.len() {
        let start = idx;
        while idx < data.len() && data[idx] != 0 && (idx - start) < 254 {
            idx += 1;
        }
        out.push((idx - start + 1) as u8);
        out.extend_from_slice(&data[start..idx]);
        if idx < data.len() && data[idx] == 0 {
            idx += 1;
        }
    }
    out.push(0x00); // sentinel
    out
}

// ── Symlink ─────────────────────────────────────────────────

fn create_symlink(port: &str) {
    // Remove existing
    if std::path::Path::new(SYMLINK).exists() || std::path::Path::new(SYMLINK).is_symlink() {
        let _ = fs::remove_file(SYMLINK);
    }
    match std::os::unix::fs::symlink(port, SYMLINK) {
        Ok(()) => {}
        Err(e) => eprintln!("{YELLOW}Warning: cannot create {SYMLINK}: {e}{RESET}"),
    }
}

// ── Utilities ───────────────────────────────────────────────

fn require_demo_name(args: &[String]) -> String {
    if args.is_empty() {
        eprintln!("{RED}Error: demo name required{RESET}");
        eprintln!("  Use `cargo xtask list` to see available demos");
        exit(1);
    }
    // Strip .rs suffix if provided
    args[0].strip_suffix(".rs").unwrap_or(&args[0]).to_string()
}

fn die(msg: &str) -> ! {
    eprintln!("{RED}ERROR:{RESET} {msg}");
    exit(1);
}
