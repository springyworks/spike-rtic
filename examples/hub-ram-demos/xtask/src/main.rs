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
use std::io::{self, BufRead, Read, Write};
use std::path::{Path, PathBuf};
use std::os::unix::process::CommandExt;
use std::process::{exit, Command, Stdio};
use std::time::{Duration, Instant};

const LEGO_VID: u16 = 0x0694;
const RUNTIME_PID: u16 = 0x0042;
const BAUD: u32 = 115_200;
const SYMLINK: &str = "/tmp/spike-hub";
const STATE_FILE: &str = "/tmp/spike-hub-state";
const GDB_TCP_PORT: u16 = 3333;

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
        None => interactive(),
        Some(other) => {
            // Treat unknown arg as a demo name → interactive action picker
            let name = other.strip_suffix(".rs").unwrap_or(other).to_string();
            interactive_for(&name);
        }
    }
}

fn usage() {
    eprintln!("{BOLD}demo-xtask{RESET} — SPIKE hub RAM demo manager");
    eprintln!();
    eprintln!("Usage: cargo example [demo_name]");
    eprintln!("       cargo example <command> [demo_name]");
    eprintln!();
    eprintln!("  {CYAN}(none){RESET}           Interactive: pick demo → pick action");
    eprintln!("  {CYAN}<name>{RESET}            Pick action for a specific demo");
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
                // Port may be stale after detach (hub re-enumerated)
                eprintln!("  {YELLOW}Port may be stale — attempting re-detection...{RESET}");
                *serial = reopen_or_die();
                if !sync_shell(serial) {
                    die("Hub did not return to shell after RSP detach + port re-detection");
                }
            }
            eprintln!("  {GREEN}✓{RESET} Recovered to shell mode");
        }
        HubMode::Unknown => {
            eprintln!("  {YELLOW}Hub mode unknown — trying full recovery...{RESET}");
            // Shotgun approach: Ctrl-C, RSP detach, kill, empty line
            let _ = serial.write_all(b"\x03");
            std::thread::sleep(Duration::from_millis(100));
            let _ = serial.write_all(b"$D#44");
            let _ = serial.flush();
            std::thread::sleep(Duration::from_millis(500));
            drain(serial);
            send_line(serial, "kill");
            drain_for(serial, Duration::from_millis(500));
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
                    // Port may be stale (hub re-enumerated to a different
                    // ttyACM).  Try re-detecting before giving up.
                    eprintln!("  {YELLOW}Port may be stale — attempting re-detection...{RESET}");
                    *serial = reopen_or_die();
                    if !sync_shell(serial) {
                        die(&format!("Cannot reach shell mode after port re-detection. Got: {:?}", resp2));
                    }
                }
            }
            eprintln!("  {GREEN}✓{RESET} Recovered to shell mode");
        }
    }
}

// ── Interactive mode ────────────────────────────────────────

fn list_demos() -> Vec<String> {
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
    demos
}

fn pick_demo_interactive() -> String {
    let demos = list_demos();
    if demos.is_empty() {
        die("No demos found");
    }

    eprintln!();
    eprintln!("  {BOLD}Available demos:{RESET}");
    eprintln!("  {}", "─".repeat(44));

    let mid = (demos.len() + 1) / 2;
    for i in 0..mid {
        let left = format!("{:>3}. {}", i + 1, demos[i]);
        let right = if i + mid < demos.len() {
            format!("{:>3}. {}", i + mid + 1, demos[i + mid])
        } else {
            String::new()
        };
        eprintln!("  {:<26}{}", left, right);
    }
    eprintln!();

    let input = prompt_input("  Pick demo [number or name]: ");

    // Try as number
    if let Ok(n) = input.parse::<usize>() {
        if n >= 1 && n <= demos.len() {
            return demos[n - 1].clone();
        }
    }
    // Prefix match
    if let Some(d) = demos.iter().find(|d| d.starts_with(&input)) {
        return d.clone();
    }
    // Exact match
    if demos.contains(&input) {
        return input;
    }

    die(&format!("Demo not found: {input}"));
}

fn pick_action(name: &str) {
    eprintln!();
    eprintln!("  {BOLD}Demo:{RESET} {CYAN}{name}{RESET}");
    eprintln!("  {}", "─".repeat(44));
    eprintln!("  {BOLD}[1]{RESET} Upload & run");
    eprintln!("  {BOLD}[2]{RESET} Build only (.bin)");
    eprintln!("  {BOLD}[3]{RESET} Debug (build → upload → GDB RSP)");
    eprintln!("  {DIM}[q] Cancel{RESET}");
    eprintln!();

    let choice = prompt_input("  Choose [1-3/q]: ");
    let name_args = [name.to_string()];
    match choice.as_str() {
        "1" | "upload" | "u" => cmd_upload(&name_args),
        "2" | "build" | "b" => cmd_build(&name_args),
        "3" | "debug" | "d" => cmd_debug(&name_args),
        "q" | "Q" | "" => {
            eprintln!("  {DIM}Cancelled.{RESET}");
        }
        _ => eprintln!("  {RED}Invalid choice.{RESET}"),
    }
}

fn interactive() {
    let name = pick_demo_interactive();
    pick_action(&name);
}

fn interactive_for(name: &str) {
    // Verify the demo exists
    let demos = list_demos();
    let resolved = if demos.contains(&name.to_string()) {
        name.to_string()
    } else if let Some(d) = demos.iter().find(|d| d.starts_with(name)) {
        d.clone()
    } else {
        die(&format!("Demo not found: {name}. Use `list` to see available demos."));
    };
    pick_action(&resolved);
}

fn prompt_input(msg: &str) -> String {
    eprint!("{msg}");
    io::stderr().flush().unwrap();
    let mut buf = String::new();
    io::stdin().lock().read_line(&mut buf).unwrap();
    buf.trim().to_string()
}

// ── Commands ────────────────────────────────────────────────

fn cmd_debug(args: &[String]) {
    let name = require_demo_name(args);
    let root = demo_root();

    // 1. Build first (no port needed yet)
    let bin_path = build_and_objcopy(&root, &name);

    // 2. Find hub port
    let mut port = find_hub_port_or_die();

    // 3. Ensure port is free (auto-kill blockers)
    ensure_port_free(&port);

    // 3b. Re-detect port: killing a blocker (e.g. CodeLLDB adapter) can
    // cause the hub to re-enumerate on a different ttyACM.
    std::thread::sleep(Duration::from_millis(500));
    let new_port = find_hub_port_or_die();
    if new_port != port {
        eprintln!("  {YELLOW}Port changed after free: {port} → {new_port}{RESET}");
        port = new_port;
    }

    // 4. Open serial
    eprintln!("{BOLD}Uploading & entering RSP mode...{RESET}");
    let mut serial = open_serial_robust(&port);

    // 5. Probe hub state and get to shell mode
    ensure_shell_mode(&mut serial);

    // 6. Kill any running demo cooperatively (SIGTERM).
    // NEVER use "kill -9" here — force_kill_sandbox from the USB ISR
    // context leaves BASEPRI elevated, permanently blocking priority 2
    // tasks (heartbeat, sensor, USB) → watchdog reset after ~5 s.
    send_line(&mut serial, "kill");
    drain_for(&mut serial, Duration::from_millis(500));

    // 7. Upload
    let bin_data = fs::read(&bin_path).unwrap_or_else(|e| {
        die(&format!("Cannot read {}: {e}", bin_path.display()));
    });
    sync_shell_or_die(&mut serial, "upload");
    // Bump timeout for upload protocol (needs longer reads)
    let _ = serial.set_timeout(Duration::from_secs(2));
    upload_binary(&mut serial, &bin_data);
    // Restore fast timeout
    let _ = serial.set_timeout(Duration::from_millis(100));

    // 8. Start demo in debug mode — combined go + gdb in one command.
    // The firmware's `debug` command:
    //   - Plants BKPT at demo entry (_start at 0x20040000)
    //   - Enters GDB/RSP mode (gdb_active, pending_stop)
    //   - Starts demo in Thread mode (privileged, no MPU)
    //   - Demo hits BKPT immediately → DebugMonitor halts at _start
    // This replaces separate go + gdb and fixes the priority conflict
    // that prevented DebugMonitor from halting go! (Handler mode) demos.
    start_debug_session(&mut serial);

    // Re-detect port: hub may have rebooted during recovery, changing
    // ttyACM number (Linux CDC ACM re-enumeration quirk).
    if let Some(name) = serial.name() {
        if name != port {
            eprintln!("  {YELLOW}Port changed: {port} → {name}{RESET}");
            port = name;
        }
    }

    // 9. Find the dedicated GDB port (ttyACM1 / interface 2)
    //    With dual-CDC firmware, GDB RSP goes on a separate USB serial port.
    //    Shell port (ttyACM0) stays available for shell commands.
    let gdb_port = find_gdb_port().unwrap_or_else(|| {
        // Fallback: assume port+1 (e.g., ttyACM0 → ttyACM1)
        if let Some(num_str) = port.strip_prefix("/dev/ttyACM") {
            if let Ok(n) = num_str.parse::<u32>() {
                eprintln!("  {YELLOW}GDB port not found by sysfs — trying ttyACM{}{RESET}", n + 1);
                return format!("/dev/ttyACM{}", n + 1);
            }
        }
        // No GDB port at all — fall back to shell port (single-CDC compat)
        eprintln!("  {YELLOW}No GDB port found — using shell port for RSP{RESET}");
        port.clone()
    });

    // 10. Release shell port — keep it free for interactive use
    drop(serial);
    std::thread::sleep(Duration::from_millis(100));

    // 11. Configure GDB port TTY settings
    configure_tty(&gdb_port);

    // 12. Create stable symlinks
    create_symlink(&port);
    // Also create /tmp/spike-gdb symlink
    let gdb_symlink = "/tmp/spike-gdb";
    let _ = std::fs::remove_file(gdb_symlink);
    let _ = std::os::unix::fs::symlink(&gdb_port, gdb_symlink);

    // 13. Write state file so other chain participants can check
    let elf = elf_path(&root, &name);
    write_hub_state_file(&port, &name, &elf);

    eprintln!("{GREEN}✓{RESET} Shell port: {CYAN}{port}{RESET}");
    eprintln!("{GREEN}✓{RESET} GDB port: {CYAN}{gdb_port}{RESET}");
    eprintln!("{GREEN}✓{RESET} Symlink {CYAN}{SYMLINK}{RESET} → {port}");
    eprintln!("{GREEN}✓{RESET} Symlink {CYAN}{gdb_symlink}{RESET} → {gdb_port}");
    eprintln!("{GREEN}✓{RESET} State written to {CYAN}{STATE_FILE}{RESET}");

    // 14. Start TCP→serial bridge on GDB port (not shell port)
    start_gdb_bridge(&gdb_port);

    eprintln!();
    eprintln!("{DIM}Connect with:{RESET}");
    eprintln!("  gdb-multiarch {} \\", elf.display());
    eprintln!("    -ex \"set architecture arm\" \\");
    eprintln!("    -ex \"target remote localhost:3333\"");
}

/// Start a debug session using the firmware's `debug` shell command.
/// This combines demo launch + GDB RSP entry into one command:
///   - Plants BKPT at demo entry (halt on first instruction)
///   - Enters RSP mode
///   - Starts demo in Thread mode (privileged, DebugMonitor works)
/// Retries once on failure.
fn start_debug_session(serial: &mut Box<dyn SerialPort>) {
    for attempt in 1..=2 {
        if attempt > 1 {
            eprintln!("  {YELLOW}Retry: recovering to shell...{RESET}");
            let _ = serial.write_all(b"$D#44");
            let _ = serial.flush();
            std::thread::sleep(Duration::from_millis(300));
            drain(serial);
            send_line(serial, "kill");
            drain_for(serial, Duration::from_millis(500));
            send_line(serial, "");
            let resp = read_until(serial, "spike>", Duration::from_secs(3));
            if !resp.contains("spike>") {
                eprintln!("  {YELLOW}Hub unresponsive — checking for reboot...{RESET}");
                drop(std::mem::replace(serial, reopen_or_die()));
                send_line(serial, "");
                let resp2 = read_until(serial, "spike>", Duration::from_secs(8));
                if !resp2.contains("spike>") {
                    die("Cannot reach hub after recovery attempt");
                }
                eprintln!("  {GREEN}✓{RESET} Hub reconnected after reboot");
            }
        }

        sync_shell_or_die(serial, "debug");

        // Send the combined debug command
        send_line(serial, "debug");
        let greeting = read_until_verbose(serial, "exit GDB mode", Duration::from_secs(3));

        if greeting.contains("GDB debug active") {
            eprintln!("  {GREEN}✓{RESET} Debug session started (demo halted at entry)");
            drain_for(serial, Duration::from_millis(200));
            let _ = serial.clear(serialport::ClearBuffer::All);
            return;
        }

        // Greeting not seen — probe RSP directly
        eprintln!("  {YELLOW}Debug greeting not seen ({} bytes: {:?}) — probing RSP...{RESET}",
            greeting.len(), &greeting[..greeting.len().min(120)]);
        drain_for(serial, Duration::from_millis(100));
        let _ = serial.clear(serialport::ClearBuffer::All);

        if verify_rsp_packet(serial) {
            eprintln!("  {GREEN}✓{RESET} RSP probe confirmed — debug session active");
            return;
        }

        eprintln!("  {RED}RSP probe failed (attempt {attempt}/2){RESET}");
    }

    die(
        "Debug session failed after 2 attempts.\n\
         Possible causes:\n\
         - No binary loaded (upload failed?)\n\
         - Firmware doesn't support 'debug' command (needs reflash)\n\
         - Serial communication issue",
    );
}

/// Start demo with "go", enter GDB RSP mode with "gdb", and *verify*
/// RSP is actually active. Retries once on failure. Dies if RSP cannot
/// be confirmed — the preLaunchTask must fail so VS Code doesn't launch
/// a debugger that will time out.
fn start_demo_and_enter_rsp(serial: &mut Box<dyn SerialPort>, go_cmd: &str) {
    for attempt in 1..=2 {
        if attempt > 1 {
            eprintln!("  {YELLOW}Retry: recovering to shell...{RESET}");
            // Try to get back to shell: detach RSP if stuck, kill demo
            let _ = serial.write_all(b"$D#44");
            let _ = serial.flush();
            std::thread::sleep(Duration::from_millis(300));
            drain(serial);
            send_line(serial, "kill");
            drain_for(serial, Duration::from_millis(500));
            // Verify we're in shell
            send_line(serial, "");
            let resp = read_until(serial, "spike>", Duration::from_secs(3));
            if !resp.contains("spike>") {
                // Hub may have rebooted (watchdog / crash).  Try to
                // re-detect the port and re-open.
                eprintln!("  {YELLOW}Hub unresponsive — checking for reboot...{RESET}");
                drop(std::mem::replace(serial, reopen_or_die()));
                // After reboot, hub is in shell mode with no demo running.
                // Must send data first to trigger host_synced on the hub —
                // until feed() is called, shell output drain is gated off.
                send_line(serial, "");
                let resp2 = read_until(serial, "spike>", Duration::from_secs(8));
                if !resp2.contains("spike>") {
                    die("Cannot reach hub after recovery attempt");
                }
                eprintln!("  {GREEN}✓{RESET} Hub reconnected after reboot");
            }
        }

        // Sync: confirm shell is responsive before starting demo
        sync_shell_or_die(serial, "go");

        // Start demo
        send_line(serial, go_cmd);
        let go_resp = read_until_timeout(serial, Duration::from_millis(600));
        if go_resp.contains("ERR:") {
            eprintln!("  {RED}go command failed: {}{RESET}", go_resp.trim());
            if attempt < 2 { continue; }
            die("Cannot start demo — 'go' command failed");
        }
        if go_resp.contains("go ") || go_resp.contains("sandboxed") || go_resp.contains("PRIVILEGED") {
            eprintln!("  {GREEN}✓{RESET} Demo started");
        } else {
            eprintln!("  {DIM}go response: {:?}{RESET}", go_resp.trim());
        }

        // Small pause for demo to initialize — but log what we eat
        let drained = read_until_timeout(serial, Duration::from_millis(300));
        if !drained.is_empty() {
            eprintln!("  {DIM}drain after go ({} bytes): {:?}{RESET}",
                drained.len(), &drained[..drained.len().min(120)]);
        }

        // ── Serial health check ─────────────────────────────────
        // Verify the serial link still works before entering GDB mode.
        // If the demo faulted (e.g. sandboxed demo hit MPU) and corrupted
        // USB state, or if the tty fd went stale, we catch it here.
        if !sync_shell(serial) {
            eprintln!("  {RED}Serial health check FAILED — hub not responding{RESET}");
            if attempt < 2 { continue; }
            die("Serial link broken after starting demo");
        }
        eprintln!("  {DIM}serial health check: OK{RESET}");

        // Enter GDB mode — with verbose error detection
        if let Err(e) = serial.write_all(b"gdbrsp\r\n") {
            eprintln!("  {RED}write_all(gdb) failed: {e}{RESET}");
            if attempt < 2 { continue; }
            die("Cannot write to serial port");
        }
        if let Err(e) = serial.flush() {
            eprintln!("  {RED}flush after gdb failed: {e}{RESET}");
        }

        let greeting = read_until_verbose(serial, "exit GDB mode", Duration::from_secs(3));

        if greeting.contains("GDB RSP active") {
            eprintln!("  {GREEN}✓{RESET} GDB greeting confirmed");
            drain_for(serial, Duration::from_millis(200));
            let _ = serial.clear(serialport::ClearBuffer::All);
            return; // Success!
        }

        // Greeting not seen — probe RSP directly
        eprintln!("  {YELLOW}GDB greeting not seen ({} bytes: {:?}) — probing RSP...{RESET}",
            greeting.len(), &greeting[..greeting.len().min(120)]);
        drain_for(serial, Duration::from_millis(100));
        let _ = serial.clear(serialport::ClearBuffer::All);

        if verify_rsp_packet(serial) {
            eprintln!("  {GREEN}✓{RESET} RSP probe confirmed — hub is in GDB mode");
            return; // Success despite missing greeting
        }

        eprintln!("  {RED}RSP probe failed (attempt {attempt}/2){RESET}");
    }

    die(
        "Hub is NOT in RSP mode after 2 attempts.\n\
         Possible causes:\n\
         - No binary loaded (upload failed?)\n\
         - Demo crashed before GDB stub could activate\n\
         - Serial communication issue\n\
         Check hub state with: cargo xtask port",
    );
}

/// Send an RSP status query ($?#3f) and check for a valid RSP response.
/// Returns true if the hub responds with RSP-framed data ($ or +).
/// This is safe: consuming the stop reply has no side effect because the
/// target is genuinely halted (dwt::is_halted() stays true), so GDB's
/// subsequent $? will still get T05.
fn verify_rsp_packet(serial: &mut Box<dyn SerialPort>) -> bool {
    let _ = serial.write_all(b"$?#3f");
    let _ = serial.flush();
    let resp = read_until_timeout(serial, Duration::from_millis(1500));

    // RSP response: + (ack) followed by $T05#b9 or $S05#b8
    // Shell response: echo/ERR text without $ or +
    let is_rsp = resp.contains('$') || resp.starts_with('+');
    if is_rsp {
        // Drain any trailing RSP data
        drain_for(serial, Duration::from_millis(100));
        let _ = serial.clear(serialport::ClearBuffer::All);
    } else if !resp.is_empty() {
        eprintln!("  {DIM}RSP probe got: {:?}{RESET}",
            resp.chars().take(80).collect::<String>());
    }
    is_rsp
}

/// After dropping the serial port, briefly re-open it, send an RSP probe,
/// and confirm the hub is still in GDB mode. This catches issues where
/// dropping the port (DTR, HUPCL) kicked the hub out of RSP mode.
fn verify_rsp_after_release(port: &str) {
    // Small settle time for kernel tty cleanup
    std::thread::sleep(Duration::from_millis(300));

    let serial = serialport::new(port, BAUD)
        .timeout(Duration::from_millis(200))
        .open();
    let mut serial = match serial {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{YELLOW}Warning: post-release verify skipped (cannot open {port}: {e}){RESET}");
            return;
        }
    };

    // The DTR rising edge resets host_synced. We need to send something
    // so feed() sets host_synced=true before the stub can respond.
    // Our $? packet serves this purpose.
    let _ = serial.write_all(b"$?#3f");
    let _ = serial.flush();
    let resp = read_until_timeout(&mut serial, Duration::from_millis(1500));

    let is_rsp = resp.contains('$') || resp.starts_with('+');
    if is_rsp {
        eprintln!("  {GREEN}✓{RESET} Post-release RSP verified");
    } else {
        eprintln!("{RED}ERROR: Hub exited RSP mode after port release!{RESET}");
        eprintln!("  {DIM}Got: {:?}{RESET}", resp.chars().take(80).collect::<String>());
        drop(serial);
        die("Hub is no longer in RSP mode — debug session will fail");
    }

    // Clean up for GDB
    drain_for(&mut serial, Duration::from_millis(100));
    let _ = serial.clear(serialport::ClearBuffer::All);
    drop(serial);

    // Re-apply stty after our verification open/close cycle
    configure_tty(port);
}

/// Start a socat TCP→serial bridge so CodeLLDB can connect via TCP.
/// CodeLLDB's `process connect --plugin gdb-remote serial://` doesn't
/// work, but `localhost:3333` does.  Kills any previous bridge first.
fn start_gdb_bridge(port: &str) {
    // Kill any existing bridge
    let _ = std::process::Command::new("pkill")
        .args(["-f", &format!("socat.*TCP-LISTEN:{GDB_TCP_PORT}")])
        .status();
    std::thread::sleep(Duration::from_millis(100));

    // Start socat: TCP listen on GDB_TCP_PORT ↔ serial port (raw, 115200)
    // - fork: accept multiple connections (survives client disconnect/reconnect)
    // - process_group(0): new process group so SIGHUP from VS Code task
    //   terminal close doesn't kill socat
    let child = std::process::Command::new("socat")
        .args([
            &format!("TCP-LISTEN:{GDB_TCP_PORT},reuseaddr,fork"),
            &format!("FILE:{port},b115200,raw,echo=0"),
        ])
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .process_group(0)
        .spawn();

    match child {
        Ok(c) => {
            // Brief pause so socat binds the port before CodeLLDB connects
            std::thread::sleep(Duration::from_millis(200));
            eprintln!("{GREEN}✓{RESET} GDB bridge: localhost:{GDB_TCP_PORT} → {port} (socat pid {})", c.id());
        }
        Err(e) => {
            eprintln!("{YELLOW}Warning: cannot start socat bridge: {e}{RESET}");
            eprintln!("{YELLOW}  Install socat or use gdb-multiarch with serial://{RESET}");
        }
    }
}

/// Write a JSON state file that chain participants (launch configs, scripts)
/// can read to know the hub's current state.
fn write_hub_state_file(port: &str, demo: &str, elf: &Path) {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let content = format!(
        concat!(
            "{{\n",
            "  \"mode\": \"rsp\",\n",
            "  \"port\": \"{}\",\n",
            "  \"symlink\": \"{}\",\n",
            "  \"demo\": \"{}\",\n",
            "  \"elf\": \"{}\",\n",
            "  \"timestamp\": {},\n",
            "  \"pid\": {}\n",
            "}}\n"
        ),
        port, SYMLINK, demo, elf.display(), now, std::process::id()
    );
    match fs::write(STATE_FILE, &content) {
        Ok(()) => {}
        Err(e) => eprintln!("{YELLOW}Warning: cannot write {STATE_FILE}: {e}{RESET}"),
    }
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
    send_line(&mut serial, "kill");
    drain_for(&mut serial, Duration::from_millis(500));

    let bin_data = fs::read(&bin_path).unwrap_or_else(|e| {
        die(&format!("Cannot read {}: {e}", bin_path.display()));
    });
    sync_shell_or_die(&mut serial, "upload");
    upload_binary(&mut serial, &bin_data);

    sync_shell_or_die(&mut serial, "go");
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

/// Check if a demo requires privileged mode (`go!`).
///
/// Scans the first 20 lines of the demo source for the marker
/// `MUST run with \`go!\`` (used by reg_dump, pwm_diag, etc.).
/// Returns "go!" if found, "go" otherwise.
fn detect_go_command<'a>(root: &Path, name: &str) -> &'a str {
    let src = root.join(format!("examples/{name}.rs"));
    if let Ok(content) = fs::read_to_string(&src) {
        // Only check the doc-comment header (first 20 lines)
        for line in content.lines().take(20) {
            if line.contains("MUST run with") && line.contains("go!") {
                return "go!";
            }
        }
    }
    "go"
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

/// Find a SPIKE hub serial port by USB interface number.
///
/// With dual-CDC firmware, interface 0 = shell, interface 2 = GDB RSP.
/// Falls back to `/dev/spike-shell` or `/dev/spike-gdb` symlinks (udev).
fn find_hub_port_by_interface(iface_num: u8) -> Option<String> {
    // Check udev symlinks first (fastest, most reliable)
    let symlink = match iface_num {
        0 => "/dev/spike-shell",
        2 => "/dev/spike-gdb",
        _ => "",
    };
    if !symlink.is_empty() && std::path::Path::new(symlink).exists() {
        return Some(symlink.to_string());
    }

    // Scan sysfs for matching VID:PID + interface number
    if let Ok(entries) = fs::read_dir("/sys/class/tty") {
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if !name.starts_with("ttyACM") {
                continue;
            }
            let base = format!("/sys/class/tty/{name}/device/..");
            let vid = fs::read_to_string(format!("{base}/idVendor")).unwrap_or_default();
            let pid = fs::read_to_string(format!("{base}/idProduct")).unwrap_or_default();
            if vid.trim() != "0694" || pid.trim() != "0042" {
                continue;
            }
            // Read bInterfaceNumber from the interface directory
            let iface = fs::read_to_string(format!("/sys/class/tty/{name}/device/bInterfaceNumber"))
                .unwrap_or_default();
            let iface_str = format!("{:02}", iface_num);
            if iface.trim() == iface_str {
                return Some(format!("/dev/{name}"));
            }
        }
    }

    None
}

fn find_hub_port() -> Option<String> {
    // Prefer interface-based detection (dual-CDC aware)
    if let Some(port) = find_hub_port_by_interface(0) {
        return Some(port);
    }

    // Legacy fallback: any port with matching VID:PID
    if let Ok(ports) = serialport::available_ports() {
        for p in &ports {
            if let SerialPortType::UsbPort(usb) = &p.port_type {
                if usb.vid == LEGO_VID && usb.pid == RUNTIME_PID {
                    return Some(p.port_name.clone());
                }
            }
        }
    }

    None
}

/// Find the GDB RSP serial port (interface 2 / ttyACM1).
/// Returns None if not found (single-CDC firmware or not plugged in).
fn find_gdb_port() -> Option<String> {
    find_hub_port_by_interface(2)
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

/// Re-detect the hub port (it may have changed after a reboot/watchdog
/// reset) and open a fresh serial connection.  Waits up to 8 seconds for
/// the hub to re-enumerate on USB.
fn reopen_or_die() -> Box<dyn SerialPort> {
    let deadline = Instant::now() + Duration::from_secs(8);
    loop {
        if let Some(port) = find_hub_port() {
            ensure_port_free(&port);
            match serialport::new(&port, BAUD)
                .timeout(Duration::from_millis(100))
                .open()
            {
                Ok(s) => {
                    eprintln!("  {GREEN}✓{RESET} Reopened {CYAN}{port}{RESET}");
                    return s;
                }
                Err(_) => {}
            }
        }
        if Instant::now() > deadline {
            die("Hub did not re-enumerate within 8 seconds after reboot");
        }
        std::thread::sleep(Duration::from_millis(500));
    }
}

fn send_line(serial: &mut Box<dyn SerialPort>, cmd: &str) {
    let mut data = cmd.as_bytes().to_vec();
    data.extend_from_slice(b"\r\n");
    if let Err(e) = serial.write_all(&data) {
        eprintln!("{RED}send_line write error: {e}{RESET}");
    }
    if let Err(e) = serial.flush() {
        eprintln!("{RED}send_line flush error: {e}{RESET}");
    }
}

/// Synchronize with the shell: send an empty line, wait for `spike>`.
///
/// This serves two critical purposes:
///   1. **Triggers `host_synced`** on the hub — until the host sends
///      at least one byte, the firmware's USB ISR gates all output drain.
///      After DTR rising edge (port open), `host_synced` is false and
///      any queued output (READY, prompt, etc.) is silently held.
///   2. **Confirms the serial link is alive** — if the hub crashed,
///      rebooted, or the tty fd went stale, we detect it here.
///
/// Tries twice (the first attempt may arrive during a USB/shell
/// transition after RSP detach where the hub is briefly unresponsive).
/// Returns `true` if `spike>` was seen, `false` otherwise.
fn sync_shell(serial: &mut Box<dyn SerialPort>) -> bool {
    for attempt in 1..=3 {
        drain(serial);
        send_line(serial, "");
        let resp = read_until(serial, "spike>", Duration::from_secs(2));
        if resp.contains("spike>") {
            return true;
        }
        if attempt < 3 {
            eprintln!("  {DIM}sync_shell: no prompt (attempt {attempt}/3), retrying...{RESET}");
            // After RSP detach the hub can be sluggish — give it time
            std::thread::sleep(Duration::from_millis(500));
        }
    }
    false
}

/// Like `sync_shell` but recovers from broken-pipe / stale fd by
/// re-detecting the hub port and reopening.  Dies only if all
/// recovery attempts fail.
fn sync_shell_or_die(serial: &mut Box<dyn SerialPort>, context: &str) {
    if sync_shell(serial) {
        return;
    }
    // The fd may be stale (hub re-enumerated, ttyACM number changed).
    // Try to find the hub again and reopen.
    eprintln!("  {YELLOW}sync_shell failed — attempting port re-detection...{RESET}");
    let new = reopen_or_die();
    *serial = new;
    if sync_shell(serial) {
        eprintln!("  {GREEN}✓{RESET} Recovered on {}", serial.name().unwrap_or_default());
        return;
    }
    die(&format!("Shell sync failed before {context} — hub not responding after reopen"));
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

/// Like read_until, but logs detailed error/timeout statistics.
/// Use for diagnosing "0 bytes received" failures.
fn read_until_verbose(serial: &mut Box<dyn SerialPort>, needle: &str, timeout: Duration) -> String {
    let deadline = Instant::now() + timeout;
    let mut buf = [0u8; 512];
    let mut acc = Vec::new();
    let mut timeout_count = 0u32;
    let mut err_count = 0u32;
    let mut first_err: Option<String> = None;

    while Instant::now() < deadline {
        match serial.read(&mut buf) {
            Ok(n) if n > 0 => {
                acc.extend_from_slice(&buf[..n]);
                let text = String::from_utf8_lossy(&acc);
                if text.contains(needle) {
                    return text.into_owned();
                }
            }
            Ok(_) => {
                // Ok(0) — shouldn't happen with blocking reads but count it
                timeout_count += 1;
                std::thread::sleep(Duration::from_millis(20));
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                timeout_count += 1;
                std::thread::sleep(Duration::from_millis(20));
            }
            Err(ref e) => {
                err_count += 1;
                if first_err.is_none() {
                    first_err = Some(format!("{e} (kind: {:?})", e.kind()));
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        }
    }

    // Log diagnostics when needle was NOT found
    eprintln!("  {DIM}read_until_verbose({:?}): {} bytes, {} timeouts, {} errors{RESET}",
        needle, acc.len(), timeout_count, err_count);
    if let Some(e) = first_err {
        eprintln!("  {RED}  first read error: {e}{RESET}");
    }
    if !acc.is_empty() {
        let show = acc.len().min(80);
        eprintln!("  {DIM}  raw: {:?}{RESET}",
            String::from_utf8_lossy(&acc[..show]));
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

// ── Symlink + tty setup ─────────────────────────────────────

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

/// Configure the Linux tty for raw serial access.
///
/// Without this, `cat > /dev/ttyACM0` and `echo cmd > /dev/ttyACM0`
/// fail because the kernel tty layer defaults to canonical mode with
/// HUPCL (drops DTR on close, racing USB) and local echo (doubles
/// characters).  This persists until device re-enumeration.
fn configure_tty(port: &str) {
    let result = Command::new("stty")
        .args(["-F", port, "115200", "raw", "-echo", "-hupcl"])
        .output();
    match result {
        Ok(out) if out.status.success() => {
            eprintln!("  {DIM}stty: OK{RESET}");
        }
        Ok(out) => {
            let err = String::from_utf8_lossy(&out.stderr);
            eprintln!("{YELLOW}Warning: stty failed for {port}: {}{RESET}", err.trim());
        }
        Err(e) => {
            eprintln!("{YELLOW}Warning: stty exec failed: {e}{RESET}");
        }
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
