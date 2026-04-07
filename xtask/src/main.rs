//! SPIKE RTIC Hub Manager — interactive CLI for firmware workflows.
//!
//! Provides `cargo xtask` as the single entry-point for all hub operations:
//! flash firmware, build & upload demos, open the serial shell, check status.
//!
//! Also acts as the `cargo run` handler: when the ARM binary is built,
//! the runner script delegates here to offer a flash dialog.
//!
//! # Usage
//! ```text
//! cargo xtask              # interactive menu
//! cargo xtask flash        # build release + flash to hub
//! cargo xtask upload       # pick & upload a demo
//! cargo xtask upload NAME  # build + upload specific demo
//! cargo xtask connect      # open serial shell
//! cargo xtask status       # show hub state & tool availability
//! cargo xtask build        # build firmware (release)
//! cargo xtask build-demos  # build all demos
//! ```

use std::env;
use std::fs;
use std::io::{self, BufRead, Write};
use std::path::{Path, PathBuf};
use std::process::{exit, Command, Stdio};
use std::time::{Duration, Instant};

// ── ANSI Colors ─────────────────────────────────────────────
const RED: &str = "\x1b[91m";
const GREEN: &str = "\x1b[92m";
const YELLOW: &str = "\x1b[93m";
const CYAN: &str = "\x1b[96m";
const BOLD: &str = "\x1b[1m";
const DIM: &str = "\x1b[2m";
const RESET: &str = "\x1b[0m";

// ── Box-drawing (double-line) ───────────────────────────────
const BOX_W: usize = 49;

fn box_top() {
    println!("  ╔{}╗", "═".repeat(BOX_W));
}
fn box_mid() {
    println!("  ╠{}╣", "═".repeat(BOX_W));
}
fn box_bot() {
    println!("  ╚{}╝", "═".repeat(BOX_W));
}
fn box_row(text: &str) {
    let vis = visible_len(text);
    let pad = BOX_W.saturating_sub(vis + 1);
    println!("  ║ {}{} ║", text, " ".repeat(pad));
}

/// Compute display-width of a string, stripping ANSI escapes and
/// counting emoji (U+1F000+) as two columns.
fn visible_len(s: &str) -> usize {
    let mut len = 0usize;
    let mut in_esc = false;
    for c in s.chars() {
        if c == '\x1b' {
            in_esc = true;
        } else if in_esc {
            if c.is_ascii_alphabetic() {
                in_esc = false;
            }
        } else {
            len += if (c as u32) >= 0x1F000 { 2 } else { 1 };
        }
    }
    len
}

fn prompt(msg: &str) -> String {
    print!("  {msg}");
    io::stdout().flush().unwrap();
    let mut buf = String::new();
    io::stdin().lock().read_line(&mut buf).unwrap();
    buf.trim().to_string()
}

// ── Hub state detection ─────────────────────────────────────

#[derive(Debug)]
enum HubState {
    Running(String), // serial port path
    Dfu,
    Disconnected,
}

impl HubState {
    fn icon(&self) -> &str {
        match self {
            HubState::Running(_) => "🟢",
            HubState::Dfu => "🟡",
            HubState::Disconnected => "🔴",
        }
    }
    fn label(&self) -> String {
        match self {
            HubState::Running(p) => format!("Running  ({p})"),
            HubState::Dfu => "DFU mode  (ready to flash)".into(),
            HubState::Disconnected => "Disconnected".into(),
        }
    }
    fn color(&self) -> &str {
        match self {
            HubState::Running(_) => GREEN,
            HubState::Dfu => YELLOW,
            HubState::Disconnected => RED,
        }
    }
}

fn detect_hub() -> HubState {
    // 1. Look for ttyACM* with LEGO VID (0694)
    if let Ok(entries) = fs::read_dir("/dev") {
        let mut acm: Vec<String> = entries
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
        acm.sort();

        // Try to verify LEGO VID via sysfs
        for port in &acm {
            let dev = port.strip_prefix("/dev/").unwrap_or(port);
            // sysfs path: /sys/class/tty/<dev>/device/../idVendor
            let vid_path = format!("/sys/class/tty/{dev}/device/../idVendor");
            if let Ok(vid) = fs::read_to_string(&vid_path) {
                if vid.trim() == "0694" {
                    return HubState::Running(port.clone());
                }
            }
        }
        // Fallback: any ttyACM is probably the hub
        if let Some(port) = acm.first() {
            return HubState::Running(port.clone());
        }
    }

    // 2. Check lsusb for DFU mode (VID:PID 0694:0011)
    if let Ok(out) = Command::new("lsusb").output() {
        let text = String::from_utf8_lossy(&out.stdout).to_lowercase();
        if text.contains("0694:0011") {
            return HubState::Dfu;
        }
    }

    HubState::Disconnected
}

fn wait_for_dfu(timeout_secs: u64) -> bool {
    let deadline = Instant::now() + Duration::from_secs(timeout_secs);
    while Instant::now() < deadline {
        if matches!(detect_hub(), HubState::Dfu) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(500));
    }
    false
}

fn wait_for_serial(timeout_secs: u64) -> Option<String> {
    let deadline = Instant::now() + Duration::from_secs(timeout_secs);
    while Instant::now() < deadline {
        if let HubState::Running(port) = detect_hub() {
            return Some(port);
        }
        std::thread::sleep(Duration::from_millis(500));
    }
    None
}

// ── Project root ────────────────────────────────────────────

fn project_root() -> PathBuf {
    // 1. Honour PROJECT_ROOT if set and valid
    if let Ok(root) = env::var("PROJECT_ROOT") {
        let p = PathBuf::from(&root);
        if p.join(".git").is_dir() {
            return p;
        }
    }
    // 2. Walk up from CWD looking for .git
    if let Ok(cwd) = env::current_dir() {
        let mut dir = cwd.as_path();
        loop {
            if dir.join(".git").is_dir() {
                return dir.to_path_buf();
            }
            match dir.parent() {
                Some(p) => dir = p,
                None => break,
            }
        }
    }
    // 3. Fallback: CWD
    env::current_dir().unwrap_or_else(|_| PathBuf::from("."))
}

fn short_path(p: &Path) -> String {
    let s = p.display().to_string();
    if let Ok(home) = env::var("HOME") {
        if let Some(rest) = s.strip_prefix(&home) {
            return format!("~{rest}");
        }
    }
    s
}

fn rel_path(root: &Path, full: &Path) -> String {
    full.strip_prefix(root)
        .map(|p| p.display().to_string())
        .unwrap_or_else(|_| full.display().to_string())
}

fn format_size(bytes: u64) -> String {
    if bytes >= 1024 * 1024 {
        format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
    } else if bytes >= 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else {
        format!("{bytes} B")
    }
}

// ── Serial helper (uses Python/pyserial) ────────────────────

/// Configure the Linux tty for raw serial access so `cat`/`echo` work.
fn configure_tty(port: &str) {
    let _ = Command::new("stty")
        .args(["-F", port, "115200", "raw", "-echo", "-hupcl"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}

fn send_serial_cmd(port: &str, cmd: &str) {
    let script = format!(
        "import serial,time; s=serial.Serial('{}',115200,timeout=1); \
         s.write(b'{}\\r\\n'); time.sleep(0.5); s.close()",
        port, cmd
    );
    let _ = Command::new("python3")
        .args(["-c", &script])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}

// ── objcopy + flash helpers ─────────────────────────────────

fn objcopy(root: &Path, elf: &Path) -> Option<PathBuf> {
    let stem = elf.file_stem().unwrap().to_string_lossy();
    let bin = root.join("target").join(format!("{stem}.bin"));

    println!("  Converting ELF → .bin ...");
    let status = Command::new("arm-none-eabi-objcopy")
        .args(["-O", "binary"])
        .arg(elf)
        .arg(&bin)
        .status();

    match status {
        Ok(s) if s.success() => {
            let sz = fs::metadata(&bin).map(|m| m.len()).unwrap_or(0);
            println!(
                "  {GREEN}✓{RESET} {}: {}",
                rel_path(root, &bin),
                format_size(sz)
            );
            Some(bin)
        }
        Ok(s) => {
            println!(
                "  {RED}✗ objcopy failed (exit {}){RESET}",
                s.code().unwrap_or(-1)
            );
            None
        }
        Err(e) => {
            println!("  {RED}✗ arm-none-eabi-objcopy not found:{RESET} {e}");
            println!("    Install: sudo apt install binutils-arm-none-eabi");
            None
        }
    }
}

fn dfu_flash(_root: &Path, bin: &Path) {
    let hub = detect_hub();

    let ready = match &hub {
        HubState::Dfu => true,
        HubState::Running(port) => {
            println!("  Hub is running — sending 'dfu' command...");
            send_serial_cmd(port, "dfu");
            println!("  Waiting for DFU mode...");
            std::thread::sleep(Duration::from_secs(2));
            if matches!(detect_hub(), HubState::Dfu) {
                true
            } else {
                println!("  {YELLOW}Hub didn't switch. Hold center button for DFU.{RESET}");
                wait_for_dfu(20)
            }
        }
        HubState::Disconnected => {
            println!("  {RED}Hub not connected.{RESET}");
            println!("  Power on and hold center button for DFU mode.");
            println!("  Waiting up to 30 s...");
            wait_for_dfu(30)
        }
    };

    if !ready {
        println!("  {RED}✗ Timed out waiting for DFU.{RESET}");
        return;
    }

    let sz = fs::metadata(bin).map(|m| m.len()).unwrap_or(0);
    println!("  Flashing {} to 0x08008000...", format_size(sz));

    let status = Command::new("dfu-util")
        .args(["-d", "0694:0011", "-a", "0", "-s", "0x08008000:leave", "-D"])
        .arg(bin)
        .stdin(Stdio::inherit())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status();

    match status {
        Ok(s) if s.success() => {
            println!("  {GREEN}✓ Flash complete — hub will reboot.{RESET}");
            println!("  Waiting for serial...");
            if let Some(port) = wait_for_serial(20) {
                configure_tty(&port);
                println!("  {GREEN}✓ Hub ready on {port}{RESET}");
            }
        }
        Ok(s) => println!(
            "  {RED}✗ dfu-util failed (exit {}){RESET}",
            s.code().unwrap_or(-1)
        ),
        Err(_) => {
            println!("  {RED}✗ dfu-util not found.{RESET}");
            println!("    Install: sudo apt install dfu-util");
        }
    }
}

// ── Subcommands ─────────────────────────────────────────────

fn interactive_menu(root: &Path) {
    loop {
        let hub = detect_hub();

        println!();
        box_top();
        box_row(&format!("{BOLD}🔧 SPIKE RTIC Hub Manager{RESET}"));
        box_mid();
        box_row(&format!(
            "Hub:  {} {}{}{}",
            hub.icon(),
            hub.color(),
            hub.label(),
            RESET
        ));
        box_row(&format!("Root: {CYAN}{}{RESET}", short_path(root)));
        box_mid();
        box_row(&format!("{BOLD}[1]{RESET} Flash firmware to hub"));
        box_row(&format!("{BOLD}[2]{RESET} Build & upload a demo"));
        box_row(&format!("{BOLD}[3]{RESET} Open hub shell"));
        box_row(&format!("{BOLD}[4]{RESET} Hub status / diagnostics"));
        box_row(&format!("{BOLD}[5]{RESET} Build firmware (release)"));
        box_row(&format!("{BOLD}[6]{RESET} Build all demos"));
        box_row(&format!("{DIM}[q] Quit{RESET}"));
        box_bot();

        let choice = prompt("Choose [1-6/q]: ");
        println!();
        match choice.as_str() {
            "1" => cmd_flash(root),
            "2" => cmd_upload(root, &[]),
            "3" => cmd_connect(root),
            "4" => cmd_status(root),
            "5" => cmd_build_firmware(root),
            "6" => cmd_build_demos(root),
            "q" | "Q" | "" => {
                println!("  {DIM}Goodbye.{RESET}");
                break;
            }
            _ => println!("  {RED}Invalid choice.{RESET}"),
        }
    }
}

fn cmd_run_elf(root: &Path, args: &[String]) {
    let elf_str = args.first().unwrap_or_else(|| {
        eprintln!("  {RED}run-elf requires ELF path{RESET}");
        exit(1);
    });
    let elf = Path::new(elf_str);

    let size = fs::metadata(elf).map(|m| m.len()).unwrap_or(0);
    let hub = detect_hub();
    let is_firmware = elf_str.contains("spike-rtic");

    println!();
    box_top();
    box_row(&format!("{BOLD}{GREEN}✓ Build successful{RESET}"));
    box_mid();
    box_row(&format!("ELF:  {CYAN}{}{RESET}", rel_path(root, elf)));
    box_row(&format!("Size: {}", format_size(size)));
    box_row(&format!(
        "Hub:  {} {}{}{}",
        hub.icon(),
        hub.color(),
        hub.label(),
        RESET
    ));
    box_mid();

    if is_firmware {
        box_row(&format!(
            "{BOLD}[1]{RESET} Flash to hub (objcopy + dfu-util)"
        ));
        box_row(&format!("{BOLD}[2]{RESET} Convert to .bin only"));
        box_row(&format!("{BOLD}[m]{RESET} Open full menu"));
        box_row(&format!("{DIM}[q] Done{RESET}"));
        box_bot();

        let choice = prompt("Choose [1-2/m/q]: ");
        println!();
        match choice.as_str() {
            "1" => {
                if let Some(bin) = objcopy(root, elf) {
                    dfu_flash(root, &bin);
                }
            }
            "2" => {
                objcopy(root, elf);
            }
            "m" | "M" => interactive_menu(root),
            _ => {}
        }
    } else {
        box_row(&format!("{BOLD}[1]{RESET} Upload & run on hub"));
        box_row(&format!("{BOLD}[2]{RESET} Convert to .bin only"));
        box_row(&format!("{BOLD}[m]{RESET} Open full menu"));
        box_row(&format!("{DIM}[q] Done{RESET}"));
        box_bot();

        let choice = prompt("Choose [1-2/m/q]: ");
        println!();
        match choice.as_str() {
            "1" => {
                if let Some(bin) = objcopy(root, elf) {
                    upload_bin(root, &bin);
                }
            }
            "2" => {
                objcopy(root, elf);
            }
            "m" | "M" => interactive_menu(root),
            _ => {}
        }
    }
}

fn cmd_flash(root: &Path) {
    println!("  Building firmware (release)...");
    let status = Command::new("cargo")
        .args(["build", "--release"])
        .current_dir(root)
        .stdin(Stdio::inherit())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status();

    if !status.map(|s| s.success()).unwrap_or(false) {
        println!("  {RED}✗ Build failed{RESET}");
        return;
    }
    println!("  {GREEN}✓ Build OK{RESET}");

    let elf = root.join("target/thumbv7em-none-eabihf/release/spike-rtic");
    if let Some(bin) = objcopy(root, &elf) {
        dfu_flash(root, &bin);
    }
}

fn cmd_upload(root: &Path, args: &[String]) {
    let demo_dir = root.join("examples/hub-ram-demos/examples");

    let demo_name = if let Some(name) = args.first() {
        name.clone()
    } else {
        pick_demo(&demo_dir)
    };
    if demo_name.is_empty() {
        return;
    }

    println!("  Building demo: {CYAN}{demo_name}{RESET}...");
    let status = Command::new("cargo")
        .args(["build", "--release", "--example", &demo_name])
        .current_dir(root.join("examples/hub-ram-demos"))
        .stdin(Stdio::inherit())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status();

    if !status.map(|s| s.success()).unwrap_or(false) {
        println!("  {RED}✗ Demo build failed{RESET}");
        return;
    }
    println!("  {GREEN}✓ Build OK{RESET}");

    let elf = root.join(format!(
        "examples/hub-ram-demos/target/thumbv7em-none-eabihf/release/examples/{demo_name}"
    ));
    let bin = PathBuf::from(format!("/tmp/{demo_name}.bin"));

    println!("  Converting to .bin...");
    let objcopy_ok = Command::new("arm-none-eabi-objcopy")
        .args(["-O", "binary"])
        .arg(&elf)
        .arg(&bin)
        .status()
        .map(|s| s.success())
        .unwrap_or(false);

    if !objcopy_ok {
        println!("  {RED}✗ objcopy failed{RESET}");
        return;
    }
    let sz = fs::metadata(&bin).map(|m| m.len()).unwrap_or(0);
    println!("  {GREEN}✓{RESET} /tmp/{demo_name}.bin: {}", format_size(sz));

    upload_bin(root, &bin);
}

fn pick_demo(demo_dir: &Path) -> String {
    let mut demos: Vec<String> = fs::read_dir(demo_dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            name.strip_suffix(".rs").map(|s| s.to_string())
        })
        .collect();
    demos.sort();

    if demos.is_empty() {
        println!("  {RED}No demos found in {}{RESET}", demo_dir.display());
        return String::new();
    }

    println!("  {BOLD}Available demos:{RESET}");
    println!("  {}", "─".repeat(44));

    let mid = (demos.len() + 1) / 2;
    for i in 0..mid {
        let left = format!("{:>3}. {}", i + 1, demos[i]);
        let right = if i + mid < demos.len() {
            format!("{:>3}. {}", i + mid + 1, demos[i + mid])
        } else {
            String::new()
        };
        println!("  {:<26}{}", left, right);
    }
    println!();

    let input = prompt("Enter number or name: ");

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

    println!("  {RED}Demo not found: {input}{RESET}");
    String::new()
}

fn upload_bin(root: &Path, bin: &Path) {
    let script = root.join("helper-tools/upload_demo.py");
    if !script.exists() {
        println!("  {RED}✗ helper-tools/upload_demo.py not found{RESET}");
        return;
    }

    let hub = detect_hub();
    match hub {
        HubState::Running(_) => {
            println!("  Uploading & running...");
            println!();
            let _ = Command::new("python3")
                .arg(&script)
                .arg(bin)
                .stdin(Stdio::inherit())
                .stdout(Stdio::inherit())
                .stderr(Stdio::inherit())
                .status();
        }
        _ => {
            println!("  {RED}Hub not ready — need running firmware for upload.{RESET}");
            println!("  Flash firmware first: cargo xtask flash");
        }
    }
}

fn cmd_connect(root: &Path) {
    let hub = detect_hub();
    match hub {
        HubState::Running(ref port) => {
            // Configure tty for raw access (so cat/echo also work after exiting)
            configure_tty(port);
            println!("  Connecting to hub on {CYAN}{port}{RESET}...");
            println!("  {DIM}(Ctrl-C to exit){RESET}");
            println!();

            // Try picocom → miniterm → spike_hub_controller.py
            let picocom_ok = Command::new("which")
                .arg("picocom")
                .output()
                .map(|o| o.status.success())
                .unwrap_or(false);
            if picocom_ok {
                let _ = Command::new("picocom")
                    .args(["-b", "115200", port])
                    .stdin(Stdio::inherit())
                    .stdout(Stdio::inherit())
                    .stderr(Stdio::inherit())
                    .status();
                return;
            }

            let miniterm_ok = Command::new("python3")
                .args(["-c", "import serial.tools.miniterm"])
                .output()
                .map(|o| o.status.success())
                .unwrap_or(false);
            if miniterm_ok {
                let _ = Command::new("python3")
                    .args(["-m", "serial.tools.miniterm", port, "115200"])
                    .stdin(Stdio::inherit())
                    .stdout(Stdio::inherit())
                    .stderr(Stdio::inherit())
                    .status();
                return;
            }

            let shbc = root.join("helper-tools/spike_hub_controller.py");
            if shbc.exists() {
                let _ = Command::new("python3")
                    .arg(&shbc)
                    .arg("shell")
                    .stdin(Stdio::inherit())
                    .stdout(Stdio::inherit())
                    .stderr(Stdio::inherit())
                    .status();
                return;
            }

            println!("  {RED}No serial terminal found.{RESET}");
            println!("    Install: sudo apt install picocom");
            println!("    Or:      pip install pyserial");
        }
        HubState::Dfu => {
            println!("  {YELLOW}Hub is in DFU mode — can't open shell.{RESET}");
            println!("  Flash firmware first, or power-cycle without holding the button.");
        }
        HubState::Disconnected => {
            println!("  {RED}Hub not connected.{RESET}");
        }
    }
}

fn cmd_status(root: &Path) {
    let hub = detect_hub();

    println!("  {BOLD}Hub Status{RESET}");
    println!("  {}", "─".repeat(44));
    println!(
        "  State: {} {}{}{}",
        hub.icon(),
        hub.color(),
        hub.label(),
        RESET
    );

    // Tool availability
    println!();
    println!("  {BOLD}Tools{RESET}");
    println!("  {}", "─".repeat(44));
    for tool in [
        "arm-none-eabi-objcopy",
        "dfu-util",
        "python3",
        "picocom",
        "cargo",
    ] {
        let ok = Command::new("which")
            .arg(tool)
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false);
        let icon = if ok {
            format!("{GREEN}✓{RESET}")
        } else {
            format!("{RED}✗{RESET}")
        };
        println!("  {icon} {tool}");
    }

    // pyserial
    let pyserial = Command::new("python3")
        .args(["-c", "import serial; print(serial.VERSION)"])
        .output();
    match pyserial {
        Ok(o) if o.status.success() => {
            let ver = String::from_utf8_lossy(&o.stdout).trim().to_string();
            println!("  {GREEN}✓{RESET} pyserial {ver}");
        }
        _ => println!("  {RED}✗{RESET} pyserial  (pip install pyserial)"),
    }

    // Project info
    println!();
    println!("  {BOLD}Project{RESET}");
    println!("  {}", "─".repeat(44));
    println!("  Root:         {}", root.display());
    println!(
        "  PROJECT_ROOT: {}",
        env::var("PROJECT_ROOT").unwrap_or_else(|_| "(auto-detected)".into())
    );

    // Count demos
    let demo_dir = root.join("examples/hub-ram-demos/examples");
    if let Ok(entries) = fs::read_dir(&demo_dir) {
        let count = entries
            .flatten()
            .filter(|e| e.file_name().to_string_lossy().ends_with(".rs"))
            .count();
        println!("  Demos:        {count} available");
    }

    // Pre-built bins
    let bin_dir = root.join("examples/hub-ram-demos/target/spike-usr_bins");
    if bin_dir.is_dir() {
        if let Ok(entries) = fs::read_dir(&bin_dir) {
            let count = entries
                .flatten()
                .filter(|e| e.file_name().to_string_lossy().ends_with(".bin"))
                .count();
            println!("  Pre-built:    {count} .bin files in spike-usr_bins/");
        }
    }

    println!();
}

fn cmd_build_firmware(root: &Path) {
    println!("  Building firmware (release)...");
    let status = Command::new("cargo")
        .args(["build", "--release"])
        .current_dir(root)
        .stdin(Stdio::inherit())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status();

    match status {
        Ok(s) if s.success() => println!("  {GREEN}✓ Firmware build OK{RESET}"),
        _ => println!("  {RED}✗ Build failed{RESET}"),
    }
}

fn cmd_build_demos(root: &Path) {
    println!("  Building all demos (release)...");
    let status = Command::new("cargo")
        .args(["build", "--release", "--examples"])
        .current_dir(root.join("examples/hub-ram-demos"))
        .stdin(Stdio::inherit())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status();

    match status {
        Ok(s) if s.success() => println!("  {GREEN}✓ All demos built{RESET}"),
        _ => println!("  {RED}✗ Demo build failed{RESET}"),
    }
}

fn usage() {
    println!("{BOLD}SPIKE RTIC Hub Manager{RESET}");
    println!();
    println!("Usage: cargo xtask [command]");
    println!();
    println!("Commands:");
    println!("  {CYAN}(none){RESET}       Interactive menu");
    println!("  {CYAN}flash{RESET}        Build release + flash to hub");
    println!("  {CYAN}upload{RESET} [N]   Build & upload a demo");
    println!("  {CYAN}connect{RESET}      Open hub serial shell");
    println!("  {CYAN}status{RESET}       Show hub state & tool availability");
    println!("  {CYAN}build{RESET}        Build firmware (release)");
    println!("  {CYAN}build-demos{RESET}  Build all demos");
    println!();
}

// ── Entry point ─────────────────────────────────────────────

fn main() {
    let root = project_root();
    // Export PROJECT_ROOT for all child processes
    env::set_var("PROJECT_ROOT", &root);

    let args: Vec<String> = env::args().skip(1).collect();

    match args.first().map(|s| s.as_str()) {
        None | Some("--help") | Some("-h") => {
            if args.is_empty() {
                interactive_menu(&root);
            } else {
                usage();
            }
        }
        Some("run-elf") => cmd_run_elf(&root, &args[1..]),
        Some("flash") => cmd_flash(&root),
        Some("upload") => cmd_upload(&root, &args[1..]),
        Some("connect") => cmd_connect(&root),
        Some("status") => cmd_status(&root),
        Some("build") => cmd_build_firmware(&root),
        Some("build-demos") => cmd_build_demos(&root),
        Some(other) => {
            eprintln!("  {RED}Unknown command: {other}{RESET}");
            usage();
            exit(1);
        }
    }
}
