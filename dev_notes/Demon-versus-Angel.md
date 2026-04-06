# ARM Demon and Its Relationship to the SPIKE RTIC Firmware

---

# ARM Demon — The Debug Monitor the SPIKE Firmware Recreates

The SPIKE RTIC firmware describes its shell as an "Angel‑style debug monitor," but functionally the design is _much closer to ARM Demon_ than to Angel. The following sections explain why.

---

# What ARM Demon Was

Demon was:

*   A **ROM‑resident monitor** running inside the SoC
*   A **serial shell** for direct operator interaction
*   A tool for **md/mw**, register dumps, and memory pokes
*   Capable of **downloading and executing RAM code**
*   An implementation of the **RDP protocol** (GDB's `target rdp`)
*   Usable with **no JTAG** or external debugger
*   Booted straight into a **command prompt**

The SPIKE firmware implements the same workflow:

*   `md`, `mw`, `regs`, `gpio`, `clocks`, `crc`, etc.
*   Upload binary → run from RAM (`go` / `go!`); unprivileged or privileged
*   Interactive shell remains active while user code runs
*   No external debugger required
*   The monitor itself serves as the operating environment

This is Demon's philosophy, faithfully reproduced.

---

# Demon vs Angel vs the SPIKE RTIC Monitor

| Feature | **ARM Demon** | **ARM Angel** | **SPIKE RTIC Monitor** |
| --- | --- | --- | --- |
| Shell on target | **Yes** | No | **Yes** |
| Memory poke/dump | **Yes** | Limited | **Yes** |
| Run RAM code | **Yes** | Yes (via ADP) | **Yes** (`go`, `go!`) |
| Protocol | RDP | ADP (Angel Debug Protocol) | Custom ASCII + COBS |
| Semihosting | No | **Yes** | No |
| Boot behavior | Prompt immediately | Wait for debugger | Prompt immediately |
| Philosophy | "Standalone monitor" | "Debugger stub" | "Standalone monitor" |

The SPIKE RTIC design is a **modern Rust re‑imagining of ARM Demon**, not Angel.

---

# Why Demon Existed

Before JTAG/SWD was inexpensive or standardized, ARM needed a way to:

*   Bring up new silicon
*   Test memory maps
*   Run small test programs
*   Debug without hardware probes

Demon lived in ROM and provided a **command-line operating environment** for the chip. The SPIKE firmware fulfills the same role for the STM32F413.

---

# Demon Command Set (Historical Reconstruction)

Typical Demon commands (varied by board):

*   `MD addr [count]` — memory dump
*   `MW addr value` — memory write
*   `RD` — register dump
*   `GO addr` — execute code
*   `LOAD` — receive binary (X/Y-modem or RDP)
*   `BP addr` — set breakpoint
*   `HELP` — list commands

The SPIKE firmware recreates all of these and adds:

*   MPU sandbox
*   COBS upload
*   Trace buffer
*   Motor/sensor drivers
*   Process management
*   RTTY output
*   External SPI flash access

---

# Architectural Evolution: Demon + 30 Years

The SPIKE monitor extends the Demon concept with several modern capabilities:

### MPU Sandboxing

Demon had none. The SPIKE firmware's `go` vs `go!` distinction provides OS‑grade memory protection for user code.

### Callback Table ABI (MonitorApi)

Demon offered no stable ABI for user code. The SPIKE firmware exposes a versioned `#[repr(C)]` callback table.

### COBS Upload + CRC‑32

Demon used X/Y-modem or raw serial. The SPIKE firmware's COBS framing with CRC‑32 verification is more robust.

### Background Shell While Code Runs

Demon typically blocked during code execution. The SPIKE firmware keeps the shell responsive via RTIC async tasks.

### Trace Buffer

Demon had no tracing facility. The SPIKE firmware provides a 2 KB atomic ring buffer for fast, non-blocking trace output.

### Motor/Sensor Subsystems

Demon was never required to drive LEGO motors at 100 Hz or maintain LUMP keepalive for external sensors.

---

# Areas for Historical Reconstruction

The following aspects of Demon's internals can be reconstructed from available documentation:

*   The RDP packet format
*   Demon's breakpoint implementation
*   Exception handling in Demon
*   GDB's interface to Demon via `target rdp`
*   Porting Demon to ARM6/ARM7 boards
*   Differences between Demon and the earlier "ARM Debug Monitor"

---

# Potential Extensions Toward a More Demon‑Like Design

### 1\. Remote Debug Protocol (RDP‑like)

A wire protocol enabling GDB to attach directly to the SPIKE monitor.

### 2\. Software Breakpoints

Inserting `BKPT` instructions into RAM demo code at runtime.

### 3\. Single‑Step via DebugMonitor

Cortex‑M supports monitor-mode debugging, which could be integrated into the existing exception handler infrastructure.

### 4\. Symbol-Aware Dumps

Parsing ELF symbol tables uploaded alongside binaries to provide symbolic memory inspection.

### 5\. Tiny File System in SPI Flash

Demon had no file system. The SPIKE firmware's access to external SPI flash makes a minimal FS feasible.