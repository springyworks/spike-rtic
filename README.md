# LEGO SPIKE Prime Hub — RTIC v2 Firmware

[User Manual](USER_MANUAL.md) · [Reference Manual](REFERENCE_MANUAL.md) · [API Reference](spike-hub-api/README.md) · [RAM Demos](examples/hub-ram-demos/README.md) · [Helper Tools](helper-tools/README.md) · [Dev Notes](dev_notes/)

---

Bare-metal Rust firmware for the **LEGO SPIKE Prime Hub** using **RTIC v2**.  
An interactive **Demon-style debug monitor** over USB CDC serial — poke  
registers, dump memory, control motors and sensors, upload binaries,  
set DWT hardware watchpoints, and connect GDB over RSP — all without a  
hardware debugger.

> **Status:** proof-of-concept, improving steadily.  
> **Firmware size:** ~153 KB of the available 992 KB internal flash.

---

## Documentation


| Document                                                 | Contents                                                                                                  |
| -------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| **[USER_MANUAL.md](USER_MANUAL.md)**                     | Quick start, shell commands, process management, RAM demos, DFU recovery, Python tools, known limitations |
| **[REFERENCE_MANUAL.md](REFERENCE_MANUAL.md)**           | Hardware specs, memory map, architecture, MonitorApi callback table, DWT/GDB internals, design philosophy |
| [hub-ram-demos README](examples/hub-ram-demos/README.md) | Writing and building RAM demo binaries                                                                    |
| [MonitorApi README](spike-hub-api/README.md)             | Shared API crate for firmware↔demo interface                                                              |
| [dev_notes/](dev_notes/)                                 | Design notes, protocol analysis, motor/sensor deep-dives                                                  |


---

## Quick Start

```bash
# Build the firmware
cd $PROJECT_ROOT
cargo build --release

# Convert to binary (ALWAYS to target/spike-rtic.bin)
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/spike-rtic \
    target/spike-rtic.bin

# Flash (enter DFU mode first — see USER_MANUAL §5.1)
dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D $PROJECT_ROOT/target/spike-rtic.bin

# Connect
picocom /dev/ttyACM0
```

Type `help` at the `spike>` prompt.  See [USER_MANUAL.md](USER_MANUAL.md)  
for the full walkthrough.

---

## Hardware


| Component      | Details                                   |
| -------------- | ----------------------------------------- |
| MCU            | STM32F413VGT6 (Cortex-M4F @ 96 MHz)       |
| Internal Flash | 1 MB (32 KB bootloader + 992 KB firmware) |
| Internal RAM   | 320 KB (256 KB SRAM1 + 64 KB SRAM2)       |
| External Flash | 32 MB Winbond W25Q256JV (SPI2)            |
| Display        | 5×5 LED matrix (TLC5955 SPI1)             |
| Motors         | 6× H-bridge PWM (ports A–F)               |
| Sensors        | 6× LPF2 LUMP UART                         |
| USB            | CDC serial (0x0694:0x0042)                |


Full hardware details in [REFERENCE_MANUAL.md §1](REFERENCE_MANUAL.md#1-hardware).

---

## Features at a Glance

- **50+ [shell commands](USER_MANUAL.md#2-shell-commands)** — memory dump, register inspect, motor/sensor control, SPI flash
- **[RAM demo sandbox](USER_MANUAL.md#4-ram-demos)** — upload binaries via COBS, execute sandboxed (MPU + SVC) or privileged
- **[DWT hardware watchpoints](USER_MANUAL.md#27-dwt-watchpoints)** — 4 self-hosted comparators, DebugMonitor exception, no JTAG needed
- **[GDB RSP stub](USER_MANUAL.md#28-gdb-remote-debug)** — remote debug over USB CDC: continue, step, halt, registers, memory, watchpoints, backtrace — no JTAG needed
- **LLDB / [CodeLLDB support](USER_MANUAL.md#vs-code--codelldb-extension)** — same RSP stub works with LLDB (vCont, QStartNoAckMode, sequential registers)
- **[VS Code F5 debugging](examples/hub-ram-demos/xtask/README.md#the-debug-pipeline)** — automated build → upload → GDB/LLDB attach pipeline (xtask preLaunchTask)
- **Bidirectional demo I/O** — host→demo text channel via `send` shell command + [EVT_INPUT events](spike-hub-api/README.md#2-api-fields)
- **[6-DOF IMU driver](USER_MANUAL.md#29-system--misc)** — LSM6DS3TR-C accel+gyro via I2C2 bit-bang with bus-reset recovery
- **[Unix-style process management](USER_MANUAL.md#3-process-management)** — kill -9/-2/-15, Ctrl-C, ring-button zones, pause/resume
- **[MonitorApi v12](spike-hub-api/README.md)** — 26-field callback table for demo↔firmware interface
- **[32 MB external flash](USER_MANUAL.md#26-external-flash-spi2)** — store/load/run demos from SPI flash
- **[MPU guard bands](REFERENCE_MANUAL.md#5-safety--sandbox)** — stack overflow detection with fault marker surviving reset
- **65 automated tests** pass

---

## Project Structure

```
spike_rtic/
├── src/                    Firmware source (RTIC app, drivers, shell)
├── spike-hub-api/          Shared MonitorApi crate
├── examples/hub-ram-demos/ RAM demo binaries
│   └── xtask/              Demo build/upload/debug CLI
├── xtask/                  Firmware build/flash CLI
├── helper-tools/           Python & shell scripts
├── dev_notes/              Design notes & analysis
└── memory.x                Linker script
```

| Directory | README | Description |
| --- | --- | --- |
| `src/` | — | RTIC firmware: [shell](REFERENCE_MANUAL.md#31-source-tree), [sandbox](REFERENCE_MANUAL.md#5-safety--sandbox), [DWT](REFERENCE_MANUAL.md#36-dwt-self-hosted-watchpoints), [GDB RSP](REFERENCE_MANUAL.md#37-gdb-rsp-stub) |
| `spike-hub-api/` | [README](spike-hub-api/README.md) | MonitorApi callback table ([§3.3](REFERENCE_MANUAL.md#33-monitorapi--callback-table)) |
| `examples/hub-ram-demos/` | [README](examples/hub-ram-demos/README.md) | RAM demo binaries — [writing demos](examples/hub-ram-demos/README.md#4-writing-a-new-demo), [available demos](examples/hub-ram-demos/README.md#3-available-demos) |
| `examples/hub-ram-demos/xtask/` | [README](examples/hub-ram-demos/xtask/README.md) | Demo CLI — build, upload, debug pipeline |
| `xtask/` | [README](xtask/README.md) | Firmware CLI — build, flash, connect, status |
| `helper-tools/` | [README](helper-tools/README.md) | Python scripts — [upload](helper-tools/README.md#core-tools), [test suite](helper-tools/README.md#tests), [debug pipeline](helper-tools/README.md#core-tools) |
| `dev_notes/` | [README](dev_notes/README.md) | Design notes — [sensor](dev_notes/sensor-fixes.md), [motor](dev_notes/motor-position-controller.md), [GDB](dev_notes/gdb-debugging.md), [LUMP](dev_notes/pybricks-lump-protocol-reference.md) |

---

## Credits

- Pin mappings from [pybricks-micropython](https://github.com/pybricks/pybricks-micropython)
- [LUMP protocol reference](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/port_lump.c)
- RTIC people, and all Rust giants out there ; [https://github.com/rtic-rs/rtic.git](https://github.com/rtic-rs/rtic.git)
- MRI - Monitor for Remote Inspection ; [https://github.com/adamgreen/mri.git](https://github.com/adamgreen/mri.git)
- Claude Opus 4.6 (March 2026) — MCP-assisted firmware development and thus a lot of anonymous human intellect used , credits to them also.

---

*This is a proof-of-concept that works.  Read [USER_MANUAL.md §7*](USER_MANUAL.md#7-known-limitations)  
*for honest limitations before building on it.*