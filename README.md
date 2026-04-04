# LEGO SPIKE Prime Hub — RTIC v2 Firmware

Bare-metal Rust firmware for the **LEGO SPIKE Prime Hub** using **RTIC v2**.  
An interactive **Demon-style debug monitor** over USB CDC serial — poke  
registers, dump memory, control motors and sensors, upload binaries,  
set DWT hardware watchpoints, and connect GDB over RSP — all without a  
hardware debugger.

> **Status:** proof-of-concept, improving steadily.  
> **Firmware size:** ~140 KB of the available 992 KB internal flash.

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
# Build (from project root)
cargo build --release

# Convert to binary (ALWAYS to target/spike-rtic.bin)
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/spike-rtic \
    target/spike-rtic.bin

# Flash (enter DFU mode first — see USER_MANUAL §5.1)
dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D target/spike-rtic.bin

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

- **50+ shell commands** — memory dump, register inspect, motor/sensor control, SPI flash
- **RAM demo sandbox** — upload binaries via COBS, execute sandboxed (MPU + SVC) or privileged
- **DWT hardware watchpoints** — 4 self-hosted comparators, DebugMonitor exception, no JTAG needed
- **GDB RSP stub** — remote debug over USB CDC (memory/register read-write, watchpoints)
- **Unix-style process management** — kill -9/-2/-15, Ctrl-C, ring-button zones, pause/resume
- **MonitorApi v10** — 24-field callback table for demo↔firmware interface
- **32 MB external flash** — store/load/run demos from SPI flash
- **MPU guard bands** — stack overflow detection with fault marker surviving reset
- **65 automated tests** pass

---

## Project Structure

```
spike_rtic/
├── src/                  Firmware source (RTIC app, drivers, shell)
├── spike-hub-api/        Shared MonitorApi crate
├── examples/hub-ram-demos/  RAM demo binaries
├── helper-tools/         Python & shell scripts
├── dev_notes/            Design notes & analysis
├── AIcoder-thinkings/    AI agent memory
└── memory.x              Linker script
```

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