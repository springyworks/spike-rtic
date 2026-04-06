# hub_state.py -- Hub Connection State Detector

Detects the SPIKE Prime hub's connection state and optionally
enters GDB RSP mode automatically. Kills processes blocking the
serial port instead of making you do it manually.

## States

| Code | Name             | Meaning                                       |
| ---- | ---------------- | --------------------------------------------- |
| 0    | `disconnected`   | No LEGO USB device (check cable / power)      |
| 1    | `dfu`            | Hub in DFU bootloader (VID:PID 0694:0011)     |
| 2    | `serial-shell`   | Shell responds with `spike>` prompt            |
| 3    | `serial-gdb`     | Hub in GDB RSP mode (Demon mode)              |
| 4    | `serial-unknown` | Serial present, unrecognized response          |
| 5    | `serial-busy`    | Port locked -- auto-kills the blocking process |

Exit code matches the state number, so scripts can branch on `$?`.

## Usage

```bash
# Just detect and print state:
python3 helper-tools/hub_state.py

# Enter RSP mode + launch gdb-multiarch (all-in-one):
python3 helper-tools/hub_state.py gdb

# Same, but with debug symbols from a demo ELF:
python3 helper-tools/hub_state.py gdb \
  examples/hub-ram-demos/target/thumbv7em-none-eabihf/release/examples/gdb_simple

# Reconnect to hub already in RSP mode (after serial drop):
python3 helper-tools/hub_state.py gdb-reconnect

# Reconnect with symbols:
python3 helper-tools/hub_state.py gdb-reconnect \
  examples/hub-ram-demos/target/thumbv7em-none-eabihf/release/examples/gdb_simple
```

## What each action does

### `hub_state.py` (no args)

1. Checks pyserial for LEGO runtime CDC (VID 0694, PID 0042)
2. If found, opens port, sends `\r\n\r\n`, waits 0.8s
3. Classifies response: `spike>` = shell, silence/`+` = GDB RSP
4. If port can't open: finds + kills blocking process via `fuser`
5. Falls back to `lsusb` check for DFU mode
6. Prints state, exits with state code

### `hub_state.py gdb [ELF]`

1. Detects state (as above)
2. If port busy: kills blocking process, re-probes
3. If shell: sends `gdb\r\n` to enter RSP mode
4. If already in RSP: skips straight to GDB
5. Launches: `gdb-multiarch [ELF] -ex "set architecture arm" -ex "target remote /dev/ttyACMx"`

### `hub_state.py gdb-reconnect [ELF]`

1. Detects state, kills blockers if needed
2. Launches GDB without sending the `gdb` command (hub stays in current mode)
3. Use this when the USB cable was unplugged/replugged while hub is still in RSP

## Typical GDB session

```bash
# Build + objcopy the demo
cd examples/hub-ram-demos
cargo build --example gdb_simple --release
arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/examples/gdb_simple \
  target/spike-usr_bins/gdb_simple.bin
cd ../..

# Upload the demo
python3 helper-tools/upload_demo.py \
  examples/hub-ram-demos/target/spike-usr_bins/gdb_simple.bin

# One command: detect state, enter RSP, launch GDB with symbols
python3 helper-tools/hub_state.py gdb \
  examples/hub-ram-demos/target/thumbv7em-none-eabihf/release/examples/gdb_simple
```

Then in `(gdb)`:

```gdb
x/4xw 0x20040000              # read start of demo code in SRAM2
info registers                 # show r0-r15, cpsr
watch *(uint32_t*)0x2004XXXX   # DWT write watchpoint (use printed addr)
continue                       # run until watchpoint fires
detach                         # return to shell
```

## Port-busy auto-kill

When the serial port is locked (e.g. a stale `picocom`, `screen`,
or previous `gdb-multiarch` that didn't exit cleanly), `hub_state.py`
automatically:

1. Runs `fuser /dev/ttyACMx` (falls back to `lsof -t`)
2. Identifies the blocking PID and process name
3. Sends `SIGKILL` (kill -9)
4. Waits 0.5s, re-probes the port

If `kill` fails due to permissions, it prints the manual command.

## Requirements

- Python 3.6+
- `pyserial` (`pip install pyserial`)
- `gdb-multiarch` (for the `gdb` action)
- `fuser` or `lsof` (for port-busy detection -- usually pre-installed)
