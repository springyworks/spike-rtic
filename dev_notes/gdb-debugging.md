# GDB Remote Debugging — Architecture & Lessons Learned

Self-hosted GDB debugging over USB CDC serial — no JTAG, no SWD,
no external debug probe.  The hub acts as both target and debug
server simultaneously.

---

## Architecture Overview

The debug stack has three layers:

1. **DebugMonitor exception** (`dwt.rs`) — Cortex-M4 hardware halts
   the target at watchpoints/breakpoints/single-step, then waits in
   a WFE loop for GDB to inspect and resume.
2. **GDB RSP stub** (`gdb_rsp.rs`) — Parses GDB Remote Serial Protocol
   packets over USB CDC, translates them into register/memory reads,
   watchpoint configuration, and DebugMonitor control.
3. **Pipeline tools** (`debug_pipeline.py`, `enter_rsp.py`) — Python
   scripts that upload the demo, enter RSP mode, and release the port
   for GDB.

```
┌─────────────────────────────────────────────────┐
│   GDB (host)         gdb-multiarch               │
│     ↕ RSP packets over USB CDC serial             │
│   Shell (hub)        shell.rs routes to gdb_rsp   │
│     ↕ register/memory reads, DWT config           │
│   DebugMonitor (hub) dwt.rs exception handler     │
│     ↕ WFE halt / resume / single-step             │
│   Demo (hub)         SRAM2 user code              │
└─────────────────────────────────────────────────┘
```

---

## RTIC Priority Mapping

This is the most critical design constraint.  DebugMonitor must be at
the **lowest** hardware priority so it never blocks firmware tasks.

| RTIC priority | HW priority byte | Tasks                        |
| ------------- | ---------------- | ---------------------------- |
| 3             | 0xD0             | UART ISRs (sensor)           |
| 2             | 0xE0             | USB ISR, heartbeat, motor, sensor, shell |
| 1             | 0xF0             | run_demo, test_all           |
| —             | 0xF0             | **DebugMonitor** (same as pri 1) |

Formula: `hw_priority = (16 - rtic_priority) << 4`

DebugMonitor at 0xF0 means:
- All firmware tasks (pri 2+) preempt it → USB, shell, watchdog keep running
- run_demo (pri 1) is at the same level → DebugMonitor can halt the demo
  because they don't preempt each other
- Watchdog feeding continues via heartbeat (pri 2) while demo is halted

**Earlier bug:** DebugMonitor was initially 0xE0 (same as USB ISR).
Same-priority exceptions can't preempt each other → DebugMonitor blocked
all firmware → watchdog crash.

---

## Trampoline Design (LR Preservation)

The DebugMonitor handler is a naked function that must preserve
EXC_RETURN in LR across the `bl` call to the Rust handler.

```asm
push {r4-r11, lr}      // save callee-saved + EXC_RETURN
bl   debug_monitor_handler
pop  {r4-r11, lr}      // restore EXC_RETURN into LR
bx   lr                 // exception return
```

**Critical lesson:** `bl` clobbers LR with the return address.  If you
only `push {r4-r11}`, the EXC_RETURN value (0xFFFFFFFD for Thread mode
PSP) is lost.  After `pop {r4-r11}`, `bx lr` jumps to the middle of the
handler instead of performing exception return → HardFault or corruption.

The MSP frame offset for reading the stacked context is **+36** (9 words:
r4-r11 + lr), not +32.

---

## GDB_ACTIVE Guard

Problem: When GDB disconnects (close port, `quit` without `detach`),
no RSP packet is sent.  The DebugMonitor is still armed and will WFE-halt
the next time a watchpoint fires — but nobody is listening to resume it.
Result: watchdog timeout → crash.

Solution: `GDB_ACTIVE` atomic flag in `dwt.rs`:
- Set to 1 when `gdb_rsp::enter()` is called
- Cleared by `cleanup_for_detach()` on `D`/`k` packets or auto-detach
- DebugMonitor handler checks `GDB_ACTIVE` before entering WFE halt
- If not active: clears MON_STEP, MON_PEND, and returns immediately

Auto-detach triggers:
- RSP `D` (detach) or `k` (kill) packets → explicit cleanup
- USB bus disconnect (`usb_disconnected()` in `shell.rs`) → `auto_detach()`
- Triple Ctrl-C in RSP mode → exit to shell

**Not** triggered by DTR drop — DTR drops when *any* program closes the
port, including the pipeline script handing off to GDB.  DTR-based detach
broke the pipeline→GDB handoff.

---

## DebugMonitor Halt/Resume Protocol

When DebugMonitor fires (watchpoint hit, single-step, or pending):

1. Handler saves all registers (r0-r12, sp, lr, pc, xpsr) from the
   exception frame into static atomics
2. Sets `REGS_VALID = 1`
3. Checks `GDB_ACTIVE` — if zero, clears MON_STEP/MON_PEND, returns
4. Enters WFE loop: `while TARGET_HALTED.load() != 0 { wfe; }`
5. On resume: checks `SINGLE_STEP_REQUEST` → sets MON_STEP in DEMCR
6. Returns (exception return resumes the demo)

GDB side (`gdb_rsp.rs`):
- `c` (continue): calls `dwt::resume_target()` which clears
  `TARGET_HALTED` and executes `sev` to wake the WFE loop
- `s` (step): sets `SINGLE_STEP_REQUEST`, then resumes
- Register reads (`g`): reads from `SAVED_REGS` atomics

---

## Workflow

### Automated (VS Code F5)

`tasks.json` defines a dependency chain:
1. `build-gdb-exercise` — cargo build
2. `objcopy-gdb-exercise` — extract .bin
3. `upload-and-enter-rsp` — `debug_pipeline.py` uploads, sends `go` +
   `gdb`, releases port
4. `launch.json` cppdbg config connects to `/tmp/spike-hub`

### Manual (two terminals)

```bash
# Terminal 1: upload + enter RSP
python3 helper-tools/debug_pipeline.py \
    examples/hub-ram-demos/target/spike-usr_bins/gdb_exercise.bin \
    /dev/ttyACM0

# Terminal 2: connect GDB
gdb-multiarch \
    examples/hub-ram-demos/target/thumbv7em-none-eabihf/release/examples/gdb_exercise \
    -ex "set architecture arm" \
    -ex "target remote /dev/ttyACM0"
```

### GDB commands that work

```gdb
info registers          # r0-r15, cpsr
x/4xw 0x20040000        # read SRAM2
bt                       # backtrace (with symbols)
continue                 # resume target
# Ctrl-C                 # break into halted state
watch *(uint32_t*)ADDR   # DWT write watchpoint
detach                   # clean exit → hub returns to shell
```

---

## Verified Test Session (2025-07-01)

Full clean cycle with no crashes:

1. `debug_pipeline.py` → upload + go + gdb → port released
2. `gdb-multiarch` → `target remote /dev/ttyACM0` → connected
3. `info registers` → valid r0-r15 (PSP in SRAM2 range), xpsr
4. `bt` → `gdb_exercise::_start at gdb_exercise.rs:201`
5. `continue` → demo runs → SIGTRAP (SVC in delay_ms)
6. `continue` → demo continues
7. Ctrl-C → SIGINT (clean break)
8. `quit` → detach → hub back at `spike>` shell
9. No watchdog crash, no corruption.

---

## Known Limitations

- **FPB covers Flash only** — FPBv1 range is 0x0000_0000–0x1FFF_FFFF.
  SRAM2 demos run at 0x2004_xxxx, outside FPB range.  Software
  breakpoints (BKPT instruction patching) work in SRAM2 instead.
- **No thread awareness** — GDB sees one "thread" (the DebugMonitor
  context).  RTIC tasks are not exposed as separate threads.
- **Register snapshot is from exception entry** — r0-r12 come from the
  stacked frame at the moment DebugMonitor fired, not from arbitrary
  points during execution.
- **SVC triggers SIGTRAP** — The demo's `delay_ms` uses SVC, which
  GDB reports as SIGTRAP.  Just `continue` past it.
- **Single serial port** — GDB and the shell share USB CDC.  The shell
  routes bytes to the RSP parser when in GDB mode.  You can't use
  picocom and GDB simultaneously.
