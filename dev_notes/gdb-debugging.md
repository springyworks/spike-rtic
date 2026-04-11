# GDB / LLDB Remote Debugging — Architecture & Lessons Learned

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---

Self-hosted debugging over USB CDC serial — no JTAG, no SWD,  
no external debug probe. The hub acts as both target and debug  
server simultaneously. Works with **gdb-multiarch** (cppdbg) and  
**LLDB** (CodeLLDB) in VS Code.

> **Status: alpha** — register read, memory read/write, software  
> breakpoints, watchpoints, continue, step, halt, and detach all  
> work. No FPU regs, no thread awareness.

---

## Architecture Overview

The debug stack has three layers:

1.  **DebugMonitor exception** (`dwt.rs`) — Cortex-M4 hardware halts  
    the target at watchpoints/breakpoints/single-step, then waits in  
    a WFE loop for GDB to inspect and resume.
2.  **GDB RSP stub** (`gdb_rsp.rs`) — Parses GDB Remote Serial Protocol  
    packets over USB CDC, translates them into register/memory reads,  
    watchpoint configuration, and DebugMonitor control. Supports both  
    GDB and LLDB protocols (vCont, QStartNoAckMode, sequential register  
    numbering).
3.  **Pipeline tool** (`examples/hub-ram-demos/xtask/`) — Rust xtask  
    that builds, objcopy's, uploads the demo, enters RSP mode, and  
    releases the port for GDB/LLDB. Auto-kills port-blocking processes  
    and adapts to any hub state (shell, RSP, unknown).

```
┌───────────────────────────────────────────────────┐
│   GDB/LLDB (host)    gdb-multiarch or CodeLLDB    │
│     ↕ RSP packets over USB CDC serial             │
│   Shell (hub)        shell.rs routes to gdb_rsp   │
│     ↕ register/memory reads, DWT config           │
│   DebugMonitor (hub) dwt.rs exception handler     │
│     ↕ WFE halt / resume / single-step             │
│   Demo (hub)         SRAM2 user code              │
└───────────────────────────────────────────────────┘
```

---

## RTIC Priority Mapping

This is the most critical design constraint. DebugMonitor must be at  
the **lowest** hardware priority so it never blocks firmware tasks.

| RTIC priority | HW priority byte | Tasks |
| --- | --- | --- |
| 3 | 0xD0 | UART ISRs (sensor) |
| 2 | 0xE0 | USB ISR, heartbeat, motor, sensor, shell |
| 1 | 0xF0 | run\_demo, test\_all |
| — | 0xF0 | **DebugMonitor** (same as pri 1) |

Formula: `hw_priority = (16 - rtic_priority) << 4`

DebugMonitor at 0xF0 means:

*   All firmware tasks (pri 2+) preempt it → USB, shell, watchdog keep running
*   run\_demo (pri 1) is at the same level → DebugMonitor can halt the demo  
    because they don't preempt each other
*   Watchdog feeding continues via heartbeat (pri 2) while demo is halted

**Earlier bug:** DebugMonitor was initially 0xE0 (same as USB ISR).  
Same-priority exceptions can't preempt each other → DebugMonitor blocked  
all firmware → watchdog crash.

---

## Trampoline Design (LR Preservation)

The DebugMonitor handler is a naked function that must preserve  
EXC\_RETURN in LR across the `bl` call to the Rust handler.

```
push {r4-r11, lr}      // save callee-saved + EXC_RETURN
bl   debug_monitor_handler
pop  {r4-r11, lr}      // restore EXC_RETURN into LR
bx   lr                 // exception return
```

**Critical lesson:** `bl` clobbers LR with the return address. If you  
only `push {r4-r11}`, the EXC\_RETURN value (0xFFFFFFFD for Thread mode  
PSP) is lost. After `pop {r4-r11}`, `bx lr` jumps to the middle of the  
handler instead of performing exception return → HardFault or corruption.

The MSP frame offset for reading the stacked context is **+36** (9 words:  
r4-r11 + lr), not +32.

---

## GDB\_ACTIVE Guard

Problem: When GDB disconnects (close port, `quit` without `detach`),  
no RSP packet is sent. The DebugMonitor is still armed and will WFE-halt  
the next time a watchpoint fires — but nobody is listening to resume it.  
Result: watchdog timeout → crash.

Solution: `GDB_ACTIVE` atomic flag in `dwt.rs`:

*   Set to 1 when `gdb_rsp::enter()` is called
*   Cleared by `cleanup_for_detach()` on `D`/`k` packets or auto-detach
*   DebugMonitor handler checks `GDB_ACTIVE` before entering WFE halt
*   If not active: clears MON\_STEP, MON\_PEND, and returns immediately

Auto-detach triggers:

*   RSP `D` (detach) or `k` (kill) packets → explicit cleanup
*   USB bus disconnect (`usb_disconnected()` in `shell.rs`) → `auto_detach()`
*   Triple Ctrl-C in RSP mode → exit to shell

**Not** triggered by DTR drop — DTR drops when _any_ program closes the  
port, including the pipeline script handing off to GDB. DTR-based detach  
broke the pipeline→GDB handoff.

---

## DebugMonitor Halt/Resume Protocol

When DebugMonitor fires (watchpoint hit, single-step, or pending):

1.  Handler saves all registers (r0-r12, sp, lr, pc, xpsr) from the  
    exception frame into static atomics
2.  Sets `REGS_VALID = 1`
3.  Checks `GDB_ACTIVE` — if zero, clears MON\_STEP/MON\_PEND, returns
4.  Enters WFE loop: `while TARGET_HALTED.load() != 0 { wfe; }`
5.  On resume: checks `SINGLE_STEP_REQUEST` → sets MON\_STEP in DEMCR
6.  Returns (exception return resumes the demo)

GDB side (`gdb_rsp.rs`):

*   `c` (continue): calls `dwt::resume_target()` which clears  
    `TARGET_HALTED` and executes `sev` to wake the WFE loop
*   `s` (step): sets `SINGLE_STEP_REQUEST`, then resumes
*   Register reads (`g`): reads from `SAVED_REGS` atomics

---

## Workflow

### Automated (VS Code F5)

The Rust xtask handles the full pipeline as a VS Code preLaunchTask:

1.  `xtask debug <name>` — build + objcopy + upload + go + enter RSP
2.  Creates `/tmp/spike-hub` symlink → actual serial port
3.  `launch.json` connects GDB or LLDB to the symlink

Two generic launch configs work for **any** demo:

*   **"Debug demo (LLDB)"** — CodeLLDB; uses `process connect --plugin gdb-remote serial:///tmp/spike-hub?baud=115200`
*   **"Debug demo (GDB)"** — cppdbg/gdb-multiarch; uses `target remote /tmp/spike-hub`

Both prompt for the demo name (e.g. `gdb_simple`, `color_seeker`).  
Quick-launch configs (no prompt) exist for `gdb_simple` and `gdb_exercise`.

### Manual (two terminals)

```
# Terminal 1: build + upload + enter RSP
cd examples/hub-ram-demos/xtask
cargo run -- debug gdb_simple

# Terminal 2a: connect GDB
gdb-multiarch \
    ../target/thumbv7em-none-eabihf/release/examples/gdb_simple \
    -ex "set serial baud 115200" \
    -ex "target remote /tmp/spike-hub"

# Terminal 2b: OR connect LLDB
lldb \
    -o "target create ../target/thumbv7em-none-eabihf/release/examples/gdb_simple" \
    -o "process connect --plugin gdb-remote serial:///tmp/spike-hub?baud=115200"
```

### GDB commands that work

```
info registers          # r0-r15, cpsr
x/4xw 0x20040000        # read SRAM2
bt                       # backtrace (with symbols)
continue                 # resume target
# Ctrl-C                 # break into halted state
watch *(uint32_t*)ADDR   # DWT write watchpoint
detach                   # clean exit → hub returns to shell
```

### LLDB commands that work

```
register read            # r0-r15, xpsr (all 17 registers)
memory read 0x20040000 0x20040020
bt                       # backtrace
continue                 # resume
process interrupt        # halt
process detach           # clean exit
```

---

## Verified Test Session — GDB (2025-07-01)

Full clean cycle with no crashes:

1.  `debug_pipeline.py` → upload + go + gdb → port released
2.  `gdb-multiarch` → `target remote /dev/ttyACM0` → connected
3.  `info registers` → valid r0-r15 (PSP in SRAM2 range), xpsr
4.  `bt` → `gdb_exercise::_start at gdb_exercise.rs:201`
5.  `continue` → demo runs → SIGTRAP (SVC in delay\_ms)
6.  `continue` → demo continues
7.  Ctrl-C → SIGINT (clean break)
8.  `quit` → detach → hub back at `spike>` shell
9.  No watchdog crash, no corruption.

## Verified Test Session — LLDB (2026-04-07)

Full clean cycle with CodeLLDB (lldb-18) — batch mode:

1.  `xtask debug gdb_simple` → build + upload + RSP (~3 seconds)
2.  `lldb -b -s test.txt` → `process connect serial:///dev/ttyACM0?baud=115200`
3.  `register read` → all 17 registers (r0-r15 + xpsr) with valid values
4.  `memory read` → 32 bytes from SRAM at 0x20000000
5.  `bt` → backtrace with frame #0
6.  `process detach` → "Process 1 detached" → clean exit
7.  Hub returned to shell mode, no watchdog crash.
8.  Also tested via VS Code GUI (CodeLLDB extension) — source-level  
    display with green line indicator working.

---

## LLDB / CodeLLDB Compatibility

The RSP stub supports LLDB in addition to GDB. Key differences handled:

| Feature | GDB | LLDB | Stub support |
| --- | --- | --- | --- |
| Register numbering | regnum="25" for cpsr | Sequential 0–16 | Both accepted (p handler checks 16 AND 25) |
| Continue | `c` | `vCont;c` | Both handled identically |
| Single step | `s` | `vCont;s` | Both handled |
| No-ack mode | Not used | `QStartNoAckMode` | Supported — suppresses +/- |
| Memory reads | Small | Up to 512 bytes | Capped to 260 bytes (stack buffer) |

qSupported advertises: `PacketSize=1024;swbreak+;hwbreak+;qXfer:features:read+;QStartNoAckMode+;vContSupported+`

TARGET\_XML uses sequential register numbering (0–16, no gaps) which  
satisfies both GDB and LLDB.

---

## Lessons Learned — Buffer Sizes

**Memory read overflow (caught 2026-04-07):** The `m` (memory read)  
packet handler had a stack buffer of 520 bytes (260 data bytes × 2  
hex chars). But `max_bytes` was calculated from `RESP_BUF_SIZE`  
(1024), allowing up to 507 bytes — LLDB's `m addr,200` (512 bytes)  
overflowed the stack buffer → HardFault → watchdog reset.

**Fix:** Cap `max_bytes` by `buf.len() / 2` — always bounded by the  
actual stack buffer, not the response buffer.

**Rule:** In ISR context, stack space is precious. Always size your  
cap from the actual buffer, not from an unrelated constant.

---

## Known Limitations

*   **Alpha status** — works for interactive debugging but may have edge  
    cases. Tested with GDB 13 and LLDB 18.
*   **FPB covers Flash only** — FPBv1 range is 0x0000\_0000–0x1FFF\_FFFF.  
    SRAM2 demos run at 0x2004\_xxxx, outside FPB range. Software  
    breakpoints (BKPT instruction patching) work in SRAM2 instead.
*   **No FPU registers** — only r0-r15 + xPSR. No s0-s31/d0-d15.
*   **No thread awareness** — GDB/LLDB sees one "thread" (the DebugMonitor  
    context). RTIC tasks are not exposed as separate threads.
*   **Register snapshot is from exception entry** — r0-r12 come from the  
    stacked frame at the moment DebugMonitor fired, not from arbitrary  
    points during execution.
*   **SVC triggers SIGTRAP** — The demo's `delay_ms` uses SVC, which  
    GDB reports as SIGTRAP. Just `continue` past it.
*   **Single serial port** — GDB and the shell share USB CDC. The shell  
    routes bytes to the RSP parser when in GDB mode. You can't use  
    picocom and GDB simultaneously.
*   **Memory read capped at 260 bytes** — per-packet limit to avoid  
    stack overflow in ISR context. GDB/LLDB transparently split larger  
    reads into multiple packets.