# How the hub handles both GDB and CodeLLDB

**The hub doesn't know or care which client is talking to it.** It speaks one protocol: **GDB Remote Serial Protocol (RSP)**. That's it.

## The protocol stack

```
CodeLLDB (VS Code)  ─┐
                      ├─► RSP packets over TCP ─► socat ─► /dev/spike-gdb (ttyACM1) ─► firmware RSP stub
gdb-multiarch         ─┘
```

Both clients send the exact same RSP packets (`$g#67`, `$Z0,addr,kind#xx`, `$c#63`, etc.) over the same TCP socket to `localhost:3333`.

## What CodeLLDB actually is

CodeLLDB is **LLDB** (the LLVM debugger) with a VS Code GUI wrapper. LLDB has a built-in GDB RSP client — when you say `target remote localhost:3333`, LLDB speaks the same RSP protocol as gdb-multiarch. It sends `qSupported`, `?`, `g`, `Z0`, `c`, `s` — the same packets.

The differences are cosmetic:
- **GDB** might send `vCont;c` while **LLDB** might send `c` (or vice versa)
- **LLDB** might query different `qSupported` features
- The `qXfer:features:read:target.xml` parsing differs slightly
- LLDB may use register number 25 for xPSR while GDB uses 16

Our stub handles all these variations — that's why the `p` handler accepts both register 16 and 25 for xPSR, and we support both `c` and `vCont;c`.

## Why CodeLLDB "adds features"

CodeLLDB doesn't add features to the protocol — it adds features **on top**:
- Variable inspection, hover, watch panel → just `p` (register read) and `m` (memory read) packets
- Pretty-printing Rust types → LLDB reads DWARF debug info from the ELF, reads raw memory via `m`, then formats it
- Call stack → `g` (registers) + `m` (stack memory) + DWARF unwind info from ELF
- Source mapping → DWARF line tables in the ELF, no protocol needed

All the "smart" stuff happens client-side. The hub just answers "give me register N" and "give me memory at address X".

## The stale T05 race fix

The fix we landed (checking `debugmon_count` before reporting halts) was the only thing preventing gdb-multiarch from working. CodeLLDB probably avoided it by slightly different packet timing — LLDB may have had a longer delay between sending `c` and polling for the response, giving the DebugMonitor handler time to clear `TARGET_HALTED` before the next USB ISR poll.

## TL;DR

The hub is a dumb RSP server. GDB, LLDB, CodeLLDB, and even a Python script sending raw `$c#63` over a socket all work the same way.
