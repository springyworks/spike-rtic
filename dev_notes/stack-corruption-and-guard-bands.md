# Stack Corruption, Guard Bands, and the Limits of SoC Reliability

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---


Technical analysis of stack overflow risks in the SPIKE RTIC firmware,
Rust's async/await stackless advantage, MPU-based guard bands,
and why general-purpose SoCs are unsuitable for safety-critical devices.

---

## 1. The Stack Corruption Problem in Bare-Metal Systems

On a Cortex-M4 with no OS, the hardware stack pointer (MSP or PSP) grows
downward through RAM with **no hardware limit check**. When a function
call chain or interrupt nesting pushes the stack pointer past the end of
its intended region, it silently overwrites whatever lies below:

- RTIC task state (`.bss` / `.data`)
- DMA buffers
- Ring buffers (trace, USB)
- Other tasks' saved async state machines

The corruption is **silent**. The processor does not fault. Symptoms may
appear hundreds of milliseconds later as wrong sensor readings, motor
glitches, USB protocol errors, or sudden HardFaults with misleading
stacked PCs. Debugging such failures is extremely difficult because the
cause and effect are separated in both time and address space.

### Why traditional C/RTOS stacks make this worse

In FreeRTOS or similar, each task has a fixed-size stack (e.g., 512 bytes).
If task A overflows, it corrupts task B's stack — not its own. The
corrupted task may not fault until its next context switch restores
garbage from the damaged stack frame. Stack overflow hooks
(`vApplicationStackOverflowHook`) only catch the overflow **after** the
damage is done, and only if the topmost canary word was hit.

---

## 2. Rust Async/Await: The Stackless Advantage

RTIC v2 uses Rust's `async`/`await` model, which compiles each async
task into a **state machine struct** stored in a static RTIC resource —
not on a runtime stack.

### How it works

```rust
#[task(priority = 2, shared = [motor_cfg])]
async fn pid_run(mut cx: pid_run::Context) {
    loop {
        // ... compute PID ...
        Mono::delay(20.millis()).await;   // <── yield point
    }
}
```

At each `.await`, the compiler transforms the function into an enum
whose variants hold the live local variables at each yield point. This
enum is stored in a **static**, compiler-sized allocation — its size is
known at compile time and cannot grow at runtime.

**Consequence:** An async task that `.await`s in a loop does **not**
accumulate stack frames. Each iteration reuses the same state machine
storage. In contrast, a C RTOS task running an equivalent loop would
keep its full stack allocated for the task's entire lifetime, whether
active or blocked.

### What still uses the hardware stack

Even with async tasks, the hardware stack (MSP) is used for:

1. **The synchronous prefix of each poll** — local variables in the
   currently executing code between two `.await` points.
2. **Interrupt/exception entry** — Cortex-M hardware pushes 8 words
   (32 bytes) per exception frame, more with FP context.
3. **Nested preemption** — priority 3 UART ISR preempts priority 2
   firmware task, which preempts priority 1 user task. Each level
   adds one exception frame plus the ISR's own locals.
4. **Inline assembly** — the `run_sandboxed()` block, SVC handler,
   and MemManage handler all have significant stack footprints.

The async model dramatically **reduces** the peak MSP depth compared to
an equivalent RTOS design, but does not eliminate stack usage entirely.

### Quantifying the current firmware's stack budget

```
SRAM1 total                     256 KB  (0x2000_0000 – 0x2003_FFFF)
.data + .bss (firmware globals)  ~20 KB  (varies with build)
Available for MSP               ~236 KB
───────────────────────────────────────
Worst-case nesting:
  idle (Thread)                  ~100 B
  + priority 1 (run_demo)       ~200 B
  + priority 2 (sensor_poll)   ~1000 B
  + priority 3 (UART ISR)       ~200 B
  + exception frame × 3          ~96 B
  ─────────────────────────────────────
  Peak estimate                 ~1.6 KB
```

With ~236 KB available and ~1.6 KB peak usage, the firmware has roughly
a **150× safety margin** on the main stack. This is why no overflow has
been observed so far — but "has not been observed" is not a guarantee.

---

## 3. MPU Guard Bands: Catching Overflow Before Corruption

An MPU guard band is a small memory region (typically 32–256 bytes)
placed at the bottom of the stack's intended range, configured as
**no-access** in the MPU. If the stack pointer grows into this region,
the next push triggers a **MemManage fault** instead of silently
corrupting whatever lies below.

### Principle

```
High address
  ┌───────────────────────┐  ← _stack_start (MSP initial value)
  │  Stack (grows down)   │
  │         ...           │
  │         ...           │
  ├───────────────────────┤  ← stack limit (intended bottom)
  │  GUARD BAND (32 B)    │  ← MPU region: no access, any privilege
  ├───────────────────────┤
  │  .bss / .data         │  ← firmware globals
  └───────────────────────┘
Low address
```

Any write into the guard band — from privileged or unprivileged code —
generates an immediate, synchronous MemManage fault with the faulting
address in the MMFAR register. The fault handler can:

1. Log the fault address and stacked PC
2. Attempt a controlled shutdown (power off motors, LEDs off)
3. Trigger a system reset (write AIRCR.SYSRESETREQ)

### Why 32 bytes is the minimum practical guard

Cortex-M4's MPU v7 requires region sizes that are powers of two, minimum
32 bytes, aligned to their own size. A 32-byte guard catches any
sequential stack growth (push, function prologue). It does **not** catch
a large `alloca`-style jump that skips over the guard — but Rust does
not have `alloca`, and stack frames are statically sized.

### Implementation sketch for the SPIKE firmware

The firmware currently configures 5 MPU regions (0–4). Cortex-M4 has 8
regions total, so regions 5–7 are available. A guard band would use one:

```
Region 5: Stack guard
  Base:  0x2000_4FE0   (just above .bss/.data end, aligned to 32)
  Size:  32 bytes
  AP:    0b000  (no access, any privilege level)
  XN:    1
  TEX/S/C/B:  normal memory
```

The exact base address depends on the link-time size of `.bss` + `.data`.
It can be computed via a linker symbol:

```
/* In memory.x */
_stack_guard_bottom = ALIGN(ADDR(.bss) + SIZEOF(.bss), 32);
```

Then in `mpu_configure()`:

```rust
// Region 5: Stack guard band — 32 bytes, no access
let guard_base = unsafe { &_stack_guard_bottom as *const u32 as u32 };
core::ptr::write_volatile(MPU_RNR, 5);
core::ptr::write_volatile(MPU_RBAR, guard_base);
core::ptr::write_volatile(MPU_RASR,
    AP_NONE | NORMAL_MEM | RASR_XN | size_field(4) | RASR_ENABLE);
// size_field(4) = 2^5 = 32 bytes
```

### Guard band for the demo PSP (SRAM2)

The demo stack occupies the top 4 KB of SRAM2 (`0x2004_E000` –
`0x2004_F000`, PSP initialized to `0x2004_F000`). Below that lies the
demo's code and data. A guard band at `0x2004_DFE0` (32 bytes) would
catch demo stack overflow before it corrupts the demo's own `.text`.

This is already partially handled: if the demo's PSP descends below
`0x2004_0000` (SRAM2 base), it hits SRAM1's priv-only MPU region and
faults. But growth from `0x2004_E000` into the demo's own code section
goes undetected, because MPU region 0 grants full RW to all of SRAM2.

A dedicated guard region would close this gap:

```
Region 6: Demo stack guard
  Base:  0x2004_DFE0
  Size:  32 bytes
  AP:    0b000  (no access)
```

---

## 4. The Firmware MSP Guard: What to Do When It Fires

A MemManage fault on the **firmware** MSP (not the demo PSP) means the
monitor itself has overflowed. This is categorically different from a
demo fault:

- **Demo fault:** sandbox catches it, reports error, firmware continues.
- **Firmware fault:** the monitor's own state is potentially corrupted.
  No part of the system can be trusted.

### The only safe response: reset

When the MSP guard band is hit:

1. Power off motor PWM (write TIM registers directly — no function calls
   that might touch corrupted memory)
2. Write a fault marker to a reserved RAM address that survives reset
   (e.g., last 4 bytes of SRAM1 before the stack, or the DFU magic area)
3. Trigger `SCB.AIRCR = 0x05FA_0004` (system reset request)

After reset, the firmware's `init()` can check the fault marker and
print a diagnostic: "Reset after stack overflow at 0x200XXXXX."

**Do not** attempt to continue operation. The stack corruption may have
damaged RTIC dispatcher state, async task futures, or ring buffer
metadata. Any continued operation may produce incorrect motor commands,
missed sensor keepalives, or silent data corruption.

---

## 5. Rust Async Does Not Eliminate All Risk

Although Rust's async state machines do not grow the stack, the following
scenarios can still cause stack overflow in the SPIKE firmware:

### a) Deep synchronous call chains between await points

If a non-async helper function called from an async task has deep
recursion or large local arrays, those frames accumulate on MSP during
that synchronous segment. The LUMP handshake state machine and the shell
command parser are the deepest synchronous chains in the current firmware.

### b) Interrupt nesting at the worst moment

Priority 3 UART ISR preempts priority 2 firmware tasks, which preempt
priority 1 user tasks. If all three levels are active simultaneously,
their combined frames share the one MSP. Adding a priority 4 ISR (e.g.,
for DMA) would increase worst-case depth.

### c) FPU context stacking

If the FPU is active, Cortex-M4 automatically pushes 18 additional words
(72 bytes) per exception frame for lazy FPU context saving. The PID
controller and servo observer use floating-point math. A UART ISR
preempting mid-float-operation adds 72 bytes to the exception frame.

### d) Compiler-generated stack usage changes per build

Different optimization levels, LTO settings, or code changes can alter
function frame sizes unpredictably. A change to `sensor_poll` that adds
one more local variable may push the worst-case depth past a threshold
that was previously safe.

---

## 6. Why General-Purpose SoCs Cannot Be Trusted for Safety-Critical Devices

The SPIKE Prime hub uses an STM32F413 — a general-purpose Cortex-M4
microcontroller. This class of SoC is explicitly **not suitable** for
safety-critical applications such as pacemakers, automotive braking, or
flight control. The reasons are architectural, not merely regulatory.

### a) The processor is too flexible

A general-purpose SoC executes arbitrary instruction sequences. Any
firmware bug — a wrong pointer, a missing bounds check, a race condition
— can cause the processor to execute unintended code paths, including
writing to peripheral control registers that drive physical actuators.
The MPU mitigates this for unprivileged code, but the firmware itself
runs privileged and can corrupt anything.

Safety-critical processors (e.g., TI TMS570, Infineon AURIX) use
**lockstep dual-core execution**: two identical cores run the same code
in parallel, and a hardware comparator resets the system if their outputs
diverge. A single-core Cortex-M4 has no such redundancy.

### b) No hardware stack limit enforcement

As described above, the Cortex-M4 MSP has no hardware bounds register.
The stack can grow into any RAM address without triggering a fault
unless an MPU region is explicitly configured as a guard. Even with a
guard, a sufficiently large stack frame can skip over the guard region
entirely (though Rust's lack of `alloca` makes this unlikely).

ARM's Cortex-M33 (ARMv8-M) adds **stack limit registers** (MSPLIM,
PSPLIM) that trigger a UsageFault when the stack pointer goes below the
configured limit. The STM32F413's Cortex-M4 (ARMv7-M) does not have
these registers.

### c) No ECC on SRAM

The STM32F413's SRAM has no error-correcting code (ECC) protection. A
single-bit flip from radiation, power glitch, or marginal voltage causes
silent data corruption. Safety-critical microcontrollers (e.g., STM32L5
series, TMS570) include ECC on all SRAM banks, detecting and correcting
single-bit errors and detecting double-bit errors.

In the SPIKE firmware, a bit flip in the RTIC dispatcher's task queue
could cause a task to fire at the wrong time, be skipped entirely, or
execute with corrupted arguments.

### d) Clock and power integrity assumptions

The STM32F413 trusts that its clock source (HSE, PLL) and supply voltage
are within spec. A power supply glitch can cause:

- Incorrect instruction fetch (executing wrong opcode)
- SRAM corruption (incomplete write)
- Flash read error (bit flip in fetched instruction)
- Peripheral register corruption (wrong timer period → wrong PWM)

Safety-critical designs use redundant clock monitors (the STM32F413 has
CSS — Clock Security System — but it only detects total HSE failure, not
drift or jitter) and brown-out detectors with fast response. The
STM32F413's BOR (Brown-Out Reset) is relatively coarse.

### e) No temporal isolation

On the Cortex-M4, a runaway ISR can starve all lower-priority tasks
indefinitely. The watchdog timer provides a coarse timeout (seconds), but
a safety-critical system requires **microsecond-level temporal
guarantees**. This is why automotive AUTOSAR mandates hardware timing
protection units that can interrupt and kill individual tasks after
specific deadlines.

### f) Implications for the SPIKE firmware

None of this means the SPIKE firmware is poorly designed. For its purpose
— an interactive debug monitor for a LEGO robotics hub — the reliability
level is appropriate. The watchdog catches lockups, the MPU sandbox
catches demo faults, and the async model minimizes stack pressure.

But if the same hardware and firmware architecture were proposed for a
medical implant or a vehicle braking controller, the answer would be:
**no**. The SoC lacks the hardware redundancy, memory protection, and
deterministic execution guarantees required by IEC 61508 (functional
safety) or IEC 62304 (medical device software).

---

## 7. Recommended Improvements for the SPIKE Firmware

These are practical, low-risk additions — not speculative features.

### a) Add MSP guard band (MPU region 5)

Cost: ~15 lines of code in `mpu_configure()` and `memory.x`.
Requires: one additional MPU region (3 of 8 currently unused).
Benefit: converts silent MSP overflow into an immediate, diagnosable
MemManage fault with clean reset.

### b) Add PSP guard band for demos (MPU region 6)

Cost: ~10 lines of code. Activate only when sandbox is enabled.
Benefit: catches demo stack overflow before it corrupts demo code/data,
instead of waiting for the cruder "fell out of SRAM2" detection.

### c) Paint the stack with a sentinel pattern at boot

At `init()`, fill the MSP region with a known pattern (e.g., `0xDEAD_BEEF`).
Periodically (in `heartbeat`), scan upward from the guard band to find
the high-water mark. Report it via the `info` shell command.

Cost: ~20 lines. No runtime overhead except the periodic scan.
Benefit: provides empirical worst-case stack depth data without requiring
static analysis tools.

### d) Implement MemManage handler for MSP faults

The current `MemoryManagement` handler only handles sandbox (PSP) faults.
Add a branch: if `SANDBOXED` is false, the fault is on the firmware MSP.
In that case, disable motors, log the fault address, and reset.

### e) Consider `-Z stack-usage` during development

Rust nightly supports `-Z emit-stack-sizes`, which emits per-function
stack frame sizes into the ELF. Combined with `cargo-call-stack`, this
gives a static worst-case stack depth analysis. Use it as a CI check:
fail the build if worst-case depth exceeds a threshold.

---

## 8. Summary

| Mechanism | Status in firmware | Risk addressed |
|-----------|-------------------|----------------|
| Async/await stackless tasks | **Active** | Eliminates per-task stack allocation |
| MPU sandbox for demos | **Active** | Demo cannot corrupt SRAM1 |
| Watchdog timer | **Active** | Catches total lockup (5 s) |
| MSP guard band | **Not implemented** | Silent MSP overflow |
| PSP guard band (demo) | **Not implemented** | Demo stack into demo code |
| Stack painting + high-water | **Not implemented** | Empirical depth measurement |
| MSP fault → reset path | **Not implemented** | Clean recovery from firmware overflow |
| Static stack analysis (CI) | **Not implemented** | Compile-time depth verification |

The firmware is well-protected against **demo-induced faults** (MPU +
MemManage + abort landing). The residual risk is **firmware-side MSP
overflow**, which is unlikely given the 150× safety margin but remains
undetected if it occurs. Adding an MPU guard band (region 5) and a
reset-on-MSP-fault path would close this gap with minimal code.
