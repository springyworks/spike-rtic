# RTIC v2 Async/Await Patterns for SPIKE RTIC (2026-04-03)

[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---


## Our Setup

- **RTIC v2** with `rtic-monotonics` (SysTick at 1 kHz)
- All firmware tasks are `async fn` — RTIC v2's recommended pattern
- `Mono::delay(N.millis()).await` for all waits (no busy-waits in firmware)

## Basic Async Task Pattern

Every RTIC task is an `async fn` that loops with `Mono::delay().await`:

```rust
#[task(shared = [some_resource], priority = 2)]
async fn my_task(mut cx: my_task::Context) {
    loop {
        cx.shared.some_resource.lock(|res| {
            // do work with shared resource
        });
        Mono::delay(20.millis()).await;  // yield, let other tasks run
    }
}
```

## Spawning Multiple Concurrent Tasks

RTIC v2 tasks are **independently spawned** — each runs in its own async
executor slot.  "Fire and forget" spawning:

```rust
// In #[init] or from another task:
sensor_poll::spawn(PORT_E).ok();
motor_poll::spawn(PORT_A).ok();
heartbeat::spawn().ok();
// All three now run concurrently, preempted by priority
```

There is **no built-in `join!` or `select!`** in RTIC v2.  Instead:

### Pattern 1: Independent Periodic Tasks (what we use now)

Each task loops independently.  Coordination via shared resources + locks:

```rust
#[task(shared = [sensor_data], priority = 2)]
async fn sensor_poll(mut cx: sensor_poll::Context) {
    loop {
        let reading = read_sensor();
        cx.shared.sensor_data.lock(|d| *d = reading);
        Mono::delay(20.millis()).await;
    }
}

#[task(shared = [sensor_data, motor_cmd], priority = 2)]
async fn control_loop(mut cx: control_loop::Context) {
    loop {
        let data = cx.shared.sensor_data.lock(|d| *d);
        let cmd = compute(data);
        cx.shared.motor_cmd.lock(|m| *m = cmd);
        Mono::delay(10.millis()).await;
    }
}
```

### Pattern 2: Wait-for-Completion via Atomics

When you need "do A and B, then C after both finish":

```rust
use core::sync::atomic::{AtomicBool, Ordering};
static TASK_A_DONE: AtomicBool = AtomicBool::new(false);
static TASK_B_DONE: AtomicBool = AtomicBool::new(false);

#[task(priority = 2)]
async fn task_a(cx: task_a::Context) {
    // ... do async work ...
    Mono::delay(1000.millis()).await;
    TASK_A_DONE.store(true, Ordering::Release);
}

#[task(priority = 2)]
async fn task_b(cx: task_b::Context) {
    // ... do async work ...
    Mono::delay(500.millis()).await;
    TASK_B_DONE.store(true, Ordering::Release);
}

#[task(priority = 1)]
async fn orchestrator(cx: orchestrator::Context) {
    // Spawn both
    task_a::spawn().ok();
    task_b::spawn().ok();
    // Wait for both — NOT busy-wait, polls with yield
    loop {
        if TASK_A_DONE.load(Ordering::Acquire) && TASK_B_DONE.load(Ordering::Acquire) {
            break;
        }
        Mono::delay(10.millis()).await;  // yield while waiting
    }
    // Both done — proceed with next step
    do_next_thing();
}
```

### Pattern 3: RTIC Channel (rtic-sync)

`rtic-sync` provides `Channel` for async task communication:

```rust
use rtic_sync::channel::{Sender, Receiver};

#[shared]
struct Shared {}

#[local]
struct Local {
    tx: Sender<'static, SensorReading, 4>,
    rx: Receiver<'static, SensorReading, 4>,
}

#[task(local = [tx], priority = 2)]
async fn producer(cx: producer::Context) {
    loop {
        let reading = read_sensor();
        cx.local.tx.send(reading).await.ok();
        Mono::delay(20.millis()).await;
    }
}

#[task(local = [rx], priority = 1)]
async fn consumer(cx: consumer::Context) {
    loop {
        if let Ok(reading) = cx.local.rx.recv().await {
            process(reading);
        }
    }
}
```

## For User Apps (RAM Demos)

User apps do NOT use RTIC async — they're bare `extern "C" fn _start(api)`
running from Thread mode (idle).  They get concurrency from firmware's
preemptive RTIC tasks running at higher priority:

```
Priority 3: UART ISRs (byte-level)
Priority 2: sensor_poll, motor_poll, USB, heartbeat, rtty_tx (async)
Priority 1: run_demo (calls user app from idle via atomics)
Thread/Idle: User app code (can use SVC for delay_ms)
```

User apps achieve "parallel" behavior because firmware tasks preempt them
continuously — sensor data and motor encoder updates happen at 50 Hz even
while the user app is busy printing or computing.

## Sound Advice for Our Architecture

1. **Keep it simple** — Independent periodic tasks + shared resources is the
   right pattern for embedded.  No need for complex future combinators.

2. **RTIC handles scheduling** — You don't need `select!` or `join!` when
   tasks at different priorities naturally preempt.  A priority-2 sensor task
   will always interrupt a priority-1 user task — that IS your concurrency.

3. **Channels for producer/consumer** — If you need explicit async data flow
   between tasks, use `rtic_sync::channel`.

4. **Atomics for state flags** — Our `task_state` module already does this
   for spawn guards.  Same pattern works for completion signaling.

5. **Never block in async** — Every wait must be `Mono::delay().await`.
   Blocking kills the entire priority level.
