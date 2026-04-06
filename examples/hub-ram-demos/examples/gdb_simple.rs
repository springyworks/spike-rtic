//! gdb_simple — Minimal demo for exercising the GDB RSP stub.
//!
//! This is the simplest possible target for GDB remote debugging
//! over the hub's "Demon mode" RSP stub.  It has:
//!
//!   - Static variables at known SRAM2 addresses (printed at start)
//!   - A slow loop incrementing a counter (ideal for watchpoints)
//!   - A small array modified each iteration (for memory reads)
//!   - Exits on center-button press or after ~60 iterations
//!
//! ## Workflow
//!
//! 1. Build + objcopy:
//!    ```sh
//!    cd examples/hub-ram-demos
//!    cargo build --example gdb_simple --release
//!    arm-none-eabi-objcopy -O binary \
//!      target/thumbv7em-none-eabihf/release/examples/gdb_simple \
//!      target/spike-usr_bins/gdb_simple.bin
//!    ```
//!
//! 2. Upload & start:
//!    ```sh
//!    python3 helper-tools/upload_demo.py target/spike-usr_bins/gdb_simple.bin
//!    ```
//!    In the shell: `go` (sandboxed) or `go!` (privileged)
//!
//! 3. While demo runs, enter GDB mode (in another terminal or after
//!    the demo prints its addresses):
//!    ```sh
//!    # in the shell:
//!    gdb
//!    ```
//!    Then connect:
//!    ```sh
//!    arm-none-eabi-gdb -ex "target remote /dev/ttyACM0"
//!    ```
//!
//! 4. In GDB, use the printed addresses:
//!    ```gdb
//!    # Read the counter (replace ADDR with printed address):
//!    x/1xw 0x2004XXXX
//!
//!    # Set a write watchpoint on the counter:
//!    watch *(uint32_t*)0x2004XXXX
//!    continue
//!    ```

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

// ── Static variables in SRAM2 .bss — GDB can read/watch these ──

/// A simple counter that increments each loop iteration.
static mut COUNTER: u32 = 0;

/// A small array modified each iteration — good for `m` (memory read).
static mut DATA: [u8; 16] = [0u8; 16];

/// A flag the user can set from GDB with `M` (memory write) to exit early.
static mut QUIT_FLAG: u32 = 0;

// ── Helpers ──

fn print_u32(api: &MonitorApi, val: u32) {
    let mut buf = [b'0'; 10];
    let mut v = val;
    let mut i = 9usize;
    if v == 0 {
        api.print(b"0");
        return;
    }
    while v > 0 {
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if i == 0 { break; }
        i -= 1;
    }
    api.print(&buf[i..]);
}

fn print_hex(api: &MonitorApi, val: u32) {
    let hex = b"0123456789abcdef";
    let mut buf = [b'0'; 8];
    for i in 0..8 {
        buf[7 - i] = hex[((val >> (i * 4)) & 0xF) as usize];
    }
    api.print(b"0x");
    api.print(&buf);
}

// ════════════════════════════════════════════════════════
//  Entry point
// ════════════════════════════════════════════════════════

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    // Version gate — only need v1 (write_fn + delay_ms)
    if api.version < 1 {
        return 1;
    }

    api.print(b"\r\n=== GDB Simple Demo ===\r\n");
    api.print(b"Slow counter loop for GDB RSP testing.\r\n\r\n");

    // Print addresses so the user can target them in GDB
    let counter_addr = &raw const COUNTER as u32;
    let data_addr = &raw const DATA as u32;
    let quit_addr = &raw const QUIT_FLAG as u32;

    api.print(b"Addresses (use in GDB 'm' / 'watch'):\r\n");
    api.print(b"  COUNTER   @ ");
    print_hex(api, counter_addr);
    api.print(b"\r\n");
    api.print(b"  DATA[16]  @ ");
    print_hex(api, data_addr);
    api.print(b"\r\n");
    api.print(b"  QUIT_FLAG @ ");
    print_hex(api, quit_addr);
    api.print(b"\r\n\r\n");

    api.print(b"Tip: in GDB, set a write-watchpoint on COUNTER:\r\n");
    api.print(b"  watch *(uint32_t*)0x....\r\n");
    api.print(b"  continue\r\n\r\n");

    api.print(b"Press CENTER button to exit (or set QUIT_FLAG=1).\r\n\r\n");

    // Reset statics
    unsafe {
        COUNTER = 0;
        DATA = [0u8; 16];
        QUIT_FLAG = 0;
    }

    // ── Main loop: slow counter ──
    loop {
        let count = unsafe {
            COUNTER = COUNTER.wrapping_add(1);
            COUNTER
        };

        // Fill DATA with a simple pattern based on counter
        unsafe {
            for i in 0..16usize {
                DATA[i] = ((count as u8).wrapping_add(i as u8)).wrapping_mul(7);
            }
        }

        // Print progress every iteration
        api.print(b"  tick ");
        print_u32(api, count);
        api.print(b"\r\n");

        // Check exit conditions
        if unsafe { QUIT_FLAG } != 0 {
            api.print(b"QUIT_FLAG set by GDB - exiting.\r\n");
            break;
        }

        // Check center button (needs v1+, read_buttons is v1)
        let btns = (api.read_buttons)();
        if btns & 0x01 != 0 {
            api.print(b"CENTER pressed - exiting.\r\n");
            break;
        }

        // Timeout after 60 iterations (~60s)
        if count >= 60 {
            api.print(b"Timeout (60 ticks) - exiting.\r\n");
            break;
        }

        // 1-second delay — gives GDB plenty of time
        (api.delay_ms)(1000);
    }

    let final_count = unsafe { COUNTER };
    api.print(b"\r\nFinal COUNTER = ");
    print_u32(api, final_count);
    api.print(b"\r\n=== GDB Simple Demo done ===\r\n");

    0
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
