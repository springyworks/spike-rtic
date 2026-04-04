//! tiny_test — Minimal sandbox diagnostic.
//!
//! Does almost nothing: prints one line and returns 42.
//! If this faults in sandbox mode, the sandbox mechanism itself is broken.

#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };
    api.print(b"tiny_test: alive!\r\n");
    (api.delay_ms)(200);
    api.print(b"tiny_test: done.\r\n");
    42
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
