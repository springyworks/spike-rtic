#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

/// Minimal sandbox test: print, delay, return.
/// No motors, sensors, RTTY, or complex calls.
#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    api.print(b"simple_test: hello from sandbox\r\n");

    (api.delay_ms)(500);

    api.print(b"simple_test: waited 500ms\r\n");

    (api.delay_ms)(500);

    api.print(b"simple_test: done, returning 42\r\n");

    42
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
