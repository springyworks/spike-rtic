#![no_std]
#![no_main]

use spike_hub_api::MonitorApi;

const RTTY_MSG: &[u8] = b"RTTY TEST\r\n";

#[no_mangle]
#[link_section = ".text._start"]
pub extern "C" fn _start(api: *const MonitorApi) -> u32 {
    let api = unsafe { &*api };

    // Check API version
    if api.version < 7 {
        api.print(b"ERR: need API v7+\r\n");
        return 1;
    }

    api.print(b"hub-usr-app: RTTY every 4s test demo\r\n");

    loop {
        // Wait until RTTY is not busy
        while (api.rtty_busy)() != 0 {}
        // Send RTTY message
        (api.rtty_say)(RTTY_MSG.as_ptr(), RTTY_MSG.len() as u32);
        // Print to shell as well
        api.print(b"RTTY sent\r\n");
        // Wait 4 seconds (4000 ms)
        (api.delay_ms)(4000);
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
