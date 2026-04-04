//! Cargo build script for hub-ram-demos.
//!
//! Copies the custom linker script (`hub-ram-demo.x`) into the build output
//! directory so `cargo` can find it via `-Thub-ram-demo.x`.  Also writes an
//! empty `link.x` stub to satisfy the `thumbv7em-none-eabihf` target spec's
//! default `-Tlink.x` expectation — the real layout is in `hub-ram-demo.x`.
//!
//! The linker script places demos at `0x2004_0000` (SRAM2 base) so the
//! firmware can upload and execute them without reflashing.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy("hub-ram-demo.x", out.join("hub-ram-demo.x")).unwrap();
    // Provide an empty link.x to satisfy the thumbv7em target spec's
    // default `-Tlink.x`. Our actual layout is in hub-ram-demo.x.
    fs::write(out.join("link.x"), "/* empty — hub-ram-demo.x is our linker script */\n").unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=hub-ram-demo.x");
}
