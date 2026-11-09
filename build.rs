use std::{env, fs, path::PathBuf};

fn main() {
    if env::var("TARGET").as_deref() != Ok("thumbv7m-none-eabi") {
        return;
    }
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by Cargo"));
    fs::copy("memory.x", out.join("memory.x")).expect("copy bootloader memory layout");
    // The polling firmware's vector table points directly at DefaultHandler;
    // there are no named peripheral handlers requiring linker aliases.
    fs::write(out.join("device.x"), "/* IRQ table supplied by main.rs. */\n")
        .expect("write runtime device linker include");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
