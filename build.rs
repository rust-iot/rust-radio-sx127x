#![feature(extern_prelude)]

extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    let out_path = PathBuf::new(); //from(env::var("OUT_DIR").unwrap());
    let src_path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // Build libraries
    cc::Build::new()
        .file("src/sx1276/sx1276.c")
        .include("src")
        .compile(out_path.join("sx1276").to_str().unwrap());

    // Link the library
    println!("cargo:rustc-link-lib=sx1276");

    // Build bindings
    if false {
        let bindings = bindgen::Builder::default()
            .generate_comments(false)
            .use_core()
            .ctypes_prefix("libc")
            .header("src/wrapper.h")
            .generate()
            .expect("Unable to generate bindings");
        bindings
            .write_to_file(src_path.join("src/sx1276/mod.rs"))
            .expect("Couldn't write bindings!");
    }

}
