#![feature(extern_prelude)]

extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Build libraries
    cc::Build::new()
        .file("src/sx1276/sx1276.c")
        .include("src")
        .compile(out_path.join("sx1276").to_str().unwrap());


    // Link the library
    println!("cargo:rustc-link-lib=sx1276");

    // Build bindings
    let bindings = bindgen::Builder::default()
        .header("src/wrapper.h")
        .generate()
        .expect("Unable to generate bindings");
    bindings
        .write_to_file(out_path.join("sx1276.rs"))
        .expect("Couldn't write bindings!");

}