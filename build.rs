//! SX127x Radio Driver
//! Copyright 2018 Ryan Kurte

#![feature(extern_prelude)]

extern crate bindgen;

use std::env;
use std::boxed::Box;
use std::fs::File;
use std::path::PathBuf;
use std::io::prelude::*;
use std::error::Error;

fn main() {
    let out_path = PathBuf::new(); //from(env::var("OUT_DIR").unwrap());
    let src_path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // Build bindings
    let bindings = bindgen::Builder::default()
        .generate_comments(false)
        .use_core()
        .ctypes_prefix("libc")
        .header("src/wrapper.h")
        .generate()
        .expect("Unable to generate bindings");

    // Open a file for writing
    let binding_path = src_path.join("src/sx1276/mod.rs");
    let mut file = match File::create(&binding_path) {
        Err(e) => panic!("Error opening file {}: {}", binding_path.display(), e.description()),
        Ok(f) => f,
    };

    // Patch
    file.write_all(b"#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]\nuse libc;\n\n").unwrap();

    // Write bindings
    bindings
        .write(Box::new(file))
        .expect("Couldn't write bindings!");

    // Build libraries
    cc::Build::new()
        .file("src/sx1276/sx1276.c")
        .include("src")
        .compile(out_path.join("sx1276").to_str().unwrap());

    // Link the library
    println!("cargo:rustc-link-lib=sx1276");
}
