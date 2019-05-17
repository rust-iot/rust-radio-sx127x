//! SX128x Radio Driver
//! Copyright 2018 Ryan Kurte


extern crate bindgen;
extern crate git2;
use git2::Repository;

use std::env;
use std::boxed::Box;
use std::fs::File;
use std::path::PathBuf;
use std::error::Error;
use std::collections::HashSet;

#[derive(Debug)]
struct IgnoreMacros(HashSet<String>);

impl bindgen::callbacks::ParseCallbacks for IgnoreMacros {
    fn will_parse_macro(&self, name: &str) -> bindgen::callbacks::MacroParsingBehavior {
        if self.0.contains(name) {
            bindgen::callbacks::MacroParsingBehavior::Ignore
        } else {
            bindgen::callbacks::MacroParsingBehavior::Default
        }
    }
}

const REPO_VAR: &str = "LIBSX127X_DIR";
const REPO_URL: &str = "https://github.com/ryankurte/libsx127x";
const REPO_NAME: &str = "libsx127x-src";

fn main() {
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Select path from 
    let repo_path = match env::var(REPO_VAR) {
        Ok(d) => PathBuf::from(d),
        Err(_) => {
            let mut repo_path = out_path.clone();
            repo_path.push(REPO_NAME);
            repo_path
        }
    };

    println!("Using libsx127x from: {} (source: {})", &repo_path.to_str().unwrap(), REPO_URL);

    let _repo = match repo_path.exists() {
        false => {
            println!("Cloning into: '{:?}'", &repo_path);
            match Repository::clone(REPO_URL, &repo_path) {
                Ok(repo) => repo,
                Err(e) => panic!("failed to clone: {}", e),
            }
        },
        true => {
            println!("Connecting to existing repo: '{:?}'", &repo_path);
            match Repository::open(&repo_path) {
                Ok(repo) => repo,
                Err(e) => panic!("failed to clone: {}", e),
            }
        }
    };

    // Build bindings
    let ignored_macros = IgnoreMacros(
        vec![
            "FP_INFINITE".into(),
            "FP_NAN".into(),
            "FP_NORMAL".into(),
            "FP_SUBNORMAL".into(),
            "FP_ZERO".into(),
            "IPPORT_RESERVED".into(),
        ]
        .into_iter()
        .collect(),
    );

    println!("Generating bindings");
    let bindings = bindgen::Builder::default()
        .generate_comments(false)
        .parse_callbacks(Box::new(ignored_macros))
        .use_core()
        .ctypes_prefix("libc")
        //.clang_arg("-I/usr/include")
        .clang_arg(format!("-I{}/lib", &repo_path.to_str().unwrap()))
        .header("wrapper.h")
        .generate()
        .expect("Unable to generate bindings");

    // Open a file for writing
    let binding_path = out_path.join("sx127x.rs");
    let file = match File::create(&binding_path) {
        Err(e) => panic!("Error opening file {}: {}", binding_path.display(), e.description()),
        Ok(f) => f,
    };

    // Write bindings
    bindings
        .write(Box::new(file))
        .expect("Couldn't write bindings!");

    // Build libraries
    println!("Building library");
    cc::Build::new()
        .file(format!("{}/lib/sx1276.c", &repo_path.to_str().unwrap()))
        .include(format!("{}/lib", &repo_path.to_str().unwrap()))
        .debug(true)
        .flag("-std=c11")
        .flag("-Wno-unused-parameter")
        .flag("-Wno-int-conversion")
        .flag("-Wno-implicit-function-declaration")
        .flag("-Wno-sign-compare")
        .compile("sx1280");

    // Link the library
    println!("cargo:rustc-link-lib=sx1280");
    println!("cargo:rerun-if-changed={}", &repo_path.to_str().unwrap());
}
