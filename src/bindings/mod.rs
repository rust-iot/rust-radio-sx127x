//! Sx127x Binding injection
//! 
//! Copyright 2019 Ryan Kurte

// Bindings may not fit rusts idea of good formatting
#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]

// Libc required to resolve c types
use libc;

// Include generated outputs
include!(concat!(env!("OUT_DIR"), "/sx127x.rs"));
