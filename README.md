# rust-radio-sx127x

A primarily rust driver for the [Semtech SX1276](https://www.semtech.com/products/wireless-rf/lora-transceivers/rohs-compliant/SX1276) sub ghz ISM band radio IC. 

This currently uses [libsx127x](https://github.com/ryankurte/libsx127x) via FFI with the intent of slowly replacing the underlying c components with rust.


## Status

[![GitHub tag](https://img.shields.io/github/tag/ryankurte/rust-radio-sx127x.svg)](https://github.com/ryankurte/rust-radio-sx127x)
[![Travis CI Build Status](https://travis-ci.com/ryankurte/rust-radio-sx127x.svg?branch=master)](https://travis-ci.com/ryankurte/rust-radio-sx127x)
[![BuildKite Build Status](https://badge.buildkite.com/e104ee3bdc9521bc3cd74ab1de43f984bab5da1327549c35e8.svg)](https://buildkite.com/ryankurte/rust-radio-sx127x)
[![Crates.io](https://img.shields.io/crates/v/radio-sx127x.svg)](https://crates.io/crates/radio-sx127x)
[![Docs.rs](https://docs.rs/radio-sx127x/badge.svg)](https://docs.rs/radio-sx127x)

[Open Issues](https://github.com/ryankurte/rust-radio-sx127x/issues)


## Useful Resources
- [Datasheet](https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V6.pdf)
- [libsx127x](https://github.com/ryankurte/libsx127x) semtech c driver port


## Building

The build process will automatically clone `libsx127x` into the output directory, should an alternative directory be required (ie. for working on the c library) this can be set exporting the `LIBSX127X_DIR` environmental variable during the first cargo build.



