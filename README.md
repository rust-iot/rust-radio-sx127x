# rust-radio-sx127x

A primarily rust driver (and command line utility) for the [Semtech SX1276](https://www.semtech.com/products/wireless-rf/lora-transceivers/rohs-compliant/SX1276) sub ghz ISM band radio IC. 


## Status

WIP. Basic LoRa functionality working.

[![GitHub tag](https://img.shields.io/github/tag/ryankurte/rust-radio-sx127x.svg)](https://github.com/ryankurte/rust-radio-sx127x)
[![Build Status](https://travis-ci.com/ryankurte/rust-radio-sx127x.svg?branch=master)](https://travis-ci.com/ryankurte/rust-radio-sx127x)
[![BuildKite Build Status](https://badge.buildkite.com/e104ee3bdc9521bc3cd74ab1de43f984bab5da1327549c35e8.svg)](https://buildkite.com/ryankurte/rust-radio-sx127x)
[![Crates.io](https://img.shields.io/crates/v/radio-sx127x.svg)](https://crates.io/crates/radio-sx127x)
[![Docs.rs](https://docs.rs/radio-sx127x/badge.svg)](https://docs.rs/radio-sx127x)
[![Snap Status](https://build.snapcraft.io/badge/ryankurte/rust-radio-sx127x.svg)](https://build.snapcraft.io/user/ryankurte/rust-radio-sx127x)

[Open Issues](https://github.com/ryankurte/rust-radio-sx127x/issues)

## Usage

Add to your project with `cargo add radio-sx127x`

Install the utility with one of the following methods:

- using a pre-packaged snap with `snap install sx127x-util`
- using a precompiled binary from the [releases](https://github.com/ryankurte/rust-radio-sx127x/releases/) page
- from source using cargo with `cargo install radio-sx127x`

## As a `no_std` Library

The radio-sx127x crate can be used as an interface library for the sx127x radio on other
embedded devices.  To enable `no_std` usage, add `default-features = false` to your
`Cargo.toml`


## Useful Resources
- [Datasheet](https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V6.pdf)
- [libsx127x](https://github.com/ryankurte/libsx127x) semtech c driver port




