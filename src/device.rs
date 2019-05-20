//! SX127x Radio Driver
//! Copyright 2018 Ryan Kurte

#![allow(unused)]

use core::convert::TryFrom;


pub const OPMODE_STATE_MASK: u8 = 0b0000_0111;

/// Sx127x radio state enumeration
pub enum State {
    Sleep       = 0b000,
    Standby     = 0b001,
    FsTx        = 0b010,
    Tx          = 0b011,
    FsRx        = 0b100,
    Rx          = 0b101,
}

pub const OPMODE_SHAPING_MASK: u8 = 0b0001_1000;

pub enum Shaping {
    Shaping00 = 0x00,
    Shaping01 = 0x08,
    Shaping10 = 0x10,
    Shaping11 = 0x18,
}

pub const OPMODE_MODULATION_MASK: u8 = 0b0010_0000;

pub enum Modulation {
    Fsk = 0x00,
    Ook = 0x20,
}

pub const OPMODE_LONGRANGEMODE_MASK: u8 = 0b1000_0000;

pub enum LongRangeMode {
    Off = 0x00,
    On = 0x80,
}

pub enum Modem {
    Standard,
    LoRa,
}

impl TryFrom<u8> for State {
    type Error = ();

    fn try_from(v: u8) -> Result<Self, ()> {
        match v {
            v if v == State::Sleep as u8      => Ok(State::Sleep),
            v if v == State::Standby as u8    => Ok(State::Standby),
            v if v == State::FsTx as u8       => Ok(State::FsTx),
            v if v == State::Tx as u8         => Ok(State::Tx),
            v if v == State::FsRx as u8       => Ok(State::FsRx),
            v if v == State::Rx as u8         => Ok(State::Rx),
            _ => Err(())
        }
    }
}

/// Receive configuration
pub struct RxConfig {
    modem: Modem,
    bandwidth: u32,
    datarate: u32,
    coderate: u8,
    bandwitdth_afc: u32,
    preamble_len: u16,
    symbol_timeout: u16,
    fixed_len: bool,
    payload_len: u8,
    crc_on: bool,
    freq_hop_on: bool,
    hop_period: u8,
    iq_inverted: bool,
    rx_continuous: bool,
}

/// Transmit configuration
pub struct TxConfig {
    modem: Modem,
    power: i8,
    fdev: u32,
    bandwidth: u32,
    datarate: u32,
    coderate: u8,
    preamble_len: u16,
    fixed_len: bool,
    crc_on: bool,
    freq_hop_on: bool,
    hop_period: u8,
    iq_inverted: bool,
    timeout: u32,
}


pub const XTAL_FREQ     : u32 = 32000000;
pub const FREQ_STEP     : f32 = 61.03515625;

pub const RX_BUFFER_SIZE: u32 = 256;

// Offsets for RSSI calculation
pub const RSSI_OFFSET_LF: i16 = -164;
pub const RSSI_OFFSET_HF: i16 = -157;

pub mod fsk {

    // FSK bandwidth register values
    pub enum Bandwidth {
        Bandwidth2600 = 0x17,
        Bandwidth3100 = 0x0F,
        Bandwidth3900 = 0x07,
        Bandwidth5200 = 0x16,
        Bandwidth6300 = 0x0E,
        Bandwidth7800 = 0x06,
        Bandwidth10400 = 0x15,
        Bandwidth12500 = 0x0D,
        Bandwidth15600 = 0x05,
        Bandwidth20800 = 0x14,
        Bandwidth25000 = 0x0C,
        Bandwidth31300 = 0x04,
        Bandwidth41700 = 0x13,
        Bandwidth50000 = 0x0B,
        Bandwidth62500 = 0x03,
        Bandwidth83333 = 0x12,
        Bandwidth100000 = 0x0A,
        Bandwidth125000 = 0x02,
        Bandwidth166700 = 0x11,
        Bandwidth200000 = 0x09,
        Bandwidth250000 = 0x01,
    }

    pub const OPMODE_LRMODE_MASK: u8 = 0x80;
    
    pub const OPMODE_MODTYPE_MASK: u8 = 0x30;
    pub enum ModType {
        Fsk = 0x00,
        Ook = 0x20,
    }

    pub const OPMODE_MODSHAPE_MASK: u8 = 0x80;
    pub enum ModulationShaping {
        Shape00 = 0x00,
        Shape01 = 0x08,
        Shape10 = 0x10,
        Shape11 = 0x11,
    }

    pub const OPMODE_OPMODE_MASK: u8 = 0xF8;

    /// Operating mode
    pub enum OpMode {
        Sleep = 0x00,
        Standby = 0x01,
        SynthesizerTx = 0x02,
        Transmitter = 0x03,
        SynthesizerRx = 0x04,
        Receiver = 0x05,
    }

}

pub mod lora {
    pub struct Config {

    }

    impl Default for Config {
        fn default() -> Self {
            Config{

            }
        }
    }
}