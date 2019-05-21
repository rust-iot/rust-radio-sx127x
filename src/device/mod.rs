//! SX127x Radio Driver
//! Copyright 2018 Ryan Kurte

#![allow(unused)]

use core::convert::TryFrom;

pub mod regs;
use regs::{Register, Common, Fsk, LoRa};

pub mod lora;
pub mod fsk;

pub const OPMODE_STATE_MASK: u8 = 0b0000_0111;

/// Sx127x radio state enumeration
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum State {
    Sleep       = 0x00,
    Standby     = 0x01,
    FsTx        = 0x02,
    Tx          = 0x03,
    FsRx        = 0x04,
    Rx          = 0x05,

    /// Lora specific single receive mode
    RxOnce      = 0x06,
    /// Lora specific channel activity detection mode
    Cad         = 0x07,
}

/// OPMODE register LowFrequencyMode bit mask
pub const OPMODE_LF_MASK: u8 = 0b0000_1000;

/// Lora Frequency Mode
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum FrequencyMode {
    /// High frequency mode
    Hf = 0x00,
    /// Low frequency mode
    Lf = 0x08,
}


pub const OPMODE_LONGRANGEMODE_MASK: u8 = 0b1000_0000;

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum LongRangeMode {
    Off = 0x00,
    On = 0x80,
}

#[derive(Copy, Clone, PartialEq, Debug)]
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

/// Device frequency step
pub const FREQ_STEP     : f32 = 61.03515625;

pub const RX_BUFFER_SIZE: u32 = 256;

/// Offset for LF RSSI calculation
pub const RSSI_OFFSET_LF: i16 = -164;

/// Offset for HF RSSI calculation
pub const RSSI_OFFSET_HF: i16 = -157;

// Register initialisation values
pub const REGISTERS_INIT: &[(Modem, Register, u8)] = &[
    ( Modem::Standard , Register::Common(Common::LNA)          , 0x23 ),
    ( Modem::Standard , Register::Fsk(Fsk::RXCONFIG)           , 0x1E ),
    ( Modem::Standard , Register::Fsk(Fsk::RSSICONFIG)         , 0xD2 ),
    ( Modem::Standard , Register::Fsk(Fsk::AFCFEI)             , 0x01 ),
    ( Modem::Standard , Register::Fsk(Fsk::PREAMBLEDETECT)     , 0xAA ),
    ( Modem::Standard , Register::Fsk(Fsk::OSC)                , 0x07 ),
    ( Modem::Standard , Register::Fsk(Fsk::SYNCCONFIG)         , 0x12 ),
    ( Modem::Standard , Register::Fsk(Fsk::SYNCVALUE1)         , 0xC1 ),
    ( Modem::Standard , Register::Fsk(Fsk::SYNCVALUE2)         , 0x94 ),
    ( Modem::Standard , Register::Fsk(Fsk::SYNCVALUE3)         , 0xC1 ),
    ( Modem::Standard , Register::Fsk(Fsk::PACKETCONFIG1)      , 0xD8 ),
    ( Modem::Standard , Register::Fsk(Fsk::FIFOTHRESH)         , 0x8F ),
    ( Modem::Standard , Register::Fsk(Fsk::IMAGECAL)           , 0x02 ),
    ( Modem::Standard , Register::Common(Common::DIOMAPPING1)  , 0x00 ),
    ( Modem::Standard , Register::Common(Common::DIOMAPPING2)  , 0x30 ),
    ( Modem::LoRa     , Register::LoRa(LoRa::PAYLOADMAXLENGTH) , 0x40 ),
];
