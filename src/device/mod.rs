//! SX127x general device definitions
//!
// Copyright 2018 Ryan Kurte

#![allow(unused)]

use core::convert::TryFrom;

pub mod regs;
use radio::RadioState;
use regs::{Common, Fsk, LoRa, Register};

pub mod common;

pub mod lora;
use self::lora::{LoRaChannel, LoRaConfig};
pub mod fsk;
use self::fsk::{FskChannel, FskConfig};

pub const OPMODE_STATE_MASK: u8 = 0b0000_0111;

/// Sx127x radio configuration
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub struct Config {
    /// Device modem configuration
    pub modem: Modem,

    /// Device channel configuration
    pub channel: Channel,

    /// Device power amplifier configuration
    pub pa_config: PaConfig,

    /// Device crystal frequency (defaults to 32MHz)
    pub xtal_freq: u32,

    /// Timeout for internally blocking radio operations
    pub timeout_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            modem: Modem::LoRa(LoRaConfig::default()),
            channel: Channel::LoRa(LoRaChannel::default()),
            pa_config: PaConfig::default(),
            xtal_freq: 32000000,
            timeout_ms: 100,
        }
    }
}

/// LoRa Received packet information
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub struct PacketInfo {
    /// Received Signal Strength Indication
    pub rssi: i16,
    /// Signal to Noise Ratio
    pub snr: Option<i16>,
}

impl Default for PacketInfo {
    fn default() -> Self {
        Self { rssi: 0, snr: None }
    }
}


impl radio::ReceiveInfo for PacketInfo {
    fn rssi(&self) -> i16 {
        self.rssi
    }
}

/// Radio modem configuration contains fields for each modem mode
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum Modem {
    /// Modem not configured
    None,
    /// Modem configured in LoRa mode
    LoRa(LoRaConfig),
    /// Modem configured in Standard (FSK/OOK) mode
    FskOok(FskConfig),
}

/// Radio channel configuration contains channel options for each mode
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum Channel {
    /// Channel not configured
    None,
    /// Channel configured in LoRa mode
    LoRa(LoRaChannel),
    /// Channel configured in Standard (FSK/OOK) mode
    FskOok(FskChannel),
}

impl Default for Channel {
    fn default() -> Self {
        Channel::LoRa(LoRaChannel::default())
    }
}

/// Sx127x radio state enumeration
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum State {
    Sleep = 0x00,
    Standby = 0x01,
    FsTx = 0x02,
    Tx = 0x03,
    FsRx = 0x04,
    Rx = 0x05,

    /// Lora specific single receive mode
    RxOnce = 0x06,
    /// Lora specific channel activity detection mode
    Cad = 0x07,
}

impl RadioState for State {
    fn idle() -> Self {
        State::Standby
    }

    fn sleep() -> Self {
        State::Sleep
    }
}

/// Sx127x Power Amplifier (PA) configuration
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub struct PaConfig {
    /// Power amplifier output selection (defaults to PA_BOOST output)
    pub output: PaSelect,
    /// Output power in dBm (defaults to 10dBm)
    pub power: i8,
}

impl Default for PaConfig {
    fn default() -> PaConfig {
        Self {
            output: PaSelect::Boost,
            power: 10,
        }
    }
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

pub const OPMODE_MODULATION_MASK: u8 = 0b0110_0000;

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum ModulationType {
    Fsk = 0b0000_0000,
    Ook = 0b0010_0000,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum ModemMode {
    Standard,
    LoRa,
}

/// Radio modem configuration contains fields for each modem mode
#[derive(Clone, PartialEq, Debug)]
pub enum Interrupts {
    /// LoRa mode interrupts
    LoRa(self::lora::Irq),
    /// FskOok mode interrupts
    FskOok((self::fsk::Irq1, self::fsk::Irq2)),
}

impl TryFrom<u8> for State {
    type Error = ();

    fn try_from(v: u8) -> Result<Self, ()> {
        match v {
            v if v == State::Sleep as u8 => Ok(State::Sleep),
            v if v == State::Standby as u8 => Ok(State::Standby),
            v if v == State::FsTx as u8 => Ok(State::FsTx),
            v if v == State::Tx as u8 => Ok(State::Tx),
            v if v == State::FsRx as u8 => Ok(State::FsRx),
            v if v == State::Rx as u8 => Ok(State::Rx),
            _ => Err(()),
        }
    }
}

/// Receive configuration
pub struct RxConfig {
    modem: ModemMode,
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
    modem: ModemMode,
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
pub const FREQ_STEP: f32 = 61.03515625;

pub const RX_BUFFER_SIZE: u32 = 256;

/// Offset for LF RSSI calculation
pub const RSSI_OFFSET_LF: i16 = -164;

/// Offset for HF RSSI calculation
pub const RSSI_OFFSET_HF: i16 = -157;

// Register initialisation values
pub const REGISTERS_INIT: &[(ModemMode, Register, u8)] = &[
    (ModemMode::Standard, Register::Common(Common::LNA), 0x23),
    (ModemMode::Standard, Register::Fsk(Fsk::RXCONFIG), 0x1E),
    (ModemMode::Standard, Register::Fsk(Fsk::RSSICONFIG), 0xD2),
    (ModemMode::Standard, Register::Fsk(Fsk::AFCFEI), 0x01),
    (
        ModemMode::Standard,
        Register::Fsk(Fsk::PREAMBLEDETECT),
        0xAA,
    ),
    (ModemMode::Standard, Register::Fsk(Fsk::OSC), 0x07),
    (ModemMode::Standard, Register::Fsk(Fsk::SYNCCONFIG), 0x12),
    (ModemMode::Standard, Register::Fsk(Fsk::SYNCVALUE1), 0xC1),
    (ModemMode::Standard, Register::Fsk(Fsk::SYNCVALUE2), 0x94),
    (ModemMode::Standard, Register::Fsk(Fsk::SYNCVALUE3), 0xC1),
    (ModemMode::Standard, Register::Fsk(Fsk::PACKETCONFIG1), 0xD8),
    (ModemMode::Standard, Register::Fsk(Fsk::FIFOTHRESH), 0x8F),
    (ModemMode::Standard, Register::Fsk(Fsk::IMAGECAL), 0x02),
    (
        ModemMode::Standard,
        Register::Common(Common::DIOMAPPING1),
        0x00,
    ),
    (
        ModemMode::Standard,
        Register::Common(Common::DIOMAPPING2),
        0x30,
    ),
    (
        ModemMode::LoRa,
        Register::LoRa(LoRa::PAYLOADMAXLENGTH),
        0x40,
    ),
];

pub const PASELECT_MASK: u8 = 0b1000_0000;
pub const PASELECT_RFO: u8 = 0b0000_0000;
pub const PASELECT_PA_BOOST: u8 = 0b1000_0000;

/// Select the power amplifier output configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum PaSelect {
    /// RFO pin, output power limited to +14dBm
    /// with specified maximum output value, defaults to 0x04 for 14dBm output
    /// Pout = PMax-(15-power) where PMax = 10.8+0.6*maximum
    Rfo(u8),
    /// PA_BOOST pin and output power limited to +20dBm
    /// Pout = 17-(15-power)
    Boost,
}

pub const MAXPOWER_MASK: u8 = 0b0111_0000;
pub const MAXPOWER_SHIFT: u8 = 4;

pub const OUTPUTPOWER_MASK: u8 = 0b0000_1111;

pub const PADAC_MASK: u8 = 0b0000_0111;
pub const PADAC_20DBM_ON: u8 = 0x07;
pub const PADAC_20DBM_OFF: u8 = 0x04;
