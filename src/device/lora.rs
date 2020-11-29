//! Sx127x LoRa mode definitions
//!
//! Copyright 2019 Ryan Kurte

use bitflags::bitflags;

pub use super::common::*;

/// LoRa Radio Configuration Object
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
#[cfg_attr(feature = "serde", serde(default))]
pub struct LoRaConfig {
    /// LoRa Frequency hopping configuration (defaults to disabled)
    pub frequency_hop: FrequencyHopping,
    /// Preamble length in symbols (defaults to 0x8)
    /// (note that hardware adds four additional symbols in LoRa mode)
    pub preamble_len: u16,
    /// Payload length configuration (defaults to Variable / Explicit header mode)
    pub payload_len: PayloadLength,
    /// Payload RX CRC configuration (defaults to enabled)
    pub payload_crc: PayloadCrc,
    /// IQ inversion configuration (defaults to disabled)
    pub invert_iq: bool,
    /// TxSingle timeout value (defaults to 0x64)
    pub symbol_timeout: u16,
}

impl Default for LoRaConfig {
    fn default() -> Self {
        LoRaConfig {
            preamble_len: 0x8,
            symbol_timeout: 0x64,
            payload_len: PayloadLength::Variable,
            payload_crc: PayloadCrc::Enabled,
            frequency_hop: FrequencyHopping::Disabled,
            invert_iq: false,
        }
    }
}

/// LoRa radio channel configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
#[cfg_attr(feature = "serde", serde(default))]
pub struct LoRaChannel {
    /// LoRa frequency in Hz (defaults to 434 MHz)
    pub freq: u32,
    /// LoRa channel bandwidth (defaults to 125kHz)
    pub bw: Bandwidth,
    /// LoRa spreading factor (defaults to SF7)
    pub sf: SpreadingFactor,
    /// LoRa coding rate (defaults to 4/5)
    pub cr: CodingRate,
}

impl Default for LoRaChannel {
    fn default() -> Self {
        Self {
            freq: 434e6 as u32,
            bw: Bandwidth::Bw125kHz,
            sf: SpreadingFactor::Sf7,
            cr: CodingRate::Cr4_5,
        }
    }
}

pub const BANDWIDTH_MASK: u8 = 0b1111_0000;

/// LoRa channel bandwidth in kHz
#[allow(non_snake_case)]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum Bandwidth {
    /// 62.5kHz bandwidth
    //Bandwidth62_5kHz = 0b0110_0000,
    /// 125kHz bandwidth
    Bw125kHz = 0b0111_0000,
    /// 250kHz bandwidth
    Bw250kHz = 0b1000_0000,
    /// 500kHz bandwidth
    Bw500kHz = 0b1001_0000,
}

pub const SPREADING_FACTOR_MASK: u8 = 0b1111_0000;

/// LoRa spreading factor in chips / symbol
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum SpreadingFactor {
    /// Sf6: 64 chips / symbol
    Sf6 = 0b0110_0000,
    /// Sf7: 128 chips / symbol
    Sf7 = 0b0111_0000,
    /// Sf8: 256 chips / symbol
    Sf8 = 0b1000_0000,
    /// Sf9: 512 chips / symbol
    Sf9 = 0b1001_0000,
    /// Sf10: 1024 chips / symbol
    Sf10 = 0b1010_0000,
    /// Sf11: 2048 chips / symbol
    Sf11 = 0b1011_0000,
    /// Sf12: 4096 chips / symbol
    Sf12 = 0b1100_0000,
}

pub const CODERATE_MASK: u8 = 0b0000_1110;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum CodingRate {
    /// LoRa Coding rate 4/5
    Cr4_5 = 0b0000_0010,
    /// LoRa Coding rate 4/6
    Cr4_6 = 0b0000_0100,
    /// LoRa Coding rate 4/7
    Cr4_7 = 0b0000_0110,
    /// LoRa Coding rate 4/8
    Cr4_8 = 0b0000_1000,
}

pub const IMPLICITHEADER_MASK: u8 = 0b0000_0001;
pub const IMPLICITHEADER_ENABLE: u8 = 0b0000_0001;
pub const IMPLICITHEADER_DISABLE: u8 = 0b0000_0000;

pub const RXPAYLOADCRC_MASK: u8 = 0b0000_0100;

/// Payload RX CRC configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum PayloadCrc {
    Disabled = 0x00,
    Enabled = 0x04,
}

pub const SYMBTIMEOUTMSB_MASK: u8 = 0b0000_0011;

pub const ACG_AUTO_ON_MASK: u8 = 0b0000_0100;
pub const ACG_AUTO_ON_ENABLED: u8 = 0b0000_0100;
pub const ACG_AUTO_ON_DISABLED: u8 = 0b0000_0000;

pub const LOWDATARATEOPTIMIZE_MASK: u8 = 0b0000_1000;

/// Low datarate optimization state
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum LowDatarateOptimise {
    /// Low datarate optimizations disabled
    Disabled = 0x00,
    /// Low datarate optimizations enabled, this is required when symbol length > 16ms
    Enabled = 0x08,
}

pub const PLLHOP_FASTHOP_MASK: u8 = 0b1000_0000;
pub const PLLHOP_FASTHOP_ON: u8 = 0b1000_0000;
pub const PLLHOP_FASTHOP_OFF: u8 = 0b0000_0000;

/// Frequency hopping configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum FrequencyHopping {
    Disabled,
    /// Enabled specifies the number of symbol periods between frequency hops
    Enabled(u16),
}

pub const RF_MID_BAND_THRESH: u32 = 525000000;

pub const DETECTIONOPTIMIZE_MASK: u8 = 0b0000_0011;

pub const AUTOMATICIF_MASK: u8 = 0b1000_0000;
pub const AUTOMATICIF_ON: u8 = 0b1000_0000;
pub const AUTOMATICIF_OFF: u8 = 0b0000_0000;

/// LoRa detection optimization mode
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))] 
pub enum DetectionOptimize {
    /// Optimised for Sf7 to Sf12
    Sf7To12 = 0x03,
    /// Optimised for Sf6
    Sf6 = 0x05,
}

pub const INVERTIQ_RX_MASK: u8 = 0xBF;
pub const INVERTIQ_RX_OFF: u8 = 0x00;
pub const INVERTIQ_RX_ON: u8 = 0x40;
pub const INVERTIQ_TX_MASK: u8 = 0xFE;
pub const INVERTIQ_TX_OFF: u8 = 0x01;
pub const INVERTIQ_TX_ON: u8 = 0x00;

pub const INVERTIQ2_ON: u8 = 0x19;
pub const INVERTIQ2_OFF: u8 = 0x1D;

bitflags! {
    /// Interrupt flags register 1
    pub struct Irq: u8 {
        /// Timeout interrupt, manually cleared
        const RX_TIMEOUT        = 0b1000_0000;
        /// Packet receipt complete
        const RX_DONE           = 0b0100_0000;
        /// Indicates an invalid CRC was received
        const CRC_ERROR         = 0b0010_0000;
        /// Indicates a valid header has been received
        const VALID_HEADER      = 0b0001_0000;
        /// Packet sending complete
        const TX_DONE           = 0b0000_1000;
        /// Set when a timeout occurs
        const CAD_DONE          = 0b0000_0100;
        /// Set when a preamble is detected (must be manually cleared)
        const PREAMBLED_DETECT  = 0b0000_0010;
        /// Set when Sync and Address (if enabled) are detected
        const SYNC_ADDR_MATCH   = 0b0000_0001;
    }
}

bitflags! {
    ///Modem Status flags
    pub struct ModemStatus: u8 {

        const MODEM_CLEAR         = 0b0001_0000;

        const HEADER_VALID        = 0b0000_1000;

        const RX_ONGOING          = 0b0000_0100;

        const SIGNAL_SYNCHRONIZED = 0b0000_0010;

        const SIGNAL_DETECTED     = 0b0000_0001;
    }
}
