//! Sx127x FSK mode definitions
//!
//! Copyright 2019 Ryan Kurte

use bitflags::bitflags;

pub use super::common::*;

/// FSK and OOK mode configuration
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct FskConfig {
    /// Preamble length in symbols (defaults to 0x8)
    pub preamble_len: u16,

    /// Payload length configuration (defaults to Variable / Explicit header mode)
    pub payload_len: PayloadLength,

    /// DC-Free encoding/decoding
    pub dc_free: DcFree,

    /// Payload RX CRC configuration (defaults to enabled)
    pub crc: Crc,

    /// Disable auto-clear FIFO and restart RX on CRC failure
    pub crc_autoclear: CrcAutoClear,

    /// Address filtering in RX mode
    pub address_filter: AddressFilter,

    /// Select CRC whitening algorithm
    pub crc_whitening: CrcWhitening,

    /// Set data processing mode
    pub data_mode: DataMode,

    /// Enable io-homecontrol compatibility mode
    pub io_home: IoHome,

    /// Enable beacon mode in fixed packet format
    pub beacon: Beacon,

    /// Receive mode Auto Frequency Compensation (AFC)
    pub rx_afc: RxAfc,

    /// Receive mode Auto Gain Compensation (AGC)
    pub rx_agc: RxAgc,

    /// Receive mode trigger
    pub rx_trigger: RxTrigger,

    /// Node address for filtering
    pub node_address: u8,

    /// Broadcast address for filtering
    pub broadcast_address: u8,

    /// IQ inversion configuration (defaults to disabled)
    pub invert_iq: bool,

    /// RX continuous mode
    pub rx_continuous: bool,

    /// Preamble
    pub preamble: u16,
}

impl Default for FskConfig {
    fn default() -> Self {
        Self {
            preamble_len: 0x8,
            payload_len: PayloadLength::Variable,
            dc_free: DcFree::Whitening,
            crc: Crc::On,
            crc_autoclear: CrcAutoClear::Off,
            address_filter: AddressFilter::Off,
            crc_whitening: CrcWhitening::Ccitt,
            data_mode: DataMode::Packet,
            io_home: IoHome::Off,
            beacon: Beacon::Off,
            rx_afc: RxAfc::On,
            rx_agc: RxAgc::On,
            rx_trigger: RxTrigger::PreambleDetect,
            node_address: 0,
            broadcast_address: 0,
            invert_iq: false,
            rx_continuous: false,
            preamble: 0x0003,
        }
    }
}

/// Fsk radio channel configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct FskChannel {
    /// (G)FSK frequency in Hz (defaults to 434 MHz)
    pub freq: u32,

    /// (G)FSK  channel baud-rate (defaults to 5kbps)
    pub br: u32,

    /// (G)FSK channel bandwidth
    pub bw: Bandwidth,

    /// (G)FSK AFC channel bandwidth
    pub bw_afc: Bandwidth,

    /// Frequency deviation in Hz (defaults to 5kHz)
    pub fdev: u32,
}

impl Default for FskChannel {
    fn default() -> Self {
        Self {
            freq: 434_000_000,
            br: 4_800,
            bw: Bandwidth::Bw12500,
            bw_afc: Bandwidth::Bw12500,
            fdev: 5_000,
        }
    }
}

// FSK bandwidth register values
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum Bandwidth {
    Bw2600 = 0x17,
    Bw3100 = 0x0F,
    Bw3900 = 0x07,
    Bw5200 = 0x16,
    Bw6300 = 0x0E,
    Bw7800 = 0x06,
    Bw10400 = 0x15,
    Bw12500 = 0x0D,
    Bw15600 = 0x05,
    Bw20800 = 0x14,
    Bw25000 = 0x0C,
    Bw31300 = 0x04,
    Bw41700 = 0x13,
    Bw50000 = 0x0B,
    Bw62500 = 0x03,
    Bw83333 = 0x12,
    Bw100000 = 0x0A,
    Bw125000 = 0x02,
    Bw166700 = 0x11,
    Bw200000 = 0x09,
    Bw250000 = 0x01,
}

pub const OPMODE_LRMODE_MASK: u8 = 0x80;

pub const OPMODE_SHAPING_MASK: u8 = 0b0001_1000;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum Shaping {
    Sh00 = 0x00,
    Sh01 = 0x08,
    Sh10 = 0x10,
    Sh11 = 0x18,
}

pub const OPMODE_MODULATION_MASK: u8 = 0b0010_0000;

/// Modulation modes for the standard modem
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum Modulation {
    /// Frequency Shift Keying
    Fsk = 0x00,
    /// On Off Keying
    Ook = 0x20,
}

pub const PACKETFORMAT_MASK: u8 = 0x80;
pub const PACKETFORMAT_VARIABLE: u8 = 0x80;
pub const PACKETFORMAT_FIXED: u8 = 0x80;

pub const DCFREE_MASK: u8 = 0x60;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum DcFree {
    Off = 0x00,
    Manchester = 0x20,
    Whitening = 0x40,
}

pub const CRC_MASK: u8 = 0x10;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum Crc {
    Off = 0x00,
    On = 0x10,
}

pub const CRC_AUTOCLEAR_MASK: u8 = 0x08;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum CrcAutoClear {
    Off = 0x08,
    On = 0x00,
}

pub const ADDRESS_FILTER_MASK: u8 = 0x08;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum AddressFilter {
    Off = 0x00,
    Node = 0x02,
    NodeBroadcast = 0x04,
}

pub const CRC_WHITENING_MASK: u8 = 0x01;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum CrcWhitening {
    Ccitt = 0x00,
    Ibm = 0x01,
}

pub const WMBUS_CRC_MASK: u8 = 0x80;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum WmbusCrc {
    Off = 0x00,
    On = 0x80,
}

pub const DATAMODE_MASK: u8 = 0x40;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum DataMode {
    Continuous = 0x00,
    Packet = 0x40,
}

pub const IOHOME_MASK: u8 = 0x20;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum IoHome {
    Off = 0x00,
    On = 0x20,
}

pub const BEACON_MASK: u8 = 0x08;

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum Beacon {
    Off = 0x00,
    On = 0x08,
}

pub const PAYLOADLEN_MSB_MASK: u8 = 0x07;

pub const TX_START_FIFOLEVEL: u8 = 0x00;
pub const TX_START_FIFOEMPTY: u8 = 0x80;

pub const TX_FIFOTHRESH_MASK: u8 = 0x1f;

pub const RXCONFIG_RESTARTRXONCOLLISION_MASK: u8 = 0x7F;
pub const RXCONFIG_RESTARTRXONCOLLISION_ON: u8 = 0x80;
pub const RXCONFIG_RESTARTRXONCOLLISION_OFF: u8 = 0x00; // Default

pub const RXCONFIG_RESTARTRX_PLL_MASK: u8 = 0b0110_0000;
pub const RXCONFIG_RESTARTRXWITHOUTPLLLOCK: u8 = 0x40; // Write only
pub const RXCONFIG_RESTARTRXWITHPLLLOCK: u8 = 0x20; // Write only

pub const RXCONFIG_AFCAUTO_MASK: u8 = 0xEF;
pub const RXCONFIG_AGCAUTO_MASK: u8 = 0xF7;
pub const RXCONFIG_RXTRIGER_MASK: u8 = 0xF8;

/// Receive mode Auto Frequency Calibration (AFC)
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum RxAfc {
    On = 0x10,
    Off = 0x00,
}

/// Receive mode Auto Gain Compensation (AGC)
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum RxAgc {
    On = 0x08,
    Off = 0x00,
}

/// Receive mode trigger configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum RxTrigger {
    Off = 0x00,
    Rssi = 0x01,
    PreambleDetect = 0x06,
    RssiPreambleDetect = 0x07,
}

/// Control preamble detector state
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum PreambleDetect {
    On = 0x80,
    Off = 0x00,
}

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum PreambleDetectSize {
    /// Interrupt on one byte
    Ps1 = 0b0000_0000,
    /// Interrupt on two bytes
    Ps2 = 0b0010_0000,
    /// Interrupt on three bytes
    Ps3 = 0b0100_0000,
}

pub const PREAMBLE_DETECTOR_TOL: u8 = 0x0A;

bitflags! {
    /// Interrupt flags register 1
    pub struct Irq1: u8 {
        /// Set when a given mode request is complete
        const MODE_READY        = 0b1000_0000;
        /// Set in RSSI mode after RSSI, AGC and AFC
        const RX_READY          = 0b0100_0000;
        /// Set in TX mode after PA ramp-up
        const TX_READY          = 0b0010_0000;
        /// Set in (FS, Rx, Tx) when PLL is locked
        const PLL_LOCKED        = 0b0001_0000;
        /// Set in Rx when RSSI exceeds the programmed RSSI threshold
        const RSSI              = 0b0000_1000;
        /// Set when a timeout occurs
        const TIMEOUT           = 0b0000_0100;
        /// Set when a preamble is detected (must be manually cleared)
        const PREAMBLED_DETECT  = 0b0000_0010;
        /// Set when Sync and Address (if enabled) are detected
        const SYNC_ADDR_MATCH   = 0b0000_0001;
    }
}

bitflags! {
    /// Interrupt flags register two
    pub struct Irq2: u8 {
        /// Set when the FIFO is full
        const FIFO_FULL        = 0b1000_0000;
        /// Set when the FIFO is empty
        const FIFO_EMPTY          = 0b0100_0000;
        /// Set when the number of bytes in the FIFO exceeds the fifo threshold
        const FIFO_LEVEL          = 0b0010_0000;
        /// Set when a FIFO overrun occurs
        const FIFO_OVERRUN        = 0b0001_0000;
        /// Set in Tx when a packet has been sent
        const PACKET_SENT              = 0b0000_1000;
        /// Set in Rx when a packet has been received and CRC has passed
        const PAYLOAD_READY           = 0b0000_0100;
        /// Set in Rx when a packet CRC is okay
        const CRC_OK  = 0b0000_0010;
        /// Set when the battery voltage drops below the low battery threshold
        const LOW_BAT   = 0b0000_0001;
    }
}
