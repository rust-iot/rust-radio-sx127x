//! Sx127x FSK mode definitions
//! 
//! Copyright 2019 Ryan Kurte

/// FSK and OOK mode configuration
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct FskConfig {
    /// FSK/OOK channel configuration
    pub channel: FskChannel,
}

impl Default for FskConfig {
    fn default() -> Self {
        Self {
            channel: FskChannel::default(),

        }
    }
}

/// Fsk radio channel configuration
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct FskChannel {
    /// Channel bit-rate
    bitrate: u16,

    /// Channel bandwidth
    bandwidth: Bandwidth,
}

impl Default for FskChannel {
    fn default() -> Self {
        Self {
            bitrate: 0,
            bandwidth: Bandwidth::Bw12500,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct FskInfo {

}

// FSK bandwidth register values
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
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

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum Shaping {
    Sh00 = 0x00,
    Sh01 = 0x08,
    Sh10 = 0x10,
    Sh11 = 0x18,
}

pub const OPMODE_MODULATION_MASK: u8 = 0b0010_0000;

/// Modulation modes for the standard modem
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum Modulation {
    /// Frequency Shift Keying
    Fsk = 0x00,
    /// On Off Keying
    Ook = 0x20,
}

bitflags! {
    /// Interrupt flags register 1
    struct Irq1: u8 {
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
    struct Irq2: u8 {
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