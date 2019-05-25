//! Sx127x LoRa mode definitions
//! 
//! Copyright 2019 Ryan Kurte

pub const BANDWIDTH_MASK: u8 = 0b1111_0000;

/// LoRa channel bandwidth in kHz
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Bandwidth {
    /// 62.5kHz bandwidth
    //Bandwidth62_5kHz = 0b0110_0000,
    /// 125kHz bandwidth
    Bandwidth125kHz  = 0b0111_0000,
    /// 250kHz bandwidth
    Bandwidth250kHz  = 0b1000_0000,
    /// 500kHz bandwidth
    Bandwidth500kHz  = 0b1001_0000,
}

pub const SPREADING_FACTOR_MASK: u8 = 0b1111_0000;

/// LoRa spreading factor in chips / symbol
#[derive(Copy, Clone, PartialEq, Debug)]
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
pub enum Coderate {
    /// 4/5
    CodingRate1 = 0b0000_0010,
    /// 4/6
    CodingRate2 = 0b0000_0100,
    /// 4/7
    CodingRate3 = 0b0000_0110,
    /// 4/8
    CodingRate4 = 0b0000_1000,
}

pub const IMPLICITHEADER_MASK:    u8 = 0b0000_0001;
pub const IMPLICITHEADER_ENABLE:  u8 = 0b0000_0001;
pub const IMPLICITHEADER_DISABLE: u8 = 0b0000_0000;

/// Payload length configuration
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum PayloadLength {
    /// Constant length payloads use implicit headers
    Constant(u16),
    /// Variable length payloads must be contain explicit headers
    Variable
}

pub const RXPAYLOADCRC_MASK: u8 = 0b0000_0100;

/// Payload RX CRC configuration
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum PayloadCrc {
    Disabled = 0x00,
    Enabled  = 0x04,
}

pub const SYMBTIMEOUTMSB_MASK: u8 = 0b0000_0011;


pub const ACG_AUTO_ON_MASK: u8 = 0b0000_0100;
pub const ACG_AUTO_ON_ENABLED: u8 = 0b0000_0100;
pub const ACG_AUTO_ON_DISABLED: u8 = 0b0000_0000;

pub const LOWDATARATEOPTIMIZE_MASK: u8 = 0b0000_1000;

/// Low datarate optimization state
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum LowDatarateOptimise {
    /// Low datarate optimizations disabled
    Disabled = 0x00,
    /// Low datarate optimizations enabled, this is required when symbol length > 16ms
    Enabled = 0x08,
}

pub const PLLHOP_FASTHOP_MASK: u8 = 0b1000_0000;
pub const PLLHOP_FASTHOP_ON:   u8 = 0b1000_0000;
pub const PLLHOP_FASTHOP_OFF:   u8 = 0b0000_0000;

/// Frequency hopping configuration
#[derive(Copy, Clone, PartialEq, Debug)]
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
pub enum DetectionOptimize {
    /// Optimised for Sf7 to Sf12
    Sf7To12 = 0x03,
    /// Optimised for Sf6
    Sf6 = 0x05,
}


pub const INVERTIQ_RX_MASK: u8 = 0xBF;
pub const INVERTIQ_RX_OFF:  u8 = 0x00;
pub const INVERTIQ_RX_ON:   u8 = 0x40;
pub const INVERTIQ_TX_MASK: u8 = 0xFE;
pub const INVERTIQ_TX_OFF:  u8 = 0x01;
pub const INVERTIQ_TX_ON:   u8 = 0x00;

pub const INVERTIQ2_ON:  u8 = 0x19;
pub const INVERTIQ2_OFF: u8 = 0x1D;

pub const PASELECT_MASK:     u8 = 0b1000_0000;
pub const PASELECT_RFO:      u8 = 0b0000_0000;
pub const PASELECT_PA_BOOST: u8 = 0b1000_0000;

/// Select the power amplifier output configuration
#[derive(Copy, Clone, PartialEq, Debug)]
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