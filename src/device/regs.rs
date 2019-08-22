//! SX127x Register Definitions
//!
//! Copyright 2019 Ryan Kurte

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Register {
    Common(Common),
    Fsk(Fsk),
    LoRa(LoRa),
}

impl Into<u8> for Register {
    fn into(self) -> u8 {
        match self {
            Register::Common(c) => c as u8,
            Register::Fsk(f) => f as u8,
            Register::LoRa(l) => l as u8,
        }
    }
}

impl Into<u8> for Common {
    fn into(self) -> u8 {
        self as u8
    }
}

impl Into<u8> for LoRa {
    fn into(self) -> u8 {
        self as u8
    }
}

impl Into<u8> for Fsk {
    fn into(self) -> u8 {
        self as u8
    }
}

impl From<Common> for Register {
    fn from(common: Common) -> Register {
        Register::Common(common)
    }
}

impl From<LoRa> for Register {
    fn from(lora: LoRa) -> Register {
        Register::LoRa(lora)
    }
}

impl From<Fsk> for Register {
    fn from(fsk: Fsk) -> Register {
        Register::Fsk(fsk)
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Common {
    FIFO = 0x00,
    // Common settings
    OPMODE = 0x01,
    FRFMSB = 0x06,
    FRFMID = 0x07,
    FRFLSB = 0x08,
    // Tx settings
    PACONFIG = 0x09,
    PARAMP = 0x0A,
    OCP = 0x0B,
    // Rx settings
    LNA = 0x0C,
    // I/O settings
    DIOMAPPING1 = 0x40,
    DIOMAPPING2 = 0x41,
    // Version
    VERSION = 0x42,
    // Additional settings
    PLLHOP = 0x44,
    TCXO = 0x4B,
    PADAC = 0x4D,
    FORMERTEMP = 0x5B,
    BITRATEFRAC = 0x5D,
    AGCREF = 0x61,
    AGCTHRESH1 = 0x62,
    AGCTHRESH2 = 0x63,
    AGCTHRESH3 = 0x64,
    PLL = 0x70,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum LoRa {
    // LoRa registers
    FIFOADDRPTR = 0x0D,
    FIFOTXBASEADDR = 0x0E,
    FIFORXBASEADDR = 0x0F,
    FIFORXCURRENTADDR = 0x10,
    IRQFLAGSMASK = 0x11,
    IRQFLAGS = 0x12,
    RXNBBYTES = 0x13,
    RXHEADERCNTVALUEMSB = 0x14,
    RXHEADERCNTVALUELSB = 0x15,
    RXPACKETCNTVALUEMSB = 0x16,
    RXPACKETCNTVALUELSB = 0x17,
    MODEMSTAT = 0x18,
    PKTSNRVALUE = 0x19,
    PKTRSSIVALUE = 0x1A,
    RSSIVALUE = 0x1B,
    HOPCHANNEL = 0x1C,
    MODEMCONFIG1 = 0x1D,
    MODEMCONFIG2 = 0x1E,
    SYMBTIMEOUTLSB = 0x1F,
    PREAMBLEMSB = 0x20,
    PREAMBLELSB = 0x21,
    PAYLOADLENGTH = 0x22,
    PAYLOADMAXLENGTH = 0x23,
    HOPPERIOD = 0x24,
    FIFORXBYTEADDR = 0x25,
    MODEMCONFIG3 = 0x26,
    FEIMSB = 0x28,
    FEIMID = 0x29,
    FEILSB = 0x2A,
    RSSIWIDEBAND = 0x2C,
    TEST2F = 0x2F,
    TEST30 = 0x30,
    DETECTOPTIMIZE = 0x31,
    INVERTIQ = 0x33,
    TEST36 = 0x36,
    DETECTIONTHRESHOLD = 0x37,
    SYNCWORD = 0x39,
    TEST3A = 0x3A,
    INVERTIQ2 = 0x3B,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Fsk {
    BITRATEMSB = 0x02,
    BITRATELSB = 0x03,
    FDEVMSB = 0x04,
    FDEVLSB = 0x05,
    RXCONFIG = 0x0D,
    RSSICONFIG = 0x0E,
    RSSICOLLISION = 0x0F,
    RSSITHRESH = 0x10,
    RSSIVALUE = 0x11,
    RXBW = 0x12,
    AFCBW = 0x13,
    OOKPEAK = 0x14,
    OOKFIX = 0x15,
    OOKAVG = 0x16,
    RES17 = 0x17,
    RES18 = 0x18,
    RES19 = 0x19,
    AFCFEI = 0x1A,
    AFCMSB = 0x1B,
    AFCLSB = 0x1C,
    FEIMSB = 0x1D,
    FEILSB = 0x1E,
    PREAMBLEDETECT = 0x1F,
    RXTIMEOUT1 = 0x20,
    RXTIMEOUT2 = 0x21,
    RXTIMEOUT3 = 0x22,
    RXDELAY = 0x23,
    // Oscillator settings
    OSC = 0x24,
    // Packet handler settings
    PREAMBLEMSB = 0x25,
    PREAMBLELSB = 0x26,
    SYNCCONFIG = 0x27,
    SYNCVALUE1 = 0x28,
    SYNCVALUE2 = 0x29,
    SYNCVALUE3 = 0x2A,
    SYNCVALUE4 = 0x2B,
    SYNCVALUE5 = 0x2C,
    SYNCVALUE6 = 0x2D,
    SYNCVALUE7 = 0x2E,
    SYNCVALUE8 = 0x2F,
    PACKETCONFIG1 = 0x30,
    PACKETCONFIG2 = 0x31,
    PAYLOADLENGTH = 0x32,
    NODEADRS = 0x33,
    BROADCASTADRS = 0x34,
    FIFOTHRESH = 0x35,
    // SM settings
    SEQCONFIG1 = 0x36,
    SEQCONFIG2 = 0x37,
    TIMERRESOL = 0x38,
    TIMER1COEF = 0x39,
    TIMER2COEF = 0x3A,
    // Service settings
    IMAGECAL = 0x3B,
    TEMP = 0x3C,
    LOWBAT = 0x3D,
    // Status
    IRQFLAGS1 = 0x3E,
    IRQFLAGS2 = 0x3F,
}

pub const RF_IMAGECAL_AUTOIMAGECAL_MASK: u8 = 0x7F;
pub const RF_IMAGECAL_AUTOIMAGECAL_ON: u8 = 0x80;
pub const RF_IMAGECAL_AUTOIMAGECAL_OFF: u8 = 0x00;

pub const RF_IMAGECAL_IMAGECAL_MASK: u8 = 0xBF;
pub const RF_IMAGECAL_IMAGECAL_START: u8 = 0x40;

pub const RF_IMAGECAL_IMAGECAL_RUNNING: u8 = 0x20;
pub const RF_IMAGECAL_IMAGECAL_DONE: u8 = 0x00;
