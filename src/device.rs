

pub enum State {
    Idle = 0,
    RxRunning = 1,
    TxRunning = 2,
    RdCad = 3,
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