
// Offsets for RSSI calculation
const RSSI_OFFSET_LF: usize = -164;
const RSSI_OFFSET_HF: usize = -157;

// FSK bandwidth register values
pub enum FskBandwidth
{
    Bandwidth2600   = 0x17,
    Bandwidth3100   = 0x0F,
    Bandwidth3900   = 0x07,
    Bandwidth5200   = 0x16,
    Bandwidth6300   = 0x0E,
    Bandwidth7800   = 0x06,
    Bandwidth10400  = 0x15,
    Bandwidth12500  = 0x0D,
    Bandwidth15600  = 0x05,
    Bandwidth20800  = 0x14,
    Bandwidth25000  = 0x0C,
    Bandwidth31300  = 0x04,
    Bandwidth41700  = 0x13,
    Bandwidth50000  = 0x0B,
    Bandwidth62500  = 0x03,
    Bandwidth83333  = 0x12,
    Bandwidth100000 = 0x0A,
    Bandwidth125000 = 0x02,
    Bandwidth166700 = 0x11,
    Bandwidth200000 = 0x09,
    Bandwidth250000 = 0x01,
}

