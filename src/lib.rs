//! SX127x Radio Driver
//! Copyright 2018 Ryan Kurte

#![no_std]
#![feature(never_type)]
#![feature(unproven)]
extern crate embedded_hal as hal;
extern crate futures;
extern crate libc;
extern crate nb;

use hal::blocking::spi;
use hal::digital::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};

pub mod device;
use device::{State};
pub mod regs;

pub mod sx1276;
use sx1276::{SX1276_s};

/// SX127x SPI operating mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// SX127x device object
pub struct SX127x<SPI, OUTPUT, INPUT> {
    spi: SPI,
    sdn: OUTPUT,
    cs: OUTPUT,
    gpio: [Option<INPUT>; 4],
    state: State,
    c: SX1276_s,
}

pub struct SPIHelper {
    
}


extern fn SpiInOut() {

}

extern fn GpioWrite() {

}

extern fn DelayMs(ms: u32) {

}

pub enum Sx127xError<SPIError> {
    SPI(SPIError)
}

impl <SPIError>From<SPIError> for Sx127xError<SPIError> {
	fn from(e: SPIError) -> Sx127xError<SPIError> {
		Sx127xError::SPI(e)
	}
}

pub enum Modem {
    FSK = 0,
    LoRA = 1,
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

impl<E, SPI, OUTPUT, INPUT> SX127x<SPI, OUTPUT, INPUT>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    OUTPUT: OutputPin,
    INPUT: InputPin,
{
    pub fn new(spi: SPI, sdn: OUTPUT, cs: OUTPUT, gpio: [Option<INPUT>; 4]) -> Result<Self, Sx127xError<E>> {
        let mut SX127x = SX127x { spi, sdn, cs, gpio, state: State::Idle, c: SX1276_s{} };

        // Reset IC
        SX127x.sdn.set_low();

        // Calibrate RX chain

        // Init IRQs (..?)

        // Confiure modem(s)

        // Set state to idle
        SX127x.state = State::Idle;

        Ok(SX127x)
    }

    /// Read data from a specified register address
    /// This consumes the provided input data array and returns a reference to this on success
    fn reg_read<'a>(&mut self, reg: u8, data: &'a mut [u8]) -> Result<&'a [u8], Sx127xError<E>> {
        // Setup read command
        let out_buf: [u8; 1] = [reg as u8 & 0x7F];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&out_buf) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(Sx127xError::SPI(e));
            }
        };
        // Transfer data
        let res = match self.spi.transfer(data) {
            Ok(r) => r,
            Err(e) => {
                self.cs.set_high();
                return Err(Sx127xError::SPI(e));
            }
        };
        // Clear CS
        self.cs.set_high();
        // Return result (contains returned data)
        Ok(res)
    }

    /// Write data to a specified register address
    pub fn reg_write(&mut self, reg: u8, data: &[u8]) -> Result<(), Sx127xError<E>> {
        // Setup write command
        let out_buf: [u8; 1] = [reg as u8 | 0x80];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&out_buf) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(Sx127xError::SPI(e));
            }
        };
        // Transfer data
        match self.spi.write(&data) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(Sx127xError::SPI(e));
            }
        };
        // Clear CS
        self.cs.set_high();

        Ok(())
    }

    // Calculate a channel number from a given frequency
    pub fn freq_to_channel(freq: f32) -> u32 {
        (freq / device::FREQ_STEP) as u32
    }

    // Set the channel
    pub fn set_channel(&mut self, channel: u32) -> Result<(), Sx127xError<E>> {
        self.reg_write(regs::Common::FRFMSB as u8, &[(channel >> 16) as u8])?;
        self.reg_write(regs::Common::FRFMID as u8, &[(channel >> 8) as u8])?;
        self.reg_write(regs::Common::FRFLSB as u8, &[(channel >> 0) as u8])?;
        Ok(())
    }

    pub fn channel_clear(&mut self) -> Result<bool, Sx127xError<E>> {
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
