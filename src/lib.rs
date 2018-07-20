//! SX127x Radio Driver
//! Copyright 2018 Ryan Kurte

#![no_std]
#![feature(never_type)]
#![feature(unproven)]
extern crate embedded_hal as hal;
extern crate futures;
extern crate libc;
extern crate nb;

use core::mem;
use core::ptr;

use hal::blocking::{spi, delay};
use hal::digital::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};

pub mod device;
use device::{State};
pub mod regs;

pub mod sx1276;
use sx1276::{SX1276_s};

pub use sx1276::RadioSettings_t as Settings;

/// SX127x SPI operating mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// SX127x device object
#[repr(C)]
pub struct SX127x<SPI, OUTPUT, INPUT, DELAY> {
    spi: SPI,
    sdn: OUTPUT,
    cs: OUTPUT,
    gpio: [Option<INPUT>; 4],
    delay: DELAY,
    state: State,
    c: SX1276_s,
}

extern fn DelayMs(ms: u32) {

}
extern fn SX1276Reset(sx1276: *mut SX1276_s) {

}
extern fn SX1276WriteBuffer(sx1276: *mut SX1276_s, addr: u8, buffer: *mut u8, size: u8) {

}
extern fn SX1276ReadBuffer(sx1276: *mut SX1276_s, addr: u8, buffer: *mut u8, size: u8) {

}

pub enum Sx127xError<SPIError> {
    SPI(SPIError)
}

impl <SPIError>From<SPIError> for Sx127xError<SPIError> {
	fn from(e: SPIError) -> Sx127xError<SPIError> {
		Sx127xError::SPI(e)
	}
}


impl<E, SPI, OUTPUT, INPUT, DELAY> SX127x<SPI, OUTPUT, INPUT, DELAY>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    OUTPUT: OutputPin,
    INPUT: InputPin,
    DELAY: delay::DelayMs<usize>,
{
    pub fn new(spi: SPI, sdn: OUTPUT, cs: OUTPUT, gpio: [Option<INPUT>; 4], delay: DELAY, settings: Settings) -> Result<Self, Sx127xError<E>> {
        unsafe {
            let mut sx127x = SX127x { spi, sdn, cs, gpio, delay, state: State::Idle, c: SX1276_s{settings: settings, ctx: mem::uninitialized()} };
            let mut sx127x_pty = &sx127x as *mut SX127x<SPI, OUTPUT, INPUT, DELAY>;
            sx127x.c.ctx = sx127x_pty as *mut libc::c_void;

            // Reset IC
            sx127x.sdn.set_low();
            sx127x.delay.delay_ms(1);
            sx127x.sdn.set_high();
            sx127x.delay.delay_ms(10);

            // Calibrate RX chain

            // Init IRQs (..?)

            // Confiure modem(s)

            // Set state to idle
            sx127x.state = State::Idle;

            Ok(sx127x)
        }
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
