//! Sx127x Radio Driver
//! Copyright 2018 Ryan Kurte

#![no_std]

extern crate libc;
extern crate log;

use core::convert::TryFrom;

extern crate embedded_hal as hal;
use hal::blocking::{delay};
use hal::digital::v2::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};
use hal::blocking::spi::{Transfer, Write};

extern crate embedded_spi;
use embedded_spi::{Error as WrapError, wrapper::Wrapper as SpiWrapper};

pub mod bindings;

#[cfg(feature = "ffi")]
use bindings::{SX1276_t};

pub mod base;

#[cfg(feature = "ffi")]
pub mod ffi;


pub mod device;
use device::{State};
pub mod regs;

/// Sx127x Spi operating mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// Sx127x device object
pub struct Sx127x<Hal, CommsError, OutputPin, InputPin, PinError, Delay>{
    hal: Hal,

    delay: Delay,

    sdn: OutputPin,

    _dio: [Option<InputPin>; 3],

    #[cfg(feature = "ffi")]
    c: Option<SX1276_t>,

    #[cfg(feature = "ffi")]
    err: Option<Sx127xError<CommsError, PinError>>,
}


pub struct Settings {

}

impl Default for Settings {
    fn default() -> Self {
        Self{}
    }
}

/// Sx127x error type
#[derive(Debug, Clone, PartialEq)]
pub enum Sx127xError<CommsError, PinError> {
    /// Communications (SPI or UART) error
    Comms(CommsError),
    /// Pin control error
    Pin(PinError),
    /// Transaction aborted
    Aborted,
    /// Invalid response from device
    InvalidResponse,
}

impl <CommsError, PinError> From<WrapError<CommsError, PinError>> for Sx127xError<CommsError, PinError> {
    fn from(e: WrapError<CommsError, PinError>) -> Self {
        match e {
            WrapError::Spi(e) => Sx127xError::Comms(e),
            WrapError::Pin(e) => Sx127xError::Pin(e),
            WrapError::Aborted => Sx127xError::Aborted,
        }
    }
}

impl<Spi, CommsError, Output, Input, PinError, Delay> Sx127x<SpiWrapper<Spi, CommsError, Output, Input, PinError>, CommsError, Output, Input, PinError, Delay>
where
    Spi: Transfer<u8, Error = CommsError> + Write<u8, Error = CommsError>,
    Output: OutputPin<Error = PinError>,
    Input: InputPin<Error = PinError>,
    Delay: delay::DelayMs<u32>,
{
    /// Create an Sx127x with the provided `Spi` implementation and pins
    pub fn spi(spi: Spi, cs: Output, busy: Input, sdn: Output, delay: Delay, settings: Settings) -> Result<Self, Sx127xError<CommsError, PinError>> {
        // Create SpiWrapper over spi/cs/busy
        let mut hal = SpiWrapper::new(spi, cs);
        hal.with_busy(busy);
        // Create instance with new hal
        Self::new(hal, sdn, delay, settings)
    }
}



impl<Hal, CommsError, Output, Input, PinError, Delay> Sx127x<Hal, CommsError, Output, Input, PinError, Delay>
where
    Hal: base::Hal<CommsError, PinError>,
    Output: OutputPin<Error = PinError>,
    Input: InputPin<Error = PinError>,
    Delay: delay::DelayMs<u32>,
{
    pub fn new(hal: Hal, sdn: Output, delay: Delay, settings: Settings) -> Result<Self, Sx127xError<CommsError, PinError>> {
        // Build container object
        let mut sx127x = Self::build(hal, sdn, delay, settings);

        // Reset IC
        sx127x.reset()?;

        // Calibrate RX chain
        //sx1276::RxChainCalibration(&sx127x.c);

        // Init IRQs (..?)

        // Confiure modem(s)

        // Set state to idle


        Ok(sx127x)
    }

    pub(crate) fn build(hal: Hal, sdn: Output, delay: Delay, _settings: Settings) -> Self {
        Sx127x { 
            hal, sdn, delay, 
            _dio: [None, None, None],
            #[cfg(feature = "ffi")]
            c: None, 
            #[cfg(feature = "ffi")]
            err: None,
        }
    }

    /// Fetch device silicon version
    pub fn silicon_version(&mut self) -> Result<u8, Sx127xError<CommsError, PinError>> {
        let mut data = [0u8; 1];
        self.hal.reg_read(regs::Common::VERSION as u16, &mut data)?;
        Ok(data[0])
    }

    /// Fetch device state
    pub fn get_state(&mut self) -> Result<State, Sx127xError<CommsError, PinError>> {
        let mut data = [0u8; 1];
        self.hal.reg_read(regs::Common::OPMODE as u16, &mut data)?;
        let state = State::try_from(data[0]).map_err(|_| Sx127xError::InvalidResponse)?;
        Ok(state)
    }

    // Calculate a channel number from a given frequency
    pub fn freq_to_channel(freq: f32) -> u32 {
        (freq / device::FREQ_STEP) as u32
    }

    // Set the channel
    pub fn set_channel(&mut self, channel: u32) -> Result<(), Sx127xError<CommsError, PinError>> {
        self.hal.reg_write(regs::Common::FRFMSB as u16, &[(channel >> 16) as u8])?;
        self.hal.reg_write(regs::Common::FRFMID as u16, &[(channel >> 8) as u8])?;
        self.hal.reg_write(regs::Common::FRFLSB as u16, &[(channel >> 0) as u8])?;
        Ok(())
    }

    pub fn channel_clear(&mut self) -> Result<bool, Sx127xError<CommsError, PinError>> {
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
