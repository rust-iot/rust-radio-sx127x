//! Sx127x Radio Driver
//! Copyright 2018 Ryan Kurte

#![no_std]

extern crate libc;
#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate log;

use core::convert::TryFrom;
use core::marker::PhantomData;

extern crate embedded_hal as hal;
use hal::blocking::{delay};
use hal::digital::v2::{InputPin, OutputPin};
use hal::spi::{Mode as SpiMode, Phase, Polarity};
use hal::blocking::spi::{Transfer, Write};

extern crate embedded_spi;
use embedded_spi::{Error as WrapError, wrapper::Wrapper as SpiWrapper};

#[cfg(feature = "ffi")]
pub mod bindings;

#[cfg(feature = "ffi")]
use bindings::{SX1276_t};

pub mod base;

#[cfg(feature = "ffi")]
pub mod ffi;

pub mod device;
use device::{State, Modem, regs};
pub use device::lora::{Config as LoRaConfig};

pub mod lora;

/// Sx127x Spi operating mode
pub const SPI_MODE: SpiMode = SpiMode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// Sx127x device object
pub struct Sx127x<Hal, CommsError, PinError, Config>{
    hal: Hal,

    #[cfg(feature = "ffi")]
    c: Option<SX1276_t>,

    #[cfg(feature = "ffi")]
    err: Option<Sx127xError<CommsError, PinError>>,

    _ce: PhantomData<CommsError>,
    _pe: PhantomData<PinError>,

    settings: Settings,

    config: Config,
}

/// Initial radio configuration object
/// This contains general information for either radio configuration
#[derive(Clone, PartialEq, Debug)]
pub struct Settings {
    /// Radio crystal frequency
    xtal_freq: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self{
            xtal_freq: 32000000,
        }
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
    /// Timeout while awaiting operation completion
    Timeout,
    /// incoming packet CRC error
    Crc,
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

impl<Spi, CommsError, Output, Input, PinError, Delay> Sx127x<SpiWrapper<Spi, CommsError, Output, Input, PinError, Delay>, CommsError, PinError, ()>
where
    Spi: Transfer<u8, Error = CommsError> + Write<u8, Error = CommsError>,
    Output: OutputPin<Error = PinError>,
    Input: InputPin<Error = PinError>,
    Delay: delay::DelayMs<u32>,
{
    /// Create an Sx127x with the provided `Spi` implementation and pins
    pub fn spi(spi: Spi, cs: Output, busy: Input, sdn: Output, delay: Delay, settings: Settings) -> Result<Self, Sx127xError<CommsError, PinError>> {
        // Create SpiWrapper over spi/cs/busy
        let mut hal = SpiWrapper::new(spi, cs, delay);
        hal.with_busy(busy);
        hal.with_reset(sdn);
        // Create instance with new hal
        Self::new(hal, settings)
    }
}



impl<Hal, CommsError, PinError> Sx127x<Hal, CommsError, PinError, ()>
where
    Hal: base::Hal<CommsError, PinError>,
{
    /// Create a new radio instance
    pub fn new(hal: Hal, settings: Settings) -> Result<Self, Sx127xError<CommsError, PinError>> {
        // Build container object
        let mut sx127x = Self::build(hal, settings, ());

        // Reset IC
        sx127x.hal.reset()?;

        // Calibrate RX chain
        sx127x.rf_chain_calibration()?;

        // Set state to sleep
        sx127x.set_state(State::Sleep)?;

        // Load initial configuration values
        for (modem, reg, val) in device::REGISTERS_INIT {
            sx127x.set_modem(*modem)?;
            sx127x.write_reg(*reg, *val)?;
        }

        Ok(sx127x)
    }

    /// Configure the modem into LoRa mode
    pub fn lora(self, lora_config: LoRaConfig) -> Result<Sx127x<Hal, CommsError, PinError, LoRaConfig>, Sx127xError<CommsError, PinError>> {
        // Destructure existing object
        let Self{hal, settings, config, _ce, _pe} = self;

        // Create new object
        let mut s = Sx127x { 
            hal, settings, 
            config: lora_config.clone(),
            #[cfg(feature = "ffi")]
            c: None, 
            #[cfg(feature = "ffi")]
            err: None,
            _ce,
            _pe,
        };

        // Configure LoRa mode
        s.configure(&lora_config)?;

        // Return new object
        Ok(s)
    }
}

impl<Hal, CommsError, PinError, Config> Sx127x<Hal, CommsError, PinError, Config>
where
    Hal: base::Hal<CommsError, PinError>,
{

    pub(crate) fn build(hal: Hal, settings: Settings, config: Config) -> Self {
        Sx127x { 
            hal, settings, config,
            #[cfg(feature = "ffi")]
            c: None, 
            #[cfg(feature = "ffi")]
            err: None,
            _ce: PhantomData,
            _pe: PhantomData,
        }
    }
}

impl<Hal, CommsError, PinError, Config> Sx127x<Hal, CommsError, PinError, Config>
where
    Hal: base::Hal<CommsError, PinError>,
{

    /// Read a u8 value from the specified register
    pub fn read_reg<R>(&mut self, reg: R) -> Result<u8, Sx127xError<CommsError, PinError>> 
    where R: Copy + Clone + Into<u8> {
        let mut incoming = [0u8; 1];
        self.hal.reg_read(reg.into(), &mut incoming)?;
        Ok(incoming[0])
    }

    /// Write a u8 value to the specified register
    pub fn write_reg<R>(&mut self, reg: R, value: u8) -> Result<(), Sx127xError<CommsError, PinError>> 
    where R: Copy + Clone + Into<u8> {
        self.hal.reg_write(reg.into(), &[value])?;
        Ok(())
    }

    /// Update the specified register with the provided (value & mask)
    pub fn update_reg<R>(&mut self, reg: R, mask: u8, value: u8) -> Result<u8, Sx127xError<CommsError, PinError>> 
    where R: Copy + Clone + Into<u8> {
        let existing = self.read_reg(reg)?;
        let updated = (existing & !mask) | (value & mask);
        self.write_reg(reg, updated)?;
        Ok(updated)
    }

    /// Fetch device silicon version
    pub fn silicon_version(&mut self) -> Result<u8, Sx127xError<CommsError, PinError>> {
        self.read_reg(regs::Common::VERSION)
    }

    /// Fetch device state
    pub fn get_state(&mut self) -> Result<State, Sx127xError<CommsError, PinError>> {
        let state = self.read_reg(regs::Common::OPMODE)?;
        let state = State::try_from(state & device::OPMODE_STATE_MASK ).map_err(|_| Sx127xError::InvalidResponse)?;
        Ok(state)
    }

    /// Set device state
    pub fn set_state(&mut self, state: State) -> Result<(), Sx127xError<CommsError, PinError>> {
        self.update_reg(regs::Common::OPMODE, device::OPMODE_STATE_MASK, state as u8)?;
        Ok(())
    }

    // Calculate a channel number from a given frequency
    fn freq_to_channel_index(&self, freq: u32) -> u32 {
        let step = (self.settings.xtal_freq as f32) / (2u32.pow(19) as f32);
        let ch = (freq as f32) / step;
        ch as u32
    }

    fn channel_index_to_freq(&self, ch: u32) -> u32 {
        let step = (self.settings.xtal_freq as f32) / (2u32.pow(19) as f32);
        let freq = (ch as f32) * step;
        freq as u32
    }

    // Set the channel by frequency
    pub fn set_frequency(&mut self, freq: u32) -> Result<(), Sx127xError<CommsError, PinError>> {
        let channel = self.freq_to_channel_index(freq);
        
        let outgoing = [
            (channel >> 16) as u8,
            (channel >> 8) as u8,
            (channel >> 0) as u8,
        ];
        
        self.hal.reg_write(regs::Common::FRFMSB as u8, &outgoing)?;

        Ok(())
    }

    // Fetch the current channel index
    pub fn get_frequency(&mut self) -> Result<u32, Sx127xError<CommsError, PinError>> {
        let mut incoming = [0u8; 3];

        self.hal.reg_read(regs::Common::FRFMSB as u8, &mut incoming)?;
        let ch = (incoming[0] as u32) << 16 | (incoming[1] as u32) << 8 | (incoming[2] as u32) << 0;

        let freq = self.channel_index_to_freq(ch);

        Ok(freq)
    }

    /// Set operating modem for the device
    pub fn set_modem(&mut self, modem: Modem) -> Result<(), Sx127xError<CommsError, PinError>> {
        match modem {
            Modem::Standard => {
                self.set_state(State::Sleep)?;
                self.update_reg(regs::Common::OPMODE, device::OPMODE_LONGRANGEMODE_MASK, device::LongRangeMode::Off as u8)?;

            },
            Modem::LoRa => {
                self.set_state(State::Sleep)?;
                self.update_reg(regs::Common::OPMODE, device::OPMODE_LONGRANGEMODE_MASK, device::LongRangeMode::On as u8)?;
            }
        }

        Ok(())
    }

    /// Calibrate the device RF chain
    /// This MUST be called directly after resetting the module
    pub fn rf_chain_calibration(&mut self) -> Result<(), Sx127xError<CommsError, PinError>> {
        // Load initial PA config
        let frequency = self.get_frequency()?;
        let pa_config = self.read_reg(regs::Common::PACONFIG)?;

        // Zero out PA config
        self.write_reg(regs::Common::PACONFIG, 0x00)?;

        // Launch calibration for the LF band
        self.update_reg(regs::Fsk::IMAGECAL, regs::RF_IMAGECAL_IMAGECAL_MASK as u8, regs::RF_IMAGECAL_IMAGECAL_START as u8)?;

        // Block on calibration complete
        while self.read_reg(regs::Fsk::IMAGECAL)? & (regs::RF_IMAGECAL_IMAGECAL_RUNNING as u8) != 0 {}

        // Set a channel in the HF band
        self.set_frequency( 868e6 as u32 )?;

        // Launch calibration for the HF band
        self.update_reg(regs::Fsk::IMAGECAL, regs::RF_IMAGECAL_IMAGECAL_MASK as u8, regs::RF_IMAGECAL_IMAGECAL_START as u8)?;

        // Block on calibration complete
        while self.read_reg(regs::Fsk::IMAGECAL)? & (regs::RF_IMAGECAL_IMAGECAL_RUNNING as u8) != 0 {}

        // Restore PA config and channel
        self.set_frequency(frequency)?;
        self.write_reg(regs::Common::PACONFIG, pa_config)?;

        Ok(())
    }
}


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
