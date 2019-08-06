//! Sx127x Rust Radio Driver
//! 
//! This library implements a driver for the Semtec Sx127x series of sub-GHz ISM band Radio ICs.
//! 
//! For use with [embedded-hal](https://github.com/rust-embedded/embedded-hal) implementations.
//! 
// Copyright 2018 Ryan Kurte

#![no_std]

extern crate libc;
#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate log;
#[macro_use]
extern crate serde;

use core::convert::TryFrom;
use core::marker::PhantomData;

extern crate embedded_hal as hal;
use hal::blocking::{delay};
use hal::digital::v2::{InputPin, OutputPin};
use hal::spi::{Mode as SpiMode, Phase, Polarity};
use hal::blocking::spi::{Transfer, Write};

extern crate embedded_spi;
use embedded_spi::{Error as WrapError, wrapper::Wrapper as SpiWrapper};

extern crate radio;
use radio::{State as _, Power as _};

#[cfg(feature = "ffi")]
pub mod bindings;

#[cfg(feature = "ffi")]
use bindings::{SX1276_t};

pub mod base;

#[cfg(feature = "ffi")]
pub mod ffi;

pub mod device;
use device::{State, ModemMode, PaConfig, PaSelect, regs};
use device::lora::LoRaConfig;

pub mod lora;


pub mod fsk;

pub mod prelude;

/// Sx127x Spi operating mode (MODE: 0, CPOL: 0, CPHA: 0)
pub const SPI_MODE: SpiMode = SpiMode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// Sx127x device object
/// 
/// Operating functions are implemented as traits from the [radio] package
/// 
pub struct Sx127x<Base, CommsError, PinError, Config>{
    hal: Base,

    #[cfg(feature = "ffi")]
    c: Option<SX1276_t>,

    #[cfg(feature = "ffi")]
    err: Option<Error<CommsError, PinError>>,

    _ce: PhantomData<CommsError>,
    _pe: PhantomData<PinError>,

    settings: Settings,

    config: Config,
}

/// Initial radio configuration object
/// This contains general information for either radio configuration
#[derive(Clone, PartialEq, Debug)]
pub struct Settings {
    /// Radio crystal frequency (defaults to 32MHz)
    pub xtal_freq: u32,
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
pub enum Error<CommsError, PinError> {
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
    /// Received packet exceeds buffer size
    BufferSize,
    /// Invalid or unrecognised device
    InvalidDevice(u8),
}

impl <CommsError, PinError> From<WrapError<CommsError, PinError>> for Error<CommsError, PinError> {
    fn from(e: WrapError<CommsError, PinError>) -> Self {
        match e {
            WrapError::Spi(e) => Error::Comms(e),
            WrapError::Pin(e) => Error::Pin(e),
            WrapError::Aborted => Error::Aborted,
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
    /// Create an Sx127x with the provided SPI implementation and pins
    pub fn spi(spi: Spi, cs: Output, busy: Input, sdn: Output, delay: Delay, settings: Settings) -> Result<Self, Error<CommsError, PinError>> {
        // Create SpiWrapper over spi/cs/busy
        let mut hal = SpiWrapper::new(spi, cs, delay);
        hal.with_busy(busy);
        hal.with_reset(sdn);
        // Create instance with new hal
        Self::new(hal, settings)
    }
}



impl<Base, CommsError, PinError> Sx127x<Base, CommsError, PinError, ()>
where
    Base: base::Base<CommsError, PinError>,
{
    /// Create a new radio instance over an arbitrary base::Base implementation
    pub fn new(hal: Base, settings: Settings) -> Result<Self, Error<CommsError, PinError>> {
        

        // Build container object
        let mut sx127x = Self::build(hal, settings, ());

        // Reset IC
        sx127x.hal.reset()?;

        let version = sx127x.silicon_version()?;
        if version != 0x12 {
            return Err(Error::InvalidDevice(version))
        }

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
    pub fn lora(self, lora_config: LoRaConfig) -> Result<Sx127x<Base, CommsError, PinError, LoRaConfig>, Error<CommsError, PinError>> {
        // Destructure existing object
        let Self{hal, settings, config: _, _ce, _pe} = self;

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
        s.configure_lora(&lora_config)?;

        // Return new object
        Ok(s)
    }
}

impl<Base, CommsError, PinError, Config> Sx127x<Base, CommsError, PinError, Config>
where
    Base: base::Base<CommsError, PinError>,
{
    pub(crate) fn build(hal: Base, settings: Settings, config: Config) -> Self {
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

impl<Base, CommsError, PinError, Config> Sx127x<Base, CommsError, PinError, Config>
where
    Base: base::Base<CommsError, PinError>,
{
    /// Read a u8 value from the specified register
    pub fn read_reg<R>(&mut self, reg: R) -> Result<u8, Error<CommsError, PinError>> 
    where R: Copy + Clone + Into<u8> {
        self.hal.read_reg(reg.into())
    }

    /// Write a u8 value to the specified register
    pub fn write_reg<R>(&mut self, reg: R, value: u8) -> Result<(), Error<CommsError, PinError>> 
    where R: Copy + Clone + Into<u8> {
        self.hal.write_reg(reg.into(), value)
    }

    /// Update the specified register with the provided (value & mask)
    pub fn update_reg<R>(&mut self, reg: R, mask: u8, value: u8) -> Result<u8, Error<CommsError, PinError>> 
    where R: Copy + Clone + Into<u8> {
        self.hal.update_reg(reg.into(), mask, value)
    }
}

impl<Base, CommsError, PinError, Config> radio::State for Sx127x<Base, CommsError, PinError, Config>
where
    Base: base::Base<CommsError, PinError>,
{
    type State = State;
    type Error = Error<CommsError, PinError>;

    /// Fetch device state
    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        let state = self.read_reg(regs::Common::OPMODE)?;
        let state = State::try_from(state & device::OPMODE_STATE_MASK ).map_err(|_| Error::InvalidResponse)?;
        Ok(state)
    }

    /// Set device state
    fn set_state(&mut self, state: Self::State) -> Result<(), Self::Error> {
        self.update_reg(regs::Common::OPMODE, device::OPMODE_STATE_MASK, state as u8)?;
        Ok(())
    }

}

impl<Base, CommsError, PinError, Config> radio::Power for Sx127x<Base, CommsError, PinError, Config>
where
    Base: base::Base<CommsError, PinError>,
{
    type Error = Error<CommsError, PinError>;

    /// Set transmit power (using the existing `PaConfig`)
    fn set_power(&mut self, power: i8) -> Result<(), Error<CommsError, PinError>> {
        use device::*;

        // Limit to viable input range
        let power = core::cmp::max(power, 0);

        // Read from config to determine PA mode
        let config = self.read_reg(regs::Common::PACONFIG)?;

        match config & PASELECT_MASK {
            PASELECT_RFO => {
                let max = ((config & MAXPOWER_MASK) >> MAXPOWER_SHIFT) as i8;

                let power = core::cmp::min(power, 17i8);
                let value = power - max + 15;

                let v = self.update_reg(regs::Common::PACONFIG, OUTPUTPOWER_MASK, value as u8)?;

                debug!("Updated RFO PA_CONFIG for: {} dBm to: {:b}", power, v);
            },
            PASELECT_PA_BOOST => {

                let power = core::cmp::min(power, 20i8);
                let value = power - 17 + 15;

                self.update_reg(regs::Common::PACONFIG, OUTPUTPOWER_MASK, value as u8)?;

                let pa_dac_enable = if power > 17 { PADAC_20DBM_ON } else { PADAC_20DBM_OFF };
                let v = self.update_reg(regs::Common::PADAC, PADAC_MASK, pa_dac_enable)?;

                debug!("Updated BOOST PA_CONFIG for: {} dBm to: {:b}", power, v);
            },
            _ => ()
        }

        Ok(())
    }
}

impl<Base, CommsError, PinError, Config> Sx127x<Base, CommsError, PinError, Config>
where
    Base: base::Base<CommsError, PinError>,
{
        
    /// Fetch device silicon version
    pub fn silicon_version(&mut self) -> Result<u8, Error<CommsError, PinError>> {
        self.read_reg(regs::Common::VERSION)
    }


    pub(crate) fn configure_pa(&mut self, pa_config: &PaConfig) -> Result<(), Error<CommsError, PinError>> {
        use device::*;
        
        // Set output configuration
        match pa_config.output {
            PaSelect::Rfo(max) => {
                self.update_reg(regs::Common::PACONFIG,
                    PASELECT_MASK | MAXPOWER_MASK,
                    PASELECT_RFO | ((max << MAXPOWER_SHIFT) & MAXPOWER_MASK)
                )?;
            },
            PaSelect::Boost => {
                self.update_reg(regs::Common::PACONFIG, PASELECT_MASK, PASELECT_PA_BOOST)?;
            }
        }

        self.set_power(config.power)?;

        Ok(())
    }

    pub(crate) fn set_state_checked(&mut self, state: State) -> Result<(), Error<CommsError, PinError>> {
        self.set_state(state)?;
        loop {
            let s = self.get_state()?;
            if state == s {
                break;
            }
            self.hal.delay_ms(1);
        }
        Ok(())
    }

    // Calculate a channel number from a given frequency
    pub(crate) fn freq_to_channel_index(&self, freq: u32) -> u32 {
        let step = (self.settings.xtal_freq as f32) / (2u32.pow(19) as f32);
        let ch = (freq as f32) / step;
        ch as u32
    }

    pub(crate) fn channel_index_to_freq(&self, ch: u32) -> u32 {
        let step = (self.settings.xtal_freq as f32) / (2u32.pow(19) as f32);
        let freq = (ch as f32) * step;
        freq as u32
    }

    /// Set operating mode for the device
    pub(crate) fn set_modem(&mut self, modem: ModemMode) -> Result<(), Error<CommsError, PinError>> {
        match modem {
            ModemMode::Standard => {
                self.set_state(State::Sleep)?;
                self.update_reg(regs::Common::OPMODE, device::OPMODE_LONGRANGEMODE_MASK, device::LongRangeMode::Off as u8)?;
            },
            ModemMode::LoRa => {
                self.set_state(State::Sleep)?;
                self.update_reg(regs::Common::OPMODE, device::OPMODE_LONGRANGEMODE_MASK, device::LongRangeMode::On as u8)?;
            }
        }

        Ok(())
    }

    // Set the channel by frequency
    pub (crate) fn set_frequency(&mut self, freq: u32) -> Result<(), Error<CommsError, PinError>> {
        let channel = self.freq_to_channel_index(freq);
        
        let outgoing = [
            (channel >> 16) as u8,
            (channel >> 8) as u8,
            (channel >> 0) as u8,
        ];
        
        self.hal.write_regs(regs::Common::FRFMSB as u8, &outgoing)?;

        Ok(())
    }

    // Fetch the current channel index
    pub (crate) fn get_frequency(&mut self) -> Result<u32, Error<CommsError, PinError>> {
        let mut incoming = [0u8; 3];

        self.hal.read_regs(regs::Common::FRFMSB as u8, &mut incoming)?;
        let ch = (incoming[0] as u32) << 16 | (incoming[1] as u32) << 8 | (incoming[2] as u32) << 0;

        let freq = self.channel_index_to_freq(ch);

        Ok(freq)
    }


    /// Calibrate the device RF chain
    /// This MUST be called directly after resetting the module
    pub(crate) fn rf_chain_calibration(&mut self) -> Result<(), Error<CommsError, PinError>> {
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
