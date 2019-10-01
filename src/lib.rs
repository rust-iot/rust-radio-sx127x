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
use core::fmt::Debug;

extern crate embedded_hal as hal;
use hal::blocking::delay;
use hal::blocking::spi::{Transfer, Write};
use hal::digital::v2::{InputPin, OutputPin};
use hal::spi::{Mode as SpiMode, Phase, Polarity};

extern crate embedded_spi;
use embedded_spi::{wrapper::Wrapper as SpiWrapper, Error as WrapError};

extern crate radio;
use radio::{Power as _, State as _};

pub mod base;

pub mod device;
use device::{regs, Channel, Interrupts, Config, Modem, ModemMode, PaConfig, PacketInfo, State};

pub mod fsk;
pub mod lora;

pub mod prelude;

/// Sx127x Spi operating mode (MODE: 0, CPOL: 0, CPHA: 0)
pub const SPI_MODE: SpiMode = SpiMode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// Sx127x operating mode enumeration
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Mode {
    Unconfigured,
    LoRa,
    FskOok,
}

/// Sx127x device object
///
/// Operating functions are implemented as traits from the [radio] package
///
pub struct Sx127x<Base, CommsError, PinError> {
    hal: Base,

    _ce: PhantomData<CommsError>,
    _pe: PhantomData<PinError>,

    mode: Mode,
    config: Config,
}

/// Sx127x error type
#[derive(Debug, Clone, PartialEq)]
pub enum Error<CommsError, PinError> {
    /// Communications (SPI or UART) error
    Comms(CommsError),
    /// Pin control error
    Pin(PinError),
    /// Invalid configuration
    InvalidConfiguration,
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

impl<CommsError, PinError> From<WrapError<CommsError, PinError>> for Error<CommsError, PinError> {
    fn from(e: WrapError<CommsError, PinError>) -> Self {
        match e {
            WrapError::Spi(e) => Error::Comms(e),
            WrapError::Pin(e) => Error::Pin(e),
            WrapError::Aborted => Error::Aborted,
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct Settings {
    /// Device crystal frequency (defaults to 32MHz)
    pub xtal_freq: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            xtal_freq: 32000000,
        }
    }
}

impl<Spi, CommsError, Output, Input, PinError, Delay>
    Sx127x<SpiWrapper<Spi, CommsError, Output, Input, PinError, Delay>, CommsError, PinError>
where
    Spi: Transfer<u8, Error = CommsError> + Write<u8, Error = CommsError>,
    Output: OutputPin<Error = PinError>,
    Input: InputPin<Error = PinError>,
    Delay: delay::DelayMs<u32>,
{
    /// Create an Sx127x with the provided SPI implementation and pins
    pub fn spi(
        spi: Spi,
        cs: Output,
        busy: Input,
        sdn: Output,
        delay: Delay,
        config: &Config,
    ) -> Result<Self, Error<CommsError, PinError>> {
        // Create SpiWrapper over spi/cs/busy
        let mut hal = SpiWrapper::new(spi, cs, delay);
        hal.with_busy(busy);
        hal.with_reset(sdn);
        // Create instance with new hal
        Self::new(hal, config)
    }
}

impl<Base, CommsError, PinError> Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    /// Create a new radio instance over an arbitrary base::Base implementation
    pub fn new(hal: Base, config: &Config) -> Result<Self, Error<CommsError, PinError>> {
        // Build container object
        let mut sx127x = Self::build(hal, config.clone());

        // Reset IC
        sx127x.hal.reset()?;

        let version = sx127x.silicon_version()?;
        if version != 0x12 {
            return Err(Error::InvalidDevice(version));
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

        // Configure
        sx127x.configure(config)?;

        Ok(sx127x)
    }

    /// (re)apply device configuration
    pub fn configure(&mut self, config: &Config) -> Result<(), Error<CommsError, PinError>> {
        // Configure the modem as appropriate
        match (&config.modem, &config.channel) {
            (Modem::LoRa(lora_modem), Channel::LoRa(lora_channel)) => {
                self.lora_configure(lora_modem, lora_channel)?;
            }
            (Modem::FskOok(fsk_modem), Channel::FskOok(fsk_channel)) => {
                self.fsk_configure(fsk_modem, fsk_channel)?;
            }
            _ => panic!("Invalid configuration, mismatch between Modem ({:?}) and Channel ({:?}  modes", &config.modem, &config.channel),
        }

        // Configure power amplifier
        self.configure_pa(&config.pa_config)?;

        // Update config
        self.config = config.clone();

        Ok(())
    }

    /// Fetch device silicon version
    pub fn silicon_version(&mut self) -> Result<u8, Error<CommsError, PinError>> {
        self.read_reg(regs::Common::VERSION)
    }

    /// Configure power amplifier
    pub(crate) fn configure_pa(
        &mut self,
        pa_config: &PaConfig,
    ) -> Result<(), Error<CommsError, PinError>> {
        use device::*;

        // Set output configuration
        match pa_config.output {
            PaSelect::Rfo(max) => {
                self.update_reg(
                    regs::Common::PACONFIG,
                    PASELECT_MASK | MAXPOWER_MASK,
                    PASELECT_RFO | ((max << MAXPOWER_SHIFT) & MAXPOWER_MASK),
                )?;
            }
            PaSelect::Boost => {
                self.update_reg(regs::Common::PACONFIG, PASELECT_MASK, PASELECT_PA_BOOST)?;
            }
        }

        self.set_power(pa_config.power)?;

        Ok(())
    }

    pub(crate) fn set_state_checked(
        &mut self,
        state: State,
    ) -> Result<(), Error<CommsError, PinError>> {
        trace!("Set state to: {:?} (0x{:02x})", state, state as u8);
        self.set_state(state)?;
        loop {
            let s = self.get_state()?;
            trace!("Received: {:?}", s);
            if state == s {
                break;
            }
            self.hal.delay_ms(1);
        }
        Ok(())
    }

    // Calculate a channel number from a given frequency
    pub(crate) fn freq_to_channel_index(&self, freq: u32) -> u32 {
        let step = (self.config.xtal_freq as f32) / (2u32.pow(19) as f32);
        let ch = (freq as f32) / step;
        ch as u32
    }

    pub(crate) fn channel_index_to_freq(&self, ch: u32) -> u32 {
        let step = (self.config.xtal_freq as f32) / (2u32.pow(19) as f32);
        let freq = (ch as f32) * step;
        freq as u32
    }

    /// Set operating mode for the device
    pub(crate) fn set_modem(
        &mut self,
        modem: ModemMode,
    ) -> Result<(), Error<CommsError, PinError>> {
        match modem {
            ModemMode::Standard => {
                self.set_state(State::Sleep)?;
                self.update_reg(
                    regs::Common::OPMODE,
                    device::OPMODE_LONGRANGEMODE_MASK,
                    device::LongRangeMode::Off as u8,
                )?;
            }
            ModemMode::LoRa => {
                self.set_state(State::Sleep)?;
                self.update_reg(
                    regs::Common::OPMODE,
                    device::OPMODE_LONGRANGEMODE_MASK,
                    device::LongRangeMode::On as u8,
                )?;
            }
        }

        Ok(())
    }

    // Set the channel by frequency
    pub(crate) fn set_frequency(&mut self, freq: u32) -> Result<(), Error<CommsError, PinError>> {
        let channel = self.freq_to_channel_index(freq);

        let outgoing = [
            (channel >> 16) as u8,
            (channel >> 8) as u8,
            (channel >> 0) as u8,
        ];

        debug!("Set channel to index: {:?} (freq: {:?})", channel, freq);

        self.hal.write_regs(regs::Common::FRFMSB as u8, &outgoing)?;

        Ok(())
    }

    // Fetch the current channel index
    pub(crate) fn get_frequency(&mut self) -> Result<u32, Error<CommsError, PinError>> {
        let mut incoming = [0u8; 3];

        self.hal
            .read_regs(regs::Common::FRFMSB as u8, &mut incoming)?;
        let ch = (incoming[0] as u32) << 16 | (incoming[1] as u32) << 8 | (incoming[2] as u32) << 0;

        let freq = self.channel_index_to_freq(ch);

        Ok(freq)
    }

    /// Calibrate the device RF chain
    /// This MUST be called directly after resetting the module
    pub(crate) fn rf_chain_calibration(&mut self) -> Result<(), Error<CommsError, PinError>> {
        debug!("Running calibration");

        // Load initial PA config
        let frequency = self.get_frequency()?;
        let pa_config = self.read_reg(regs::Common::PACONFIG)?;

        // Zero out PA config
        self.write_reg(regs::Common::PACONFIG, 0x00)?;

        // Launch calibration for the LF band
        self.update_reg(
            regs::Fsk::IMAGECAL,
            regs::RF_IMAGECAL_IMAGECAL_MASK as u8,
            regs::RF_IMAGECAL_IMAGECAL_START as u8,
        )?;

        // Block on calibration complete
        // TODO: make this fallible with a timeout?
        while self.read_reg(regs::Fsk::IMAGECAL)? & (regs::RF_IMAGECAL_IMAGECAL_RUNNING as u8) != 0
        {
        }

        // Set a channel in the HF band
        self.set_frequency(868e6 as u32)?;

        // Launch calibration for the HF band
        self.update_reg(
            regs::Fsk::IMAGECAL,
            regs::RF_IMAGECAL_IMAGECAL_MASK as u8,
            regs::RF_IMAGECAL_IMAGECAL_START as u8,
        )?;

        // Block on calibration complete
        // TODO: make this fallible with a timeout?
        while self.read_reg(regs::Fsk::IMAGECAL)? & (regs::RF_IMAGECAL_IMAGECAL_RUNNING as u8) != 0
        {
        }

        // Restore PA config and channel
        self.set_frequency(frequency)?;
        self.write_reg(regs::Common::PACONFIG, pa_config)?;

        debug!("Calibration done");

        Ok(())
    }
}

impl<Base, CommsError, PinError> Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    pub(crate) fn build(hal: Base, config: Config) -> Self {
        Sx127x {
            hal,
            config,
            mode: Mode::Unconfigured,
            _ce: PhantomData,
            _pe: PhantomData,
        }
    }
}

impl<Base, CommsError, PinError> delay::DelayMs<u32> for Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    fn delay_ms(&mut self, t: u32) {
        self.hal.delay_ms(t)
    }
}

impl<Base, CommsError, PinError> Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    /// Read a u8 value from the specified register
    pub fn read_reg<R>(&mut self, reg: R) -> Result<u8, Error<CommsError, PinError>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        let value = self.hal.read_reg(reg.into())?;

        trace!("Read reg:   {:?} (0x{:02x}): 0x{:02x}", reg, reg.into(), value);

        Ok(value)
    }

    /// Write a u8 value to the specified register
    pub fn write_reg<R>(&mut self, reg: R, value: u8) -> Result<(), Error<CommsError, PinError>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        trace!("Write reg:  {:?} (0x{:02x}): 0x{:02x}", reg, reg.into(), value);

        self.hal.write_reg(reg.into(), value)
    }

    /// Update the specified register with the provided (value & mask)
    pub fn update_reg<R>(
        &mut self,
        reg: R,
        mask: u8,
        value: u8,
    ) -> Result<u8, Error<CommsError, PinError>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        trace!("Update reg: {:?} (0x{:02x}): 0x{:02x} (0x{:02x})", reg, reg.into(), value, mask);

        self.hal.update_reg(reg.into(), mask, value)
    }
}

impl<Base, CommsError, PinError> radio::State for Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    type State = State;
    type Error = Error<CommsError, PinError>;

    /// Fetch device state
    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        let state = self.read_reg(regs::Common::OPMODE)?;
        let state = State::try_from(state & device::OPMODE_STATE_MASK)
            .map_err(|_| Error::InvalidResponse)?;
        Ok(state)
    }

    /// Set device state
    fn set_state(&mut self, state: Self::State) -> Result<(), Self::Error> {
        self.update_reg(regs::Common::OPMODE, device::OPMODE_STATE_MASK, state as u8)?;
        Ok(())
    }
}

impl<Base, CommsError, PinError> radio::Power for Sx127x<Base, CommsError, PinError>
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
            }
            PASELECT_PA_BOOST => {
                let power = core::cmp::min(power, 20i8);
                let value = power - 17 + 15;

                self.update_reg(regs::Common::PACONFIG, OUTPUTPOWER_MASK, value as u8)?;

                let pa_dac_enable = if power > 17 {
                    PADAC_20DBM_ON
                } else {
                    PADAC_20DBM_OFF
                };
                let v = self.update_reg(regs::Common::PADAC, PADAC_MASK, pa_dac_enable)?;

                debug!("Updated BOOST PA_CONFIG for: {} dBm to: {:b}", power, v);
            }
            _ => (),
        }

        Ok(())
    }
}

impl<Base, CommsError, PinError> radio::Interrupts for Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    type Irq = Interrupts;
    type Error = Error<CommsError, PinError>;

    /// Fetch pending interrupts from the device
    /// If the clear option is set, this will also clear any pending flags
    fn get_interrupts(&mut self, clear: bool) -> Result<Self::Irq, Self::Error> {
        match self.mode {
            Mode::LoRa => Ok(Interrupts::LoRa(self.lora_get_interrupts(clear)?)),
            Mode::FskOok => Ok(Interrupts::FskOok(self.fsk_get_interrupts(clear)?)),
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

impl<Base, CommsError, PinError> radio::Channel for Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    type Channel = Channel;
    type Error = Error<CommsError, PinError>;

    /// Set the LoRa mode channel for future receive or transmit operations
    fn set_channel(&mut self, channel: &Self::Channel) -> Result<(), Error<CommsError, PinError>> {
        match (self.mode, channel) {
            (Mode::LoRa, Channel::LoRa(channel)) => self.lora_set_channel(channel),
            (Mode::FskOok, Channel::FskOok(channel)) => self.fsk_set_channel(channel),
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

impl<Base, CommsError, PinError> radio::Transmit for Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    type Error = Error<CommsError, PinError>;

    /// Start sending a packet
    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        match self.mode {
            Mode::LoRa => self.lora_start_transmit(data),
            Mode::FskOok => self.fsk_start_transmit(data),
            _ => Err(Error::InvalidConfiguration),
        }
    }

    /// Check for transmission completion
    /// This method should be polled (or checked following and interrupt) to indicate sending
    /// has completed
    fn check_transmit(&mut self) -> Result<bool, Error<CommsError, PinError>> {
        match self.mode {
            Mode::LoRa => self.lora_check_transmit(),
            Mode::FskOok => self.fsk_check_transmit(),
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

impl<Base, CommsError, PinError> radio::Receive for Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    type Info = PacketInfo;
    type Error = Error<CommsError, PinError>;

    /// Start receive mode
    fn start_receive(&mut self) -> Result<(), Self::Error> {
        match self.mode {
            Mode::LoRa => self.lora_start_receive(),
            Mode::FskOok => self.fsk_start_receive(),
            _ => Err(Error::InvalidConfiguration),
        }
    }

    /// Check receive state
    ///
    /// This returns true if a boolean indicating whether a packet has been received.
    /// The restart option specifies whether transient timeout or CRC errors should be
    /// internally handled (returning Ok(false)) or passed back to the caller as errors.
    fn check_receive(&mut self, restart: bool) -> Result<bool, Self::Error> {
        match self.mode {
            Mode::LoRa => self.lora_check_receive(restart),
            Mode::FskOok => self.fsk_check_receive(restart),
            _ => Err(Error::InvalidConfiguration),
        }
    }

    /// Fetch a received message
    ///
    /// This copies data into the provided slice, updates the provided information object,
    ///  and returns the number of bytes received on success
    fn get_received(
        &mut self,
        info: &mut Self::Info,
        data: &mut [u8],
    ) -> Result<usize, Self::Error> {
        match self.mode {
            Mode::LoRa => self.lora_get_received(info, data),
            Mode::FskOok => self.fsk_get_received(info, data),
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

impl<Base, CommsError, PinError> radio::Rssi for Sx127x<Base, CommsError, PinError>
where
    Base: base::Base<CommsError, PinError>,
{
    type Error = Error<CommsError, PinError>;

    /// Poll for the current channel RSSI
    /// This should only be called in receive mode
    fn poll_rssi(&mut self) -> Result<i16, Error<CommsError, PinError>> {
        match self.mode {
            Mode::LoRa => self.lora_poll_rssi(),
            Mode::FskOok => self.fsk_poll_rssi(),
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
