//! Sx127x Rust Radio Driver
//!
//! This library implements a driver for the Semtec Sx127x series of sub-GHz ISM band Radio ICs.
//!
//! For use with [embedded-hal](https://github.com/rust-embedded/embedded-hal) implementations.
//!
// Copyright 2018 Ryan Kurte

#![no_std]

use core::convert::TryFrom;
use core::fmt::Debug;

use base::{Base, HalError};
use log::{debug, trace, warn};

use embedded_hal::delay::DelayUs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{ErrorType, Mode as SpiMode, Phase, Polarity, SpiDevice};

use radio::{Power as _, State as _};

pub mod base;

pub mod device;
use device::{regs, Channel, Config, Interrupts, Modem, ModemMode, PaConfig, PacketInfo, State};

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
pub struct Sx127x<Hal> {
    hal: Hal,

    mode: Mode,
    config: Config,
}

/// Sx127x error type
#[derive(Debug, Clone, PartialEq)]
pub enum Error<HalError: Debug + 'static> {
    /// Communications (SPI, UART, pin, delay) error
    Hal(HalError),
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

impl<HalError: Debug + 'static> From<HalError> for Error<HalError> {
    fn from(e: HalError) -> Self {
        Error::Hal(e)
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

pub type Sx127xSpi<Spi, CsPin, BusyPin, ReadyPin, SdnPin, Delay> =
    Sx127x<base::Base<Spi, CsPin, BusyPin, ReadyPin, SdnPin, Delay>>;

impl<Spi, CsPin, BusyPin, ReadyPin, SdnPin, PinError, Delay>
    Sx127x<Base<Spi, CsPin, BusyPin, ReadyPin, SdnPin, Delay>>
where
    Spi: SpiDevice<u8>,
    <Spi as ErrorType>::Error: Debug,

    CsPin: OutputPin<Error = PinError>,
    BusyPin: InputPin<Error = PinError>,
    ReadyPin: InputPin<Error = PinError>,
    SdnPin: OutputPin<Error = PinError>,
    PinError: Debug,

    Delay: DelayUs,
{
    /// Create an Sx127x with the provided SPI implementation and pins
    pub fn spi(
        spi: Spi,
        cs: CsPin,
        busy: BusyPin,
        ready: ReadyPin,
        sdn: SdnPin,
        delay: Delay,
        config: &Config,
    ) -> Result<Self, Error<HalError<<Spi as ErrorType>::Error, PinError>>> {
        // Create SpiWrapper over spi/cs/busy/ready/reset
        let base = Base {
            spi,
            cs,
            sdn,
            busy,
            ready,
            delay,
        };

        // Create instance with new hal
        Self::new(base, config)
    }
}

impl<Hal> Sx127x<Hal>
where
    Hal: base::Hal,
{
    /// Create a new radio instance over an arbitrary base::Base implementation
    pub fn new(hal: Hal, config: &Config) -> Result<Self, Error<<Hal as base::Hal>::Error>> {
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

    /// Reset the device. This recalibrates and patches the device,
    /// however, still requires configuration.
    pub fn reset(&mut self) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        self.hal.reset()?;

        // Calibrate RX chain
        self.rf_chain_calibration()?;

        // Set state to sleep
        self.set_state(State::Sleep)?;

        // Load initial configuration values
        for (modem, reg, val) in device::REGISTERS_INIT {
            self.set_modem(*modem)?;
            self.write_reg(*reg, *val)?;
        }

        Ok(())
    }

    /// (re)apply device configuration
    pub fn configure(&mut self, config: &Config) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        // Configure the modem as appropriate
        match (&config.modem, &config.channel) {
            (Modem::LoRa(lora_modem), Channel::LoRa(lora_channel)) => {
                self.lora_configure(lora_modem, lora_channel)?;
            }
            (Modem::FskOok(fsk_modem), Channel::FskOok(fsk_channel)) => {
                self.fsk_configure(fsk_modem, fsk_channel)?;
            }
            _ => panic!(
                "Invalid configuration, mismatch between Modem ({:?}) and Channel ({:?}  modes",
                &config.modem, &config.channel
            ),
        }

        // Configure power amplifier
        self.configure_pa(&config.pa_config)?;

        // Update config
        self.config = config.clone();

        Ok(())
    }

    /// Fetch device silicon version
    pub fn silicon_version(&mut self) -> Result<u8, Error<<Hal as base::Hal>::Error>> {
        self.read_reg(regs::Common::VERSION)
    }

    /// Configure power amplifier
    pub(crate) fn configure_pa(
        &mut self,
        pa_config: &PaConfig,
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
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
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        // Send set state command
        trace!("Set state to: {:?} (0x{:02x})", state, state as u8);
        self.set_state(state)?;

        let mut ticks = 0;
        loop {
            // Fetch current state
            let s = self.get_state()?;
            trace!("Received: {:?}", s);

            // Check for expected state
            if state == s {
                break;
            }
            // Timeout eventually
            if ticks >= self.config.timeout_ms {
                warn!("Set state timeout");
                return Err(Error::Timeout);
            }

            self.hal.delay_ms(1);
            ticks += 1;
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
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        match modem {
            ModemMode::Standard => {
                self.set_state(State::Sleep)?;
                self.update_reg(
                    regs::Common::OPMODE,
                    device::OPMODE_LONGRANGEMODE_MASK | device::OPMODE_MODULATION_MASK,
                    device::LongRangeMode::Off as u8 | device::ModulationType::Fsk as u8,
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
    pub(crate) fn set_frequency(
        &mut self,
        freq: u32,
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        let channel = self.freq_to_channel_index(freq);

        let outgoing = [
            (channel >> 16) as u8,
            (channel >> 8) as u8,
            channel as u8,
        ];

        debug!("Set channel to index: {:?} (freq: {:?})", channel, freq);

        self.hal.write_regs(regs::Common::FRFMSB as u8, &outgoing)?;

        Ok(())
    }

    // Fetch the current channel index
    pub(crate) fn get_frequency(&mut self) -> Result<u32, Error<<Hal as base::Hal>::Error>> {
        let mut incoming = [0u8; 3];

        self.hal
            .read_regs(regs::Common::FRFMSB as u8, &mut incoming)?;
        let ch = (incoming[0] as u32) << 16 | (incoming[1] as u32) << 8 | (incoming[2] as u32);

        let freq = self.channel_index_to_freq(ch);

        Ok(freq)
    }

    /// Calibrate the device RF chain
    /// This MUST be called directly after resetting the module
    pub(crate) fn rf_chain_calibration(&mut self) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        debug!("Running calibration");

        // Load initial PA config
        let frequency = self.get_frequency()?;
        let pa_config = self.read_reg(regs::Common::PACONFIG)?;

        // Zero out PA config
        self.write_reg(regs::Common::PACONFIG, 0x00)?;

        // Launch calibration for the LF band
        self.update_reg(
            regs::Fsk::IMAGECAL,
            regs::RF_IMAGECAL_IMAGECAL_MASK,
            regs::RF_IMAGECAL_IMAGECAL_START,
        )?;

        // Block on calibration complete
        // TODO: make this fallible with a timeout?
        while self.read_reg(regs::Fsk::IMAGECAL)? & regs::RF_IMAGECAL_IMAGECAL_RUNNING != 0
        {
        }

        // Set a channel in the HF band
        self.set_frequency(868e6 as u32)?;

        // Launch calibration for the HF band
        self.update_reg(
            regs::Fsk::IMAGECAL,
            regs::RF_IMAGECAL_IMAGECAL_MASK,
            regs::RF_IMAGECAL_IMAGECAL_START,
        )?;

        // Block on calibration complete
        // TODO: make this fallible with a timeout?
        while self.read_reg(regs::Fsk::IMAGECAL)? & regs::RF_IMAGECAL_IMAGECAL_RUNNING != 0
        {
        }

        // Restore PA config and channel
        self.set_frequency(frequency)?;
        self.write_reg(regs::Common::PACONFIG, pa_config)?;

        debug!("Calibration done");

        Ok(())
    }
}

impl<Hal> Sx127x<Hal>
where
    Hal: base::Hal,
{
    pub(crate) fn build(hal: Hal, config: Config) -> Self {
        Sx127x {
            hal,
            config,
            mode: Mode::Unconfigured,
        }
    }
}

impl<Hal> DelayUs for Sx127x<Hal>
where
    Hal: base::Hal,
{
    fn delay_us(&mut self, t: u32) {
        self.hal.delay_us(t);
    }
}

impl<Hal> Sx127x<Hal>
where
    Hal: base::Hal,
{
    /// Read a u8 value from the specified register
    pub fn read_reg<R>(&mut self, reg: R) -> Result<u8, Error<<Hal as base::Hal>::Error>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        let value = self.hal.read_reg(reg.into())?;

        trace!(
            "Read reg:   {:?} (0x{:02x}): 0x{:02x}",
            reg,
            reg.into(),
            value
        );

        Ok(value)
    }

    /// Write a u8 value to the specified register
    pub fn write_reg<R>(
        &mut self,
        reg: R,
        value: u8,
    ) -> Result<(), Error<<Hal as base::Hal>::Error>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        trace!(
            "Write reg:  {:?} (0x{:02x}): 0x{:02x}",
            reg,
            reg.into(),
            value
        );

        self.hal.write_reg(reg.into(), value).map_err(Error::Hal)
    }

    /// Update the specified register with the provided (value & mask)
    pub fn update_reg<R>(
        &mut self,
        reg: R,
        mask: u8,
        value: u8,
    ) -> Result<u8, Error<<Hal as base::Hal>::Error>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        trace!(
            "Update reg: {:?} (0x{:02x}): 0x{:02x} (0x{:02x})",
            reg,
            reg.into(),
            value,
            mask
        );

        self.hal
            .update_reg(reg.into(), mask, value)
            .map_err(Error::Hal)
    }
}

impl<Hal> radio::State for Sx127x<Hal>
where
    Hal: base::Hal,
{
    type State = State;
    type Error = Error<<Hal as base::Hal>::Error>;

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

impl<Hal> radio::Power for Sx127x<Hal>
where
    Hal: base::Hal,
{
    type Error = Error<<Hal as base::Hal>::Error>;

    /// Set transmit power (using the existing `PaConfig`)
    fn set_power(&mut self, power: i8) -> Result<(), Error<<Hal as base::Hal>::Error>> {
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

impl<Hal> radio::Interrupts for Sx127x<Hal>
where
    Hal: base::Hal,
{
    type Irq = Interrupts;
    type Error = Error<<Hal as base::Hal>::Error>;

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

impl<Hal> radio::Channel for Sx127x<Hal>
where
    Hal: base::Hal,
{
    type Channel = Channel;
    type Error = Error<<Hal as base::Hal>::Error>;

    /// Set the LoRa mode channel for future receive or transmit operations
    fn set_channel(
        &mut self,
        channel: &Self::Channel,
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        match (self.mode, channel) {
            (Mode::LoRa, Channel::LoRa(channel)) => self.lora_set_channel(channel),
            (Mode::FskOok, Channel::FskOok(channel)) => self.fsk_set_channel(channel),
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

impl<Hal> radio::Transmit for Sx127x<Hal>
where
    Hal: base::Hal,
{
    type Error = Error<<Hal as base::Hal>::Error>;

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
    fn check_transmit(&mut self) -> Result<bool, Error<<Hal as base::Hal>::Error>> {
        match self.mode {
            Mode::LoRa => self.lora_check_transmit(),
            Mode::FskOok => self.fsk_check_transmit(),
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

impl<Hal> radio::Receive for Sx127x<Hal>
where
    Hal: base::Hal,
{
    type Info = PacketInfo;
    type Error = Error<<Hal as base::Hal>::Error>;

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
    fn get_received(&mut self, buff: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        match self.mode {
            Mode::LoRa => self.lora_get_received(buff),
            Mode::FskOok => self.fsk_get_received(buff),
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

impl<Hal> radio::Rssi for Sx127x<Hal>
where
    Hal: base::Hal,
{
    type Error = Error<<Hal as base::Hal>::Error>;

    /// Poll for the current channel RSSI
    /// This should only be called in receive mode
    fn poll_rssi(&mut self) -> Result<i16, Error<<Hal as base::Hal>::Error>> {
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
