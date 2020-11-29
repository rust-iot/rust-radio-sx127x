//! Basic HAL functions for communicating with the radio device
//!
//! This provides decoupling between embedded hal traits and the RF device implementation.
// Copyright 2019 Ryan Kurte

use core::fmt::Debug;

use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::blocking::spi::Transactional;

use driver_pal::{Reset, Busy, PinState, PrefixRead, PrefixWrite};
use driver_pal::Error as WrapError;

use crate::Error;

/// Base implementation can be generic over SPI or UART connections
pub trait Base<CommsError, PinError, DelayError> {
    /// Reset the device
    fn reset(&mut self) -> Result<(), Error<CommsError, PinError, DelayError>>;

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Error<CommsError, PinError, DelayError>>;

    /// Delay for the specified time
    fn try_delay_ms(&mut self, ms: u32) -> Result<(), DelayError>;

    /// Write a slice of data to the specified register
    fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<CommsError, PinError, DelayError>>;
    /// Read a slice of data from the specified register
    fn read_regs(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Error<CommsError, PinError, DelayError>>;

    /// Write to the specified FIFO buffer
    fn write_buff(&mut self, data: &[u8]) -> Result<(), Error<CommsError, PinError, DelayError>>;
    /// Read from the specified FIFO buffer
    fn read_buff(&mut self, data: &mut [u8]) -> Result<(), Error<CommsError, PinError, DelayError>>;

    /// Read a single u8 value from the specified register
    fn read_reg(&mut self, reg: u8) -> Result<u8, Error<CommsError, PinError, DelayError>> {
        let mut incoming = [0u8; 1];
        self.read_regs(reg.into(), &mut incoming)?;
        Ok(incoming[0])
    }

    /// Write a single u8 value to the specified register
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<CommsError, PinError, DelayError>> {
        self.write_regs(reg.into(), &[value])?;
        Ok(())
    }

    /// Update the specified register with the provided value & mask
    fn update_reg(
        &mut self,
        reg: u8,
        mask: u8,
        value: u8,
    ) -> Result<u8, Error<CommsError, PinError, DelayError>> {
        let existing = self.read_reg(reg)?;
        let updated = (existing & !mask) | (value & mask);
        self.write_reg(reg, updated)?;
        Ok(updated)
    }
}

/// Implement HAL for embedded helper trait implementers
impl<T, CommsError, PinError, DelayError> Base<CommsError, PinError, DelayError> for T
where
    T: Transactional<u8, Error = WrapError<CommsError, PinError, DelayError>> + PrefixRead<Error=WrapError<CommsError, PinError, DelayError>> + PrefixWrite<Error=WrapError<CommsError, PinError, DelayError>>,
    T: Reset<Error = WrapError<CommsError, PinError, DelayError>>,
    T: Busy<Error = WrapError<CommsError, PinError, DelayError>>,
    T: DelayMs<u32, Error=DelayError> + DelayUs<u32, Error=DelayError>,
    CommsError: Debug + Sync + Send + 'static,
    PinError: Debug + Sync + Send + 'static,
    DelayError: Debug + Sync + Send + 'static,
{
    /// Reset the radio
    fn reset(&mut self) -> Result<(), Error<CommsError, PinError, DelayError>> {
        self.set_reset(PinState::Low).map_err(|e| Error::from(e))?;
        self.try_delay_ms(1).map_err(WrapError::Delay)?;
        self.set_reset(PinState::High).map_err(|e| Error::from(e))?;
        self.try_delay_ms(10).map_err(WrapError::Delay)?;

        Ok(())
    }

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Error<CommsError, PinError, DelayError>> {
        // TODO: suspiciously unimplemented?!
        Ok(())
    }

    /// Delay for the specified time
    fn try_delay_ms(&mut self, ms: u32) -> Result<(), DelayError> {
        DelayMs::try_delay_ms(self, ms)?;
        Ok(())
    }

    /// Read from the specified register
    fn read_regs<'a>(
        &mut self,
        reg: u8,
        data: &mut [u8],
    ) -> Result<(), Error<CommsError, PinError, DelayError>> {
        // Setup register read
        let out_buf: [u8; 1] = [reg as u8 & 0x7F];
        self.wait_busy()?;
        let r = self
            .try_prefix_read(&out_buf, data)
            .map(|_| ())
            .map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Write to the specified register
    fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<CommsError, PinError, DelayError>> {
        // Setup register write
        let out_buf: [u8; 1] = [reg as u8 | 0x80];
        self.wait_busy()?;
        let r = self.try_prefix_write(&out_buf, data).map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Write to the specified buffer
    fn write_buff(&mut self, data: &[u8]) -> Result<(), Error<CommsError, PinError, DelayError>> {
        // Setup fifo buffer write
        let out_buf: [u8; 1] = [0x00 | 0x80];
        self.wait_busy()?;
        let r = self.try_prefix_write(&out_buf, data).map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Read from the specified buffer
    fn read_buff<'a>(&mut self, data: &mut [u8]) -> Result<(), Error<CommsError, PinError, DelayError>> {
        // Setup fifo buffer read
        let out_buf: [u8; 1] = [0x00];
        self.wait_busy()?;
        let r = self
            .try_prefix_read(&out_buf, data)
            .map(|_| ())
            .map_err(|e| e.into());
        self.wait_busy()?;
        r
    }
}
