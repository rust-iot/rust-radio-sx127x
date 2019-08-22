//! Basic HAL functions for communicating with the radio device
//!
//! This provides decoupling between embedded hal traits and the RF device implementation.
// Copyright 2019 Ryan Kurte

use hal::blocking::delay::DelayMs;

use embedded_spi::Error as WrapError;
use embedded_spi::{Busy, PinState, Reset, Transactional};

use crate::Error;

/// Base implementation can be generic over SPI or UART connections
pub trait Base<CommsError, PinError> {
    /// Reset the device
    fn reset(&mut self) -> Result<(), Error<CommsError, PinError>>;

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Error<CommsError, PinError>>;

    /// Delay for the specified time
    fn delay_ms(&mut self, ms: u32);

    /// Write a slice of data to the specified register
    fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<CommsError, PinError>>;
    /// Read a slice of data from the specified register
    fn read_regs(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Error<CommsError, PinError>>;

    /// Write to the specified FIFO buffer
    fn write_buff(&mut self, data: &[u8]) -> Result<(), Error<CommsError, PinError>>;
    /// Read from the specified FIFO buffer
    fn read_buff(&mut self, data: &mut [u8]) -> Result<(), Error<CommsError, PinError>>;

    /// Read a single u8 value from the specified register
    fn read_reg(&mut self, reg: u8) -> Result<u8, Error<CommsError, PinError>> {
        let mut incoming = [0u8; 1];
        self.read_regs(reg.into(), &mut incoming)?;
        Ok(incoming[0])
    }

    /// Write a single u8 value to the specified register
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<CommsError, PinError>> {
        self.write_regs(reg.into(), &[value])?;
        Ok(())
    }

    /// Update the specified register with the provided value & mask
    fn update_reg(
        &mut self,
        reg: u8,
        mask: u8,
        value: u8,
    ) -> Result<u8, Error<CommsError, PinError>> {
        let existing = self.read_reg(reg)?;
        let updated = (existing & !mask) | (value & mask);
        self.write_reg(reg, updated)?;
        Ok(updated)
    }
}

/// Implement HAL for embedded helper trait implementers
impl<T, CommsError, PinError> Base<CommsError, PinError> for T
where
    T: Transactional<Error = WrapError<CommsError, PinError>>,
    T: Reset<Error = WrapError<CommsError, PinError>>,
    T: Busy<Error = WrapError<CommsError, PinError>>,
    T: DelayMs<u32>,
{
    /// Reset the radio
    fn reset(&mut self) -> Result<(), Error<CommsError, PinError>> {
        self.set_reset(PinState::Low).map_err(|e| Error::from(e))?;
        self.delay_ms(1);
        self.set_reset(PinState::High).map_err(|e| Error::from(e))?;
        self.delay_ms(10);

        Ok(())
    }

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Error<CommsError, PinError>> {
        Ok(())
    }

    /// Delay for the specified time
    fn delay_ms(&mut self, ms: u32) {
        self.delay_ms(ms);
    }

    /// Read from the specified register
    fn read_regs<'a>(
        &mut self,
        reg: u8,
        data: &mut [u8],
    ) -> Result<(), Error<CommsError, PinError>> {
        // Setup register read
        let out_buf: [u8; 1] = [reg as u8 & 0x7F];
        self.wait_busy()?;
        let r = self
            .spi_read(&out_buf, data)
            .map(|_| ())
            .map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Write to the specified register
    fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<CommsError, PinError>> {
        // Setup register write
        let out_buf: [u8; 1] = [reg as u8 | 0x80];
        self.wait_busy()?;
        let r = self.spi_write(&out_buf, data).map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Write to the specified buffer
    fn write_buff(&mut self, data: &[u8]) -> Result<(), Error<CommsError, PinError>> {
        // Setup fifo buffer write
        let out_buf: [u8; 1] = [0x00 | 0x80];
        self.wait_busy()?;
        let r = self.spi_write(&out_buf, data).map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Read from the specified buffer
    fn read_buff<'a>(&mut self, data: &mut [u8]) -> Result<(), Error<CommsError, PinError>> {
        // Setup fifo buffer read
        let out_buf: [u8; 1] = [0x00];
        self.wait_busy()?;
        let r = self
            .spi_read(&out_buf, data)
            .map(|_| ())
            .map_err(|e| e.into());
        self.wait_busy()?;
        r
    }
}
