//! Basic HAL functions for communicating with the radio device

use hal::blocking::delay::DelayMs;
use hal::digital::v2::{InputPin, OutputPin};

use embedded_spi::{Transactional, Reset, Busy, PinState};
use embedded_spi::{Error as WrapError};

use crate::{Sx127x, Sx127xError};

/// Hal implementation can be generic over SPI or UART connections
pub trait Hal<CommsError, PinError> {
    /// Reset the device
    fn reset(&mut self) -> Result<(), Sx127xError<CommsError, PinError>>;

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Sx127xError<CommsError, PinError>>;

    /// Delay for the specified time
    fn delay_ms(&mut self, ms: u32);

    /// Write to the specified register
    fn reg_write(&mut self, reg: u8, data: &[u8]) -> Result<(), Sx127xError<CommsError, PinError>>;
    /// Read from the specified register
    fn reg_read(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Sx127xError<CommsError, PinError>>;

    /// Write to the specified FIFO buffer
    fn buff_write(&mut self, data: &[u8]) -> Result<(), Sx127xError<CommsError, PinError>>;
    /// Read from the specified FIFO buffer
    fn buff_read(&mut self, data: &mut [u8]) -> Result<(), Sx127xError<CommsError, PinError>>;
}

/// Implement HAL for embedded helper trait implementers
impl<T, CommsError, PinError> Hal<CommsError, PinError> for T
where
    T: Transactional<Error=WrapError<CommsError, PinError>>,
    T: Reset<Error=WrapError<CommsError, PinError>>,
    T: Busy<Error=WrapError<CommsError, PinError>>,
    T: DelayMs<u32>,
{    
    /// Reset the radio
    fn reset(&mut self) -> Result<(), Sx127xError<CommsError, PinError>> {
        self.set_reset(PinState::Low).map_err(|e| Sx127xError::from(e) )?;
        self.delay_ms(1);
        self.set_reset(PinState::High).map_err(|e| Sx127xError::from(e) )?;
        self.delay_ms(10);

        Ok(())
    }

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Sx127xError<CommsError, PinError>> {
        Ok(())
    }

    /// Delay for the specified time
    fn delay_ms(&mut self, ms: u32) {
        self.delay_ms(ms);
    }

    /// Read from the specified register
    fn reg_read<'a>(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Sx127xError<CommsError, PinError>> {
        // Setup register read
        let out_buf: [u8; 1] = [reg as u8 & 0x7F];
        self.wait_busy()?;
        let r = self.spi_read(&out_buf, data).map(|_| () ).map_err(|e| e.into() );
        self.wait_busy()?;
        r
    }

    /// Write to the specified register
    fn reg_write(&mut self, reg: u8, data: &[u8]) -> Result<(), Sx127xError<CommsError, PinError>> {
        // Setup register write
        let out_buf: [u8; 1] = [reg as u8 | 0x80];
        self.wait_busy()?;
        let r = self.spi_write(&out_buf, data).map_err(|e| e.into() );
        self.wait_busy()?;
        r
    }

        /// Write to the specified buffer
    fn buff_write(&mut self, data: &[u8]) -> Result<(), Sx127xError<CommsError, PinError>> {
        // Setup fifo buffer write
        let out_buf: [u8; 1] = [ 0x00 ];
        self.wait_busy()?;
        let r = self.spi_write(&out_buf, data).map_err(|e| e.into() );
        self.wait_busy()?;
        r
    }

    /// Read from the specified buffer
    fn buff_read<'a>(&mut self, data: &mut [u8]) -> Result<(), Sx127xError<CommsError, PinError>> {
        // Setup fifo buffer read
        let out_buf: [u8; 1] = [ 0x00 ];
        self.wait_busy()?;
        let r = self.spi_read(&out_buf, data).map(|_| () ).map_err(|e| e.into() );
        self.wait_busy()?;
        r
    }
}
