//! Basic HAL functions for communicating with the radio device

use hal::blocking::{delay};
use hal::digital::v2::{InputPin, OutputPin};

use embedded_spi::{Transactional};
use embedded_spi::{Error as WrapError};

use crate::{Sx127x, Sx127xError};

/// Comms implementation can be generic over SPI or UART connections
pub trait Hal<CommsError, PinError> {
    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Sx127xError<CommsError, PinError>>;

    /// Write to the specified register
    fn reg_write(&mut self, reg: u16, data: &[u8]) -> Result<(), Sx127xError<CommsError, PinError>>;
    /// Read from the specified register
    fn reg_read(&mut self, reg: u16, data: &mut [u8]) -> Result<(), Sx127xError<CommsError, PinError>>;

    /// Write to the specified FIFO buffer
    fn buff_write(&mut self, data: &[u8]) -> Result<(), Sx127xError<CommsError, PinError>>;
    /// Read from the specified FIFO buffer
    fn buff_read(&mut self, data: &mut [u8]) -> Result<(), Sx127xError<CommsError, PinError>>;
}

impl<Comms, CommsError, Output, Input, PinError, Delay> Sx127x<Comms, CommsError, Output, Input, PinError, Delay>
where
    Output: OutputPin<Error = PinError>,
    Input: InputPin<Error = PinError>,
    Delay: delay::DelayMs<u32>,
{ 
    /// Reset the radio
    pub fn reset(&mut self) -> Result<(), Sx127xError<CommsError, PinError>> {
        self.sdn.set_low().map_err(|e| Sx127xError::Pin(e) )?;
        self.delay.delay_ms(1);
        self.sdn.set_high().map_err(|e| Sx127xError::Pin(e) )?;
        self.delay.delay_ms(10);

        Ok(())
    }
}

impl<T, CommsError, PinError> Hal<CommsError, PinError> for T
where
    T: Transactional<Error=WrapError<CommsError, PinError>>,
{    
    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Sx127xError<CommsError, PinError>> {

        Ok(())
    }

    /// Read from the specified register
    fn reg_read<'a>(&mut self, reg: u16, data: &mut [u8]) -> Result<(), Sx127xError<CommsError, PinError>> {
        // Setup register read
        let out_buf: [u8; 1] = [reg as u8 & 0x7F];
        self.wait_busy()?;
        let r = self.spi_read(&out_buf, data).map(|_| () ).map_err(|e| e.into() );
        self.wait_busy()?;
        r
    }

    /// Write to the specified register
    fn reg_write(&mut self, reg: u16, data: &[u8]) -> Result<(), Sx127xError<CommsError, PinError>> {
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
