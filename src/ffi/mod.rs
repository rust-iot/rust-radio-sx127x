//! Compat module implements FFI bindings to an underlying C driver instance
//! 
//! This is provided to enable full API access and piece-wise migration to a pure-rust driver
//! 
//! Copyright 2019 Ryan Kurte

use embedded_spi::{Transactional, PinState, Error as WrapError};
use embedded_spi::ffi::{Cursed, Conv};

use hal::blocking::{delay};
use hal::digital::v2::{InputPin, OutputPin};

use crate::{Sx127x, Error};
use crate::bindings::{self as sx127x, SX1276_s};

// Mark Sx127x object as cursed to forever wander the lands of ffi
impl <Conn, CommsError, Output, Input, PinError, Delay> Cursed for Sx127x <Conn, CommsError, Output, Input, PinError, Delay> {}


impl<T, CommsError, Output, Input, PinError, Delay> Sx127x<T, CommsError, Output, Input, PinError, Delay>
where
    T: Transactional<Error = WrapError<CommsError, PinError>>,

    Output: OutputPin<Error = PinError>,
    Input: InputPin<Error = PinError>,

    Delay: delay::DelayMs<u32>,
    CommsError: core::fmt::Debug,
    PinError: core::fmt::Debug,
{
    /// Create and bind an internal C object to support the bound C api
    pub fn bind(s: &mut Self) {
        let ctx = Self::to_c_ptr(s);

        // Create base C object
        let c = SX1276_s {
            settings: unsafe { core::mem::zeroed() },

            ctx,

            set_reset: Some(Self::set_reset),
            //get_busy: Some(Self::get_busy),
            
            spi_write: Some(Self::spi_write),
            spi_read: Some(Self::spi_read),

            //get_dio: [None; 4],

            delay_ms: Some(Self::delay_ms),
        };

        // Store C object in structure
        s.c = Some(c);

    }

    #[allow(dead_code)]
    extern fn set_reset(ctx: *mut libc::c_void, value: bool) -> i32 {
        let sx127x = Self::from_c_ptr(ctx);
        let r = match value {
            true => sx127x.sdn.set_high(),
            false => sx127x.sdn.set_low(),
        };
        match r {
            Ok(_) => 0,
            Err(e) => {
                sx127x.err = Some(Error::Pin(e));
                -1
            }
        }
    }

    #[allow(dead_code)]
    extern fn get_busy(ctx: *mut libc::c_void) -> i32 {
        let sx127x = Self::from_c_ptr(ctx);
        let r = sx127x.hal.spi_busy();
        match r {
            Ok(PinState::High) => 1,
            Ok(PinState::Low) => 0,
            Err(e) => {
                sx127x.err = Some(e.into());
                -1
            }
        }
    }

    #[allow(dead_code)]
    extern fn spi_write(ctx: *mut libc::c_void, prefix: *mut u8, prefix_len: u16, data: *mut u8, data_len: u16) -> i32 {
        // Coerce back into rust
        let s = Self::from_c_ptr(ctx);

        // Parse buffers
        let prefix: &[u8] = unsafe { core::slice::from_raw_parts(prefix, prefix_len as usize) };
        let data: &[u8] = unsafe { core::slice::from_raw_parts(data, data_len as usize) };

        // Execute command and handle errors
        match s.hal.spi_write(&prefix, &data) {
            Ok(_) => 0,
            Err(e) => {
                s.err = Some(e.into());
                -1
            },
        }
    }
    
    #[allow(dead_code)]
    extern fn spi_read(ctx: *mut libc::c_void, prefix: *mut u8, prefix_len: u16, data: *mut u8, data_len: u16) -> i32 {
         // Coerce back into rust
        let s = Self::from_c_ptr(ctx);

        // Parse buffers
        let prefix: &[u8] = unsafe { core::slice::from_raw_parts(prefix, prefix_len as usize) };
        let mut data: &mut [u8] = unsafe { core::slice::from_raw_parts_mut(data, data_len as usize) };

        // Execute command and handle errors
        match s.hal.spi_read(&prefix, &mut data) {
            Ok(_) => 0,
            Err(e) => {
                s.err = Some(e.into());
                -1
            },
        }
    }

    #[allow(dead_code)]
    extern fn delay_ms(ctx: *mut libc::c_void, ms: u32) {
        let sx127x = Self::from_c_ptr(ctx);;
        let _ = sx127x.delay.delay_ms(ms as u32);
    }

}

impl<T, CommsError, Output, Input, PinError, Delay> Sx127x<T, CommsError, Output, Input, PinError, Delay>
where
    T: Transactional<Error = WrapError<CommsError, PinError>>,

    Output: OutputPin<Error = PinError>,
    Input: InputPin<Error = PinError>,
    Delay: delay::DelayMs<u32>,
{

    /// Read status register using FFI bound method
    pub fn ffi_status(&mut self) -> Result<sx127x::RadioState_t, Error<CommsError, PinError>> {
        let mut ctx = self.c.unwrap();
        let status = unsafe { sx127x::SX1276GetStatus(&mut ctx) };
        Ok(status)
    }

    pub fn ffi_rx_chain_calibration(&mut self) -> Result<(), Error<CommsError, PinError>> {
        let mut ctx = self.c.unwrap();

        //unsafe { sx127x::RxChainCalibration(&mut ctx) };

        Ok(())
    }

}


#[cfg(test)]
mod tests {
    use crate::{Sx127x, Settings};

    use embedded_spi::ffi::{Conv};
    use embedded_spi::mock::{Mock, MockTransaction as Mt, Spi, Pin};

    extern crate color_backtrace;

    type Radio = Sx127x<embedded_spi::mock::Spi, (), embedded_spi::mock::Pin, embedded_spi::mock::Pin, (), embedded_spi::mock::Delay>;

    #[test]
    fn test_ffi_passthrough() {
        color_backtrace::install();

        let mut m = Mock::new();
        let (spi, sdn, _busy, delay) = (m.spi(), m.pin(), m.pin(), m.delay());

        let mut radio = Sx127x::<Spi, _, Pin, Pin, _, _>::build(spi.clone(), sdn.clone(), delay.clone(), Settings::default());

        Radio::bind(&mut radio);
        let ptr = Radio::to_c_ptr(&mut radio);

        assert!(ptr != (0 as *mut libc::c_void), "to_c is not void");
        assert_eq!(ptr, radio.c.unwrap().ctx);

        m.expect(&[
            Mt::set_high(&sdn),
        ]);

        assert_eq!(0, Radio::set_reset(ptr, true));
        
        m.finalise();
    }

}