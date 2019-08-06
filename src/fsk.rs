//! TODO: Sx127x FSK/OOK mode RF implementation
//! 
//! This module implements FSK and OOK radio functionality for the Sx127x series devices
//! 
//! Copyright 2019 Ryan Kurte

use crate::{Sx127x, Error};
use crate::base::Base as Sx127xBase;
use crate::device::{self, State, ModemMode, regs};
use crate::device::fsk::*;


/// Marker struct for FSK/OOK operating mode
pub struct FskOokMode ();

impl<Base, CommsError, PinError, Mode> Sx127x<Base, CommsError, PinError, Mode>
where
    Base: Sx127xBase<CommsError, PinError>,
{

}

impl<Base, CommsError, PinError> Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    pub fn configure_fsk(&mut self, _config: &FskConfig, _channel: &FskChannel) -> Result<(), Error<CommsError, PinError>> {
        debug!("Configuring FSK/OOK mode");

        unimplemented!()
    }

}

impl<Base, CommsError, PinError> radio::Channel for Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Channel = FskChannel;
    type Error = Error<CommsError, PinError>;

    /// Set the Fsk mode channel for future receive or transmit operations
    fn set_channel(&mut self, _channel: &FskChannel) -> Result<(), Error<CommsError, PinError>> {
        unimplemented!()
    }

}

impl<Base, CommsError, PinError> radio::Transmit for Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Error = Error<CommsError, PinError>;

    /// Start sending a packet
    fn start_transmit(&mut self, _data: &[u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn check_transmit(&mut self) -> Result<bool, Error<CommsError, PinError>> {
        unimplemented!()
    }
}

impl<Base, CommsError, PinError> radio::Receive for Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Info = FskInfo;
    type Error = Error<CommsError, PinError>;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        unimplemented!()
    }

    /// Check receive state
    /// 
    /// This returns true if a boolean indicating whether a packet has been received.
    /// The restart option specifies whether transient timeout or CRC errors should be 
    /// internally handled (returning Ok(false)) or passed back to the caller as errors.
    fn check_receive(&mut self, _restart: bool) -> Result<bool, Self::Error> {
        unimplemented!()
    }

    /// Fetch a received message
    /// 
    /// This copies data into the provided slice, updates the provided information object,
    ///  and returns the number of bytes received on success
    fn get_received(&mut self, _info: &mut Self::Info, _data: &mut[u8]) -> Result<usize, Self::Error> {
        unimplemented!()
    }


}