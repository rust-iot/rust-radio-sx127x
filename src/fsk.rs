//! TODO: Sx127x FSK mode RF implementation
//! 
//! This module implements FSK and OOK radio functionality for the Sx127x series devices
//! 
//! Copyright 2019 Ryan Kurte

use crate::{Sx127x, Error};
use crate::base::Base as Sx127xBase;
use crate::device::{self, State, ModemMode, regs};



impl<Base, CommsError, PinError, Config> Sx127x<Base, CommsError, PinError, Config>
where
    Base: Sx127xBase<CommsError, PinError>,
{

}