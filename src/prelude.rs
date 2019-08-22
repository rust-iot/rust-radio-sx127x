//! Common requirements for crate consumers

pub use crate::{Error, Sx127x};

pub use crate::device::fsk::{FskChannel, FskConfig};
pub use crate::device::lora::{LoRaChannel, LoRaConfig};
pub use crate::device::{Channel, Config, Modem, PacketInfo};
