//! TODO: Sx127x FSK/OOK mode RF implementation
//! 
//! This module implements FSK and OOK radio functionality for the Sx127x series devices
//! 
//! Copyright 2019 Ryan Kurte

use radio::{State as _, Channel as _, Interrupts as _};

use crate::{Sx127x, Error};
use crate::base::Base as Sx127xBase;
use crate::device::{self, State, ModemMode, PacketInfo, regs};
use crate::device::fsk::*;
use crate::device::fsk::{Irq1, Irq2};


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
    pub fn configure_fsk(&mut self, config: &FskConfig, channel: &FskChannel) -> Result<(), Error<CommsError, PinError>> {
        debug!("Configuring FSK/OOK mode");

        // Switch to sleep to change modem mode
        self.set_state(State::Sleep)?;

        // Switch to standard mode
        self.set_modem(ModemMode::Standard)?;

        // Set channel configuration
        self.set_channel(channel)?;

        // Set preamble length
        self.write_reg(regs::Fsk::PREAMBLEMSB, (config.preamble_len >> 8) as u8)?;
        self.write_reg(regs::Fsk::PREAMBLELSB, (config.preamble_len & 0xFF) as u8)?;

        // Set payload length
        let (packet_format, packet_len_msb) = match config.payload_len {
            PayloadLength::Constant(v) => {
                self.write_reg(regs::Fsk::PAYLOADLENGTH, v as u8)?;
                (PACKETFORMAT_VARIABLE, (v >> 8) & 0x07)
            },
            PayloadLength::Variable => {
                self.write_reg(regs::Fsk::PAYLOADLENGTH, 0xFF)?;
                (PACKETFORMAT_FIXED, 0)
            }
        };

        // Set packetconfig1 register
        self.write_reg(regs::Fsk::PACKETCONFIG1, 
            packet_format | config.dc_free as u8 | config.crc as u8 | config.crc_autoclear as u8 |
            config.address_filter as u8 | config.crc_whitening as u8 
        )?;

        // Set packetconfig2 register
        self.write_reg(regs::Fsk::PACKETCONFIG2,
            config.data_mode as u8 | config.io_home as u8 | config.beacon as u8 | 
            packet_len_msb as u8
        )?;

        // Set preamble
        self.write_reg(regs::Fsk::PREAMBLEMSB, (config.preamble >> 8) as u8)?;
        self.write_reg(regs::Fsk::PREAMBLELSB, config.preamble as u8)?;

        Ok(())
    }

}


impl<Base, CommsError, PinError> radio::Interrupts for Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Irq = (Irq1, Irq2);
    type Error = Error<CommsError, PinError>;

    /// Fetch pending LoRa mode interrupts from the device
    /// If the clear option is set, this will also clear any pending flags
    fn get_interrupts(&mut self, clear: bool) -> Result<Self::Irq, Self::Error> {
        let reg = self.read_reg(regs::Fsk::IRQFLAGS1)?;
        let irq1 = Irq1::from_bits(reg).unwrap();

        if clear {
            self.write_reg(regs::Fsk::IRQFLAGS1, reg)?;
        }

        let reg = self.read_reg(regs::Fsk::IRQFLAGS2)?;
        let irq2 = Irq2::from_bits(reg).unwrap();

        if clear {
            self.write_reg(regs::Fsk::IRQFLAGS2, reg)?;
        }

        Ok((irq1, irq2))
    }
}

impl<Base, CommsError, PinError> radio::Channel for Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Channel = FskChannel;
    type Error = Error<CommsError, PinError>;

    /// Set the Fsk mode channel for future receive or transmit operations
    fn set_channel(&mut self, channel: &FskChannel) -> Result<(), Error<CommsError, PinError>> {
        // Set frequency
        self.set_frequency(channel.freq)?;

        // Set frequency deviation
        let fdev = ((channel.fdev as f32) / device::FREQ_STEP) as u32;
        self.write_reg(regs::Fsk::FDEVMSB, (fdev >> 8) as u8)?;
        self.write_reg(regs::Fsk::FDEVLSB, (fdev & 0xFF) as u8)?;

        // Set bitrate
        let datarate = self.config.xtal_freq / channel.br;
        self.write_reg(regs::Fsk::BITRATEMSB, (datarate >> 8) as u8)?;
        self.write_reg(regs::Fsk::BITRATELSB, (datarate & 0xFF) as u8)?;

        // Set bandwidths
        self.write_reg(regs::Fsk::RXBW, channel.bw as u8)?;
        self.write_reg(regs::Fsk::AFCBW, channel.bw_afc as u8)?;

        Ok(())
    }

}

impl<Base, CommsError, PinError> radio::Transmit for Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Error = Error<CommsError, PinError>;

    /// Start sending a packet
    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        debug!("Starting send (data: {:?})", data);

        // TODO: support large packet sending
        assert!(data.len() < 255);

        self.set_state_checked(State::Standby)?;

        // Write length as first element in buffer
        self.hal.write_buff(&[data.len() as u8])?;

        // Write the rest of the data
        self.hal.write_buff(data)?;

        // Start TX
        self.set_state_checked(State::Tx)?;

        Ok(())
    }

    /// Check for packet send completion
    fn check_transmit(&mut self) -> Result<bool, Error<CommsError, PinError>> {
        // Fetch interrupts
        let (_i1, i2) = self.get_interrupts(true)?;

        // Check for completion
        if i2.contains(Irq2::PACKET_SENT) {
            return Ok(true)
        }

        Ok(false)
    }
}

impl<Base, CommsError, PinError> radio::Receive for Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Info = PacketInfo;
    type Error = Error<CommsError, PinError>;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        debug!("Starting receive");

        // Revert to standby state
        self.set_state_checked(State::Standby)?;

        // Set RX configuration
        self.write_reg(regs::Fsk::RXCONFIG, 
            RXCONFIG_AFCAUTO_ON | RXCONFIG_AGCAUTO_ON | RXCONFIG_RXTRIGER_PREAMBLEDETECT
        )?;

        // Enter Rx mode
        self.set_state_checked(State::RxOnce)?;

        Ok(())
    }

    /// Check receive state
    /// 
    /// This returns true if a boolean indicating whether a packet has been received.
    /// The restart option specifies whether transient timeout or CRC errors should be 
    /// internally handled (returning Ok(false)) or passed back to the caller as errors.
    fn check_receive(&mut self, restart: bool) -> Result<bool, Self::Error> {
        // Fetch interrupts
        let (i1, i2) = self.get_interrupts(true)?;
        let mut res = Ok(false);

        // Check for completion
        if i2.contains(Irq2::PAYLOAD_READY) {
            debug!("RX complete!");
            res = Ok(true)
        } else if i1.contains(Irq1::TIMEOUT) {
            debug!("RX timeout");
            res = Err(Error::Timeout);
        }

        // Handle restart logic
        match (restart, res) {
            (true, Err(_)) => {
                debug!("RX restarting");
                self.start_receive()?;
                Ok(false)
            },
            (_, r) => r
        }
    }

    /// Fetch a received message
    /// 
    /// This copies data into the provided slice, updates the provided information object,
    ///  and returns the number of bytes received on success
    fn get_received(&mut self, _info: &mut Self::Info, data: &mut[u8]) -> Result<usize, Self::Error> {
        let mut len = [0u8; 1];
        // Read the length byte from the FIFO
        self.hal.read_buff(&mut len)?;

        let len = len[0] as usize;
        if len > data.len() {
            return Err(Error::BufferSize)
        }

        // Read the rest of the fifo data
        self.hal.read_buff(&mut data[..len])?;

        Ok(len)
    }
}


impl<Base, CommsError, PinError> radio::Rssi for Sx127x<Base, CommsError, PinError, FskOokMode>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Error = Error<CommsError, PinError>;

    /// Poll for the current channel RSSI
    /// This should only be called in receive mode
    fn poll_rssi(&mut self) -> Result<i16, Error<CommsError, PinError>> {
        let raw = self.read_reg(regs::LoRa::RSSIVALUE)? as i8;

        let rssi = (raw / 2) as i16;

        Ok(rssi)
    }
}
