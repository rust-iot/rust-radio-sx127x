//! TODO: Sx127x FSK/OOK mode RF implementation
//!
//! This module implements FSK and OOK radio functionality for the Sx127x series devices
//!
//! Copyright 2019 Ryan Kurte


use radio::State as _;

use log::{trace, debug};

use crate::{Error, Mode, Sx127x};

use crate::base;
use crate::device::fsk::*;
use crate::device::fsk::{Irq1, Irq2};
use crate::device::{self, regs, Channel, Modem, ModemMode, PacketInfo, State};

/// Marker struct for FSK/OOK operating mode
pub struct FskOokMode();

impl<Hal> Sx127x<Hal>
where
    Hal: base::Hal,
{
    pub(crate) fn fsk_configure(
        &mut self,
        config: &FskConfig,
        channel: &FskChannel,
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        debug!("Configuring FSK/OOK mode: {:?} {:?}", config, channel);

        // Switch to sleep to change modem mode
        self.set_state(State::Sleep)?;

        // Switch to standard mode
        self.set_modem(ModemMode::Standard)?;

        // Set channel configuration
        self.fsk_set_channel(channel)?;

        // Revert to standby mode
        self.set_state(State::Standby)?;

        // Set preamble length
        self.write_reg(regs::Fsk::PREAMBLEMSB, (config.preamble_len >> 8) as u8)?;
        self.write_reg(regs::Fsk::PREAMBLELSB, (config.preamble_len & 0xFF) as u8)?;

        // Set payload length
        let (packet_format, packet_len_msb) = match config.payload_len {
            PayloadLength::Constant(v) => {
                self.write_reg(regs::Fsk::PAYLOADLENGTH, v as u8)?;
                (PACKETFORMAT_FIXED, (v >> 8) & 0x07)
            }
            PayloadLength::Variable => {
                self.write_reg(regs::Fsk::PAYLOADLENGTH, 0xFF)?;
                (PACKETFORMAT_VARIABLE, 0)
            }
        };

        // Set packetconfig1 register
        self.write_reg(
            regs::Fsk::PACKETCONFIG1,
            packet_format
                | config.dc_free as u8
                | config.crc as u8
                | config.crc_autoclear as u8
                | config.address_filter as u8
                | config.crc_whitening as u8,
        )?;

        // Set packetconfig2 register
        self.write_reg(
            regs::Fsk::PACKETCONFIG2,
            config.data_mode as u8
                | config.io_home as u8
                | config.beacon as u8
                | packet_len_msb as u8,
        )?;

        // Set preamble
        self.write_reg(regs::Fsk::PREAMBLEMSB, (config.preamble >> 8) as u8)?;
        self.write_reg(regs::Fsk::PREAMBLELSB, config.preamble as u8)?;

        // Configure preamble detector
        self.write_reg(regs::Fsk::PREAMBLEDETECT, 
            PreambleDetect::On as u8 |
            PreambleDetectSize::Ps2 as u8 | 
            PREAMBLE_DETECTOR_TOL
        )?;

        // Configure TXStart
        self.write_reg(
            regs::Fsk::FIFOTHRESH,
            TX_START_FIFOLEVEL | (TX_FIFOTHRESH_MASK & 0x02),
        )?;

        self.mode = Mode::FskOok;
        self.config.modem = Modem::FskOok(config.clone());
        self.config.channel = Channel::FskOok(channel.clone());

        Ok(())
    }

    pub(crate) fn fsk_get_config(&self) -> &FskConfig {
        match &self.config.modem {
            Modem::FskOok(c) => c,
            _ => panic!("Attempted to fetch config from invalid mode"),
        }
    }

    /// Fetch pending FskOok mode interrupts from the device
    /// If the clear option is set, this will also clear any pending flags
    pub(crate) fn fsk_get_interrupts(
        &mut self,
        clear: bool,
    ) -> Result<(Irq1, Irq2), Error<<Hal as base::Hal>::Error>> {
        let reg = self.read_reg(regs::Fsk::IRQFLAGS1)?;
        let irq1 = Irq1::from_bits(reg).unwrap();

        if clear {
            self.write_reg(regs::Fsk::IRQFLAGS1, 
                (irq1 & (Irq1::RSSI | Irq1::PREAMBLED_DETECT)).bits())?;
        }

        let reg = self.read_reg(regs::Fsk::IRQFLAGS2)?;
        let irq2 = Irq2::from_bits(reg).unwrap();

        if clear {
            self.write_reg(regs::Fsk::IRQFLAGS2, 
                (irq2 & (Irq2::LOW_BAT)).bits())?;
        }

        Ok((irq1, irq2))
    }

    /// Set the Fsk mode channel for future receive or transmit operations
    pub(crate) fn fsk_set_channel(
        &mut self,
        channel: &FskChannel,
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        // Set frequency
        self.set_frequency(channel.freq)?;

        // Need our own rounding alogrithm because corelib doesn't have
        // f32::round
        let round = |x: f32| -> u32 {
            let integer = x as u32;
                if (x - (integer as f32)) < 0.5 {
                   integer
                }
                else {
                   integer + 1
                }
        };

        // Calculate channel configuration
        let fdev = round((channel.fdev as f32) / device::FREQ_STEP);
        let datarate = round(self.config.xtal_freq as f32 / channel.br as f32);
        trace!("fdev: {} bitrate: {}", fdev, datarate);

        // Set frequency deviation
        self.write_reg(regs::Fsk::FDEVMSB, (fdev >> 8) as u8)?;
        self.write_reg(regs::Fsk::FDEVLSB, (fdev & 0xFF) as u8)?;

        // Set bitrate
        self.write_reg(regs::Fsk::BITRATEMSB, (datarate >> 8) as u8)?;
        self.write_reg(regs::Fsk::BITRATELSB, (datarate & 0xFF) as u8)?;

        // Set bandwidths
        self.write_reg(regs::Fsk::RXBW, channel.bw as u8)?;
        self.write_reg(regs::Fsk::AFCBW, channel.bw_afc as u8)?;

        Ok(())
    }

    /// Start sending a packet
    pub(crate) fn fsk_start_transmit(
        &mut self,
        data: &[u8],
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        debug!("Starting send (data: {:?})", data);

        // TODO: support large packet sending
        assert!(data.len() < 255);

        self.set_state_checked(State::Standby)?;

        let (i1, i2) = self.fsk_get_interrupts(true)?;
        trace!("clearing interrupts (irq1: {:?} irq2: {:?})", i1, i2);

        // Write length as first element in buffer
        self.hal.write_buff(&[data.len() as u8])?;

        // Write the rest of the data
        self.hal.write_buff(data)?;

        // Start TX
        self.set_state_checked(State::Tx)?;

        Ok(())
    }

    /// Check for packet send completion
    pub(crate) fn fsk_check_transmit(&mut self) -> Result<bool, Error<<Hal as base::Hal>::Error>> {
        // Fetch interrupts
        let (i1, i2) = self.fsk_get_interrupts(true)?;

        trace!("Check transmit IRQ1: {:?} IRQ2: {:?}", i1, i2);

        // Check for completion
        if i2.contains(Irq2::PACKET_SENT) {
            return Ok(true);
        }

        Ok(false)
    }

    pub(crate) fn fsk_start_receive(&mut self) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        debug!("Starting receive");

        // Revert to standby state
        self.set_state_checked(State::Standby)?;

        // Set RX configuration
        let rx_config = self.fsk_get_config().rx_agc as u8
            | self.fsk_get_config().rx_afc as u8
            | self.fsk_get_config().rx_trigger as u8;
        self.write_reg(regs::Fsk::RXCONFIG, rx_config)?;

        // Clear interrupts
        let (i1, i2) = self.fsk_get_interrupts(true)?;
        debug!("clearing interrupts (irq1: {:?} irq2: {:?})", i1, i2);

        // Enter Rx mode (unchecked as we enter FsRx by default)
        // And RX on PreambleDetect (also by default)
        self.set_state(State::Rx)?;

        debug!("started receive");

        Ok(())
    }

    /// Check receive state
    ///
    /// This returns true if a boolean indicating whether a packet has been received.
    /// The restart option specifies whether transient timeout or CRC errors should be
    /// internally handled (returning Ok(false)) or passed back to the caller as errors.
    pub(crate) fn fsk_check_receive(
        &mut self,
        restart: bool,
    ) -> Result<bool, Error<<Hal as base::Hal>::Error>> {
        // Fetch interrupts
        let (i1, i2) = self.fsk_get_interrupts(true)?;
        let s = self.get_state()?;
        let mut res = Ok(false);

        trace!(
            "check receive (state: {:?}, irq1: {:?} irq2: {:?})",
            s, i1, i2
        );

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
                self.fsk_start_receive()?;
                Ok(false)
            }
            (_, r) => r,
        }
    }

    /// Fetch a received message
    ///
    /// This copies data into the provided slice, updates the provided information object,
    ///  and returns the number of bytes received on success
    pub(crate) fn fsk_get_received(
        &mut self,
        data: &mut [u8],
    ) -> Result<(usize, PacketInfo), Error<<Hal as base::Hal>::Error>> {

        let mut len = [0u8; 1];
        // Read the length byte from the FIFO
        self.hal.read_buff(&mut len)?;

        let len = len[0] as usize;
        if len > data.len() {
            return Err(Error::BufferSize);
        }

        // Read the rest of the fifo data
        self.hal.read_buff(&mut data[..len])?;

        // Read the RSSI
        let info = PacketInfo{
            rssi: self.fsk_poll_rssi()?,
            snr: None,
        };

        debug!("Received data: {:?} info: {:?}", &data[0..len], &info);

        Ok((len, info))
    }

    /// Poll for the current channel RSSI
    /// This should only be called in receive mode
    pub(crate) fn fsk_poll_rssi(&mut self) -> Result<i16, Error<<Hal as base::Hal>::Error>> {
        let raw_rssi = self.read_reg(regs::Fsk::RSSIVALUE)?;

        let rssi = -((raw_rssi / 2) as i16);

        Ok(rssi)
    }
}
