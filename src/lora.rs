//! Sx127x LoRa mode RF implementation
//!
//! This module implements LoRa radio functionality for the Sx127x series devices
//!
//! Copyright 2019 Ryan Kurte

use log::{trace, debug, warn, error};

use radio::State as _;

use crate::base;
use crate::device::lora::*;
use crate::device::{self, regs, Channel, Modem, ModemMode, PacketInfo, State};
use crate::{Error, Mode, Sx127x};

impl<Hal> Sx127x<Hal>
where
    Hal: base::Hal,
{
    /// Configure the radio in lora mode with the provided configuration and channel
    pub(crate) fn lora_configure(
        &mut self,
        config: &LoRaConfig,
        channel: &LoRaChannel,
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        debug!("Configuring lora mode");

        // Update internal config
        self.config.channel = Channel::LoRa(channel.clone());
        self.config.modem = Modem::LoRa(config.clone());

        // Switch to sleep to change modem mode
        self.set_state(State::Sleep)?;

        // Switch to LoRa mode (this must happen before set_channel)
        self.set_modem(ModemMode::LoRa)?;

        // Set channel configuration
        self.lora_set_channel(channel)?;

        // Set symbol timeout
        self.write_reg(
            regs::LoRa::SYMBTIMEOUTLSB,
            (config.symbol_timeout & 0xFF) as u8,
        )?;

        // Set preamble length
        self.write_reg(regs::LoRa::PREAMBLEMSB, (config.preamble_len >> 8) as u8)?;
        self.write_reg(regs::LoRa::PREAMBLELSB, (config.preamble_len & 0xFF) as u8)?;

        // Set payload length if constant
        if let PayloadLength::Constant(len) = config.payload_len {
            debug!("Using constant length mode with length: {}", len);
            self.write_reg(regs::LoRa::PAYLOADLENGTH, len as u8)?;
        }

        // Configure frequency hopping if enabled
        if let FrequencyHopping::Enabled(symbol_time) = config.frequency_hop {
            debug!(
                "Enabling frequency hopping with symbol time: {}",
                symbol_time
            );
            self.update_reg(regs::Common::PLLHOP, PLLHOP_FASTHOP_MASK, PLLHOP_FASTHOP_ON)?;
        }

        self.mode = Mode::LoRa;
        self.config.modem = Modem::LoRa(config.clone());
        self.config.channel = Channel::LoRa(channel.clone());

        Ok(())
    }

    pub(crate) fn lora_get_config(&self) -> LoRaConfig {
        match self.config.modem {
            Modem::LoRa(v) => v,
            _ => unreachable!(),
        }
    }

    pub(crate) fn lora_get_channel(&self) -> LoRaChannel {
        match self.config.channel {
            Channel::LoRa(v) => v,
            _ => unreachable!(),
        }
    }

    pub(crate) fn lora_process_rssi_snr(&self, rssi: i16, snr: i16) -> (i16, i16) {
        // Compute SNR
        let snr = if snr & 0x80 != 0 {
            // Invert and divide by four
            -(((!snr + 1) & 0xFF) >> 2)
        } else {
            // Divide by four
            (snr & 0xFF) >> 2
        };

        // Select RSSI offset by band
        let offset = if self.lora_get_channel().freq > RF_MID_BAND_THRESH {
            device::RSSI_OFFSET_HF
        } else {
            device::RSSI_OFFSET_LF
        };

        // Calculate RSSI, depends on SNR
        let rssi = if snr < 0 {
            offset + rssi + (rssi >> 4) + snr
        } else {
            offset + rssi + (rssi >> 4)
        };

        (rssi, snr)
    }

    /// Fetch pending LoRa mode interrupts from the device
    /// If the clear option is set, this will also clear any pending flags
    pub(crate) fn lora_get_interrupts(
        &mut self,
        clear: bool,
    ) -> Result<Irq, Error<<Hal as base::Hal>::Error>> {
        let reg = self.read_reg(regs::LoRa::IRQFLAGS)?;
        let irq = Irq::from_bits(reg).unwrap();

        if clear {
            self.write_reg(regs::LoRa::IRQFLAGS, reg)?;
        }

        Ok(irq)
    }

    /// Set the LoRa mode channel for future receive or transmit operations
    pub(crate) fn lora_set_channel(
        &mut self,
        channel: &LoRaChannel,
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        use device::lora::{Bandwidth::*, SpreadingFactor::*};

        // Set the frequency
        self.set_frequency(channel.freq)?;

        // TODO: this calculation does not encompass all configurations
        let low_dr_optimise = if ((channel.bw as u8) < (Bw125kHz as u8))
            || (channel.bw as u8 == Bw125kHz as u8 && (channel.sf == Sf11 || channel.sf == Sf12))
            || (channel.bw as u8 == Bw250kHz as u8 && channel.sf == Sf12)
        {
            debug!("Using low data rate optimization");
            LowDatarateOptimise::Enabled
        } else {
            LowDatarateOptimise::Disabled
        };

        let implicit_header = match self.lora_get_config().payload_len {
            PayloadLength::Variable => IMPLICITHEADER_DISABLE,
            PayloadLength::Constant(_len) => IMPLICITHEADER_ENABLE,
        };

        // Set modem configuration registers
        self.update_reg(
            regs::LoRa::MODEMCONFIG1,
            BANDWIDTH_MASK | CODERATE_MASK | IMPLICITHEADER_MASK,
            channel.bw as u8 | channel.cr as u8 | implicit_header,
        )?;

        let symbol_timeout_msb = (self.lora_get_config().symbol_timeout >> 8) & 0b0011;
        let payload_crc = self.lora_get_config().payload_crc;

        self.update_reg(
            regs::LoRa::MODEMCONFIG2,
            SPREADING_FACTOR_MASK | RXPAYLOADCRC_MASK | SYMBTIMEOUTMSB_MASK,
            channel.sf as u8 | payload_crc as u8 | symbol_timeout_msb as u8,
        )?;

        self.update_reg(
            regs::LoRa::MODEMCONFIG3,
            LOWDATARATEOPTIMIZE_MASK,
            low_dr_optimise as u8,
        )?;

        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        if channel.bw == Bw500kHz {
            if channel.freq > RF_MID_BAND_THRESH {
                self.write_reg(regs::LoRa::TEST36, 0x02)?;
                self.write_reg(regs::LoRa::TEST3A, 0x64)?;
            } else {
                self.write_reg(regs::LoRa::TEST36, 0x02)?;
                self.write_reg(regs::LoRa::TEST3A, 0x7F)?;
            }
        } else {
            self.write_reg(regs::LoRa::TEST36, 0x03)?;
        }

        // Configure detection optimisation
        // TODO: should we also configure detection thresholds here..?
        if channel.sf == Sf6 {
            self.update_reg(
                regs::LoRa::DETECTOPTIMIZE,
                DETECTIONOPTIMIZE_MASK,
                DetectionOptimize::Sf6 as u8,
            )?;
        } else {
            self.update_reg(
                regs::LoRa::DETECTOPTIMIZE,
                DETECTIONOPTIMIZE_MASK,
                DetectionOptimize::Sf7To12 as u8,
            )?;
        }

        // Update internal channel state
        self.config.channel = Channel::LoRa(channel.clone());

        Ok(())
    }

    /// Start sending a packet
    pub(crate) fn lora_start_transmit(
        &mut self,
        data: &[u8],
    ) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        debug!("Starting send (data: {:?})", data);

        // TODO: support large packet sending
        assert!(data.len() < 255);

        self.set_state_checked(State::Standby)?;

        // Configure IQ inversion
        // TODO: seems this shouldn't be required every time?
        // TODO: Disabled because this isn't working

        #[cfg(feature = "nope")]
        {
            if self.config.invert_iq {
                self.update_reg(
                    regs::LoRa::INVERTIQ,
                    INVERTIQ_TX_MASK | INVERTIQ_RX_MASK,
                    INVERTIQ_RX_OFF | INVERTIQ_TX_ON,
                )?;
                self.write_reg(regs::LoRa::INVERTIQ2, INVERTIQ2_ON)?;
            } else {
                self.update_reg(
                    regs::LoRa::INVERTIQ,
                    INVERTIQ_TX_MASK | INVERTIQ_RX_MASK,
                    INVERTIQ_RX_OFF | INVERTIQ_TX_OFF,
                )?;
                self.write_reg(regs::LoRa::INVERTIQ2, INVERTIQ2_OFF)?;
            }
        }

        // Use whole buffer for TX
        self.write_reg(regs::LoRa::FIFOTXBASEADDR, 0x00)?;
        self.write_reg(regs::LoRa::FIFOADDRPTR, 0x00)?;

        // Write to the FIFO
        self.hal.write_buff(data)?;

        // Set TX length
        self.write_reg(regs::LoRa::PAYLOADLENGTH, data.len() as u8)?;

        // Start TX
        self.set_state_checked(State::Tx)?;

        Ok(())
    }

    /// Check for transmission completion
    /// This method should be polled (or checked following and interrupt) to indicate sending
    /// has completed
    pub(crate) fn lora_check_transmit(&mut self) -> Result<bool, Error<<Hal as base::Hal>::Error>> {
        let irq = self.lora_get_interrupts(true)?;
        trace!("Poll check send, irq: {:?}", irq);

        if irq.contains(Irq::TX_DONE) {
            debug!("Send complete!");
            Ok(true)
        } else {
            trace!("Send pending");
            Ok(false)
        }
    }

    /// Start receive mode
    pub(crate) fn lora_start_receive(&mut self) -> Result<(), Error<<Hal as base::Hal>::Error>> {
        use device::lora::Bandwidth::*;

        debug!("Starting receive");

        self.set_state_checked(State::Standby)?;

        // Configure IQ inversion
        // TODO: seems this shouldn't be required every time?
        // TODO: Disabled because this isn't working
        #[cfg(feature = "nope")]
        {
            if self.config.invert_iq {
                self.update_reg(
                    regs::LoRa::INVERTIQ,
                    INVERTIQ_TX_MASK | INVERTIQ_RX_MASK,
                    INVERTIQ_RX_ON | INVERTIQ_TX_OFF,
                )?;
                self.write_reg(regs::LoRa::INVERTIQ2, INVERTIQ2_ON)?;
            } else {
                self.update_reg(
                    regs::LoRa::INVERTIQ,
                    INVERTIQ_TX_MASK | INVERTIQ_RX_MASK,
                    INVERTIQ_RX_OFF | INVERTIQ_TX_OFF,
                )?;
                self.write_reg(regs::LoRa::INVERTIQ2, INVERTIQ2_OFF)?;
            }
        }

        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        match self.lora_get_channel().bw {
            Bw125kHz => {
                self.write_reg(regs::LoRa::TEST2F, 0x40)?;
            }
            Bw250kHz => {
                self.write_reg(regs::LoRa::TEST2F, 0x40)?;
            }
            _ => {
                self.update_reg(regs::LoRa::DETECTOPTIMIZE, AUTOMATICIF_MASK, AUTOMATICIF_ON)?;
            }
        }

        // Use whole buffer for TX
        self.write_reg(regs::LoRa::FIFORXBASEADDR, 0x00)?;
        self.write_reg(regs::LoRa::FIFOADDRPTR, 0x00)?;

        // Set RX packet max length
        self.write_reg(regs::LoRa::PAYLOADMAXLENGTH, 254)?;

        // Set RX mode
        self.set_state_checked(State::Rx)?;

        Ok(())
    }

    /// Check receive state
    ///
    /// This returns true if a boolean indicating whether a packet has been received.
    /// The restart option specifies whether transient timeout or CRC errors should be
    /// internally handled (returning Ok(false)) or passed back to the caller as errors.
    pub(crate) fn lora_check_receive(
        &mut self,
        restart: bool,
    ) -> Result<bool, Error<<Hal as base::Hal>::Error>> {
        let irq = self.lora_get_interrupts(true)?;
        let mut res = Ok(false);

        if !irq.is_empty() {
            trace!("Poll check receive, irq: {:?}", irq);
        }

        let state = self.get_state()?;
        if state != State::Rx {
            warn!("Check receive unexpected state: {:?}", state);
            res = Err(Error::Aborted);
        }

        // Process flags

        if irq.contains(Irq::RX_DONE) {
            debug!("RX complete");
            res = Ok(true);
        } else if irq.contains(Irq::CRC_ERROR) {
            debug!("RX CRC error");
            res = Err(Error::Crc);
        } else if irq.contains(Irq::RX_TIMEOUT) {
            debug!("RX timeout");
            res = Err(Error::Timeout);
        } else {
            trace!("RX poll");
        }

        match (restart, res) {
            (true, Err(_)) => {
                debug!("RX restarting");
                self.lora_start_receive()?;
                Ok(false)
            }
            (_, r) => r,
        }
    }

    /// Fetch a received message
    ///
    /// This copies data into the provided slice, updates the provided information object,
    ///  and returns the number of bytes received on success
    pub(crate) fn lora_get_received(
        &mut self,
        data: &mut [u8],
    ) -> Result<(usize, PacketInfo), Error<<Hal as base::Hal>::Error>> {
        // Fetch the number of bytes and current RX address pointer
        let n = self.read_reg(regs::LoRa::RXNBBYTES)? as usize;
        let r = self.read_reg(regs::LoRa::FIFORXCURRENTADDR)?;

        let rssi = self.read_reg(regs::LoRa::PKTRSSIVALUE)? as i16;
        let snr = self.read_reg(regs::LoRa::PKTSNRVALUE)? as i16;

        let (rssi, snr) = self.lora_process_rssi_snr(rssi, snr);
        let info = PacketInfo{
            rssi, snr: Some(snr),
        };

        trace!("FIFO RX {} bytes with fifo rx ptr: {}", n, r);

        // Update FIFO pointer to current RX address
        self.write_reg(regs::LoRa::FIFOADDRPTR, r)?;

        if n > data.len() {
            error!(
                "received packet size exceeds buffer size (n: {}, r: {}, max: {})",
                n,
                r,
                data.len()
            );
            return Err(Error::BufferSize);
        }

        // Read data from FIFO
        self.hal.read_buff(&mut data[0..n])?;

        debug!("Received data: {:?} info: {:?}", &data[0..n], &info);

        Ok((n, info))
    }

    /// Poll for the current channel RSSI
    /// This should only be called in receive mode
    pub(crate) fn lora_poll_rssi(&mut self) -> Result<i16, Error<<Hal as base::Hal>::Error>> {
        let raw = self.read_reg(regs::LoRa::RSSIVALUE)?;

        // TODO: hf/lf port switch
        let rssi = match false {
            true => raw as i16 - 157,
            false => raw as i16 - 164,
        };

        Ok(rssi)
    }
}
