//! LoRa RF implementation
//! 
//! This module implements LoRa radio functionality
//! 

use crate::{Sx127x, Sx127xError};
use crate::base::Hal as Sx127xHal;
use crate::device::{State, Modem, regs};
use crate::device::lora::*;

impl<Hal, CommsError, PinError> Sx127x<Hal, CommsError, PinError, Config>
where
    Hal: Sx127xHal<CommsError, PinError>,
{
    pub fn set_power(&mut self, power: u8) -> Result<(), Sx127xError<CommsError, PinError>> {
        let mut power = power;

        // Limit to viable input range
        if power > 15 {
            power = 15;
        }

        // Read from config to determine PA mode
        let config = self.read_reg(regs::Common::PACONFIG)?;

        match config & PASELECT_MASK {
            PASELECT_RFO => {
                let max = (config & MAXPOWER_MASK) >> MAXPOWER_SHIFT;

                let power = core::cmp::max(power, 17u8);
                let value = power - max + 15;

                let v = self.update_reg(regs::Common::PACONFIG, OUTPUTPOWER_MASK, value)?;

                debug!("Updated RFO PA_CONFIG to: {:b}", v);
            },
            PASELECT_PA_BOOST => {

                let power = core::cmp::max(power, 20u8);
                let value = power - 17 + 15;

                self.update_reg(regs::Common::PACONFIG, OUTPUTPOWER_MASK, value)?;

                let pa_dac_enable = if power > 17 { PADAC_20DBM_ON } else { PADAC_20DBM_OFF };
                let v = self.update_reg(regs::Common::PADAC, PADAC_MASK, pa_dac_enable)?;

                debug!("Updated BOOST PA_CONFIG to: {:b}", v);
            },
            _ => ()
        }

        Ok(())
    }

    /// Configure the radio in lora mode with the provided configuration
    pub fn configure(&mut self, config: &Config) -> Result<(), Sx127xError<CommsError, PinError>> {
        use device::lora::{SpreadingFactor::*, Bandwidth::*};

        debug!("Configuring lora mode");

        // Switch to sleep to change modem mode
        self.set_state(State::Sleep)?;

        // Switch to LoRa mode
        self.set_modem(Modem::LoRa)?;

        // Set the channel
        self.set_frequency(config.frequency)?;

        // TODO: this calculation does not encompass all configurations
        let low_dr_optimise = if ((config.bandwidth as u8) < (Bandwidth125kHz as u8))
                || (config.bandwidth as u8 == Bandwidth125kHz as u8 && (config.sf == Sf11 || config.sf == Sf12))
                || (config.bandwidth as u8 == Bandwidth250kHz as u8 && config.sf == Sf12) {
            debug!("Using low data rate optimization");
            LowDatarateOptimise::Enabled
        } else {
            LowDatarateOptimise::Disabled
        };

        let implicit_header = match config.payload_len {
            PayloadLength::Variable => IMPLICITHEADER_DISABLE,
            PayloadLength::Constant(_len) => IMPLICITHEADER_ENABLE,
        };

        // Set modem configuration registers

        self.update_reg(regs::LoRa::MODEMCONFIG1, 
            BANDWIDTH_MASK | CODERATE_MASK | IMPLICITHEADER_MASK,
            config.bandwidth as u8 | config.coderate as u8 | implicit_header)?;

        let symbol_timeout_msb = (config.symbol_timeout >> 8) & 0b0011;

        self.update_reg(regs::LoRa::MODEMCONFIG2, 
            SPREADING_FACTOR_MASK | RXPAYLOADCRC_MASK | SYMBTIMEOUTMSB_MASK,
            config.sf as u8 | config.payload_crc as u8 | symbol_timeout_msb as u8 )?;

        self.update_reg(regs::LoRa::MODEMCONFIG3, 
            LOWDATARATEOPTIMIZE_MASK,
            low_dr_optimise as u8 )?;

        // Set symbol timeout
        self.write_reg(regs::LoRa::SYMBTIMEOUTLSB, (config.symbol_timeout & 0xFF) as u8 )?;

        // Set preamble length
        self.write_reg(regs::LoRa::PREAMBLEMSB, (config.preamble_len >> 8) as u8 )?;
        self.write_reg(regs::LoRa::PREAMBLELSB, (config.preamble_len & 0xFF) as u8 )?;

        // Set payload length if constant
        if let PayloadLength::Constant(len) = config.payload_len {
            debug!("Using constant length mode with length: {}", len);
            self.write_reg(regs::LoRa::PAYLOADLENGTH, len as u8 )?;
        }

        // Configure frequency hopping if enabled
        if let FrequencyHopping::Enabled(symbol_time) = config.frequency_hop {
            debug!("Enabling frequency hopping with symbol time: {}", symbol_time);
            self.update_reg(regs::Common::PLLHOP, PLLHOP_FASTHOP_MASK, PLLHOP_FASTHOP_ON)?;
        }

        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        if config.bandwidth == Bandwidth500kHz {
            if config.frequency > RF_MID_BAND_THRESH {
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
        if config.sf == Sf6 {
            self.update_reg(regs::LoRa::DETECTOPTIMIZE, DETECTIONOPTIMIZE_MASK, DetectionOptimize::Sf6 as u8)?;
        } else {
            self.update_reg(regs::LoRa::DETECTOPTIMIZE, DETECTIONOPTIMIZE_MASK, DetectionOptimize::Sf7To12 as u8)?;
        }

        // Set output configuration
        match config.pa_output {
            PaSelect::Rfo(max) => {
                self.update_reg(regs::Common::PACONFIG,
                    PASELECT_MASK | MAXPOWER_MASK,
                    PASELECT_RFO | ((max << MAXPOWER_SHIFT) & MAXPOWER_MASK)
                )?;
            },
            PaSelect::Boost => {
                self.update_reg(regs::Common::PACONFIG, PASELECT_MASK, PASELECT_PA_BOOST)?;
            }
        }

        self.set_power(config.power)?;

        Ok(())
    }

    /// Fetch pending interrupts from the device
    /// If the clear option is set, this will also clear any pending flags
    pub fn get_interrupts(&mut self, clear: bool) -> Result<Irq, Sx127xError<CommsError, PinError>> {
        let reg = self.read_reg(regs::LoRa::IRQFLAGS)?;
        let irq = Irq::from_bits(reg).unwrap();

        if clear {
            self.write_reg(regs::LoRa::IRQFLAGS, reg)?;
        }

        Ok(irq)
    }

    pub fn start_send(&mut self, data: &[u8]) -> Result<(), Sx127xError<CommsError, PinError>> {
        debug!("Starting send (data: {:?})", data);

        // TODO: support large packet sending
        assert!(data.len() < 255);

        self.set_state_checked(State::Standby)?;

        // Configure IQ inversion
        // TODO: seems this shouldn't be required every time?
        // TODO: Disabled because this isn't working

        #[cfg(feature="nope")]
        {
        if self.config.invert_iq {
            self.update_reg(regs::LoRa::INVERTIQ, INVERTIQ_TX_MASK | INVERTIQ_RX_MASK, INVERTIQ_RX_OFF | INVERTIQ_TX_ON)?;
            self.write_reg(regs::LoRa::INVERTIQ2, INVERTIQ2_ON)?;
        } else {
            self.update_reg(regs::LoRa::INVERTIQ, INVERTIQ_TX_MASK | INVERTIQ_RX_MASK, INVERTIQ_RX_OFF | INVERTIQ_TX_OFF)?;
            self.write_reg(regs::LoRa::INVERTIQ2, INVERTIQ2_OFF)?;
        }
        }

        // Use whole buffer for TX
        self.write_reg(regs::LoRa::FIFOTXBASEADDR, 0x00)?;
        self.write_reg(regs::LoRa::FIFOADDRPTR, 0x00)?;

        // Write to the FIFO
        self.hal.buff_write(data)?;

        // Set TX length
        self.write_reg(regs::LoRa::PAYLOADLENGTH, data.len() as u8)?;

        // Start TX
        self.set_state_checked(State::Tx)?;

        Ok(())
    }

    pub fn check_send(&mut self) -> Result<bool, Sx127xError<CommsError, PinError>> {
        let irq = self.get_interrupts(true)?;
        debug!("Poll check send, irq: {:?}", irq);

        if irq.contains(Irq::TX_DONE) {
            debug!("Send complete!");
            Ok(true)
        } else {
            debug!("Send pending");
            Ok(false)
        }
    }

    pub fn start_receive(&mut self) -> Result<(), Sx127xError<CommsError, PinError>> {
        use device::lora::Bandwidth::*;

        debug!("Starting receive");

        self.set_state_checked(State::Standby)?;
        
        // Configure IQ inversion
        // TODO: seems this shouldn't be required every time?
        // TODO: Disabled because this isn't working
        #[cfg(feature="nope")]
        {
        if self.config.invert_iq {
            self.update_reg(regs::LoRa::INVERTIQ, INVERTIQ_TX_MASK | INVERTIQ_RX_MASK, INVERTIQ_RX_ON | INVERTIQ_TX_OFF)?;
            self.write_reg(regs::LoRa::INVERTIQ2, INVERTIQ2_ON)?;
        } else {
            self.update_reg(regs::LoRa::INVERTIQ, INVERTIQ_TX_MASK | INVERTIQ_RX_MASK, INVERTIQ_RX_OFF | INVERTIQ_TX_OFF)?;
            self.write_reg(regs::LoRa::INVERTIQ2, INVERTIQ2_OFF)?;
        }
        }

        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        match self.config.bandwidth {
            Bandwidth125kHz => {
                self.write_reg(regs::LoRa::TEST2F, 0x40)?;
            },
            Bandwidth250kHz => {
                self.write_reg(regs::LoRa::TEST2F, 0x40)?;
            },
            _ => {
                self.update_reg(regs::LoRa::DETECTOPTIMIZE, AUTOMATICIF_MASK, AUTOMATICIF_ON)?;
            }
        }

        // Use whole buffer for TX
        self.write_reg(regs::LoRa::FIFORXBASEADDR, 0x00)?;
        self.write_reg(regs::LoRa::FIFOADDRPTR, 0x00)?;

        self.hal.buff_write(&[0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff])?;
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
    pub fn check_receive(&mut self, restart: bool) -> Result<bool, Sx127xError<CommsError, PinError>> {
        let irq = self.get_interrupts(true)?;
        let mut res = Ok(false);

        if !irq.is_empty() {
            debug!("Poll check receive, irq: {:?}", irq);
        }
       
        // Process flags
        
        if irq.contains(Irq::RX_DONE) {
            debug!("RX complete");
            res = Ok(true);
        } else if irq.contains(Irq::CRC_ERROR) {
            debug!("RX CRC error");
            res = Err(Sx127xError::Crc);
        } else if irq.contains(Irq::RX_TIMEOUT) {
            debug!("RX timeout");
            res = Err(Sx127xError::Timeout);
        } else   {
            trace!("RX poll");
        }

        match (restart, res) {
            (true, Err(_)) => {
                debug!("RX restarting");
                self.start_receive()?;
                Ok(false)
            },
            (_, r) => r
        }
    }

    pub fn get_received(&mut self, data: &mut[u8]) -> Result<u8, Sx127xError<CommsError, PinError>> {
        // Fetch the number of bytes and current RX address pointer
        let n = self.read_reg(regs::LoRa::RXNBBYTES)?;
        let r = self.read_reg(regs::LoRa::FIFORXCURRENTADDR)?;

        debug!("FIFO RX {} bytes with fifo rx ptr: {}", n, r);

        // Update FIFO pointer to current RX address
        self.write_reg(regs::LoRa::FIFOADDRPTR, r)?;

        // Read data from FIFO
        self.hal.buff_read(&mut data[0..n as usize])?;

        debug!("Read data: {:?}", &data[0..n as usize]);

        Ok(n)
    }

    pub fn poll_rssi(&mut self) -> Result<i16, Sx127xError<CommsError, PinError>> {
        let raw = self.read_reg(regs::LoRa::RSSIVALUE)?;

        // TODO: hf/lf port switch
        let rssi = match false {
            true  => raw as i16 - 157,
            false => raw as i16 - 164,
        };

        Ok(rssi)
    }
}