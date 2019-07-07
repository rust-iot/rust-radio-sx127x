//! Sx127x LoRa mode RF implementation
//! 
//! This module implements LoRa radio functionality for the Sx127x series devices
//! 
//! Copyright 2019 Ryan Kurte

use radio::{State as _, Channel as _, Power as _, Interrupts as _};

use crate::{Sx127x, Error};
use crate::base::Base as Sx127xBase;
use crate::device::{self, State, Modem, regs};
use crate::device::lora::*;

/// LoRa Radio Configuration Object
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct LoRaConfig {
    /// LoRa channel configuration
    pub channel: Channel,
    /// LoRa preamble length in symbols (defaults to 0x8)
    /// (not that hardware adds four additional symbols)
    pub preamble_len: u16,
    /// TxSingle timeout value (defaults to 0x64)
    pub symbol_timeout: u16,
    /// Payload length configuration (defaults to Variable / Explicit header mode)
    pub payload_len: PayloadLength,
    /// Payload RX CRC configuration (defaults to enabled)
    pub payload_crc: PayloadCrc,
    /// Frequency hopping configuration (defaults to disabled)
    pub frequency_hop: FrequencyHopping,
    /// Power amplifier output selection (defaults to PA_BOOST output)
    pub pa_output: PaSelect,
    /// Output power in dBm (defaults to 10dBm)
    pub power: i8,
    /// IQ inversion configuration (defaults to disabled)
    pub invert_iq: bool,
}

impl Default for LoRaConfig {
    fn default() -> Self {
        LoRaConfig {
            channel: Channel::default(),
            preamble_len: 0x8,
            symbol_timeout: 0x64,
            payload_len: PayloadLength::Variable,
            payload_crc: PayloadCrc::Enabled,
            frequency_hop: FrequencyHopping::Disabled,
            pa_output: PaSelect::Boost,
            power: 10,
            invert_iq: false,
        }
    }
}

/// LoRa radio channel configuration
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct Channel {
    /// LoRa frequency in Hz (defaults to 434 MHz)
    pub frequency: u32,
    /// LoRa channel bandwidth (defaults to 125kHz)
    pub bandwidth: Bandwidth,
    /// LoRa spreading factor (defaults to SF7)
    pub sf: SpreadingFactor,
    /// LoRa coding rate (defaults to 4/5)
    pub coderate: Coderate,
}

impl Default for Channel {
    fn default() -> Self {
        Channel {
            frequency: 434e6 as u32,
            bandwidth: Bandwidth::Bandwidth125kHz,
            sf: SpreadingFactor::Sf7,
            coderate: Coderate::CodingRate1,
        }
    }
}

/// Received packet information
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct Info {
    /// Received Signal Strength Indication
    pub rssi: i16,
    /// Signal to Noise Ratio
    pub snr: i16,
}

impl Default for Info {
    fn default() -> Self {
        Info {
            rssi: 0,
            snr: 0,
        }
    }
}

impl<Base, CommsError, PinError> Sx127x<Base, CommsError, PinError, LoRaConfig>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    /// Configure the radio in lora mode with the provided configuration
    pub fn configure(&mut self, config: &LoRaConfig) -> Result<(), Error<CommsError, PinError>> {
        debug!("Configuring lora mode");

        // Switch to sleep to change modem mode
        self.set_state(State::Sleep)?;

        // Switch to LoRa mode
        self.set_modem(Modem::LoRa)?;

        // Set channel configuration
        self.set_channel(&config.channel)?;

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

        self.config = *config;

        Ok(())
    }

    fn process_rssi_snr(&self, rssi: i16, snr: i16) -> (i16, i16) {
        // Compute SNR
        let snr = if snr & 0x80 != 0 {
            // Invert and divide by four
            -((( !snr + 1 ) & 0xFF) >> 2)
        } else {
            // Divide by four
            (snr & 0xFF) >> 2
        };
       
        // Select RSSI offset by band
        let offset = if self.config.channel.frequency > RF_MID_BAND_THRESH {
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
}

impl<Base, CommsError, PinError> radio::Interrupts for Sx127x<Base, CommsError, PinError, LoRaConfig>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Irq = Irq;
    type Error = Error<CommsError, PinError>;

    /// Fetch pending LoRa mode interrupts from the device
    /// If the clear option is set, this will also clear any pending flags
    fn get_interrupts(&mut self, clear: bool) -> Result<Self::Irq, Self::Error> {
        let reg = self.read_reg(regs::LoRa::IRQFLAGS)?;
        let irq = Irq::from_bits(reg).unwrap();

        if clear {
            self.write_reg(regs::LoRa::IRQFLAGS, reg)?;
        }

        Ok(irq)
    }
}

impl<Base, CommsError, PinError> radio::Power for Sx127x<Base, CommsError, PinError, LoRaConfig>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Error = Error<CommsError, PinError>;

    /// Set LoRa mode transmit power
    fn set_power(&mut self, power: i8) -> Result<(), Error<CommsError, PinError>> {
        // Limit to viable input range
        let power = core::cmp::max(power, 0);

        // Read from config to determine PA mode
        let config = self.read_reg(regs::Common::PACONFIG)?;

        match config & PASELECT_MASK {
            PASELECT_RFO => {
                let max = ((config & MAXPOWER_MASK) >> MAXPOWER_SHIFT) as i8;

                let power = core::cmp::min(power, 17i8);
                let value = power - max + 15;

                let v = self.update_reg(regs::Common::PACONFIG, OUTPUTPOWER_MASK, value as u8)?;

                debug!("Updated RFO PA_CONFIG for: {} dBm to: {:b}", power, v);
            },
            PASELECT_PA_BOOST => {

                let power = core::cmp::min(power, 20i8);
                let value = power - 17 + 15;

                self.update_reg(regs::Common::PACONFIG, OUTPUTPOWER_MASK, value as u8)?;

                let pa_dac_enable = if power > 17 { PADAC_20DBM_ON } else { PADAC_20DBM_OFF };
                let v = self.update_reg(regs::Common::PADAC, PADAC_MASK, pa_dac_enable)?;

                debug!("Updated BOOST PA_CONFIG for: {} dBm to: {:b}", power, v);
            },
            _ => ()
        }

        Ok(())
    }

}

impl<Base, CommsError, PinError> radio::Channel for Sx127x<Base, CommsError, PinError, LoRaConfig>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Channel = Channel;
    type Error = Error<CommsError, PinError>;

    /// Set the LoRa mode channel for future receive or transmit operations
    fn set_channel(&mut self, channel: &Channel) -> Result<(), Error<CommsError, PinError>> {
        use device::lora::{SpreadingFactor::*, Bandwidth::*};
        
        // Set the frequency
        self.set_frequency(channel.frequency)?;

        // TODO: this calculation does not encompass all configurations
        let low_dr_optimise = if ((channel.bandwidth as u8) < (Bandwidth125kHz as u8))
                || (channel.bandwidth as u8 == Bandwidth125kHz as u8 && (channel.sf == Sf11 || channel.sf == Sf12))
                || (channel.bandwidth as u8 == Bandwidth250kHz as u8 && channel.sf == Sf12) {
            debug!("Using low data rate optimization");
            LowDatarateOptimise::Enabled
        } else {
            LowDatarateOptimise::Disabled
        };

        let implicit_header = match self.config.payload_len {
            PayloadLength::Variable => IMPLICITHEADER_DISABLE,
            PayloadLength::Constant(_len) => IMPLICITHEADER_ENABLE,
        };

        // Set modem configuration registers
        self.update_reg(regs::LoRa::MODEMCONFIG1, 
            BANDWIDTH_MASK | CODERATE_MASK | IMPLICITHEADER_MASK,
            channel.bandwidth as u8 | channel.coderate as u8 | implicit_header)?;

        let symbol_timeout_msb = (self.config.symbol_timeout >> 8) & 0b0011;
        let payload_crc = self.config.payload_crc;

        self.update_reg(regs::LoRa::MODEMCONFIG2, 
            SPREADING_FACTOR_MASK | RXPAYLOADCRC_MASK | SYMBTIMEOUTMSB_MASK,
            channel.sf as u8 | payload_crc as u8 | symbol_timeout_msb as u8 )?;

        self.update_reg(regs::LoRa::MODEMCONFIG3, 
            LOWDATARATEOPTIMIZE_MASK,
            low_dr_optimise as u8 )?;

        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        if channel.bandwidth == Bandwidth500kHz {
            if channel.frequency > RF_MID_BAND_THRESH {
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
            self.update_reg(regs::LoRa::DETECTOPTIMIZE, DETECTIONOPTIMIZE_MASK, DetectionOptimize::Sf6 as u8)?;
        } else {
            self.update_reg(regs::LoRa::DETECTOPTIMIZE, DETECTIONOPTIMIZE_MASK, DetectionOptimize::Sf7To12 as u8)?;
        }

        // Update internal channel state
        self.config.channel = *channel;

        Ok(())
    }
}

impl<Base, CommsError, PinError> radio::Transmit for Sx127x<Base, CommsError, PinError, LoRaConfig>
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
    fn check_transmit(&mut self) -> Result<bool, Error<CommsError, PinError>> {
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
}

impl<Base, CommsError, PinError> radio::Receive for Sx127x<Base, CommsError, PinError, LoRaConfig>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Info = Info;
    type Error = Error<CommsError, PinError>;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
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
        match self.config.channel.bandwidth {
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
    fn check_receive(&mut self, restart: bool) -> Result<bool, Self::Error> {
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
            res = Err(Error::Crc);
        } else if irq.contains(Irq::RX_TIMEOUT) {
            debug!("RX timeout");
            res = Err(Error::Timeout);
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

    /// Fetch a received message
    /// 
    /// This copies data into the provided slice, updates the provided information object,
    ///  and returns the number of bytes received on success
    fn get_received(&mut self, info: &mut Self::Info, data: &mut[u8]) -> Result<usize, Self::Error> {
        // Fetch the number of bytes and current RX address pointer
        let n = self.read_reg(regs::LoRa::RXNBBYTES)? as usize;
        let r = self.read_reg(regs::LoRa::FIFORXCURRENTADDR)?;

        let rssi = self.read_reg(regs::LoRa::PKTRSSIVALUE)? as i16;
        let snr = self.read_reg(regs::LoRa::PKTSNRVALUE)? as i16;

        let (rssi, snr) = self.process_rssi_snr(rssi, snr);
        info.rssi = rssi;
        info.snr = snr;

        debug!("FIFO RX {} bytes with fifo rx ptr: {}", n, r);

        // Update FIFO pointer to current RX address
        self.write_reg(regs::LoRa::FIFOADDRPTR, r)?;

        // Read data from FIFO
        self.hal.read_buff(&mut data[0..n])?;

        debug!("Read data: {:?}", &data[0..n]);

        Ok(n)
    }


}

impl<Base, CommsError, PinError> radio::Rssi for Sx127x<Base, CommsError, PinError, LoRaConfig>
where
    Base: Sx127xBase<CommsError, PinError>,
{
    type Error = Error<CommsError, PinError>;

    /// Poll for the current channel RSSI
    /// This should only be called in receive mode
    fn poll_rssi(&mut self) -> Result<i16, Error<CommsError, PinError>> {
        let raw = self.read_reg(regs::LoRa::RSSIVALUE)?;

        // TODO: hf/lf port switch
        let rssi = match false {
            true  => raw as i16 - 157,
            false => raw as i16 - 164,
        };

        Ok(rssi)
    }
}