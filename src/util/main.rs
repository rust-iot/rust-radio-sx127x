//! Sx127x command line utility
//!
//! Provides mechanisms for command line interaction with Sx127x devices using linux spidev and sysfs_gpio
//!
//! Copyright 2019 Ryan Kurte

use log::{trace, debug, info, error};
use simplelog::{TermLogger, TerminalMode, LevelFilter};

use structopt::StructOpt;


use driver_pal::hal::{HalInst, HalDelay};

use radio_sx127x::prelude::*;


mod options;
use options::*;

mod operations;
use operations::*;

fn main() {
    // Load options
    let opts = Options::from_args();

    // Setup logging
    let mut log_config = simplelog::ConfigBuilder::new();
    log_config.add_filter_ignore("driver_pal".to_string());
    log_config.set_location_level(LevelFilter::Off);

    TermLogger::init(opts.log_level, log_config.build(), TerminalMode::Mixed).unwrap();

    debug!("Connecting to SPI device");

    // Connect to SPI peripheral
    debug!("Connecting to platform SPI");
    trace!("with config: {:?}", opts.spi_config);
    let HalInst{base: _, spi, pins} = match HalInst::load(&opts.spi_config) {
        Ok(v) => v,
        Err(e) => {
            error!("Error connecting to platform HAL: {:?}", e);
            return;
        }
    };

    // Generate configuration
    debug!("Generating configuration");
    let mut config = Config::default();
    match &opts.command {
        Command::LoRa(lora_config) => {
            let modem = LoRaConfig::default();
            let mut channel = LoRaChannel::default();

            if let Some(f) = lora_config.freq_mhz {
                channel.freq = f * 1_000_000;
            }

            config.modem = Modem::LoRa(modem);
            config.channel = Channel::LoRa(channel);
        }
        Command::Gfsk(gfsk_config) => {
            let modem = FskConfig::default();
            let mut channel = FskChannel::default();

            if let Some(f) = gfsk_config.freq_mhz {
                channel.freq = f * 1_000_000;
            }

            config.modem = Modem::FskOok(modem);
            config.channel = Channel::FskOok(channel);
        },
        _ => (),
    }

    debug!("Creating radio instance");
    let mut radio =
        Sx127x::spi(spi, pins.cs, pins.busy, pins.ready, pins.reset, HalDelay {}, &config).expect("error creating device");

    debug!("Executing command");
    match opts.command {
        Command::SiliconVersion => {
            let version = radio
                .silicon_version()
                .expect("error fetching chip version");
            info!("Silicon version: 0x{:X}", version);
            return;
        }
        Command::LoRa(lora_config) => {
            do_command(radio, lora_config.operation).expect("error executing command");
        }
        Command::Gfsk(gfsk_config) => {
            do_command(radio, gfsk_config.operation).expect("error executing command");
        }
    }
}
