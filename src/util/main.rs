//! Sx127x command line utility
//!
//! Provides mechanisms for command line interaction with Sx127x devices using linux spidev and sysfs_gpio
//!
//! Copyright 2019 Ryan Kurte

#[macro_use]
extern crate log;
extern crate simplelog;
use simplelog::{TermLogger, TerminalMode, LevelFilter};

extern crate structopt;
use structopt::StructOpt;

extern crate humantime;

extern crate linux_embedded_hal;
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{spidev, Delay, Pin as PinDev, Spidev};

extern crate radio;

extern crate radio_sx127x;
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
    log_config.add_filter_ignore("embedded_spi".to_string());
    log_config.set_location_level(LevelFilter::Off);

    TermLogger::init(opts.level, log_config.build(), TerminalMode::Mixed).unwrap();

    debug!("Connecting to SPI device");

    // Connect to hardware
    let mut spi = Spidev::open(opts.spi).expect("error opening spi device");
    let mut config = spidev::SpidevOptions::new();
    config.mode(spidev::SPI_MODE_0 | spidev::SPI_NO_CS);
    config.max_speed_hz(opts.baud);
    spi.configure(&config)
        .expect("error configuring spi device");

    debug!("Configuring I/O pins");

    let cs = PinDev::new(opts.cs);
    cs.export().expect("error exporting cs pin");
    cs.set_direction(Direction::Out)
        .expect("error setting cs pin direction");

    let rst = PinDev::new(opts.rst);
    rst.export().expect("error exporting rst pin");
    rst.set_direction(Direction::Out)
        .expect("error setting rst pin direction");

    let busy = PinDev::new(opts.busy);
    busy.export().expect("error exporting busy pin");
    busy.set_direction(Direction::Out)
        .expect("error setting busy pin direction");

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
        Sx127x::spi(spi, cs, busy, rst, Delay {}, &config).expect("error creating device");

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
