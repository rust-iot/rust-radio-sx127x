//! Sx127x command line utility
//! 
//! Provides mechanisms for command line interaction with Sx127x devices using linux spidev and sysfs_gpio 
//! 
//! Copyright 2019 Ryan Kurte

#[macro_use] extern crate log;
extern crate simplelog;
use simplelog::{TermLogger};

extern crate structopt;
use structopt::StructOpt;

extern crate humantime;

extern crate linux_embedded_hal;
use linux_embedded_hal::{spidev, Spidev, Pin as PinDev, Delay};
use linux_embedded_hal::sysfs_gpio::Direction;

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
    TermLogger::init(opts.level, simplelog::Config::default()).unwrap();

    debug!("Connecting to SPI device");

    // Connect to hardware
    let mut spi = Spidev::open(opts.spi).expect("error opening spi device");
    let mut config = spidev::SpidevOptions::new();
    config.mode(spidev::SPI_MODE_0 | spidev::SPI_NO_CS);
    config.max_speed_hz(opts.baud);
    spi.configure(&config).expect("error configuring spi device");

    debug!("Configuring I/O pins");

    let cs = PinDev::new(opts.cs);
    cs.export().expect("error exporting cs pin");
    cs.set_direction(Direction::Out).expect("error setting cs pin direction");

    let rst = PinDev::new(opts.rst);
    rst.export().expect("error exporting rst pin");
    rst.set_direction(Direction::Out).expect("error setting rst pin direction");

    let busy = PinDev::new(opts.busy);
    busy.export().expect("error exporting busy pin");
    busy.set_direction(Direction::Out).expect("error setting busy pin direction");

    debug!("Creating radio instance");
    let mut radio = Sx127x::spi(spi, cs, busy, rst, Delay{}).expect("error creating device");

    debug!("Executing command");
    match opts.command {
        Command::SiliconVersion => {
            let version = radio.silicon_version().expect("error fetching chip version");
            info!("Silicon version: 0x{:X}", version);
            return
        },
        Command::LoRa(lora_config) => {
            let config = LoRaConfig::default();
            let mut channel = LoRaChannel::default();

            channel.freq = lora_config.freq;
            
            let radio = radio.lora(&config, &channel).expect("error configuring lora mode");

            do_command(radio, lora_config.operation).expect("error executing command");
        },
        Command::Gfsk(gfsk_config) => {
            let config = FskConfig::default();
            let channel = FskChannel::default();

            let radio = radio.fsk(&config, &channel).expect("error configuring gfsk mode");

            do_command(radio, gfsk_config.operation).expect("error executing command");
        }
    }

}

