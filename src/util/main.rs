//! Sx127x command line utility
//! 
//! Provides mechanisms for command line interaction with Sx127x devices using linux spidev and sysfs_gpio 
//! 
//! Copyright 2019 Ryan Kurte

use std::time::Duration;

#[macro_use] extern crate log;
extern crate simplelog;
use simplelog::{TermLogger, LevelFilter};

extern crate structopt;
use structopt::StructOpt;

extern crate humantime;
use humantime::{Duration as HumanDuration};

extern crate linux_embedded_hal;
use linux_embedded_hal::{spidev, Spidev, Pin as PinDev, Delay};
use linux_embedded_hal::sysfs_gpio::Direction;

extern crate radio;

extern crate radio_sx127x;
use radio_sx127x::prelude::*;

#[derive(StructOpt)]
#[structopt(name = "Sx127x-util")]
/// A Command Line Interface (CLI) for interacting with a local Sx127x radio device
/// 
/// Configuration 1:  --cs-pin 16 --rst-pin 17 --busy-pin 5
/// 
/// Configuration 2:  --cs-pin 13 --rst-pin 18 --busy-pin 8
/// 
/// Configuration 3:  --cs-pin 8 --rst-pin 17 --busy-pin 5
/// 
pub struct Options {

    #[structopt(subcommand)]
    /// Request for remote-hal server
    command: Command,


    #[structopt(long = "spi", default_value = "/dev/spidev0.0", env = "SX127X_SPI")]
    /// Specify the hostname of the remote-hal server
    spi: String,

    /// Chip Select (output) pin
    #[structopt(long = "cs-pin", default_value = "16", env = "SX127X_CS")]
    cs: u64,

    /// Reset (output) pin
    #[structopt(long = "rst-pin", default_value = "17", env = "SX127X_RST")]
    rst: u64,

    /// Busy (input) pin
    #[structopt(long = "busy-pin", default_value = "5", env = "SX127X_BUSY")]
    busy: u64,

    /// Baud rate setting
    #[structopt(long = "baud", default_value = "1000000", env = "SX127X_BAUD")]
    baud: u32,

    #[structopt(long = "log-level", default_value = "info")]
    /// Enable verbose logging
    level: LevelFilter,
}

#[derive(StructOpt, PartialEq, Debug)]
pub enum Command {
    #[structopt(name="silicon-version")]
    /// Fetch the device firmware version
    SiliconVersion,

    #[structopt(name="tx")]
    /// Transmit a (string) packet
    Transmit(Transmit),

    #[structopt(name="rx")]
    /// Receive a (string) packet
    Receive(Receive),

    #[structopt(name="rssi")]
    /// Poll for RSSI on the specified channel
    Rssi(Rssi),

    #[structopt(name="repeat")]
    /// Repeat received messages
    Repeat(Repeat),
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Transmit {
    /// Data to be transmitted
    #[structopt(long = "data")]
    data: String,

    /// Run continuously
    #[structopt(long = "continuous")]
    continuous: bool,

    /// Power in dBm
    #[structopt(long = "power", default_value="13")]
    power: i8,

    /// Specify period for transmission
    #[structopt(long = "period", default_value="1s")]
    period: HumanDuration,

    /// Specify period for polling for device status
    #[structopt(long = "poll-interval", default_value="1ms")]
    poll_interval: HumanDuration,
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Receive {
    /// Run continuously
    #[structopt(long = "continuous")]
    continuous: bool,

    /// Specify period for polling for device status
    #[structopt(long = "poll-interval", default_value="1ms")]
    poll_interval: HumanDuration,
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Rssi {
    /// Specify period for RSSI polling
    #[structopt(long = "period", default_value="1s")]
    period: HumanDuration,

    /// Run continuously
    #[structopt(long = "continuous")]
    continuous: bool,
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Repeat {
    /// Run continuously
    #[structopt(long = "continuous")]
    continuous: bool,
    
    /// Power in dBm
    #[structopt(long = "power", default_value="13")]
    power: i8,

    /// Specify period for polling for device status
    #[structopt(long = "poll-interval", default_value="1ms")]
    poll_interval: HumanDuration,

    /// Specify delay for response message
    #[structopt(long = "delay", default_value="100ms")]
    delay: HumanDuration,

    /// Append RSSI and LQI to repeated message
    #[structopt(long = "append-info")]
    append_info: bool,
}


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

    let config = LoRaConfig::default();
    let channel = LoRaChannel::default();

    let radio = Sx127x::spi(spi, cs, busy, rst, Delay{}).expect("error creating device");
    let mut radio = radio.lora(&config, &channel).expect("error configuring lora mode");

    debug!("Executing command");

    // TODO: the rest
    match opts.command {
        Command::SiliconVersion => {
            let version = radio.silicon_version().expect("error fetching silicon version");
            info!("Silicon version: 0x{:X}", version);
        },
        Command::Transmit(config) => {
            do_transmit(radio, config.data.as_bytes(), config.power, config.continuous, *config.period, *config.poll_interval)
                .expect("Transmit error")
        },
        Command::Receive(config) => {
            let mut buff = [0u8; 255];
            let mut info = LoRaInfo::default();

            do_receive(radio, &mut buff, &mut info, config.continuous, *config.poll_interval)
                .expect("Receive error");
        },
        Command::Repeat(config) => {
            let mut buff = [0u8; 255];
            let mut info = LoRaInfo::default();

            do_repeat(radio, &mut buff, &mut info, config.power, config.continuous, *config.delay, *config.poll_interval)
                .expect("Repeat error");
        }
        Command::Rssi(config) => {
            do_rssi(radio, config.continuous, *config.period)
                .expect("RSSI error");
        },
        //_ => warn!("unsuppored command: {:?}", opts.command),
    }
}

fn do_transmit<T, E>(mut radio: T, data: &[u8], power: i8, continuous: bool, period: Duration, poll_interval: Duration) -> Result<(), E> 
where
    T: radio::Transmit<Error=E> + radio::Power<Error=E>
{
    radio.set_power(power)?;

    loop {
        radio.start_transmit(data)?;
        loop {
            if radio.check_transmit()? {
                debug!("Send complete");
                break;
            }
            std::thread::sleep(poll_interval);
        }

        if !continuous {  break; }
        std::thread::sleep(period);
    }

    Ok(())
}

fn do_receive<T, I, E>(mut radio: T, mut buff: &mut [u8], mut info: &mut I, continuous: bool, poll_interval: Duration) -> Result<usize, E> 
where
    T: radio::Receive<Info=I, Error=E>,
    I: std::fmt::Debug,
{
    // Start receive mode
    radio.start_receive()?;

    loop {
        if radio.check_receive(true)? {
            let n = radio.get_received(&mut info, &mut buff)?;

            match std::str::from_utf8(&buff[0..n as usize]) {
                Ok(s) => info!("Received: '{}' info: {:?}", s, info),
                Err(_) => info!("Received: '{:?}' info: {:?}", &buff[0..n as usize], info),
            }
            
            if !continuous { return Ok(n) }
        }

        std::thread::sleep(poll_interval);
    }
}

fn do_rssi<T, I, E>(mut radio: T, continuous: bool, period: Duration) -> Result<(), E> 
where
    T: radio::Receive<Info=I, Error=E> + radio::Rssi<Error=E>,
    I: std::fmt::Debug,
{
    // Enter receive mode
    radio.start_receive()?;

    // Poll for RSSI
    loop {
        let rssi = radio.poll_rssi()?;

        info!("rssi: {}", rssi);

        radio.check_receive(true)?;

        std::thread::sleep(period);

        if !continuous {
            break
        }
    }

    Ok(())
}

fn do_repeat<T, I, E>(mut radio: T, mut buff: &mut [u8], mut info: &mut I, power: i8, continuous: bool, delay: Duration, poll_interval: Duration) -> Result<usize, E> 
where
    T: radio::Receive<Info=I, Error=E> + radio::Transmit<Error=E> + radio::Power<Error=E>,
    I: std::fmt::Debug,
{
    // Set TX power
    radio.set_power(power)?;

    // Start receive mode
    radio.start_receive()?;

    loop {
        if radio.check_receive(true)? {
            let n = radio.get_received(&mut info, &mut buff)?;

            match std::str::from_utf8(&buff[0..n as usize]) {
                Ok(s) => info!("Received: '{}' info: {:?}", s, info),
                Err(_) => info!("Received: '{:?}' info: {:?}", &buff[0..n as usize], info),
            }

            std::thread::sleep(delay);

            radio.start_transmit(&buff[..n])?;
            loop {
                if radio.check_transmit()? {
                    debug!("Send complete");
                    break;
                }
                std::thread::sleep(poll_interval);
            }
            
            if !continuous { return Ok(n) }
        }

        std::thread::sleep(poll_interval);
    }
}
