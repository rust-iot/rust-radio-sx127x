
#[macro_use] extern crate log;
extern crate simplelog;
use simplelog::{TermLogger, LevelFilter};

extern crate structopt;
use structopt::StructOpt;

extern crate linux_embedded_hal;
use linux_embedded_hal::{spidev, Spidev, Pin as PinDev, Delay};
use linux_embedded_hal::sysfs_gpio::Direction;


extern crate radio_sx127x;
use radio_sx127x::{Sx127x, Settings};

#[derive(StructOpt)]
#[structopt(name = "Sx127x-util", about = "A Command Line Interface (CLI) for interacting with a local Sx127x radio device")]
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

    /// DIO1 pin
    #[structopt(long = "dio1-pin", default_value = "20", env = "SX127X_DIO1")]
    dio1: u64,

    /// DIO2 pin
    #[structopt(long = "dio2-pin", default_value = "23", env = "SX127X_DIO2")]
    _dio2: u64,

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
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Transmit {
    #[structopt(long = "data")]
    data: String
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Receive {

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
    config.mode(spidev::SPI_MODE_0);
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

    let dio1 = PinDev::new(opts.dio1);
    dio1.export().expect("error exporting dio1 pin");
    dio1.set_direction(Direction::Out).expect("error setting dio1 pin direction");


    debug!("Creating radio instance");

    let settings = Settings::default();
    let mut radio = Sx127x::spi(spi, cs, busy, rst, Delay{}, settings).expect("error creating device");

    debug!("Executing command");

    // TODO: the rest
    match opts.command {
        Command::SiliconVersion => {
            let version = radio.silicon_version().expect("error fetching silicon version");
            info!("Silicon version: 0x{:X}", version);
        }
        Command::Transmit(tx) => {
            radio.start_send( tx.data.as_bytes() ).expect("error starting send");
            while radio.check_send().expect("error checking send") != true {}
        },
        Command::Receive(rx) => {
            radio.start_receive().expect("error starting receive");
            while radio.check_receive(false).expect("error checking receive") != true {}
        },
        //_ => warn!("unsuppored command: {:?}", opts.command),
    }

}