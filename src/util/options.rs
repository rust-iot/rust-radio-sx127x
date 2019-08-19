
use structopt::StructOpt;
use simplelog::{LevelFilter};
use humantime::{Duration as HumanDuration};


#[derive(StructOpt)]
#[structopt(name = "Sx127x-util")]
/// A Command Line Interface (CLI) for interacting with a local Sx127x radio device
pub struct Options {

    #[structopt(subcommand)]
    /// Subcommand to execute
    pub command: Command,

    /// SPI device for radio connection
    #[structopt(long = "spi", default_value = "/dev/spidev0.0", env = "SX127X_SPI")]
    pub spi: String,

    /// Chip Select (output) pin
    #[structopt(long = "cs-pin", default_value = "16", env = "SX127X_CS")]
    pub cs: u64,

    /// Reset (output) pin
    #[structopt(long = "rst-pin", default_value = "17", env = "SX127X_RST")]
    pub rst: u64,

    /// Busy (input) pin
    #[structopt(long = "busy-pin", default_value = "5", env = "SX127X_BUSY")]
    pub busy: u64,

    /// Baud rate setting
    #[structopt(long = "baud", default_value = "1000000", env = "SX127X_BAUD")]
    pub baud: u32,

    /// Log verbosity setting
    #[structopt(long = "log-level", default_value = "info")]
    pub level: LevelFilter,
}

#[derive(StructOpt, PartialEq, Debug)]
pub enum Command {
    #[structopt(name="chip-version")]
    /// Fetch the device silicon/firmware version
    SiliconVersion,

    #[structopt(name="lora")]
    /// LoRa mode configuration and operations
    LoRa(LoRaCommand),

    #[structopt(name="gfsk")]
    /// GFSK mode configuration and operations
    Gfsk(GfskCommand),
}

#[derive(StructOpt, PartialEq, Debug)]
pub enum Operation {

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
pub struct LoRaCommand {
    /// LoRa frequency in MHz
    #[structopt(long = "freq-mhz")]
    pub freq_mhz: Option<u32>,

    #[structopt(subcommand)]
    /// Operation to execute
    pub operation: Operation,
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct GfskCommand {
    /// GFSK frequency in MHz
    #[structopt(long = "freq-mhz")]
    pub freq_mhz: Option<u32>,

    #[structopt(subcommand)]
    /// Operation to execute
    pub operation: Operation,
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Transmit {
    /// Data to be transmitted
    #[structopt(long = "data")]
    pub data: String,

    /// Run continuously
    #[structopt(long = "continuous")]
    pub continuous: bool,

    /// Power in dBm
    #[structopt(long = "power", default_value="13")]
    pub power: i8,

    /// Specify period for transmission
    #[structopt(long = "period", default_value="1s")]
    pub period: HumanDuration,

    /// Specify period for polling for device status
    #[structopt(long = "poll-interval", default_value="1ms")]
    pub poll_interval: HumanDuration,
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Receive {
    /// Run continuously
    #[structopt(long = "continuous")]
    pub continuous: bool,

    /// Specify period for polling for device status
    #[structopt(long = "poll-interval", default_value="1ms")]
    pub poll_interval: HumanDuration,
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Rssi {
    /// Specify period for RSSI polling
    #[structopt(long = "period", default_value="1s")]
    pub period: HumanDuration,

    /// Run continuously
    #[structopt(long = "continuous")]
    pub continuous: bool,
}

#[derive(StructOpt, PartialEq, Debug)]
pub struct Repeat {
    /// Run continuously
    #[structopt(long = "continuous")]
    pub continuous: bool,
    
    /// Power in dBm
    #[structopt(long = "power", default_value="13")]
    pub power: i8,

    /// Specify period for polling for device status
    #[structopt(long = "poll-interval", default_value="1ms")]
    pub poll_interval: HumanDuration,

    /// Specify delay for response message
    #[structopt(long = "delay", default_value="100ms")]
    pub delay: HumanDuration,

    /// Append RSSI and LQI to repeated message
    #[structopt(long = "append-info")]
    pub append_info: bool,
}