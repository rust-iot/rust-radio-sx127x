use clap::Parser;
use driver_pal::hal::DeviceConfig;
use humantime::Duration as HumanDuration;
use simplelog::LevelFilter;

#[derive(Parser)]
#[clap(name = "Sx127x-util")]
/// A Command Line Interface (CLI) for interacting with a local Sx127x radio device
pub struct Options {
    #[clap(subcommand)]
    /// Subcommand to execute
    pub command: Command,

    #[clap(flatten)]
    pub spi_config: DeviceConfig,

    /// Log verbosity setting
    #[clap(long, default_value = "info")]
    pub log_level: LevelFilter,
}

#[derive(Parser, PartialEq, Debug)]
pub enum Command {
    #[clap(name = "chip-version")]
    /// Fetch the device silicon/firmware version
    SiliconVersion,

    #[clap(name = "lora")]
    /// LoRa mode configuration and operations
    LoRa(LoRaCommand),

    #[clap(name = "gfsk")]
    /// GFSK mode configuration and operations
    Gfsk(GfskCommand),
}

#[derive(Parser, PartialEq, Debug)]
pub enum Operation {
    #[clap(name = "tx")]
    /// Transmit a (string) packet
    Transmit(Transmit),

    #[clap(name = "rx")]
    /// Receive a (string) packet
    Receive(Receive),

    #[clap(name = "rssi")]
    /// Poll for RSSI on the specified channel
    Rssi(Rssi),

    #[clap(name = "repeat")]
    /// Repeat received messages
    Repeat(Repeat),
}

#[derive(Parser, PartialEq, Debug)]
pub struct LoRaCommand {
    /// LoRa frequency in MHz
    #[clap(long = "freq-mhz")]
    pub freq_mhz: Option<u32>,

    #[clap(subcommand)]
    /// Operation to execute
    pub operation: Operation,
}

#[derive(Parser, PartialEq, Debug)]
pub struct GfskCommand {
    /// GFSK frequency in MHz
    #[clap(long = "freq-mhz")]
    pub freq_mhz: Option<u32>,

    #[clap(subcommand)]
    /// Operation to execute
    pub operation: Operation,
}

#[derive(Parser, PartialEq, Debug)]
pub struct Transmit {
    /// Data to be transmitted
    #[clap(long = "data")]
    pub data: String,

    /// Run continuously
    #[clap(long = "continuous")]
    pub continuous: bool,

    /// Power in dBm
    #[clap(long = "power", default_value = "13")]
    pub power: i8,

    /// Specify period for transmission
    #[clap(long = "period", default_value = "1s")]
    pub period: HumanDuration,

    /// Specify period for polling for device status
    #[clap(long = "poll-interval", default_value = "1ms")]
    pub poll_interval: HumanDuration,
}

#[derive(Parser, PartialEq, Debug)]
pub struct Receive {
    /// Run continuously
    #[clap(long = "continuous")]
    pub continuous: bool,

    /// Specify period for polling for device status
    #[clap(long = "poll-interval", default_value = "1ms")]
    pub poll_interval: HumanDuration,
}

#[derive(Parser, PartialEq, Debug)]
pub struct Rssi {
    /// Specify period for RSSI polling
    #[clap(long = "period", default_value = "1s")]
    pub period: HumanDuration,

    /// Run continuously
    #[clap(long = "continuous")]
    pub continuous: bool,
}

#[derive(Parser, PartialEq, Debug)]
pub struct Repeat {
    /// Run continuously
    #[clap(long = "continuous")]
    pub continuous: bool,

    /// Power in dBm
    #[clap(long = "power", default_value = "13")]
    pub power: i8,

    /// Specify period for polling for device status
    #[clap(long = "poll-interval", default_value = "1ms")]
    pub poll_interval: HumanDuration,

    /// Specify delay for response message
    #[clap(long = "delay", default_value = "100ms")]
    pub delay: HumanDuration,

    /// Append RSSI and LQI to repeated message
    #[clap(long = "append-info")]
    pub append_info: bool,
}
