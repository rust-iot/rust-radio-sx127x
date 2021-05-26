//! Serial interface read GPS on usart and transmit with LoRa using crate radio_sx127x (on SPI).
//! See example lora_spi_send for more details.
//! See the MCU device setup() sections for details on pin connections.

//https://www.rfwireless-world.com/Tutorials/LoRa-channels-list.html
// channels are as follows
//   'CH_00_900': 903.08, 'CH_01_900': 905.24, 'CH_02_900': 907.40,
//   'CH_03_900': 909.56, 'CH_04_900': 911.72, 'CH_05_900': 913.88,
//   'CH_06_900': 916.04, 'CH_07_900': 918.20, 'CH_08_900': 920.36,
//   'CH_09_900': 922.52, 'CH_10_900': 924.68, 'CH_11_900': 926.84, 'CH_12_900': 915,
//
//   'CH_10_868': 865.20, 'CH_11_868': 865.50, 'CH_12_868': 865.80,
//   'CH_13_868': 866.10, 'CH_14_868': 866.40, 'CH_15_868': 866.70,
//   'CH_16_868': 867   , 'CH_17_868': 868   ,

// See FREQUENCY below to set the channel.

#![no_std]
#![no_main]

#[cfg(debug_assertions)]
extern crate panic_semihosting;

#[cfg(not(debug_assertions))]
extern crate panic_halt;
//extern crate panic_reset;

use core::convert::Infallible;

use cortex_m_rt::entry;
use cortex_m_semihosting::*;
use nb::block;

use heapless;

use embedded_hal::blocking::delay::DelayMs;

use embedded_hal_compat::IntoCompat;

// MODE needs the old version as it is passed to the device hal crates
//use embedded_hal::{spi::{Mode, Phase, Polarity}, };
use old_e_h::spi::{Mode, Phase, Polarity};

//use asm_delay::{ AsmDelay, bitrate, };

//use cortex_m::asm;  //for breakpoint

use radio_sx127x::Error as sx127xError; // Error name conflict with hals
use radio_sx127x::{
    device::lora::{
        Bandwidth, CodingRate, FrequencyHopping, LoRaChannel, LoRaConfig, PayloadCrc,
        PayloadLength, SpreadingFactor,
    },
    device::{Channel, Modem, PaConfig, PaSelect},
    prelude::*, // prelude has Sx127x,
};

//use radio::{Receive, Transmit};
use radio::Transmit; // trait needs to be in scope to find  methods start_transmit and check_transmit.

// lora and radio parameters

pub const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

const FREQUENCY: u32 = 907_400_000; // frequency in hertz ch_12_900: 915_000_000, ch_2_900: 907_400_000

const CONFIG_CH: LoRaChannel = LoRaChannel {
    freq: FREQUENCY as u32, // frequency in hertz
    bw: Bandwidth::Bw125kHz,
    sf: SpreadingFactor::Sf7,
    cr: CodingRate::Cr4_8,
};

const CONFIG_LORA: LoRaConfig = LoRaConfig {
    preamble_len: 0x8,
    symbol_timeout: 0x64,
    payload_len: PayloadLength::Variable,
    payload_crc: PayloadCrc::Enabled,
    frequency_hop: FrequencyHopping::Disabled,
    invert_iq: false,
};

const CONFIG_PA: PaConfig = PaConfig {
    output: PaSelect::Boost,
    power: 10,
};

//let CONFIG_RADIO = Config::default() ;

const CONFIG_RADIO: radio_sx127x::device::Config = radio_sx127x::device::Config {
    modem: Modem::LoRa(CONFIG_LORA),
    channel: Channel::LoRa(CONFIG_CH),
    pa_config: CONFIG_PA,
    xtal_freq: 32000000, // CHECK
    timeout_ms: 100,
};

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    delay::Delay,
    device::USART2,
    pac::Peripherals,
    prelude::*,
    serial::{Config, Rx, Serial, Tx}, //, StopBits
    spi::{Error, Spi},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    Tx<USART2>,
    Rx<USART2>,
    impl DelayMs<u32> + Transmit<Error = sx127xError<Error, Infallible, Infallible>>,
) {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(64.mhz())
        .pclk1(32.mhz())
        .freeze(&mut p.FLASH.constrain().acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    let (tx, rx) = Serial::usart2(
        p.USART2,
        (
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl), //tx pa2  for GPS rx
            gpioa.pa3,
        ), //rx pa3  for GPS tx
        &mut afio.mapr,
        Config::default().baudrate(9_600.bps()),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    let spi = Spi::spi1(
        p.SPI1,
        (
            gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl), //   sck   on PA5
            gpioa.pa6.into_floating_input(&mut gpioa.crl),      //   miso  on PA6
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl), //   mosi  on PA7
        ),
        &mut afio.mapr,
        MODE,
        8.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let delay = Delay::new(cp.SYST, clocks);

    // Create lora radio instance

    let lora = Sx127x::spi(
        spi.compat(),                                             //Spi
        gpioa.pa1.into_push_pull_output(&mut gpioa.crl).compat(), //CsPin         on PA1
        gpiob.pb8.into_floating_input(&mut gpiob.crh).compat(),   //BusyPin  DIO0 on PB8
        gpiob.pb9.into_floating_input(&mut gpiob.crh).compat(),   //ReadyPin DIO1 on PB9
        gpioa.pa0.into_push_pull_output(&mut gpioa.crl).compat(), //ResetPin      on PA0
        delay.compat(),                                           //Delay
        &CONFIG_RADIO,                                            //&Config
    )
    .unwrap(); // should handle error

    (tx, rx, lora)
}

#[cfg(feature = "stm32f4xx")]
// eg Nucleo-64 stm32f411, blackpill stm32f411, blackpill stm32f401
use stm32f4xx_hal::{
    delay::Delay,
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
    spi::{Error, Spi},
    stm32::Peripherals,
    stm32::USART2,
    time::MegaHertz,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (
    Tx<USART2>,
    Rx<USART2>,
    impl DelayMs<u32> + Transmit<Error = sx127xError<Error, Infallible, Infallible>>,
) {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze();

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();

    let (tx, rx) = Serial::usart2(
        p.USART2,
        (
            gpioa.pa2.into_alternate_af7(), //tx pa2  for GPS rx
            gpioa.pa3.into_alternate_af7(),
        ), //rx pa3  for GPS tx
        Config::default().baudrate(9600.bps()),
        clocks,
    )
    .unwrap()
    .split();

    let spi = Spi::spi1(
        p.SPI1,
        (
            gpioa.pa5.into_alternate_af5(), // sck   on PA5
            gpioa.pa6.into_alternate_af5(), // miso  on PA6
            gpioa.pa7.into_alternate_af5(), // mosi  on PA7
        ),
        MODE,
        MegaHertz(8).into(),
        clocks,
    );

    let delay = Delay::new(cp.SYST, clocks);

    // Create lora radio instance

    // open_drain_output is really input and output. BusyPin is just input, but I think this should work
    //	    gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh),
    // however, gives trait bound  ... InputPin` is not satisfied

    let lora = Sx127x::spi(
        spi.compat(),                               //Spi
        gpioa.pa1.into_push_pull_output().compat(), //CsPin         on PA1
        gpiob.pb8.into_floating_input().compat(),   //BusyPin  DIO0 on PB8
        gpiob.pb9.into_floating_input().compat(),   //ReadyPin DIO1 on PB9
        gpioa.pa0.into_push_pull_output().compat(), //ResetPin      on PA0
        delay.compat(),                             //Delay
        &CONFIG_RADIO,                              //&Config
    )
    .unwrap(); // should handle error

    //DIO0  triggers RxDone/TxDone status.
    //DIO1  triggers RxTimeout and other errors status.
    //D02, D03 ?

    //lora.lora_configure( config_lora, &config_ch ).unwrap(); # not yet pub, to change something

    (tx, rx, lora)
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    let (mut _tx_gps, mut rx_gps, mut lora) = setup(); //  GPS, lora (delay is available in lora)

    // byte buffer   Nov 2020 limit data.len() < 255 in radio_sx127x  .start_transmit
    let mut buffer: heapless::Vec<u8, 80> = heapless::Vec::new();
    let mut buf2:   heapless::Vec<u8, 80> = heapless::Vec::new();

    //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap();  //0 of 80
    //hprintln!("buf2   at {} of {}",   buf2.len(),   buf2.capacity()).unwrap();  //0 of 80
    buffer.clear();
    buf2.clear();

    //hprintln!("going into write/read loop ^C to exit ...").unwrap();

    let e: u8 = 9; // replace char errors with "9"
    let mut good = false; // true while capturing a line

    //let mut size: usize;   // buffer size should not be needed
    //size = buffer.len();   //packet size
    //hprintln!("read buffer {} of {}", size, buffer.capacity()).unwrap();
    hprintln!("entering transmit loop").unwrap();

    loop {
        let byte = match block!(rx_gps.read()) {
            Ok(byt) => byt,
            Err(_error) => e,
        };

        if byte == 36 {
            //  $ is 36. start of a line
            buffer.clear();
            good = true; //start capturing line
        };

        if good {
            if buffer.push(byte).is_err() || byte == 13 {
                //transmit if end of line. \r is 13, \n is 10

                //hprintln!("{:?}", &buffer).unwrap();

                // this transmits the whole GPS message string

                match lora.start_transmit(&buffer) {
                    Ok(b) => b, // b is ()
                    Err(_err) => {
                        hprintln!("Error returned from lora.start_transmit().").unwrap();
                        panic!("should reset in release mode.");
                    }
                };

                // this transmits GPS N and E coordinates in hundredths of degrees

                if &buffer[0..6] == [36, 71, 80, 82, 77, 67] {
                    // if message id is $GPRMC

                    for v in buffer[19..31].iter() {
                        buf2.push(*v).unwrap();
                    } // [19..31] is north/south.
                    for v in b"   ".iter() {
                        buf2.push(*v).unwrap();
                    }
                    for v in buffer[32..45].iter() {
                        buf2.push(*v).unwrap();
                    } // [32..45] is east/west

                    //hprintln!("{:?}", &buf2).unwrap();
                    hprint!(".").unwrap(); // print "."  on transmit of $GPRMC message (but not others)

                    match lora.start_transmit(&buf2) {
                        Ok(b) => b, // b is ()
                        Err(_err) => {
                            hprintln!("Error returned from lora.start_transmit().").unwrap();
                            panic!("should reset in release mode.");
                        }
                    };
                };

                // Note hprintln! requires semihosting. If hprintln! (thus also match section below) are
                // removed then this example works on battery power with no computer attached.
                // (tested only on blackpill with stm32f411 )

                // The first transmission often return false and prints "TX not complete", but works after that.
                // If this continually returns "TX not complete" then the radio should probably be reset,
                //  but should avoid panic_reset after first transmission.

                match lora.check_transmit() {
                    Ok(b) => {
                        if !b {
                            hprintln!("TX not complete").unwrap();
                            // if multible times then panic!("should reset in release mode.");
                        }
                    }
                    Err(_err) => {
                        hprintln!("Error returned from lora.check_transmit().").unwrap();
                        panic!("should reset in release mode.");
                    }
                };

                buffer.clear();
                buf2.clear();
                good = false;
                match lora.try_delay_ms(5000u32) {
                    Ok(b) => b, // b is ()
                    Err(_err) => {
                        hprintln!("Error returned from lora.try_delay_ms().").unwrap();
                        panic!("should reset in release mode.");
                    }
                };
            };
        };
    }
}
