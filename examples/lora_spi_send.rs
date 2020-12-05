//   ********* MOVED  stm32f4xx_hal setup CODE TO MAIN and removed other setups **************

// setup() TEMPORARILY DISABLED. 

// ATTEMPT BUILD ONLY WITH  MCU stm32f411 and hal stm32f4xx_hal:
// cargo build   --target thumbv7em-none-eabihf --no-default-features  --features="stm32f411, stm32f4xx"  --example lora_spi_send


//! Transmit a simple message with LoRa using crate radio_sx127x (on SPI).

#![no_std]
#![no_main]

#[cfg(debug_assertions)]
extern crate panic_semihosting;

#[cfg(not(debug_assertions))]
extern crate panic_halt;

use cortex_m_rt::entry;
use cortex_m_semihosting::*;


use radio_sx127x::{prelude::*,                                    // prelude has Sx127x,
		   device::{Modem, Channel, PaConfig, PaSelect,},
                   device::lora::{LoRaConfig, LoRaChannel, Bandwidth, SpreadingFactor, CodingRate,
                                  PayloadLength, PayloadCrc, FrequencyHopping, },
		   };


// lora and radio parameters


const FREQUENCY: u32 = 907_400_000;           // frequency in hertz ch_12: 915_000_000, ch_2: 907_400_000

const CONFIG_CH: LoRaChannel = LoRaChannel {
	    freq: FREQUENCY as u32,	      // frequency in hertz
	    bw:   Bandwidth::Bw125kHz,
	    sf:   SpreadingFactor::Sf7,
	    cr:   CodingRate::Cr4_8,	      
	    };

const CONFIG_LORA: LoRaConfig = LoRaConfig {
    preamble_len:   0x8,
    symbol_timeout: 0x64,
    payload_len:    PayloadLength::Variable,
    payload_crc:    PayloadCrc::Enabled,
    frequency_hop:  FrequencyHopping::Disabled,
    invert_iq:      false,
    };

const CONFIG_PA: PaConfig = PaConfig {output: PaSelect::Boost, 
                                       power: 10, };


const CONFIG_RADIO: radio_sx127x::device::Config = radio_sx127x::device::Config {
	modem:      Modem::LoRa(CONFIG_LORA),
	channel:    Channel::LoRa(CONFIG_CH),
	pa_config:  CONFIG_PA,
	xtal_freq:  32000000,                  // CHECK
	timeout_ms: 100,
	};

use stm32f4xx_hal::{prelude::*,  
		    pac::Peripherals, 
		    spi::{Spi, Mode, Phase, Polarity},
		    delay::Delay,
		    time::MegaHertz,
		    }; 


// To define explicit type for the lora object
//
//use stm32f4xx_hal::{prelude::*,  
//		      pac::Peripherals, 
//		      spi::{Spi, Error, Mode, Phase, Polarity},
//		      delay::Delay,
//		      time::MegaHertz,
//		      gpio::{gpioa::{PA5, PA6, PA7}, Alternate, AF5,  
//			     gpioa::{PA0, PA1}, Output, PushPull,
//			   gpiob::{PB8, PB9}, Input, Floating},
//		      pac::SPI1,
//		      }; 
//
//    use driver_pal::wrapper::Wrapper;
//
//    type LoraType = Sx127x<Wrapper<Spi<SPI1, 
//	      (PA5<Alternate<AF5>>,    PA6<Alternate<AF5>>,   PA7<Alternate<AF5>>)>,  Error, 
//	      PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
//	      core::convert::Infallible,  Delay,  Delay>,  Error, core::convert::Infallible, Delay>;

#[entry]
fn main() -> !{

       let cp = cortex_m::Peripherals::take().unwrap();
       let p  = Peripherals::take().unwrap();

       let rcc   = p.RCC.constrain();
       let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze();
       
       let gpioa = p.GPIOA.split();
       let gpiob = p.GPIOB.split();

       let spi = Spi::spi1(
           p.SPI1,
           (gpioa.pa5.into_alternate_af5(),  // sck   on PA5
            gpioa.pa6.into_alternate_af5(),  // miso  on PA6
            gpioa.pa7.into_alternate_af5()   // mosi  on PA7
            ),
           Mode {		      //  SPI mode for radio
                phase: Phase::CaptureOnSecondTransition,
                polarity: Polarity::IdleHigh,
                },
           MegaHertz(8).into(),
           clocks,
           );
              
       let delay = Delay::new(cp.SYST, clocks);

       // Create lora radio instance 
    
       //let lora: LoraType = Sx127x::spi(
       let lora = Sx127x::spi(
    	    spi,                                                       //Spi
    	    gpioa.pa1.into_push_pull_output(),                         //CsPin         on PA1
    	    gpiob.pb8.into_floating_input(),                           //BusyPin  DI00 on PB8
            gpiob.pb9.into_floating_input(),                           //ReadyPin DI01 on PB9
    	    gpioa.pa0.into_push_pull_output(),                         //ResetPin      on PA0
    	    delay,					               //Delay
    	    &CONFIG_RADIO,					       //&Config
    	    ).unwrap();      // should handle error
  
       
    let message = b"Hello, LoRa!";
        
    hprintln!("entering loop").unwrap();
    
    loop {
       lora.start_transmit(message).unwrap();    // should handle error
       
       match lora.check_transmit() {
           Ok(b) => if b {hprintln!("TX complete").unwrap()} 
                    else {hprintln!("TX not complete").unwrap()},
           
           Err(_err) => hprintln!("Error in lora.check_transmit(). Should return True or False.").unwrap(),
           };
       
       lora.delay_ms(5000u32);
       };
}
