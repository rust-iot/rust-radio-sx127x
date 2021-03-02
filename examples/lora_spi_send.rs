//   ********* MOVED  stm32f1xx_hal setup CODE TO MAIN and removed other setups **************

// setup() TEMPORARILY DISABLED. 

// ATTEMPT BUILD ONLY WITH  MCU stm32f103 and hal stm32f1xx_hal:

//! Transmit a simple message with LoRa using crate radio_sx127x (on SPI).

#![no_std]
#![no_main]

#[cfg(debug_assertions)]
extern crate panic_semihosting;

#[cfg(not(debug_assertions))]
extern crate panic_halt;

extern crate cortex_m_rt;   // not sure why this is needed
use cortex_m_rt::entry;

extern crate cortex_m_semihosting;
use cortex_m_semihosting::*;


extern crate embedded_hal;
use embedded_hal::{blocking::delay::DelayMs,
                   spi::{Mode, Phase, Polarity }, //these need to be from same version as
                                                  //embedded_hal used by stm32*_hal supplying spi
		   };

use radio_sx127x::{prelude::Error, prelude::Sx127x};
//use crate::{prelude::*};                                   // prelude has Sx127x,
use crate::{device::{Modem, Channel, PaConfig, PaSelect,}};
use crate::{device::lora::{LoRaConfig, LoRaChannel, Bandwidth, SpreadingFactor, CodingRate,
                                  PayloadLength, PayloadCrc, FrequencyHopping, }
		   };
//use crate::device::lora::*;
//use crate::device::{self, regs, Channel, Modem, ModemMode, PacketInfo, State};
//use crate::{Error, Mode, Sx127x};

// lora and radio parameters


pub const MODE: Mode = Mode {		    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
    };

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


#[cfg(feature = "stm32f1xx")]  //  eg blue pill stm32f103
use stm32f1xx_hal::{prelude::*,   
                    pac::Peripherals, 
                    spi::{Spi,  Error, Spi1NoRemap, }, // wrapper::Wrapper },
                    delay::Delay,
                    gpio::{gpioa::{PA5, PA6, PA7, PA0, PA1}, Alternate, Output, PushPull, Input,Floating,
                           gpiob::{PB8, PB9}},
		    pac::{SPI1, },
		    }; 


#[entry]
fn main() -> !{
       let cp = cortex_m::Peripherals::take().unwrap();
       let p  = Peripherals::take().unwrap();

       let mut rcc   = p.RCC.constrain();
       let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut p.FLASH.constrain().acr);
       
       let mut afio = p.AFIO.constrain(&mut rcc.apb2);
       let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
       let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    
       let spi = Spi::spi1(
           p.SPI1,
           (gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),  //   sck   on PA5
            gpioa.pa6.into_floating_input(&mut gpioa.crl),       //   miso  on PA6
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl)   //   mosi  on PA7
            ),
               &mut afio.mapr,
           MODE,
           8.mhz(),
           clocks, 
           &mut rcc.apb2,
           );

     
       let delay = Delay::new(cp.SYST, clocks);
       //let delay = DelayMs::new(cp.SYST, clocks);

       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi,					             //Spi
    	    gpioa.pa1.into_push_pull_output(&mut gpioa.crl),         //CsPin         on PA1
    	    gpiob.pb8.into_floating_input(&mut gpiob.crh),           //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(&mut gpiob.crh),           //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(&mut gpioa.crl),         //ResetPin      on PA0
    	    delay,					             //Delay
    	    &CONFIG_RADIO,					     //&Config
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
       
       lora.try_delay_ms(5000u32);
       };
}
