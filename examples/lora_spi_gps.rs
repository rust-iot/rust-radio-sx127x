//! Serial interface read GPS on usart and transmit with LoRa using crate radio_sx127x (on SPI).
//! See example lora_spi_send for more details.

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

use nb::block;
use cortex_m_rt::entry;
use cortex_m_semihosting::*;

use heapless::{consts, Vec};

use embedded_hal::{blocking::delay::DelayMs,
		   };

use embedded_hal_compat::IntoCompat;
use embedded_hal_compat::eh1_0::blocking::delay::{DelayMs as _};

// MODE needs the old version as it is passed to the device hal crates 
//use embedded_hal::{spi::{Mode, Phase, Polarity}, };
use old_e_h::{spi::{Mode,Phase, Polarity}}; 

//use asm_delay::{ AsmDelay, bitrate, };

//use cortex_m::asm;  //for breakpoint

use radio_sx127x::Error as sx127xError;                           // Error name conflict with hals
use radio_sx127x::{prelude::*,                                    // prelude has Sx127x,
		   device::{Modem, Channel, PaConfig, PaSelect,},
                   device::lora::{LoRaConfig, LoRaChannel, Bandwidth, SpreadingFactor, CodingRate,
                                  PayloadLength, PayloadCrc, FrequencyHopping, },
		   };

//use radio::{Receive, Transmit}; 
use radio::{Transmit}; // trait needs to be in scope to find  methods start_transmit and check_transmit.

// lora and radio parameters

pub const MODE: Mode = Mode {		    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
    };

const FREQUENCY: u32 = 907_400_000;   // frequency in hertz ch_12_900: 915_000_000, ch_2_900: 907_400_000

const CONFIG_CH: LoRaChannel = LoRaChannel {
	    freq: FREQUENCY as u32,	       // frequency in hertz
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

//let CONFIG_RADIO = Config::default() ;

const CONFIG_RADIO: radio_sx127x::device::Config = radio_sx127x::device::Config {
	modem:      Modem::LoRa(CONFIG_LORA),
	channel:    Channel::LoRa(CONFIG_CH),
	pa_config:  CONFIG_PA,
	xtal_freq:  32000000,                  // CHECK
	timeout_ms: 100,
	};


// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")]  //  eg stm32f030xc
use stm32f0xx_hal::{prelude::*,   
                    pac::Peripherals, 
                    serial::{Serial, Tx, Rx},
		    pac::{USART2},
                    spi::{Spi, Error},
                    delay::Delay,
                    }; 

    #[cfg(feature = "stm32f0xx")]
    fn setup() ->  (Tx<USART2>, Rx<USART2>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Infallible, Infallible>> ) {

       let cp = cortex_m::Peripherals::take().unwrap();
       let mut p  = Peripherals::take().unwrap();
       let mut rcc = p.RCC.configure().freeze(&mut p.FLASH);

       let gpioa = p.GPIOA.split(&mut rcc);
       let gpiob = p.GPIOB.split(&mut rcc);
       
       //  stm32f030xc builds with gpiob..into_alternate_af4(cs) USART3 on tx pb10, rx pb11 
       //    but stm32f042  only has 2 usarts. 
       //  Both have gpioa..into_alternate_af1(cs) USART2 with tx on pa2 and rx pa3

       let (tx, rx, sck, miso, mosi, _rst, pa1, pb8, pb9, pa0) = cortex_m::interrupt::free(move |cs| {
            (   
                gpioa.pa2.into_alternate_af1(cs),         //tx pa2  for GPS
                gpioa.pa3.into_alternate_af1(cs),         //rx pa3  for GPS
                gpioa.pa5.into_alternate_af0(cs),         //   sck   on PA5
                gpioa.pa6.into_alternate_af0(cs),	  //   miso  on PA6
                gpioa.pa7.into_alternate_af0(cs),	  //   mosi  on PA7
                
                //gpioa.pa1.into_push_pull_output(cs),	  //   cs            on PA1
                gpiob.pb1.into_push_pull_output(cs),	  //   reset         on PB1
    	    
	        gpioa.pa1.into_push_pull_output(cs),      //   CsPin	     on PA1
    	        gpiob.pb8.into_floating_input(cs),        //   BusyPin  DIO0 on PB8
                gpiob.pb9.into_floating_input(cs),        //   ReadyPin DIO1 on PB9
    	        gpioa.pa0.into_push_pull_output(cs),      //   ResetPin	      on PA0
            )
        });

       let (tx, rx) = Serial::usart2(p.USART2, (tx, rx),  9600.bps(), &mut rcc, ).split();
   
       let spi = Spi::spi1(p.SPI1, (sck, miso, mosi), MODE, 8.mhz(), &mut rcc);
     
       let delay = Delay::new(cp.SYST, &rcc);

       // Create lora radio instance 

       let lora = Sx127x::spi( spi.compat(), pa1.compat(), pb8.compat(), pb9.compat(), pa0.compat(), 
                              delay.compat(), &CONFIG_RADIO, ).unwrap(); // should handle error


       (tx, rx,  lora)
       }


#[cfg(feature = "stm32f1xx")]  //  eg blue pill stm32f103
use stm32f1xx_hal::{prelude::*,   
                    pac::Peripherals, 
                    serial::{Config, Serial, Tx, Rx},  //, StopBits
		    device::{USART2},  
                    spi::{Spi, Error,},
                    delay::Delay,
                    }; 

    #[cfg(feature = "stm32f1xx")]
    fn setup() ->  (Tx<USART2>, Rx<USART2>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Infallible, Infallible>> ) {

       let cp = cortex_m::Peripherals::take().unwrap();
       let p  = Peripherals::take().unwrap();

       let mut rcc   = p.RCC.constrain();
       let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut p.FLASH.constrain().acr);
       
       let mut afio = p.AFIO.constrain(&mut rcc.apb2);
       let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
       let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    
       let (tx, rx) = Serial::usart2(
            p.USART2,
            (gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),    //tx pa2  for GPS rx
             gpioa.pa3), 					    //rx pa3  for GPS tx
            &mut afio.mapr,
            Config::default() .baudrate(9_600.bps()), 
            clocks,
            &mut rcc.apb1,
            ).split();

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

       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi.compat(),					             //Spi
    	    gpioa.pa1.into_push_pull_output(&mut gpioa.crl).compat(),         //CsPin         on PA1
    	    gpiob.pb8.into_floating_input(&mut gpiob.crh).compat(),           //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(&mut gpiob.crh).compat(),           //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(&mut gpioa.crl).compat(),         //ResetPin      on PA0
    	    delay.compat(),					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error
      
       (tx, rx,  lora)
       }


#[cfg(feature = "stm32f3xx")]  //  eg Discovery-stm32f303
use stm32f3xx_hal::{prelude::*, 
                    stm32::Peripherals,
                    serial::{ Serial, Tx, Rx},
		    stm32::{USART2}, 
                    spi::{Spi, Error},
                    delay::Delay,
                    };

    #[cfg(feature = "stm32f3xx")]
    fn setup() ->  (Tx<USART2>, Rx<USART2>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Infallible, Infallible>> ) {
      
       let cp = cortex_m::Peripherals::take().unwrap();
       let p  = Peripherals::take().unwrap();

       let mut rcc   = p.RCC.constrain();
       let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut p.FLASH.constrain().acr);
       
       let mut gpioa = p.GPIOA.split(&mut rcc.ahb);
       let mut gpiob = p.GPIOB.split(&mut rcc.ahb);

       let (tx, rx) = Serial::usart2(
            p.USART2,
            (gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl),    //tx pa2  for GPS rx
             gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl)),   //rx pa3  for GPS tx
            9600.bps(),    // 115_200.bps(),
            clocks,
            &mut rcc.apb1,
            ).split();


       let spi = Spi::spi1(
           p.SPI1,
           (gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl),                // sck   on PA5
            gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl),                // miso  on PA6
            gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl)                 // mosi  on PA7
            ),
           MODE,
           8.mhz(),
           clocks,
           &mut rcc.apb2,
           );
       
       let delay = Delay::new(cp.SYST, clocks);

       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi.compat(),					             //Spi
    	    gpioa.pa1.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper).compat(),   //CsPin	    on PA1
    	    gpiob.pb8.into_floating_input(  &mut gpiob.moder, &mut gpiob.pupdr).compat(),    //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(  &mut gpiob.moder, &mut gpiob.pupdr).compat(),    //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper).compat(),   //ResetPin      on PA0
    	    delay.compat(),					                            //Delay
    	    &CONFIG_RADIO,					                    //&Config
    	    ).unwrap();      // should handle error

       (tx, rx,  lora)
       }


#[cfg(feature = "stm32f4xx")] // eg Nucleo-64 stm32f411, blackpill stm32f411, blackpill stm32f401
use stm32f4xx_hal::{prelude::*,  
                    stm32::Peripherals, 
                    serial::{config::Config, Serial, Tx, Rx},
		    stm32::{USART2}, 
                    spi::{Spi, Error},
                    delay::Delay,
                    time::MegaHertz,
                    }; 

    #[cfg(feature = "stm32f4xx")]
    fn setup() ->  (Tx<USART2>, Rx<USART2>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Infallible, Infallible>> ) {

       let cp = cortex_m::Peripherals::take().unwrap();
       let p  = Peripherals::take().unwrap();

       let rcc   = p.RCC.constrain();
       let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze();
       
       let gpioa = p.GPIOA.split();
       let gpiob = p.GPIOB.split();

       let (tx, rx) = Serial::usart2(
           p.USART2,
           (gpioa.pa2.into_alternate_af7(),            //tx pa2  for GPS rx
	    gpioa.pa3.into_alternate_af7()),           //rx pa3  for GPS tx
           Config::default() .baudrate(9600.bps()), 
           clocks,
           ).unwrap().split();

       let spi = Spi::spi1(
           p.SPI1,
           (gpioa.pa5.into_alternate_af5(),  // sck   on PA5
            gpioa.pa6.into_alternate_af5(),  // miso  on PA6
            gpioa.pa7.into_alternate_af5()   // mosi  on PA7
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
    	    spi.compat(),                                                       //Spi
    	    gpioa.pa1.into_push_pull_output().compat(),                         //CsPin         on PA1
    	    gpiob.pb8.into_floating_input().compat(),                           //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input().compat(),                           //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output().compat(),                         //ResetPin      on PA0
    	    delay.compat(),					               //Delay
    	    &CONFIG_RADIO,					       //&Config
    	    ).unwrap();      // should handle error
  
       //DIO0  triggers RxDone/TxDone status.
       //DIO1  triggers RxTimeout and other errors status.
       //D02, D03 ?
    
       //lora.lora_configure( config_lora, &config_ch ).unwrap(); # not yet pub, to change something

       (tx, rx,  lora)
       }


#[cfg(feature = "stm32f7xx")] 
use stm32f7xx_hal::{prelude::*,  
                    device::Peripherals,                    // note non-standard  device vs pac
                    serial::{Config, Serial, Tx, Rx, Oversampling, },
		    device::{USART2},                       // note non-standard  device vs pac
                    spi::{Spi, ClockDivider, Error, },
                    delay::Delay,
                    }; 

    #[cfg(feature = "stm32f7xx")]
    fn setup() ->  (Tx<USART2>, Rx<USART2>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Infallible, Infallible>> ) {

       let cp = cortex_m::Peripherals::take().unwrap();
       let p  = Peripherals::take().unwrap();

       let mut rcc   = p.RCC.constrain();
       let gpioa = p.GPIOA.split();
       let gpiob = p.GPIOB.split();

       let sck  = gpioa.pa5.into_alternate_af5();  // sck   on PA5
       let miso = gpioa.pa6.into_alternate_af5();  // miso  on PA6
       let mosi = gpioa.pa7.into_alternate_af5();  // mosi  on PA7

       //   somewhere 8.mhz needs to be set in spi

       let spi = Spi::new(p.SPI1, (sck, miso, mosi)).enable::<u8>(
          &mut rcc,
          ClockDivider::DIV32,
          MODE,
          );

       // Relative to other hal setups, Serial::new is after spi::new because  clocks partially consumes rcc.
       
       let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();
       //let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze();

       let (tx, rx) = Serial::new(
           p.USART2,
           (gpioa.pa2.into_alternate_af7(),            //tx pa2  for GPS
	    gpioa.pa3.into_alternate_af7()),           //rx pa3  for GPS
           clocks,
           Config {
                baud_rate: 9600.bps(),
                oversampling: Oversampling::By16,
                },
           ).split();

       let delay = Delay::new(cp.SYST, clocks);

       // Create lora radio instance 

       let lora = Sx127x::spi(
            spi.compat(),					 //Spi
            gpioa.pa1.into_push_pull_output().compat(), 	 //CsPin         on PA1
            gpiob.pb8.into_floating_input().compat(),		 //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input().compat(),            //ReadyPin DIO1 on PB9
            gpioa.pa0.into_push_pull_output().compat(), 	 //ResetPin      on PA0
            delay.compat(),					 //Delay
            &CONFIG_RADIO,					 //&Config
            ).unwrap();      // should handle error
       
       (tx, rx,  lora)
       }


#[cfg(feature = "stm32h7xx")] 
use stm32h7xx_hal::{prelude::*,  
                    pac::Peripherals, 
                    serial::{Tx, Rx},
		    pac::{USART2}, 
                    spi::{Error},
                    delay::Delay,
                    }; 

    #[cfg(feature = "stm32h7xx")]
    fn setup() ->  (Tx<USART2>, Rx<USART2>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, stm32h7xx_hal::Never, Infallible>> ) {

       let cp = cortex_m::Peripherals::take().unwrap();
       let p      = Peripherals::take().unwrap();
       let pwr    = p.PWR.constrain();
       let vos    = pwr.freeze();
       let rcc    = p.RCC.constrain();
       let ccdr   = rcc.sys_ck(160.mhz()).freeze(vos, &p.SYSCFG);
       let clocks = ccdr.clocks;

       let gpioa  = p.GPIOA.split(ccdr.peripheral.GPIOA);
       let gpiob  = p.GPIOB.split(ccdr.peripheral.GPIOB);

       let (tx, rx) = p.USART2.serial((gpioa.pa2.into_alternate_af7(),  //tx pa2 for GPS rx
                                       gpioa.pa3.into_alternate_af7()), //rx pa3 for GPS tx
                                      9600.bps(), 
                                      ccdr.peripheral.USART2, 
                                      &clocks).unwrap().split();


       // following github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/spi.rs
       let spi = p.SPI1.spi(
           (gpioa.pa5.into_alternate_af5(),  // sck   on PA5 
            gpioa.pa6.into_alternate_af5(),  // miso  on PA6 
            gpioa.pa7.into_alternate_af5()   // mosi  on PA7
            ),
           MODE,
           8.mhz(),
           ccdr.peripheral.SPI1,
           &clocks,
           );
       
       let delay = Delay::new(cp.SYST, clocks);
       
       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi.compat(),				                     //Spi
    	    gpioa.pa1.into_push_pull_output().compat(),                       //CsPin         on PA1
    	    gpiob.pb8.into_floating_input().compat(),                         //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input().compat(),                         //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output().compat(),                       //ResetPin      on PA0
    	    delay.compat(),					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error

       (tx, rx,  lora)
       }


//   SKIP TESTING THIS, IT DOES NOT BUILD WITH RELEASE VERSION OF HAL
#[cfg(feature = "stm32l0xx")] 
use stm32l0xx_hal::{prelude::*,  
                    pac::Peripherals, 
                    rcc,   // for ::Config but note name conflict with serial
                    serial::{Config, Tx, Rx, Serial2Ext},
		    pac::{USART2}, 
                    spi::{ Error, },
                    }; 

    #[cfg(feature = "stm32l0xx")] 
    use void::Void;     

    #[cfg(feature = "stm32l0xx")]
    fn setup() ->  (Tx<USART2>, Rx<USART2>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Void, Infallible>> ) {

       let cp = cortex_m::Peripherals::take().unwrap();
       let p         = Peripherals::take().unwrap();
       let mut rcc   = p.RCC.freeze(rcc::Config::hsi16());
       let gpioa     = p.GPIOA.split(&mut rcc);
       let gpiob     = p.GPIOB.split(&mut rcc);

       let (tx, rx) = p.USART2.usart(
           gpioa.pa2,                               //tx pa2  for GPS
	   gpioa.pa3,                               //rx pa3  for GPS
           Config::default() .baudrate(9600.bps()), 
           &mut rcc
           ).unwrap().split();

       // following  github.com/stm32-rs/stm32l0xx-hal/blob/master/examples/spi.rs
       let spi = p.SPI1.spi(
                        (gpioa.pa5,   // sck   on PA5
                         gpioa.pa6,   // miso  on PA6
                         gpioa.pa7    // mosi  on PA7
                         ), 
                        MODE,
                        8.mhz(),
                        &mut rcc
                        );
             
       let delay = cp.SYST.delay(rcc.clocks);

       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi.compat(),				                     //Spi
    	    gpioa.pa1.into_push_pull_output().compat(),                       //CsPin         on PA1
    	    gpiob.pb8.into_floating_input().compat(),                         //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input().compat(),                         //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output().compat(),                       //ResetPin      on PA0
    	    delay.compat(),					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error
       
       (tx, rx,  lora)
       }



#[cfg(feature = "stm32l1xx") ] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{prelude::*, 
                    stm32::Peripherals, 
                    rcc,   // for ::Config but note name conflict with serial
                    serial::{Config, SerialExt, Tx, Rx},
		    stm32::{USART1},
                    spi::{Error,},
                    };

    #[cfg(feature = "stm32l1xx")]
    fn setup() ->  (Tx<USART1>, Rx<USART1>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Infallible, Infallible>> ) {

       let cp = cortex_m::Peripherals::take().unwrap();
       let p         = Peripherals::take().unwrap();
       let mut rcc   = p.RCC.freeze(rcc::Config::hsi());

       let gpioa = p.GPIOA.split();
       let gpiob = p.GPIOB.split();

       let (tx, rx) = p.USART1.usart(
                           (gpioa.pa9,                 //tx pa9   for GPS rx
                            gpioa.pa10),               //rx pa10  for GPS tx
                           Config::default() .baudrate(9600.bps()), 
                           &mut rcc).unwrap().split();

       let spi = p.SPI1.spi(
                          (gpioa.pa5,            // sck   on PA5   in board on Heltec
                           gpioa.pa6,            // miso  on PA6   in board on Heltec
                           gpioa.pa7             // mosi  on PA7   in board on Heltec
                           ), 
                          MODE, 
                          8.mhz(), 
                          &mut rcc
                          );
        
                     
       let delay = cp.SYST.delay(rcc.clocks);
       
       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi.compat(),				                     //Spi
    	    gpioa.pa4.into_push_pull_output().compat(),                       //CsPin         on PA4  in board on Heltec
    	    gpiob.pb11.into_floating_input().compat(),                        //BusyPin  DIO0 on PB11 in board on Heltec
            gpiob.pb10.into_floating_input().compat(),                        //ReadyPin DIO1 on PB10 in board on Heltec
    	    gpioa.pa3.into_push_pull_output().compat(),                       //ResetPin      on PA3  in board on Heltec
    	    delay.compat(),					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error
       
       (tx, rx,  lora)                                                
       }



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{prelude::*,  
                    pac::Peripherals, 
                    serial::{Config, Serial, Tx, Rx},
		    pac::{USART2}, 
                    spi::{Spi, Error,},
                    delay::Delay,
                    }; 

    #[cfg(feature = "stm32l4xx")]
    fn setup() ->  (Tx<USART2>, Rx<USART2>,
                    impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Infallible, Infallible>> ) {

       let cp        = cortex_m::Peripherals::take().unwrap();
       let p         = Peripherals::take().unwrap();
       let mut flash = p.FLASH.constrain();
       let mut rcc   = p.RCC.constrain();
       let mut pwr   = p.PWR.constrain(&mut rcc.apb1r1);
       let clocks    = rcc.cfgr .sysclk(80.mhz()) .pclk1(80.mhz()) 
                                .pclk2(80.mhz()) .freeze(&mut flash.acr, &mut pwr);
      
       let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
       let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

       let (tx, rx) = Serial::usart2(
          p.USART2,
          (gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl),            //tx pa2  for GPS
           gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl)),           //rx pa3  for GPS
          Config::default() .baudrate(9600.bps()), 
          clocks,
          &mut rcc.apb1r1,
          ).split();

       let spi = Spi::spi1(
           p.SPI1,
           (gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl),  // sck   on PA5
            gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl),  // miso  on PA6
            gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl)   // mosi  on PA7
            ),
           MODE,
           8.mhz(),
           clocks,
           &mut rcc.apb2,
           );

       let delay = Delay::new(cp.SYST, clocks);

       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi.compat(),					                             //Spi
    	    gpioa.pa1.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper).compat(),    //CsPin	     on PA1
    	    gpiob.pb8.into_floating_input(  &mut gpiob.moder, &mut gpiob.pupdr).compat(),     //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(  &mut gpiob.moder, &mut gpiob.pupdr).compat(),     //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper).compat(),    //ResetPin      on PA0
    	    delay.compat(),					                             //Delay
    	    &CONFIG_RADIO,					                     //&Config
    	    ).unwrap();      // should handle error
       
       (tx, rx,  lora) 
       }


// End of hal/MCU specific setup. Following should be generic code.


#[entry]
fn main() -> !{

    let (mut _tx_gps, mut rx_gps,   mut lora) =  setup(); //  GPS, lora (delay is available in lora)

    // byte buffer   Nov 2020 limit data.len() < 255 in radio_sx127x  .start_transmit
    let mut buffer: Vec<u8, consts::U80> = Vec::new();
    let mut buf2:   Vec<u8, consts::U80> = Vec::new();

    //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap();  //0 of 80
    //hprintln!("buf2   at {} of {}",   buf2.len(),   buf2.capacity()).unwrap();  //0 of 80
    buffer.clear();
    buf2.clear();

    //hprintln!("going into write/read loop ^C to exit ...").unwrap();

    let e: u8 = 9;           // replace char errors with "9"
    let mut good = false;    // true while capturing a line

    //let mut size: usize;   // buffer size should not be needed
    //size = buffer.len();   //packet size
    //hprintln!("read buffer {} of {}", size, buffer.capacity()).unwrap();
    hprintln!("entering transmit loop").unwrap();

    loop {
        let byte = match block!(rx_gps.read()) {
	    Ok(byt)	  => byt,
	    Err(_error) => e,
	    };
        
	if   byte == 36  {  //  $ is 36. start of a line
	   buffer.clear();
	   good = true;     //start capturing line
	   };
	
	if good {
	   if buffer.push(byte).is_err() ||  byte == 13  { //transmit if end of line. \r is 13, \n is 10
              	      
	      //hprintln!("{:?}", &buffer).unwrap();

	      // this transmits the whole GPS message string
	      
	      match lora.start_transmit(&buffer)  {
		 Ok(b)      => b,  // b is ()
		 Err(_err)  => {hprintln!("Error returned from lora.start_transmit().").unwrap();
		                panic!("should reset in release mode."); 
				},
		 };

	      // this transmits GPS N and E coordinates in hundredths of degrees
	      
	      if &buffer[0..6] == [36, 71, 80, 82, 77, 67] {   // if message id is $GPRMC
	          
		  for v in buffer[19..31].iter() { buf2.push(*v).unwrap(); };  // [19..31] is north/south.
 		  for v in         b"   ".iter() { buf2.push(*v).unwrap(); };
 		  for v in buffer[32..45].iter() { buf2.push(*v).unwrap(); };  // [32..45] is east/west
	          
	          //hprintln!("{:?}", &buf2).unwrap();
	          hprint!(".").unwrap();   // print "."  on transmit of $GPRMC message (but not others)

		  match lora.start_transmit(&buf2)  {
		     Ok(b)      => b,  // b is ()
		     Err(_err)  => {hprintln!("Error returned from lora.start_transmit().").unwrap();
		                    panic!("should reset in release mode."); 
				    },
		     };
		  };
     
	      // Note hprintln! requires semihosting. If hprintln! (thus also match section below) are 
	      // removed then this example works on battery power with no computer attached.
	      // (tested only on blackpill with stm32f411 )
     	      
	      // The first transmission often return false and prints "TX not complete", but works after that.
	      // If this continually returns "TX not complete" then the radio should probably be reset,
	      //  but should avoid panic_reset after first transmission. 
	      
	      match lora.check_transmit() {
		 Ok(b)      => if ! b  { hprintln!("TX not complete").unwrap();
		                         // if multible times then panic!("should reset in release mode."); 
					 },
		 Err(_err)  =>   {hprintln!("Error returned from lora.check_transmit().").unwrap();
		                  panic!("should reset in release mode."); 
				  },
		 };
                 	      
	      buffer.clear();
	      buf2.clear();
     	      good = false;
	      lora.try_delay_ms(5000u32);
     	      };
     	   };
     	}
}
