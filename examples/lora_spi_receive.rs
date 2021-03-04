//! Receive message with LoRa using crate radio_sx127x (on SPI).
//! See example lora_spi_send for more details.

//   Using  sck, miso, mosi, cs, reset and D00, D01. Not yet using  D02, D03
//   See setup() sections below for pins.


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

// use nb::block;
use cortex_m_rt::entry;
use cortex_m_semihosting::*;

use embedded_hal::{blocking::delay::DelayMs,
                   spi::{Mode, Phase, Polarity},
		   };

//use asm_delay::{ AsmDelay, bitrate, };

//use cortex_m::asm;  //for breakpoint

extern crate radio_sx127x;
use radio_sx127x::{prelude::*,                                     // prelude has Sx127x,
		   device::{Modem, Channel, PaConfig, PaSelect,},
                   device::lora::{LoRaConfig, LoRaChannel, Bandwidth, SpreadingFactor, CodingRate,
                                  PayloadLength, PayloadCrc, FrequencyHopping, },
		   };

// trait needs to be in scope to find  methods start_transmit and check_transmit.
//use radio::{Receive, Transmit, Radio}; 
//use radio::{Receive, ReceiveInfo}; 
use radio::{Receive, }; 
use embedded_spi::wrapper::Wrapper;

// lora and radio parameters

pub const MODE: Mode = Mode {		    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
    };

const FREQUENCY: u32 = 907_400_000;  // frequency in hertz ch_12: 915_000_000, ch_2: 907_400_000

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
                    spi::{Spi, EightBit, Error},
                    delay::Delay,
                    gpio::{gpioa::{PA5, PA6, PA7}, Alternate, Input, AF0,
                           gpioa::{PA0, PA1}, Output, PushPull,
                           gpiob::{PB8, PB9}, Floating },
                    pac::SPI1,
                    }; 


    #[cfg(feature = "stm32f0xx")]
    fn setup() ->   Sx127x<Wrapper<Spi<SPI1, 
                        PA5<Alternate<AF0>>,  PA6<Alternate<AF0>>, PA7<Alternate<AF0>>,  EightBit>, Error, 
                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
                   core::convert::Infallible,  Delay>, Error, core::convert::Infallible> {

       let cp = cortex_m::Peripherals::take().unwrap();
       let mut p  = Peripherals::take().unwrap();
       let mut rcc = p.RCC.configure().freeze(&mut p.FLASH);

       let gpioa = p.GPIOA.split(&mut rcc);
       let gpiob = p.GPIOB.split(&mut rcc);
       
       let (sck, miso, mosi, _rst, pa1, pb8, pb9, pa0) = cortex_m::interrupt::free(move |cs| {
            (   gpioa.pa5.into_alternate_af0(cs),         //   sck   on PA5
                gpioa.pa6.into_alternate_af0(cs),	  //   miso  on PA6
                gpioa.pa7.into_alternate_af0(cs),	  //   mosi  on PA7
                
                //gpioa.pa1.into_push_pull_output(cs),	  //   cs            on PA1
                gpiob.pb1.into_push_pull_output(cs),	  //   reset         on PB1
    	    
	        gpioa.pa1.into_push_pull_output(cs),      //   CsPin	     on PA1
    	        gpiob.pb8.into_floating_input(cs),        //   BusyPin  DIO0 on PB8
                gpiob.pb9.into_floating_input(cs),        //   ReadyPin DIO1 on PB9
    	        gpioa.pa0.into_push_pull_output(cs),      //   ResetPin	     on PA0
            )
        });

   
       let spi = Spi::spi1(p.SPI1, (sck, miso, mosi), MODE, 8.mhz(), &mut rcc);
     
       let delay = Delay::new(cp.SYST, &rcc);

       // Create lora radio instance 

       let lora = Sx127x::spi( spi, pa1, pb8, pb9, pa0, delay, &CONFIG_RADIO, ).unwrap(); // should handle error

       lora
       }


#[cfg(feature = "stm32f1xx")]  //  eg blue pill stm32f103
use stm32f1xx_hal::{prelude::*,   
                    pac::Peripherals, 
                    spi::{Spi, Spi1NoRemap, Error,},
                    delay::Delay,
                    gpio::{gpioa::{PA5, PA6, PA7}, Alternate, Input, Floating,  
                           gpioa::{PA0, PA1}, Output, PushPull,
			   gpiob::{PB8, PB9},  },
                    device::SPI1,
                    }; 

    #[cfg(feature = "stm32f1xx")]
    fn setup() ->  Sx127x<Wrapper<Spi<SPI1, Spi1NoRemap,
                        (PA5<Alternate<PushPull>>,  PA6<Input<Floating>>, PA7<Alternate<PushPull>>), u8>, Error, 
                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
                   core::convert::Infallible,  Delay>, Error, core::convert::Infallible> {

    //fn setup() ->  impl Transmit {


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
      
       lora
       }


#[cfg(feature = "stm32f3xx")]  //  eg Discovery-stm32f303
use stm32f3xx_hal::{prelude::*, 
                    stm32::Peripherals,
                    spi::{Spi, Error},
                    delay::Delay,
                    gpio::{gpioa::{PA5, PA6, PA7}, AF5,  
                           gpioa::{PA0, PA1}, Output, PushPull,
			   gpiob::{PB8, PB9}, Input, Floating},
                    stm32::SPI1,
                    };

    #[cfg(feature = "stm32f3xx")]
    fn setup() ->  Sx127x<Wrapper<Spi<SPI1, 
                           (PA5<AF5>,    PA6<AF5>,   PA7<AF5>)>,  Error, 
                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
                   core::convert::Infallible,  Delay>,  Error, core::convert::Infallible> {
    
    //fn setup() ->  impl Transmit {
      
       let cp = cortex_m::Peripherals::take().unwrap();
       let p  = Peripherals::take().unwrap();

       let mut rcc   = p.RCC.constrain();
       let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut p.FLASH.constrain().acr);
       
       let mut gpioa = p.GPIOA.split(&mut rcc.ahb);
       let mut gpiob = p.GPIOB.split(&mut rcc.ahb);

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
    	    spi,					             //Spi
    	    gpioa.pa1.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper),   //CsPin	    on PA1
    	    gpiob.pb8.into_floating_input(&mut gpiob.moder,   &mut gpiob.pupdr),    //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(&mut gpiob.moder,   &mut gpiob.pupdr),    //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper),   //ResetPin      on PA0
    	    delay,					                            //Delay
    	    &CONFIG_RADIO,					                    //&Config
    	    ).unwrap();      // should handle error

       lora
       }


#[cfg(feature = "stm32f4xx")] // eg Nucleo-64 stm32f411, blackpill stm32f411, blackpill stm32f401
use stm32f4xx_hal::{prelude::*,  
                    stm32::{Peripherals, SPI1},
                    spi::{Spi, Error},
                    delay::Delay,
                    gpio::{gpioa::{PA5, PA6, PA7}, Alternate, AF5,  
                           gpioa::{PA0, PA1}, Output, PushPull,
			   gpiob::{PB8, PB9}, Input, Floating},
                    time::MegaHertz,
                    }; 


    #[cfg(feature = "stm32f4xx")]
    fn setup() ->  Sx127x<Wrapper<Spi<SPI1, 
                           (PA5<Alternate<AF5>>,    PA6<Alternate<AF5>>,   PA7<Alternate<AF5>>)>,  Error, 
                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
                   core::convert::Infallible,  Delay>,  Error, core::convert::Infallible> {

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
    	    spi,                                                       //Spi
    	    gpioa.pa1.into_push_pull_output(),                         //CsPin         on PA1
    	    gpiob.pb8.into_floating_input(),                           //BusyPin  DI00 on PB8
            gpiob.pb9.into_floating_input(),                           //ReadyPin DI01 on PB9
    	    gpioa.pa0.into_push_pull_output(),                         //ResetPin      on PA0
    	    delay,					               //Delay
    	    &CONFIG_RADIO,					       //&Config
    	    ).unwrap();      // should handle error
  
       //DIO0  triggers RxDone/TxDone status.
       //DIO1  triggers RxTimeout and other errors status.
       //D02, D03 ?
    
       //lora.lora_configure( config_lora, &config_ch ).unwrap(); # not yet pub, to change something

       lora
       }


#[cfg(feature = "stm32f7xx")] 
use stm32f7xx_hal::{prelude::*,  
                    pac::Peripherals, 
                    spi::{Spi, Pins, Enabled, ClockDivider, Error,},
                    delay::Delay,
                    gpio::{gpioa::{PA0, PA1}, Output, PushPull,
			   gpiob::{PB8, PB9}, Input, Floating},
                    pac::SPI1,
                    }; 

    #[cfg(feature = "stm32f7xx")]
    fn setup() -> Sx127x<Wrapper<Spi<SPI1,impl Pins<SPI1>, Enabled<u8>>,  Error, 
                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
                   core::convert::Infallible,  Delay>,  Error, core::convert::Infallible> {


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
          &mut rcc.apb2,
          ClockDivider::DIV32,
          MODE,
          );

       let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze();

       let delay = Delay::new(cp.SYST, clocks);

       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi,					             //Spi
    	    gpioa.pa1.into_push_pull_output(),                       //CsPin         on PA1
    	    gpiob.pb8.into_floating_input(),                         //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(),                         //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(),                       //ResetPin      on PA0
    	    delay,					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error
       
       lora
       }


#[cfg(feature = "stm32h7xx")] 
use stm32h7xx_hal::{prelude::*,  
                    pac::Peripherals, 
                    spi::{Spi, Enabled, Error},
                    delay::Delay,
                    gpio::{   //gpioa::{PA5, PA6, PA7}, Alternate, AF5,  really!
                           gpioa::{PA0, PA1}, Output, PushPull,
                           gpiob::{PB8, PB9}, Input, Floating},
                    pac::SPI1,
                    }; 

    #[cfg(feature = "stm32h7xx")]
    fn setup() -> Sx127x<Wrapper<Spi<SPI1, Enabled>, Error, 
                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
                   stm32h7xx_hal::Never,  Delay>,  Error, stm32h7xx_hal::Never> {
    //fn setup() ->  impl Receive<Error=radio_sx127x::Error<Error, stm32h7xx_hal::Never>> {

       let cp = cortex_m::Peripherals::take().unwrap();
       let p      = Peripherals::take().unwrap();
       let pwr    = p.PWR.constrain();
       let vos    = pwr.freeze();
       let rcc    = p.RCC.constrain();
       let ccdr   = rcc.sys_ck(160.mhz()).freeze(vos, &p.SYSCFG);
       let clocks = ccdr.clocks;

       let gpioa  = p.GPIOA.split(ccdr.peripheral.GPIOA);
       let gpiob  = p.GPIOB.split(ccdr.peripheral.GPIOB);

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
    	    spi,				                     //Spi
    	    gpioa.pa1.into_push_pull_output(),                       //CsPin         on PA1
    	    gpiob.pb8.into_floating_input(),                         //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(),                         //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(),                       //ResetPin      on PA0
    	    delay,					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error

       lora
       }


//   SKIP TESTING THIS, IT DOES NOT BUILD WITH RELEASE VERSION OF HAL
#[cfg(feature = "stm32l0xx")] 
use stm32l0xx_hal::{prelude::*,  
                    pac::Peripherals, 
                    rcc,   // for ::Config but note name conflict with serial
                    spi::{Spi, Pins, Error, },
                    delay::Delay,
                    gpio::{gpioa::{PA0, PA1}, Output, PushPull,
			   gpiob::{PB8, PB9}, Input, Floating},
                    pac::SPI1,
                    }; 

    #[cfg(feature = "stm32l0xx")]
    fn setup() -> Sx127x<Wrapper<Spi<SPI1,impl Pins<SPI1>>, Error, 
                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
                   void::Void,  Delay>,  Error, void::Void> {

       let cp = cortex_m::Peripherals::take().unwrap();
       let p         = Peripherals::take().unwrap();
       let mut rcc   = p.RCC.freeze(rcc::Config::hsi16());
       let gpioa     = p.GPIOA.split(&mut rcc);
       let gpiob     = p.GPIOB.split(&mut rcc);

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
    	    spi,				                     //Spi
    	    gpioa.pa1.into_push_pull_output(),                       //CsPin         on PA1
    	    gpiob.pb8.into_floating_input(),                         //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(),                         //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(),                       //ResetPin      on PA0
    	    delay,					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error
       
       lora
       }



#[cfg(feature = "stm32l1xx") ] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{prelude::*, 
                    stm32::Peripherals, 
                    rcc,   // for ::Config but note name conflict with serial
                    spi::{Spi, Pins, Error,},
                    delay::Delay,
                    gpio::{//gpioa::{PA5, PA6, PA7}, Input,  Floating,   
                           gpioa::{PA3, PA4}, Output, PushPull,
			   gpiob::{PB11, PB10}, Input, Floating},
                    stm32::SPI1,
                    };

    #[cfg(feature = "stm32l1xx")]
    fn setup() -> Sx127x<Wrapper<Spi<SPI1,impl Pins<SPI1>>, Error, 
                   PA4<Output<PushPull>>,  PB11<Input<Floating>>,  PB10<Input<Floating>>,  PA3<Output<PushPull>>, 
                   core::convert::Infallible,  Delay>,  Error, core::convert::Infallible> {

       // instead of impl Pins<SPI1>  above could use 
       // Spi<SPI1, (PA5<Input<Floating>>,  PA6<Input<Floating>>, PA7<Input<Floating>>)>
       // which also requires  gpio::{gpioa::{PA5, PA6, PA7}, Input,  Floating, 
       // Possibly should also be able to use  'impl SpiExt<SPI1>' but no luck yet.

       let cp = cortex_m::Peripherals::take().unwrap();
       let p         = Peripherals::take().unwrap();
       let mut rcc   = p.RCC.freeze(rcc::Config::hsi());

       let gpioa = p.GPIOA.split();
       let gpiob = p.GPIOB.split();

       let spi = p.SPI1.spi(
                          (gpioa.pa5,            // sck   on PA5  in board on Heltec
                           gpioa.pa6,            // miso  on PA6  in board on Heltec
                           gpioa.pa7             // mosi  on PA7  in board on Heltec
                           ), 
                          MODE, 
                          8.mhz(), 
                          &mut rcc
                          );
        
                     
       let delay = cp.SYST.delay(rcc.clocks);
       
       // Create lora radio instance 

       let lora = Sx127x::spi(
    	    spi,				                     //Spi
    	    gpioa.pa4.into_push_pull_output(),                       //CsPin         on PA4  in board on Heltec
    	    gpiob.pb11.into_floating_input(),                        //BusyPin  DIO0 on PB11 in board on Heltec
            gpiob.pb10.into_floating_input(),                        //ReadyPin DIO1 on PB10 in board on Heltec
    	    gpioa.pa3.into_push_pull_output(),                       //ResetPin      on PA3  in board on Heltec
    	    delay,					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error
       
       lora                                            
       }



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{prelude::*,  
                    pac::Peripherals, 
                    spi::{Spi, Error,},
                    delay::Delay,
                    gpio::{gpioa::{PA5, PA6, PA7}, Alternate, AF5, Input, Floating, 
                           gpioa::{PA0, PA1}, Output, PushPull,
			   gpiob::{PB8, PB9},},
                    pac::SPI1,
                    }; 

    #[cfg(feature = "stm32l4xx")]
    fn setup() -> Sx127x<Wrapper<Spi<SPI1, (PA5<Alternate<AF5, Input<Floating>>>, 
		                            PA6<Alternate<AF5, Input<Floating>>>, 
					    PA7<Alternate<AF5, Input<Floating>>> )>, Error, 
                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
                   core::convert::Infallible,  Delay>,  Error, core::convert::Infallible> {

       let cp        = cortex_m::Peripherals::take().unwrap();
       let p         = Peripherals::take().unwrap();
       let mut flash = p.FLASH.constrain();
       let mut rcc   = p.RCC.constrain();
       let mut pwr   = p.PWR.constrain(&mut rcc.apb1r1);
       let clocks    = rcc.cfgr .sysclk(80.mhz()) .pclk1(80.mhz()) 
                                .pclk2(80.mhz()) .freeze(&mut flash.acr, &mut pwr);
      
       let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
       let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

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
    	    spi,					             //Spi
    	    gpioa.pa1.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper),    //CsPin	     on PA1
    	    gpiob.pb8.into_floating_input(  &mut gpiob.moder, &mut gpiob.pupdr),     //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(  &mut gpiob.moder, &mut gpiob.pupdr),     //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper),    //ResetPin      on PA0
    	    delay,					                             //Delay
    	    &CONFIG_RADIO,					                     //&Config
    	    ).unwrap();      // should handle error
       
       lora
       }


// End of hal/MCU specific setup. Following should be generic code.

fn to_str( x:&[u8] ) -> &str {
   match core::str::from_utf8(x) {
      Ok(str)     => &str,
      Err(_error) => "problem converting u8 to str ",
   }
}

#[entry]
fn main() -> !{

    let mut lora =  setup();         //delay is available in lora.delay_ms()
    
    lora.start_receive().unwrap();   // should handle error

    let mut buff = [0u8; 1024];
    let mut n: usize ;
    let mut info = PacketInfo::default();

    loop {

       let poll = lora.check_receive(false);    
       // false (the restart option) specifies whether transient timeout or CRC errors should be
       // internally handled (returning Ok(false) or passed back to the caller as errors.

       match poll {
            Ok(v)  if v  =>  {n = lora.get_received(&mut info, &mut buff).unwrap();
                              //hprintln!("RX complete ({:?}, length: {})", info, n).unwrap();
                              //hprintln!("{:?}", &buff[..n]).unwrap();
			      // for some reason the next prints twice?
                              hprintln!("{}", to_str(&buff[..n])).unwrap()
                              },

            Ok(_v)       =>  (),  // hprint!(".").unwrap(),   // print "." if nothing received
            
	    Err(err)     =>  hprintln!("poll error {:?} ", err).unwrap(),
            };

       lora.delay_ms(100u32);
       };

}
