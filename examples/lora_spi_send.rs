//! Transmit a simple message with LoRa using crate radio_sx127x (on SPI).
//! 
//!  The largest part of this file is the setup() functions used for each hal.
//!  These make the application code common.
//!
//!  In the following  
//!      - replace xxx with the example name  lora_spi_send, lora_spi_receive, or lora_spi_gps.
//!      - set TARGET, HAL, and MCU from a line in the table below (linux syntax).
//!      - no-default-features is because some default-features require std.
//!
//!  The examples can be compiled with:
//!    cargo build  --no-default-features  --target $TARGET --features=$HAL,$MCU  --example xxx
//!
//!  Before running, check  FREQUENCY below to be sure you have a channel setting appropriate for
//!  your country, hardware and any testing sender/receiver on the other end of the communication.
//!
//!  To link, loaded and run using gdb and openocd (with INTERFACE and PROC set as below):
//!  	openocd -f interface/$INTERFACE.cfg -f target/$PROC.cfg
//!  and in another window (with TARGET, HAL, and MCU set as below):
//!  	cargo  run  --no-default-features  --target $TARGET --features $HAL,$MCU    --example xxx   [ --release ]
//! 
//!  If --release is omitted then some MCUs do not have sufficient memory and loading results in
//!       '.rodata will not fit in region FLASH '
//!  Even with sufficient memory the code without --release is slower and may result in errors.
//!
//!
//!                cargo run  environment variables                        openocd        test board and processor
//!    _____________________________________________________________     _____________   ___________________________
//!    export HAL=stm32f0xx MCU=stm32f042   TARGET=thumbv6m-none-eabi	 PROC=stm32f0x  # none-stm32f042      Cortex-M0
//!    export HAL=stm32f0xx MCU=stm32f030xc TARGET=thumbv6m-none-eabi	 PROC=stm32f0x  # none-stm32f030      Cortex-M0
//!    export HAL=stm32f1xx MCU=stm32f103   TARGET=thumbv7m-none-eabi	 PROC=stm32f1x  # bluepill	      Cortex-M3
//!    export HAL=stm32f1xx MCU=stm32f100   TARGET=thumbv7m-none-eabi	 PROC=stm32f1x  # none-stm32f100      Cortex-M3
//!    export HAL=stm32f1xx MCU=stm32f101   TARGET=thumbv7m-none-eabi	 PROC=stm32f1x  # none-stm32f101      Cortex-M3
//!    export HAL=stm32f3xx MCU=stm32f303xc TARGET=thumbv7em-none-eabihf PROC=stm32f3x  # discovery-stm32f303 Cortex-M3
//!    export HAL=stm32f4xx MCU=stm32f401   TARGET=thumbv7em-none-eabihf PROC=stm32f4x  # blackpill-stm32f401 Cortex-M4
//!    export HAL=stm32f4xx MCU=stm32f411   TARGET=thumbv7em-none-eabihf PROC=stm32f4x  # blackpill-stm32f411 Cortex-M4
//!    export HAL=stm32f4xx MCU=stm32f411   TARGET=thumbv7em-none-eabihf PROC=stm32f4x  # nucleo-64	      Cortex-M4
//!    hal NOT compiling as of (Feb 2021) export HAL=stm32f7xx MCU=stm32f722 TARGET=thumbv7em-none-eabihf #none-stm32f722 Cortex-M7
//!    export HAL=stm32h7xx MCU=stm32h742   TARGET=thumbv7em-none-eabihf                # none-stm32h742      Cortex-M7
//!    export HAL=stm32l0xx MCU=stm32l0x2   TARGET=thumbv6m-none-eabi	 PROC=stm32l1   # none-stm32l0x2      Cortex-M0
//!    export HAL=stm32l1xx MCU=stm32l100   TARGET=thumbv7m-none-eabi	 PROC=stm32l1   # discovery-stm32l100 Cortex-M3
//!    export HAL=stm32l1xx MCU=stm32l151   TARGET=thumbv7m-none-eabi	 PROC=stm32l1   # heltec-lora-node151 Cortex-M3
//!    NOT compiling as of (Feb 2021) export HAL=stm32l4xx MCU=stm32l4x2   TARGET=thumbv7em-none-eabi # none-stm32l4x1      Cortex-M4
//!  
//!  Depending on the MCU connection to the computer, in the  openocd command use
//!    export INTERFACE=stlink-v2  
//!    export INTERFACE=stlink-v2-1  

//! A version of this example is reported at https://pdgilbert.github.io/eg_stm_hal/.
//! The results reported there use current git versions of the hals, whereas
//! the example here uses release versions of the hals (as of Feb 2021).


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

use embedded_hal_compat::IntoCompat;
use embedded_hal_compat::eh1_0::blocking::delay::{DelayMs as _};

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

const FREQUENCY: u32 = 907_400_000;     // frequency in hertz ch_12: 915_000_000, ch_2: 907_400_000

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


// setup() does all  hal/MCU specific setup and returns generic object for use in main code.

#[cfg(feature = "stm32f0xx")]  //  eg stm32f030xc
use stm32f0xx_hal::{prelude::*,   
                    pac::Peripherals, 
                    spi::{Spi, Error, },
                    delay::Delay,
                    }; 

    #[cfg(feature = "stm32f0xx")]
    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, core::convert::Infallible>> {

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
                    spi::{Spi,  Error,},
                    delay::Delay,
                    }; 

    #[cfg(feature = "stm32f1xx")]
    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, core::convert::Infallible>> {

    //fn setup() ->  Sx127x<Wrapper<Spi<SPI1, Spi1NoRemap,
    //                    (PA5<Alternate<PushPull>>,  PA6<Input<Floating>>, PA7<Alternate<PushPull>>), u8>, Error, 
    //               PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
    //               core::convert::Infallible,  Delay>, Error, core::convert::Infallible> {

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
    	    spi.compat(),					             //Spi
    	    gpioa.pa1.into_push_pull_output(&mut gpioa.crl),         //CsPin         on PA1
    	    gpiob.pb8.into_floating_input(&mut gpiob.crh),           //BusyPin  DIO0 on PB8
            gpiob.pb9.into_floating_input(&mut gpiob.crh),           //ReadyPin DIO1 on PB9
    	    gpioa.pa0.into_push_pull_output(&mut gpioa.crl),         //ResetPin      on PA0
    	    delay.compat(),					             //Delay
    	    &CONFIG_RADIO,					     //&Config
    	    ).unwrap();      // should handle error

       lora
       }


#[cfg(feature = "stm32f3xx")]  //  eg Discovery-stm32f303
use stm32f3xx_hal::{prelude::*, 
                    stm32::Peripherals,
                    spi::{Spi, Error},
                    delay::Delay,
                    };

    #[cfg(feature = "stm32f3xx")]
    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, core::convert::Infallible>> {

    //fn setup() ->  Sx127x<Wrapper<Spi<SPI1, 
    //                       (PA5<AF5>,    PA6<AF5>,   PA7<AF5>)>,  Error, 
    //               PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
    //               core::convert::Infallible,  Delay>,  Error, core::convert::Infallible> {
          
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
                    stm32::Peripherals, 
                    spi::{Spi, Error},
                    delay::Delay,
                    //  next would be needed to define exact type 
		    //gpio::{gpioa::{PA5, PA6, PA7}, Alternate, AF5,  
                    //       gpioa::{PA0, PA1}, Output, PushPull,
		    //       gpiob::{PB8, PB9}, Input, Floating},
                    time::MegaHertz,
                    //pac::SPI1,
                    }; 

// If the type for the lora object is needed somewhere other than just in the setup() return type then it
// may be better to explicitly define it as follows. This also require commented lines in use above.
//
//    use embedded_spi::wrapper::Wrapper;
//
//    type LoraType = Sx127x<Wrapper<Spi<SPI1, 
//                           (PA5<Alternate<AF5>>,    PA6<Alternate<AF5>>,   PA7<Alternate<AF5>>)>,  Error, 
//                   PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
//                   core::convert::Infallible,  Delay>,  Error, core::convert::Infallible>;
// then
//    fn setup() ->  LoraType {

//If access to delay in the returned object is not needed ( eg. lora.delay_ms(5000u32); ) then 
//    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, core::convert::Infallible>> {
// works.

    #[cfg(feature = "stm32f4xx")]
    fn setup() ->  impl  DelayMs<u32> + Transmit<Error=sx127xError<Error, core::convert::Infallible>> {

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
                    spi::{Spi, ClockDivider, Error,},
                    delay::Delay,
                    }; 

    #[cfg(feature = "stm32f7xx")]
    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, core::convert::Infallible>> {
    
    //fn setup() -> Sx127x<Wrapper<Spi<SPI1,impl Pins<SPI1>, Enabled<u8>>,  Error, 
    //               PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
    //               core::convert::Infallible,  Delay>,  Error, core::convert::Infallible> {


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
                    spi::{ Error},
                    delay::Delay,
                   }; 

    #[cfg(feature = "stm32h7xx")]
    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, stm32h7xx_hal::Never>> {

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
                    spi::{Error, },
                    }; 

    //#[cfg(feature = "stm32l0xx")] 
    //use void::Void;     release version of this needs std. And it should not be in return value anyway

    #[cfg(feature = "stm32l0xx")]
    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, Void>> {

    //fn setup() -> Sx127x<Wrapper<Spi<SPI1,impl Pins<SPI1>>, Error, 
    //               PA1<Output<PushPull>>,  PB8<Input<Floating>>,  PB9<Input<Floating>>,  PA0<Output<PushPull>>, 
    //               void::Void,  Delay>,  Error, void::Void> {

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
                    spi::{Error,},
                    };

    #[cfg(feature = "stm32l1xx")]
    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, core::convert::Infallible>> {
    
    //fn setup() -> Sx127x<Wrapper<Spi<SPI1,impl Pins<SPI1>>, Error, 
    //               PA4<Output<PushPull>>,  PB11<Input<Floating>>,  PB10<Input<Floating>>,  PA3<Output<PushPull>>, 
    //               core::convert::Infallible,  Delay>,  Error, core::convert::Infallible> {

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

//  Heltec lora_node STM32L151CCU6
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
                    }; 

    #[cfg(feature = "stm32l4xx")]
    fn setup() ->  impl DelayMs<u32> + Transmit<Error=sx127xError<Error, core::convert::Infallible>> {
    
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


#[entry]
fn main() -> !{

    let mut lora =  setup();  //delay is available in lora

    
    // print out configuration (for debugging)
    
//    let v = lora.lora_get_config();
//    hprintln!("configuration {}", v).unwrap();
    
//    hprintln!("chammel	  {}", lora.get_chammel()).unwrap();

    //hprintln!("mode		  {}", lora.get_mode()).unwrap();
    //hprintln!("mode		  {}", lora.read_register(Register::RegOpMode.addr())).unwrap();
    //hprintln!("bandwidth	  {:?}", lora.get_signal_bandwidth()).unwrap();
    //hprintln!("coding_rate	  {:?}",  lora.get_coding_rate_4()).unwrap();
    //hprintln!("spreading_factor {:?}",  lora.get_spreading_factor()).unwrap();
    //hprintln!("spreading_factor {:?}",  
    //hprintln!("invert_iq	  {:?}",  lora.get_invert_iq()).unwrap();
    //hprintln!("tx_power	  {:?}",  lora.get_tx_power()).unwrap();
    
    
    
    // transmit something
      
    //let buffer = &[0xaa, 0xbb, 0xcc];
    
    let message = b"Hello, LoRa!";
    
    //let mut buffer = [0;100];      //Nov 2020 limit data.len() < 255 in radio_sx127x  .start_transmit
    //for (i,c) in message.chars().enumerate() {
    //	buffer[i] = c as u8;
    //	}    
    
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
