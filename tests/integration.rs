//! Sx127x Integration testing
//!
//! Copyright 2019 Ryan Kurte

use std::collections::HashMap;
use std::thread;
use std::time::Duration;

extern crate driver_pal;
use driver_pal::utils::{load_config, DeviceConfig};

extern crate radio_sx127x;
use radio_sx127x::prelude::*;

extern crate radio;
use radio::{Receive, Transmit};

#[test]
#[ignore]
fn integration() {
    // Fetch configuration file name
    let config_file = match std::env::var("SX127x_TEST_CONFIG") {
        Ok(v) => v,
        Err(_e) => "configs/pi-ci-sx127x.toml".to_owned(),
    };

    println!("Using configuration file: {}", config_file);

    // Load configurations from file
    let configs = load_config::<HashMap<String, DeviceConfig>>(&config_file);

    let config1 = configs.get("radio-0").expect("Missing radio-0 object");
    let config2 = configs.get("radio-1").expect("Missing radio-1 object");

    let (w1, w2) = (config1.load(), config2.load());

    let mut radio1 = Sx127x::new(w1, &Config::default()).expect("error creating radio1");
    let mut radio2 = Sx127x::new(w2, &Config::default()).expect("error creating radio1");

    println!("Testing send/receive");

    let data = &[0xaa, 0xbb, 0xcc];

    // Configure receive
    radio1.start_receive().unwrap();

    // Start transmit
    radio2.start_transmit(data).unwrap();

    // Poll on tx and rx complete
    let mut sent = false;
    let mut received = false;
    let mut buff = [0u8; 1024];
    let mut n = 0;
    let mut info = PacketInfo::default();

    for _i in 0..10 {
        // Check TX state
        if radio2.check_transmit().unwrap() {
            println!("TX complete");
            sent = true;
        }

        // Check RX state
        if radio1.check_receive(false).unwrap() {
            n = radio1.get_received(&mut info, &mut buff).unwrap();
            received = true;
            println!("RX complete ({:?} {:?})", info, &buff[..n]);
        }

        thread::sleep(Duration::from_millis(100));
    }

    assert!(sent, "Send not completed");
    assert!(received, "Receive not completed");
    assert_eq!(data, &buff[..n]);
}
