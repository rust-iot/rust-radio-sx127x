
use std::collections::HashMap;

extern crate embedded_spi;
use embedded_spi::wrapper::Wrapper;
use embedded_spi::utils::{DeviceConfig, load_config};

extern crate radio_sx127x;
use radio_sx127x::{Sx127x, Settings};

#[test]
#[ignore]
fn integration() {
    // Fetch configuration file name
    let config_file = std::env::var("SX127x_TEST_CONFIG")
        .expect("Integration test conifiguratin (SX127x_TEST_CONFIG) must be specified");

    // Load configurations from file
    let configs = load_config::<HashMap<String, DeviceConfig>>(&config_file);

    let config1 = configs.get("radio-0").expect("Missing radio-0 object");
    let config2 = configs.get("radio-1").expect("Missing radio-1 object");
    
    let (w1, w2) = (config1.load(), config2.load());

    let settings1 = Settings::default();
    let mut radio1 = Sx127x::new(w1, settings1).expect("error creating radio1");

    let settings2 = Settings::default();
    let mut radio2 = Sx127x::new(w2, settings2).expect("error creating radio1");



}
