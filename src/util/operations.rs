use std::time::Duration;

use log::{debug, info};

use super::options::*;

// TODO: replace these with `radio::helpers`
pub fn do_command<T, I, E>(radio: T, operation: Operation) -> Result<(), E>
where
    T: radio::Transmit<Error = E>
        + radio::Power<Error = E>
        + radio::Receive<Info = I, Error = E>
        + radio::Rssi<Error = E>
        + radio::Power<Error = E>,
    I: Default + std::fmt::Debug,
    E: std::fmt::Debug,
{
    // TODO: the rest
    match operation {
        Operation::Transmit(config) => do_transmit(
            radio,
            config.data.as_bytes(),
            config.power,
            config.continuous,
            *config.period,
            *config.poll_interval,
        )
        .expect("Transmit error"),
        Operation::Receive(config) => {
            let mut buff = [0u8; 255];

            do_receive(
                radio,
                &mut buff,
                config.continuous,
                *config.poll_interval,
            )
            .expect("Receive error");
        }
        Operation::Repeat(config) => {
            let mut buff = [0u8; 255];

            do_repeat(
                radio,
                &mut buff,
                config.power,
                config.continuous,
                *config.delay,
                *config.poll_interval,
            )
            .expect("Repeat error");
        }
        Operation::Rssi(config) => {
            do_rssi(radio, config.continuous, *config.period).expect("RSSI error");
        }
        //_ => warn!("unsuppored command: {:?}", opts.command),
    }

    Ok(())
}

pub fn do_transmit<T, E>(
    mut radio: T,
    data: &[u8],
    power: i8,
    continuous: bool,
    period: Duration,
    poll_interval: Duration,
) -> Result<(), E>
where
    T: radio::Transmit<Error = E> + radio::Power<Error = E>,
{
    radio.set_power(power)?;

    loop {
        radio.start_transmit(data)?;
        loop {
            if radio.check_transmit()? {
                debug!("Send complete");
                break;
            }
            std::thread::sleep(poll_interval);
        }

        if !continuous {
            break;
        }
        std::thread::sleep(period);
    }

    Ok(())
}

pub fn do_receive<T, I, E>(
    mut radio: T,
    mut buff: &mut [u8],
    continuous: bool,
    poll_interval: Duration,
) -> Result<usize, E>
where
    T: radio::Receive<Info = I, Error = E>,
    I: Default + std::fmt::Debug,
{
    // Start receive mode
    radio.start_receive()?;

    loop {
        if radio.check_receive(true)? {
            let (n, info) = radio.get_received(&mut buff)?;

            match std::str::from_utf8(&buff[0..n as usize]) {
                Ok(s) => info!("Received: '{}' info: {:?}", s, info),
                Err(_) => info!("Received: '{:?}' info: {:?}", &buff[0..n as usize], info),
            }

            if !continuous {
                return Ok(n);
            }
        }

        std::thread::sleep(poll_interval);
    }
}

pub fn do_rssi<T, I, E>(mut radio: T, continuous: bool, period: Duration) -> Result<(), E>
where
    T: radio::Receive<Info = I, Error = E> + radio::Rssi<Error = E>,
    I: std::fmt::Debug,
{
    // Enter receive mode
    radio.start_receive()?;

    // Poll for RSSI
    loop {
        let rssi = radio.poll_rssi()?;

        info!("rssi: {}", rssi);

        radio.check_receive(true)?;

        std::thread::sleep(period);

        if !continuous {
            break;
        }
    }

    Ok(())
}

pub fn do_repeat<T, I, E>(
    mut radio: T,
    mut buff: &mut [u8],
    power: i8,
    continuous: bool,
    delay: Duration,
    poll_interval: Duration,
) -> Result<usize, E>
where
    T: radio::Receive<Info = I, Error = E> + radio::Transmit<Error = E> + radio::Power<Error = E>,
    I: Default + std::fmt::Debug,
{
    // Set TX power
    radio.set_power(power)?;

    // Start receive mode
    radio.start_receive()?;

    loop {
        if radio.check_receive(true)? {
            let (n, info) = radio.get_received(&mut buff)?;

            match std::str::from_utf8(&buff[0..n as usize]) {
                Ok(s) => info!("Received: '{}' info: {:?}", s, info),
                Err(_) => info!("Received: '{:?}' info: {:?}", &buff[0..n as usize], info),
            }

            std::thread::sleep(delay);

            radio.start_transmit(&buff[..n])?;
            loop {
                if radio.check_transmit()? {
                    debug!("Send complete");
                    break;
                }
                std::thread::sleep(poll_interval);
            }

            if !continuous {
                return Ok(n);
            }
        }

        std::thread::sleep(poll_interval);
    }
}
