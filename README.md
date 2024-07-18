# BH1750 driver
[![Crates.io](https://img.shields.io/crates/d/bh1750.svg)](https://crates.io/crates/bh1750)
[![Crates.io](https://img.shields.io/crates/v/bh1750.svg)](https://crates.io/crates/bh1750)
[![Released API docs](https://docs.rs/bh1750/badge.svg)](https://docs.rs/bh1750)

A platform-agnostic, 'no_std' compatible Rust driver for the BH1750 ambient light sensor using the `embedded-hal` traits.

The I²C instruction set is based on the following datasheet: [BH1750 datasheet](https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf) \
All instructions are implemented and supported.

The raw values read from the sensor are converted to lux, taking into account the resolution mode and measurement time register value.

## Usage
To use this driver, import it and an `embedded_hal` implementation, then create an instance of the driver.

You can call the `get_one_time_measurement` function to get a single measurement from the sensor.

Alternatively, you can call `start_continuous_measurement` to start continuous measurements and then call `get_current_measurement` to get the latest measurement.

## Example
This example uses the `esp-hal` crate to interface with the sensor on an ESP32 microcontroller.

```rust no_run
#![no_std]
#![no_main]

use esp_backtrace as _;
use bh1750::{BH1750, Resolution};
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    gpio::Io
};
use esp_hal::i2c::I2C;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio8,
        io.pins.gpio9,
        100.kHz(),
        &clocks,
        None
    );

    let mut bh1750 = BH1750::new(i2c, delay, false);

    loop {
        let lux = bh1750.get_one_time_measurement(Resolution::High)
            .expect("Failed to read BH1750");

        log::info!("Lux = {:.2}", lux);

        delay.delay(1000.millis());
    }
}
```
