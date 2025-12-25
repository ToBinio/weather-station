use std::sync::Mutex;

use bosch_bme680::{Bme680, DeviceAddress};
use embedded_hal_bus::i2c;
use esp_idf_hal::delay::Delay;
use esp_idf_hal::i2c::I2cConfig;
use esp_idf_hal::units::*;
use esp_idf_hal::{i2c::I2cDriver, prelude::Peripherals};

//TODO
const LCD_ADDRESS: u8 = 0x27;

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");

    let peripherals = Peripherals::take().unwrap();

    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
        &I2cConfig::new().baudrate(100.kHz().into()),
    )
    .unwrap();
    let i2c_mutex = Mutex::new(i2c);

    let mut bme = Bme680::new(
        i2c::MutexDevice::new(&i2c_mutex),
        DeviceAddress::Primary,
        Delay::new_default(),
        &bosch_bme680::Configuration::default(),
        20,
    )
    .unwrap();

    let data = bme.measure().unwrap();
    log::info!("Got data: {:?}", data);

    let mut i2c_lcd = i2c::MutexDevice::new(&i2c_mutex);
    let mut i2c_delay = Delay::new_default();
    let mut lcd = lcd_lcm1602_i2c::sync_lcd::Lcd::new(&mut i2c_lcd, &mut i2c_delay)
        .with_address(LCD_ADDRESS)
        .with_cursor_on(false)
        .with_rows(2)
        .init()
        .unwrap();

    lcd.write_str("Hello, world!").unwrap();
    log::info!("Wrote to lcd");
}
