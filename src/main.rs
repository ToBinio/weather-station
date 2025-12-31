#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::cell::RefCell;

use alloc::rc::Rc;
use bosch_bme680::{Bme680, DeviceAddress};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::i2c::RcDevice;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{Async, Blocking};
use scd4x::Scd4x;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // generator version: 1.1.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    esp_println::println!("Embassy initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let pin = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());

    let i2c_v33 = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_scl(peripherals.GPIO22)
        .with_sda(peripherals.GPIO21);
    let shared_i2c_v33 = Rc::new(RefCell::new(i2c_v33));

    spawner.spawn(flash_led(pin)).unwrap();
    spawner
        .spawn(read_bme_680(RcDevice::new(shared_i2c_v33.clone())))
        .unwrap();
    spawner
        .spawn(read_scd_41(RcDevice::new(shared_i2c_v33.clone())))
        .unwrap();
}

#[embassy_executor::task]
async fn flash_led(mut pin: Output<'static>) {
    loop {
        pin.toggle();
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn read_bme_680(i2c: RcDevice<I2c<'static, Blocking>>) {
    let mut sensor = Bme680::new(
        i2c,
        DeviceAddress::Primary,
        Delay,
        &bosch_bme680::Configuration::default(),
        20,
    )
    .unwrap();

    loop {
        Timer::after(Duration::from_secs(5)).await;

        let data = sensor.measure().unwrap();
        esp_println::println!("Got data: {:?}", data);
    }
}

#[embassy_executor::task]
async fn read_scd_41(i2c: RcDevice<I2c<'static, Blocking>>) {
    let mut sensor = Scd4x::new(i2c, Delay);
    sensor.wake_up();
    sensor.stop_periodic_measurement().unwrap();
    sensor.reinit().unwrap();

    let serial = sensor.serial_number().unwrap();
    esp_println::println!("serial: {serial:#04x}");

    sensor.start_periodic_measurement().unwrap();
    esp_println::println!("Waiting for first measurement... (5 sec)");

    loop {
        Timer::after(Duration::from_secs(5)).await;

        let data = sensor.measurement().unwrap();
        esp_println::println!(
            "CO2: {}, Temperature: {} °C, Humidity: {} RH",
            data.co2,
            data.temperature,
            data.humidity
        );
    }
}
