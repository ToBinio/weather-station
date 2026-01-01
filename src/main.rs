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
use alloc::string::ToString;
use bosch_bme680::{Bme680, DeviceAddress};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::i2c::RcDevice;
use esp_backtrace as _;
use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::timer::timg::TimerGroup;
use lcd_lcm1602_i2c::sync_lcd::Lcd;
use scd4x::Scd4x;
use static_cell::StaticCell;

extern crate alloc;

//TODO: make full async - https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/shared_bus.rs

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Debug)]
pub enum Events {
    BME680 {
        // Temperature in Celsius
        temp: f32,
        // Relative humidity in %
        humidity: f32,
        // Pressure in Pa
        pressure: f32,
        // Gas resistance in Ohms
        gas_resistance: f32,
    },
    SCD41 {
        // CO2 concentration in ppm
        co2: u16,
        // Temperature in Celsius
        temperature: f32,
        // Relative humidity in %
        humidity: f32,
    },
    ButtonPress,
}

static CHANNEL: StaticCell<Channel<NoopRawMutex, Events, 1>> = StaticCell::new();

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

    esp_println::println!("RTOS initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    esp_println::println!("WiFi initialized!");

    let button_pin = Input::new(
        peripherals.GPIO23,
        InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );

    let i2c_v33 = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_scl(peripherals.GPIO22)
        .with_sda(peripherals.GPIO21);
    let shared_i2c_v33 = Rc::new(RefCell::new(i2c_v33));

    let i2c_v5 = I2c::new(peripherals.I2C1, I2cConfig::default())
        .unwrap()
        .with_scl(peripherals.GPIO26)
        .with_sda(peripherals.GPIO25);

    let event_channel = CHANNEL.init(Channel::new());

    spawner
        .spawn(read_bme_680(
            RcDevice::new(shared_i2c_v33.clone()),
            event_channel.sender(),
        ))
        .unwrap();
    spawner
        .spawn(read_scd_41(
            RcDevice::new(shared_i2c_v33.clone()),
            event_channel.sender(),
        ))
        .unwrap();
    spawner
        .spawn(listen_button(button_pin, event_channel.sender()))
        .unwrap();
    spawner
        .spawn(display_data(i2c_v5, event_channel.receiver()))
        .unwrap();

    esp_println::println!("Tasks initialized!");
}

#[embassy_executor::task]
async fn read_bme_680(
    i2c: RcDevice<I2c<'static, Blocking>>,
    sender: Sender<'static, NoopRawMutex, Events, 1>,
) {
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
        sender
            .send(Events::BME680 {
                temp: data.temperature,
                humidity: data.humidity,
                pressure: data.pressure,
                gas_resistance: data.gas_resistance.unwrap_or(0.),
            })
            .await;
    }
}

#[embassy_executor::task]
async fn read_scd_41(
    i2c: RcDevice<I2c<'static, Blocking>>,
    sender: Sender<'static, NoopRawMutex, Events, 1>,
) {
    let mut sensor = Scd4x::new(i2c, Delay);
    sensor.wake_up();
    sensor.stop_periodic_measurement().unwrap();
    sensor.reinit().unwrap();

    sensor.start_periodic_measurement().unwrap();
    loop {
        Timer::after(Duration::from_secs(5)).await;

        let data = sensor.measurement().unwrap();
        sender
            .send(Events::SCD41 {
                co2: data.co2,
                temperature: data.temperature,
                humidity: data.humidity,
            })
            .await;
    }
}

#[embassy_executor::task]
async fn listen_button(mut pin: Input<'static>, sender: Sender<'static, NoopRawMutex, Events, 1>) {
    loop {
        pin.wait_for_high().await;
        pin.wait_for_low().await;

        sender.send(Events::ButtonPress).await;
    }
}

struct Data {
    // Temperature in Celsius
    temp: f32,
    // Relative Humidity in percentage
    humidity: f32,
    // Pressure in hPa
    pressure: f32,
    // Gas Resistance in Ohms
    gas_resistance: f32,
    // CO2 concentration in ppm
    co2: u16,
}

enum DisplayMode {
    Temperature,
    Humidity,
    Pressure,
    GasResistance,
    CO2,
}

impl DisplayMode {
    fn display(&self, data: &Data, lcd: &mut Lcd<'_, I2c<'static, Blocking>, Delay>) {
        match self {
            DisplayMode::Temperature => {
                lcd.clear().unwrap();
                lcd.write_str("Temperature").unwrap();
                lcd.set_cursor(1, 0).unwrap();
                lcd.write_str(&data.temp.to_string()).unwrap();
                lcd.write_str("C").unwrap();
            }
            DisplayMode::Humidity => {
                lcd.clear().unwrap();
                lcd.write_str("Humidity").unwrap();
                lcd.set_cursor(1, 0).unwrap();
                lcd.write_str(&data.humidity.to_string()).unwrap();
                lcd.write_str("%").unwrap();
            }
            DisplayMode::Pressure => {
                lcd.clear().unwrap();
                lcd.write_str("Pressure").unwrap();
                lcd.set_cursor(1, 0).unwrap();
                lcd.write_str(&data.pressure.to_string()).unwrap();
                lcd.write_str("hPa").unwrap();
            }
            DisplayMode::GasResistance => {
                lcd.clear().unwrap();
                lcd.write_str("Gas Resistance").unwrap();
                lcd.set_cursor(1, 0).unwrap();
                lcd.write_str(&data.gas_resistance.to_string()).unwrap();
                lcd.write_str("Ohm").unwrap();
            }
            DisplayMode::CO2 => {
                lcd.clear().unwrap();
                lcd.write_str("CO2").unwrap();
                lcd.set_cursor(1, 0).unwrap();
                lcd.write_str(&data.co2.to_string()).unwrap();
                lcd.write_str("ppm").unwrap();
            }
        }
    }

    fn next(self) -> DisplayMode {
        match self {
            DisplayMode::Temperature => DisplayMode::Humidity,
            DisplayMode::Humidity => DisplayMode::Pressure,
            DisplayMode::Pressure => DisplayMode::GasResistance,
            DisplayMode::GasResistance => DisplayMode::CO2,
            DisplayMode::CO2 => DisplayMode::Temperature,
        }
    }
}

#[embassy_executor::task]
async fn display_data(
    mut i2c: I2c<'static, Blocking>,
    receiver: Receiver<'static, NoopRawMutex, Events, 1>,
) {
    let mut delay = Delay;
    let mut lcd = Lcd::new(&mut i2c, &mut delay)
        .with_address(0x27)
        .with_cursor_on(false)
        .with_rows(2)
        .init()
        .unwrap();

    let mut data = Data {
        temp: 0.0,
        humidity: 0.0,
        pressure: 0.0,
        gas_resistance: 0.0,
        co2: 0,
    };
    let mut mode = DisplayMode::Temperature;

    loop {
        match receiver.receive().await {
            Events::BME680 {
                temp,
                humidity,
                pressure,
                gas_resistance,
            } => {
                data.temp = temp;
                data.humidity = humidity;
                data.pressure = pressure / 100.0;
                data.gas_resistance = gas_resistance;
            }
            Events::SCD41 { co2, .. } => {
                data.co2 = co2;
            }
            Events::ButtonPress => {
                mode = mode.next();
            }
        }

        mode.display(&data, &mut lcd);
    }
}
