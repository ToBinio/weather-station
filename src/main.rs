#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::string::ToString;
use bosch_bme680::{AsyncBme680, DeviceAddress};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use esp_backtrace as _;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::peripherals::Peripherals;
use esp_hal::timer::timg::TimerGroup;
use lcd_lcm1602_i2c::async_lcd::Lcd;
use scd4x::Scd4xAsync;
use static_cell::StaticCell;
use weather_station::ha::{HaSensors, init_ha};
use weather_station::wifi::init_wifi;

extern crate alloc;

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

type I2c1BusV33 = Mutex<NoopRawMutex, I2c<'static, Async>>;

type EventChannel = Channel<NoopRawMutex, Events, 1>;
type EventSender = Sender<'static, NoopRawMutex, Events, 1>;
type EventReceiver = Receiver<'static, NoopRawMutex, Events, 1>;

fn init_hardware() -> Peripherals {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    peripherals
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = init_hardware();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let stack = init_wifi(peripherals.WIFI, &spawner).await;
    esp_println::println!("WiFi initialized!");

    let ha_sensors = init_ha(stack, &spawner);

    let button_pin = Input::new(
        peripherals.GPIO23,
        InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );

    let i2c_v33 = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_scl(peripherals.GPIO22)
        .with_sda(peripherals.GPIO21)
        .into_async();
    static I2C_V33_BUS: StaticCell<I2c1BusV33> = StaticCell::new();
    let i2c_v33 = I2C_V33_BUS.init(Mutex::new(i2c_v33));

    let i2c_v5 = I2c::new(peripherals.I2C1, I2cConfig::default())
        .unwrap()
        .with_scl(peripherals.GPIO26)
        .with_sda(peripherals.GPIO25)
        .into_async();

    static CHANNEL: StaticCell<EventChannel> = StaticCell::new();
    let event_channel = CHANNEL.init(Channel::new());

    spawner
        .spawn(read_bme_680(i2c_v33, event_channel.sender()))
        .unwrap();
    spawner
        .spawn(read_scd_41(i2c_v33, event_channel.sender()))
        .unwrap();
    spawner
        .spawn(listen_button(button_pin, event_channel.sender()))
        .unwrap();
    spawner
        .spawn(process_data(i2c_v5, ha_sensors, event_channel.receiver()))
        .unwrap();

    esp_println::println!("Tasks initialized!");
}

#[embassy_executor::task]
async fn read_bme_680(i2c_bus: &'static I2c1BusV33, sender: EventSender) {
    let i2c = I2cDevice::new(i2c_bus);
    let mut sensor = AsyncBme680::new(i2c, DeviceAddress::Primary, Delay, 20);
    sensor
        .initialize(&bosch_bme680::Configuration::default())
        .await
        .unwrap();

    loop {
        let data = sensor.measure().await.unwrap();
        sender
            .send(Events::BME680 {
                temp: data.temperature,
                humidity: data.humidity,
                pressure: data.pressure,
                gas_resistance: data.gas_resistance.unwrap_or(0.),
            })
            .await;

        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn read_scd_41(i2c_bus: &'static I2c1BusV33, sender: EventSender) {
    let i2c = I2cDevice::new(i2c_bus);
    let mut sensor = Scd4xAsync::new(i2c, Delay);
    sensor.wake_up().await;
    sensor.stop_periodic_measurement().await.unwrap();
    sensor.reinit().await.unwrap();

    sensor.start_periodic_measurement().await.unwrap();
    loop {
        Timer::after(Duration::from_secs(60)).await;

        let data = sensor.measurement().await.unwrap();
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
async fn listen_button(mut pin: Input<'static>, sender: EventSender) {
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
    async fn display(&self, data: &Data, lcd: &mut Lcd<'_, I2c<'static, Async>, Delay>) {
        let (header, value, unit) = match self {
            DisplayMode::Temperature => ("Temperature", &data.temp.to_string(), "C"),
            DisplayMode::Humidity => ("Humidity", &data.humidity.to_string(), "%"),
            DisplayMode::Pressure => ("Pressure", &data.pressure.to_string(), "hPa"),
            DisplayMode::CO2 => ("CO2", &data.co2.to_string(), "ppm"),
            DisplayMode::GasResistance => {
                ("Gas Resistance", &data.gas_resistance.to_string(), "Ohm")
            }
        };

        lcd.clear().await.unwrap();
        lcd.write_str(header).await.unwrap();
        lcd.set_cursor(1, 0).await.unwrap();
        lcd.write_str(value).await.unwrap();
        lcd.write_str(unit).await.unwrap();
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
async fn process_data(
    mut i2c: I2c<'static, Async>,
    mut ha_sensors: HaSensors,
    receiver: EventReceiver,
) {
    let mut data = Data {
        temp: 0.0,
        humidity: 0.0,
        pressure: 0.0,
        gas_resistance: 0.0,
        co2: 0,
    };
    let mut mode = DisplayMode::Temperature;

    let mut delay = Delay;
    let mut lcd = Lcd::new(&mut i2c, &mut delay)
        .with_address(0x27)
        .with_cursor_on(false)
        .with_rows(2)
        .init()
        .await
        .unwrap();

    mode.display(&data, &mut lcd).await;

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

                ha_sensors.temperature.publish(data.temp);
                ha_sensors.humidity.publish(data.humidity);
                ha_sensors.pressure.publish(data.pressure);
                ha_sensors.gas_resistance.publish(data.gas_resistance);
            }
            Events::SCD41 { co2, .. } => {
                data.co2 = co2;

                ha_sensors.co2.publish(data.co2 as f32);
            }
            Events::ButtonPress => {
                mode = mode.next();
            }
        }

        mode.display(&data, &mut lcd).await;
    }
}
