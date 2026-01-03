use embassy_executor::Spawner;
use embassy_net::Stack;
use static_cell::StaticCell;

pub struct HaSensors {
    pub temperature: embassy_ha::Sensor<'static>,
    pub humidity: embassy_ha::Sensor<'static>,
    pub pressure: embassy_ha::Sensor<'static>,
    pub gas_resistance: embassy_ha::Sensor<'static>,
    pub co2: embassy_ha::Sensor<'static>,
}

pub fn init_ha(stack: Stack<'static>, spawner: &Spawner) -> HaSensors {
    static HA_RESOURCES: StaticCell<embassy_ha::DeviceResources> = StaticCell::new();
    let ha_device = embassy_ha::new(
        HA_RESOURCES.init(Default::default()),
        embassy_ha::DeviceConfig {
            device_id: "indoor-weather-station",
            device_name: "Indoor Weather Station",
            manufacturer: "Custom",
            model: "Indoor Weather Station",
        },
    );

    let temperature = embassy_ha::create_sensor(
        &ha_device,
        "temp-sensor",
        embassy_ha::SensorConfig {
            class: embassy_ha::SensorClass::Temperature,
            state_class: embassy_ha::StateClass::Measurement,
            unit: Some(embassy_ha::constants::HA_UNIT_TEMPERATURE_CELSIUS),
            ..Default::default()
        },
    );

    let humidity = embassy_ha::create_sensor(
        &ha_device,
        "humidity-sensor",
        embassy_ha::SensorConfig {
            class: embassy_ha::SensorClass::Humidity,
            state_class: embassy_ha::StateClass::Measurement,
            unit: Some(embassy_ha::constants::HA_UNIT_PERCENTAGE),
            ..Default::default()
        },
    );

    let pressure = embassy_ha::create_sensor(
        &ha_device,
        "pressure-sensor",
        embassy_ha::SensorConfig {
            class: embassy_ha::SensorClass::Pressure,
            state_class: embassy_ha::StateClass::Measurement,
            unit: Some(embassy_ha::constants::HA_UNIT_PRESSURE_HPA),
            ..Default::default()
        },
    );

    let gas_resistance = embassy_ha::create_sensor(
        &ha_device,
        "gas-resistance-sensor",
        embassy_ha::SensorConfig {
            common: embassy_ha::EntityCommonConfig {
                name: Some("Gas Resistance"),
                ..Default::default()
            },
            class: embassy_ha::SensorClass::Generic,
            state_class: embassy_ha::StateClass::Measurement,
            unit: Some("Ω"),
            ..Default::default()
        },
    );

    let co2 = embassy_ha::create_sensor(
        &ha_device,
        "co2-sensor",
        embassy_ha::SensorConfig {
            class: embassy_ha::SensorClass::CarbonDioxide,
            state_class: embassy_ha::StateClass::Measurement,
            unit: Some("ppm"),
            ..Default::default()
        },
    );

    spawner.spawn(ha_task(stack, ha_device)).unwrap();

    HaSensors {
        temperature,
        humidity,
        pressure,
        co2,
        gas_resistance,
    }
}

#[embassy_executor::task]
async fn ha_task(stack: embassy_net::Stack<'static>, device: embassy_ha::Device<'static>) {
    const MQTT_ADDRESS: &str = env!("MQTT_ADDRESS");
    embassy_ha::connect_and_run(stack, device, MQTT_ADDRESS).await;
}
