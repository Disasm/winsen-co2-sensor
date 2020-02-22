use linux_embedded_hal::Serial;
use winsen_co2_sensor::WinsenSensor;
use std::time::Instant;

fn main() {
    let serial_path = std::env::args().nth(1).unwrap_or("/dev/ttyUSB0".into());
    let serial = Serial::open(serial_path).unwrap();
    let mut sensor = WinsenSensor::new(serial, Instant::now());

    let probe_ok = sensor.probe().unwrap();
    println!("Probe: {:?}", probe_ok);
    if !probe_ok {
        return;
    }

    println!("CO2: {}", sensor.read_co2_concentration().unwrap());
    println!("detection range: {}", sensor.get_detection_range().unwrap());
    println!("analog bounds: {:?}", sensor.get_analog_bounds().unwrap());
    println!("fw version: {:?}", sensor.get_firmware_version().unwrap());
}
