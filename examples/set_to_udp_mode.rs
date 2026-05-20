use std::env;
use std::path::Path;
use std::time::Duration;
use unilidar_sdk2_cxx::{
    SerialConfig, UdpConfig, set_lidar_udp_config_serial, set_lidar_work_mode_serial,
};

const DEFAULT_SERIAL_PORT: &str = "/dev/ttyACM0";
const LIDAR_SERIAL_BY_ID: &str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A2A026768-if00";
const UDP_WORK_MODE: u32 = 0;

/// Switch the lidar's persistent transport mode from serial back to UDP.
///
/// This writes the work-mode command directly to the serial device. The precompiled Unitree SDK
/// can open this serial adapter but does not reliably parse/control it on this host.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let config = SerialConfig::default()
        .port(serial_port(&args))
        .baudrate(serial_baudrate(&args)?);
    let udp_config = udp_config(&args);

    println!(
        "setting UDP endpoint: lidar {}:{} -> host {}:{}",
        udp_config.lidar_ip, udp_config.lidar_port, udp_config.local_ip, udp_config.local_port
    );
    set_lidar_udp_config_serial(
        SerialConfig {
            port: config.port.clone(),
            baudrate: config.baudrate,
            ..SerialConfig::default()
        },
        udp_config,
    )?;
    std::thread::sleep(Duration::from_millis(250));

    println!(
        "setting work mode to {UDP_WORK_MODE} (udp) over {} at {} baud",
        config.port, config.baudrate
    );

    for attempt in 1..=3 {
        set_lidar_work_mode_serial(
            SerialConfig {
                port: config.port.clone(),
                baudrate: config.baudrate,
                ..SerialConfig::default()
            },
            UDP_WORK_MODE,
        )?;
        println!("sent udp mode command ({attempt}/3)");
        std::thread::sleep(Duration::from_millis(250));
    }

    println!("done - power-cycle or reset the lidar if UDP packets do not appear immediately");
    Ok(())
}

fn serial_port(args: &[String]) -> String {
    if let Some(port) = arg_value(args, "--port") {
        return port;
    }

    if let Ok(port) = env::var("UNILIDAR_SERIAL_PORT") {
        return port;
    }

    if Path::new(LIDAR_SERIAL_BY_ID).exists() {
        return LIDAR_SERIAL_BY_ID.to_owned();
    }

    DEFAULT_SERIAL_PORT.to_owned()
}

fn serial_baudrate(args: &[String]) -> Result<u32, Box<dyn std::error::Error>> {
    let baudrate = arg_value(args, "--baud")
        .or_else(|| env::var("UNILIDAR_SERIAL_BAUD").ok())
        .unwrap_or_else(|| SerialConfig::default().baudrate.to_string());

    Ok(baudrate.parse()?)
}

fn udp_config(args: &[String]) -> UdpConfig {
    let mut config = UdpConfig::default();

    if let Some(lidar_ip) = arg_value(args, "--lidar-ip") {
        config.lidar_ip = lidar_ip;
    }

    if let Some(local_ip) = arg_value(args, "--local-ip") {
        config.local_ip = local_ip;
    }

    if let Some(lidar_port) = arg_value(args, "--lidar-port") {
        if let Ok(lidar_port) = lidar_port.parse() {
            config.lidar_port = lidar_port;
        }
    }

    if let Some(local_port) = arg_value(args, "--local-port") {
        if let Ok(local_port) = local_port.parse() {
            config.local_port = local_port;
        }
    }

    config
}

fn arg_value(args: &[String], name: &str) -> Option<String> {
    args.windows(2)
        .find(|pair| pair[0] == name)
        .map(|pair| pair[1].clone())
}
