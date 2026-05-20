use std::time::Duration;
use unilidar_sdk2_cxx::{UdpConfig, UnilidarL2};

/// Switch the lidar's persistent transport mode from UDP to serial.
///
/// The lidar must currently be reachable over UDP/Ethernet. After this runs,
/// the serial port (`/dev/ttyACM0`) will emit data and `serial_visualize` works.
/// To go back, run the inverse over serial with `set_lidar_work_mode(0)`.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut lidar = UnilidarL2::new();
    lidar.initialize_udp(UdpConfig::default())?;
    println!("connected over UDP");
    std::thread::sleep(Duration::from_secs(1));
    lidar.start_lidar_rotation();

    println!("setting work mode to 8 (serial)");
    lidar.set_lidar_work_mode(8);
    std::thread::sleep(Duration::from_secs(1));

    println!("resetting lidar to apply mode");
    lidar.reset_lidar();
    std::thread::sleep(Duration::from_secs(2));

    println!("done — lidar is now in serial mode");
    Ok(())
}
