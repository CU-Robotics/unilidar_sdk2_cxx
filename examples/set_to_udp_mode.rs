use std::time::Duration;
use unilidar_sdk2_cxx::{SerialConfig, UnilidarL2};

/// Switch the lidar's persistent transport mode from serial back to UDP.
///
/// The lidar must currently be reachable over serial (`/dev/ttyACM0`). After this
/// runs, the lidar emits data over UDP/Ethernet and `udp_visualize` works again.
/// To go the other way, run `set_to_serial_mode` over UDP.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut lidar = UnilidarL2::new();
    lidar.initialize_serial(SerialConfig::default())?;
    println!("connected over serial");

    lidar.start_lidar_rotation();
    std::thread::sleep(Duration::from_secs(1));

    println!("setting work mode to 0 (udp)");
    lidar.set_lidar_work_mode(0);
    std::thread::sleep(Duration::from_secs(1));

    println!("done — lidar is now in udp mode");
    Ok(())
}
