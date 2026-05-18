use std::time::{Duration, Instant};

use unilidar_sdk2_cxx::{LidarPacket, SerialConfig, UnilidarL2};

fn main() {
    let mut lidar = UnilidarL2::new();
    lidar
        .initialize_serial(SerialConfig::default())
        .expect("failed to open serial port — is the lidar plugged in and in serial mode?");

    lidar.start_lidar_rotation();
    std::thread::sleep(Duration::from_secs(1));

    lidar.set_lidar_work_mode(8);
    std::thread::sleep(Duration::from_secs(1));

    lidar.reset_lidar();
    std::thread::sleep(Duration::from_secs(1));

    let mut clouds = 0u64;
    let mut imus = 0u64;
    let mut last_report = Instant::now();

    loop {
        match lidar.run_parse() {
            LidarPacket::PointData2D => {
                let cloud = lidar.get_point_cloud();
                clouds += 1;
                if last_report.elapsed() >= Duration::from_secs(1) {
                    println!(
                        "clouds={clouds} imus={imus} last_cloud points={} ring_num={}",
                        cloud.points.len(),
                        cloud.ring_num
                    );
                    last_report = Instant::now();
                }
            }
            LidarPacket::ImuData => {
                let _ = lidar.get_imu_data();
                imus += 1;
            }
            LidarPacket::NoPacket => {
                std::thread::sleep(Duration::from_micros(100));
            }
            _ => {}
        }
    }
}
