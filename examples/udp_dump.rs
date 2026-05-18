use std::time::{Duration, Instant};

use unilidar_sdk2_cxx::{LidarPacket, UdpConfig, UnilidarL2};

fn main() {
    let mut lidar = UnilidarL2::new();
    lidar
        .initialize_udp(UdpConfig::default())
        .expect("failed to open UDP socket — check the lidar is on 192.168.1.62 and your iface has 192.168.1.2");

    println!("starting lidar rotation");
    lidar.start_lidar_rotation();
    std::thread::sleep(Duration::from_secs(1));

    println!("setting work mode to 0");
    lidar.set_lidar_work_mode(0);
    std::thread::sleep(Duration::from_secs(1));

    println!("resetting lidar");
    lidar.reset_lidar();
    std::thread::sleep(Duration::from_secs(2));

    let mut clouds = 0u64;
    let mut imus = 0u64;
    let mut last_report = Instant::now();

    loop {
        match lidar.run_parse() {
            LidarPacket::PointData2D => {
                println!("point data 2d");
                let cloud = lidar.get_point_cloud();
                clouds += 1;
                if last_report.elapsed() >= Duration::from_secs(1) {
                    println!(
                        "clouds={clouds} imus={imus} last_cloud points={} ring_num={} stamp={:.3}",
                        cloud.points.len(),
                        cloud.ring_num,
                        cloud.stamp,
                    );
                    if let Some(p) = cloud.points.first() {
                        println!(
                            "  first point: x={:.3} y={:.3} z={:.3} intensity={:.1} ring={}",
                            p.x, p.y, p.z, p.intensity, p.ring,
                        );
                    }
                    last_report = Instant::now();
                }
            }
            LidarPacket::ImuData => {
                println!("imu data");
                let imu_data = lidar.get_imu_data();
                println!("{:?}", imu_data);
                imus += 1;
            }
            LidarPacket::NoPacket => {
                println!("no packet");
                std::thread::sleep(Duration::from_micros(100));
            }
            _ => {
                println!("null");
            }
        }
    }
}
