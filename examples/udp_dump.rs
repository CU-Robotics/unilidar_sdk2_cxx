use std::time::{Duration, Instant};

use unilidar_sdk2_cxx::{LidarPacket, UdpConfig, UnilidarL2};

fn main() {
    let mut lidar = UnilidarL2::new();
    lidar
        .initialize_udp(UdpConfig::default())
        .expect("failed to open UDP socket — check the lidar is on 192.168.1.62 and your iface has 192.168.1.2");

    println!("stopping lidar rotation");
    lidar.stop_lidar_rotation();
    std::thread::sleep(Duration::from_secs(3));

    println!("starting lidar rotation");
    lidar.start_lidar_rotation();
    std::thread::sleep(Duration::from_secs(3));

    let mut clouds = 0u64;
    let mut imus = 0u64;
    let mut last_report = Instant::now();

    println!("starting loop");

    loop {
        match lidar.run_parse() {
            LidarPacket::PointData => {
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
                let _ = lidar.get_imu_data();
                imus += 1;
                if last_report.elapsed() >= Duration::from_secs(1) {
                    println!("heartbeat: clouds={clouds} imus={imus}");
                    last_report = Instant::now();
                }
            }
            LidarPacket::NoPacket => {
                std::thread::sleep(Duration::from_micros(100));
            }
            other => {
                println!("other packet: {other:?}");
            }
        }
    }
}
