use rerun::{Points3D, RecordingStreamBuilder};
use std::env;
use std::time::{Duration, Instant};
use unilidar_sdk2_cxx::{LidarPacket, UdpConfig, UnilidarL2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let no_rerun = env::args().any(|arg| arg == "--no-rerun");
    let rec = if no_rerun {
        println!("rerun disabled");
        None
    } else {
        let rec = RecordingStreamBuilder::new("unilidar_l2").spawn()?;
        println!("rerun stream spawned");
        Some(rec)
    };

    let mut lidar = UnilidarL2::new();
    let cfg = UdpConfig {
        cloud_scan_num: 18,
        ..UdpConfig::default()
    };
    lidar.initialize_udp(cfg)?;

    lidar.start_lidar_rotation();
    std::thread::sleep(Duration::from_secs(1));

    println!("setting work mode to 0 (udp)");
    lidar.set_lidar_work_mode(0);
    std::thread::sleep(Duration::from_secs(1));

    println!("resetting lidar");
    lidar.reset_lidar();
    std::thread::sleep(Duration::from_secs(2));

    println!("starting lidar rotation after reset");
    lidar.start_lidar_rotation();
    std::thread::sleep(Duration::from_secs(3));

    println!("entering parse loop");

    const TRAIL_LEN: usize = 10;
    let mut slot: usize = 0;

    let mut clouds_total = 0u64;
    let mut clouds_logged = 0u64;
    let mut point_packets = 0u64;
    let mut point_2d_packets = 0u64;
    let mut imu_packets = 0u64;
    let mut ack_packets = 0u64;
    let mut param_packets = 0u64;
    let mut other_packets = 0u64;
    let mut no_packets = 0u64;
    let mut last_report = Instant::now();

    loop {
        match lidar.run_parse() {
            packet @ (LidarPacket::PointData | LidarPacket::PointData2D) => {
                match packet {
                    LidarPacket::PointData => point_packets += 1,
                    LidarPacket::PointData2D => point_2d_packets += 1,
                    _ => unreachable!(),
                }

                if let Some(cloud) = lidar.try_get_point_cloud() {
                    clouds_total += 1;

                    if !cloud.points.is_empty() {
                        let positions: Vec<[f32; 3]> =
                            cloud.points.iter().map(|p| [p.x, p.y, p.z]).collect();

                        if let Some(rec) = &rec {
                            let colors: Vec<[u8; 3]> = cloud
                                .points
                                .iter()
                                .map(|p| {
                                    let g = p.intensity.clamp(0.0, 255.0) as u8;
                                    [g, g, g]
                                })
                                .collect();

                            rec.log(
                                format!("lidar/points/{slot}"),
                                &Points3D::new(positions).with_colors(colors),
                            )?;

                            slot = (slot + 1) % TRAIL_LEN;
                            clouds_logged += 1;
                        } else {
                            clouds_logged += 1;
                            if clouds_logged <= 3 {
                                println!(
                                    "cloud id={} points={} first_point={:?}",
                                    cloud.id,
                                    cloud.points.len(),
                                    positions.first()
                                );
                            }
                        }
                    }
                }
            }
            LidarPacket::NoPacket => {
                no_packets += 1;
                std::thread::sleep(Duration::from_micros(100));
            }
            LidarPacket::ImuData => {
                imu_packets += 1;
            }
            LidarPacket::AckData => {
                ack_packets += 1;
            }
            LidarPacket::ParamData => {
                param_packets += 1;
            }
            other => {
                other_packets += 1;
                if other_packets <= 10 {
                    println!("{:?}", other);
                }
            }
        }

        if last_report.elapsed() >= Duration::from_secs(1) {
            println!(
                "point_packets={point_packets} point_2d_packets={point_2d_packets} imu_packets={imu_packets} ack_packets={ack_packets} param_packets={param_packets} other_packets={other_packets} no_packets={no_packets} clouds_total={clouds_total} clouds_logged={clouds_logged}"
            );
            last_report = Instant::now();
        }
    }
}
