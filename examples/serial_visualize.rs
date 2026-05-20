use rerun::{Points3D, RecordingStreamBuilder};
use std::env;
use std::time::{Duration, Instant};
use unilidar_sdk2_cxx::{LidarPacket, SerialConfig, UnilidarL2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = RecordingStreamBuilder::new("unilidar_l2").spawn()?;
    println!("rerun stream spawned");

    let mut lidar = UnilidarL2::new();
    let cfg = SerialConfig {
        ..SerialConfig::default()
    };
    lidar.initialize_serial(cfg)?;
    println!("serial connection opened");

    lidar.start_lidar_rotation();
    println!("rotation started, waiting 1s");
    std::thread::sleep(Duration::from_secs(1));

    if env::args().any(|arg| arg == "--configure") {
        println!("configuring lidar for serial work mode");
        lidar.set_lidar_work_mode(8);
        std::thread::sleep(Duration::from_secs(1));

        lidar.reset_lidar();
        println!("lidar reset to apply work mode, waiting 2s");
        std::thread::sleep(Duration::from_secs(2));

        lidar.close_serial();
        println!("serial handle closed after reset, reopening");

        lidar = UnilidarL2::new();
        lidar.initialize_serial(SerialConfig::default())?;
        lidar.start_lidar_rotation();
        std::thread::sleep(Duration::from_secs(1));
    }

    println!("entering parse loop");

    const TRAIL_LEN: usize = 100;
    let mut slot: usize = 0;

    let mut clouds_total = 0u64;
    let mut clouds_logged = 0u64;
    let mut point_packets = 0u64;
    let mut point_2d_packets = 0u64;
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
                    }
                }
            }
            LidarPacket::NoPacket => {
                no_packets += 1;
                std::thread::sleep(Duration::from_micros(100));
            }
            other => {
                println!("{:?}", other);
            }
        }

        if last_report.elapsed() >= Duration::from_secs(1) {
            println!(
                "point_packets={point_packets} point_2d_packets={point_2d_packets} no_packets={no_packets} clouds_total={clouds_total} clouds_logged={clouds_logged}"
            );
            last_report = Instant::now();
        }
    }
}
