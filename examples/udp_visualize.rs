use rerun::{Points3D, RecordingStreamBuilder};
use std::time::{Duration, Instant};
use unilidar_sdk2_cxx::{LidarPacket, UdpConfig, UnilidarL2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = RecordingStreamBuilder::new("unilidar_l2").spawn()?;
    println!("rerun stream spawned");

    let mut lidar = UnilidarL2::new();
    let cfg = UdpConfig {
        ..UdpConfig::default()
    };
    lidar.initialize_udp(cfg)?;

    lidar.stop_lidar_rotation();
    std::thread::sleep(Duration::from_secs(3));
    lidar.start_lidar_rotation();
    std::thread::sleep(Duration::from_secs(3));

    println!("entering parse loop");

    const TRAIL_LEN: usize = 100;
    let mut slot: usize = 0;

    let mut clouds_total = 0u64;
    let mut clouds_logged = 0u64;
    let mut last_report = Instant::now();

    loop {
        match lidar.run_parse() {
            LidarPacket::PointData => {
                let cloud = lidar.get_point_cloud();
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

                if last_report.elapsed() >= Duration::from_secs(1) {
                    println!(
                        "clouds_total={clouds_total} clouds_logged={clouds_logged} last_size={}",
                        cloud.points.len()
                    );
                    last_report = Instant::now();
                }
            }
            LidarPacket::NoPacket => {
                std::thread::sleep(Duration::from_micros(100));
            }
            _ => {}
        }
    }
}
