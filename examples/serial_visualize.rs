use rerun::{Points3D, RecordingStreamBuilder};
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;
use unilidar_sdk2_cxx::{LidarPacket, LidarPacketCounts, SerialConfig, UnilidarL2};

const TRAIL_LEN: usize = 100;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = SerialConfig::from_env_args()?;

    let rec = RecordingStreamBuilder::new("unilidar_l2").spawn()?;
    println!("rerun stream spawned");
    println!(
        "reading serial port {} at {} baud",
        config.port, config.baudrate
    );

    let mut lidar = UnilidarL2::new();
    lidar.initialize_serial_direct(config)?;

    println!("setting work mode to 8 (serial)");
    lidar.set_lidar_work_mode(8);
    sleep(Duration::from_secs(1));
    println!("resetting lidar to apply mode");
    lidar.reset_lidar();
    sleep(Duration::from_secs(3));

    sleep(Duration::from_secs(1));
    println!("Stopping");
    lidar.stop_lidar_rotation();
    sleep(Duration::from_secs(1));
    println!("starting up again");
    lidar.start_lidar_rotation();

    let mut slot = 0usize;
    let mut clouds_logged = 0u64;
    let mut counts = LidarPacketCounts::default();
    let mut last_report = Instant::now();

    loop {
        let packet = lidar.run_parse();
        counts.record(&packet);

        match packet {
            LidarPacket::PointData | LidarPacket::PointData2D => {
                if let Some(cloud) = lidar.try_get_point_cloud() {
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
                std::thread::sleep(Duration::from_micros(100));
            }
            _ => {}
        }

        if last_report.elapsed().as_secs() >= 1 {
            println!("{counts} clouds_logged={clouds_logged}");
            last_report = Instant::now();
        }
    }
}
