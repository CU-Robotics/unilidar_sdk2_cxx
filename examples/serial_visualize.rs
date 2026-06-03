use rerun::{Clear, Points3D, RecordingStreamBuilder};
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;
use unilidar_sdk2_cxx::{LidarPacket, LidarPacketCounts, SerialConfig, UnilidarL2};

const TRAIL_LEN: usize = 100;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = SerialConfig {
        // port: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A2A046145-if00".to_string(),
        port: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A2A026768-if00".to_string(),

        baudrate: 4000000,
        // range_min: 0.25,
        ..SerialConfig::default()
    };

    let rec = RecordingStreamBuilder::new("unilidar_l2").spawn()?;
    println!("rerun stream spawned");
    rec.log("lidar/points", &Clear::recursive())?;
    rec.log_static(
        "lidar/sensor_origin",
        &Points3D::new([[0.0, 0.0, 0.0]])
            .with_colors([[255, 32, 32]])
            .with_radii([0.08])
            .with_labels(["lidar"]),
    )?;
    println!(
        "reading serial port {} at {} baud",
        config.port, config.baudrate
    );

    let mut lidar = UnilidarL2::new();
    let range_min = config.range_min;
    lidar.initialize_serial_direct(config)?;

    // lidar.stop_lidar_rotation();
    // sleep(Duration::from_millis(2000));
    // lidar.set_lidar_work_mode(8);
    // lidar.reset_lidar();
    // sleep(Duration::from_millis(1000));

    // lidar.start_lidar_rotation();

    let mut slot = 0usize;
    let mut clouds_logged = 0u64;
    let mut counts = LidarPacketCounts::default();
    let mut last_report = Instant::now();
    let mut last_cloud_points = 0usize;
    let mut last_min_xyz_range: Option<f32> = None;
    let mut last_below_visual_min = 0usize;

    loop {
        let packet = lidar.run_parse();
        counts.record(&packet);

        match packet {
            LidarPacket::PointData | LidarPacket::PointData2D => {
                if let Some(cloud) = lidar.try_get_point_cloud() {
                    let path = format!("lidar/points/{slot}");
                    last_cloud_points = cloud.points.len();
                    last_min_xyz_range = cloud
                        .points
                        .iter()
                        .map(|p| p.x.hypot(p.y).hypot(p.z))
                        .min_by(|a, b| a.total_cmp(b));
                    last_below_visual_min = cloud
                        .points
                        .iter()
                        .filter(|p| p.x.hypot(p.y).hypot(p.z) < range_min)
                        .count();

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

                        rec.log(path, &Points3D::new(positions).with_colors(colors))?;
                    } else {
                        rec.log(path, &Clear::flat())?;
                    }

                    slot = (slot + 1) % TRAIL_LEN;
                    clouds_logged += 1;
                }
            }
            LidarPacket::NoPacket => {
                std::thread::sleep(Duration::from_micros(100));
            }
            _ => {}
        }

        if last_report.elapsed().as_secs() >= 1 {
            let min_xyz = last_min_xyz_range
                .map(|range| format!("{range:.3}m"))
                .unwrap_or_else(|| "none".to_string());
            println!(
                "{counts} clouds_logged={clouds_logged} last_cloud_points={last_cloud_points} min_xyz_range={min_xyz} below_visual_{range_min:.2}m={last_below_visual_min}"
            );
            last_report = Instant::now();
        }
    }
}
