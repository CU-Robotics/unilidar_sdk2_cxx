use rerun::{Points3D, RecordingStreamBuilder};
use std::env;
use std::path::Path;
use std::time::Instant;
use unilidar_sdk2_cxx::{DirectSerialPacket, SerialConfig, SerialPointCloudReader};

const DEFAULT_SERIAL_PORT: &str = "/dev/ttyACM0";
const LIDAR_SERIAL_BY_ID: &str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A2A026768-if00";
const TRAIL_LEN: usize = 100;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let config = SerialConfig::default()
        .port(serial_port(&args))
        .baudrate(serial_baudrate(&args)?);

    let rec = RecordingStreamBuilder::new("unilidar_l2").spawn()?;
    println!("rerun stream spawned");
    println!(
        "reading serial port {} at {} baud",
        config.port, config.baudrate
    );

    let mut reader = SerialPointCloudReader::open(config)?;

    let mut slot = 0usize;
    let mut point_packets = 0u64;
    let mut point_2d_packets = 0u64;
    let mut clouds_logged = 0u64;
    let mut bytes_read = 0u64;
    let mut last_report = Instant::now();

    loop {
        let read = reader.read_next()?;
        bytes_read += read.bytes_read as u64;

        match read.packet {
            Some(DirectSerialPacket::PointData) => point_packets += 1,
            Some(DirectSerialPacket::PointData2D) => point_2d_packets += 1,
            Some(DirectSerialPacket::Other(_)) | None => {}
        }

        if let Some(cloud) = read.point_cloud {
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

        if last_report.elapsed().as_secs() >= 1 {
            println!(
                "bytes_read={bytes_read} point_packets={point_packets} point_2d_packets={point_2d_packets} clouds_logged={clouds_logged}"
            );
            last_report = Instant::now();
        }
    }
}

fn serial_port(args: &[String]) -> String {
    if let Some(port) = arg_value(args, "--port") {
        return port;
    }

    if let Ok(port) = env::var("UNILIDAR_SERIAL_PORT") {
        return port;
    }

    if Path::new(LIDAR_SERIAL_BY_ID).exists() {
        return LIDAR_SERIAL_BY_ID.to_owned();
    }

    DEFAULT_SERIAL_PORT.to_owned()
}

fn serial_baudrate(args: &[String]) -> Result<u32, Box<dyn std::error::Error>> {
    let baudrate = arg_value(args, "--baud")
        .or_else(|| env::var("UNILIDAR_SERIAL_BAUD").ok())
        .unwrap_or_else(|| SerialConfig::default().baudrate.to_string());

    Ok(baudrate.parse()?)
}

fn arg_value(args: &[String], name: &str) -> Option<String> {
    args.windows(2)
        .find(|pair| pair[0] == name)
        .map(|pair| pair[1].clone())
}
