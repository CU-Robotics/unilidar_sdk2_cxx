use rerun::{Points3D, RecordingStreamBuilder};
use std::env;
use std::fs::File;
use std::io::Read;
use std::path::Path;
use std::process::Command;
use std::time::Instant;
use unilidar_sdk2_cxx::SerialConfig;

const DEFAULT_SERIAL_PORT: &str = "/dev/ttyACM0";
const LIDAR_SERIAL_BY_ID: &str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A2A026768-if00";
const FRAME_HEADER: [u8; 4] = [0x55, 0xaa, 0x05, 0x0a];
const LIDAR_POINT_DATA_PACKET_TYPE: u32 = 102;
const LIDAR_2D_POINT_DATA_PACKET_TYPE: u32 = 103;
const MAX_FRAME_SIZE: usize = 6_000;
const TRAIL_LEN: usize = 100;

#[derive(Clone, Copy)]
struct Point {
    position: [f32; 3],
    intensity: u8,
}

struct Cloud {
    points: Vec<Point>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let port = serial_port(&args);
    let baudrate = serial_baudrate(&args)?;

    configure_tty(&port, baudrate)?;

    let rec = RecordingStreamBuilder::new("unilidar_l2").spawn()?;
    println!("rerun stream spawned");
    println!("reading serial port {port} at {baudrate} baud");

    let mut serial = File::open(&port)?;
    let mut read_buf = [0u8; 8192];
    let mut frame_buf = Vec::with_capacity(MAX_FRAME_SIZE * 2);

    let mut slot = 0usize;
    let mut point_packets = 0u64;
    let mut point_2d_packets = 0u64;
    let mut clouds_logged = 0u64;
    let mut bytes_read = 0u64;
    let mut last_report = Instant::now();

    loop {
        let n = serial.read(&mut read_buf)?;
        if n == 0 {
            continue;
        }

        bytes_read += n as u64;
        frame_buf.extend_from_slice(&read_buf[..n]);

        while let Some(frame) = next_frame(&mut frame_buf) {
            match packet_type(&frame) {
                LIDAR_POINT_DATA_PACKET_TYPE => {
                    point_packets += 1;

                    if let Some(cloud) = parse_point_cloud(&frame) {
                        if !cloud.points.is_empty() {
                            let positions: Vec<[f32; 3]> =
                                cloud.points.iter().map(|p| p.position).collect();
                            let colors: Vec<[u8; 3]> = cloud
                                .points
                                .iter()
                                .map(|p| [p.intensity, p.intensity, p.intensity])
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
                LIDAR_2D_POINT_DATA_PACKET_TYPE => {
                    point_2d_packets += 1;
                }
                _ => {}
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

fn configure_tty(port: &str, baudrate: u32) -> Result<(), Box<dyn std::error::Error>> {
    let status = Command::new("stty")
        .args([
            "-F",
            port,
            &baudrate.to_string(),
            "raw",
            "-echo",
            "-ixon",
            "-ixoff",
            "-crtscts",
        ])
        .status()?;

    if status.success() {
        Ok(())
    } else {
        Err(format!("stty failed for {port}").into())
    }
}

fn next_frame(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    let header_pos = buf
        .windows(FRAME_HEADER.len())
        .position(|window| window == FRAME_HEADER)?;

    if header_pos > 0 {
        buf.drain(..header_pos);
    }

    if buf.len() < 12 {
        return None;
    }

    let size = le_u32(buf, 8) as usize;
    if !(24..=MAX_FRAME_SIZE).contains(&size) {
        buf.drain(..FRAME_HEADER.len());
        return None;
    }

    if buf.len() < size {
        return None;
    }

    Some(buf.drain(..size).collect())
}

fn packet_type(frame: &[u8]) -> u32 {
    le_u32(frame, 4)
}

fn parse_point_cloud(frame: &[u8]) -> Option<Cloud> {
    let data = frame.get(12..)?;

    let a_axis_dist = le_f32(data, 52);
    let b_axis_dist = le_f32(data, 56);
    let theta_angle_bias = le_f32(data, 60);
    let alpha_angle_bias = le_f32(data, 64);
    let beta_angle = le_f32(data, 68);
    let xi_angle = le_f32(data, 72);
    let range_bias = le_f32(data, 76);
    let range_scale = le_f32(data, 80);

    let com_horizontal_angle_start = le_f32(data, 84);
    let com_horizontal_angle_step = le_f32(data, 88);
    let range_min = le_f32(data, 96);
    let range_max = le_f32(data, 100);
    let angle_min = le_f32(data, 104);
    let angle_increment = le_f32(data, 108);
    let point_num = le_u32(data, 116).min(300) as usize;

    let ranges_offset = 120;
    let intensities_offset = ranges_offset + 300 * 2;
    if data.len() < intensities_offset + 300 {
        return None;
    }

    let sin_beta = beta_angle.sin();
    let cos_beta = beta_angle.cos();
    let sin_xi = xi_angle.sin();
    let cos_xi = xi_angle.cos();
    let cos_beta_sin_xi = cos_beta * sin_xi;
    let sin_beta_cos_xi = sin_beta * cos_xi;
    let sin_beta_sin_xi = sin_beta * sin_xi;
    let cos_beta_cos_xi = cos_beta * cos_xi;

    let mut points = Vec::with_capacity(point_num);
    let mut alpha_cur = angle_min + alpha_angle_bias;
    let mut theta_cur = com_horizontal_angle_start + theta_angle_bias;

    for j in 0..point_num {
        let range_raw = le_u16(data, ranges_offset + j * 2);
        if range_raw < 1 {
            alpha_cur += angle_increment;
            theta_cur += com_horizontal_angle_step;
            continue;
        }

        let range = range_scale * (range_raw as f32 + range_bias);
        if range < range_min || range > range_max {
            alpha_cur += angle_increment;
            theta_cur += com_horizontal_angle_step;
            continue;
        }

        let sin_alpha = alpha_cur.sin();
        let cos_alpha = alpha_cur.cos();
        let sin_theta = theta_cur.sin();
        let cos_theta = theta_cur.cos();

        let a = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range + b_axis_dist;
        let b = cos_alpha * cos_xi * range;
        let c = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range;

        points.push(Point {
            position: [
                cos_theta * a - sin_theta * b,
                sin_theta * a + cos_theta * b,
                c + a_axis_dist,
            ],
            intensity: data[intensities_offset + j],
        });

        alpha_cur += angle_increment;
        theta_cur += com_horizontal_angle_step;
    }

    Some(Cloud { points })
}

fn le_u16(buf: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([buf[offset], buf[offset + 1]])
}

fn le_u32(buf: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        buf[offset],
        buf[offset + 1],
        buf[offset + 2],
        buf[offset + 3],
    ])
}

fn le_f32(buf: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes([
        buf[offset],
        buf[offset + 1],
        buf[offset + 2],
        buf[offset + 3],
    ])
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
