use crate::{
    DataInfo, ImuData, LIDAR_2D_POINT_DATA_PACKET_TYPE, LIDAR_ACK_DATA_PACKET_TYPE,
    LIDAR_IMU_DATA_PACKET_TYPE, LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE,
    LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE, LIDAR_PARAM_DATA_PACKET_TYPE,
    LIDAR_POINT_DATA_PACKET_TYPE, LIDAR_TIME_STAMP_PACKET_TYPE, LIDAR_USER_CMD_PACKET_TYPE,
    LIDAR_VERSION_PACKET_TYPE, LIDAR_WORK_MODE_CONFIG_PACKET_TYPE, LidarPacket, Point, PointCloud,
    SerialConfig, UdpConfig, ffi,
};
use std::env;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::net::Ipv4Addr;
use std::num::ParseIntError;
use std::path::Path;
use std::process::Command;
use std::str::FromStr;
use thiserror::Error;

const FRAME_HEADER: [u8; 4] = [0x55, 0xaa, 0x05, 0x0a];
const MAX_FRAME_SIZE: usize = 6_000;
const DEFAULT_SERIAL_PORT: &str = "/dev/ttyACM0";
const LIDAR_SERIAL_BY_ID: &str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A2A026768-if00";
const LIDAR_WORK_MODE_COMMAND_PACKET_TYPE: u32 = 2002;
const USER_CMD_RESET_TYPE: u32 = 1;
const USER_CMD_STANDBY_TYPE: u32 = 2;
const USER_CMD_STANDBY_START: u32 = 0;
const USER_CMD_STANDBY_STOP: u32 = 1;

#[derive(Error, Debug)]
pub enum DirectSerialError {
    #[error("failed to configure serial port {port}: {source}")]
    ConfigureIo {
        port: String,
        source: std::io::Error,
    },
    #[error("stty failed for serial port {port}")]
    ConfigureFailed { port: String },
    #[error("failed to open serial port {port}: {source}")]
    Open {
        port: String,
        source: std::io::Error,
    },
    #[error("failed to read serial port {port}: {source}")]
    Read {
        port: String,
        source: std::io::Error,
    },
    #[error("failed to write serial port {port}: {source}")]
    Write {
        port: String,
        source: std::io::Error,
    },
    #[error("invalid serial baudrate {baudrate:?}: {source}")]
    InvalidBaudrate {
        baudrate: String,
        source: ParseIntError,
    },
    #[error("invalid IPv4 address {address:?}")]
    InvalidIpv4 { address: String },
}

impl SerialConfig {
    pub fn from_env_args() -> Result<Self, DirectSerialError> {
        let args: Vec<String> = env::args().collect();
        Self::from_args(&args)
    }

    pub fn from_args(args: &[String]) -> Result<Self, DirectSerialError> {
        Ok(Self::default()
            .port(serial_port(args))
            .baudrate(serial_baudrate(args)?))
    }

    pub fn port(mut self, port: impl Into<String>) -> Self {
        self.port = port.into();
        self
    }

    pub fn baudrate(mut self, baudrate: u32) -> Self {
        self.baudrate = baudrate;
        self
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

fn serial_baudrate(args: &[String]) -> Result<u32, DirectSerialError> {
    let baudrate = arg_value(args, "--baud")
        .or_else(|| env::var("UNILIDAR_SERIAL_BAUD").ok())
        .unwrap_or_else(|| SerialConfig::default().baudrate.to_string());

    baudrate
        .parse()
        .map_err(|source| DirectSerialError::InvalidBaudrate { baudrate, source })
}

fn arg_value(args: &[String], name: &str) -> Option<String> {
    args.windows(2)
        .find(|pair| pair[0] == name)
        .map(|pair| pair[1].clone())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DirectSerialPacket {
    PointData,
    PointData2D,
    Other(u32),
}

impl From<DirectSerialPacket> for LidarPacket {
    fn from(packet: DirectSerialPacket) -> Self {
        match packet {
            DirectSerialPacket::PointData => Self::PointData,
            DirectSerialPacket::PointData2D => Self::PointData2D,
            DirectSerialPacket::Other(LIDAR_USER_CMD_PACKET_TYPE) => Self::UserCmd,
            DirectSerialPacket::Other(LIDAR_ACK_DATA_PACKET_TYPE) => Self::AckData,
            DirectSerialPacket::Other(LIDAR_IMU_DATA_PACKET_TYPE) => Self::ImuData,
            DirectSerialPacket::Other(LIDAR_VERSION_PACKET_TYPE) => Self::LidarVersion,
            DirectSerialPacket::Other(LIDAR_TIME_STAMP_PACKET_TYPE) => Self::TimeStamp,
            DirectSerialPacket::Other(LIDAR_WORK_MODE_CONFIG_PACKET_TYPE) => Self::WorkModeConfig,
            DirectSerialPacket::Other(LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE) => Self::IpAddressConfig,
            DirectSerialPacket::Other(LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE) => {
                Self::MacAddressConfig
            }
            DirectSerialPacket::Other(LIDAR_PARAM_DATA_PACKET_TYPE) => Self::ParamData,
            DirectSerialPacket::Other(_) => Self::NoPacket,
        }
    }
}

#[derive(Debug)]
pub struct DirectSerialRead {
    pub bytes_read: usize,
    pub packet: Option<DirectSerialPacket>,
    pub point_cloud: Option<PointCloud>,
    pub imu_data: Option<ImuData>,
}

/// Direct serial point-cloud reader for systems where Unitree's precompiled serial `runParse`
/// path opens the tty but never surfaces packets. This reads the documented SDK packet format
/// from the serial device and applies the same 3D point transform as `unitree_lidar_utilities.h`.
pub struct SerialPointCloudReader {
    port: String,
    serial: File,
    read_buf: [u8; 8192],
    frame_buf: Vec<u8>,
    range_min: f32,
    range_max: f32,
}

impl SerialPointCloudReader {
    pub fn open(config: SerialConfig) -> Result<Self, DirectSerialError> {
        configure_tty(&config.port, config.baudrate)?;
        let serial = File::open(&config.port).map_err(|source| DirectSerialError::Open {
            port: config.port.clone(),
            source,
        })?;

        Ok(Self {
            port: config.port,
            serial,
            read_buf: [0; 8192],
            frame_buf: Vec::with_capacity(MAX_FRAME_SIZE * 2),
            range_min: config.range_min,
            range_max: config.range_max,
        })
    }

    pub fn port(&self) -> &str {
        &self.port
    }

    pub fn read_next(&mut self) -> Result<DirectSerialRead, DirectSerialError> {
        loop {
            if let Some(frame) = next_frame(&mut self.frame_buf) {
                return Ok(read_from_frame(&frame, self.range_min, self.range_max));
            }

            let bytes_read =
                self.serial
                    .read(&mut self.read_buf)
                    .map_err(|source| DirectSerialError::Read {
                        port: self.port.clone(),
                        source,
                    })?;

            if bytes_read > 0 {
                self.frame_buf
                    .extend_from_slice(&self.read_buf[..bytes_read]);

                if let Some(frame) = next_frame(&mut self.frame_buf) {
                    let mut read = read_from_frame(&frame, self.range_min, self.range_max);
                    read.bytes_read = bytes_read;
                    return Ok(read);
                }
            }
        }
    }
}

pub fn set_lidar_work_mode_serial(
    config: SerialConfig,
    mode: u32,
) -> Result<(), DirectSerialError> {
    configure_tty(&config.port, config.baudrate)?;

    let mut serial = OpenOptions::new()
        .read(true)
        .write(true)
        .open(&config.port)
        .map_err(|source| DirectSerialError::Open {
            port: config.port.clone(),
            source,
        })?;

    let frame = work_mode_command_frame(mode);
    serial
        .write_all(&frame)
        .and_then(|()| serial.flush())
        .map_err(|source| DirectSerialError::Write {
            port: config.port,
            source,
        })
}

pub fn set_lidar_udp_config_serial(
    serial_config: SerialConfig,
    udp_config: UdpConfig,
) -> Result<(), DirectSerialError> {
    configure_tty(&serial_config.port, serial_config.baudrate)?;

    let mut serial = OpenOptions::new()
        .read(true)
        .write(true)
        .open(&serial_config.port)
        .map_err(|source| DirectSerialError::Open {
            port: serial_config.port.clone(),
            source,
        })?;

    let frame = udp_config_command_frame(&udp_config)?;
    serial
        .write_all(&frame)
        .and_then(|()| serial.flush())
        .map_err(|source| DirectSerialError::Write {
            port: serial_config.port,
            source,
        })
}

pub fn reset_lidar_serial(config: SerialConfig) -> Result<(), DirectSerialError> {
    send_lidar_user_control_serial(&config, USER_CMD_RESET_TYPE, 0)
}

pub fn start_lidar_rotation_serial(config: SerialConfig) -> Result<(), DirectSerialError> {
    send_lidar_user_control_serial(&config, USER_CMD_STANDBY_TYPE, USER_CMD_STANDBY_START)
}

pub fn stop_lidar_rotation_serial(config: SerialConfig) -> Result<(), DirectSerialError> {
    send_lidar_user_control_serial(&config, USER_CMD_STANDBY_TYPE, USER_CMD_STANDBY_STOP)
}

fn send_lidar_user_control_serial(
    config: &SerialConfig,
    cmd_type: u32,
    cmd_value: u32,
) -> Result<(), DirectSerialError> {
    configure_tty(&config.port, config.baudrate)?;

    let mut serial = OpenOptions::new()
        .read(true)
        .write(true)
        .open(&config.port)
        .map_err(|source| DirectSerialError::Open {
            port: config.port.clone(),
            source,
        })?;

    let frame = user_control_command_frame(cmd_type, cmd_value);
    serial
        .write_all(&frame)
        .and_then(|()| serial.flush())
        .map_err(|source| DirectSerialError::Write {
            port: config.port.clone(),
            source,
        })
}

fn configure_tty(port: &str, baudrate: u32) -> Result<(), DirectSerialError> {
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
        .status()
        .map_err(|source| DirectSerialError::ConfigureIo {
            port: port.to_owned(),
            source,
        })?;

    if status.success() {
        Ok(())
    } else {
        Err(DirectSerialError::ConfigureFailed {
            port: port.to_owned(),
        })
    }
}

fn read_from_frame(frame: &[u8], range_min: f32, range_max: f32) -> DirectSerialRead {
    let packet = packet_type(frame);
    let direct_packet = match packet {
        LIDAR_POINT_DATA_PACKET_TYPE => DirectSerialPacket::PointData,
        LIDAR_2D_POINT_DATA_PACKET_TYPE => DirectSerialPacket::PointData2D,
        other => DirectSerialPacket::Other(other),
    };

    let point_cloud = match direct_packet {
        DirectSerialPacket::PointData => parse_point_cloud(frame, range_min, range_max),
        DirectSerialPacket::PointData2D | DirectSerialPacket::Other(_) => None,
    };
    let imu_data = match direct_packet {
        DirectSerialPacket::Other(LIDAR_IMU_DATA_PACKET_TYPE) => parse_imu_data(frame),
        DirectSerialPacket::PointData
        | DirectSerialPacket::PointData2D
        | DirectSerialPacket::Other(_) => None,
    };

    DirectSerialRead {
        bytes_read: 0,
        packet: Some(direct_packet),
        point_cloud,
        imu_data,
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

fn parse_point_cloud(frame: &[u8], config_range_min: f32, config_range_max: f32) -> Option<PointCloud> {
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
    let scan_period = le_f32(data, 92);
    let range_min = le_f32(data, 96);
    let range_max = le_f32(data, 100);
    let angle_min = le_f32(data, 104);
    let angle_increment = le_f32(data, 108);
    let time_increment = le_f32(data, 112);
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
    let mut time_relative = 0.0;

    for j in 0..point_num {
        let range_raw = le_u16(data, ranges_offset + j * 2);
        if range_raw >= 1 {
            let range = range_scale * (range_raw as f32 + range_bias);
            // Match the C++ SDK: filter by the packet's own bounds, then further
            // constrain by the user-supplied config bounds.
            if range >= range_min
                && range <= range_max
                && range >= config_range_min
                && range <= config_range_max
            {
                let sin_alpha = alpha_cur.sin();
                let cos_alpha = alpha_cur.cos();
                let sin_theta = theta_cur.sin();
                let cos_theta = theta_cur.cos();

                let a = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range + b_axis_dist;
                let b = cos_alpha * cos_xi * range;
                let c = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range;

                points.push(Point {
                    x: cos_theta * a - sin_theta * b,
                    y: sin_theta * a + cos_theta * b,
                    z: c + a_axis_dist,
                    intensity: data[intensities_offset + j] as f32,
                    time: time_relative,
                    ring: 1,
                });
            }
        }

        alpha_cur += angle_increment;
        theta_cur += com_horizontal_angle_step;
        time_relative += time_increment;
    }

    Some(PointCloud {
        stamp: system_time_seconds() - scan_period as f64,
        id: 1,
        ring_num: 1,
        points,
    })
}

fn parse_imu_data(frame: &[u8]) -> Option<ImuData> {
    let data = frame.get(12..)?;
    if data.len() < 56 {
        return None;
    }

    Some(ImuData {
        info: DataInfo {
            seq: le_u32(data, 0),
            payload_size: le_u32(data, 4),
            stamp: ffi::TimeStamp {
                sec: le_u32(data, 8),
                nsec: le_u32(data, 12),
            },
        },
        quaternion: [
            le_f32(data, 16),
            le_f32(data, 20),
            le_f32(data, 24),
            le_f32(data, 28),
        ],
        angular_velocity: [le_f32(data, 32), le_f32(data, 36), le_f32(data, 40)],
        linear_acceleration: [le_f32(data, 44), le_f32(data, 48), le_f32(data, 52)],
    })
}

fn system_time_seconds() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|duration| duration.as_secs_f64())
        .unwrap_or(0.0)
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

fn work_mode_command_frame(mode: u32) -> [u8; 28] {
    let mode = mode & 0x003f_ffff;
    let data = mode.to_le_bytes();
    let crc = crc32(&data);

    let mut frame = [0u8; 28];
    frame[0..4].copy_from_slice(&FRAME_HEADER);
    frame[4..8].copy_from_slice(&LIDAR_WORK_MODE_COMMAND_PACKET_TYPE.to_le_bytes());
    frame[8..12].copy_from_slice(&28u32.to_le_bytes());
    frame[12..16].copy_from_slice(&data);
    frame[16..20].copy_from_slice(&crc.to_le_bytes());
    frame[26] = 0x00;
    frame[27] = 0xff;
    frame
}

fn user_control_command_frame(cmd_type: u32, cmd_value: u32) -> [u8; 32] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&cmd_type.to_le_bytes());
    data[4..8].copy_from_slice(&cmd_value.to_le_bytes());
    let crc = crc32(&data);

    let mut frame = [0u8; 32];
    frame[0..4].copy_from_slice(&FRAME_HEADER);
    frame[4..8].copy_from_slice(&LIDAR_USER_CMD_PACKET_TYPE.to_le_bytes());
    frame[8..12].copy_from_slice(&32u32.to_le_bytes());
    frame[12..20].copy_from_slice(&data);
    frame[20..24].copy_from_slice(&crc.to_le_bytes());
    frame[30] = 0x00;
    frame[31] = 0xff;
    frame
}

fn udp_config_command_frame(config: &UdpConfig) -> Result<[u8; 44], DirectSerialError> {
    let lidar_ip = ipv4_octets(&config.lidar_ip)?;
    let local_ip = ipv4_octets(&config.local_ip)?;
    let gateway = [0, 0, 0, 0];
    let subnet_mask = [255, 255, 255, 0];

    let mut data = [0u8; 20];
    data[0..4].copy_from_slice(&lidar_ip);
    data[4..8].copy_from_slice(&local_ip);
    data[8..12].copy_from_slice(&gateway);
    data[12..16].copy_from_slice(&subnet_mask);
    data[16..18].copy_from_slice(&config.lidar_port.to_le_bytes());
    data[18..20].copy_from_slice(&config.local_port.to_le_bytes());

    let crc = crc32(&data);

    let mut frame = [0u8; 44];
    frame[0..4].copy_from_slice(&FRAME_HEADER);
    frame[4..8].copy_from_slice(&LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE.to_le_bytes());
    frame[8..12].copy_from_slice(&44u32.to_le_bytes());
    frame[12..32].copy_from_slice(&data);
    frame[32..36].copy_from_slice(&crc.to_le_bytes());
    frame[42] = 0x00;
    frame[43] = 0xff;
    Ok(frame)
}

fn ipv4_octets(address: &str) -> Result<[u8; 4], DirectSerialError> {
    Ipv4Addr::from_str(address)
        .map(|address| address.octets())
        .map_err(|_| DirectSerialError::InvalidIpv4 {
            address: address.to_owned(),
        })
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff;
    for byte in bytes {
        crc ^= *byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xedb8_8320;
            } else {
                crc >>= 1;
            }
        }
    }

    !crc
}
