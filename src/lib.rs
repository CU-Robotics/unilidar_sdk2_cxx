pub use crate::ffi::{DataInfo, ImuData, Point, PointCloud};

use crate::ffi::LidarWrapper;
use cxx::UniquePtr;
use std::env;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::net::Ipv4Addr;
use std::num::ParseIntError;
use std::path::Path;
use std::process::Command;
use std::str::FromStr;
use std::time::Duration;
use thiserror::Error;

const FRAME_HEADER: [u8; 4] = [0x55, 0xaa, 0x05, 0x0a];
const LIDAR_POINT_DATA_PACKET_TYPE: u32 = 102;
const LIDAR_2D_POINT_DATA_PACKET_TYPE: u32 = 103;
const LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE: u32 = 108;
const LIDAR_WORK_MODE_COMMAND_PACKET_TYPE: u32 = 2002;
const MAX_FRAME_SIZE: usize = 6_000;
const DEFAULT_SERIAL_PORT: &str = "/dev/ttyACM0";
const LIDAR_SERIAL_BY_ID: &str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A2A026768-if00";

#[cxx::bridge]
mod ffi {

    #[derive(Debug, Clone, Copy)]
    /// Data recieved from the IMU.
    ///
    /// `angular_velocity` and `linear_acceleration` are probably [x, y, z].
    ///
    /// `quaternion` might be [w, x, y, z]. Don't know for sure. Could also be [x, y, z, w]
    struct ImuData {
        pub info: DataInfo,
        pub quaternion: [f32; 4],
        pub angular_velocity: [f32; 3],
        pub linear_acceleration: [f32; 3],
    }

    #[derive(Debug, Clone, Copy)]
    /// Generic packet information attached to `ImuData` and some other (not yet translated) structs.
    ///
    /// `seq` is the number of the packet, probably increasing with each one sent.
    ///
    /// `payload_size` is probably the size of the full packet in bytes.
    ///
    /// `stamp` is the time of the message.
    struct DataInfo {
        pub seq: u32,
        pub payload_size: u32,
        pub stamp: TimeStamp,
    }

    #[derive(Debug, Clone, Copy)]
    struct TimeStamp {
        pub sec: u32,
        pub nsec: u32,
    }

    #[derive(Debug)]
    struct Point {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub intensity: f32,
        pub time: f32,
        pub ring: u32,
    }

    #[derive(Debug)]
    struct PointCloud {
        pub stamp: f64,
        pub id: u32,
        pub ring_num: u32,
        pub points: Vec<Point>,
    }

    unsafe extern "C++" {
        include!("unilidar_sdk2_cxx/include/lidar_wrapper.h");

        type LidarWrapper;

        fn createLidarWrapper() -> UniquePtr<LidarWrapper>;
        //fn getValue(self: &MyClass) -> String;
        fn initializeSerial(
            self: Pin<&mut LidarWrapper>,
            port: String,
            baudrate: u32,
            cloud_scan_num: u16,
            use_system_timestamp: bool,
            range_min: f32,
            range_max: f32,
        ) -> i32;
        fn initializeUDP(
            self: Pin<&mut LidarWrapper>,
            lidar_port: u16,
            lidar_ip: String,
            local_port: u16,
            local_ip: String,
            cloud_scan_num: u16,
            use_system_timestamp: bool,
            range_min: f32,
            range_max: f32,
        ) -> i32;
        fn closeSerial(self: Pin<&mut LidarWrapper>) -> bool;
        fn closeUDP(self: Pin<&mut LidarWrapper>) -> bool;
        fn runParse(self: Pin<&mut LidarWrapper>) -> i32;
        fn resetLidar(self: Pin<&mut LidarWrapper>);
        fn startLidarRotation(self: Pin<&mut LidarWrapper>);
        fn stopLidarRotation(self: Pin<&mut LidarWrapper>);
        fn setLidarWorkMode(self: Pin<&mut LidarWrapper>, mode: u32);
        fn getPointCloud(self: Pin<&mut LidarWrapper>, rustPointCloud: &mut PointCloud) -> bool;
        fn getImuData(self: Pin<&mut LidarWrapper>, rustImuData: &mut ImuData);
    }
}

#[derive(Debug, PartialEq, Eq)]
/// A representation of all the possible packets received from the lidar.
///
/// The provided C++ SDK documentation is not that great. I've done my best to infer what each packet is, but mistakes are possible.
pub enum LidarPacket {
    /// Presumably corresponds to a command from the user. Unsure if it appears.
    UserCmd,
    /// Sent by the lidar when it receives a command from the user.
    AckData,
    /// A 3D point-cloud packet (LIDAR_POINT_DATA_PACKET_TYPE = 102). Call `get_point_cloud` to
    /// read it. This is the packet type emitted in the L2's default 3D work mode.
    PointData,
    /// A 2D point-cloud packet (LIDAR_2D_POINT_DATA_PACKET_TYPE = 103). Only emitted when the
    /// lidar is in 2D mode.
    PointData2D,
    /// Received data from the IMU. This bindings library currently does not have bindings for getting this.
    ImuData,
    /// Got the lidar version. Likely sent after the user runs the get lidar version function. (That function currently has no Rust binding)
    LidarVersion,
    /// A timestamp is attached to some packets sent by the lidar. Unsure of what just a timestamp means. Don't know if this is even sent.
    TimeStamp,
    /// Possibly sent when the lidar work mode changes.
    /// No clue what work mode is. Possibly related to the lidar being able to do 180 and 192 degree scans?
    WorkModeConfig,
    /// Presumably sent when the ip address changes.
    IpAddressConfig,
    /// Presumably sent when the mac address changes.
    MacAddressConfig,
    /// No idea what this means or if it's even sent.
    ParamData,
    /// Undocumented, but `0` is frequently sent by the lidar. Presumably means that there's nothing new.
    NoPacket,
}

/// A wrapper around the C++ Unilidar SDK2. Only some of the functions are translated.
pub struct UnilidarL2 {
    lidar_wrapper: UniquePtr<LidarWrapper>,
}

impl UnilidarL2 {
    pub fn new() -> Self {
        Self {
            lidar_wrapper: ffi::createLidarWrapper(),
        }
    }

    /// Initialize a serial connection to a Unitree L2 lidar. Returns `SerialInitializationError` if the connection couldn't be made.
    pub fn initialize_serial(
        &mut self,
        config: SerialConfig,
    ) -> Result<(), SerialInitializationError> {
        match self.lidar_wrapper.pin_mut().initializeSerial(
            config.port,
            config.baudrate,
            config.cloud_scan_num,
            config.use_system_timestamp,
            config.range_min,
            config.range_max,
        ) {
            0 => Ok(()),
            -1 => Err(SerialInitializationError),
            _ => unreachable!(),
        }
    }

    /// Close the serial connection to a Unitree L2 lidar.
    /// Unsure what the return value represents. Possibly `true` if the connection was closed, and `false` if it wasn't.
    pub fn close_serial(&mut self) -> bool {
        self.lidar_wrapper.pin_mut().closeSerial()
    }

    /// Initialize a UDP/Ethernet connection to a Unitree L2 lidar.
    pub fn initialize_udp(&mut self, config: UdpConfig) -> Result<(), UdpInitializationError> {
        match self.lidar_wrapper.pin_mut().initializeUDP(
            config.lidar_port,
            config.lidar_ip,
            config.local_port,
            config.local_ip,
            config.cloud_scan_num,
            config.use_system_timestamp,
            config.range_min,
            config.range_max,
        ) {
            0 => Ok(()),
            -1 => Err(UdpInitializationError),
            _ => unreachable!(),
        }
    }

    /// Initialize UDP mode and perform the startup sequence needed for point-cloud packets.
    ///
    /// Unitree's UDP example starts rotation, sets work mode `0`, resets the lidar, then starts
    /// rotation again. Skipping the final start can leave IMU packets flowing without point data.
    pub fn initialize_udp_streaming(
        &mut self,
        config: UdpConfig,
    ) -> Result<(), UdpInitializationError> {
        self.initialize_udp(config)?;
        self.start_lidar_rotation();
        std::thread::sleep(Duration::from_secs(1));

        self.set_lidar_work_mode(UDP_WORK_MODE);
        std::thread::sleep(Duration::from_secs(1));

        self.reset_lidar();
        std::thread::sleep(Duration::from_secs(2));

        self.start_lidar_rotation();
        std::thread::sleep(Duration::from_secs(3));

        Ok(())
    }

    pub fn close_udp(&mut self) -> bool {
        self.lidar_wrapper.pin_mut().closeUDP()
    }

    /// Gets the next packet sent by the lidar and parses it.
    /// The return value is the type of packet received.
    pub fn run_parse(&mut self) -> LidarPacket {
        // These are from unitree_lidar_protocol.h
        const LIDAR_USER_CMD_PACKET_TYPE: i32 = 100;
        const LIDAR_ACK_DATA_PACKET_TYPE: i32 = 101;
        const LIDAR_POINT_DATA_PACKET_TYPE: i32 = 102;
        const LIDAR_2D_POINT_DATA_PACKET_TYPE: i32 = 103;
        const LIDAR_IMU_DATA_PACKET_TYPE: i32 = 104;
        const LIDAR_VERSION_PACKET_TYPE: i32 = 105;
        const LIDAR_TIME_STAMP_PACKET_TYPE: i32 = 106;
        const LIDAR_WORK_MODE_CONFIG_PACKET_TYPE: i32 = 107;
        const LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE: i32 = 108;
        const LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE: i32 = 109;

        // This packet type is defined in unitree_lidar_protocol.h
        // However, 1. I couldn't figure out what it means from just the title, and 2. I did not see it sent it in testing.
        //const LIDAR_COMMAND_PACKET_TYPE = 2000;

        const LIDAR_PARAM_DATA_PACKET_TYPE: i32 = 2001;

        match self.lidar_wrapper.pin_mut().runParse() {
            LIDAR_USER_CMD_PACKET_TYPE => LidarPacket::UserCmd,
            LIDAR_ACK_DATA_PACKET_TYPE => LidarPacket::AckData,
            LIDAR_POINT_DATA_PACKET_TYPE => LidarPacket::PointData,
            LIDAR_2D_POINT_DATA_PACKET_TYPE => LidarPacket::PointData2D,
            LIDAR_IMU_DATA_PACKET_TYPE => LidarPacket::ImuData,
            LIDAR_VERSION_PACKET_TYPE => LidarPacket::LidarVersion,
            LIDAR_TIME_STAMP_PACKET_TYPE => LidarPacket::TimeStamp,
            LIDAR_WORK_MODE_CONFIG_PACKET_TYPE => LidarPacket::WorkModeConfig,
            LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE => LidarPacket::IpAddressConfig,
            LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE => LidarPacket::MacAddressConfig,
            LIDAR_PARAM_DATA_PACKET_TYPE => LidarPacket::ParamData,
            // 0 meaning there's no packet is an assumption.
            // This return value is not documented like the others, but it does occur.
            0 => LidarPacket::NoPacket,
            e => panic!("got {e} shouldnt have"),
        }
    }

    pub fn reset_lidar(&mut self) {
        self.lidar_wrapper.pin_mut().resetLidar();
    }

    pub fn start_lidar_rotation(&mut self) {
        self.lidar_wrapper.pin_mut().startLidarRotation();
    }

    pub fn stop_lidar_rotation(&mut self) {
        self.lidar_wrapper.pin_mut().stopLidarRotation();
    }

    /// Set the lidar work mode. Mode `8` is required after `initialize_serial` for the lidar to
    /// actually emit point/IMU packets over the serial link — without it the connection opens but
    /// stays silent. Other modes exist but are not documented in the SDK.
    pub fn set_lidar_work_mode(&mut self, mode: u32) {
        self.lidar_wrapper.pin_mut().setLidarWorkMode(mode);
    }

    /// Gets the latest parsed point cloud.
    // From a performance standpoint it could be faster to get the PointData2D, copy it to rust, and then parse it
    // from inside rust. Unsure, will have to test.
    pub fn get_point_cloud(&mut self) -> PointCloud {
        self.try_get_point_cloud().unwrap_or(PointCloud {
            stamp: 0.0,
            id: 0,
            ring_num: 0,
            points: Vec::new(),
        })
    }

    /// Gets the latest parsed point cloud if the SDK has accumulated a complete cloud.
    pub fn try_get_point_cloud(&mut self) -> Option<PointCloud> {
        let mut point_cloud = PointCloud {
            stamp: 0.0,
            id: 0,
            ring_num: 0,
            points: Vec::new(),
        };
        if self.lidar_wrapper.pin_mut().getPointCloud(&mut point_cloud) {
            Some(point_cloud)
        } else {
            None
        }
    }

    pub fn get_imu_data(&mut self) -> ImuData {
        let mut imu_data = ImuData {
            info: DataInfo {
                seq: 0,
                payload_size: 0,
                stamp: ffi::TimeStamp { sec: 0, nsec: 0 },
            },
            quaternion: [0.0; 4],
            angular_velocity: [0.0; 3],
            linear_acceleration: [0.0; 3],
        };
        self.lidar_wrapper.pin_mut().getImuData(&mut imu_data);
        imu_data
    }
}

const UDP_WORK_MODE: u32 = 0;

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct LidarPacketCounts {
    pub point_packets: u64,
    pub point_2d_packets: u64,
    pub imu_packets: u64,
    pub ack_packets: u64,
    pub param_packets: u64,
    pub no_packets: u64,
    pub other_packets: u64,
}

impl LidarPacketCounts {
    pub fn record(&mut self, packet: &LidarPacket) {
        match packet {
            LidarPacket::PointData => self.point_packets += 1,
            LidarPacket::PointData2D => self.point_2d_packets += 1,
            LidarPacket::ImuData => self.imu_packets += 1,
            LidarPacket::AckData => self.ack_packets += 1,
            LidarPacket::ParamData => self.param_packets += 1,
            LidarPacket::NoPacket => self.no_packets += 1,
            _ => self.other_packets += 1,
        }
    }
}

impl std::fmt::Display for LidarPacketCounts {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "point_packets={} point_2d_packets={} imu_packets={} ack_packets={} param_packets={} other_packets={} no_packets={}",
            self.point_packets,
            self.point_2d_packets,
            self.imu_packets,
            self.ack_packets,
            self.param_packets,
            self.other_packets,
            self.no_packets
        )
    }
}

#[derive(Error, Debug)]
#[error("initialziation err")]
/// Some sort of serial initialization error. Unfortunately the C++ SDK does not expose more than
/// this. It seems to print the exact cause of the error to the console, though.
///
/// Probably either 1. The specified port wasn't found. 2. No lidar at the specified port.
pub struct SerialInitializationError;

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

#[derive(Error, Debug)]
#[error("udp initialization err")]
pub struct UdpInitializationError;

impl Default for UnilidarL2 {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
/// Configuration for a lidar using a serial connection. Passed to `UnilidarL2.initialize_serial`
pub struct SerialConfig {
    pub port: String,
    pub baudrate: u32,
    pub cloud_scan_num: u16,
    pub use_system_timestamp: bool,
    pub range_min: f32,
    pub range_max: f32,
}

impl Default for SerialConfig {
    /// The default settings for a serial connection, as specified in the C++ SDK.
    /// The port is set to `/dev/ttyACM0`
    fn default() -> Self {
        Self {
            port: String::from("/dev/ttyACM0"),
            baudrate: 4000000,
            cloud_scan_num: 18,
            use_system_timestamp: true,
            range_min: 0.0,
            range_max: 100.0,
        }
    }
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

#[derive(Debug)]
pub struct DirectSerialRead {
    pub bytes_read: usize,
    pub packet: Option<DirectSerialPacket>,
    pub point_cloud: Option<PointCloud>,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct DirectSerialPacketCounts {
    pub bytes_read: u64,
    pub point_packets: u64,
    pub point_2d_packets: u64,
    pub other_packets: u64,
}

impl DirectSerialPacketCounts {
    pub fn record(&mut self, read: &DirectSerialRead) {
        self.bytes_read += read.bytes_read as u64;

        match read.packet {
            Some(DirectSerialPacket::PointData) => self.point_packets += 1,
            Some(DirectSerialPacket::PointData2D) => self.point_2d_packets += 1,
            Some(DirectSerialPacket::Other(_)) => self.other_packets += 1,
            None => {}
        }
    }
}

impl std::fmt::Display for DirectSerialPacketCounts {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "bytes_read={} point_packets={} point_2d_packets={} other_packets={}",
            self.bytes_read, self.point_packets, self.point_2d_packets, self.other_packets
        )
    }
}

/// Direct serial point-cloud reader for systems where Unitree's precompiled serial `runParse`
/// path opens the tty but never surfaces packets. This reads the documented SDK packet format
/// from the serial device and applies the same 3D point transform as `unitree_lidar_utilities.h`.
pub struct SerialPointCloudReader {
    port: String,
    serial: File,
    read_buf: [u8; 8192],
    frame_buf: Vec<u8>,
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
        })
    }

    pub fn port(&self) -> &str {
        &self.port
    }

    pub fn read_next(&mut self) -> Result<DirectSerialRead, DirectSerialError> {
        loop {
            if let Some(frame) = next_frame(&mut self.frame_buf) {
                return Ok(read_from_frame(&frame));
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
                    let mut read = read_from_frame(&frame);
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

fn read_from_frame(frame: &[u8]) -> DirectSerialRead {
    let packet = packet_type(frame);
    let direct_packet = match packet {
        LIDAR_POINT_DATA_PACKET_TYPE => DirectSerialPacket::PointData,
        LIDAR_2D_POINT_DATA_PACKET_TYPE => DirectSerialPacket::PointData2D,
        other => DirectSerialPacket::Other(other),
    };

    let point_cloud = match direct_packet {
        DirectSerialPacket::PointData => parse_point_cloud(frame),
        DirectSerialPacket::PointData2D | DirectSerialPacket::Other(_) => None,
    };

    DirectSerialRead {
        bytes_read: 0,
        packet: Some(direct_packet),
        point_cloud,
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

fn parse_point_cloud(frame: &[u8]) -> Option<PointCloud> {
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
            if range >= range_min && range <= range_max {
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

/// Configuration for a lidar using a UDP/Ethernet connection. Passed to `UnilidarL2::initialize_udp`.
#[derive(Debug)]
pub struct UdpConfig {
    pub lidar_port: u16,
    pub lidar_ip: String,
    pub local_port: u16,
    pub local_ip: String,
    pub cloud_scan_num: u16,
    pub use_system_timestamp: bool,
    pub range_min: f32,
    pub range_max: f32,
}

impl Default for UdpConfig {
    /// Factory defaults from the L2 user manual: lidar at 192.168.1.62, host at 192.168.1.2,
    /// ports 6101 (lidar tx) and 6201 (host rx).
    fn default() -> Self {
        Self {
            lidar_port: 6101,
            lidar_ip: String::from("192.168.1.62"),
            local_port: 6201,
            local_ip: String::from("192.168.1.2"),
            cloud_scan_num: 18,
            use_system_timestamp: true,
            range_min: 0.0,
            range_max: 100.0,
        }
    }
}
