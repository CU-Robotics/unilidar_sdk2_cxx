pub use crate::direct_serial::{
    DirectSerialError, DirectSerialPacket, DirectSerialRead, SerialPointCloudReader,
    reset_lidar_serial, set_lidar_udp_config_serial, set_lidar_work_mode_serial,
    start_lidar_rotation_serial, stop_lidar_rotation_serial, sync_lidar_timestamp_serial,
};
pub use crate::ffi::{DataInfo, ImuData, Point, PointCloud};

use crate::ffi::LidarWrapper;
use cxx::UniquePtr;
use std::time::Duration;
use thiserror::Error;

mod direct_serial;

pub(crate) const LIDAR_USER_CMD_PACKET_TYPE: u32 = 100;
pub(crate) const LIDAR_ACK_DATA_PACKET_TYPE: u32 = 101;
pub(crate) const LIDAR_POINT_DATA_PACKET_TYPE: u32 = 102;
pub(crate) const LIDAR_2D_POINT_DATA_PACKET_TYPE: u32 = 103;
pub(crate) const LIDAR_IMU_DATA_PACKET_TYPE: u32 = 104;
pub(crate) const LIDAR_VERSION_PACKET_TYPE: u32 = 105;
pub(crate) const LIDAR_TIME_STAMP_PACKET_TYPE: u32 = 106;
pub(crate) const LIDAR_WORK_MODE_CONFIG_PACKET_TYPE: u32 = 107;
pub(crate) const LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE: u32 = 108;
pub(crate) const LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE: u32 = 109;
pub(crate) const LIDAR_PARAM_DATA_PACKET_TYPE: u32 = 2001;

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

    #[derive(Debug, Clone)]
    struct Point {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub intensity: f32,
        pub time: f32,
        pub ring: u32,
    }

    #[derive(Debug, Clone)]
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
        fn syncLidarTimeStamp(self: Pin<&mut LidarWrapper>);
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
    direct_serial: Option<SerialPointCloudReader>,
    direct_serial_config: Option<SerialConfig>,
    direct_serial_cloud: Option<PointCloud>,
    direct_serial_imu: Option<ImuData>,
}

impl UnilidarL2 {
    pub fn new() -> Self {
        Self {
            lidar_wrapper: ffi::createLidarWrapper(),
            direct_serial: None,
            direct_serial_config: None,
            direct_serial_cloud: None,
            direct_serial_imu: None,
        }
    }

    /// Initialize a serial connection to a Unitree L2 lidar. Returns `SerialInitializationError` if the connection couldn't be made.
    pub fn initialize_serial(
        &mut self,
        config: SerialConfig,
    ) -> Result<(), SerialInitializationError> {
        self.initialize_serial_sdk(config)
    }

    /// Initialize the C++ SDK serial path. On some hosts this opens the tty but does not surface
    /// parsed packets; use `initialize_serial_direct` for the direct Rust packet reader.
    pub fn initialize_serial_sdk(
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

    /// Initialize a serial connection using the Rust direct packet reader.
    ///
    /// In this mode, `run_parse` and `try_get_point_cloud` are backed by direct frame parsing, while
    /// control methods such as `start_lidar_rotation`, `stop_lidar_rotation`, `reset_lidar`, and
    /// `set_lidar_work_mode` send command frames over the same serial device.
    pub fn initialize_serial_direct(
        &mut self,
        config: SerialConfig,
    ) -> Result<(), DirectSerialError> {
        self.direct_serial = Some(SerialPointCloudReader::open(config.clone())?);
        self.direct_serial_config = Some(config);
        self.direct_serial_cloud = None;
        self.direct_serial_imu = None;
        Ok(())
    }

    /// Close the serial connection to a Unitree L2 lidar.
    /// Unsure what the return value represents. Possibly `true` if the connection was closed, and `false` if it wasn't.
    pub fn close_serial(&mut self) -> bool {
        self.direct_serial = None;
        self.direct_serial_config = None;
        self.direct_serial_cloud = None;
        self.direct_serial_imu = None;
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
        if let Some(reader) = self.direct_serial.as_mut() {
            return match reader.read_next() {
                Ok(read) => {
                    self.direct_serial_cloud = read.point_cloud;
                    self.direct_serial_imu = read.imu_data;
                    read.packet
                        .map(LidarPacket::from)
                        .unwrap_or(LidarPacket::NoPacket)
                }
                Err(error) => {
                    // A transient read error must not bring down the reader
                    // thread (panicking here poisons the caller's ring mutex and
                    // crashes the app). Report it and let the caller retry.
                    eprintln!("direct serial read failed: {error}");
                    self.direct_serial_cloud = None;
                    self.direct_serial_imu = None;
                    LidarPacket::NoPacket
                }
            };
        }

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
        if let Some(config) = &self.direct_serial_config {
            if let Err(error) = reset_lidar_serial(config.clone()) {
                eprintln!("failed to reset lidar over serial: {error}");
            }
            return;
        }

        self.lidar_wrapper.pin_mut().resetLidar();
    }

    pub fn start_lidar_rotation(&mut self) {
        if let Some(config) = &self.direct_serial_config {
            if let Err(error) = start_lidar_rotation_serial(config.clone()) {
                eprintln!("failed to start lidar rotation over serial: {error}");
            }
            return;
        }

        self.lidar_wrapper.pin_mut().startLidarRotation();
    }

    pub fn stop_lidar_rotation(&mut self) {
        if let Some(config) = &self.direct_serial_config {
            if let Err(error) = stop_lidar_rotation_serial(config.clone()) {
                eprintln!("failed to stop lidar rotation over serial: {error}");
            }
            return;
        }

        self.lidar_wrapper.pin_mut().stopLidarRotation();
    }

    /// Set the lidar work mode. Mode `8` is required after `initialize_serial` for the lidar to
    /// actually emit point/IMU packets over the serial link — without it the connection opens but
    /// stays silent. Other modes exist but are not documented in the SDK.
    pub fn set_lidar_work_mode(&mut self, mode: u32) {
        if let Some(config) = &self.direct_serial_config {
            if let Err(error) = set_lidar_work_mode_serial(config.clone(), mode) {
                eprintln!("failed to set lidar work mode over serial: {error}");
            }
            return;
        }

        self.lidar_wrapper.pin_mut().setLidarWorkMode(mode);
    }

    /// Sync the lidar hardware timestamp to the host system timestamp.
    ///
    /// When `use_system_timestamp` is false, point clouds and IMU samples then use the same
    /// lidar-provided clock instead of mixing host-stamped clouds with hardware-stamped IMU data.
    pub fn sync_lidar_timestamp(&mut self) -> Result<(), DirectSerialError> {
        if let Some(config) = &self.direct_serial_config {
            return sync_lidar_timestamp_serial(config.clone());
        }

        self.lidar_wrapper.pin_mut().syncLidarTimeStamp();
        Ok(())
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
        if self.direct_serial.is_some() {
            return self.direct_serial_cloud.take();
        }

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
        self.try_get_imu_data().unwrap_or(ImuData {
            info: DataInfo {
                seq: 0,
                payload_size: 0,
                stamp: ffi::TimeStamp { sec: 0, nsec: 0 },
            },
            quaternion: [0.0; 4],
            angular_velocity: [0.0; 3],
            linear_acceleration: [0.0; 3],
        })
    }

    /// Gets the latest parsed IMU sample if one is available.
    pub fn try_get_imu_data(&mut self) -> Option<ImuData> {
        if self.direct_serial.is_some() {
            return self.direct_serial_imu.take();
        }

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
        Some(imu_data)
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
#[error("udp initialization err")]
pub struct UdpInitializationError;

impl Default for UnilidarL2 {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
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

/// Configuration for a lidar using a UDP/Ethernet connection. Passed to `UnilidarL2::initialize_udp`.
#[derive(Debug, Clone)]
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
