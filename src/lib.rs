pub use crate::ffi::{DataInfo, ImuData, Point, PointCloud};

use crate::ffi::LidarWrapper;
use cxx::UniquePtr;
use thiserror::Error;

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
        fn closeSerial(self: Pin<&mut LidarWrapper>) -> bool;
        fn runParse(self: Pin<&mut LidarWrapper>) -> i32;
        fn resetLidar(self: Pin<&mut LidarWrapper>);
        fn startLidarRotation(self: Pin<&mut LidarWrapper>);
        fn stopLidarRotation(self: Pin<&mut LidarWrapper>);
        fn getPointCloud(self: Pin<&mut LidarWrapper>, rustPointCloud: &mut PointCloud);
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
    /// PointData seems to represent receiving 1/6th of the full lidar scan.
    PointData,
    /// PointData2D seems to represent receiving a full lidar scan. Probably sent once after every six PointData's? Unsure, will need to test.
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

    /// Gets the latest parsed point cloud.
    // From a performance standpoint it could be faster to get the PointData2D, copy it to rust, and then parse it
    // from inside rust. Unsure, will have to test.
    pub fn get_point_cloud(&mut self) -> PointCloud {
        let mut point_cloud = PointCloud {
            stamp: 0.0,
            id: 0,
            ring_num: 0,
            points: Vec::new(),
        };
        self.lidar_wrapper.pin_mut().getPointCloud(&mut point_cloud);
        point_cloud
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

#[derive(Error, Debug)]
#[error("initialziation err")]
/// Some sort of serial initialization error. Unfortunately the C++ SDK does not expose more than
/// this. It seems to print the exact cause of the error to the console, though.
///
/// Probably either 1. The specified port wasn't found. 2. No lidar at the specified port.
pub struct SerialInitializationError;

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
    pub fn port(mut self, port: impl Into<String>) -> Self {
        self.port = port.into();
        self
    }
}
