use std::thread::sleep;
use std::time::{Duration, Instant};
use unilidar_sdk2_cxx::{LidarPacket, LidarPacketCounts, SerialConfig, UnilidarL2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = SerialConfig::from_env_args()?;

    println!(
        "reading IMU data from serial port {} at {} baud",
        config.port, config.baudrate
    );

    let mut lidar = UnilidarL2::new();
    lidar.initialize_serial_direct(config)?;

    sleep(Duration::from_secs(1));
    lidar.start_lidar_rotation();

    let mut counts = LidarPacketCounts::default();
    let mut last_report = Instant::now();

    loop {
        let packet = lidar.run_parse();
        counts.record(&packet);

        match packet {
            LidarPacket::ImuData => {
                if let Some(imu) = lidar.try_get_imu_data() {
                    println!(
                        "seq={} stamp={}.{:09} q=[{:.6}, {:.6}, {:.6}, {:.6}] gyro=[{:.6}, {:.6}, {:.6}] accel=[{:.6}, {:.6}, {:.6}]",
                        imu.info.seq,
                        imu.info.stamp.sec,
                        imu.info.stamp.nsec,
                        imu.quaternion[0],
                        imu.quaternion[1],
                        imu.quaternion[2],
                        imu.quaternion[3],
                        imu.angular_velocity[0],
                        imu.angular_velocity[1],
                        imu.angular_velocity[2],
                        imu.linear_acceleration[0],
                        imu.linear_acceleration[1],
                        imu.linear_acceleration[2],
                    );
                }
            }
            LidarPacket::NoPacket => {
                sleep(Duration::from_micros(100));
            }
            _ => {}
        }

        if last_report.elapsed().as_secs() >= 1 {
            eprintln!("{counts}");
            last_report = Instant::now();
        }
    }
}
