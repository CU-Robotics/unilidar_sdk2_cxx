#include "lidar_wrapper.h"
#include "unilidar_sdk2_cxx/src/lib.rs.h"

std::unique_ptr<LidarWrapper> createLidarWrapper() {
    return std::make_unique<LidarWrapper>();
};

LidarWrapper::LidarWrapper() :  lidarReader(createUnitreeLidarReader()), pointCloud(PointCloudUnitree{}) {};

int LidarWrapper::initializeSerial(
    rust::string port,
    uint32_t baudrate,
    uint16_t cloud_scan_num,
    bool use_system_timestamp,
    float range_min,
    float range_max
) {
    return this->lidarReader->initializeSerial(std::string(port), baudrate, cloud_scan_num, use_system_timestamp, range_min, range_max);
};

int LidarWrapper::initializeUDP(
    uint16_t lidar_port,
    rust::string lidar_ip,
    uint16_t local_port,
    rust::string local_ip,
    uint16_t cloud_scan_num,
    bool use_system_timestamp,
    float range_min,
    float range_max
) {
    return this->lidarReader->initializeUDP(
        lidar_port,
        std::string(lidar_ip),
        local_port,
        std::string(local_ip),
        cloud_scan_num,
        use_system_timestamp,
        range_min,
        range_max
    );
}

bool LidarWrapper::closeSerial() {
    return this->lidarReader->closeSerial();
}

bool LidarWrapper::closeUDP() {
    return this->lidarReader->closeUDP();
}

int LidarWrapper::runParse() {
    return this->lidarReader->runParse();
}

void LidarWrapper::resetLidar() {
    this->lidarReader->resetLidar();
}

void LidarWrapper::startLidarRotation() {
    this->lidarReader->startLidarRotation();
}

void LidarWrapper::stopLidarRotation() {
    this->lidarReader->stopLidarRotation();
}

void LidarWrapper::setLidarWorkMode(uint32_t mode) {
    this->lidarReader->setLidarWorkMode(mode);
}

void LidarWrapper::getPointCloud(PointCloud& rustPointCloud) {
    this->lidarReader->getPointCloud(this->pointCloud);
    rustPointCloud.stamp = this->pointCloud.stamp;
    rustPointCloud.id = this->pointCloud.id;
    rustPointCloud.ring_num = this->pointCloud.ringNum;

    rustPointCloud.points.clear();

    for (const PointUnitree& cppPoint : this->pointCloud.points) {
        Point rustPoint {};
        rustPoint.x = cppPoint.x;
        rustPoint.y = cppPoint.y;
        rustPoint.z = cppPoint.z;
        rustPoint.intensity = cppPoint.intensity;
        rustPoint.ring = cppPoint.ring;
        rustPoint.time = cppPoint.time;

        rustPointCloud.points.push_back(rustPoint);
    }
}

void LidarWrapper::getImuData(ImuData& rustImuData) {
     this->lidarReader->getImuData(this->imuData);

     rustImuData.info.stamp.sec = this->imuData.info.stamp.sec;
     rustImuData.info.stamp.nsec = this->imuData.info.stamp.nsec;
     rustImuData.info.payload_size = this->imuData.info.payload_size;
     rustImuData.info.seq = this->imuData.info.seq;

     rustImuData.quaternion[0] = this->imuData.quaternion[0];
     rustImuData.quaternion[1] = this->imuData.quaternion[1];
     rustImuData.quaternion[2] = this->imuData.quaternion[2];
     rustImuData.quaternion[3] = this->imuData.quaternion[3];

     rustImuData.angular_velocity[0] = this->imuData.angular_velocity[0];
     rustImuData.angular_velocity[1] = this->imuData.angular_velocity[1];
     rustImuData.angular_velocity[2] = this->imuData.angular_velocity[2];

     rustImuData.linear_acceleration[0] = this->imuData.linear_acceleration[0];
     rustImuData.linear_acceleration[1] = this->imuData.linear_acceleration[1];
     rustImuData.linear_acceleration[2] = this->imuData.linear_acceleration[2];
}
