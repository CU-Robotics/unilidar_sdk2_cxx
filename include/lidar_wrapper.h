#include <memory>
#include <string>
#include "../unitree_lidar_sdk/include/unitree_lidar_sdk.h"
#include "rust/cxx.h"

#pragma once

struct PointCloud;
struct ImuData;

using namespace unilidar_sdk2;

class LidarWrapper {
private:
    std::unique_ptr<UnitreeLidarReader> lidarReader;
    PointCloudUnitree pointCloud;
    LidarImuData imuData;
public:
    LidarWrapper();

    int initializeSerial(
        rust::string port,
        uint32_t baudrate,
        uint16_t cloud_scan_num,
        bool use_system_timestamp,
        float range_min,
        float range_max
    );

    bool closeSerial();
    int runParse();
    void resetLidar();
    void startLidarRotation();
    void stopLidarRotation();
    void getPointCloud(PointCloud& rustPointCloud);
    void getImuData(ImuData& rustImuData);
};

std::unique_ptr<LidarWrapper> createLidarWrapper();
