#include "SimSensorSource.h"

SimSensorSource::SimSensorSource(NavSim& sim, int num_lidar_rays)
    : sim_(sim), num_lidar_rays_(num_lidar_rays) {
}

LidarScan SimSensorSource::getLidarScan() {
    return sim_.simulateLidar(num_lidar_rays_);
}

Pose2D SimSensorSource::getPose() {
    return sim_.getRobotPose();
}

ImuData SimSensorSource::getImu() {
    // Simulated IMU - return pose orientation as yaw
    ImuData imu;
    imu.yaw = sim_.getRobotPose().theta;
    return imu;
}

float SimSensorSource::getBatteryPercent() {
    // Simulated battery - always full in simulation
    return 100.0f;
}
