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
