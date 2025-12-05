#pragma once

#include "sensors/ISensorSource.h"
#include "NavSim.h"

class SimSensorSource : public ISensorSource {
public:
    explicit SimSensorSource(NavSim& sim, int num_lidar_rays = 360);

    LidarScan getLidarScan() override;
    Pose2D getPose() override;
    ImuData getImu() override;
    float getBatteryPercent() override;

private:
    NavSim& sim_;
    int num_lidar_rays_;
};
