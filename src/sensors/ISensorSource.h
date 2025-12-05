#pragma once
#include <vector>
#include "util/Types.h"

struct LidarScan {
    std::vector<float> ranges;
    float angle_min = 0.0f;
    float angle_max = 6.28318f;  // 2*PI
};

class ISensorSource {
public:
    virtual ~ISensorSource() = default;
    virtual LidarScan getLidarScan() = 0;
    virtual Pose2D getPose() = 0;
    virtual ImuData getImu() = 0;
    virtual float getBatteryPercent() = 0;
};
