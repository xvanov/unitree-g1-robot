#pragma once
#include <vector>
#include <cstddef>
#include <cstdint>
#include "util/Types.h"

// 2D LiDAR scan (for SLAM compatibility)
struct LidarScan {
    std::vector<float> ranges;
    float angle_min = 0.0f;
    float angle_max = 6.28318f;  // 2*PI
};

// 3D point cloud for full reconstruction
struct PointCloud3D {
    std::vector<float> x;      // X coordinates (meters)
    std::vector<float> y;      // Y coordinates (meters)
    std::vector<float> z;      // Z coordinates (meters)
    std::vector<uint8_t> intensity;  // Reflectivity (optional, 0 if not available)

    size_t size() const { return x.size(); }
    void clear() { x.clear(); y.clear(); z.clear(); intensity.clear(); }
    void reserve(size_t n) { x.reserve(n); y.reserve(n); z.reserve(n); intensity.reserve(n); }
};

class ISensorSource {
public:
    virtual ~ISensorSource() = default;
    virtual LidarScan getLidarScan() = 0;
    virtual Pose2D getPose() = 0;
    virtual ImuData getImu() = 0;
    virtual float getBatteryPercent() = 0;
};
