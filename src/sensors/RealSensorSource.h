#pragma once

#include "ISensorSource.h"
#include "SensorManager.h"
#include <memory>
#include <atomic>

// Implements ISensorSource using real robot hardware via SensorManager
class RealSensorSource : public ISensorSource {
public:
    // Initialize with a SensorManager instance
    explicit RealSensorSource(std::shared_ptr<SensorManager> sensor_manager);
    ~RealSensorSource() override = default;

    // ISensorSource interface
    LidarScan getLidarScan() override;
    Pose2D getPose() override;

    // ISensorSource interface (continued)
    ImuData getImu() override;
    float getBatteryPercent() override;

    // Additional methods for real sensor source
    bool isConnected() const;

private:
    std::shared_ptr<SensorManager> sensor_manager_;

    // Pose estimation from IMU integration (simple dead reckoning for MVP)
    mutable std::mutex pose_mutex_;
    Pose2D estimated_pose_;
    ImuData last_imu_;
    std::chrono::steady_clock::time_point last_pose_update_;

    // Velocity estimates for dead-reckoning position
    float velocity_x_ = 0.0f;
    float velocity_y_ = 0.0f;

    // Update pose from IMU (simple integration)
    void updatePoseFromImu(const ImuData& imu);
};
