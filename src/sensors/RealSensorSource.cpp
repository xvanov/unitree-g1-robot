#include "sensors/RealSensorSource.h"
#include <cmath>
#include <iostream>

RealSensorSource::RealSensorSource(std::shared_ptr<SensorManager> sensor_manager)
    : sensor_manager_(std::move(sensor_manager))
    , last_pose_update_(std::chrono::steady_clock::now())
{
    // Register IMU callback for pose updates
    sensor_manager_->setImuCallback([this](const ImuData& imu) {
        updatePoseFromImu(imu);
    });
}

LidarScan RealSensorSource::getLidarScan() {
    // Check connection status - warn if disconnected (returning stale data)
    if (!sensor_manager_->isConnected()) {
        static bool warned = false;
        if (!warned) {
            std::cerr << "[RealSensorSource] WARNING: Sensor disconnected, returning stale LiDAR data" << std::endl;
            warned = true;
        }
    }
    return sensor_manager_->getLatestLidar();
}

Pose2D RealSensorSource::getPose() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return estimated_pose_;
}

bool RealSensorSource::isConnected() const {
    return sensor_manager_->isConnected();
}

ImuData RealSensorSource::getImu() {
    return sensor_manager_->getLatestImu();
}

float RealSensorSource::getBatteryPercent() {
    return sensor_manager_->getBatteryPercent();
}

void RealSensorSource::updatePoseFromImu(const ImuData& imu) {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_pose_update_).count();
    last_pose_update_ = now;

    // Clamp dt to prevent large jumps
    if (dt > 0.5f || dt <= 0.0f) {
        // Skip position update if dt is invalid, but still update orientation
        estimated_pose_.theta = imu.yaw;
        last_imu_ = imu;
        return;
    }

    // Update orientation from IMU yaw directly
    // NOTE: For MVP, we use IMU yaw rather than integrating gyro
    // Full SLAM will provide better pose estimation
    estimated_pose_.theta = imu.yaw;

    // Basic dead-reckoning for position using IMU acceleration
    // This is approximate - full SLAM localization will be more accurate
    //
    // Transform body-frame acceleration to world frame using yaw
    float cos_yaw = std::cos(imu.yaw);
    float sin_yaw = std::sin(imu.yaw);

    // Body-frame to world-frame rotation (only yaw, assuming level ground)
    float world_ax = imu.accel_x * cos_yaw - imu.accel_y * sin_yaw;
    float world_ay = imu.accel_x * sin_yaw + imu.accel_y * cos_yaw;

    // Simple deadband filter to reduce drift from noise
    // Only integrate if acceleration is significant (> 0.1 m/s^2)
    constexpr float ACCEL_DEADBAND = 0.1f;
    if (std::abs(world_ax) < ACCEL_DEADBAND) world_ax = 0.0f;
    if (std::abs(world_ay) < ACCEL_DEADBAND) world_ay = 0.0f;

    // Double integrate acceleration to get position
    // v = v0 + a*dt, x = x0 + v*dt
    velocity_x_ += world_ax * dt;
    velocity_y_ += world_ay * dt;

    // Apply velocity decay to reduce drift when stationary
    constexpr float VELOCITY_DECAY = 0.98f;
    velocity_x_ *= VELOCITY_DECAY;
    velocity_y_ *= VELOCITY_DECAY;

    // Clamp velocity to reasonable bounds for indoor walking
    constexpr float MAX_VELOCITY = 1.0f;  // m/s
    velocity_x_ = std::clamp(velocity_x_, -MAX_VELOCITY, MAX_VELOCITY);
    velocity_y_ = std::clamp(velocity_y_, -MAX_VELOCITY, MAX_VELOCITY);

    // Update position
    estimated_pose_.x += velocity_x_ * dt;
    estimated_pose_.y += velocity_y_ * dt;

    last_imu_ = imu;
}
