#pragma once

#include <functional>
#include <mutex>
#include <atomic>
#include <chrono>
#include <string>
#include <memory>
#include "ISensorSource.h"
#include "util/Types.h"

// Forward declaration
class LivoxLidar;

class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    // Initialize SDK channel subscribers
    // Returns true on success, false on failure
    // Set skip_lidar=true to skip Livox LiDAR initialization (for WiFi networks)
    bool init(const std::string& network_interface = "", bool skip_lidar = false);

    // Register callbacks for asynchronous data
    void setLidarCallback(std::function<void(const LidarScan&)> callback);         // 2D scan for SLAM
    void setPointCloud3DCallback(std::function<void(const PointCloud3D&)> callback); // Full 3D for recording
    void setImuCallback(std::function<void(const ImuData&)> callback);
    void setMotorStateCallback(std::function<void(const MotorState&)> callback);

    // Blocking getters for latest data (const-qualified, uses mutable mutex)
    LidarScan getLatestLidar() const;
    ImuData getLatestImu() const;

    // Battery state
    float getBatteryPercent() const;

    // Simple pose estimate from IMU yaw + velocity integration (dead reckoning)
    // Full SLAM provides better estimates - this is basic odometry
    Pose2D getEstimatedPose() const;

    // Update commanded velocity for odometry integration (called from teleop)
    // This integrates velocity over time to estimate position
    void updateVelocity(float vx, float vy, float omega);

    // Connection status
    bool isConnected() const;

    // Check and attempt reconnection if disconnected
    bool checkConnection();
    bool reconnect();

    // Get raw wireless remote data (40 bytes) for gamepad teleop (Story 2-1)
    // Returns true if data is available, false otherwise
    // Caller should provide a 40-byte buffer
    bool getRawWirelessRemote(uint8_t out_buffer[40]) const;

private:
    // Thread-safe data storage (mutable for const getters)
    mutable std::mutex lidar_mutex_;
    mutable std::mutex imu_mutex_;

    LidarScan latest_lidar_;
    ImuData latest_imu_;
    std::atomic<float> battery_percent_{0.0f};
    std::atomic<bool> connected_{false};

    // Wireless remote data for gamepad (Story 2-1)
    mutable std::mutex remote_mutex_;
    uint8_t wireless_remote_[40] = {0};
    std::atomic<bool> remote_available_{false};

    // Callbacks
    std::function<void(const LidarScan&)> lidar_callback_;
    std::function<void(const PointCloud3D&)> pointcloud3d_callback_;
    std::function<void(const ImuData&)> imu_callback_;
    std::function<void(const MotorState&)> motor_state_callback_;

    // Connection tracking
    std::chrono::steady_clock::time_point last_data_time_;
    std::string network_interface_;

    // Odometry integration for position estimation
    mutable std::mutex odom_mutex_;
    Pose2D estimated_pose_{0.0f, 0.0f, 0.0f};  // Integrated position
    float cmd_vx_{0.0f}, cmd_vy_{0.0f}, cmd_omega_{0.0f};  // Commanded velocities
    std::chrono::steady_clock::time_point last_odom_time_;

    // SDK-specific members (only compiled when SDK available)
#ifdef HAS_UNITREE_SDK2
    class Impl;
    std::unique_ptr<Impl> impl_;
#endif

    // Livox LiDAR (Mid-360 on G1 robot head)
    std::unique_ptr<LivoxLidar> livox_lidar_;
    bool use_livox_{false};

    // Internal callback handlers
    void onLidarData(const LidarScan& scan);
    void onPointCloud3D(const PointCloud3D& cloud);
    void onImuData(const ImuData& data);
    void onMotorState(const MotorState& data);
    void onWirelessRemote(const uint8_t* data);  // Story 2-1
};
