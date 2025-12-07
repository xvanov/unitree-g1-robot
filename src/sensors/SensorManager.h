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
    bool init(const std::string& network_interface = "");

    // Register callbacks for asynchronous data
    void setLidarCallback(std::function<void(const LidarScan&)> callback);
    void setImuCallback(std::function<void(const ImuData&)> callback);

    // Blocking getters for latest data (const-qualified, uses mutable mutex)
    LidarScan getLatestLidar() const;
    ImuData getLatestImu() const;

    // Battery state
    float getBatteryPercent() const;

    // Simple pose estimate from IMU yaw (dead reckoning)
    // Full SLAM provides better estimates - this is basic orientation-only
    Pose2D getEstimatedPose() const;

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
    std::function<void(const ImuData&)> imu_callback_;

    // Connection tracking
    std::chrono::steady_clock::time_point last_data_time_;
    std::string network_interface_;

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
    void onImuData(const ImuData& data);
    void onWirelessRemote(const uint8_t* data);  // Story 2-1
};
