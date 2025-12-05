#pragma once

#include <functional>
#include <mutex>
#include <atomic>
#include <chrono>
#include <string>
#include <memory>
#include "ISensorSource.h"
#include "util/Types.h"

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

    // Connection status
    bool isConnected() const;

    // Check and attempt reconnection if disconnected
    bool checkConnection();
    bool reconnect();

private:
    // Thread-safe data storage (mutable for const getters)
    mutable std::mutex lidar_mutex_;
    mutable std::mutex imu_mutex_;

    LidarScan latest_lidar_;
    ImuData latest_imu_;
    std::atomic<float> battery_percent_{0.0f};
    std::atomic<bool> connected_{false};

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

    // Internal callback handlers
    void onLidarData(const LidarScan& scan);
    void onImuData(const ImuData& data);
};
