#pragma once

#include <functional>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include "sensors/ISensorSource.h"  // For LidarScan, ImuData

/**
 * LivoxLidar - Wrapper for Livox Mid-360 LiDAR on G1 robot
 *
 * The G1's head-mounted LiDAR is a Livox Mid-360 at IP 192.168.123.120.
 * This class uses Livox SDK2 to receive point cloud data and converts
 * it to the LidarScan format used by the rest of the system.
 */
class LivoxLidar {
public:
    LivoxLidar();
    ~LivoxLidar();

    // Non-copyable
    LivoxLidar(const LivoxLidar&) = delete;
    LivoxLidar& operator=(const LivoxLidar&) = delete;

    /**
     * Initialize the Livox SDK and start receiving data
     * @param config_path Path to mid360_config.json (default: config/mid360_config.json)
     * @return true on success
     */
    bool init(const std::string& config_path = "");

    /**
     * Stop receiving data and uninitialize SDK
     */
    void shutdown();

    /**
     * Check if LiDAR is connected and receiving data
     */
    bool isConnected() const { return connected_.load(); }

    /**
     * Set callback for processed LidarScan data
     */
    void setLidarCallback(std::function<void(const LidarScan&)> callback);

    /**
     * Set callback for raw IMU data from the LiDAR (Mid-360 has built-in IMU)
     */
    void setImuCallback(std::function<void(const ImuData&)> callback);

    /**
     * Get latest scan (thread-safe copy)
     */
    LidarScan getLatestScan() const;

    /**
     * Get the LiDAR's handle (for SDK operations)
     */
    uint32_t getHandle() const { return lidar_handle_.load(); }

    // Called by C callbacks (public for access from anonymous namespace)
    void processPointCloud(uint32_t handle, const uint8_t dev_type, void* data);
    void processImu(uint32_t handle, void* data);
    void onLidarConnected(uint32_t handle);

private:

    // State
    std::atomic<bool> initialized_{false};
    std::atomic<bool> connected_{false};
    std::atomic<uint32_t> lidar_handle_{0};

    // Latest data
    mutable std::mutex scan_mutex_;
    LidarScan latest_scan_;

    // Callbacks
    std::function<void(const LidarScan&)> lidar_callback_;
    std::function<void(const ImuData&)> imu_callback_;

    // Point accumulation for building scans
    std::mutex accumulator_mutex_;
    std::vector<float> accumulated_ranges_;
    std::vector<int> range_counts_;
    uint8_t last_frame_cnt_{0};

    // Statistics
    std::atomic<uint64_t> packet_count_{0};
    std::atomic<uint64_t> point_count_{0};
};
