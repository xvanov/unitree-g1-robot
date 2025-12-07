#include "sensors/SensorManager.h"
#include "sensors/LivoxLidar.h"
#include "util/NetworkUtil.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <thread>

#ifdef HAS_UNITREE_SDK2
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/ros2/PointCloud2_.hpp"

// LiDAR processing constants
namespace LidarConfig {
    constexpr int NUM_RAYS = 360;           // Number of angular bins
    constexpr float MAX_RANGE = 10.0f;      // Maximum valid range (meters)
    constexpr float MIN_RANGE = 0.1f;       // Minimum valid range (meters)
    constexpr float ANGLE_MIN = 0.0f;       // Start angle (radians)
    constexpr float ANGLE_MAX = 2.0f * M_PI; // End angle (radians)
}

// Convert PointCloud2 to LidarScan
static LidarScan parsePointCloud(const sensor_msgs::msg::dds_::PointCloud2_& cloud) {
    LidarScan scan;
    scan.angle_min = LidarConfig::ANGLE_MIN;
    scan.angle_max = LidarConfig::ANGLE_MAX;

    scan.ranges.resize(LidarConfig::NUM_RAYS, LidarConfig::MAX_RANGE);

    std::vector<float> min_ranges(LidarConfig::NUM_RAYS, LidarConfig::MAX_RANGE);

    const size_t point_step = cloud.point_step();
    const size_t num_points = cloud.width() * cloud.height();
    const uint8_t* data = cloud.data().data();

    // Find x,y field offsets
    size_t x_offset = 0;
    size_t y_offset = 4;
    for (const auto& field : cloud.fields()) {
        if (field.name() == "x") x_offset = field.offset();
        if (field.name() == "y") y_offset = field.offset();
    }

    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* point_data = data + i * point_step;

        float x, y;
        std::memcpy(&x, point_data + x_offset, sizeof(float));
        std::memcpy(&y, point_data + y_offset, sizeof(float));

        if (!std::isfinite(x) || !std::isfinite(y)) continue;

        float range = std::sqrt(x * x + y * y);
        if (range < LidarConfig::MIN_RANGE || range > LidarConfig::MAX_RANGE) continue;

        float angle = std::atan2(y, x);
        if (angle < 0) angle += LidarConfig::ANGLE_MAX;

        int index = static_cast<int>(angle / LidarConfig::ANGLE_MAX * LidarConfig::NUM_RAYS);
        index = std::clamp(index, 0, LidarConfig::NUM_RAYS - 1);

        if (range < min_ranges[index]) {
            min_ranges[index] = range;
        }
    }

    scan.ranges = min_ranges;
    return scan;
}

// PIMPL for SDK-specific implementation
class SensorManager::Impl {
public:
    // Subscribers are constructed with channel name
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> state_sub_;
    std::unique_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>> lidar_sub_;

    SensorManager* parent_ = nullptr;

    void onLowState(const void* data) {
        if (!data || !parent_) return;
        const auto* msg = static_cast<const unitree_hg::msg::dds_::LowState_*>(data);

        ImuData imu;
        const auto& imu_state = msg->imu_state();

        // RPY from IMU
        imu.roll = imu_state.rpy()[0];
        imu.pitch = imu_state.rpy()[1];
        imu.yaw = imu_state.rpy()[2];

        // Angular velocity (gyroscope)
        imu.gyro_x = imu_state.gyroscope()[0];
        imu.gyro_y = imu_state.gyroscope()[1];
        imu.gyro_z = imu_state.gyroscope()[2];

        // Linear acceleration
        imu.accel_x = imu_state.accelerometer()[0];
        imu.accel_y = imu_state.accelerometer()[1];
        imu.accel_z = imu_state.accelerometer()[2];

        // NOTE: G1 LowState_ does not include battery data.
        // Battery monitoring requires subscribing to a separate BMS topic
        // (not implemented in MVP - getBatteryPercent() returns 0)

        parent_->onImuData(imu);

        // Store wireless remote data for teleop (Story 2-1)
        const auto& remote = msg->wireless_remote();
        parent_->onWirelessRemote(remote.data());
    }

    void onPointCloud(const void* data) {
        if (!data || !parent_) return;
        const auto* msg = static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(data);
        LidarScan scan = parsePointCloud(*msg);
        parent_->onLidarData(scan);
    }
};

#endif

SensorManager::SensorManager()
    : last_data_time_(std::chrono::steady_clock::now())
{
#ifdef HAS_UNITREE_SDK2
    impl_ = std::make_unique<Impl>();
    impl_->parent_ = this;
#endif
}

SensorManager::~SensorManager() = default;

bool SensorManager::init(const std::string& network_interface, bool skip_lidar) {
#ifdef HAS_UNITREE_SDK2
    try {
        // Store network interface for reference
        network_interface_ = network_interface.empty() ? NetworkUtil::findRobotInterface() : network_interface;

        // IMPORTANT: Initialize Livox SDK BEFORE Unitree SDK's ChannelFactory
        // Both SDKs use network sockets and there may be initialization conflicts
        // Livox must be first as it sets up UDP listeners that could conflict with DDS
        const char* skip_livox = std::getenv("SKIP_LIVOX");
        if (skip_lidar) {
            std::cout << "[SENSORS] Livox LiDAR skipped (--no-lidar)" << std::endl;
            use_livox_ = false;
        } else if (skip_livox && std::string(skip_livox) == "1") {
            std::cout << "[SENSORS] Livox LiDAR skipped (SKIP_LIVOX=1)" << std::endl;
            use_livox_ = false;
        } else {
            try {
                livox_lidar_ = std::make_unique<LivoxLidar>();
                if (livox_lidar_->init()) {
                    use_livox_ = true;
                    // Wire Livox callback to our LiDAR handler
                    livox_lidar_->setLidarCallback([this](const LidarScan& scan) {
                        onLidarData(scan);
                    });
                    std::cout << "[SENSORS] Livox Mid-360 LiDAR initialized" << std::endl;
                } else {
                    std::cout << "[SENSORS] Livox LiDAR init returned false, continuing without LiDAR" << std::endl;
                    livox_lidar_.reset();
                    use_livox_ = false;
                }
            } catch (const std::exception& e) {
                std::cerr << "[SENSORS] Livox LiDAR exception: " << e.what() << std::endl;
                livox_lidar_.reset();
                use_livox_ = false;
            } catch (...) {
                std::cerr << "[SENSORS] Livox LiDAR unknown exception" << std::endl;
                livox_lidar_.reset();
                use_livox_ = false;
            }
        }

        // Initialize ChannelFactory using centralized singleton
        // Safe to call multiple times - only initializes once across all SDK users
        if (!NetworkUtil::initChannelFactory(network_interface_)) {
            std::cerr << "[SENSORS] Failed to initialize SDK ChannelFactory" << std::endl;
            return false;
        }

        // Create LowState subscriber (for IMU and wireless remote)
        auto state_handler = [this](const void* data) { impl_->onLowState(data); };
        impl_->state_sub_ = std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
            "rt/lowstate", state_handler);
        impl_->state_sub_->InitChannel();
        std::cout << "[SENSORS] LowState subscriber initialized" << std::endl;

        // If Livox not available, try DDS fallback
        if (!use_livox_) {
            // Fallback: Create DDS LiDAR subscriber (may not work on G1)
            auto lidar_handler = [this](const void* data) { impl_->onPointCloud(data); };
            impl_->lidar_sub_ = std::make_unique<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(
                "rt/utlidar/cloud", lidar_handler);
            impl_->lidar_sub_->InitChannel();
            std::cout << "[SENSORS] DDS LiDAR subscriber initialized (fallback)" << std::endl;
        }

        // Brief settling time for DDS discovery before waiting for data
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Wait for initial connection (timeout after 5 seconds)
        // Note: Even if DDS times out, Livox LiDAR may still be working
        std::cout << "[SENSORS] Waiting for sensor data..." << std::endl;
        auto start = std::chrono::steady_clock::now();
        while (!connected_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed > std::chrono::seconds(5)) {
                if (use_livox_ && livox_lidar_ && livox_lidar_->isConnected()) {
                    // Livox is working even though DDS isn't - continue with LiDAR only
                    std::cerr << "[SENSORS] DDS timeout, but Livox LiDAR is connected - continuing" << std::endl;
                    connected_.store(true);  // Mark as connected via Livox
                    break;
                }
                std::cerr << "[SENSORS] Timeout waiting for sensor data" << std::endl;
                std::cerr << "[SENSORS] Check that robot is powered on and network is configured" << std::endl;
                return false;
            }
        }

        std::cout << "[SENSORS] Connected and receiving data" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[SENSORS] Init failed: " << e.what() << std::endl;
        return false;
    }
#else
    (void)network_interface;
    std::cerr << "[SENSORS] unitree_sdk2 not available, SensorManager disabled" << std::endl;
    return false;
#endif
}

void SensorManager::setLidarCallback(std::function<void(const LidarScan&)> callback) {
    lidar_callback_ = callback;
}

void SensorManager::setImuCallback(std::function<void(const ImuData&)> callback) {
    imu_callback_ = callback;
}

LidarScan SensorManager::getLatestLidar() const {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    return latest_lidar_;
}

ImuData SensorManager::getLatestImu() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return latest_imu_;
}

float SensorManager::getBatteryPercent() const {
    return battery_percent_.load();
}

Pose2D SensorManager::getEstimatedPose() const {
    // Simple pose estimate: use IMU yaw for orientation
    // X, Y are zeros (no odometry integration in MVP)
    // Full SLAM/odometry would provide complete pose
    std::lock_guard<std::mutex> lock(imu_mutex_);
    Pose2D pose;
    pose.x = 0.0f;
    pose.y = 0.0f;
    pose.theta = latest_imu_.yaw;
    return pose;
}

bool SensorManager::isConnected() const {
    return connected_.load();
}

bool SensorManager::checkConnection() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - last_data_time_;

    if (elapsed > std::chrono::seconds(2)) {
        connected_.store(false);
        std::cerr << "[SENSORS] Connection lost - no data for 2 seconds" << std::endl;
        return false;
    }
    return true;
}

bool SensorManager::reconnect() {
#ifdef HAS_UNITREE_SDK2
    if (connected_.load()) return true;

    std::cout << "[SENSORS] Attempting reconnection..." << std::endl;

    try {
        impl_->state_sub_->InitChannel();
        impl_->lidar_sub_->InitChannel();

        // Wait briefly for data
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (connected_.load()) {
            std::cout << "[SENSORS] Reconnected successfully" << std::endl;
            return true;
        }
    } catch (const std::exception& e) {
        std::cerr << "[SENSORS] Reconnection failed: " << e.what() << std::endl;
    }
    return false;
#else
    return false;
#endif
}

void SensorManager::onLidarData(const LidarScan& scan) {
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        latest_lidar_ = scan;
    }
    last_data_time_ = std::chrono::steady_clock::now();
    connected_.store(true);

    if (lidar_callback_) {
        lidar_callback_(scan);
    }
}

void SensorManager::onImuData(const ImuData& data) {
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_ = data;
    }
    last_data_time_ = std::chrono::steady_clock::now();
    connected_.store(true);

    if (imu_callback_) {
        imu_callback_(data);
    }
}

void SensorManager::onWirelessRemote(const uint8_t* data) {
    std::lock_guard<std::mutex> lock(remote_mutex_);
    std::memcpy(wireless_remote_, data, 40);
    remote_available_.store(true);
}

bool SensorManager::getRawWirelessRemote(uint8_t out_buffer[40]) const {
    if (!remote_available_.load()) {
        return false;
    }
    std::lock_guard<std::mutex> lock(remote_mutex_);
    std::memcpy(out_buffer, wireless_remote_, 40);
    return true;
}
