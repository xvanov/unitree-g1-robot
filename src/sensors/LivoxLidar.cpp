#include "sensors/LivoxLidar.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <filesystem>

#ifdef HAS_LIVOX_SDK2
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#endif

namespace {
    // LiDAR processing constants
    constexpr int NUM_RAYS = 360;           // Number of angular bins
    constexpr float MAX_RANGE = 40.0f;      // Mid-360 max range (meters)
    constexpr float MIN_RANGE = 0.1f;       // Minimum valid range (meters)
    constexpr float ANGLE_MIN = 0.0f;       // Start angle (radians)
    constexpr float ANGLE_MAX = 2.0f * M_PI; // End angle (radians)

    // Global instance pointer for C callbacks
    LivoxLidar* g_instance = nullptr;

#ifdef HAS_LIVOX_SDK2
    // C-style callback functions for Livox SDK (must be plain functions, not lambdas)
    static std::atomic<int> point_cloud_count{0};
    void PointCloudCb(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
        if (g_instance && data) {
            int count = ++point_cloud_count;
            if (count <= 5 || count % 1000 == 0) {
                std::cout << "[LIVOX] PointCloudCb: handle=" << handle << " dots=" << data->dot_num
                          << " type=" << (int)data->data_type << " count=" << count << std::endl;
            }
            g_instance->processPointCloud(handle, dev_type, data);
        }
    }

    void ImuDataCb(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
        if (g_instance && data) {
            g_instance->processImu(handle, data);
        }
    }

    void InfoChangeCb(uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
        if (!info) return;
        std::cout << "[LIVOX] LiDAR connected - Handle: " << handle
                  << ", SN: " << info->sn
                  << ", IP: " << info->lidar_ip << std::endl;

        if (g_instance) {
            g_instance->onLidarConnected(handle);
        }

        // Set work mode to normal (start the lidar)
        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal,
            [](livox_status status, uint32_t h, LivoxLidarAsyncControlResponse* resp, void*) {
                if (status == kLivoxLidarStatusSuccess) {
                    std::cout << "[LIVOX] Work mode set to NORMAL" << std::endl;
                } else {
                    std::cerr << "[LIVOX] Failed to set work mode: " << status << std::endl;
                }
            }, nullptr);
    }
#endif
}

LivoxLidar::LivoxLidar() {
    accumulated_ranges_.resize(NUM_RAYS, MAX_RANGE);
    range_counts_.resize(NUM_RAYS, 0);
}

LivoxLidar::~LivoxLidar() {
    shutdown();
}

bool LivoxLidar::init(const std::string& config_path) {
#ifdef HAS_LIVOX_SDK2
    if (initialized_.load()) {
        std::cerr << "[LIVOX] Already initialized" << std::endl;
        return true;
    }

    // Determine config path - MUST use absolute path as Livox SDK may change cwd
    std::string cfg_path = config_path;
    if (cfg_path.empty()) {
        // Try default locations
        std::vector<std::string> search_paths = {
            "config/mid360_config.json",
            "../config/mid360_config.json",
            "/home/k/unitree-g1-robot/config/mid360_config.json"
        };
        for (const auto& path : search_paths) {
            if (std::filesystem::exists(path)) {
                cfg_path = path;
                break;
            }
        }
    }

    if (cfg_path.empty() || !std::filesystem::exists(cfg_path)) {
        std::cerr << "[LIVOX] Config file not found. Create config/mid360_config.json" << std::endl;
        return false;
    }

    // Convert to absolute path - required because rapidjson in Livox SDK
    // can crash with relative paths due to internal file handling
    cfg_path = std::filesystem::absolute(cfg_path).string();
    std::cout << "[LIVOX] Using config: " << cfg_path << std::endl;

    // Verify config file is readable
    FILE* test_file = std::fopen(cfg_path.c_str(), "rb");
    if (!test_file) {
        std::cerr << "[LIVOX] Cannot open config file for reading: " << cfg_path << std::endl;
        return false;
    }
    fseek(test_file, 0, SEEK_END);
    long file_size = ftell(test_file);
    fclose(test_file);
    std::cout << "[LIVOX] Config file size: " << file_size << " bytes" << std::endl;

    // Store global instance for callbacks
    g_instance = this;

    std::cout << "[LIVOX] Calling LivoxLidarSdkInit..." << std::endl;
    std::cout.flush();  // Force flush before SDK call
    std::cerr.flush();

    // Initialize Livox SDK
    if (!LivoxLidarSdkInit(cfg_path.c_str())) {
        std::cerr << "[LIVOX] SDK init failed" << std::endl;
        g_instance = nullptr;
        return false;
    }

    std::cout << "[LIVOX] SDK init succeeded, registering callbacks..." << std::endl;

    // Register callbacks using plain C functions
    SetLivoxLidarPointCloudCallBack(PointCloudCb, nullptr);
    std::cout << "[LIVOX] Point cloud callback registered" << std::endl;

    SetLivoxLidarImuDataCallback(ImuDataCb, nullptr);
    std::cout << "[LIVOX] IMU callback registered" << std::endl;

    SetLivoxLidarInfoChangeCallback(InfoChangeCb, nullptr);
    std::cout << "[LIVOX] Info change callback registered" << std::endl;

    initialized_.store(true);
    std::cout << "[LIVOX] SDK initialized, waiting for LiDAR connection..." << std::endl;

    return true;
#else
    std::cerr << "[LIVOX] Livox SDK2 not available (compile with HAS_LIVOX_SDK2)" << std::endl;
    return false;
#endif
}

void LivoxLidar::shutdown() {
#ifdef HAS_LIVOX_SDK2
    if (initialized_.load()) {
        LivoxLidarSdkUninit();
        initialized_.store(false);
        connected_.store(false);
        g_instance = nullptr;
        std::cout << "[LIVOX] SDK shutdown complete" << std::endl;
    }
#endif
}

void LivoxLidar::setLidarCallback(std::function<void(const LidarScan&)> callback) {
    lidar_callback_ = callback;
}

void LivoxLidar::setImuCallback(std::function<void(const ImuData&)> callback) {
    imu_callback_ = callback;
}

LidarScan LivoxLidar::getLatestScan() const {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return latest_scan_;
}

#ifdef HAS_LIVOX_SDK2
void LivoxLidar::onLidarConnected(uint32_t handle) {
    lidar_handle_.store(handle);
    connected_.store(true);
}

void LivoxLidar::processPointCloud(uint32_t handle, const uint8_t dev_type, void* data_ptr) {
    auto* packet = static_cast<LivoxLidarEthernetPacket*>(data_ptr);
    if (!packet || packet->dot_num == 0) return;

    packet_count_++;
    point_count_ += packet->dot_num;

    // Mid-360 doesn't reliably use frame_cnt, so use packet count for frame boundaries
    // Emit a scan every ~100 packets (approximately 10Hz at 200k pts/sec, 96 pts/packet)
    constexpr int PACKETS_PER_FRAME = 100;
    bool new_frame = (packet_count_ % PACKETS_PER_FRAME == 0);

    // If new frame, emit the accumulated scan
    if (new_frame) {
        LidarScan scan;
        scan.angle_min = ANGLE_MIN;
        scan.angle_max = ANGLE_MAX;
        scan.ranges.resize(NUM_RAYS);

        {
            std::lock_guard<std::mutex> lock(accumulator_mutex_);
            for (int i = 0; i < NUM_RAYS; i++) {
                // Use minimum range per bin (accumulated_ranges_ stores min, not sum)
                scan.ranges[i] = (range_counts_[i] > 0)
                    ? accumulated_ranges_[i]  // Already the minimum
                    : MAX_RANGE;
            }

            // Reset accumulators
            std::fill(accumulated_ranges_.begin(), accumulated_ranges_.end(), 0.0f);
            std::fill(range_counts_.begin(), range_counts_.end(), 0);
        }

        // Update latest scan
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            latest_scan_ = scan;
        }

        // Invoke callback
        if (lidar_callback_) {
            static int scan_emit_count = 0;
            if (++scan_emit_count <= 5 || scan_emit_count % 100 == 0) {
                std::cout << "[LIVOX] Emitting scan #" << scan_emit_count << " (callback=" << (lidar_callback_ ? "set" : "null") << ")" << std::endl;
            }
            lidar_callback_(scan);
        } else {
            static int no_callback_count = 0;
            if (++no_callback_count <= 5) {
                std::cout << "[LIVOX] WARNING: new_frame but no callback set!" << std::endl;
            }
        }
    }

    // Accumulate points based on data type
    std::lock_guard<std::mutex> lock(accumulator_mutex_);

    if (packet->data_type == kLivoxLidarCartesianCoordinateHighData) {
        auto* points = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(packet->data);
        for (uint32_t i = 0; i < packet->dot_num; i++) {
            // Skip noisy points (tag filtering as per Unitree docs)
            if (points[i].tag > 0 && points[i].tag < 16) continue;

            // Convert mm to meters
            float x = points[i].x / 1000.0f;
            float y = points[i].y / 1000.0f;
            float z = points[i].z / 1000.0f;

            // Compute 2D range and angle (project to XY plane)
            float range = std::sqrt(x * x + y * y);
            if (range < MIN_RANGE || range > MAX_RANGE) continue;

            float angle = std::atan2(y, x);
            if (angle < 0) angle += ANGLE_MAX;

            int bin = static_cast<int>(angle / ANGLE_MAX * NUM_RAYS);
            bin = std::clamp(bin, 0, NUM_RAYS - 1);

            // Keep minimum range per bin
            if (range_counts_[bin] == 0 || range < accumulated_ranges_[bin]) {
                accumulated_ranges_[bin] = range;
            }
            range_counts_[bin]++;
        }
    }
    else if (packet->data_type == kLivoxLidarCartesianCoordinateLowData) {
        auto* points = reinterpret_cast<LivoxLidarCartesianLowRawPoint*>(packet->data);
        for (uint32_t i = 0; i < packet->dot_num; i++) {
            if (points[i].tag > 0 && points[i].tag < 16) continue;

            // Convert cm to meters
            float x = points[i].x / 100.0f;
            float y = points[i].y / 100.0f;

            float range = std::sqrt(x * x + y * y);
            if (range < MIN_RANGE || range > MAX_RANGE) continue;

            float angle = std::atan2(y, x);
            if (angle < 0) angle += ANGLE_MAX;

            int bin = static_cast<int>(angle / ANGLE_MAX * NUM_RAYS);
            bin = std::clamp(bin, 0, NUM_RAYS - 1);

            if (range_counts_[bin] == 0 || range < accumulated_ranges_[bin]) {
                accumulated_ranges_[bin] = range;
            }
            range_counts_[bin]++;
        }
    }
    else if (packet->data_type == kLivoxLidarSphericalCoordinateData) {
        auto* points = reinterpret_cast<LivoxLidarSpherPoint*>(packet->data);
        for (uint32_t i = 0; i < packet->dot_num; i++) {
            if (points[i].tag > 0 && points[i].tag < 16) continue;

            // depth is in mm, theta is horizontal angle in 0.01 degree units
            float range = points[i].depth / 1000.0f;
            if (range < MIN_RANGE || range > MAX_RANGE) continue;

            float theta_deg = points[i].theta / 100.0f;
            float angle = theta_deg * M_PI / 180.0f;
            if (angle < 0) angle += ANGLE_MAX;

            int bin = static_cast<int>(angle / ANGLE_MAX * NUM_RAYS);
            bin = std::clamp(bin, 0, NUM_RAYS - 1);

            if (range_counts_[bin] == 0 || range < accumulated_ranges_[bin]) {
                accumulated_ranges_[bin] = range;
            }
            range_counts_[bin]++;
        }
    }
}

void LivoxLidar::processImu(uint32_t handle, void* data_ptr) {
    auto* packet = static_cast<LivoxLidarEthernetPacket*>(data_ptr);
    if (!packet || packet->data_type != kLivoxLidarImuData) return;

    auto* imu_points = reinterpret_cast<LivoxLidarImuRawPoint*>(packet->data);
    if (!imu_points || packet->dot_num == 0) return;

    // Use last IMU sample in the packet
    const auto& raw = imu_points[packet->dot_num - 1];

    ImuData imu;
    // Livox IMU provides raw gyro/accel, not orientation
    // We pass through gyro and accel; orientation would need integration
    imu.gyro_x = raw.gyro_x;
    imu.gyro_y = raw.gyro_y;
    imu.gyro_z = raw.gyro_z;
    imu.accel_x = raw.acc_x;
    imu.accel_y = raw.acc_y;
    imu.accel_z = raw.acc_z;

    // Note: roll/pitch/yaw would need to be computed from accelerometer + gyro integration
    // For now leave as 0 - the robot's main IMU should be used for orientation
    imu.roll = 0;
    imu.pitch = 0;
    imu.yaw = 0;

    if (imu_callback_) {
        imu_callback_(imu);
    }
}
#else
// Stubs when SDK not available
void LivoxLidar::onLidarConnected(uint32_t) {}
void LivoxLidar::processPointCloud(uint32_t, const uint8_t, void*) {}
void LivoxLidar::processImu(uint32_t, void*) {}
#endif
