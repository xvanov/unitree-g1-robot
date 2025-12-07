#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

#include "sensors/ISensorSource.h"
#include "replay/SensorReplayer.h"
#include "replay/ReplayController.h"

namespace replay {

// Pose mode for replay
enum class PoseMode {
    GROUND_TRUTH,     // Return recorded pose as-is
    SIMULATED_DRIFT,  // Add synthetic drift to test localization
    ESTIMATED         // Return what localization system computes
};

/**
 * ReplaySensorSource - ISensorSource implementation for replay mode
 *
 * Feeds recorded sensor data through the standard sensor interface,
 * respecting timestamps for real-time playback.
 */
class ReplaySensorSource : public ISensorSource {
public:
    ReplaySensorSource();
    ~ReplaySensorSource() override;

    // Initialize with replayer and controller
    bool init(SensorReplayer* replayer, ReplayController* controller);

    // Start/stop playback thread
    bool start();
    void stop();
    bool isRunning() const { return running_.load(); }

    // ISensorSource interface
    LidarScan getLidarScan() override;
    Pose2D getPose() override;
    ImuData getImu() override;
    float getBatteryPercent() override;

    // Extended interface for replay
    cv::Mat getImage();
    bool hasNewImage();
    int getImageSequence() const;

    // Ground truth access
    Pose2D getGroundTruthPose() const;
    void setPoseMode(PoseMode mode) { pose_mode_ = mode; }
    PoseMode getPoseMode() const { return pose_mode_; }

    // For ESTIMATED mode - set by external localization
    void setEstimatedPose(const Pose2D& pose);

    // Drift simulation parameters
    void setDriftRate(float linear_m_per_s, float angular_rad_per_s);

    // Get recording directory for image loading
    std::string getRecordingDir() const;

private:
    // Replay thread function
    void replayThread();

    // Process a single message
    void processMessage(const ReplayMessage& msg);

    // Calculate wait time for timing-accurate playback
    std::chrono::microseconds calculateWaitTime(int64_t msg_timestamp_us);

    // Add simulated drift to pose
    Pose2D addDrift(const Pose2D& pose);

    // Load image from file
    cv::Mat loadImage(const std::string& filename);

    // Components
    SensorReplayer* replayer_ = nullptr;
    ReplayController* controller_ = nullptr;

    // Thread control
    std::atomic<bool> running_{false};
    std::thread replay_thread_;

    // Latest sensor data (thread-safe)
    mutable std::mutex data_mutex_;
    LidarScan latest_lidar_;
    ImuData latest_imu_;
    Pose2D ground_truth_pose_;
    Pose2D estimated_pose_;
    cv::Mat latest_image_;
    int latest_image_sequence_ = -1;
    std::atomic<bool> new_image_available_{false};

    // Pose mode
    PoseMode pose_mode_ = PoseMode::GROUND_TRUTH;

    // Drift simulation
    float drift_linear_ = 0.0f;
    float drift_angular_ = 0.0f;
    Pose2D accumulated_drift_{0, 0, 0};
    int64_t last_drift_update_us_ = 0;

    // Timing
    std::chrono::steady_clock::time_point playback_wall_start_;
    int64_t recording_start_us_ = 0;

    // Timing drift correction - recalibrate every N messages
    static constexpr int TIMING_RECALIBRATE_INTERVAL = 1000;  // Every 1000 messages
    int messages_since_recalibrate_ = 0;
    int64_t last_recalibrate_recording_us_ = 0;

    // Per-session warning counter (resets on init)
    int image_not_found_warnings_ = 0;
    static constexpr int MAX_IMAGE_WARNINGS = 5;
};

} // namespace replay
