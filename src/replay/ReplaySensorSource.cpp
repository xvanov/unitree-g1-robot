#include "replay/ReplaySensorSource.h"

#include <iostream>
#include <filesystem>

// Reference to global running flag from main.cpp
extern std::atomic<bool> g_running;

namespace replay {

ReplaySensorSource::ReplaySensorSource() = default;

ReplaySensorSource::~ReplaySensorSource() {
    stop();
}

bool ReplaySensorSource::init(SensorReplayer* replayer, ReplayController* controller) {
    if (!replayer || !controller) {
        std::cerr << "[REPLAY_SOURCE] Invalid replayer or controller" << std::endl;
        return false;
    }

    if (!replayer->isOpen()) {
        std::cerr << "[REPLAY_SOURCE] Replayer not open" << std::endl;
        return false;
    }

    replayer_ = replayer;
    controller_ = controller;

    // Get recording timestamps
    auto metadata = replayer_->getMetadata();
    recording_start_us_ = metadata.start_time_us;

    // Set total time in controller
    controller_->setTotalTime(metadata.duration_seconds);

    return true;
}

bool ReplaySensorSource::start() {
    if (running_.load()) {
        return true;  // Already running
    }

    if (!replayer_ || !controller_) {
        std::cerr << "[REPLAY_SOURCE] Not initialized" << std::endl;
        return false;
    }

    running_.store(true);
    controller_->play();

    // Record wall-clock start time
    playback_wall_start_ = std::chrono::steady_clock::now();
    controller_->setPlaybackStartTime(playback_wall_start_);

    // Start replay thread
    replay_thread_ = std::thread(&ReplaySensorSource::replayThread, this);

    std::cout << "[REPLAY_SOURCE] Playback started" << std::endl;
    return true;
}

void ReplaySensorSource::stop() {
    // Signal thread to stop (if still running)
    running_.store(false);

    if (controller_) {
        controller_->stop();
    }

    // Always try to join the thread if it's joinable
    if (replay_thread_.joinable()) {
        replay_thread_.join();
    }
}

void ReplaySensorSource::replayThread() {
    ReplayMessage msg;

    while (running_.load() && g_running.load()) {
        // Check for pause
        while (controller_->isPaused() && running_.load() && g_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!running_.load() || !g_running.load()) {
            break;
        }

        // Handle seeking
        if (controller_->isSeekPending()) {
            float target = controller_->getSeekTarget();
            if (!replayer_->seek(target)) {
                std::cerr << "[REPLAY_SOURCE] Seek failed" << std::endl;
            }
            // Reset timing after seek
            playback_wall_start_ = std::chrono::steady_clock::now();
            controller_->setPlaybackStartTime(playback_wall_start_);
            controller_->clearSeekPending();
            continue;
        }

        // Read next message
        if (!replayer_->readNext(msg)) {
            // End of recording
            controller_->setFinished();

            if (controller_->isLoopEnabled()) {
                // Loop mode - seek back to start
                continue;
            } else {
                break;
            }
        }

        // Calculate wait time for timing-accurate playback
        auto wait_time = calculateWaitTime(msg.timestamp_us);
        if (wait_time.count() > 0) {
            std::this_thread::sleep_for(wait_time);
        }

        // Process the message
        processMessage(msg);

        // Update controller progress
        controller_->setProgress(replayer_->getProgress());
        float elapsed_s = static_cast<float>(msg.timestamp_us - recording_start_us_) / 1000000.0f;
        controller_->setElapsedTime(elapsed_s);
    }

    running_.store(false);
}

std::chrono::microseconds ReplaySensorSource::calculateWaitTime(int64_t msg_timestamp_us) {
    // Periodic timing recalibration to prevent drift accumulation
    messages_since_recalibrate_++;
    if (messages_since_recalibrate_ >= TIMING_RECALIBRATE_INTERVAL) {
        // Recalibrate: reset wall start to now, update recording reference point
        playback_wall_start_ = std::chrono::steady_clock::now();
        last_recalibrate_recording_us_ = msg_timestamp_us;
        messages_since_recalibrate_ = 0;
    }

    // Calculate elapsed time from last recalibration point
    int64_t recording_elapsed_us = msg_timestamp_us -
        (last_recalibrate_recording_us_ > 0 ? last_recalibrate_recording_us_ : recording_start_us_);

    // Apply playback speed
    float speed = controller_->getSpeed();
    int64_t scaled_elapsed_us = static_cast<int64_t>(recording_elapsed_us / speed);

    // Calculate target wall-clock time
    auto target_time = playback_wall_start_ + std::chrono::microseconds(scaled_elapsed_us);

    // Calculate wait duration
    auto now = std::chrono::steady_clock::now();
    auto wait_duration = std::chrono::duration_cast<std::chrono::microseconds>(target_time - now);

    return wait_duration;
}

void ReplaySensorSource::processMessage(const ReplayMessage& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    switch (msg.type) {
        case recording::MessageType::LIDAR_SCAN: {
            DecodedLidarScan decoded;
            if (SensorReplayer::decodeLidarScan(msg, decoded)) {
                latest_lidar_ = decoded.scan;
                controller_->incrementLidarCount();
            }
            break;
        }

        case recording::MessageType::IMU_DATA: {
            DecodedImuData decoded;
            if (SensorReplayer::decodeImuData(msg, decoded)) {
                latest_imu_ = decoded.imu;
                controller_->incrementImuCount();
            }
            break;
        }

        case recording::MessageType::POSE: {
            DecodedPose decoded;
            if (SensorReplayer::decodePose(msg, decoded)) {
                ground_truth_pose_ = decoded.pose;

                // Update drift simulation
                if (last_drift_update_us_ > 0 && pose_mode_ == PoseMode::SIMULATED_DRIFT) {
                    float dt = (msg.timestamp_us - last_drift_update_us_) / 1000000.0f;
                    accumulated_drift_.x += drift_linear_ * dt * std::cos(ground_truth_pose_.theta);
                    accumulated_drift_.y += drift_linear_ * dt * std::sin(ground_truth_pose_.theta);
                    accumulated_drift_.theta += drift_angular_ * dt;
                }
                last_drift_update_us_ = msg.timestamp_us;

                controller_->incrementPoseCount();
            }
            break;
        }

        case recording::MessageType::IMAGE: {
            DecodedImage decoded;
            if (SensorReplayer::decodeImage(msg, decoded)) {
                // Load the actual image file
                std::string image_path = replayer_->getRecordingDir() + "/images/" + decoded.filename;
                cv::Mat img = loadImage(image_path);
                if (!img.empty()) {
                    latest_image_ = img;
                    latest_image_sequence_ = decoded.sequence_number;
                    new_image_available_.store(true);
                    controller_->incrementImageCount();
                }
            }
            break;
        }

        case recording::MessageType::VIDEO_FRAME: {
            DecodedVideoFrame decoded;
            if (SensorReplayer::decodeVideoFrame(msg, decoded)) {
                // Decode JPEG to cv::Mat
                if (!decoded.jpeg_data.empty()) {
                    cv::Mat img = cv::imdecode(decoded.jpeg_data, cv::IMREAD_COLOR);
                    if (!img.empty()) {
                        latest_image_ = img;
                        latest_image_sequence_ = decoded.frame_number;
                        new_image_available_.store(true);
                        controller_->incrementVideoFrameCount();
                    }
                }
            }
            break;
        }

        case recording::MessageType::MOTOR_STATE:
            // Motor state replay not implemented in MVP
            // Future: could expose motor positions for analysis
            break;

        case recording::MessageType::TELEOP_CMD:
            // Teleop commands are recorded but not replayed to locomotion
            // (replay is for sensor data, not command execution)
            break;

        case recording::MessageType::METADATA:
            // Metadata messages are handled at file level, not per-message
            break;

        default:
            // Unknown message type - skip silently
            break;
    }
}

cv::Mat ReplaySensorSource::loadImage(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        // Log warning only occasionally to avoid spam (per-session counter)
        if (image_not_found_warnings_ < MAX_IMAGE_WARNINGS) {
            image_not_found_warnings_++;
            std::cerr << "[REPLAY_SOURCE] Image not found: " << path << std::endl;
            if (image_not_found_warnings_ == MAX_IMAGE_WARNINGS) {
                std::cerr << "[REPLAY_SOURCE] (suppressing further image warnings)" << std::endl;
            }
        }
        return cv::Mat();
    }

    return cv::imread(path);
}

// ISensorSource interface implementation

LidarScan ReplaySensorSource::getLidarScan() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_lidar_;
}

Pose2D ReplaySensorSource::getPose() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    switch (pose_mode_) {
        case PoseMode::GROUND_TRUTH:
            return ground_truth_pose_;

        case PoseMode::SIMULATED_DRIFT:
            return addDrift(ground_truth_pose_);

        case PoseMode::ESTIMATED:
            return estimated_pose_;
    }

    return ground_truth_pose_;
}

ImuData ReplaySensorSource::getImu() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_imu_;
}

float ReplaySensorSource::getBatteryPercent() {
    // Battery is not recorded, return full
    return 100.0f;
}

// Extended interface

cv::Mat ReplaySensorSource::getImage() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    new_image_available_.store(false);
    return latest_image_.clone();
}

bool ReplaySensorSource::hasNewImage() {
    return new_image_available_.load();
}

int ReplaySensorSource::getImageSequence() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_image_sequence_;
}

Pose2D ReplaySensorSource::getGroundTruthPose() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return ground_truth_pose_;
}

void ReplaySensorSource::setEstimatedPose(const Pose2D& pose) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    estimated_pose_ = pose;
}

void ReplaySensorSource::setDriftRate(float linear_m_per_s, float angular_rad_per_s) {
    drift_linear_ = linear_m_per_s;
    drift_angular_ = angular_rad_per_s;
}

Pose2D ReplaySensorSource::addDrift(const Pose2D& pose) {
    Pose2D drifted;
    drifted.x = pose.x + accumulated_drift_.x;
    drifted.y = pose.y + accumulated_drift_.y;
    drifted.theta = pose.theta + accumulated_drift_.theta;
    return drifted;
}

std::string ReplaySensorSource::getRecordingDir() const {
    if (replayer_) {
        return replayer_->getRecordingDir();
    }
    return "";
}

} // namespace replay
