#include "capture/ImageCapture.h"
#include "plan/PlanManager.h"
#include "sensors/ISensorSource.h"

#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <algorithm>

// JSON serialization for existing types
namespace nlohmann {
    template <>
    struct adl_serializer<Point2D> {
        static void to_json(json& j, const Point2D& p) {
            j = json{{"x", p.x}, {"y", p.y}};
        }
        static void from_json(const json& j, Point2D& p) {
            j.at("x").get_to(p.x);
            j.at("y").get_to(p.y);
        }
    };

    template <>
    struct adl_serializer<Pose2D> {
        static void to_json(json& j, const Pose2D& p) {
            j = json{{"x", p.x}, {"y", p.y}, {"theta", p.theta}};
        }
        static void from_json(const json& j, Pose2D& p) {
            j.at("x").get_to(p.x);
            j.at("y").get_to(p.y);
            j.at("theta").get_to(p.theta);
        }
    };

    // HIGH-2 FIX: Add ImageMetadata serialization for future metadata reload capability
    template <>
    struct adl_serializer<ImageMetadata> {
        static void to_json(json& j, const ImageMetadata& m) {
            j = json{
                {"image_path", m.image_path},
                {"timestamp_ms", m.timestamp_ms},
                {"robot_pose", m.robot_pose},
                {"plan_coords", m.plan_coords},
                {"camera_yaw", m.camera_yaw},
                {"sequence_number", m.sequence_number}
            };
        }
        static void from_json(const json& j, ImageMetadata& m) {
            j.at("image_path").get_to(m.image_path);
            j.at("timestamp_ms").get_to(m.timestamp_ms);
            j.at("robot_pose").get_to(m.robot_pose);
            j.at("plan_coords").get_to(m.plan_coords);
            j.at("camera_yaw").get_to(m.camera_yaw);
            j.at("sequence_number").get_to(m.sequence_number);
        }
    };
}

ImageCapture::ImageCapture(ISensorSource* sensors, PlanManager* plan)
    : sensors_(sensors)
    , plan_manager_(plan)
{
}

ImageCapture::~ImageCapture() {
    // HIGH-4 FIX: Signal shutdown before waiting for async saves
    shutting_down_.store(true);
    if (capturing_) {
        stopCapture();
    }
}

bool ImageCapture::startCapture(const std::string& session_id) {
    if (capturing_) {
        std::cerr << "[CAPTURE] Already capturing - stop first" << std::endl;
        return false;
    }

    // MEDIUM-2 FIX: Check disk space before starting capture
    size_t available = getAvailableDiskSpace(output_dir_);
    if (available > 0 && available < MIN_DISK_SPACE_BYTES) {
        std::cerr << "[CAPTURE] Insufficient disk space: " << (available / 1024 / 1024)
                  << "MB available, need at least " << (MIN_DISK_SPACE_BYTES / 1024 / 1024)
                  << "MB" << std::endl;
        return false;
    }

    session_id_ = session_id;
    session_dir_ = output_dir_ + "/" + session_id;

    // Create directory structure before saving any files
    try {
        std::filesystem::create_directories(session_dir_ + "/images");
        std::cout << "[CAPTURE] Created session directory: " << session_dir_ << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[CAPTURE] Failed to create directory: " << e.what() << std::endl;
        return false;
    }

    if (!initCamera()) {
        std::cout << "[CAPTURE] Warning: Camera not available, capture disabled" << std::endl;
        // Continue anyway - graceful degradation
    }

    capturing_ = true;
    image_count_ = 0;
    captured_images_.clear();
    intensity_warning_count_ = 0;  // Reset warning counter for new session
    last_capture_time_ = std::chrono::steady_clock::now();

    std::cout << "[CAPTURE] Session started: " << session_id_ << std::endl;
    return true;
}

bool ImageCapture::initCamera() {
    // Try to open camera
    camera_.open(camera_index_);

    if (!camera_.isOpened()) {
        std::cerr << "[CAPTURE] Warning: Camera " << camera_index_
                  << " not available. Capture disabled." << std::endl;
        camera_available_ = false;
        return false;
    }

    // Set resolution (1080p if supported, fallback to default)
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    // Verify actual resolution
    int width = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "[CAPTURE] Camera initialized: " << width << "x" << height << std::endl;
    camera_available_ = true;
    return true;
}

void ImageCapture::closeCamera() {
    if (camera_.isOpened()) {
        camera_.release();
    }
    camera_available_ = false;
}

bool ImageCapture::captureFrame(const Pose2D& robot_pose) {
    if (!capturing_) return false;

    // MEDIUM-1 FIX: Only clean up when pending saves exceed threshold (5)
    // At 1fps capture, we rarely have more than 1-2 pending saves
    // This avoids calling cleanup 20x/second when main loop runs at 20Hz
    bool should_cleanup = false;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        should_cleanup = (pending_saves_.size() > 5);
    }
    if (should_cleanup) {
        cleanupPendingSaves();
    }

    if (!camera_available_) return false;

    // Check capture interval
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - last_capture_time_).count();
    if (elapsed < capture_interval_s_) {
        return false;  // Not time yet
    }

    // Capture frame
    cv::Mat frame;
    camera_ >> frame;
    if (frame.empty()) {
        std::cerr << "[CAPTURE] Warning: Empty frame captured" << std::endl;
        return false;
    }

    // LOW-3 FIX: Use named constants for frame validation thresholds
    if (frame.cols < MIN_FRAME_WIDTH || frame.rows < MIN_FRAME_HEIGHT) {
        std::cerr << "[CAPTURE] Warning: Frame too small (" << frame.cols << "x" << frame.rows << ")" << std::endl;
        return false;
    }

    // Check for corrupted frames (all black or all white)
    // MEDIUM-3 FIX: Rate limit warnings to avoid log flooding in dark construction sites
    cv::Scalar mean_val = cv::mean(frame);
    double avg_intensity = (mean_val[0] + mean_val[1] + mean_val[2]) / 3.0;
    if (avg_intensity < MIN_INTENSITY || avg_intensity > MAX_INTENSITY) {
        intensity_warning_count_++;
        if (intensity_warning_count_ <= MAX_INTENSITY_WARNINGS) {
            std::cerr << "[CAPTURE] Warning: Possibly corrupted frame (avg intensity: "
                      << avg_intensity << ")" << std::endl;
            if (intensity_warning_count_ == MAX_INTENSITY_WARNINGS) {
                std::cerr << "[CAPTURE] Warning: Suppressing further intensity warnings" << std::endl;
            }
        }
        // Still save it - might be legitimate dark/bright scene
    }

    // Save asynchronously to avoid blocking main loop
    saveImageAsync(frame.clone(), robot_pose);
    last_capture_time_ = now;
    return true;
}

// HIGH-2 FIX: Test support method to inject frames for unit testing
bool ImageCapture::captureTestFrame(const cv::Mat& frame, const Pose2D& robot_pose) {
    if (!capturing_) return false;

    // Check capture interval (same as captureFrame)
    // First capture always succeeds (image_count_ == 0), then enforce interval
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - last_capture_time_).count();
    if (image_count_.load() > 0 && elapsed < capture_interval_s_) {
        return false;  // Not time yet (skip check for first capture)
    }

    // Validate frame
    if (frame.empty()) {
        std::cerr << "[CAPTURE] Warning: Empty test frame" << std::endl;
        return false;
    }

    if (frame.cols < MIN_FRAME_WIDTH || frame.rows < MIN_FRAME_HEIGHT) {
        std::cerr << "[CAPTURE] Warning: Test frame too small" << std::endl;
        return false;
    }

    // Save asynchronously
    saveImageAsync(frame.clone(), robot_pose);
    last_capture_time_ = now;
    return true;
}

void ImageCapture::saveImageAsync(const cv::Mat& frame, const Pose2D& pose) {
    // HIGH-1 FIX: Use atomic fetch_add to get unique sequence number atomically
    // This prevents race conditions when multiple frames are captured in quick succession
    int seq = ++image_count_;  // Atomic increment returns the NEW value

    // Launch async save
    // HIGH-4 FIX: Check shutting_down_ flag early in lambda to prevent use-after-free
    auto future = std::async(std::launch::async, [this, frame, pose, seq]() {
        // Early exit if object is being destroyed
        if (shutting_down_.load()) {
            return;
        }

        // Build file paths - use 8 digits to support long inspections (>2.7 hours)
        char filename[32];
        std::snprintf(filename, sizeof(filename), "img_%08d.jpg", seq);
        std::string img_path = session_dir_ + "/images/" + filename;

        // Save JPEG with quality setting
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        if (!cv::imwrite(img_path, frame, params)) {
            std::cerr << "[CAPTURE] Failed to save: " << img_path << std::endl;
            return;
        }

        // Build metadata
        ImageMetadata meta;
        meta.image_path = std::string("images/") + filename;
        meta.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        meta.robot_pose = pose;

        // Get plan coordinates if plan manager available
        // Note: ImageCapture uses PlanManager directly for coordinate transforms in metadata.
        // PlanCorrelator is used separately for coverage tracking (driven by main.cpp).
        // This duplication is intentional - metadata generation happens async in save thread,
        // while coverage updates happen synchronously in main loop.
        if (plan_manager_ && plan_manager_->isLoaded()) {
            meta.plan_coords = plan_manager_->robotToPlanCoords(pose);
        } else {
            meta.plan_coords = {pose.x, pose.y};  // Use robot coords directly
        }

        meta.camera_yaw = pose.theta;
        meta.sequence_number = seq;

        // Save metadata file
        saveMetadata(meta);

        // Store in captured images list (thread-safe)
        {
            std::lock_guard<std::mutex> lock(images_mutex_);
            captured_images_.push_back(meta);
        }

        std::cout << "[CAPTURE] Saved: " << filename << std::endl;
    });

    // Track pending saves
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_saves_.push_back(std::move(future));
}

std::vector<ImageMetadata> ImageCapture::getCapturedImages() const {
    std::lock_guard<std::mutex> lock(images_mutex_);
    return captured_images_;  // Return copy for thread safety
}

void ImageCapture::saveMetadata(const ImageMetadata& meta) {
    nlohmann::json j;
    j["image_path"] = meta.image_path;
    j["timestamp_ms"] = meta.timestamp_ms;
    j["robot_pose"] = meta.robot_pose;
    j["plan_coords"] = meta.plan_coords;
    j["camera_yaw"] = meta.camera_yaw;
    j["sequence_number"] = meta.sequence_number;

    // Build metadata file path (replace .jpg with .json)
    std::string meta_path = session_dir_ + "/" +
        meta.image_path.substr(0, meta.image_path.find_last_of('.')) + ".json";

    std::ofstream file(meta_path);
    if (file.is_open()) {
        file << j.dump(2);  // Pretty print with 2-space indent
        file.close();
    } else {
        std::cerr << "[CAPTURE] Failed to save metadata: " << meta_path << std::endl;
    }
}

void ImageCapture::cleanupPendingSaves() {
    std::lock_guard<std::mutex> lock(pending_mutex_);

    // Remove completed futures
    pending_saves_.erase(
        std::remove_if(pending_saves_.begin(), pending_saves_.end(),
            [](std::future<void>& f) {
                return f.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
            }),
        pending_saves_.end()
    );

    // MEDIUM-4 fix: Shrink vector capacity if significantly oversized
    // This prevents memory bloat during long inspection sessions
    if (pending_saves_.capacity() > 100 && pending_saves_.size() < pending_saves_.capacity() / 4) {
        pending_saves_.shrink_to_fit();
    }
}

void ImageCapture::waitForPendingSaves() {
    // HIGH-4 FIX: Wait for futures WITHOUT holding the mutex to avoid potential deadlock
    std::vector<std::future<void>> futures_to_wait;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        futures_to_wait = std::move(pending_saves_);
        pending_saves_.clear();
    }

    for (auto& future : futures_to_wait) {
        if (future.valid()) {
            future.wait();
        }
    }
}

void ImageCapture::stopCapture() {
    if (!capturing_) return;

    capturing_ = false;

    // Wait for all pending saves to complete (uses improved non-blocking pattern)
    waitForPendingSaves();

    closeCamera();
    std::cout << "[CAPTURE] Session stopped. " << image_count_.load() << " images captured." << std::endl;
}

// Session management (MEDIUM-3 fix)
std::vector<std::string> ImageCapture::listSessions(const std::string& base_dir) {
    std::vector<std::string> sessions;

    try {
        if (!std::filesystem::exists(base_dir)) {
            return sessions;
        }

        for (const auto& entry : std::filesystem::directory_iterator(base_dir)) {
            if (entry.is_directory()) {
                sessions.push_back(entry.path().filename().string());
            }
        }

        // Sort by name (which includes timestamp, so chronological)
        std::sort(sessions.begin(), sessions.end());
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[CAPTURE] Error listing sessions: " << e.what() << std::endl;
    }

    return sessions;
}

bool ImageCapture::deleteSession(const std::string& session_path) {
    try {
        if (!std::filesystem::exists(session_path)) {
            std::cerr << "[CAPTURE] Session not found: " << session_path << std::endl;
            return false;
        }

        std::filesystem::remove_all(session_path);
        std::cout << "[CAPTURE] Deleted session: " << session_path << std::endl;
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[CAPTURE] Error deleting session: " << e.what() << std::endl;
        return false;
    }
}

size_t ImageCapture::getTotalDiskUsage(const std::string& base_dir) {
    size_t total = 0;

    try {
        if (!std::filesystem::exists(base_dir)) {
            return 0;
        }

        for (const auto& entry : std::filesystem::recursive_directory_iterator(base_dir)) {
            if (entry.is_regular_file()) {
                total += entry.file_size();
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[CAPTURE] Error calculating disk usage: " << e.what() << std::endl;
    }

    return total;
}

// MEDIUM-2 FIX: Check available disk space
size_t ImageCapture::getAvailableDiskSpace(const std::string& path) {
    try {
        std::filesystem::space_info space = std::filesystem::space(path);
        return space.available;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[CAPTURE] Error checking disk space: " << e.what() << std::endl;
        return 0;
    }
}

// HIGH-2 FIX: Load metadata from JSON file
bool ImageCapture::loadMetadata(const std::string& json_path, ImageMetadata& meta) {
    try {
        std::ifstream file(json_path);
        if (!file.is_open()) {
            std::cerr << "[CAPTURE] Failed to open metadata file: " << json_path << std::endl;
            return false;
        }

        nlohmann::json j = nlohmann::json::parse(file);
        meta = j.get<ImageMetadata>();
        return true;
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[CAPTURE] JSON parse error in " << json_path << ": " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[CAPTURE] Error loading metadata: " << e.what() << std::endl;
        return false;
    }
}
