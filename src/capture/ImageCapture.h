#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <future>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "util/Types.h"

class ISensorSource;
class PlanManager;

// Image metadata struct for each captured image
struct ImageMetadata {
    std::string image_path;              // Relative path to image file
    int64_t timestamp_ms = 0;            // Unix timestamp in milliseconds
    Pose2D robot_pose;                   // Robot position when captured
    Point2D plan_coords;                 // Position on plan in meters
    float camera_yaw = 0.0f;             // Camera orientation (radians)
    int sequence_number = 0;             // Image sequence in session
};

class ImageCapture {
public:
    ImageCapture(ISensorSource* sensors = nullptr, PlanManager* plan = nullptr);
    ~ImageCapture();

    // Session management
    bool startCapture(const std::string& session_id);
    void stopCapture();
    bool isCapturing() const { return capturing_; }

    // Frame capture (call each loop iteration - enforces interval internally)
    bool captureFrame(const Pose2D& robot_pose);

    // Configuration
    void setOutputDir(const std::string& path) { output_dir_ = path; }
    void setCaptureInterval(float seconds) { capture_interval_s_ = seconds; }
    void setCameraIndex(int index) { camera_index_ = index; }
    void setJpegQuality(int quality) { jpeg_quality_ = quality; }

    // Status
    int getImageCount() const { return image_count_.load(); }
    // Returns copy of captured images list. NOTE: This may be temporarily behind
    // getImageCount() because metadata is populated asynchronously after capture.
    // Call waitForPendingSaves() first if you need an accurate count.
    std::vector<ImageMetadata> getCapturedImages() const;
    std::string getCurrentSessionDir() const { return session_dir_; }

    // Wait for all pending async saves to complete (useful before reading getCapturedImages)
    void waitForPendingSaves();
    bool isCameraAvailable() const { return camera_available_; }

    // Session management (MEDIUM-3 fix)
    static std::vector<std::string> listSessions(const std::string& base_dir = "data/inspections");
    static bool deleteSession(const std::string& session_path);
    static size_t getTotalDiskUsage(const std::string& base_dir = "data/inspections");

    // HIGH-2 FIX: Load metadata from JSON file (for session resume/reload)
    static bool loadMetadata(const std::string& json_path, ImageMetadata& meta);

    // MEDIUM-2 FIX: Check available disk space (returns bytes available)
    static size_t getAvailableDiskSpace(const std::string& path);

    // Minimum disk space required to start capture (50MB default)
    static constexpr size_t MIN_DISK_SPACE_BYTES = 50 * 1024 * 1024;

    // LOW-3 FIX: Frame validation constants (previously magic numbers)
    static constexpr int MIN_FRAME_WIDTH = 320;    // Minimum valid frame width
    static constexpr int MIN_FRAME_HEIGHT = 240;   // Minimum valid frame height
    static constexpr double MIN_INTENSITY = 5.0;   // Below this = likely all black (corrupted)
    static constexpr double MAX_INTENSITY = 250.0; // Above this = likely all white (corrupted)

    // Testing support: inject a frame for capture testing without real camera
    // Returns true if frame was captured (interval elapsed), false otherwise
    bool captureTestFrame(const cv::Mat& frame, const Pose2D& robot_pose);

private:
    bool initCamera();
    void closeCamera();
    void saveImageAsync(const cv::Mat& frame, const Pose2D& pose);
    void saveMetadata(const ImageMetadata& meta);
    void cleanupPendingSaves();

    ISensorSource* sensors_;
    PlanManager* plan_manager_;
    cv::VideoCapture camera_;

    std::string output_dir_ = "data/inspections";
    std::string session_dir_;
    std::string session_id_;
    float capture_interval_s_ = 1.0f;
    int camera_index_ = 0;
    int jpeg_quality_ = 85;

    bool capturing_ = false;
    bool camera_available_ = false;
    std::atomic<int> image_count_{0};
    std::chrono::steady_clock::time_point last_capture_time_;
    std::vector<ImageMetadata> captured_images_;

    // Async save tracking
    std::mutex pending_mutex_;
    std::vector<std::future<void>> pending_saves_;

    // Thread-safe captured images list
    mutable std::mutex images_mutex_;

    // HIGH-4 FIX: Flag to prevent async saves from accessing destroyed object
    std::atomic<bool> shutting_down_{false};

    // MEDIUM-3 FIX: Rate limit frame validation warnings to reduce log noise
    int intensity_warning_count_ = 0;
    static constexpr int MAX_INTENSITY_WARNINGS = 5;  // Max warnings before suppressing
};
