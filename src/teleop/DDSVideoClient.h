#pragma once

#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <string>
#include <opencv2/opencv.hpp>

/**
 * DDS-based video client for Unitree robots.
 *
 * Uses the Unitree SDK's VideoClient to pull JPEG frames from the robot's
 * videohub service via DDS RPC. This eliminates the need for SSH and GStreamer.
 *
 * The client runs a background thread that continuously fetches frames,
 * making them available to the main application via getLatestFrame().
 */
class DDSVideoClient {
public:
    DDSVideoClient();
    ~DDSVideoClient();

    // Initialize the video client
    // network_interface: e.g., "eth0", "enx00e04c3e0270"
    // Returns true on success
    bool init(const std::string& network_interface);

    // Start fetching frames in background thread
    void start();

    // Stop fetching frames
    void stop();

    // Check if running
    bool isRunning() const { return running_.load(); }

    // Get the latest frame (thread-safe)
    // Returns empty Mat if no frame available yet
    cv::Mat getLatestFrame();

    // Get latest JPEG data (for recording)
    // Returns empty vector if no data available
    std::vector<uint8_t> getLatestJpegData();

    // Get current FPS
    float getFps() const { return current_fps_.load(); }

    // Get frame count
    int getFrameCount() const { return frame_count_.load(); }

    // Get error count
    int getErrorCount() const { return error_count_.load(); }

    // Set frame callback (optional - called when new frame arrives)
    using FrameCallback = std::function<void(const cv::Mat&, const std::vector<uint8_t>&)>;
    void setFrameCallback(FrameCallback callback) { frame_callback_ = callback; }

private:
    // Background thread function
    void fetchLoop();

    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::thread fetch_thread_;

    // Latest frame data (protected by mutex)
    mutable std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    std::vector<uint8_t> latest_jpeg_;

    // Statistics
    std::atomic<int> frame_count_{0};
    std::atomic<int> error_count_{0};
    std::atomic<float> current_fps_{0.0f};

    // Callback
    FrameCallback frame_callback_;

    // Network interface for DDS
    std::string network_interface_;
};
