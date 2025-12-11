#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <functional>

#ifdef HAS_REALSENSE
#include "capture/DepthCapture.h"
#endif

/**
 * DDS-based depth streaming service for RealSense cameras.
 *
 * Runs on the robot and publishes depth frames over CycloneDDS
 * so they can be received and recorded on the dev machine.
 *
 * Uses a custom topic "rt/depth" with compressed depth data.
 */
class DepthStreamer {
public:
    DepthStreamer();
    ~DepthStreamer();

    // Initialize DDS and RealSense
    // network_interface: e.g., "eth0"
    bool init(const std::string& network_interface);

    // Start streaming depth over DDS
    bool start();

    // Stop streaming
    void stop();

    // Check if running
    bool isRunning() const { return running_.load(); }

    // Get statistics
    uint64_t getFrameCount() const { return frame_count_.load(); }
    float getFps() const { return current_fps_.load(); }

    // Configuration
    void setResolution(int width, int height);
    void setFramerate(int fps);
    void setCompression(bool enable) { compress_ = enable; }

private:
    void streamLoop();

    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::thread stream_thread_;

    // RealSense capture
#ifdef HAS_REALSENSE
    std::unique_ptr<DepthCapture> depth_capture_;
#endif

    // Configuration
    int width_ = 640;
    int height_ = 480;
    int fps_ = 30;
    bool compress_ = true;  // Use PNG compression for depth

    // Statistics
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<float> current_fps_{0.0f};

    // Network interface
    std::string network_interface_;
};
