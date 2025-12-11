#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <thread>
#include <vector>
#include <functional>
#include <opencv2/opencv.hpp>

// Forward declarations for librealsense (avoid header dependency)
namespace rs2 {
    class pipeline;
    class config;
    class frameset;
    class frame;
}

// Depth frame data
struct DepthFrame {
    cv::Mat color;           // BGR color image
    cv::Mat depth;           // 16-bit depth in mm
    cv::Mat depth_colorized; // Colorized depth for visualization
    double timestamp;        // Timestamp in seconds
    uint64_t frame_number;

    // Intrinsics for point cloud generation
    float fx, fy;            // Focal lengths
    float cx, cy;            // Principal point
    float depth_scale;       // Depth units (typically 0.001 for mm to meters)
};

// Callback type for depth frames
using DepthFrameCallback = std::function<void(const DepthFrame&)>;

// RealSense depth camera capture and recording
class DepthCapture {
public:
    DepthCapture();
    ~DepthCapture();

    // Initialize the RealSense camera
    // Returns false if no camera found
    bool init();

    // Start streaming (non-blocking)
    bool startCapture();

    // Stop streaming
    void stopCapture();

    // Start recording to session directory
    // Creates: <session_dir>/color/, <session_dir>/depth/, <session_dir>/depth_meta.json
    bool startRecording(const std::string& session_dir);

    // Stop recording
    void stopRecording();

    // Get latest frame (thread-safe copy)
    DepthFrame getLatestFrame();

    // Set callback for new frames
    void setFrameCallback(DepthFrameCallback callback) { frame_callback_ = callback; }

    // Check status
    bool isCapturing() const { return capturing_.load(); }
    bool isRecording() const { return recording_.load(); }

    // Get statistics
    uint64_t getFrameCount() const { return frame_count_.load(); }
    uint64_t getRecordedFrames() const { return recorded_frames_.load(); }

    // Configuration
    void setResolution(int width, int height);
    void setFramerate(int fps);
    void setAlignToColor(bool align) { align_to_color_ = align; }

private:
    // Capture thread function
    void captureLoop();

    // Save a single frame to disk
    void saveFrame(const DepthFrame& frame);

    // RealSense pipeline (pimpl to avoid header dependency)
    struct RealSenseImpl;
    std::unique_ptr<RealSenseImpl> rs_impl_;

    // Configuration
    int width_ = 640;
    int height_ = 480;
    int fps_ = 30;
    bool align_to_color_ = true;

    // State
    std::atomic<bool> initialized_{false};
    std::atomic<bool> capturing_{false};
    std::atomic<bool> recording_{false};
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> recorded_frames_{0};

    // Capture thread
    std::thread capture_thread_;

    // Latest frame (protected by mutex)
    mutable std::mutex frame_mutex_;
    DepthFrame latest_frame_;

    // Recording
    std::string session_dir_;
    std::mutex recording_mutex_;

    // Callback
    DepthFrameCallback frame_callback_;

    // Camera intrinsics (set after init)
    float fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    float depth_scale_ = 0.001f;
};
