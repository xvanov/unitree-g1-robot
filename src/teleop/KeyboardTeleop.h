#pragma once

#include <string>
#include <atomic>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include "util/Types.h"

class SensorManager;
class LocoController;
class SensorRecorder;
class DDSVideoClient;
class DepthStreamClient;

// Video source options
enum class VideoSource {
    DDS,        // DDS-based VideoClient (default, recommended)
    GSTREAMER,  // SSH + GStreamer UDP stream (legacy)
    LOCAL,      // Local camera
    NONE        // No video
};

class KeyboardTeleop {
public:
    // Constructor with required dependencies
    KeyboardTeleop(SensorManager* sensors, LocoController* loco,
                   SensorRecorder* recorder = nullptr);
    ~KeyboardTeleop();

    // Run blocking teleop loop (returns when user quits)
    void run();

    // Signal the teleop loop to exit
    void stop() { running_ = false; }

    // Get last captured frame (for external use)
    cv::Mat getLastFrame() const;

    // Configuration
    void setMaxVelocityScale(float scale) { velocity_scale_ = scale; }
    void setCameraIndex(int index) { camera_index_ = index; }
    void setGStreamerPipeline(const std::string& pipeline) { gstreamer_pipeline_ = pipeline; }
    void setRobotIP(const std::string& ip) { robot_ip_ = ip; }
    void setVideoSource(VideoSource source) { video_source_ = source; }
    void setNetworkInterface(const std::string& iface) { network_interface_ = iface; }
    void setDepthPort(int port) { depth_port_ = port; }  // 0 = disabled

private:
    // Process keyboard input, returns true to continue, false to quit
    bool processKey(int key);

    // Update camera frame (from any source)
    bool updateCamera();

    // Initialize video source based on video_source_ setting
    bool initVideoSource();

    // Initialize specific video sources
    bool initDDSVideo();
    bool initGStreamerVideo();
    bool initLocalCamera();
    bool initDepthStream();

    // Draw overlay on camera frame
    void drawOverlay(cv::Mat& frame);

    // Send velocity command to robot
    void sendVelocity();

    // Format duration as HH:MM:SS
    std::string formatDuration(float seconds) const;

    // Format file size (bytes to MB/GB)
    std::string formatSize(uint64_t bytes) const;

    // Dependencies
    SensorManager* sensors_;
    LocoController* loco_;
    SensorRecorder* recorder_;

    // Video sources
    cv::VideoCapture camera_;         // For GStreamer/local camera
    std::unique_ptr<DDSVideoClient> dds_video_;  // For DDS video
    std::unique_ptr<DepthStreamClient> depth_client_;  // For depth stream
    int camera_index_ = 0;
    bool camera_available_ = false;
    bool depth_available_ = false;
    cv::Mat last_frame_;
    cv::Mat last_depth_;              // Last depth frame (16-bit)
    cv::Mat last_depth_color_;        // Colorized depth for display
    std::vector<uint8_t> last_jpeg_;  // For recording
    std::vector<uint8_t> last_depth_png_;  // For depth recording
    float depth_fx_ = 0, depth_fy_ = 0, depth_cx_ = 0, depth_cy_ = 0;
    float depth_scale_ = 0.001f;
    std::string gstreamer_pipeline_;  // Custom GStreamer pipeline (empty = use camera_index_)
    std::string robot_ip_;            // Robot IP for UDP stream receiving
    std::string network_interface_;   // Network interface for DDS
    VideoSource video_source_ = VideoSource::DDS;  // Default to DDS
    int depth_port_ = 0;              // Depth stream port (0 = disabled)

    // Control state
    std::atomic<bool> running_{false};
    float velocity_scale_ = 1.0f;

    // Current velocity command
    float cmd_vx_ = 0.0f;
    float cmd_vy_ = 0.0f;
    float cmd_omega_ = 0.0f;

    // Velocity increment per keypress
    static constexpr float VEL_INCREMENT = 0.1f;
    static constexpr float OMEGA_INCREMENT = 0.1f;

    // Recording state
    bool is_recording_ = false;
    std::chrono::steady_clock::time_point recording_start_;

    // Window name
    static constexpr const char* WINDOW_NAME = "Teleop - Press Q to quit";
};
