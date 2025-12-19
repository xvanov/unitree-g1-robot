#pragma once

#include <string>
#include <memory>
#include <opencv2/opencv.hpp>

#include "replay/SensorReplayer.h"
#include "replay/ReplayController.h"
#include "slam/GridMapper.h"
#include "slam/SlamVisualizer.h"
#include "recording/RecordingTypes.h"
#include "sensors/ISensorSource.h"
#include "util/Types.h"

namespace replay {

// Window layout options
enum class WindowLayout {
    GRID_2X2,        // 2x2 grid layout
    HORIZONTAL_STRIP // Side-by-side horizontal
};

/**
 * StreamReplayViewer - Multi-window synchronized replay visualization
 *
 * Displays up to 5 synchronized windows during replay:
 * 1. RGB Video - Head camera feed (VIDEO_FRAME or DEPTH_FRAME color)
 * 2. Webcam - USB webcam feed (IMAGE messages from images/ folder)
 * 3. Depth Colorized - TURBO colormap depth visualization
 * 4. SLAM Visualizer - Occupancy grid building progressively
 * 5. Replay Status - Playback controls, time, frame counts
 *
 * Story D-1: Multi-Stream Synchronized Replay Viewer
 */
class StreamReplayViewer {
public:
    StreamReplayViewer();
    ~StreamReplayViewer();

    // Configuration
    bool init(const std::string& recording_path);
    void setWindowLayout(WindowLayout layout) { layout_ = layout; }
    void setInitialSpeed(float speed) { initial_speed_ = speed; }

    // Main execution
    int run();  // Main loop, returns exit code

private:
    // Existing components (reused)
    std::unique_ptr<SensorReplayer> replayer_;
    std::unique_ptr<ReplayController> controller_;
    std::unique_ptr<GridMapper> mapper_;
    std::unique_ptr<SlamVisualizer> slam_viz_;

    // Pre-allocated display buffers (avoid per-frame allocation)
    cv::Mat rgb_display_;
    cv::Mat webcam_display_;
    cv::Mat depth_display_;
    cv::Mat status_canvas_;

    // Latest sensor data
    cv::Mat latest_rgb_;
    cv::Mat latest_webcam_;     // USB webcam frame (from IMAGE messages)
    cv::Mat latest_depth_raw_;  // 16-bit depth in mm
    LidarScan latest_scan_;
    Pose2D latest_pose_;
    ImuData latest_imu_;

    // Display windows
    void createWindows();
    void renderRgbWindow();
    void renderWebcamWindow();
    void renderDepthWindow();
    void renderSlamWindow();
    void renderStatusOverlay();
    void updateAllDisplays();

    // Image loading helper (for webcam frames stored in images/ folder)
    cv::Mat loadImageFile(const std::string& filename);

    // Message processing
    void processMessage(const ReplayMessage& msg);
    bool processKey(int key);
    void seekTo(float target_time);

    // Helpers
    std::string formatTimestamp(float seconds) const;

    // Message counters
    uint32_t rgb_frame_count_ = 0;
    uint32_t webcam_frame_count_ = 0;
    uint32_t depth_frame_count_ = 0;
    uint32_t lidar_count_ = 0;
    uint32_t pose_count_ = 0;

    // Timing and synchronization
    int64_t start_timestamp_ = 0;
    int64_t latest_rgb_timestamp_ = 0;
    int64_t latest_webcam_timestamp_ = 0;
    int64_t latest_depth_timestamp_ = 0;
    int64_t latest_lidar_timestamp_ = 0;
    int64_t latest_pose_timestamp_ = 0;

    // Webcam image loading state
    bool has_webcam_data_ = false;

    // Configuration
    WindowLayout layout_ = WindowLayout::GRID_2X2;
    float initial_speed_ = 1.0f;
    bool use_depth_for_rgb_ = false;  // Use DEPTH_FRAME color_jpeg for RGB if no VIDEO_FRAME

    // Window names
    static constexpr const char* WINDOW_RGB = "RGB Video";
    static constexpr const char* WINDOW_WEBCAM = "Webcam";
    static constexpr const char* WINDOW_DEPTH = "Depth Colorized";
    static constexpr const char* WINDOW_STATUS = "Replay Status";

    // Display constants
    static constexpr int DISPLAY_WIDTH = 640;
    static constexpr int DISPLAY_HEIGHT = 480;
    static constexpr int STATUS_HEIGHT = 200;
    static constexpr double MAX_DEPTH_MM = 10000.0;  // 10 meters in mm

    // Status overlay constants
    static constexpr int BAR_WIDTH = 600;
    static constexpr int BAR_X = 20;
    static constexpr int BAR_Y = 160;
    static constexpr int BAR_HEIGHT = 20;
};

} // namespace replay
