#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include "util/Types.h"
#include "slam/GridMapper.h"

class SlamVisualizer {
public:
    // Constructor with configurable window size
    SlamVisualizer(int window_width = 800, int window_height = 600);
    ~SlamVisualizer();

    // Update with latest map and pose
    // scan is optional - only needed for LiDAR ray visualization
    void update(const GridMapper& mapper, const Pose2D& robot_pose,
                const LidarScan* scan = nullptr);

    // Render and display (call from main loop)
    void render();

    // Controls
    void setShowLidarRays(bool show) { show_lidar_rays_ = show; }
    void setZoom(float zoom);
    void panTo(float world_x, float world_y);
    void centerOnRobot();
    void setMapOrigin(float origin_x, float origin_y);

    // Check if window was closed
    bool shouldClose() const { return should_close_; }

    // Handle keyboard input (returns processed key, -1 if none)
    int processKey(int key);

    // Get current stats as formatted string
    std::string getStatsString() const;

private:
    // Rendering helpers
    void drawOccupancyGrid(cv::Mat& canvas);
    void drawRobotPose(cv::Mat& canvas, const Pose2D& pose);
    void drawLidarRays(cv::Mat& canvas, const Pose2D& pose, const LidarScan& scan);
    void drawStats(cv::Mat& canvas);

    // Coordinate transforms
    cv::Point worldToScreen(float wx, float wy) const;
    cv::Point2f screenToWorld(int sx, int sy) const;

    // Mouse callback (static for OpenCV)
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);
    void handleMouse(int event, int x, int y, int flags);

    // Window properties
    int window_width_;
    int window_height_;
    static constexpr const char* WINDOW_NAME = "SLAM Visualizer";

    // Pre-allocated canvas (avoid heap allocation every frame)
    cv::Mat canvas_;

    // Map data (cached from last update)
    const GridMapper* mapper_ = nullptr;
    Pose2D robot_pose_{0, 0, 0};
    LidarScan latest_scan_;
    bool has_scan_ = false;

    // Map origin offset (for non-zero world origins)
    float map_origin_x_ = 0.0f;
    float map_origin_y_ = 0.0f;

    // View state
    float zoom_ = 20.0f;  // pixels per meter
    float pan_x_ = 0.0f;  // world coordinates
    float pan_y_ = 0.0f;
    bool show_lidar_rays_ = false;
    bool should_close_ = false;

    // Mouse interaction state
    bool is_dragging_ = false;
    int drag_start_x_ = 0;
    int drag_start_y_ = 0;
    float drag_start_pan_x_ = 0.0f;
    float drag_start_pan_y_ = 0.0f;

    // Statistics
    int cells_mapped_ = 0;
    int scan_count_ = 0;

    // Zoom limits
    static constexpr float MIN_ZOOM = 5.0f;   // pixels per meter
    static constexpr float MAX_ZOOM = 100.0f;
    static constexpr float DEFAULT_ZOOM = 20.0f;  // pixels per meter

    // Robot marker size (world units)
    static constexpr float ROBOT_LENGTH = 0.4f;  // meters
    static constexpr float ROBOT_WIDTH = 0.3f;   // meters

    // Robot marker scaling bounds (relative to default zoom)
    static constexpr float MIN_MARKER_SCALE = 0.5f;
    static constexpr float MAX_MARKER_SCALE = 2.0f;

    // Update throttling
    std::chrono::steady_clock::time_point last_update_;
    static constexpr int UPDATE_INTERVAL_MS = 100;  // 10Hz
};
