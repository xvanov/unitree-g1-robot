#include "slam/SlamVisualizer.h"
#include "slam/OccupancyGrid.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

SlamVisualizer::SlamVisualizer(int window_width, int window_height)
    : window_width_(window_width)
    , window_height_(window_height)
    , last_update_(std::chrono::steady_clock::now())
{
    // Pre-allocate canvas to avoid heap allocation every frame
    canvas_ = cv::Mat(window_height_, window_width_, CV_8UC3);

    // Create window
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

    // Set mouse callback
    cv::setMouseCallback(WINDOW_NAME, mouseCallback, this);

    std::cout << "[SLAM VIZ] Window created: " << window_width_ << "x" << window_height_ << std::endl;
    std::cout << "[SLAM VIZ] Controls: 'l' toggle LiDAR rays, 'r' reset view, scroll zoom, drag pan" << std::endl;
}

SlamVisualizer::~SlamVisualizer() {
    cv::destroyWindow(WINDOW_NAME);
}

void SlamVisualizer::update(const GridMapper& mapper, const Pose2D& robot_pose,
                            const LidarScan* scan) {
    // Throttle updates to 10Hz to avoid impacting teleop performance (AC6)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_);
    if (elapsed.count() < UPDATE_INTERVAL_MS) {
        return;  // Skip this update, not enough time has passed
    }
    last_update_ = now;

    // Store references for render()
    mapper_ = &mapper;
    robot_pose_ = robot_pose;

    if (scan) {
        latest_scan_ = *scan;
        has_scan_ = true;
        scan_count_++;
    }

    // Count mapped cells (cells with significant log-odds)
    // This is done in update() to avoid recounting every render.
    // Note: This is O(n) but acceptable since update() is throttled to 10Hz.
    // For larger maps (>400x400), consider tracking incrementally in GridMapper.
    cells_mapped_ = 0;
    const auto& log_odds = mapper.getLogOddsMap();
    for (float l : log_odds) {
        if (std::abs(l) > 0.4f) {
            cells_mapped_++;
        }
    }
}

void SlamVisualizer::render() {
    if (!mapper_) {
        return;
    }

    // Clear canvas to gray (unknown space color)
    canvas_.setTo(cv::Scalar(128, 128, 128));

    // Draw layers in order
    drawOccupancyGrid(canvas_);
    drawRobotPose(canvas_, robot_pose_);

    if (show_lidar_rays_ && has_scan_) {
        drawLidarRays(canvas_, robot_pose_, latest_scan_);
    }

    drawStats(canvas_);

    // Display
    cv::imshow(WINDOW_NAME, canvas_);
}

void SlamVisualizer::drawOccupancyGrid(cv::Mat& canvas) {
    if (!mapper_) return;

    const auto& log_odds = mapper_->getLogOddsMap();
    int map_width = mapper_->getWidth();
    int map_height = mapper_->getHeight();
    float resolution = mapper_->getResolution();

    // Calculate visible cell range from current zoom/pan (viewport clipping)
    float left_world = screenToWorld(0, window_height_ / 2).x;
    float right_world = screenToWorld(window_width_, window_height_ / 2).x;
    float top_world = screenToWorld(window_width_ / 2, 0).y;
    float bottom_world = screenToWorld(window_width_ / 2, window_height_).y;

    // Convert to grid coordinates with margin
    int min_gx = std::max(0, static_cast<int>((left_world - map_origin_x_) / resolution) - 1);
    int max_gx = std::min(map_width - 1, static_cast<int>((right_world - map_origin_x_) / resolution) + 1);
    int min_gy = std::max(0, static_cast<int>((bottom_world - map_origin_y_) / resolution) - 1);
    int max_gy = std::min(map_height - 1, static_cast<int>((top_world - map_origin_y_) / resolution) + 1);

    // Draw only visible cells
    for (int gy = min_gy; gy <= max_gy; ++gy) {
        for (int gx = min_gx; gx <= max_gx; ++gx) {
            int idx = gy * map_width + gx;
            float l = log_odds[idx];

            // Skip unknown cells (log-odds near 0)
            if (std::abs(l) < 0.1f) {
                continue;
            }

            // Convert log-odds to grayscale using existing function
            uint8_t pixel = logOddsToPng(l);

            // Calculate world position of cell center
            float world_x = gx * resolution + map_origin_x_ + resolution / 2;
            float world_y = gy * resolution + map_origin_y_ + resolution / 2;

            // Convert to screen coordinates
            cv::Point screen_pos = worldToScreen(world_x, world_y);

            // Calculate cell size in pixels
            int cell_size = std::max(1, static_cast<int>(resolution * zoom_));

            // Draw cell
            cv::rectangle(canvas,
                         cv::Point(screen_pos.x - cell_size / 2, screen_pos.y - cell_size / 2),
                         cv::Point(screen_pos.x + cell_size / 2, screen_pos.y + cell_size / 2),
                         cv::Scalar(pixel, pixel, pixel),
                         cv::FILLED);
        }
    }
}

void SlamVisualizer::drawRobotPose(cv::Mat& canvas, const Pose2D& pose) {
    // Robot marker: isoceles triangle pointing in heading direction
    // Triangle size scales with zoom but has min/max bounds
    float scale = std::max(MIN_MARKER_SCALE, std::min(MAX_MARKER_SCALE, zoom_ / DEFAULT_ZOOM));
    float length = ROBOT_LENGTH * scale;
    float width = ROBOT_WIDTH * scale;

    // Calculate triangle vertices in world coordinates
    float cos_t = std::cos(pose.theta);
    float sin_t = std::sin(pose.theta);

    // Tip of triangle (front)
    float tip_x = pose.x + length * cos_t;
    float tip_y = pose.y + length * sin_t;

    // Base corners (left and right, perpendicular to heading)
    float base_left_x = pose.x - width / 2 * sin_t;
    float base_left_y = pose.y + width / 2 * cos_t;
    float base_right_x = pose.x + width / 2 * sin_t;
    float base_right_y = pose.y - width / 2 * cos_t;

    // Convert to screen coordinates
    std::vector<cv::Point> triangle_pts;
    triangle_pts.push_back(worldToScreen(tip_x, tip_y));
    triangle_pts.push_back(worldToScreen(base_left_x, base_left_y));
    triangle_pts.push_back(worldToScreen(base_right_x, base_right_y));

    // Draw filled triangle (blue in BGR)
    cv::fillPoly(canvas, std::vector<std::vector<cv::Point>>{triangle_pts},
                cv::Scalar(255, 0, 0));

    // Draw white outline
    cv::polylines(canvas, std::vector<std::vector<cv::Point>>{triangle_pts},
                 true, cv::Scalar(255, 255, 255), 1);

    // Draw heading line extending from robot center (cyan)
    float heading_len = 0.3f * scale;  // 0.3m in world units
    float heading_x = pose.x + heading_len * cos_t;
    float heading_y = pose.y + heading_len * sin_t;
    cv::line(canvas, worldToScreen(pose.x, pose.y), worldToScreen(heading_x, heading_y),
            cv::Scalar(255, 255, 0), 2);  // Cyan
}

void SlamVisualizer::drawLidarRays(cv::Mat& canvas, const Pose2D& pose,
                                   const LidarScan& scan) {
    if (scan.ranges.empty()) return;

    float angle_increment = (scan.angle_max - scan.angle_min) /
                           static_cast<float>(scan.ranges.size() - 1);

    cv::Point robot_screen = worldToScreen(pose.x, pose.y);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];

        // Skip invalid ranges
        if (range < 0.1f || range >= 9.9f) {
            continue;
        }

        // Calculate angle in world frame
        float angle = scan.angle_min + i * angle_increment + pose.theta;

        // Calculate endpoint in world coordinates
        float end_x = pose.x + range * std::cos(angle);
        float end_y = pose.y + range * std::sin(angle);

        cv::Point end_screen = worldToScreen(end_x, end_y);

        // Draw ray (green)
        cv::line(canvas, robot_screen, end_screen,
                cv::Scalar(0, 255, 0), 1);
    }
}

void SlamVisualizer::drawStats(cv::Mat& canvas) {
    // Semi-transparent background for stats overlay using ROI (avoids full canvas clone)
    cv::Rect stats_roi(0, 0, std::min(400, canvas.cols), std::min(60, canvas.rows));
    cv::Mat roi = canvas(stats_roi);
    cv::Mat overlay(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
    cv::addWeighted(overlay, 0.6, roi, 0.4, 0, roi);

    // Draw stats text (reuse getStatsString() to avoid duplication)
    cv::putText(canvas, getStatsString(), cv::Point(10, 25),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    // Second line with controls hint
    std::string controls = "l: LiDAR rays | r: reset | scroll: zoom | drag: pan";
    cv::putText(canvas, controls, cv::Point(10, 45),
               cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
}

cv::Point SlamVisualizer::worldToScreen(float wx, float wy) const {
    // Y inverted because screen Y grows downward
    int sx = static_cast<int>((wx - pan_x_) * zoom_ + window_width_ / 2);
    int sy = static_cast<int>(window_height_ / 2 - (wy - pan_y_) * zoom_);
    return cv::Point(sx, sy);
}

cv::Point2f SlamVisualizer::screenToWorld(int sx, int sy) const {
    float wx = (sx - window_width_ / 2) / zoom_ + pan_x_;
    float wy = (window_height_ / 2 - sy) / zoom_ + pan_y_;
    return cv::Point2f(wx, wy);
}

void SlamVisualizer::mouseCallback(int event, int x, int y, int flags, void* userdata) {
    SlamVisualizer* viz = static_cast<SlamVisualizer*>(userdata);
    viz->handleMouse(event, x, y, flags);
}

void SlamVisualizer::handleMouse(int event, int x, int y, int flags) {
    switch (event) {
        case cv::EVENT_MOUSEWHEEL: {
            // Scroll zoom (flags contains wheel delta)
            int delta = cv::getMouseWheelDelta(flags);
            float zoom_factor = delta > 0 ? 1.1f : 0.9f;
            setZoom(zoom_ * zoom_factor);
            break;
        }

        case cv::EVENT_LBUTTONDOWN:
            is_dragging_ = true;
            drag_start_x_ = x;
            drag_start_y_ = y;
            drag_start_pan_x_ = pan_x_;
            drag_start_pan_y_ = pan_y_;
            break;

        case cv::EVENT_MOUSEMOVE:
            if (is_dragging_) {
                // Pan the view
                float dx = (x - drag_start_x_) / zoom_;
                float dy = (drag_start_y_ - y) / zoom_;  // Y inverted
                pan_x_ = drag_start_pan_x_ - dx;
                pan_y_ = drag_start_pan_y_ - dy;
            }
            break;

        case cv::EVENT_LBUTTONUP:
            is_dragging_ = false;
            break;
    }
}

int SlamVisualizer::processKey(int key) {
    if (key == -1) return -1;

    key = key & 0xFF;  // Mask for extended keycodes

    switch (key) {
        case 'l':
        case 'L':
            show_lidar_rays_ = !show_lidar_rays_;
            std::cout << "[SLAM VIZ] LiDAR rays: " << (show_lidar_rays_ ? "ON" : "OFF") << std::endl;
            break;

        case 'r':
        case 'R':
            centerOnRobot();
            std::cout << "[SLAM VIZ] View reset to robot" << std::endl;
            break;

        case 'q':
        case 'Q':
        case 27:  // ESC
            should_close_ = true;
            break;

        case '+':
        case '=':
            setZoom(zoom_ * 1.2f);
            break;

        case '-':
        case '_':
            setZoom(zoom_ / 1.2f);
            break;
    }

    return key;
}

void SlamVisualizer::setZoom(float zoom) {
    zoom_ = std::max(MIN_ZOOM, std::min(MAX_ZOOM, zoom));
}

void SlamVisualizer::panTo(float world_x, float world_y) {
    pan_x_ = world_x;
    pan_y_ = world_y;
}

void SlamVisualizer::centerOnRobot() {
    pan_x_ = robot_pose_.x;
    pan_y_ = robot_pose_.y;
    zoom_ = DEFAULT_ZOOM;  // Reset zoom to default
}

void SlamVisualizer::setMapOrigin(float origin_x, float origin_y) {
    map_origin_x_ = origin_x;
    map_origin_y_ = origin_y;
}

std::string SlamVisualizer::getStatsString() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Cells: " << cells_mapped_
       << " | Pose: (" << robot_pose_.x << ", " << robot_pose_.y
       << ", " << robot_pose_.theta << ")"
       << " | Scans: " << scan_count_;
    return ss.str();
}
