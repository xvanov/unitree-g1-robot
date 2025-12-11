#include "slam/GridMapper.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>

GridMapper::GridMapper(float resolution, int width, int height)
    : resolution_(resolution)
    , width_(width)
    , height_(height)
    , log_odds_(width * height, 0.0f)  // Initialize to 0.0f (unknown, p=0.5)
{
}

void GridMapper::update(const Pose2D& pose, const LidarScan& scan) {
    // Skip if no scan data
    if (scan.ranges.empty()) {
        return;
    }

    // Robot position in grid coordinates
    int robot_gx = worldToGridX(pose.x);
    int robot_gy = worldToGridY(pose.y);

    // Skip if robot is outside map
    if (!isInBounds(robot_gx, robot_gy)) {
        return;
    }

    // Calculate angle increment from scan
    float angle_increment = (scan.angle_max - scan.angle_min) /
                           static_cast<float>(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];

        // Skip invalid ranges (too close or max range)
        if (range < 0.1f || range >= 9.9f) continue;

        // CRITICAL: Convert from robot frame to world frame
        // scan angle is relative to robot, add pose.theta to get world angle
        float scan_angle = scan.angle_min + i * angle_increment;
        float world_angle = scan_angle + pose.theta;  // Robot heading added here

        // Calculate endpoint in world coordinates
        float end_x = pose.x + range * std::cos(world_angle);
        float end_y = pose.y + range * std::sin(world_angle);

        // Convert endpoint to grid coordinates
        int end_gx = worldToGridX(end_x);
        int end_gy = worldToGridY(end_y);

        // Trace ray from robot to endpoint using Bresenham
        traceRay(robot_gx, robot_gy, end_gx, end_gy);
    }
}

void GridMapper::traceRay(int x0, int y0, int x1, int y1) {
    // Integer-only Bresenham's line algorithm
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        // Update cell - all cells except endpoint are FREE
        if (x0 == x1 && y0 == y1) {
            // Endpoint: mark as OCCUPIED
            updateCell(x0, y0, l_occ_);
            break;
        } else {
            // Ray path: mark as FREE
            updateCell(x0, y0, l_free_);
        }

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void GridMapper::updateCell(int x, int y, float delta_log_odds) {
    if (!isInBounds(x, y)) return;

    int idx = y * width_ + x;
    float new_val = log_odds_[idx] + delta_log_odds;

    // Clamp to prevent saturation
    log_odds_[idx] = std::clamp(new_val, l_min_, l_max_);
}

bool GridMapper::isInBounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

void GridMapper::saveMap(const std::string& path) const {
    // Create grayscale image from log-odds
    cv::Mat img(height_, width_, CV_8UC1);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            img.at<uint8_t>(y, x) = logOddsToPng(log_odds_[idx]);
        }
    }

    cv::imwrite(path, img);
}

float GridMapper::computeAccuracy(const std::vector<uint8_t>& ground_truth) const {
    // ground_truth: flattened grayscale PNG from test_data/office.png
    // Convention: black (0) = obstacle, white (255) = free
    // Same as NavSim uses internally

    if (ground_truth.size() != static_cast<size_t>(width_ * height_)) {
        return 0.0f;  // Size mismatch
    }

    int matching = 0;
    int explored = 0;
    int total = width_ * height_;

    // Thresholds for classification (avoid comparing against 0.0f which is "unknown")
    // log_odds > 0.4 means p > 0.6, confidently occupied
    // log_odds < -0.4 means p < 0.4, confidently free
    const float occupied_threshold = 0.4f;
    const float free_threshold = -0.4f;

    for (int i = 0; i < total; i++) {
        // Ground truth: dark pixels (<128) = obstacle
        bool truth_occupied = ground_truth[i] < 128;

        // Built map: use threshold to classify confident cells only
        // Use >= and <= to include cells exactly at threshold (after single update)
        if (log_odds_[i] >= occupied_threshold) {
            // Confidently occupied
            explored++;
            if (truth_occupied) matching++;
        } else if (log_odds_[i] <= free_threshold) {
            // Confidently free
            explored++;
            if (!truth_occupied) matching++;
        }
        // Unknown cells (in (-0.4, 0.4)) are NOT counted - they don't affect accuracy
    }

    // Return accuracy only among explored cells (avoids inflated metric)
    // If no cells explored, return 0 to indicate mapping didn't work
    if (explored == 0) {
        return 0.0f;
    }

    return static_cast<float>(matching) / static_cast<float>(explored);
}
