#include "plan/PlanManager.h"
#include <iostream>
#include <algorithm>
#include <cmath>

#ifdef HAS_POPPLER
#include <poppler/cpp/poppler-document.h>
#include <poppler/cpp/poppler-page.h>
#include <poppler/cpp/poppler-page-renderer.h>
#endif

bool PlanManager::loadPlan(const std::string& path, const std::string& trade_type) {
    plan_info_.path = path;
    plan_info_.trade_type = trade_type;

    // Determine file type from extension
    std::string ext = path.substr(path.find_last_of('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    bool success = false;
    if (ext == "pdf") {
        success = parsePdf(path);
        if (!success) {
            std::cerr << "[PLAN] Failed to load PDF: " << path << std::endl;
            std::cerr << "[PLAN] Hint: Convert to PNG with: pdftoppm -png " << path << " output" << std::endl;
            return false;
        }
    } else if (ext == "png" || ext == "jpg" || ext == "jpeg") {
        success = parsePng(path);
        if (!success) {
            std::cerr << "[PLAN] Failed to load image: " << path << std::endl;
            std::cerr << "[PLAN] Hint: Check file exists and is a valid image" << std::endl;
            return false;
        }
    } else {
        std::cerr << "[PLAN] Unsupported file format: " << ext << std::endl;
        std::cerr << "[PLAN] Supported formats: PDF, PNG, JPG" << std::endl;
        return false;
    }

    extractWalls();
    generateWaypoints();
    return true;
}

#ifdef HAS_POPPLER
bool PlanManager::parsePdf(const std::string& path) {
    // Load PDF document
    std::unique_ptr<poppler::document> doc(
        poppler::document::load_from_file(path)
    );

    if (!doc || doc->is_locked()) {
        std::cerr << "[PLAN] Failed to load PDF: " << path << std::endl;
        return false;
    }

    // Get first page
    std::unique_ptr<poppler::page> page(doc->create_page(0));
    if (!page) {
        std::cerr << "[PLAN] PDF has no pages" << std::endl;
        return false;
    }

    // Render to image (150 DPI is good for floor plans)
    poppler::page_renderer renderer;
    renderer.set_render_hint(poppler::page_renderer::antialiasing, true);

    poppler::image img = renderer.render_page(page.get(), 150, 150);

    if (!img.is_valid()) {
        std::cerr << "[PLAN] Failed to render PDF page" << std::endl;
        return false;
    }

    // Convert poppler image to OpenCV Mat
    int width = img.width();
    int height = img.height();

    // Poppler image is ARGB32, convert to grayscale
    cv::Mat rgba(height, width, CV_8UC4, const_cast<char*>(img.data()));
    cv::cvtColor(rgba, plan_image_, cv::COLOR_RGBA2GRAY);

    plan_info_.width_pixels = width;
    plan_info_.height_pixels = height;
    plan_info_.width_meters = width * plan_info_.scale;
    plan_info_.height_meters = height * plan_info_.scale;

    std::cout << "[PLAN] PDF loaded: " << width << "x" << height << " px ("
              << plan_info_.width_meters << " x " << plan_info_.height_meters << " m)"
              << std::endl;

    return true;
}
#else
bool PlanManager::parsePdf(const std::string& path) {
    std::cerr << "[PLAN] PDF support not available (poppler not linked)" << std::endl;
    std::cerr << "[PLAN] Convert PDF to PNG first: pdftoppm -png " << path << " output" << std::endl;
    return false;
}
#endif

bool PlanManager::parsePng(const std::string& path) {
    // Load grayscale image
    plan_image_ = cv::imread(path, cv::IMREAD_GRAYSCALE);

    if (plan_image_.empty()) {
        std::cerr << "[PLAN] Failed to load image: " << path << std::endl;
        return false;
    }

    plan_info_.width_pixels = plan_image_.cols;
    plan_info_.height_pixels = plan_image_.rows;
    plan_info_.width_meters = plan_image_.cols * plan_info_.scale;
    plan_info_.height_meters = plan_image_.rows * plan_info_.scale;

    std::cout << "[PLAN] Image loaded: " << plan_image_.cols << "x" << plan_image_.rows
              << " px (" << plan_info_.width_meters << " x " << plan_info_.height_meters << " m)"
              << std::endl;

    return true;
}

void PlanManager::extractWalls() {
    if (plan_image_.empty()) return;

    // Threshold to binary: black (< 128) = wall, white (>= 128) = free
    // wall_mask_: 255 = wall, 0 = free
    cv::threshold(plan_image_, wall_mask_, 128, 255, cv::THRESH_BINARY_INV);

    std::cout << "[PLAN] Wall extraction complete" << std::endl;
}

void PlanManager::generateWaypoints() {
    waypoints_.clear();

    if (plan_image_.empty()) return;

    // Grid-based coverage waypoints
    float waypoint_spacing = 2.0f;  // meters between waypoints
    float margin = 0.5f;            // stay away from walls (meters)

    int spacing_cells = static_cast<int>(waypoint_spacing / plan_info_.scale);
    int margin_cells = static_cast<int>(margin / plan_info_.scale);

    // Ensure minimum spacing of 1 cell
    if (spacing_cells < 1) spacing_cells = 1;
    if (margin_cells < 1) margin_cells = 1;

    for (int y = margin_cells; y < plan_image_.rows - margin_cells; y += spacing_cells) {
        for (int x = margin_cells; x < plan_image_.cols - margin_cells; x += spacing_cells) {
            // Check if this cell and surrounding area is free (white)
            if (isCellFree(x, y, margin_cells)) {
                Point2D wp;
                wp.x = x * plan_info_.scale;
                wp.y = y * plan_info_.scale;
                waypoints_.push_back(wp);
            }
        }
    }

    plan_info_.waypoint_count = static_cast<int>(waypoints_.size());
    std::cout << "[PLAN] Generated " << waypoints_.size() << " waypoints" << std::endl;
}

bool PlanManager::isCellFree(int cx, int cy, int margin) const {
    // Check margin x margin region around cell
    for (int dy = -margin; dy <= margin; dy++) {
        for (int dx = -margin; dx <= margin; dx++) {
            int x = cx + dx;
            int y = cy + dy;
            if (x < 0 || x >= plan_image_.cols || y < 0 || y >= plan_image_.rows) {
                return false;  // Out of bounds = blocked
            }
            if (plan_image_.at<uint8_t>(y, x) < 128) {
                return false;  // Dark pixel = wall
            }
        }
    }
    return true;
}

PlanInfo PlanManager::getPlanInfo() const {
    return plan_info_;
}

void PlanManager::setStartPosition(Point2D position, float orientation) {
    origin_.x = position.x;
    origin_.y = position.y;
    origin_.theta = orientation;
    origin_set_ = true;
    std::cout << "[PLAN] Robot origin set at (" << position.x << ", "
              << position.y << ") theta=" << orientation << std::endl;
}

Point2D PlanManager::robotToPlanCoords(const Pose2D& robot_pose) const {
    // Transform robot coordinates to plan coordinates
    // Robot pose is relative to origin set by calibrate command

    if (!origin_set_) {
        // No origin set, assume robot is at plan origin
        return {robot_pose.x, robot_pose.y};
    }

    // Rotate and translate
    float cos_t = std::cos(origin_.theta);
    float sin_t = std::sin(origin_.theta);

    Point2D plan_point;
    plan_point.x = origin_.x + robot_pose.x * cos_t - robot_pose.y * sin_t;
    plan_point.y = origin_.y + robot_pose.x * sin_t + robot_pose.y * cos_t;

    return plan_point;
}

Point2D PlanManager::planToRobotCoords(Point2D plan_point) const {
    // Inverse transform: plan coordinates to robot coordinates

    if (!origin_set_) {
        return plan_point;
    }

    // Translate then rotate inverse
    float dx = plan_point.x - origin_.x;
    float dy = plan_point.y - origin_.y;

    float cos_t = std::cos(-origin_.theta);
    float sin_t = std::sin(-origin_.theta);

    Point2D robot_point;
    robot_point.x = dx * cos_t - dy * sin_t;
    robot_point.y = dx * sin_t + dy * cos_t;

    return robot_point;
}

std::vector<uint8_t> PlanManager::getOccupancyGrid() const {
    std::vector<uint8_t> grid;

    if (wall_mask_.empty()) {
        return grid;
    }

    // Convert wall_mask_ to occupancy grid format
    // wall_mask_: 255 = wall, 0 = free
    // occupancy: 0 = free, 100 = occupied, 255 = unknown
    grid.resize(wall_mask_.rows * wall_mask_.cols);

    for (int y = 0; y < wall_mask_.rows; y++) {
        for (int x = 0; x < wall_mask_.cols; x++) {
            int idx = y * wall_mask_.cols + x;
            if (wall_mask_.at<uint8_t>(y, x) > 127) {
                grid[idx] = 100;  // occupied
            } else {
                grid[idx] = 0;    // free
            }
        }
    }

    return grid;
}
