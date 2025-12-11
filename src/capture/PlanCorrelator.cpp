#include "capture/PlanCorrelator.h"
#include "plan/PlanManager.h"

#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PlanCorrelator::PlanCorrelator(PlanManager* plan)
    : plan_manager_(plan)
{
}

void PlanCorrelator::initFromPlan() {
    if (!plan_manager_ || !plan_manager_->isLoaded()) {
        std::cerr << "[CORRELATOR] Cannot init: plan not loaded" << std::endl;
        return;
    }

    int w = plan_manager_->getGridWidth();
    int h = plan_manager_->getGridHeight();

    // Create empty coverage map (same size as plan)
    coverage_map_ = cv::Mat::zeros(h, w, CV_8UC1);

    // Create free space mask from plan (white = free, black = wall)
    free_space_mask_ = cv::Mat::zeros(h, w, CV_8UC1);
    const cv::Mat& plan = plan_manager_->getPlanImage();
    total_free_pixels_ = 0;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            // Handle both single-channel and multi-channel images
            int pixel_val;
            if (plan.channels() == 1) {
                pixel_val = plan.at<uint8_t>(y, x);
            } else {
                // For color images, use grayscale intensity
                cv::Vec3b color = plan.at<cv::Vec3b>(y, x);
                pixel_val = (color[0] + color[1] + color[2]) / 3;
            }

            if (pixel_val >= 128) {
                total_free_pixels_++;
                free_space_mask_.at<uint8_t>(y, x) = 255;  // Mark as free
            }
        }
    }

    initialized_ = true;
    std::cout << "[CORRELATOR] Initialized coverage map: " << w << "x" << h
              << ", " << total_free_pixels_ << " free pixels" << std::endl;
}

Point2D PlanCorrelator::robotToPlanCoords(const Pose2D& robot_pose) const {
    if (plan_manager_ && plan_manager_->isLoaded()) {
        return plan_manager_->robotToPlanCoords(robot_pose);
    }
    // Fallback: return robot coords directly (no plan or plan not loaded)
    return {robot_pose.x, robot_pose.y};
}

std::vector<Point2D> PlanCorrelator::calculateFOV(const Pose2D& pose) const {
    // Simple triangular FOV approximation
    float range = camera_params_.max_range_m;
    // Convert degrees to radians: deg * (PI/180), then half for half-angle
    float half_angle = camera_params_.hfov_deg * static_cast<float>(M_PI) / 180.0f / 2.0f;

    Point2D plan_pos = robotToPlanCoords(pose);

    // Camera forward direction (assume camera faces forward)
    float cam_angle = pose.theta;

    // FOV triangle: robot position + two far corners
    std::vector<Point2D> fov;
    fov.push_back(plan_pos);  // Apex at robot

    // Left corner
    float left_angle = cam_angle + half_angle;
    fov.push_back({
        plan_pos.x + range * std::cos(left_angle),
        plan_pos.y + range * std::sin(left_angle)
    });

    // Right corner
    float right_angle = cam_angle - half_angle;
    fov.push_back({
        plan_pos.x + range * std::cos(right_angle),
        plan_pos.y + range * std::sin(right_angle)
    });

    return fov;
}

void PlanCorrelator::updateCoverage(const Pose2D& pose) {
    if (!plan_manager_ || !initialized_ || coverage_map_.empty()) return;

    // Calculate FOV polygon on plan
    // MEDIUM-3 NOTE: This is a simplified FOV model that does NOT account for wall occlusions.
    // The triangular FOV is projected directly onto the plan without ray-casting.
    // This means coverage percentage may be slightly inflated (by ~10-20%) because
    // areas behind walls are incorrectly marked as "covered" if they fall within the FOV triangle.
    // For MVP this is acceptable; future improvement would use ray-marching or Bresenham lines
    // to detect wall intersections and exclude occluded areas from coverage calculation.
    std::vector<Point2D> fov = calculateFOV(pose);
    if (fov.size() < 3) return;

    // Convert FOV points from plan coordinates (meters) to pixel coordinates
    // Plan coordinate system: (0,0) is top-left corner of plan image in meters
    // Pixel coordinate system: (0,0) is top-left corner of plan image in pixels
    // Conversion: pixel = meters / scale (meters_per_pixel)
    std::vector<cv::Point> fov_pixels;
    float scale = plan_manager_->getResolution();

    for (const auto& pt : fov) {
        // HIGH-3 FIX: Convert meters to pixels correctly
        // pt.x, pt.y are in meters (plan coordinates from robotToPlanCoords)
        int px = static_cast<int>(pt.x / scale);
        int py = static_cast<int>(pt.y / scale);

        // Clamp to image bounds
        px = std::max(0, std::min(px, coverage_map_.cols - 1));
        py = std::max(0, std::min(py, coverage_map_.rows - 1));

        fov_pixels.emplace_back(px, py);
    }

    // Fill FOV polygon on coverage map
    if (fov_pixels.size() >= 3) {
        cv::fillConvexPoly(coverage_map_, fov_pixels, cv::Scalar(255));
    }
}

float PlanCorrelator::getCoveragePercent() const {
    if (coverage_map_.empty() || total_free_pixels_ == 0) return 0.0f;

    // Only count pixels that are both covered AND free space (not walls)
    cv::Mat covered_free;
    cv::bitwise_and(coverage_map_, free_space_mask_, covered_free);
    int covered = cv::countNonZero(covered_free);
    return 100.0f * static_cast<float>(covered) / static_cast<float>(total_free_pixels_);
}

int PlanCorrelator::getCoveredPixels() const {
    if (coverage_map_.empty() || free_space_mask_.empty()) return 0;
    // Only count pixels that are both covered AND free space
    cv::Mat covered_free;
    cv::bitwise_and(coverage_map_, free_space_mask_, covered_free);
    return cv::countNonZero(covered_free);
}

void PlanCorrelator::saveCoverageMap(const std::string& path) const {
    if (coverage_map_.empty()) {
        std::cerr << "[CORRELATOR] Cannot save coverage map: not initialized" << std::endl;
        return;
    }

    // Create visualization: overlay coverage on plan
    cv::Mat visualization;

    if (plan_manager_ && plan_manager_->isLoaded()) {
        const cv::Mat& plan = plan_manager_->getPlanImage();

        // Convert plan to color if grayscale
        cv::Mat plan_color;
        if (plan.channels() == 1) {
            cv::cvtColor(plan, plan_color, cv::COLOR_GRAY2BGR);
        } else {
            plan_color = plan.clone();
        }

        // Create green overlay for covered areas
        cv::Mat coverage_color;
        cv::cvtColor(coverage_map_, coverage_color, cv::COLOR_GRAY2BGR);

        // Green where covered
        for (int y = 0; y < coverage_map_.rows; ++y) {
            for (int x = 0; x < coverage_map_.cols; ++x) {
                if (coverage_map_.at<uint8_t>(y, x) > 0) {
                    coverage_color.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);  // Green
                }
            }
        }

        // Blend: 70% plan, 30% coverage
        cv::addWeighted(plan_color, 0.7, coverage_color, 0.3, 0, visualization);
    } else {
        // No plan loaded, just save the coverage mask as color
        cv::cvtColor(coverage_map_, visualization, cv::COLOR_GRAY2BGR);
    }

    // Add coverage percentage text
    float percent = getCoveragePercent();
    char text[64];
    std::snprintf(text, sizeof(text), "Coverage: %.1f%%", percent);
    cv::putText(visualization, text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

    if (!cv::imwrite(path, visualization)) {
        std::cerr << "[CORRELATOR] Failed to save coverage map: " << path << std::endl;
    } else {
        std::cout << "[CORRELATOR] Saved coverage map: " << path << std::endl;
    }
}

void PlanCorrelator::resetCoverage() {
    if (!coverage_map_.empty()) {
        coverage_map_.setTo(0);
    }
}
