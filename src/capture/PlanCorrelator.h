#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "util/Types.h"

class PlanManager;

struct CameraParams {
    float hfov_deg = 60.0f;    // Horizontal field of view
    float vfov_deg = 45.0f;    // Vertical field of view
    float max_range_m = 5.0f;  // Maximum effective range
};

class PlanCorrelator {
public:
    explicit PlanCorrelator(PlanManager* plan = nullptr);

    // Must call after plan is loaded to initialize coverage map
    void initFromPlan();

    // Set plan manager (if not set in constructor)
    void setPlanManager(PlanManager* plan) { plan_manager_ = plan; }

    // Coordinate transforms (delegates to PlanManager)
    Point2D robotToPlanCoords(const Pose2D& robot_pose) const;

    // FOV and coverage
    void setCameraParams(const CameraParams& params) { camera_params_ = params; }
    std::vector<Point2D> calculateFOV(const Pose2D& pose) const;
    void updateCoverage(const Pose2D& pose);
    float getCoveragePercent() const;

    // Visualization
    void saveCoverageMap(const std::string& path) const;
    void resetCoverage();

    // Status
    bool isInitialized() const { return initialized_; }
    int getTotalFreePixels() const { return total_free_pixels_; }
    int getCoveredPixels() const;

private:
    PlanManager* plan_manager_;
    CameraParams camera_params_;
    cv::Mat coverage_map_;  // Binary mask: 255=covered, 0=not covered
    cv::Mat free_space_mask_;  // Mask of free pixels from plan (for accurate coverage)
    int total_free_pixels_ = 0;
    bool initialized_ = false;
};
