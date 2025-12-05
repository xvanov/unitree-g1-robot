#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "util/Types.h"

struct PlanInfo {
    std::string path;
    std::string trade_type;
    int width_pixels = 0;
    int height_pixels = 0;
    float width_meters = 0.0f;
    float height_meters = 0.0f;
    int waypoint_count = 0;
    float scale = 0.02f;  // meters per pixel (default 2cm/pixel)
};

class PlanManager {
public:
    PlanManager() = default;

    // Load plan from file (PDF or PNG)
    bool loadPlan(const std::string& path, const std::string& trade_type = "finishes");

    // Accessors
    const cv::Mat& getPlanImage() const { return plan_image_; }
    const std::vector<Point2D>& getInspectionWaypoints() const { return waypoints_; }
    PlanInfo getPlanInfo() const;
    bool isLoaded() const { return !plan_image_.empty(); }

    // Coordinate transforms
    void setStartPosition(Point2D position, float orientation);
    Point2D robotToPlanCoords(const Pose2D& robot_pose) const;
    Point2D planToRobotCoords(Point2D plan_point) const;

    // For navigation integration
    std::vector<uint8_t> getOccupancyGrid() const;
    int getGridWidth() const { return plan_image_.cols; }
    int getGridHeight() const { return plan_image_.rows; }
    float getResolution() const { return plan_info_.scale; }

private:
    bool parsePdf(const std::string& path);
    bool parsePng(const std::string& path);
    void extractWalls();
    void generateWaypoints();
    bool isCellFree(int cx, int cy, int margin) const;

    cv::Mat plan_image_;           // Grayscale plan image
    cv::Mat wall_mask_;            // Binary mask (255=wall, 0=free)
    std::vector<Point2D> waypoints_;
    PlanInfo plan_info_;
    Pose2D origin_;                // Robot origin on plan
    bool origin_set_ = false;
};
