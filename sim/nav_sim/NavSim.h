#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "util/Types.h"
#include "sensors/ISensorSource.h"

class NavSim {
public:
    // Load map from PNG (black=obstacle, white=free)
    explicit NavSim(const std::string& map_png_path, float resolution = 0.05f);

    // Place robot in simulation
    void setRobotPose(const Pose2D& pose);

    // Get current robot pose
    Pose2D getRobotPose() const;

    // Integrate velocity to update pose
    void applyVelocity(float vx, float vy, float omega, float dt);

    // Raycast to generate simulated LiDAR scan
    LidarScan simulateLidar(int num_rays);

    // Test if robot overlaps obstacle
    bool checkCollision() const;

    // Test if robot at goal
    bool checkGoalReached(const Point2D& goal, float tolerance = 0.2f) const;

    // Save current state as PNG with trajectory
    void saveSnapshot(const std::string& path) const;

    // Save JSON with metrics
    void saveMetrics(const std::string& path, bool goal_reached,
                     float path_length, int collisions, float time_s) const;

    // Record trajectory point
    void recordTrajectoryPoint();

    // Get map dimensions
    int getWidth() const { return map_.cols; }
    int getHeight() const { return map_.rows; }
    float getResolution() const { return resolution_; }

private:
    // Single ray to obstacle distance
    float raycast(const Point2D& origin, float angle) const;

    cv::Mat map_;              // Grayscale map
    float resolution_;         // meters per pixel
    Pose2D robot_pose_;
    std::vector<Point2D> trajectory_;
    float robot_radius_;       // For collision checking
};
