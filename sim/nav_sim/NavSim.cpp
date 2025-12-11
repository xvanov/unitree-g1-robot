#include "NavSim.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <stdexcept>

NavSim::NavSim(const std::string& map_png_path, float resolution)
    : resolution_(resolution), robot_radius_(0.3f) {

    map_ = cv::imread(map_png_path, cv::IMREAD_GRAYSCALE);
    if (map_.empty()) {
        throw std::runtime_error("Failed to load map: " + map_png_path);
    }

    robot_pose_ = {0, 0, 0};
}

void NavSim::setRobotPose(const Pose2D& pose) {
    robot_pose_ = pose;
    recordTrajectoryPoint();
}

Pose2D NavSim::getRobotPose() const {
    return robot_pose_;
}

void NavSim::applyVelocity(float vx, float vy, float omega, float dt) {
    // Simple kinematic integration
    // Transform velocity from robot frame to world frame
    float cos_theta = std::cos(robot_pose_.theta);
    float sin_theta = std::sin(robot_pose_.theta);

    robot_pose_.x += (vx * cos_theta - vy * sin_theta) * dt;
    robot_pose_.y += (vx * sin_theta + vy * cos_theta) * dt;
    robot_pose_.theta += omega * dt;

    // Normalize theta to [-PI, PI]
    while (robot_pose_.theta > M_PI) robot_pose_.theta -= 2.0f * M_PI;
    while (robot_pose_.theta < -M_PI) robot_pose_.theta += 2.0f * M_PI;

    recordTrajectoryPoint();
}

LidarScan NavSim::simulateLidar(int num_rays) {
    LidarScan scan;
    scan.angle_min = 0.0f;
    scan.angle_max = 2.0f * M_PI;
    scan.ranges.resize(num_rays);

    Point2D origin{robot_pose_.x, robot_pose_.y};

    for (int i = 0; i < num_rays; ++i) {
        float angle = scan.angle_min +
            (scan.angle_max - scan.angle_min) * i / num_rays + robot_pose_.theta;
        scan.ranges[i] = raycast(origin, angle);
    }

    return scan;
}

bool NavSim::checkCollision() const {
    // Check multiple points around robot footprint
    int check_radius = static_cast<int>(std::ceil(robot_radius_ / resolution_));

    int cx = static_cast<int>(std::floor(robot_pose_.x / resolution_));
    int cy = static_cast<int>(std::floor(robot_pose_.y / resolution_));

    for (int dy = -check_radius; dy <= check_radius; ++dy) {
        for (int dx = -check_radius; dx <= check_radius; ++dx) {
            float dist = std::sqrt(dx * dx + dy * dy) * resolution_;
            if (dist > robot_radius_) continue;

            int px = cx + dx;
            int py = cy + dy;

            if (px < 0 || px >= map_.cols || py < 0 || py >= map_.rows) {
                return true;  // Out of bounds = collision
            }

            if (map_.at<uint8_t>(py, px) < 128) {
                return true;  // Hit obstacle (black = obstacle)
            }
        }
    }

    return false;
}

bool NavSim::checkGoalReached(const Point2D& goal, float tolerance) const {
    float dx = goal.x - robot_pose_.x;
    float dy = goal.y - robot_pose_.y;
    return std::sqrt(dx * dx + dy * dy) < tolerance;
}

void NavSim::saveSnapshot(const std::string& path) const {
    // Convert to color image
    cv::Mat output;
    cv::cvtColor(map_, output, cv::COLOR_GRAY2BGR);

    // Draw trajectory in blue
    if (trajectory_.size() > 1) {
        std::vector<cv::Point> pts;
        for (const auto& p : trajectory_) {
            int px = static_cast<int>(p.x / resolution_);
            int py = static_cast<int>(p.y / resolution_);
            pts.push_back(cv::Point(px, py));
        }
        cv::polylines(output, pts, false, cv::Scalar(255, 0, 0), 2);
    }

    // Draw start (green circle)
    if (!trajectory_.empty()) {
        int sx = static_cast<int>(trajectory_.front().x / resolution_);
        int sy = static_cast<int>(trajectory_.front().y / resolution_);
        cv::circle(output, cv::Point(sx, sy), 5, cv::Scalar(0, 255, 0), -1);
    }

    // Draw current position (red circle)
    int rx = static_cast<int>(robot_pose_.x / resolution_);
    int ry = static_cast<int>(robot_pose_.y / resolution_);
    cv::circle(output, cv::Point(rx, ry), 5, cv::Scalar(0, 0, 255), -1);

    cv::imwrite(path, output);
}

void NavSim::saveMetrics(const std::string& path, bool goal_reached,
                         float path_length, int collisions, float time_s) const {
    nlohmann::json metrics;
    metrics["goal_reached"] = goal_reached;
    metrics["path_length"] = path_length;
    metrics["collisions"] = collisions;
    metrics["time_s"] = time_s;
    metrics["final_pose"] = {
        {"x", robot_pose_.x},
        {"y", robot_pose_.y},
        {"theta", robot_pose_.theta}
    };

    std::ofstream file(path);
    file << metrics.dump(2);
}

void NavSim::recordTrajectoryPoint() {
    trajectory_.push_back({robot_pose_.x, robot_pose_.y});
}

float NavSim::raycast(const Point2D& origin, float angle) const {
    float max_range = 10.0f;  // meters
    // Use resolution as step size for better performance (5cm vs 1cm = 5x faster)
    float step = resolution_;

    for (float r = 0; r < max_range; r += step) {
        float x = origin.x + r * std::cos(angle);
        float y = origin.y + r * std::sin(angle);

        int gx = static_cast<int>(std::floor(x / resolution_));
        int gy = static_cast<int>(std::floor(y / resolution_));

        if (gx < 0 || gx >= map_.cols || gy < 0 || gy >= map_.rows) {
            return r;  // Out of bounds
        }

        if (map_.at<uint8_t>(gy, gx) < 128) {
            return r;  // Hit obstacle (black = obstacle)
        }
    }

    return max_range;
}
