#include "navigation/Costmap.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <stdexcept>

Costmap::Costmap(float resolution, int width, int height)
    : resolution_(resolution), width_(width), height_(height),
      data_(width * height, 0) {
}

Costmap Costmap::fromImage(const std::string& path, float resolution) {
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("Failed to load map: " + path);
    }

    Costmap costmap(resolution, image.cols, image.rows);

    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            uint8_t pixel = image.at<uint8_t>(y, x);
            // Black (0) = obstacle (255), White (255) = free (0)
            costmap.data_[y * image.cols + x] = 255 - pixel;
        }
    }

    return costmap;
}

void Costmap::updateFromScan(const LidarScan& scan, const Pose2D& robot_pose) {
    if (scan.ranges.empty()) return;

    float angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges.size();

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];
        if (range <= 0 || std::isinf(range) || std::isnan(range)) continue;

        float angle = scan.angle_min + i * angle_increment + robot_pose.theta;
        float world_x = robot_pose.x + range * std::cos(angle);
        float world_y = robot_pose.y + range * std::sin(angle);

        GridCell cell = worldToGrid(world_x, world_y);
        if (cell.x >= 0 && cell.x < width_ && cell.y >= 0 && cell.y < height_) {
            setCost(cell.x, cell.y, COST_OBSTACLE);  // Mark as obstacle
        }
    }
}

uint8_t Costmap::getCost(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return COST_LETHAL;  // Out of bounds treated as obstacle
    }
    return data_[y * width_ + x];
}

void Costmap::setCost(int x, int y, uint8_t cost) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        data_[y * width_ + x] = cost;
    }
}

void Costmap::inflate(float robot_radius) {
    int inflate_cells = static_cast<int>(std::ceil(robot_radius / resolution_));

    // Create a copy to avoid modifying while iterating
    std::vector<uint8_t> inflated = data_;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (data_[y * width_ + x] >= OBSTACLE_THRESHOLD) {  // Is obstacle
                // Inflate around this cell
                for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            float dist = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (dist <= robot_radius) {
                                // Use max to preserve higher costs
                                uint8_t& cell = inflated[ny * width_ + nx];
                                cell = std::max(cell, OBSTACLE_THRESHOLD);
                            }
                        }
                    }
                }
            }
        }
    }

    data_ = inflated;
}

GridCell Costmap::worldToGrid(float x, float y) const {
    return {
        static_cast<int>(std::floor(x / resolution_)),
        static_cast<int>(std::floor(y / resolution_))
    };
}

Point2D Costmap::gridToWorld(int gx, int gy) const {
    return {
        (gx + 0.5f) * resolution_,
        (gy + 0.5f) * resolution_
    };
}

void Costmap::savePng(const std::string& path) const {
    cv::Mat image(height_, width_, CV_8UC1);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            // Invert: 0 (free) -> 255 (white), 255 (obstacle) -> 0 (black)
            image.at<uint8_t>(y, x) = 255 - data_[y * width_ + x];
        }
    }

    cv::imwrite(path, image);
}

void Costmap::loadFromImage(const std::string& path) {
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("Failed to load map: " + path);
    }

    width_ = image.cols;
    height_ = image.rows;
    data_.resize(width_ * height_);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            uint8_t pixel = image.at<uint8_t>(y, x);
            // Black (0) = obstacle (255), White (255) = free (0)
            data_[y * width_ + x] = 255 - pixel;
        }
    }
}
