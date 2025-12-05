#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include "util/Types.h"
#include "sensors/ISensorSource.h"

// Cost thresholds for costmap interpretation
constexpr uint8_t COST_FREE = 0;
constexpr uint8_t COST_UNKNOWN = 128;
constexpr uint8_t OBSTACLE_THRESHOLD = 250;  // Cells >= this are obstacles
constexpr uint8_t COST_OBSTACLE = 254;
constexpr uint8_t COST_LETHAL = 255;

struct GridCell {
    int x, y;

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const GridCell& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

namespace std {
    template<>
    struct hash<GridCell> {
        size_t operator()(const GridCell& c) const {
            return hash<int>()(c.x) ^ (hash<int>()(c.y) << 16);
        }
    };
}

class Costmap {
public:
    // Constructor with resolution (meters/cell) and dimensions (cells)
    Costmap(float resolution, int width, int height);

    // Factory method: Create costmap from image file
    // Use this instead of constructor + loadFromImage() for cleaner initialization
    static Costmap fromImage(const std::string& path, float resolution);

    // Update costmap from LiDAR scan data
    void updateFromScan(const LidarScan& scan, const Pose2D& robot_pose);

    // Get cell cost (0=free, 255=obstacle)
    uint8_t getCost(int x, int y) const;

    // Set cell cost
    void setCost(int x, int y, uint8_t cost);

    // Inflate obstacles for safe planning
    void inflate(float robot_radius);

    // Convert world coordinates to grid coordinates
    GridCell worldToGrid(float x, float y) const;

    // Convert grid coordinates to world coordinates
    Point2D gridToWorld(int gx, int gy) const;

    // Export map as PNG
    void savePng(const std::string& path) const;

    // Accessors
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getResolution() const { return resolution_; }

    // Initialize from a grayscale image (for simulation)
    void loadFromImage(const std::string& path);

private:
    float resolution_;  // meters per cell
    int width_;         // cells
    int height_;        // cells
    std::vector<uint8_t> data_;  // row-major grid data
};
