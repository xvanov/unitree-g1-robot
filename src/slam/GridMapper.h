#pragma once

#include <vector>
#include <string>
#include <cmath>
#include "util/Types.h"
#include "sensors/ISensorSource.h"
#include "slam/OccupancyGrid.h"

class GridMapper {
public:
    // Constructor: resolution in meters/cell, dimensions in cells
    // Origin defaults to (0,0) - grid cell (0,0) is at world (0,0)
    GridMapper(float resolution, int width, int height);

    // Main update function - integrates scan into map
    // COORDINATE FRAME: LidarScan ranges are in ROBOT frame (angles relative to robot heading)
    // This function converts to WORLD frame using pose.theta before raytracing
    void update(const Pose2D& pose, const LidarScan& scan);

    // Accessors
    const std::vector<float>& getLogOddsMap() const { return log_odds_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getResolution() const { return resolution_; }

    // Export as PNG (0=obstacle/black, 255=free/white - matches NavSim)
    void saveMap(const std::string& path) const;

    // Accuracy comparison against ground truth PNG
    // ground_truth: grayscale image where black (<128) = obstacle
    float computeAccuracy(const std::vector<uint8_t>& ground_truth) const;

private:
    void traceRay(int x0, int y0, int x1, int y1);
    void updateCell(int x, int y, float delta_log_odds);
    bool isInBounds(int x, int y) const;

    // World to grid conversion (origin at 0,0)
    int worldToGridX(float x) const { return static_cast<int>(x / resolution_); }
    int worldToGridY(float y) const { return static_cast<int>(y / resolution_); }

    float resolution_;            // meters per cell (0.05f default)
    int width_, height_;          // grid dimensions in cells
    std::vector<float> log_odds_; // log-odds values, initialized to 0.0f

    // Log-odds constants
    static constexpr float l_occ_  = 0.85f;   // Occupied (hit) - high confidence
    static constexpr float l_free_ = -0.4f;   // Free (ray passed) - lower confidence
    static constexpr float l_max_  = 5.0f;    // Clamping upper bound
    static constexpr float l_min_  = -5.0f;   // Clamping lower bound
};
