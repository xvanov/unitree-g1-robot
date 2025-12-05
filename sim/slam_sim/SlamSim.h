#pragma once

#include <string>
#include <vector>
#include "util/Types.h"
#include "slam/GridMapper.h"
#include "nav_sim/NavSim.h"

class SlamSim {
public:
    // Load ground truth map via NavSim
    SlamSim(const std::string& map_path, float resolution = 0.05f);

    // Execute full mapping simulation
    void run();

    // Get the built occupancy grid
    const std::vector<float>& getBuiltMap() const { return mapper_.getLogOddsMap(); }

    // Compare built map vs ground truth
    float computeAccuracy();

    // Export built map as PNG (0=obstacle, 255=free)
    void saveBuiltMap(const std::string& path) const;

    // Save accuracy metrics as JSON
    void saveAccuracy(const std::string& path, float accuracy) const;

    // Get trajectory size for metrics
    size_t getTrajectorySize() const { return trajectory_.size(); }

private:
    // Generate snake pattern trajectory through map
    std::vector<Pose2D> generateSnakeTrajectory();

    NavSim nav_sim_;
    GridMapper mapper_;
    float resolution_;
    std::vector<Pose2D> trajectory_;
    std::string map_path_;
};
