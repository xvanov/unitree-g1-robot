#include "slam_sim/SlamSim.h"
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <cmath>

SlamSim::SlamSim(const std::string& map_path, float resolution)
    : nav_sim_(map_path, resolution)
    , mapper_(resolution, nav_sim_.getWidth(), nav_sim_.getHeight())
    , resolution_(resolution)
    , map_path_(map_path)
{
}

std::vector<Pose2D> SlamSim::generateSnakeTrajectory() {
    std::vector<Pose2D> traj;
    float margin = 0.5f;  // Stay 0.5m from edges
    float spacing = 0.5f; // Vertical spacing between passes
    float step = 0.1f;    // Step size along path

    // World dimensions from NavSim
    float world_width = nav_sim_.getWidth() * resolution_;
    float world_height = nav_sim_.getHeight() * resolution_;

    // BOUNDS CHECK: Ensure map is large enough for trajectory generation
    // Minimum required: 2*margin + spacing = 1.5m in each dimension
    if (world_width < 2.0f * margin + step || world_height < 2.0f * margin + spacing) {
        // Map too small - return single pose at center
        traj.push_back({world_width / 2.0f, world_height / 2.0f, 0.0f});
        return traj;
    }

    bool left_to_right = true;
    for (float y = margin; y < world_height - margin; y += spacing) {
        if (left_to_right) {
            for (float x = margin; x < world_width - margin; x += step) {
                traj.push_back({x, y, 0.0f});  // theta=0 facing right
            }
        } else {
            for (float x = world_width - margin; x > margin; x -= step) {
                traj.push_back({x, y, static_cast<float>(M_PI)});  // theta=PI facing left
            }
        }
        left_to_right = !left_to_right;
    }
    return traj;
}

void SlamSim::run() {
    trajectory_ = generateSnakeTrajectory();

    for (const auto& pose : trajectory_) {
        nav_sim_.setRobotPose(pose);
        LidarScan scan = nav_sim_.simulateLidar(360);  // 360 rays
        mapper_.update(pose, scan);
    }
}

float SlamSim::computeAccuracy() {
    // Load ground truth as grayscale
    cv::Mat truth = cv::imread(map_path_, cv::IMREAD_GRAYSCALE);
    if (truth.empty()) {
        return 0.0f;
    }

    // Convert to flat vector
    std::vector<uint8_t> truth_vec(truth.data, truth.data + truth.total());

    return mapper_.computeAccuracy(truth_vec);
}

void SlamSim::saveBuiltMap(const std::string& path) const {
    mapper_.saveMap(path);
}

void SlamSim::saveAccuracy(const std::string& path, float accuracy) const {
    nlohmann::json metrics;

    // Count occupied, free, and unexplored cells
    // Use same thresholds as GridMapper::computeAccuracy (>= 0.4, <= -0.4)
    int cells_occupied = 0;
    int cells_free = 0;
    const auto& log_odds = mapper_.getLogOddsMap();
    for (float l : log_odds) {
        if (l >= 0.4f) cells_occupied++;
        else if (l <= -0.4f) cells_free++;
    }
    int cells_explored = cells_occupied + cells_free;
    int cells_total = mapper_.getWidth() * mapper_.getHeight();
    int cells_unexplored = cells_total - cells_explored;

    // Round accuracy to 4 decimal places for cleaner output
    metrics["accuracy"] = std::round(static_cast<double>(accuracy) * 10000.0) / 10000.0;
    metrics["cells_total"] = cells_total;
    metrics["cells_explored"] = cells_explored;
    metrics["cells_unexplored"] = cells_unexplored;
    metrics["cells_occupied"] = cells_occupied;
    metrics["cells_free"] = cells_free;
    // Round resolution using double for cleaner JSON output
    metrics["resolution"] = std::round(static_cast<double>(resolution_) * 100.0) / 100.0;
    metrics["trajectory_poses"] = trajectory_.size();

    std::ofstream file(path);
    file << metrics.dump(2);
}
