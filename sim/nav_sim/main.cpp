#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <chrono>

#include "NavSim.h"
#include "SimLocomotion.h"
#include "SimSensorSource.h"
#include "navigation/Costmap.h"
#include "navigation/Planner.h"
#include "navigation/PathFollower.h"

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " --map <map.png> --start x,y,theta --goal x,y --output <dir>\n";
    std::cout << "\nOptions:\n";
    std::cout << "  --map     Path to map PNG (black=obstacle, white=free)\n";
    std::cout << "  --start   Start pose: x,y,theta (meters, radians)\n";
    std::cout << "  --goal    Goal position: x,y (meters)\n";
    std::cout << "  --output  Output directory for results\n";
}

bool parsePoint(const std::string& str, float& x, float& y) {
    std::stringstream ss(str);
    char comma;
    if (!(ss >> x >> comma >> y) || comma != ',') {
        return false;
    }
    return true;
}

bool parsePose(const std::string& str, float& x, float& y, float& theta) {
    std::stringstream ss(str);
    char c1, c2;
    if (!(ss >> x >> c1 >> y >> c2 >> theta) || c1 != ',' || c2 != ',') {
        return false;
    }
    return true;
}

// Goal tolerance - must match PathFollower::GOAL_TOLERANCE for consistent behavior
constexpr float GOAL_TOLERANCE = 0.2f;

int main(int argc, char* argv[]) {
    std::string map_path;
    std::string output_dir;
    Pose2D start_pose{1.0f, 1.0f, 0.0f};
    Point2D goal{8.0f, 5.0f};

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--map" && i + 1 < argc) {
            map_path = argv[++i];
        } else if (arg == "--start" && i + 1 < argc) {
            if (!parsePose(argv[++i], start_pose.x, start_pose.y, start_pose.theta)) {
                std::cerr << "Error: Invalid start format. Expected: x,y,theta\n";
                return 1;
            }
        } else if (arg == "--goal" && i + 1 < argc) {
            if (!parsePoint(argv[++i], goal.x, goal.y)) {
                std::cerr << "Error: Invalid goal format. Expected: x,y\n";
                return 1;
            }
        } else if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (map_path.empty()) {
        std::cerr << "Error: --map is required\n";
        printUsage(argv[0]);
        return 1;
    }

    if (output_dir.empty()) {
        output_dir = "./outputs";
    }

    // Create output directory if it doesn't exist
    std::filesystem::create_directories(output_dir);

    // Open log file
    std::ofstream log_file(output_dir + "/nav.log");
    auto log = [&log_file](const std::string& msg) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::string time_str = std::ctime(&time);
        // Remove trailing newline from ctime()
        if (!time_str.empty() && time_str.back() == '\n') {
            time_str.pop_back();
        }
        log_file << "[INFO] " << time_str << " " << msg << "\n";
        std::cout << "[INFO] " << msg << "\n";
    };

    log("Starting NavSim");
    log("Map: " + map_path);
    log("Start: (" + std::to_string(start_pose.x) + ", " +
        std::to_string(start_pose.y) + ", " + std::to_string(start_pose.theta) + ")");
    log("Goal: (" + std::to_string(goal.x) + ", " + std::to_string(goal.y) + ")");

    try {
        // Initialize simulation
        const float resolution = 0.05f;  // 5cm per pixel
        NavSim sim(map_path, resolution);

        // Validate start and goal positions are within map bounds
        float map_width = sim.getWidth() * resolution;
        float map_height = sim.getHeight() * resolution;

        if (start_pose.x < 0 || start_pose.x >= map_width ||
            start_pose.y < 0 || start_pose.y >= map_height) {
            std::cerr << "Error: Start position (" << start_pose.x << ", " << start_pose.y
                      << ") is outside map bounds (0-" << map_width << ", 0-" << map_height << ")\n";
            return 1;
        }

        if (goal.x < 0 || goal.x >= map_width ||
            goal.y < 0 || goal.y >= map_height) {
            std::cerr << "Error: Goal position (" << goal.x << ", " << goal.y
                      << ") is outside map bounds (0-" << map_width << ", 0-" << map_height << ")\n";
            return 1;
        }

        sim.setRobotPose(start_pose);

        SimLocomotion locomotion(sim);
        SimSensorSource sensors(sim);

        // Create costmap from map using factory method
        Costmap costmap = Costmap::fromImage(map_path, resolution);
        costmap.inflate(0.4f);  // Robot radius (0.3m) + safety margin (0.1m) for path tracking

        // Plan path
        Planner planner(costmap);
        auto path = planner.planPath({start_pose.x, start_pose.y}, goal);

        if (path.empty()) {
            log("ERROR: No path found!");
            sim.saveMetrics(output_dir + "/metrics.json", false, 0, 0, 0);
            sim.saveSnapshot(output_dir + "/trajectory.png");
            return 1;
        }

        log("Path found with " + std::to_string(path.size()) + " waypoints");

        // Setup path follower
        PathFollower follower;
        follower.setPath(path);

        // Simulation loop
        const float dt = 0.1f;   // 100ms timestep
        const float max_time = 120.0f;  // 2 minutes max
        float elapsed_time = 0.0f;
        float path_length = 0.0f;
        int collisions = 0;
        Pose2D prev_pose = sim.getRobotPose();

        while (!follower.isComplete() && elapsed_time < max_time) {
            // Get velocity command
            Pose2D current_pose = sensors.getPose();
            Velocity cmd = follower.getVelocityCommand(current_pose);

            // Apply velocity
            locomotion.setVelocity(cmd.vx, cmd.vy, cmd.omega);
            locomotion.step();

            // Check for collision
            if (sim.checkCollision()) {
                collisions++;
                log("Collision detected at (" + std::to_string(current_pose.x) +
                    ", " + std::to_string(current_pose.y) + ")");
                break;  // Stop on collision
            }

            // Update path length
            Pose2D new_pose = sim.getRobotPose();
            float dx = new_pose.x - prev_pose.x;
            float dy = new_pose.y - prev_pose.y;
            path_length += std::sqrt(dx * dx + dy * dy);
            prev_pose = new_pose;

            elapsed_time += dt;
        }

        bool goal_reached = sim.checkGoalReached(goal, GOAL_TOLERANCE);

        if (goal_reached) {
            log("Goal reached!");
        } else if (collisions > 0) {
            log("Stopped due to collision");
        } else if (elapsed_time >= max_time) {
            log("Timeout: max simulation time exceeded");
        }

        log("Simulation complete. Time: " + std::to_string(elapsed_time) +
            "s, Path length: " + std::to_string(path_length) +
            "m, Collisions: " + std::to_string(collisions));

        // Save outputs
        sim.saveSnapshot(output_dir + "/trajectory.png");
        sim.saveMetrics(output_dir + "/metrics.json", goal_reached,
                        path_length, collisions, elapsed_time);

        log("Outputs saved to " + output_dir);

        return goal_reached ? 0 : 1;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
