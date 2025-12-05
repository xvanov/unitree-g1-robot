#include "slam_sim/SlamSim.h"
#include <iostream>
#include <string>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " --map <path> --output <dir>" << std::endl;
    std::cerr << "  --map     Path to ground truth map PNG" << std::endl;
    std::cerr << "  --output  Output directory for results" << std::endl;
}

int main(int argc, char* argv[]) {
    std::string map_path;
    std::string output_dir;

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc) {
            map_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    if (map_path.empty() || output_dir.empty()) {
        printUsage(argv[0]);
        return 1;
    }

    // Validate map file exists
    if (!fs::exists(map_path)) {
        std::cerr << "ERROR: Map file not found: " << map_path << std::endl;
        return 1;
    }

    // Validate map can be loaded
    cv::Mat test_map = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    if (test_map.empty()) {
        std::cerr << "ERROR: Failed to load map: " << map_path << std::endl;
        return 1;
    }

    // Create output directory if needed
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }

    std::cout << "Running SLAM simulation..." << std::endl;
    std::cout << "  Map: " << map_path << std::endl;
    std::cout << "  Output: " << output_dir << std::endl;

    // Run simulation
    SlamSim slam(map_path);
    slam.run();

    // Compute accuracy
    float accuracy = slam.computeAccuracy();
    std::cout << "  Accuracy: " << (accuracy * 100.0f) << "%" << std::endl;
    std::cout << "  Trajectory poses: " << slam.getTrajectorySize() << std::endl;

    // Save outputs (use filesystem path for proper concatenation)
    fs::path output_path(output_dir);
    std::string built_map_path = (output_path / "built_map.png").string();
    std::string accuracy_path = (output_path / "accuracy.json").string();

    slam.saveBuiltMap(built_map_path);
    slam.saveAccuracy(accuracy_path, accuracy);

    std::cout << "  Built map saved: " << built_map_path << std::endl;
    std::cout << "  Accuracy saved: " << accuracy_path << std::endl;

    // Check accuracy threshold
    if (accuracy >= 0.9f) {
        std::cout << "SUCCESS: Accuracy >= 90%" << std::endl;
        return 0;
    } else {
        std::cout << "WARNING: Accuracy below 90% threshold" << std::endl;
        return 0;  // Still exit 0 - this is a warning, not an error
    }
}
