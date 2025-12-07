// Point Cloud Generation Tool
// Usage: generate_pointcloud <session_dir> [output.ply] [--poses poses.csv]
//
// Processes recorded RGB-D frames and generates a colored point cloud.
// Can use robot poses for registration if available.

#include "mapping/PointCloudGenerator.h"
#include <iostream>
#include <string>

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " <session_dir> [output.ply] [options]\n"
              << "\n"
              << "Arguments:\n"
              << "  session_dir    Directory with recorded depth data (from --record-depth)\n"
              << "  output.ply     Output file (default: session_dir/pointcloud.ply)\n"
              << "\n"
              << "Options:\n"
              << "  --poses <file>     Robot poses file (CSV or JSON)\n"
              << "  --voxel <size>     Voxel size for downsampling (default: 0.02m)\n"
              << "  --max-depth <m>    Maximum depth in meters (default: 5.0)\n"
              << "  --min-depth <m>    Minimum depth in meters (default: 0.3)\n"
              << "  --downsample <n>   Skip every N pixels (default: 2)\n"
              << "\n"
              << "Examples:\n"
              << "  " << prog << " data/recordings/session_001\n"
              << "  " << prog << " data/recordings/session_001 output.ply --voxel 0.05\n"
              << "  " << prog << " data/recordings/session_001 map.ply --poses poses.csv\n"
              << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string session_dir = argv[1];
    std::string output_file;
    std::string poses_file;
    float voxel_size = 0.02f;
    float max_depth = 5.0f;
    float min_depth = 0.3f;
    int downsample = 2;

    // Parse arguments
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--poses" && i + 1 < argc) {
            poses_file = argv[++i];
        } else if (arg == "--voxel" && i + 1 < argc) {
            voxel_size = std::stof(argv[++i]);
        } else if (arg == "--max-depth" && i + 1 < argc) {
            max_depth = std::stof(argv[++i]);
        } else if (arg == "--min-depth" && i + 1 < argc) {
            min_depth = std::stof(argv[++i]);
        } else if (arg == "--downsample" && i + 1 < argc) {
            downsample = std::stoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (output_file.empty() && arg[0] != '-') {
            output_file = arg;
        }
    }

    // Default output file
    if (output_file.empty()) {
        output_file = session_dir + "/pointcloud.ply";
    }

    std::cout << "=== Point Cloud Generator ===" << std::endl;
    std::cout << "Session:    " << session_dir << std::endl;
    std::cout << "Output:     " << output_file << std::endl;
    std::cout << "Voxel size: " << voxel_size << " m" << std::endl;
    std::cout << "Depth:      " << min_depth << " - " << max_depth << " m" << std::endl;
    std::cout << "Downsample: " << downsample << "x" << std::endl;
    if (!poses_file.empty()) {
        std::cout << "Poses:      " << poses_file << std::endl;
    }
    std::cout << std::endl;

    // Create generator
    PointCloudGenerator generator;
    generator.setVoxelSize(voxel_size);
    generator.setMaxDepth(max_depth);
    generator.setMinDepth(min_depth);
    generator.setDownsampleFactor(downsample);

    // Process session
    size_t total_points = generator.processSession(session_dir, poses_file);

    if (total_points == 0) {
        std::cerr << "Error: No points generated" << std::endl;
        return 1;
    }

    // Save point cloud
    if (!generator.savePointCloud(output_file)) {
        std::cerr << "Error: Failed to save point cloud" << std::endl;
        return 1;
    }

    std::cout << "\nDone! Generated " << total_points << " points" << std::endl;
    std::cout << "View with: pcl_viewer " << output_file << std::endl;
    std::cout << "Or:        meshlab " << output_file << std::endl;

    return 0;
}
