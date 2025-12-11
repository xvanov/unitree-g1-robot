#include "mapping/PointCloudGenerator.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

PointCloudGenerator::PointCloudGenerator() = default;
PointCloudGenerator::~PointCloudGenerator() = default;

bool PointCloudGenerator::loadIntrinsics(const std::string& meta_path) {
    try {
        std::ifstream f(meta_path);
        if (!f.is_open()) {
            std::cerr << "[PCL] Cannot open " << meta_path << std::endl;
            return false;
        }

        json meta;
        f >> meta;

        intrinsics_.fx = meta.value("fx", 0.0f);
        intrinsics_.fy = meta.value("fy", 0.0f);
        intrinsics_.cx = meta.value("cx", 0.0f);
        intrinsics_.cy = meta.value("cy", 0.0f);
        intrinsics_.depth_scale = meta.value("depth_scale", 0.001f);
        intrinsics_.width = meta.value("width", 640);
        intrinsics_.height = meta.value("height", 480);

        std::cout << "[PCL] Loaded intrinsics: " << intrinsics_.width << "x" << intrinsics_.height
                  << " fx=" << intrinsics_.fx << " fy=" << intrinsics_.fy << std::endl;

        return intrinsics_.fx > 0 && intrinsics_.fy > 0;

    } catch (const std::exception& e) {
        std::cerr << "[PCL] Error loading intrinsics: " << e.what() << std::endl;
        return false;
    }
}

std::vector<ColorPoint> PointCloudGenerator::generateFromFrame(
    const cv::Mat& color, const cv::Mat& depth) {

    std::vector<ColorPoint> points;

    if (color.empty() || depth.empty()) {
        return points;
    }

    // Reserve space (estimate)
    int estimated = (color.rows * color.cols) / (downsample_factor_ * downsample_factor_);
    points.reserve(estimated / 4);  // Not all pixels have valid depth

    for (int v = 0; v < depth.rows; v += downsample_factor_) {
        for (int u = 0; u < depth.cols; u += downsample_factor_) {
            // Get depth value (16-bit, in mm typically)
            uint16_t d_raw = depth.at<uint16_t>(v, u);
            if (d_raw == 0) continue;

            // Convert to meters
            float d = d_raw * intrinsics_.depth_scale;

            // Filter by depth range
            if (d < min_depth_ || d > max_depth_) continue;

            // Project to 3D (camera coordinates)
            ColorPoint pt;
            pt.z = d;
            pt.x = (u - intrinsics_.cx) * d / intrinsics_.fx;
            pt.y = (v - intrinsics_.cy) * d / intrinsics_.fy;

            // Get color
            cv::Vec3b bgr = color.at<cv::Vec3b>(v, u);
            pt.r = bgr[2];
            pt.g = bgr[1];
            pt.b = bgr[0];

            points.push_back(pt);
        }
    }

    return points;
}

ColorPoint PointCloudGenerator::transformPoint(const ColorPoint& pt, const RobotPose& pose) {
    // Convert quaternion to rotation matrix
    double qw = pose.qw, qx = pose.qx, qy = pose.qy, qz = pose.qz;

    // Rotation matrix from quaternion
    double r00 = 1 - 2*(qy*qy + qz*qz);
    double r01 = 2*(qx*qy - qz*qw);
    double r02 = 2*(qx*qz + qy*qw);
    double r10 = 2*(qx*qy + qz*qw);
    double r11 = 1 - 2*(qx*qx + qz*qz);
    double r12 = 2*(qy*qz - qx*qw);
    double r20 = 2*(qx*qz - qy*qw);
    double r21 = 2*(qy*qz + qx*qw);
    double r22 = 1 - 2*(qx*qx + qy*qy);

    // Transform point
    ColorPoint result = pt;
    result.x = r00*pt.x + r01*pt.y + r02*pt.z + pose.x;
    result.y = r10*pt.x + r11*pt.y + r12*pt.z + pose.y;
    result.z = r20*pt.x + r21*pt.y + r22*pt.z + pose.z;

    return result;
}

std::vector<ColorPoint> PointCloudGenerator::generateFromFrame(
    const cv::Mat& color, const cv::Mat& depth, const RobotPose& pose) {

    auto points = generateFromFrame(color, depth);

    // Transform all points to world frame
    for (auto& pt : points) {
        pt = transformPoint(pt, pose);
    }

    return points;
}

std::vector<RobotPose> PointCloudGenerator::loadPoses(const std::string& poses_file) {
    std::vector<RobotPose> poses;

    // Try JSON format first
    if (poses_file.find(".json") != std::string::npos) {
        try {
            std::ifstream f(poses_file);
            json j;
            f >> j;

            for (const auto& p : j["poses"]) {
                RobotPose pose;
                pose.timestamp = p.value("timestamp", 0.0);
                pose.x = p.value("x", 0.0);
                pose.y = p.value("y", 0.0);
                pose.z = p.value("z", 0.0);
                pose.qw = p.value("qw", 1.0);
                pose.qx = p.value("qx", 0.0);
                pose.qy = p.value("qy", 0.0);
                pose.qz = p.value("qz", 0.0);
                poses.push_back(pose);
            }
        } catch (...) {}
    } else {
        // Try CSV format: timestamp,x,y,z,qw,qx,qy,qz
        std::ifstream f(poses_file);
        std::string line;
        std::getline(f, line);  // Skip header

        while (std::getline(f, line)) {
            std::istringstream ss(line);
            RobotPose pose;
            char comma;
            ss >> pose.timestamp >> comma
               >> pose.x >> comma >> pose.y >> comma >> pose.z >> comma
               >> pose.qw >> comma >> pose.qx >> comma >> pose.qy >> comma >> pose.qz;
            poses.push_back(pose);
        }
    }

    return poses;
}

size_t PointCloudGenerator::processSession(const std::string& session_dir,
                                           const std::string& poses_file) {
    // Load intrinsics
    if (!loadIntrinsics(session_dir + "/depth_meta.json")) {
        std::cerr << "[PCL] Failed to load intrinsics" << std::endl;
        return 0;
    }

    // Load poses if provided
    std::vector<RobotPose> poses;
    if (!poses_file.empty()) {
        poses = loadPoses(poses_file);
        std::cout << "[PCL] Loaded " << poses.size() << " poses" << std::endl;
    }

    // Load timestamps
    std::vector<double> timestamps;
    std::ifstream ts_file(session_dir + "/timestamps.txt");
    if (ts_file.is_open()) {
        int idx;
        double ts;
        while (ts_file >> idx >> ts) {
            timestamps.push_back(ts);
        }
    }

    // Get list of depth frames
    std::vector<std::string> depth_files;
    for (const auto& entry : std::filesystem::directory_iterator(session_dir + "/depth")) {
        if (entry.path().extension() == ".png") {
            depth_files.push_back(entry.path().filename().string());
        }
    }
    std::sort(depth_files.begin(), depth_files.end());

    std::cout << "[PCL] Processing " << depth_files.size() << " frames..." << std::endl;

    size_t total_points = 0;

    for (size_t i = 0; i < depth_files.size(); i++) {
        std::string base = depth_files[i].substr(0, depth_files[i].find('.'));

        cv::Mat color = cv::imread(session_dir + "/color/" + base + ".jpg");
        cv::Mat depth = cv::imread(session_dir + "/depth/" + base + ".png", cv::IMREAD_UNCHANGED);

        if (color.empty() || depth.empty()) {
            continue;
        }

        std::vector<ColorPoint> frame_points;

        // If we have poses, find matching pose and transform
        if (!poses.empty() && i < timestamps.size()) {
            double ts = timestamps[i];

            // Find closest pose by timestamp
            RobotPose closest_pose = poses[0];
            double min_diff = std::abs(poses[0].timestamp - ts);

            for (const auto& p : poses) {
                double diff = std::abs(p.timestamp - ts);
                if (diff < min_diff) {
                    min_diff = diff;
                    closest_pose = p;
                }
            }

            frame_points = generateFromFrame(color, depth, closest_pose);
        } else {
            // No poses - use identity (good for single-frame testing)
            frame_points = generateFromFrame(color, depth);
        }

        // Add to accumulated cloud
        cloud_.insert(cloud_.end(), frame_points.begin(), frame_points.end());
        total_points += frame_points.size();

        // Progress
        if ((i + 1) % 100 == 0 || i == depth_files.size() - 1) {
            std::cout << "[PCL] Processed " << (i + 1) << "/" << depth_files.size()
                     << " frames (" << total_points << " points)" << std::endl;
        }
    }

    // Downsample accumulated cloud
    if (voxel_size_ > 0) {
        std::cout << "[PCL] Downsampling with voxel size " << voxel_size_ << "m..." << std::endl;
        size_t before = cloud_.size();
        voxelDownsample();
        std::cout << "[PCL] Downsampled: " << before << " -> " << cloud_.size() << " points" << std::endl;
    }

    return cloud_.size();
}

void PointCloudGenerator::voxelDownsample() {
    if (cloud_.empty() || voxel_size_ <= 0) return;

    // Hash-based voxel grid
    auto voxelKey = [this](const ColorPoint& pt) {
        int vx = static_cast<int>(std::floor(pt.x / voxel_size_));
        int vy = static_cast<int>(std::floor(pt.y / voxel_size_));
        int vz = static_cast<int>(std::floor(pt.z / voxel_size_));
        return std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);
    };

    // Accumulate points per voxel
    struct VoxelData {
        float x = 0, y = 0, z = 0;
        float r = 0, g = 0, b = 0;
        int count = 0;
    };

    std::unordered_map<std::string, VoxelData> voxels;

    for (const auto& pt : cloud_) {
        std::string key = voxelKey(pt);
        auto& v = voxels[key];
        v.x += pt.x;
        v.y += pt.y;
        v.z += pt.z;
        v.r += pt.r;
        v.g += pt.g;
        v.b += pt.b;
        v.count++;
    }

    // Create downsampled cloud
    cloud_.clear();
    cloud_.reserve(voxels.size());

    for (const auto& [key, v] : voxels) {
        ColorPoint pt;
        pt.x = v.x / v.count;
        pt.y = v.y / v.count;
        pt.z = v.z / v.count;
        pt.r = static_cast<uint8_t>(v.r / v.count);
        pt.g = static_cast<uint8_t>(v.g / v.count);
        pt.b = static_cast<uint8_t>(v.b / v.count);
        cloud_.push_back(pt);
    }
}

bool PointCloudGenerator::savePointCloud(const std::string& output_path) {
    if (cloud_.empty()) {
        std::cerr << "[PCL] No points to save" << std::endl;
        return false;
    }

    // Determine format from extension
    bool is_ply = output_path.find(".ply") != std::string::npos;

    std::ofstream f(output_path);
    if (!f.is_open()) {
        std::cerr << "[PCL] Cannot open " << output_path << std::endl;
        return false;
    }

    if (is_ply) {
        // PLY format with color
        f << "ply\n";
        f << "format ascii 1.0\n";
        f << "element vertex " << cloud_.size() << "\n";
        f << "property float x\n";
        f << "property float y\n";
        f << "property float z\n";
        f << "property uchar red\n";
        f << "property uchar green\n";
        f << "property uchar blue\n";
        f << "end_header\n";

        for (const auto& pt : cloud_) {
            f << pt.x << " " << pt.y << " " << pt.z << " "
              << (int)pt.r << " " << (int)pt.g << " " << (int)pt.b << "\n";
        }
    } else {
        // PCD format
        f << "# .PCD v0.7 - Point Cloud Data file format\n";
        f << "VERSION 0.7\n";
        f << "FIELDS x y z rgb\n";
        f << "SIZE 4 4 4 4\n";
        f << "TYPE F F F U\n";
        f << "COUNT 1 1 1 1\n";
        f << "WIDTH " << cloud_.size() << "\n";
        f << "HEIGHT 1\n";
        f << "VIEWPOINT 0 0 0 1 0 0 0\n";
        f << "POINTS " << cloud_.size() << "\n";
        f << "DATA ascii\n";

        for (const auto& pt : cloud_) {
            uint32_t rgb = ((uint32_t)pt.r << 16) | ((uint32_t)pt.g << 8) | pt.b;
            f << pt.x << " " << pt.y << " " << pt.z << " " << rgb << "\n";
        }
    }

    f.close();
    std::cout << "[PCL] Saved " << cloud_.size() << " points to " << output_path << std::endl;
    return true;
}

bool PointCloudGenerator::saveAsXYZ(const std::string& output_path) {
    std::ofstream f(output_path);
    if (!f.is_open()) return false;

    for (const auto& pt : cloud_) {
        f << pt.x << " " << pt.y << " " << pt.z << "\n";
    }

    f.close();
    return true;
}
