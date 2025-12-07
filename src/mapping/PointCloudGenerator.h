#pragma once

#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

// Point with color
struct ColorPoint {
    float x, y, z;
    uint8_t r, g, b;
};

// Camera intrinsics for depth projection
struct CameraIntrinsics {
    float fx, fy;       // Focal lengths
    float cx, cy;       // Principal point
    float depth_scale;  // Depth units (e.g., 0.001 for mm -> meters)
    int width, height;
};

// Robot pose for point cloud registration
struct RobotPose {
    double x, y, z;        // Position in meters
    double qw, qx, qy, qz; // Orientation quaternion
    double timestamp;
};

// Point cloud generator from RGB-D frames
class PointCloudGenerator {
public:
    PointCloudGenerator();
    ~PointCloudGenerator();

    // Load camera intrinsics from depth_meta.json
    bool loadIntrinsics(const std::string& meta_path);

    // Set intrinsics directly
    void setIntrinsics(const CameraIntrinsics& intrinsics) { intrinsics_ = intrinsics; }

    // Generate point cloud from a single RGB-D frame
    // Returns points in camera frame
    std::vector<ColorPoint> generateFromFrame(const cv::Mat& color, const cv::Mat& depth);

    // Generate point cloud from a single frame and transform to world frame
    std::vector<ColorPoint> generateFromFrame(const cv::Mat& color, const cv::Mat& depth,
                                               const RobotPose& pose);

    // Process entire recorded session
    // Loads all frames, applies poses, builds accumulated point cloud
    // Returns total point count
    size_t processSession(const std::string& session_dir,
                         const std::string& poses_file = "");

    // Save accumulated point cloud
    bool savePointCloud(const std::string& output_path);  // .ply or .pcd

    // Save as simple XYZ text file (for debugging)
    bool saveAsXYZ(const std::string& output_path);

    // Get accumulated point cloud
    const std::vector<ColorPoint>& getPointCloud() const { return cloud_; }

    // Clear accumulated cloud
    void clear() { cloud_.clear(); }

    // Configuration
    void setMaxDepth(float max_meters) { max_depth_ = max_meters; }
    void setMinDepth(float min_meters) { min_depth_ = min_meters; }
    void setDownsampleFactor(int factor) { downsample_factor_ = factor; }
    void setVoxelSize(float size_meters) { voxel_size_ = size_meters; }

private:
    // Transform point by pose
    ColorPoint transformPoint(const ColorPoint& pt, const RobotPose& pose);

    // Load poses from file (CSV or JSON)
    std::vector<RobotPose> loadPoses(const std::string& poses_file);

    // Voxel grid downsampling
    void voxelDownsample();

    CameraIntrinsics intrinsics_;

    // Accumulated point cloud
    std::vector<ColorPoint> cloud_;

    // Configuration
    float max_depth_ = 5.0f;   // Max depth in meters
    float min_depth_ = 0.3f;   // Min depth (RealSense minimum)
    int downsample_factor_ = 2; // Skip every N pixels
    float voxel_size_ = 0.02f;  // 2cm voxel for downsampling
};
