#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

#include "capture/ImageCapture.h"
#include "capture/PlanCorrelator.h"
#include "plan/PlanManager.h"

namespace fs = std::filesystem;

// JSON serialization for existing types (matching ImageCapture.cpp)
namespace nlohmann {
    template <>
    struct adl_serializer<Point2D> {
        static void to_json(json& j, const Point2D& p) {
            j = json{{"x", p.x}, {"y", p.y}};
        }
        static void from_json(const json& j, Point2D& p) {
            j.at("x").get_to(p.x);
            j.at("y").get_to(p.y);
        }
    };

    template <>
    struct adl_serializer<Pose2D> {
        static void to_json(json& j, const Pose2D& p) {
            j = json{{"x", p.x}, {"y", p.y}, {"theta", p.theta}};
        }
        static void from_json(const json& j, Pose2D& p) {
            j.at("x").get_to(p.x);
            j.at("y").get_to(p.y);
            j.at("theta").get_to(p.theta);
        }
    };
}

// Test fixture for ImageCapture tests
class ImageCaptureTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a temporary test directory
        test_dir_ = "test_capture_output";
        fs::create_directories(test_dir_);
    }

    void TearDown() override {
        // Clean up test directory
        if (fs::exists(test_dir_)) {
            fs::remove_all(test_dir_);
        }
    }

    std::string test_dir_;
};

// Test fixture for PlanCorrelator tests
class PlanCorrelatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        plan_manager_ = std::make_unique<PlanManager>();

        // Create a simple test plan image (100x100 white image with some black walls)
        test_plan_ = cv::Mat(100, 100, CV_8UC1, cv::Scalar(255));
        // Add a wall around the edge
        cv::rectangle(test_plan_, cv::Point(0, 0), cv::Point(99, 99), cv::Scalar(0), 1);

        // Save test plan
        test_plan_path_ = "test_plan.png";
        cv::imwrite(test_plan_path_, test_plan_);
    }

    void TearDown() override {
        if (fs::exists(test_plan_path_)) {
            fs::remove(test_plan_path_);
        }
        if (fs::exists("test_coverage.png")) {
            fs::remove("test_coverage.png");
        }
    }

    std::unique_ptr<PlanManager> plan_manager_;
    cv::Mat test_plan_;
    std::string test_plan_path_;
};

// ============================================
// ImageCapture Tests
// ============================================

TEST_F(ImageCaptureTest, Constructor) {
    ImageCapture capture;
    EXPECT_FALSE(capture.isCapturing());
    EXPECT_EQ(capture.getImageCount(), 0);
}

TEST_F(ImageCaptureTest, StartStopCapture) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);

    // Start capture
    EXPECT_TRUE(capture.startCapture("test_session"));
    EXPECT_TRUE(capture.isCapturing());

    // Verify session directory was created
    std::string session_dir = test_dir_ + "/test_session/images";
    EXPECT_TRUE(fs::exists(session_dir));

    // Stop capture
    capture.stopCapture();
    EXPECT_FALSE(capture.isCapturing());
}

TEST_F(ImageCaptureTest, CaptureIntervalEnforced) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);
    capture.setCaptureInterval(1.0f);  // 1 second interval

    EXPECT_TRUE(capture.startCapture("interval_test"));

    // Without a camera, captureFrame should return false
    // but it should not cause errors
    Pose2D pose{1.0f, 2.0f, 0.5f};
    bool result = capture.captureFrame(pose);

    // Camera not available, so result should be false
    EXPECT_FALSE(result);

    // But the capture should still be active
    EXPECT_TRUE(capture.isCapturing());

    capture.stopCapture();
}

TEST_F(ImageCaptureTest, SessionDirectoryCreation) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);

    // Start multiple sessions
    EXPECT_TRUE(capture.startCapture("session_1"));
    capture.stopCapture();

    EXPECT_TRUE(capture.startCapture("session_2"));
    capture.stopCapture();

    // Verify both directories exist
    EXPECT_TRUE(fs::exists(test_dir_ + "/session_1/images"));
    EXPECT_TRUE(fs::exists(test_dir_ + "/session_2/images"));
}

TEST_F(ImageCaptureTest, CannotStartCaptureWhileCapturing) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);

    EXPECT_TRUE(capture.startCapture("first"));
    EXPECT_TRUE(capture.isCapturing());

    // Second start should fail
    EXPECT_FALSE(capture.startCapture("second"));

    capture.stopCapture();
}

TEST_F(ImageCaptureTest, OutputDirConfiguration) {
    ImageCapture capture;

    std::string custom_dir = test_dir_ + "/custom";
    capture.setOutputDir(custom_dir);

    EXPECT_TRUE(capture.startCapture("test"));
    EXPECT_TRUE(fs::exists(custom_dir + "/test/images"));

    capture.stopCapture();
}

TEST_F(ImageCaptureTest, GracefulCameraUnavailable) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);
    capture.setCameraIndex(999);  // Non-existent camera

    // Should still start successfully (graceful degradation)
    EXPECT_TRUE(capture.startCapture("no_camera_test"));
    EXPECT_TRUE(capture.isCapturing());
    EXPECT_FALSE(capture.isCameraAvailable());

    // Capture frame should handle missing camera gracefully
    Pose2D pose{0, 0, 0};
    EXPECT_FALSE(capture.captureFrame(pose));

    capture.stopCapture();
}

// ============================================
// PlanCorrelator Tests
// ============================================

TEST_F(PlanCorrelatorTest, Constructor) {
    PlanCorrelator correlator;
    EXPECT_FALSE(correlator.isInitialized());
    EXPECT_EQ(correlator.getCoveragePercent(), 0.0f);
}

TEST_F(PlanCorrelatorTest, InitFromPlan) {
    EXPECT_TRUE(plan_manager_->loadPlan(test_plan_path_));

    PlanCorrelator correlator(plan_manager_.get());
    correlator.initFromPlan();

    EXPECT_TRUE(correlator.isInitialized());
    EXPECT_GT(correlator.getTotalFreePixels(), 0);
    EXPECT_EQ(correlator.getCoveragePercent(), 0.0f);  // No coverage yet
}

TEST_F(PlanCorrelatorTest, CoverageUpdates) {
    EXPECT_TRUE(plan_manager_->loadPlan(test_plan_path_));
    plan_manager_->setStartPosition({0.5f, 0.5f}, 0.0f);

    PlanCorrelator correlator(plan_manager_.get());
    correlator.initFromPlan();

    // Initial coverage should be 0
    float initial_coverage = correlator.getCoveragePercent();
    EXPECT_FLOAT_EQ(initial_coverage, 0.0f);

    // Update coverage at a position
    Pose2D pose{0.5f, 0.5f, 0.0f};
    correlator.updateCoverage(pose);

    // Coverage should have increased
    float new_coverage = correlator.getCoveragePercent();
    EXPECT_GT(new_coverage, 0.0f);
}

TEST_F(PlanCorrelatorTest, FOVCalculation) {
    PlanCorrelator correlator;

    CameraParams params;
    params.hfov_deg = 60.0f;
    params.max_range_m = 5.0f;
    correlator.setCameraParams(params);

    Pose2D pose{1.0f, 1.0f, 0.0f};
    auto fov = correlator.calculateFOV(pose);

    // FOV should have 3 points (triangle)
    EXPECT_EQ(fov.size(), 3u);

    // First point is robot position
    EXPECT_FLOAT_EQ(fov[0].x, 1.0f);
    EXPECT_FLOAT_EQ(fov[0].y, 1.0f);

    // Other points should be in front of robot (positive x direction when theta=0)
    EXPECT_GT(fov[1].x, 1.0f);
    EXPECT_GT(fov[2].x, 1.0f);
}

TEST_F(PlanCorrelatorTest, ResetCoverage) {
    EXPECT_TRUE(plan_manager_->loadPlan(test_plan_path_));
    plan_manager_->setStartPosition({0.5f, 0.5f}, 0.0f);

    PlanCorrelator correlator(plan_manager_.get());
    correlator.initFromPlan();

    // Add some coverage
    Pose2D pose{0.5f, 0.5f, 0.0f};
    correlator.updateCoverage(pose);
    EXPECT_GT(correlator.getCoveragePercent(), 0.0f);

    // Reset coverage
    correlator.resetCoverage();
    EXPECT_FLOAT_EQ(correlator.getCoveragePercent(), 0.0f);
}

TEST_F(PlanCorrelatorTest, SaveCoverageMap) {
    EXPECT_TRUE(plan_manager_->loadPlan(test_plan_path_));
    plan_manager_->setStartPosition({0.5f, 0.5f}, 0.0f);

    PlanCorrelator correlator(plan_manager_.get());
    correlator.initFromPlan();

    // Add some coverage
    Pose2D pose{0.5f, 0.5f, 0.0f};
    correlator.updateCoverage(pose);

    // Save coverage map
    std::string coverage_path = "test_coverage.png";
    correlator.saveCoverageMap(coverage_path);

    // Verify file was created
    EXPECT_TRUE(fs::exists(coverage_path));
}

TEST_F(PlanCorrelatorTest, NoPlanLoaded) {
    PlanManager empty_plan;
    PlanCorrelator correlator(&empty_plan);

    // initFromPlan should handle gracefully
    correlator.initFromPlan();
    EXPECT_FALSE(correlator.isInitialized());

    // updateCoverage should not crash
    Pose2D pose{0, 0, 0};
    correlator.updateCoverage(pose);  // Should not crash

    EXPECT_FLOAT_EQ(correlator.getCoveragePercent(), 0.0f);
}

// HIGH-3 FIX: Test coverage coordinate system with specific robot positions
TEST_F(PlanCorrelatorTest, CoverageCoordinateSystem) {
    // Plan is 100x100 at 0.02m/pixel = 2m x 2m
    EXPECT_TRUE(plan_manager_->loadPlan(test_plan_path_));

    // Set robot start at plan position (0.5m, 0.5m) = pixel (25, 25)
    plan_manager_->setStartPosition({0.5f, 0.5f}, 0.0f);

    PlanCorrelator correlator(plan_manager_.get());
    correlator.initFromPlan();

    EXPECT_TRUE(correlator.isInitialized());

    // Robot at (0,0) in robot frame = (0.5, 0.5) in plan frame = pixel (25, 25)
    Pose2D pose{0.0f, 0.0f, 0.0f};

    // Update coverage - should mark area in center of plan, not at (0,0) pixel
    correlator.updateCoverage(pose);

    // Coverage should increase
    float coverage = correlator.getCoveragePercent();
    EXPECT_GT(coverage, 0.0f);
    EXPECT_LT(coverage, 100.0f);  // Should not cover entire plan

    // Get pixel count for verification
    int covered = correlator.getCoveredPixels();
    EXPECT_GT(covered, 0);
}

TEST_F(PlanCorrelatorTest, CoverageMultiplePositions) {
    EXPECT_TRUE(plan_manager_->loadPlan(test_plan_path_));
    plan_manager_->setStartPosition({0.5f, 0.5f}, 0.0f);

    PlanCorrelator correlator(plan_manager_.get());
    correlator.initFromPlan();

    // Get initial coverage
    float initial = correlator.getCoveragePercent();
    EXPECT_FLOAT_EQ(initial, 0.0f);

    // Update at one position
    Pose2D pose1{0.0f, 0.0f, 0.0f};
    correlator.updateCoverage(pose1);
    float after_first = correlator.getCoveragePercent();
    EXPECT_GT(after_first, 0.0f);

    // Update at different position - should increase coverage
    Pose2D pose2{0.5f, 0.0f, 0.0f};  // Move right
    correlator.updateCoverage(pose2);
    float after_second = correlator.getCoveragePercent();

    // Coverage should be >= first (monotonic increase)
    EXPECT_GE(after_second, after_first);
}

// ============================================
// ImageMetadata Tests
// ============================================

TEST(ImageMetadataTest, DefaultValues) {
    ImageMetadata meta;
    EXPECT_TRUE(meta.image_path.empty());
    EXPECT_EQ(meta.timestamp_ms, 0);
    EXPECT_EQ(meta.sequence_number, 0);
}

TEST(ImageMetadataTest, Assignment) {
    ImageMetadata meta;
    meta.image_path = "images/img_0001.jpg";
    meta.timestamp_ms = 1733356800000;
    meta.robot_pose = {1.5f, 2.5f, 0.785f};
    meta.plan_coords = {5.0f, 6.0f};
    meta.camera_yaw = 0.785f;
    meta.sequence_number = 1;

    EXPECT_EQ(meta.image_path, "images/img_0001.jpg");
    EXPECT_EQ(meta.timestamp_ms, 1733356800000);
    EXPECT_FLOAT_EQ(meta.robot_pose.x, 1.5f);
    EXPECT_FLOAT_EQ(meta.robot_pose.y, 2.5f);
    EXPECT_FLOAT_EQ(meta.robot_pose.theta, 0.785f);
    EXPECT_FLOAT_EQ(meta.plan_coords.x, 5.0f);
    EXPECT_FLOAT_EQ(meta.plan_coords.y, 6.0f);
    EXPECT_EQ(meta.sequence_number, 1);
}

// ============================================
// JSON Serialization Tests
// ============================================

TEST(JsonSerializationTest, Point2DSerialization) {
    Point2D original{3.14f, 2.71f};

    // Serialize
    nlohmann::json j = original;
    EXPECT_EQ(j["x"], 3.14f);
    EXPECT_EQ(j["y"], 2.71f);

    // Deserialize
    Point2D restored = j.get<Point2D>();
    EXPECT_FLOAT_EQ(restored.x, original.x);
    EXPECT_FLOAT_EQ(restored.y, original.y);
}

TEST(JsonSerializationTest, Pose2DSerialization) {
    Pose2D original{1.5f, 2.5f, 0.785f};

    // Serialize
    nlohmann::json j = original;
    EXPECT_EQ(j["x"], 1.5f);
    EXPECT_EQ(j["y"], 2.5f);
    EXPECT_EQ(j["theta"], 0.785f);

    // Deserialize
    Pose2D restored = j.get<Pose2D>();
    EXPECT_FLOAT_EQ(restored.x, original.x);
    EXPECT_FLOAT_EQ(restored.y, original.y);
    EXPECT_FLOAT_EQ(restored.theta, original.theta);
}

TEST(JsonSerializationTest, ImageMetadataToJson) {
    ImageMetadata meta;
    meta.image_path = "images/img_00000001.jpg";
    meta.timestamp_ms = 1733356800000;
    meta.robot_pose = {1.5f, 2.5f, 0.785f};
    meta.plan_coords = {5.0f, 6.0f};
    meta.camera_yaw = 0.785f;
    meta.sequence_number = 1;

    // Manual serialization (matching ImageCapture::saveMetadata pattern)
    nlohmann::json j;
    j["image_path"] = meta.image_path;
    j["timestamp_ms"] = meta.timestamp_ms;
    j["robot_pose"] = meta.robot_pose;
    j["plan_coords"] = meta.plan_coords;
    j["camera_yaw"] = meta.camera_yaw;
    j["sequence_number"] = meta.sequence_number;

    // Verify structure
    EXPECT_EQ(j["image_path"], "images/img_00000001.jpg");
    EXPECT_EQ(j["timestamp_ms"], 1733356800000);
    EXPECT_EQ(j["robot_pose"]["x"], 1.5f);
    EXPECT_EQ(j["robot_pose"]["y"], 2.5f);
    EXPECT_EQ(j["robot_pose"]["theta"], 0.785f);
    EXPECT_EQ(j["plan_coords"]["x"], 5.0f);
    EXPECT_EQ(j["plan_coords"]["y"], 6.0f);
    EXPECT_EQ(j["camera_yaw"], 0.785f);
    EXPECT_EQ(j["sequence_number"], 1);
}

TEST(JsonSerializationTest, ImageMetadataRoundTrip) {
    // Create original metadata
    ImageMetadata original;
    original.image_path = "images/img_00000042.jpg";
    original.timestamp_ms = 1733356812345;
    original.robot_pose = {10.5f, 20.3f, 1.57f};
    original.plan_coords = {15.2f, 25.7f};
    original.camera_yaw = 1.57f;
    original.sequence_number = 42;

    // Serialize to JSON
    nlohmann::json j;
    j["image_path"] = original.image_path;
    j["timestamp_ms"] = original.timestamp_ms;
    j["robot_pose"] = original.robot_pose;
    j["plan_coords"] = original.plan_coords;
    j["camera_yaw"] = original.camera_yaw;
    j["sequence_number"] = original.sequence_number;

    // Convert to string and back (simulates file I/O)
    std::string json_str = j.dump(2);
    nlohmann::json restored_j = nlohmann::json::parse(json_str);

    // Deserialize back
    ImageMetadata restored;
    restored.image_path = restored_j["image_path"];
    restored.timestamp_ms = restored_j["timestamp_ms"];
    restored.robot_pose = restored_j["robot_pose"].get<Pose2D>();
    restored.plan_coords = restored_j["plan_coords"].get<Point2D>();
    restored.camera_yaw = restored_j["camera_yaw"];
    restored.sequence_number = restored_j["sequence_number"];

    // Verify round-trip
    EXPECT_EQ(restored.image_path, original.image_path);
    EXPECT_EQ(restored.timestamp_ms, original.timestamp_ms);
    EXPECT_FLOAT_EQ(restored.robot_pose.x, original.robot_pose.x);
    EXPECT_FLOAT_EQ(restored.robot_pose.y, original.robot_pose.y);
    EXPECT_FLOAT_EQ(restored.robot_pose.theta, original.robot_pose.theta);
    EXPECT_FLOAT_EQ(restored.plan_coords.x, original.plan_coords.x);
    EXPECT_FLOAT_EQ(restored.plan_coords.y, original.plan_coords.y);
    EXPECT_FLOAT_EQ(restored.camera_yaw, original.camera_yaw);
    EXPECT_EQ(restored.sequence_number, original.sequence_number);
}

TEST(JsonSerializationTest, JsonFileWriteRead) {
    // Create metadata
    ImageMetadata meta;
    meta.image_path = "images/img_00000001.jpg";
    meta.timestamp_ms = 1733356800000;
    meta.robot_pose = {1.5f, 2.5f, 0.785f};
    meta.plan_coords = {5.0f, 6.0f};
    meta.camera_yaw = 0.785f;
    meta.sequence_number = 1;

    // Write to temp file
    std::string temp_path = "test_metadata.json";
    {
        nlohmann::json j;
        j["image_path"] = meta.image_path;
        j["timestamp_ms"] = meta.timestamp_ms;
        j["robot_pose"] = meta.robot_pose;
        j["plan_coords"] = meta.plan_coords;
        j["camera_yaw"] = meta.camera_yaw;
        j["sequence_number"] = meta.sequence_number;

        std::ofstream out(temp_path);
        ASSERT_TRUE(out.is_open());
        out << j.dump(2);
        out.close();
    }

    // Read back
    {
        std::ifstream in(temp_path);
        ASSERT_TRUE(in.is_open());
        nlohmann::json j = nlohmann::json::parse(in);

        EXPECT_EQ(j["image_path"], meta.image_path);
        EXPECT_EQ(j["timestamp_ms"], meta.timestamp_ms);
        EXPECT_FLOAT_EQ(j["robot_pose"]["x"].get<float>(), meta.robot_pose.x);
    }

    // Cleanup
    fs::remove(temp_path);
}

// HIGH-2 FIX: Test loadMetadata static function
TEST(JsonSerializationTest, LoadMetadataFromFile) {
    // Create a test metadata JSON file
    std::string temp_path = "test_load_metadata.json";
    {
        nlohmann::json j;
        j["image_path"] = "images/img_00000042.jpg";
        j["timestamp_ms"] = 1733356812345;
        j["robot_pose"] = {{"x", 10.5f}, {"y", 20.3f}, {"theta", 1.57f}};
        j["plan_coords"] = {{"x", 15.2f}, {"y", 25.7f}};
        j["camera_yaw"] = 1.57f;
        j["sequence_number"] = 42;

        std::ofstream out(temp_path);
        ASSERT_TRUE(out.is_open());
        out << j.dump(2);
        out.close();
    }

    // Use loadMetadata to read it
    ImageMetadata loaded;
    ASSERT_TRUE(ImageCapture::loadMetadata(temp_path, loaded));

    EXPECT_EQ(loaded.image_path, "images/img_00000042.jpg");
    EXPECT_EQ(loaded.timestamp_ms, 1733356812345);
    EXPECT_FLOAT_EQ(loaded.robot_pose.x, 10.5f);
    EXPECT_FLOAT_EQ(loaded.robot_pose.y, 20.3f);
    EXPECT_FLOAT_EQ(loaded.robot_pose.theta, 1.57f);
    EXPECT_FLOAT_EQ(loaded.plan_coords.x, 15.2f);
    EXPECT_FLOAT_EQ(loaded.plan_coords.y, 25.7f);
    EXPECT_FLOAT_EQ(loaded.camera_yaw, 1.57f);
    EXPECT_EQ(loaded.sequence_number, 42);

    // Cleanup
    fs::remove(temp_path);
}

TEST(JsonSerializationTest, LoadMetadataNonexistentFile) {
    ImageMetadata loaded;
    EXPECT_FALSE(ImageCapture::loadMetadata("nonexistent_file_xyz.json", loaded));
}

TEST(JsonSerializationTest, LoadMetadataInvalidJson) {
    // Create a file with invalid JSON
    std::string temp_path = "test_invalid_json.json";
    {
        std::ofstream out(temp_path);
        out << "{ invalid json content";
        out.close();
    }

    ImageMetadata loaded;
    EXPECT_FALSE(ImageCapture::loadMetadata(temp_path, loaded));

    fs::remove(temp_path);
}

// ============================================
// Disk Space Tests (MEDIUM-2 fix)
// ============================================

TEST(DiskSpaceTest, GetAvailableDiskSpace) {
    // Test on current directory (should always have some space)
    size_t available = ImageCapture::getAvailableDiskSpace(".");
    EXPECT_GT(available, 0u);
}

TEST(DiskSpaceTest, GetAvailableDiskSpaceNonexistent) {
    // Nonexistent path should return 0
    size_t available = ImageCapture::getAvailableDiskSpace("/nonexistent/path/xyz");
    EXPECT_EQ(available, 0u);
}

// ============================================
// Session Management Tests (MEDIUM-3 fix)
// ============================================

TEST_F(ImageCaptureTest, ListSessions) {
    // Create some test sessions
    fs::create_directories(test_dir_ + "/session_a/images");
    fs::create_directories(test_dir_ + "/session_b/images");
    fs::create_directories(test_dir_ + "/session_c/images");

    auto sessions = ImageCapture::listSessions(test_dir_);

    EXPECT_EQ(sessions.size(), 3u);
    // Should be sorted alphabetically
    EXPECT_EQ(sessions[0], "session_a");
    EXPECT_EQ(sessions[1], "session_b");
    EXPECT_EQ(sessions[2], "session_c");
}

TEST_F(ImageCaptureTest, ListSessionsEmptyDir) {
    auto sessions = ImageCapture::listSessions(test_dir_);
    EXPECT_EQ(sessions.size(), 0u);
}

TEST_F(ImageCaptureTest, ListSessionsNonexistentDir) {
    auto sessions = ImageCapture::listSessions("nonexistent_dir_xyz");
    EXPECT_EQ(sessions.size(), 0u);
}

TEST_F(ImageCaptureTest, DeleteSession) {
    // Create a test session with some files
    std::string session_path = test_dir_ + "/to_delete";
    fs::create_directories(session_path + "/images");

    // Create a dummy file
    std::ofstream(session_path + "/images/test.jpg") << "dummy";

    EXPECT_TRUE(fs::exists(session_path));

    // Delete it
    EXPECT_TRUE(ImageCapture::deleteSession(session_path));
    EXPECT_FALSE(fs::exists(session_path));
}

TEST_F(ImageCaptureTest, DeleteNonexistentSession) {
    EXPECT_FALSE(ImageCapture::deleteSession("nonexistent_session_xyz"));
}

TEST_F(ImageCaptureTest, GetTotalDiskUsage) {
    // Create a test session with known file sizes
    std::string session_path = test_dir_ + "/usage_test";
    fs::create_directories(session_path);

    // Create files with known content
    std::string content1 = "Hello, World!";  // 13 bytes
    std::string content2 = "Test data here"; // 14 bytes

    std::ofstream(session_path + "/file1.txt") << content1;
    std::ofstream(session_path + "/file2.txt") << content2;

    size_t usage = ImageCapture::getTotalDiskUsage(test_dir_);
    EXPECT_GE(usage, 27u);  // At least 27 bytes
}

TEST_F(ImageCaptureTest, GetTotalDiskUsageNonexistentDir) {
    size_t usage = ImageCapture::getTotalDiskUsage("nonexistent_dir_xyz");
    EXPECT_EQ(usage, 0u);
}

// ============================================
// HIGH-2 FIX: Mock Camera Tests (actual capture verification)
// ============================================

TEST_F(ImageCaptureTest, CaptureTestFrameSavesImage) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);
    capture.setCaptureInterval(0.0f);  // No interval for testing

    ASSERT_TRUE(capture.startCapture("mock_capture_test"));

    // Create a test frame (640x480 with gradient pattern)
    cv::Mat test_frame(480, 640, CV_8UC3);
    for (int y = 0; y < 480; ++y) {
        for (int x = 0; x < 640; ++x) {
            test_frame.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<uint8_t>(x % 256),
                static_cast<uint8_t>(y % 256),
                128);
        }
    }

    Pose2D pose{1.5f, 2.5f, 0.785f};
    EXPECT_TRUE(capture.captureTestFrame(test_frame, pose));

    // Wait for async save to complete
    capture.waitForPendingSaves();

    // Verify image was saved
    std::string img_path = test_dir_ + "/mock_capture_test/images/img_00000001.jpg";
    EXPECT_TRUE(fs::exists(img_path)) << "Image file should exist: " << img_path;

    // Verify metadata was saved
    std::string meta_path = test_dir_ + "/mock_capture_test/images/img_00000001.json";
    EXPECT_TRUE(fs::exists(meta_path)) << "Metadata file should exist: " << meta_path;

    // Verify image count
    EXPECT_EQ(capture.getImageCount(), 1);

    // Verify getCapturedImages returns the metadata
    auto images = capture.getCapturedImages();
    EXPECT_EQ(images.size(), 1u);
    if (!images.empty()) {
        EXPECT_EQ(images[0].sequence_number, 1);
        EXPECT_FLOAT_EQ(images[0].robot_pose.x, 1.5f);
        EXPECT_FLOAT_EQ(images[0].robot_pose.y, 2.5f);
    }

    capture.stopCapture();
}

TEST_F(ImageCaptureTest, CaptureTestFrameMetadataContent) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);
    capture.setCaptureInterval(0.0f);

    ASSERT_TRUE(capture.startCapture("metadata_test"));

    cv::Mat test_frame(480, 640, CV_8UC3, cv::Scalar(100, 150, 200));
    Pose2D pose{10.0f, 20.0f, 1.57f};

    EXPECT_TRUE(capture.captureTestFrame(test_frame, pose));
    capture.waitForPendingSaves();

    // Load and verify metadata
    std::string meta_path = test_dir_ + "/metadata_test/images/img_00000001.json";
    ImageMetadata loaded;
    ASSERT_TRUE(ImageCapture::loadMetadata(meta_path, loaded));

    EXPECT_EQ(loaded.image_path, "images/img_00000001.jpg");
    EXPECT_FLOAT_EQ(loaded.robot_pose.x, 10.0f);
    EXPECT_FLOAT_EQ(loaded.robot_pose.y, 20.0f);
    EXPECT_FLOAT_EQ(loaded.robot_pose.theta, 1.57f);
    EXPECT_EQ(loaded.sequence_number, 1);
    EXPECT_GT(loaded.timestamp_ms, 0);  // Should have valid timestamp

    capture.stopCapture();
}

TEST_F(ImageCaptureTest, CaptureTestFrameIntervalEnforced) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);
    capture.setCaptureInterval(0.5f);  // 500ms interval (shorter for faster test)

    ASSERT_TRUE(capture.startCapture("interval_test_real"));

    cv::Mat test_frame(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
    Pose2D pose{0, 0, 0};

    // First capture should succeed
    EXPECT_TRUE(capture.captureTestFrame(test_frame, pose));

    // Immediate second capture should fail (interval not elapsed)
    EXPECT_FALSE(capture.captureTestFrame(test_frame, pose));

    // Wait for interval plus margin
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // Now should succeed
    EXPECT_TRUE(capture.captureTestFrame(test_frame, pose));

    capture.waitForPendingSaves();
    EXPECT_EQ(capture.getImageCount(), 2);

    capture.stopCapture();
}

TEST_F(ImageCaptureTest, CaptureTestFrameMultipleImages) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);
    capture.setCaptureInterval(0.0f);  // No interval

    ASSERT_TRUE(capture.startCapture("multi_image_test"));

    cv::Mat test_frame(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));

    // Capture 5 frames with different poses
    for (int i = 0; i < 5; ++i) {
        Pose2D pose{static_cast<float>(i), static_cast<float>(i * 2), static_cast<float>(i) * 0.1f};
        EXPECT_TRUE(capture.captureTestFrame(test_frame, pose));
        // Small delay to ensure unique timestamps
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    capture.waitForPendingSaves();

    EXPECT_EQ(capture.getImageCount(), 5);

    // Verify all files exist
    for (int i = 1; i <= 5; ++i) {
        char filename[64];
        std::snprintf(filename, sizeof(filename), "img_%08d.jpg", i);
        std::string path = test_dir_ + "/multi_image_test/images/" + filename;
        EXPECT_TRUE(fs::exists(path)) << "Missing: " << path;
    }

    // Verify getCapturedImages has all entries
    auto images = capture.getCapturedImages();
    EXPECT_EQ(images.size(), 5u);

    capture.stopCapture();
}

TEST_F(ImageCaptureTest, WaitForPendingSaves) {
    ImageCapture capture;
    capture.setOutputDir(test_dir_);
    capture.setCaptureInterval(0.0f);

    ASSERT_TRUE(capture.startCapture("wait_test"));

    cv::Mat test_frame(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
    Pose2D pose{0, 0, 0};

    EXPECT_TRUE(capture.captureTestFrame(test_frame, pose));

    // Immediately after capture, getCapturedImages might not have the entry
    // (async save in progress)

    // After waiting, it should be there
    capture.waitForPendingSaves();
    auto images = capture.getCapturedImages();
    EXPECT_EQ(images.size(), 1u);

    capture.stopCapture();
}

// ============================================
// Integration Tests
// ============================================

TEST_F(ImageCaptureTest, ImageCaptureWithPlanManager) {
    // Create a plan
    auto plan_manager = std::make_unique<PlanManager>();

    // Create a simple test plan
    cv::Mat test_plan(100, 100, CV_8UC1, cv::Scalar(255));
    std::string plan_path = test_dir_ + "/test_plan.png";
    cv::imwrite(plan_path, test_plan);

    EXPECT_TRUE(plan_manager->loadPlan(plan_path));

    // Create capture with plan manager
    ImageCapture capture(nullptr, plan_manager.get());
    capture.setOutputDir(test_dir_);

    EXPECT_TRUE(capture.startCapture("integrated_test"));
    capture.stopCapture();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
