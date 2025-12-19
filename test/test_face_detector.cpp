#include <gtest/gtest.h>
#include "greeter/FaceDetector.h"
#include <opencv2/opencv.hpp>

using namespace greeter;

class FaceDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Model paths
        prototxt_path_ = "models/face_detection/deploy.prototxt";
        caffemodel_path_ = "models/face_detection/res10_300x300_ssd_iter_140000.caffemodel";
    }

    std::string prototxt_path_;
    std::string caffemodel_path_;
};

// Test: Model loads from valid paths
TEST_F(FaceDetectorTest, ModelLoadsFromValidPaths) {
    FaceDetector detector;
    bool result = detector.init(prototxt_path_, caffemodel_path_);
    EXPECT_TRUE(result);
    EXPECT_TRUE(detector.isInitialized());
}

// Test: Model fails gracefully with invalid paths (returns false, no crash)
TEST_F(FaceDetectorTest, FailsGracefullyWithInvalidPaths) {
    FaceDetector detector;
    bool result = detector.init("nonexistent.prototxt", "nonexistent.caffemodel");
    EXPECT_FALSE(result);
    EXPECT_FALSE(detector.isInitialized());
}

// Test: Returns empty for uninitialized detector
TEST_F(FaceDetectorTest, ReturnsEmptyForUninitializedDetector) {
    FaceDetector detector;
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    std::vector<FaceRect> faces = detector.detect(frame);
    EXPECT_TRUE(faces.empty());
}

// Test: Returns empty for empty frame
TEST_F(FaceDetectorTest, ReturnsEmptyForEmptyFrame) {
    FaceDetector detector;
    detector.init(prototxt_path_, caffemodel_path_);

    cv::Mat empty_frame;
    std::vector<FaceRect> faces = detector.detect(empty_frame);
    EXPECT_TRUE(faces.empty());
}

// Test: Detection time measurement works
TEST_F(FaceDetectorTest, DetectionTimeMeasurementWorks) {
    FaceDetector detector;
    detector.init(prototxt_path_, caffemodel_path_);

    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    detector.detect(frame);

    double time_ms = detector.getLastDetectionTimeMs();
    EXPECT_GE(time_ms, 0.0);
}

// Test: Confidence filtering (only >threshold)
TEST_F(FaceDetectorTest, ConfidenceFiltering) {
    FaceDetector detector;
    detector.init(prototxt_path_, caffemodel_path_);

    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);

    // Detect with high threshold - should return fewer faces
    std::vector<FaceRect> high_threshold = detector.detect(frame, 0.9f);
    std::vector<FaceRect> low_threshold = detector.detect(frame, 0.1f);

    // Can't guarantee what's detected, but high threshold should be <= low
    EXPECT_LE(high_threshold.size(), low_threshold.size());

    // All faces should be above threshold
    for (const auto& face : high_threshold) {
        EXPECT_GT(face.confidence, 0.9f);
    }
}

// Test: FaceRect helper methods work correctly
TEST_F(FaceDetectorTest, FaceRectHelperMethods) {
    FaceRect face;
    face.bounding_box = cv::Rect(100, 50, 80, 100);
    face.confidence = 0.95f;

    EXPECT_EQ(face.centerX(), 140);  // 100 + 80/2
    EXPECT_EQ(face.centerY(), 100);  // 50 + 100/2
    EXPECT_EQ(face.bottom(), 150);   // 50 + 100
}

// TODO: Add test with real face image (test/assets/test_face.jpg)
// Currently all tests use synthetic images. A test with a real face would
// provide stronger validation of AC1 (face detection with >0.5 confidence).
// Test asset should be 640x480 with a single clearly visible face.

// Test: Bounding box within image bounds (synthetic test)
TEST_F(FaceDetectorTest, BoundingBoxWithinImageBounds) {
    FaceDetector detector;
    detector.init(prototxt_path_, caffemodel_path_);

    // Create a synthetic image with a face-like region
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    // Draw a skin-colored oval in the center
    cv::ellipse(frame, cv::Point(320, 240), cv::Size(60, 80), 0, 0, 360,
                cv::Scalar(100, 140, 180), -1);

    std::vector<FaceRect> faces = detector.detect(frame, 0.3f);

    for (const auto& face : faces) {
        // All coordinates should be within image bounds
        EXPECT_GE(face.bounding_box.x, 0);
        EXPECT_GE(face.bounding_box.y, 0);
        EXPECT_LT(face.bounding_box.x + face.bounding_box.width, 641);
        EXPECT_LT(face.bounding_box.y + face.bounding_box.height, 481);
        EXPECT_GT(face.bounding_box.width, 0);
        EXPECT_GT(face.bounding_box.height, 0);
    }
}

// Test: Detection on black frame returns empty (no false positives)
TEST_F(FaceDetectorTest, NoFalsePositivesOnBlackFrame) {
    FaceDetector detector;
    detector.init(prototxt_path_, caffemodel_path_);

    cv::Mat black_frame = cv::Mat::zeros(480, 640, CV_8UC3);
    std::vector<FaceRect> faces = detector.detect(black_frame, 0.5f);

    // Black frame should have no high-confidence detections
    // (model may still produce low-confidence noise)
    for (const auto& face : faces) {
        // If anything is detected, it should be low confidence
        EXPECT_LT(face.confidence, 0.8f);
    }
}

// Test: Performance requirement (<100ms per frame on CPU)
TEST_F(FaceDetectorTest, PerformanceRequirement) {
    FaceDetector detector;
    detector.init(prototxt_path_, caffemodel_path_);

    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);

    // Warm up
    detector.detect(frame);

    // Run multiple detections and check average time
    double total_time = 0;
    const int num_runs = 5;

    for (int i = 0; i < num_runs; i++) {
        detector.detect(frame);
        total_time += detector.getLastDetectionTimeMs();
    }

    double avg_time = total_time / num_runs;
    std::cout << "Average detection time: " << avg_time << "ms" << std::endl;

    // AC2: <100ms per frame on CPU
    EXPECT_LT(avg_time, 100.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
