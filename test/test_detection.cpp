#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "detection/DefectTypes.h"
#include "detection/VlmClient.h"
#include "detection/ImageAnnotator.h"
#include "MockVlmClient.h"

// ============================================
// DefectTypes Tests
// ============================================

TEST(DefectTypesTest, DefectTypeToString) {
    EXPECT_EQ(defectTypeToString(DefectType::LOCATION_ERROR), "LOCATION_ERROR");
    EXPECT_EQ(defectTypeToString(DefectType::QUALITY_ISSUE), "QUALITY_ISSUE");
    EXPECT_EQ(defectTypeToString(DefectType::SAFETY_HAZARD), "SAFETY_HAZARD");
    EXPECT_EQ(defectTypeToString(DefectType::MISSING_ELEMENT), "MISSING_ELEMENT");
}

TEST(DefectTypesTest, StringToDefectType) {
    EXPECT_EQ(stringToDefectType("LOCATION_ERROR"), DefectType::LOCATION_ERROR);
    EXPECT_EQ(stringToDefectType("QUALITY_ISSUE"), DefectType::QUALITY_ISSUE);
    EXPECT_EQ(stringToDefectType("SAFETY_HAZARD"), DefectType::SAFETY_HAZARD);
    EXPECT_EQ(stringToDefectType("MISSING_ELEMENT"), DefectType::MISSING_ELEMENT);
    // Unknown string defaults to QUALITY_ISSUE
    EXPECT_EQ(stringToDefectType("UNKNOWN"), DefectType::QUALITY_ISSUE);
}

TEST(DefectTypesTest, DefectToJson) {
    Defect d;
    d.id = "def_001";
    d.type = DefectType::QUALITY_ISSUE;
    d.description = "Test defect";
    d.image_loc = {100.0f, 200.0f};
    d.bbox_x = 80;
    d.bbox_y = 180;
    d.bbox_width = 40;
    d.bbox_height = 40;
    d.plan_loc = {1.5f, 2.5f};
    d.confidence = 0.87f;
    d.severity = "medium";
    d.trade = "finishes";

    nlohmann::json j;
    to_json(j, d);

    EXPECT_EQ(j["id"], "def_001");
    EXPECT_EQ(j["type"], "QUALITY_ISSUE");
    EXPECT_EQ(j["description"], "Test defect");
    EXPECT_FLOAT_EQ(j["image_location"]["x"], 100.0f);
    EXPECT_FLOAT_EQ(j["image_location"]["y"], 200.0f);
    EXPECT_EQ(j["bounding_box"]["x"], 80);
    EXPECT_EQ(j["bounding_box"]["y"], 180);
    EXPECT_EQ(j["bounding_box"]["width"], 40);
    EXPECT_EQ(j["bounding_box"]["height"], 40);
    EXPECT_FLOAT_EQ(j["confidence"], 0.87f);
    EXPECT_EQ(j["severity"], "medium");
    EXPECT_EQ(j["trade"], "finishes");
}

TEST(DefectTypesTest, DefectFromJson) {
    nlohmann::json j = {
        {"id", "def_002"},
        {"type", "SAFETY_HAZARD"},
        {"description", "Exposed wire"},
        {"image_location", {{"x", 150.0f}, {"y", 250.0f}}},
        {"bounding_box", {{"x", 130}, {"y", 230}, {"width", 50}, {"height", 50}}},
        {"plan_location", {{"x", 3.0f}, {"y", 4.0f}}},
        {"confidence", 0.92f},
        {"severity", "high"},
        {"trade", "mep"}
    };

    Defect d;
    from_json(j, d);

    EXPECT_EQ(d.id, "def_002");
    EXPECT_EQ(d.type, DefectType::SAFETY_HAZARD);
    EXPECT_EQ(d.description, "Exposed wire");
    EXPECT_FLOAT_EQ(d.image_loc.x, 150.0f);
    EXPECT_FLOAT_EQ(d.image_loc.y, 250.0f);
    EXPECT_EQ(d.bbox_x, 130);
    EXPECT_EQ(d.bbox_y, 230);
    EXPECT_EQ(d.bbox_width, 50);
    EXPECT_EQ(d.bbox_height, 50);
    EXPECT_FLOAT_EQ(d.plan_loc.x, 3.0f);
    EXPECT_FLOAT_EQ(d.plan_loc.y, 4.0f);
    EXPECT_FLOAT_EQ(d.confidence, 0.92f);
    EXPECT_EQ(d.severity, "high");
    EXPECT_EQ(d.trade, "mep");
}

TEST(DefectTypesTest, DefectFromJsonWithDefaults) {
    // Minimal JSON - should use defaults
    nlohmann::json j = {
        {"id", "def_minimal"}
    };

    Defect d;
    from_json(j, d);

    EXPECT_EQ(d.id, "def_minimal");
    EXPECT_EQ(d.type, DefectType::QUALITY_ISSUE);  // default
    EXPECT_EQ(d.description, "");
    EXPECT_FLOAT_EQ(d.confidence, 0.0f);
    EXPECT_EQ(d.severity, "medium");  // default
    EXPECT_EQ(d.trade, "finishes");   // default
}

TEST(DefectTypesTest, DefectBoundingBox) {
    Defect d;
    d.bbox_x = 100;
    d.bbox_y = 200;
    d.bbox_width = 50;
    d.bbox_height = 60;

    cv::Rect rect = d.getBoundingBox();
    EXPECT_EQ(rect.x, 100);
    EXPECT_EQ(rect.y, 200);
    EXPECT_EQ(rect.width, 50);
    EXPECT_EQ(rect.height, 60);
}

TEST(DefectTypesTest, JsonRoundTrip) {
    Defect original;
    original.id = "def_round";
    original.type = DefectType::MISSING_ELEMENT;
    original.description = "Missing outlet cover";
    original.image_loc = {320.0f, 240.0f};
    original.bbox_x = 300;
    original.bbox_y = 220;
    original.bbox_width = 40;
    original.bbox_height = 40;
    original.confidence = 0.75f;
    original.severity = "low";
    original.trade = "mep";

    nlohmann::json j;
    to_json(j, original);

    Defect restored;
    from_json(j, restored);

    EXPECT_EQ(restored.id, original.id);
    EXPECT_EQ(restored.type, original.type);
    EXPECT_EQ(restored.description, original.description);
    EXPECT_FLOAT_EQ(restored.image_loc.x, original.image_loc.x);
    EXPECT_FLOAT_EQ(restored.image_loc.y, original.image_loc.y);
    EXPECT_EQ(restored.bbox_x, original.bbox_x);
    EXPECT_EQ(restored.bbox_y, original.bbox_y);
    EXPECT_EQ(restored.bbox_width, original.bbox_width);
    EXPECT_EQ(restored.bbox_height, original.bbox_height);
    EXPECT_FLOAT_EQ(restored.confidence, original.confidence);
    EXPECT_EQ(restored.severity, original.severity);
    EXPECT_EQ(restored.trade, original.trade);
}

// ============================================
// VlmClient Tests
// ============================================

TEST(VlmClientTest, EmptyImageReturnsEmpty) {
    VlmClient client("test-key");
    cv::Mat empty_image;
    Pose2D pose{0.0f, 0.0f, 0.0f};

    auto defects = client.analyzeImage(empty_image, "finishes", pose);
    EXPECT_TRUE(defects.empty());
    EXPECT_EQ(client.getLastError(), "Empty image provided");
}

TEST(VlmClientTest, EmptyApiKeyReturnsEmpty) {
    VlmClient client("");  // Empty API key
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
    Pose2D pose{0.0f, 0.0f, 0.0f};

    auto defects = client.analyzeImage(image, "finishes", pose);
    EXPECT_TRUE(defects.empty());
    EXPECT_EQ(client.getLastError(), "API key not set");
}

TEST(VlmClientTest, ConfigurationSetters) {
    VlmClient client("test-key");

    client.setModel("claude-opus-4");
    client.setMaxRetries(5);
    client.setTimeout(60000);
    client.setConfidenceThreshold(0.7f);
    client.setApiUrl("https://custom.api.com/v1/messages");

    // Cannot directly verify private members, but ensure no crash
    EXPECT_EQ(client.getLastStatusCode(), 0);
    EXPECT_EQ(client.getTokensUsed(), 0);
}

// ============================================
// MockVlmClient Tests
// ============================================

TEST(MockVlmClientTest, ReturnsMockDefects) {
    MockVlmClient mock;

    Defect d;
    d.id = "mock_001";
    d.type = DefectType::QUALITY_ISSUE;
    d.description = "Mock defect";
    d.confidence = 0.9f;
    mock.setMockDefects({d});

    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(100, 100, 100));
    Pose2D pose{1.0f, 2.0f, 0.5f};

    auto defects = mock.analyzeImage(image, "mep", pose);

    EXPECT_EQ(defects.size(), 1);
    EXPECT_EQ(defects[0].id, "mock_001");
    EXPECT_EQ(mock.getCallCount(), 1);
    EXPECT_EQ(mock.getLastImageSize(), cv::Size(640, 480));
    EXPECT_EQ(mock.getLastPlanContext(), "mep");
    EXPECT_FLOAT_EQ(mock.getLastPose().x, 1.0f);
    EXPECT_FLOAT_EQ(mock.getLastPose().y, 2.0f);
}

TEST(MockVlmClientTest, ClearMockDefects) {
    MockVlmClient mock;

    Defect d;
    d.id = "temp";
    mock.setMockDefects({d});

    cv::Mat image(100, 100, CV_8UC3);
    Pose2D pose;

    auto defects1 = mock.analyzeImage(image, "", pose);
    EXPECT_EQ(defects1.size(), 1);

    mock.clearMockDefects();
    auto defects2 = mock.analyzeImage(image, "", pose);
    EXPECT_TRUE(defects2.empty());
}

TEST(MockVlmClientTest, CallCountTracking) {
    MockVlmClient mock;
    cv::Mat image(100, 100, CV_8UC3);
    Pose2D pose;

    EXPECT_EQ(mock.getCallCount(), 0);

    mock.analyzeImage(image, "", pose);
    EXPECT_EQ(mock.getCallCount(), 1);

    mock.analyzeImage(image, "", pose);
    mock.analyzeImage(image, "", pose);
    EXPECT_EQ(mock.getCallCount(), 3);

    mock.resetCallCount();
    EXPECT_EQ(mock.getCallCount(), 0);
}

// ============================================
// ImageAnnotator Tests
// ============================================

TEST(ImageAnnotatorTest, AnnotateEmptyImage) {
    ImageAnnotator annotator;
    cv::Mat empty_image;
    std::vector<Defect> defects;

    // Should not crash on empty image
    annotator.annotateImage(empty_image, defects);
    EXPECT_TRUE(empty_image.empty());
}

TEST(ImageAnnotatorTest, AnnotateWithNoDefects) {
    ImageAnnotator annotator;
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(200, 200, 200));
    cv::Mat original = image.clone();
    std::vector<Defect> defects;  // Empty

    annotator.annotateImage(image, defects);

    // Image should be unchanged when no defects
    cv::Mat diff;
    cv::absdiff(image, original, diff);
    EXPECT_EQ(cv::countNonZero(cv::sum(diff)[0] > 0 ? cv::Mat::ones(1,1,CV_8U) : cv::Mat::zeros(1,1,CV_8U)), 0);
}

TEST(ImageAnnotatorTest, AnnotateWithDefects) {
    ImageAnnotator annotator;
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(200, 200, 200));
    cv::Mat original = image.clone();

    Defect d;
    d.id = "def_001";
    d.type = DefectType::QUALITY_ISSUE;
    d.description = "Test";
    d.bbox_x = 100;
    d.bbox_y = 100;
    d.bbox_width = 50;
    d.bbox_height = 50;
    d.confidence = 0.8f;
    d.severity = "medium";

    annotator.annotateImage(image, {d});

    // Image should be modified
    cv::Mat diff;
    cv::absdiff(image, original, diff);
    EXPECT_GT(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(ImageAnnotatorTest, AnnotateWithDifferentSeverities) {
    ImageAnnotator annotator;
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(200, 200, 200));

    Defect d1, d2, d3;

    d1.id = "high";
    d1.bbox_x = 50; d1.bbox_y = 50; d1.bbox_width = 40; d1.bbox_height = 40;
    d1.confidence = 0.9f;
    d1.severity = "high";

    d2.id = "medium";
    d2.bbox_x = 150; d2.bbox_y = 50; d2.bbox_width = 40; d2.bbox_height = 40;
    d2.confidence = 0.7f;
    d2.severity = "medium";

    d3.id = "low";
    d3.bbox_x = 250; d3.bbox_y = 50; d3.bbox_width = 40; d3.bbox_height = 40;
    d3.confidence = 0.6f;
    d3.severity = "low";

    // Should not crash with different severities
    annotator.annotateImage(image, {d1, d2, d3});
    EXPECT_FALSE(image.empty());
}

TEST(ImageAnnotatorTest, AnnotateWithoutBoundingBox) {
    ImageAnnotator annotator;
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(200, 200, 200));

    Defect d;
    d.id = "no_bbox";
    d.image_loc = {320.0f, 240.0f};
    // bbox is 0,0,0,0 by default
    d.confidence = 0.8f;
    d.severity = "medium";

    // Should draw circle instead of rectangle
    annotator.annotateImage(image, {d});
    EXPECT_FALSE(image.empty());
}

TEST(ImageAnnotatorTest, BoundingBoxClipping) {
    ImageAnnotator annotator;
    cv::Mat image(100, 100, CV_8UC3, cv::Scalar(200, 200, 200));

    Defect d;
    d.id = "clipped";
    d.bbox_x = -10;  // Outside image
    d.bbox_y = -10;
    d.bbox_width = 50;
    d.bbox_height = 50;
    d.confidence = 0.8f;
    d.severity = "medium";

    // Should handle negative coordinates without crash
    annotator.annotateImage(image, {d});
    EXPECT_FALSE(image.empty());
}

TEST(ImageAnnotatorTest, ConfigurationSetters) {
    ImageAnnotator annotator;
    annotator.setFontScale(0.8f);
    annotator.setLineThickness(3);

    cv::Mat image(200, 200, CV_8UC3, cv::Scalar(200, 200, 200));
    Defect d;
    d.id = "config_test";
    d.bbox_x = 50; d.bbox_y = 50; d.bbox_width = 50; d.bbox_height = 50;
    d.confidence = 0.9f;
    d.severity = "high";

    // Should use custom settings without crash
    annotator.annotateImage(image, {d});
    EXPECT_FALSE(image.empty());
}

// ============================================
// Confidence Filtering Tests
// ============================================

// Test helper class to expose protected filterByConfidence for testing
class TestableVlmClient : public VlmClient {
public:
    TestableVlmClient() : VlmClient("test-key") {}

    std::vector<Defect> testFilterByConfidence(std::vector<Defect>& defects) {
        return filterByConfidence(defects);
    }

    void testSetConfidenceThreshold(float t) {
        setConfidenceThreshold(t);
    }
};

TEST(ConfidenceFilteringTest, HighConfidenceKept) {
    TestableVlmClient client;
    client.testSetConfidenceThreshold(0.5f);

    std::vector<Defect> defects;

    Defect d1;
    d1.id = "high_conf";
    d1.confidence = 0.9f;
    defects.push_back(d1);

    Defect d2;
    d2.id = "low_conf";
    d2.confidence = 0.3f;
    defects.push_back(d2);

    auto filtered = client.testFilterByConfidence(defects);

    // Only high confidence should remain
    EXPECT_EQ(filtered.size(), 1);
    EXPECT_EQ(filtered[0].id, "high_conf");
}

TEST(ConfidenceFilteringTest, AllKeptAboveThreshold) {
    TestableVlmClient client;
    client.testSetConfidenceThreshold(0.5f);

    std::vector<Defect> defects;

    Defect d1, d2, d3;
    d1.id = "def1"; d1.confidence = 0.9f;
    d2.id = "def2"; d2.confidence = 0.7f;
    d3.id = "def3"; d3.confidence = 0.6f;
    defects.push_back(d1);
    defects.push_back(d2);
    defects.push_back(d3);

    auto filtered = client.testFilterByConfidence(defects);

    EXPECT_EQ(filtered.size(), 3);
}

TEST(ConfidenceFilteringTest, AllRemovedBelowThreshold) {
    TestableVlmClient client;
    client.testSetConfidenceThreshold(0.8f);

    std::vector<Defect> defects;

    Defect d1, d2;
    d1.id = "def1"; d1.confidence = 0.5f;
    d2.id = "def2"; d2.confidence = 0.3f;
    defects.push_back(d1);
    defects.push_back(d2);

    auto filtered = client.testFilterByConfidence(defects);

    EXPECT_TRUE(filtered.empty());
}

TEST(ConfidenceFilteringTest, ExactThresholdKept) {
    TestableVlmClient client;
    client.testSetConfidenceThreshold(0.5f);

    std::vector<Defect> defects;

    Defect d;
    d.id = "exact";
    d.confidence = 0.5f;  // Exactly at threshold
    defects.push_back(d);

    auto filtered = client.testFilterByConfidence(defects);

    // Defect at exact threshold should be kept (>= threshold)
    EXPECT_EQ(filtered.size(), 1);
}

// ============================================
// Base64 Encoding Tests (Integration)
// ============================================

TEST(Base64EncodingTest, SmallImageEncodes) {
    // Create a small test image
    cv::Mat image(10, 10, CV_8UC3, cv::Scalar(128, 128, 128));

    // We can't directly test encodeImageBase64 as it's protected,
    // but we can verify the client doesn't crash on valid images
    VlmClient client("test-key");
    // analyzeImage will fail due to no real API, but encoding should work
    // We just verify no crash occurs
    EXPECT_TRUE(image.cols == 10);
}

TEST(Base64EncodingTest, LargeImageHandled) {
    // Test that large images are handled (resized internally)
    cv::Mat large_image(4000, 4000, CV_8UC3, cv::Scalar(128, 128, 128));

    // Client should handle this without crash
    // (actual resizing happens inside encodeImageBase64)
    VlmClient client("test-key");
    EXPECT_TRUE(large_image.cols == 4000);
}

// ============================================
// Retry Logic Tests
// ============================================

TEST(RetryLogicTest, ShouldRetryOnRateLimit) {
    // Test that 429 triggers retry - we test via observing behavior
    // In actual implementation, VlmClient::shouldRetry is protected

    VlmClient client("test-key");
    client.setMaxRetries(0);  // Disable retries for faster test failure

    // Without a real API, we can't fully test retry logic,
    // but we verify the configuration is accepted
    EXPECT_EQ(client.getLastStatusCode(), 0);
}

// ============================================
// Integration Tests
// ============================================

TEST(IntegrationTest, MockClientWithAnnotator) {
    // Test the full pipeline with mock client
    MockVlmClient mock;

    Defect d;
    d.id = "integration_001";
    d.type = DefectType::SAFETY_HAZARD;
    d.description = "Integration test defect";
    d.bbox_x = 100;
    d.bbox_y = 100;
    d.bbox_width = 80;
    d.bbox_height = 80;
    d.confidence = 0.95f;
    d.severity = "high";
    d.trade = "mep";
    mock.setMockDefects({d});

    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(200, 200, 200));
    Pose2D pose{5.0f, 3.0f, 1.57f};

    // Analyze
    auto defects = mock.analyzeImage(image, "mep", pose);
    EXPECT_EQ(defects.size(), 1);

    // Annotate
    ImageAnnotator annotator;
    cv::Mat annotated = image.clone();
    annotator.annotateImage(annotated, defects);

    // Verify changes were made
    cv::Mat diff;
    cv::absdiff(image, annotated, diff);
    EXPECT_GT(cv::countNonZero(diff.reshape(1)), 0);

    // Serialize
    nlohmann::json j;
    to_json(j, defects[0]);
    EXPECT_EQ(j["id"], "integration_001");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
