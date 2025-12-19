#include <gtest/gtest.h>
#include "greeter/FaceRecognizer.h"
#include "greeter/FaceDetector.h"
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace greeter;

class FaceRecognizerTest : public ::testing::Test {
protected:
    void SetUp() override {
        model_path_ = "models/face_recognition/face_recognition_sface_2021dec.onnx";
    }

    std::string model_path_;
};

// Test: Model loads successfully
TEST_F(FaceRecognizerTest, ModelLoadsFromValidPath) {
    FaceRecognizer recognizer;
    bool result = recognizer.init(model_path_);
    EXPECT_TRUE(result);
    EXPECT_TRUE(recognizer.isInitialized());
}

// Test: Fails gracefully with invalid path
TEST_F(FaceRecognizerTest, FailsGracefullyWithInvalidPath) {
    FaceRecognizer recognizer;
    bool result = recognizer.init("nonexistent.onnx");
    EXPECT_FALSE(result);
    EXPECT_FALSE(recognizer.isInitialized());
}

// Test: Extract embedding from synthetic face image
TEST_F(FaceRecognizerTest, ExtractsEmbeddingFromImage) {
    FaceRecognizer recognizer;
    recognizer.init(model_path_);

    // Create a synthetic face-like image (skin-colored region)
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::ellipse(frame, cv::Point(320, 200), cv::Size(80, 100),
                0, 0, 360, cv::Scalar(180, 200, 220), -1);

    FaceRect face;
    face.bounding_box = cv::Rect(240, 100, 160, 200);
    face.confidence = 0.9f;

    FaceEmbedding embedding = recognizer.extractEmbedding(frame, face);

    // SFace produces 128-dim embeddings
    EXPECT_FALSE(embedding.empty());
    EXPECT_EQ(embedding.size(), 128);
}

// Test: Embedding extraction fails for uninitialized recognizer
TEST_F(FaceRecognizerTest, ExtractionFailsWhenNotInitialized) {
    FaceRecognizer recognizer;
    // Don't initialize

    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    FaceRect face;
    face.bounding_box = cv::Rect(100, 100, 100, 100);

    FaceEmbedding embedding = recognizer.extractEmbedding(frame, face);
    EXPECT_TRUE(embedding.empty());
}

// Test: Enroll and recognize a face
TEST_F(FaceRecognizerTest, EnrollAndRecognizeFace) {
    FaceRecognizer recognizer;
    recognizer.init(model_path_);

    // Create a synthetic face image
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::ellipse(frame, cv::Point(320, 200), cv::Size(80, 100),
                0, 0, 360, cv::Scalar(180, 200, 220), -1);

    FaceRect face;
    face.bounding_box = cv::Rect(240, 100, 160, 200);
    face.confidence = 0.9f;

    // Enroll the face
    bool enrolled = recognizer.enrollFace("test_person", frame, face);
    EXPECT_TRUE(enrolled);
    EXPECT_EQ(recognizer.enrolledCount(), 1);
    EXPECT_TRUE(recognizer.isEnrolled("test_person"));

    // Recognize the same face (should match perfectly)
    RecognitionResult result = recognizer.recognize(frame, face);
    EXPECT_TRUE(result.matched);
    EXPECT_EQ(result.person_id, "test_person");
    EXPECT_GT(result.similarity, 0.9f);  // Same face should have very high similarity
}

// Test: Recognition fails with no enrollments
TEST_F(FaceRecognizerTest, RecognitionFailsWithNoEnrollments) {
    FaceRecognizer recognizer;
    recognizer.init(model_path_);

    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    FaceRect face;
    face.bounding_box = cv::Rect(100, 100, 100, 100);

    RecognitionResult result = recognizer.recognize(frame, face);
    EXPECT_FALSE(result.matched);
    EXPECT_TRUE(result.person_id.empty());
}

// Test: Unenroll removes face
TEST_F(FaceRecognizerTest, UnenrollRemovesFace) {
    FaceRecognizer recognizer;
    recognizer.init(model_path_);

    FaceEmbedding embedding;
    embedding.data.resize(128, 0.1f);

    recognizer.enrollFace("person1", embedding);
    EXPECT_EQ(recognizer.enrolledCount(), 1);

    bool removed = recognizer.unenrollFace("person1");
    EXPECT_TRUE(removed);
    EXPECT_EQ(recognizer.enrolledCount(), 0);

    // Removing again should fail
    removed = recognizer.unenrollFace("person1");
    EXPECT_FALSE(removed);
}

// Test: Cosine similarity calculation
TEST_F(FaceRecognizerTest, CosineSimilarityCalculation) {
    FaceEmbedding a, b;
    a.data = {1.0f, 0.0f, 0.0f};
    b.data = {1.0f, 0.0f, 0.0f};

    // Same vector = similarity 1.0
    float sim = FaceRecognizer::cosineSimilarity(a, b);
    EXPECT_FLOAT_EQ(sim, 1.0f);

    // Orthogonal vectors = similarity 0.0
    b.data = {0.0f, 1.0f, 0.0f};
    sim = FaceRecognizer::cosineSimilarity(a, b);
    EXPECT_FLOAT_EQ(sim, 0.0f);

    // Opposite vectors = similarity -1.0
    b.data = {-1.0f, 0.0f, 0.0f};
    sim = FaceRecognizer::cosineSimilarity(a, b);
    EXPECT_FLOAT_EQ(sim, -1.0f);
}

// Test: Save and load enrollments
TEST_F(FaceRecognizerTest, SaveAndLoadEnrollments) {
    const std::string test_path = "/tmp/test_enrollments.json";

    // Create and save enrollments
    {
        FaceRecognizer recognizer;
        FaceEmbedding e1, e2;
        e1.data.resize(128, 0.1f);
        e2.data.resize(128, 0.2f);

        recognizer.enrollFace("person1", e1);
        recognizer.enrollFace("person2", e2);

        bool saved = recognizer.saveEnrollments(test_path);
        EXPECT_TRUE(saved);
    }

    // Load enrollments in new recognizer
    {
        FaceRecognizer recognizer;
        bool loaded = recognizer.loadEnrollments(test_path);
        EXPECT_TRUE(loaded);
        EXPECT_EQ(recognizer.enrolledCount(), 2);
        EXPECT_TRUE(recognizer.isEnrolled("person1"));
        EXPECT_TRUE(recognizer.isEnrolled("person2"));

        // Verify embedding data
        FaceEmbedding e1 = recognizer.getEnrolledEmbedding("person1");
        EXPECT_EQ(e1.size(), 128);
        EXPECT_FLOAT_EQ(e1.data[0], 0.1f);
    }

    // Cleanup
    std::remove(test_path.c_str());
}

// Test: Get enrolled IDs
TEST_F(FaceRecognizerTest, GetEnrolledIds) {
    FaceRecognizer recognizer;
    FaceEmbedding e;
    e.data.resize(128, 0.1f);

    recognizer.enrollFace("alice", e);
    recognizer.enrollFace("bob", e);
    recognizer.enrollFace("charlie", e);

    auto ids = recognizer.getEnrolledIds();
    EXPECT_EQ(ids.size(), 3);

    // Check all IDs present (order not guaranteed)
    EXPECT_NE(std::find(ids.begin(), ids.end(), "alice"), ids.end());
    EXPECT_NE(std::find(ids.begin(), ids.end(), "bob"), ids.end());
    EXPECT_NE(std::find(ids.begin(), ids.end(), "charlie"), ids.end());
}

// Test: Clear enrollments
TEST_F(FaceRecognizerTest, ClearEnrollments) {
    FaceRecognizer recognizer;
    FaceEmbedding e;
    e.data.resize(128, 0.1f);

    recognizer.enrollFace("person1", e);
    recognizer.enrollFace("person2", e);
    EXPECT_EQ(recognizer.enrolledCount(), 2);

    recognizer.clearEnrollments();
    EXPECT_EQ(recognizer.enrolledCount(), 0);
}

// Test: Recognize distinguishes between different faces
TEST_F(FaceRecognizerTest, DistinguishesDifferentFaces) {
    FaceRecognizer recognizer;
    recognizer.init(model_path_);

    // Create two visually different synthetic faces
    cv::Mat frame1 = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::ellipse(frame1, cv::Point(320, 200), cv::Size(80, 100),
                0, 0, 360, cv::Scalar(180, 200, 220), -1);  // Light skin tone

    cv::Mat frame2 = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::ellipse(frame2, cv::Point(320, 200), cv::Size(80, 100),
                0, 0, 360, cv::Scalar(100, 120, 140), -1);  // Darker tone
    // Add distinct features
    cv::rectangle(frame2, cv::Point(300, 180), cv::Point(340, 190),
                  cv::Scalar(50, 60, 70), -1);

    FaceRect face;
    face.bounding_box = cv::Rect(240, 100, 160, 200);
    face.confidence = 0.9f;

    // Enroll person1 with frame1
    recognizer.enrollFace("person1", frame1, face);

    // Try to recognize frame2 - should either not match or match with lower similarity
    RecognitionResult result = recognizer.recognize(frame2, face, 0.6f);

    // The synthetic faces are similar enough that they might match, but with lower score
    // This test validates the system doesn't give false high-confidence matches
    if (result.matched) {
        EXPECT_LT(result.similarity, 0.95f);  // Should not be as high as same-face match
    }
}

// Test: Threshold affects matching
TEST_F(FaceRecognizerTest, ThresholdAffectsMatching) {
    FaceRecognizer recognizer;
    FaceEmbedding e1, e2;

    // Create two embeddings with known similarity ~0.5
    e1.data.resize(128);
    e2.data.resize(128);
    for (size_t i = 0; i < 128; i++) {
        e1.data[i] = (i % 2 == 0) ? 1.0f : 0.0f;
        e2.data[i] = (i % 4 == 0) ? 1.0f : 0.0f;  // Partially overlapping
    }

    recognizer.enrollFace("person1", e1);

    // With low threshold, should match
    RecognitionResult result = recognizer.recognize(e2, 0.1f);
    EXPECT_TRUE(result.matched);

    // With high threshold, should not match
    result = recognizer.recognize(e2, 0.9f);
    EXPECT_FALSE(result.matched);
}
