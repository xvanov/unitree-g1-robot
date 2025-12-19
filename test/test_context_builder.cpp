#include <gtest/gtest.h>
#include "greeter/ContextBuilder.h"
#include "greeter/PersonnelDatabase.h"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

using namespace greeter;

class ContextBuilderTest : public ::testing::Test {
protected:
    void SetUp() override {
        builder_ = std::make_unique<ContextBuilder>();

        // Load personnel database for identity tests
        personnel_db_.loadFromFile("data/personnel/gauntlet_personnel.json");
        builder_->setPersonnelDatabase(&personnel_db_);
    }

    std::unique_ptr<ContextBuilder> builder_;
    PersonnelDatabase personnel_db_;
};

// Test: Build context JSON with all fields
TEST_F(ContextBuilderTest, BuildContextJsonWithAllFields) {
    // Set up frame
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    // Set up faces
    std::vector<FaceRect> faces;
    FaceRect face;
    face.bounding_box = cv::Rect(200, 100, 100, 120);
    face.confidence = 0.95f;
    faces.push_back(face);
    builder_->setDetectedFaces(faces);

    // Set up environment
    EnvironmentContext env;
    env.near_staircase = true;
    env.witnesses_present = false;
    env.camera_coverage = "partial";
    env.active_observations.push_back("Dim lighting");
    builder_->setEnvironment(env);

    // Set up robot state
    RobotStateContext robot;
    robot.current_pose = "walking";
    robot.battery_percent = 87.5f;
    robot.current_action = "APPROACH";
    robot.recent_actions = {"WAVE_HAND", "FOLLOW"};
    builder_->setRobotState(robot);

    // Add overheard conversation
    builder_->addOverheardConversation("Hello there!");

    // Build JSON
    nlohmann::json context = builder_->buildContextJson();

    // Verify all top-level fields exist
    EXPECT_TRUE(context.contains("camera_frame"));
    EXPECT_TRUE(context.contains("detected_faces"));
    EXPECT_TRUE(context.contains("environment"));
    EXPECT_TRUE(context.contains("robot_state"));
    EXPECT_TRUE(context.contains("overheard_conversations"));

    // Verify environment fields
    EXPECT_TRUE(context["environment"]["near_staircase"].get<bool>());
    EXPECT_FALSE(context["environment"]["witnesses_present"].get<bool>());
    EXPECT_EQ(context["environment"]["camera_coverage"].get<std::string>(), "partial");

    // Verify robot state
    EXPECT_EQ(context["robot_state"]["current_pose"].get<std::string>(), "walking");
    EXPECT_FLOAT_EQ(context["robot_state"]["battery_percent"].get<float>(), 87.5f);

    // Verify conversations
    EXPECT_EQ(context["overheard_conversations"].size(), 1);
    EXPECT_EQ(context["overheard_conversations"][0].get<std::string>(), "Hello there!");
}

// Test: Posture: face at bottom -> bent_forward
TEST_F(ContextBuilderTest, PostureBentForwardWhenFaceAtBottom) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    // Face at bottom (bottom at 360/480 = 75% > 70%)
    std::vector<FaceRect> faces;
    FaceRect face;
    face.bounding_box = cv::Rect(200, 260, 100, 100);  // bottom = 260 + 100 = 360
    face.confidence = 0.9f;
    faces.push_back(face);
    builder_->setDetectedFaces(faces);

    auto identified = builder_->getIdentifiedFaces();
    ASSERT_EQ(identified.size(), 1);
    EXPECT_EQ(identified[0].posture, "bent_forward");
}

// Test: Posture: face in middle -> standing
TEST_F(ContextBuilderTest, PostureStandingWhenFaceInMiddle) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    // Face in middle (bottom at 240/480 = 50% < 70%)
    std::vector<FaceRect> faces;
    FaceRect face;
    face.bounding_box = cv::Rect(200, 140, 100, 100);  // bottom = 240
    face.confidence = 0.9f;
    faces.push_back(face);
    builder_->setDetectedFaces(faces);

    auto identified = builder_->getIdentifiedFaces();
    ASSERT_EQ(identified.size(), 1);
    EXPECT_EQ(identified[0].posture, "standing");
}

// Test: Attention: centered face -> looking_at_robot
TEST_F(ContextBuilderTest, AttentionLookingAtRobotWhenCentered) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    // Face centered (center at 320/640 = 50%, which is in 35-65% range)
    // Aspect ratio 90/100 = 0.9 (in 0.7-1.3 range for frontal)
    std::vector<FaceRect> faces;
    FaceRect face;
    face.bounding_box = cv::Rect(275, 100, 90, 100);  // centerX = 275 + 45 = 320
    face.confidence = 0.9f;
    faces.push_back(face);
    builder_->setDetectedFaces(faces);

    auto identified = builder_->getIdentifiedFaces();
    ASSERT_EQ(identified.size(), 1);
    EXPECT_EQ(identified[0].attention, "looking_at_robot");
}

// Test: Attention: off-center -> looking_away
TEST_F(ContextBuilderTest, AttentionLookingAwayWhenOffCenter) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    // Face off-center (center at 100/640 = 15.6%, which is < 35%)
    std::vector<FaceRect> faces;
    FaceRect face;
    face.bounding_box = cv::Rect(50, 100, 100, 100);  // centerX = 50 + 50 = 100
    face.confidence = 0.9f;
    faces.push_back(face);
    builder_->setDetectedFaces(faces);

    auto identified = builder_->getIdentifiedFaces();
    ASSERT_EQ(identified.size(), 1);
    EXPECT_EQ(identified[0].attention, "looking_away");
}

// Test: Attention: profile face (narrow aspect) -> looking_away
TEST_F(ContextBuilderTest, AttentionLookingAwayWhenProfileFace) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    // Face centered but profile (aspect ratio 50/100 = 0.5 < 0.7)
    std::vector<FaceRect> faces;
    FaceRect face;
    face.bounding_box = cv::Rect(295, 100, 50, 100);  // centerX = 320, aspect = 0.5
    face.confidence = 0.9f;
    faces.push_back(face);
    builder_->setDetectedFaces(faces);

    auto identified = builder_->getIdentifiedFaces();
    ASSERT_EQ(identified.size(), 1);
    EXPECT_EQ(identified[0].attention, "looking_away");
}

// Test: Base64 produces valid string
TEST_F(ContextBuilderTest, Base64ProducesValidString) {
    // Create a simple colored frame
    cv::Mat frame = cv::Mat::zeros(100, 100, CV_8UC3);
    cv::rectangle(frame, cv::Rect(10, 10, 80, 80), cv::Scalar(0, 255, 0), -1);
    builder_->setFrame(frame);

    nlohmann::json context = builder_->buildContextJson();

    std::string base64 = context["camera_frame"].get<std::string>();
    EXPECT_FALSE(base64.empty());

    // Check valid base64 characters
    for (char c : base64) {
        bool valid = (c >= 'A' && c <= 'Z') ||
                     (c >= 'a' && c <= 'z') ||
                     (c >= '0' && c <= '9') ||
                     c == '+' || c == '/' || c == '=';
        EXPECT_TRUE(valid) << "Invalid base64 character: " << c;
    }
}

// Test: Multiple faces handled
TEST_F(ContextBuilderTest, MultipleFacesHandled) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    std::vector<FaceRect> faces;

    // Face 1 - centered, standing
    FaceRect face1;
    face1.bounding_box = cv::Rect(270, 100, 100, 100);
    face1.confidence = 0.95f;
    faces.push_back(face1);

    // Face 2 - off-center, bent forward
    FaceRect face2;
    face2.bounding_box = cv::Rect(50, 280, 100, 100);
    face2.confidence = 0.88f;
    faces.push_back(face2);

    // Face 3 - profile
    FaceRect face3;
    face3.bounding_box = cv::Rect(400, 150, 50, 100);
    face3.confidence = 0.75f;
    faces.push_back(face3);

    builder_->setDetectedFaces(faces);

    auto identified = builder_->getIdentifiedFaces();
    EXPECT_EQ(identified.size(), 3);

    nlohmann::json context = builder_->buildContextJson();
    EXPECT_EQ(context["detected_faces"].size(), 3);
}

// Test: Empty frame handled
TEST_F(ContextBuilderTest, EmptyFrameHandled) {
    cv::Mat empty_frame;
    builder_->setFrame(empty_frame);

    nlohmann::json context = builder_->buildContextJson();
    EXPECT_TRUE(context["camera_frame"].get<std::string>().empty());
}

// Test: Overheard conversations in context
TEST_F(ContextBuilderTest, OverheardConversationsInContext) {
    cv::Mat frame = cv::Mat::zeros(100, 100, CV_8UC3);
    builder_->setFrame(frame);

    builder_->addOverheardConversation("First conversation");
    builder_->addOverheardConversation("Second conversation");
    builder_->addOverheardConversation("Third conversation");

    nlohmann::json context = builder_->buildContextJson();
    EXPECT_EQ(context["overheard_conversations"].size(), 3);
    EXPECT_EQ(context["overheard_conversations"][0].get<std::string>(), "First conversation");
    EXPECT_EQ(context["overheard_conversations"][2].get<std::string>(), "Third conversation");

    // Test clear
    builder_->clearOverheardConversations();
    context = builder_->buildContextJson();
    EXPECT_EQ(context["overheard_conversations"].size(), 0);
}

// Test: Distance estimation based on face size
TEST_F(ContextBuilderTest, DistanceEstimation) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    // Large face (25%+ of frame height) -> ~1m
    std::vector<FaceRect> faces;
    FaceRect large_face;
    large_face.bounding_box = cv::Rect(200, 100, 100, 130);  // 130/480 = 27%
    large_face.confidence = 0.9f;
    faces.push_back(large_face);
    builder_->setDetectedFaces(faces);

    auto identified = builder_->getIdentifiedFaces();
    EXPECT_FLOAT_EQ(identified[0].estimated_distance, 1.0f);

    // Medium face (15-25%) -> ~2m
    faces.clear();
    FaceRect medium_face;
    medium_face.bounding_box = cv::Rect(200, 100, 100, 90);  // 90/480 = 18.75%
    medium_face.confidence = 0.9f;
    faces.push_back(medium_face);
    builder_->setDetectedFaces(faces);

    identified = builder_->getIdentifiedFaces();
    EXPECT_FLOAT_EQ(identified[0].estimated_distance, 2.0f);

    // Small face (<5%) -> ~5m
    faces.clear();
    FaceRect small_face;
    small_face.bounding_box = cv::Rect(200, 100, 20, 20);  // 20/480 = 4.2%
    small_face.confidence = 0.9f;
    faces.push_back(small_face);
    builder_->setDetectedFaces(faces);

    identified = builder_->getIdentifiedFaces();
    EXPECT_FLOAT_EQ(identified[0].estimated_distance, 5.0f);
}

// Test: Personnel lookup uses id field correctly (AC3)
TEST_F(ContextBuilderTest, PersonnelLookupUsesIdField) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    // Add a face
    std::vector<FaceRect> faces;
    FaceRect face;
    face.bounding_box = cv::Rect(200, 100, 100, 100);
    face.confidence = 0.9f;
    faces.push_back(face);
    builder_->setDetectedFaces(faces);

    // Set identity to a known person from the database
    // The personnel database is loaded in SetUp() from gauntlet_personnel.json
    builder_->setFaceIdentity(0, "adam_isom");

    // Get identified faces and verify the PersonnelRecord was looked up
    auto identified = builder_->getIdentifiedFaces();
    ASSERT_EQ(identified.size(), 1);
    ASSERT_TRUE(identified[0].identity.has_value());

    // Verify the correct PersonnelRecord was returned
    const auto& person = identified[0].identity.value();
    EXPECT_EQ(person.id, "adam_isom");
    EXPECT_EQ(person.name, "Adam Isom");
    EXPECT_FALSE(person.role.empty());

    // Also verify it appears correctly in the JSON output
    nlohmann::json context = builder_->buildContextJson();
    ASSERT_EQ(context["detected_faces"].size(), 1);
    EXPECT_EQ(context["detected_faces"][0]["id"].get<std::string>(), "adam_isom");
    EXPECT_EQ(context["detected_faces"][0]["name"].get<std::string>(), "Adam Isom");
}

// Test: Unknown identity produces correct JSON
TEST_F(ContextBuilderTest, UnknownIdentityJson) {
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    builder_->setFrame(frame);

    std::vector<FaceRect> faces;
    FaceRect face;
    face.bounding_box = cv::Rect(200, 100, 100, 100);
    face.confidence = 0.9f;
    faces.push_back(face);
    builder_->setDetectedFaces(faces);

    nlohmann::json context = builder_->buildContextJson();
    auto& face_json = context["detected_faces"][0];

    EXPECT_TRUE(face_json["id"].is_null());
    EXPECT_EQ(face_json["name"].get<std::string>(), "Unknown");
    EXPECT_TRUE(face_json["role"].is_null());
    EXPECT_TRUE(face_json["context_notes"].is_array());
    EXPECT_EQ(face_json["context_notes"].size(), 0);
}

// Test: Robot state defaults
TEST_F(ContextBuilderTest, RobotStateDefaults) {
    cv::Mat frame = cv::Mat::zeros(100, 100, CV_8UC3);
    builder_->setFrame(frame);

    // Don't set robot state, use defaults
    nlohmann::json context = builder_->buildContextJson();

    EXPECT_EQ(context["robot_state"]["current_pose"].get<std::string>(), "standing");
    EXPECT_FLOAT_EQ(context["robot_state"]["battery_percent"].get<float>(), 100.0f);
    EXPECT_EQ(context["robot_state"]["current_action"].get<std::string>(), "WAITING");
    EXPECT_EQ(context["robot_state"]["recent_actions"].size(), 0);
}

// Test: Environment defaults
TEST_F(ContextBuilderTest, EnvironmentDefaults) {
    cv::Mat frame = cv::Mat::zeros(100, 100, CV_8UC3);
    builder_->setFrame(frame);

    // Don't set environment, use defaults
    nlohmann::json context = builder_->buildContextJson();

    EXPECT_FALSE(context["environment"]["near_staircase"].get<bool>());
    EXPECT_FALSE(context["environment"]["witnesses_present"].get<bool>());
    EXPECT_EQ(context["environment"]["camera_coverage"].get<std::string>(), "full");
    EXPECT_EQ(context["environment"]["observations"].size(), 0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
