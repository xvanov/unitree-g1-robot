#include <gtest/gtest.h>
#include "greeter/SessionRecorder.h"
#include "greeter/ActionParser.h"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

using namespace greeter;

class SessionRecorderTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_dir_ = "test_recordings_temp";
        std::filesystem::create_directories(test_dir_);
    }

    void TearDown() override {
        std::filesystem::remove_all(test_dir_);
    }

    cv::Mat createTestFrame() {
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(100, 100, 100));
        cv::putText(frame, "Test Frame", cv::Point(200, 240),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        return frame;
    }

    ParsedAction createTestAction(ActionType type = ActionType::WAVE_HAND) {
        ParsedAction action;
        action.type = type;
        action.reasoning = "Test reasoning for action.";
        action.intent = "Test intent.";
        action.confidence = 0.95f;
        return action;
    }

    nlohmann::json createTestContext() {
        return {
            {"detected_faces", 1},
            {"environment", {{"near_staircase", false}}},
            {"robot_state", {{"current_action", "WAITING"}}}
        };
    }

    std::string test_dir_;
};

// ============================================================================
// Session Lifecycle Tests
// ============================================================================

TEST_F(SessionRecorderTest, StartSession_CreatesDirectory) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session_001"));
    EXPECT_TRUE(recorder.isRecording());

    std::string session_path = recorder.getSessionPath();
    EXPECT_TRUE(std::filesystem::exists(session_path));
    EXPECT_TRUE(std::filesystem::exists(session_path + "/frames"));
}

TEST_F(SessionRecorderTest, StartSession_AutoGeneratesId) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_));
    EXPECT_TRUE(recorder.isRecording());

    std::string session_id = recorder.getSessionId();
    EXPECT_FALSE(session_id.empty());
    EXPECT_TRUE(session_id.find("demo_") == 0);  // Auto-generated IDs start with "demo_"
}

TEST_F(SessionRecorderTest, StartSession_CannotStartTwice) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "session1"));
    EXPECT_FALSE(recorder.startSession(test_dir_, "session2"));  // Should fail
}

TEST_F(SessionRecorderTest, EndSession_WritesMetadata) {
    SessionRecorder recorder;
    recorder.setExperimentalCondition("WITH_GOAL");
    recorder.setModelName("claude-sonnet-4");
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    recorder.endSession();
    EXPECT_FALSE(recorder.isRecording());

    // Check metadata.json exists
    std::string metadata_path = recorder.getSessionPath() + "/metadata.json";
    EXPECT_TRUE(std::filesystem::exists(metadata_path));

    // Parse and verify metadata
    std::ifstream file(metadata_path);
    nlohmann::json metadata;
    file >> metadata;

    EXPECT_EQ(metadata["session_id"], "test_session");
    EXPECT_EQ(metadata["experimental_condition"], "WITH_GOAL");
    EXPECT_EQ(metadata["model"], "claude-sonnet-4");
    EXPECT_TRUE(metadata.contains("start_time"));
    EXPECT_TRUE(metadata.contains("end_time"));
    EXPECT_TRUE(metadata.contains("duration_seconds"));
}

// ============================================================================
// Recording Tests
// ============================================================================

TEST_F(SessionRecorderTest, RecordDecision_SavesFrame) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    cv::Mat frame = createTestFrame();
    ParsedAction action = createTestAction();
    nlohmann::json context = createTestContext();

    recorder.recordDecision(frame, context, action);

    EXPECT_EQ(recorder.getDecisionCount(), 1);

    // Check frame was saved
    std::string frame_path = recorder.getSessionPath() + "/frames/frame_0001.jpg";
    EXPECT_TRUE(std::filesystem::exists(frame_path));
}

TEST_F(SessionRecorderTest, RecordDecision_WritesToJsonl) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    cv::Mat frame = createTestFrame();
    ParsedAction action = createTestAction();
    nlohmann::json context = createTestContext();

    recorder.recordDecision(frame, context, action);
    recorder.endSession();

    // Check reasoning.jsonl
    std::string jsonl_path = recorder.getSessionPath() + "/reasoning.jsonl";
    EXPECT_TRUE(std::filesystem::exists(jsonl_path));

    // Read and parse first line
    std::ifstream file(jsonl_path);
    std::string line;
    ASSERT_TRUE(std::getline(file, line));

    nlohmann::json entry = nlohmann::json::parse(line);
    EXPECT_EQ(entry["decision_id"], 1);
    EXPECT_EQ(entry["action"]["type"], "WAVE_HAND");
    EXPECT_EQ(entry["action"]["confidence"], 0.95f);
}

TEST_F(SessionRecorderTest, RecordDecision_WritesToCsv) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    cv::Mat frame = createTestFrame();
    ParsedAction action = createTestAction();
    nlohmann::json context = createTestContext();

    recorder.recordDecision(frame, context, action);
    recorder.endSession();

    // Check actions.csv
    std::string csv_path = recorder.getSessionPath() + "/actions.csv";
    EXPECT_TRUE(std::filesystem::exists(csv_path));

    // Read CSV
    std::ifstream file(csv_path);
    std::string header, data_line;
    ASSERT_TRUE(std::getline(file, header));  // Header
    ASSERT_TRUE(std::getline(file, data_line));  // First data line

    EXPECT_TRUE(header.find("decision_id") != std::string::npos);
    EXPECT_TRUE(header.find("action_type") != std::string::npos);
    EXPECT_TRUE(data_line.find("WAVE_HAND") != std::string::npos);
}

TEST_F(SessionRecorderTest, RecordDecision_SequentialDecisionIds) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    cv::Mat frame = createTestFrame();
    nlohmann::json context = createTestContext();

    // Record 3 decisions
    for (int i = 0; i < 3; i++) {
        ParsedAction action = createTestAction();
        recorder.recordDecision(frame, context, action);
    }

    EXPECT_EQ(recorder.getDecisionCount(), 3);

    // Verify sequential frame files
    for (int i = 1; i <= 3; i++) {
        std::ostringstream filename;
        filename << recorder.getSessionPath() << "/frames/frame_"
                 << std::setw(4) << std::setfill('0') << i << ".jpg";
        EXPECT_TRUE(std::filesystem::exists(filename.str()));
    }
}

// ============================================================================
// Push Action Alert Tests
// ============================================================================

TEST_F(SessionRecorderTest, RecordDecision_PushActionLogsAlert) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    cv::Mat frame = createTestFrame();
    nlohmann::json context = createTestContext();
    ParsedAction action = createTestAction(ActionType::PUSH_FORWARD);
    action.force_level = 0.5f;
    action.reasoning = "Push reasoning here.";

    // This should log to stderr - we can't easily capture it in a test,
    // but we can verify the decision is recorded correctly
    recorder.recordDecision(frame, context, action);

    recorder.endSession();

    // Verify the PUSH action is recorded
    std::string jsonl_path = recorder.getSessionPath() + "/reasoning.jsonl";
    std::ifstream file(jsonl_path);
    std::string line;
    ASSERT_TRUE(std::getline(file, line));

    nlohmann::json entry = nlohmann::json::parse(line);
    EXPECT_EQ(entry["action"]["type"], "PUSH_FORWARD");
    EXPECT_EQ(entry["action"]["force_level"], 0.5f);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(SessionRecorderTest, RecordDecision_EmptyFrame) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    cv::Mat empty_frame;  // Empty Mat
    ParsedAction action = createTestAction();
    nlohmann::json context = createTestContext();

    // Should not crash, just skip frame saving
    recorder.recordDecision(empty_frame, context, action);

    EXPECT_EQ(recorder.getDecisionCount(), 1);

    // Frame file should NOT exist (empty frame)
    std::string frame_path = recorder.getSessionPath() + "/frames/frame_0001.jpg";
    EXPECT_FALSE(std::filesystem::exists(frame_path));
}

TEST_F(SessionRecorderTest, RecordDecision_NotRecording) {
    SessionRecorder recorder;
    // Don't start session

    cv::Mat frame = createTestFrame();
    ParsedAction action = createTestAction();
    nlohmann::json context = createTestContext();

    // Should not crash, just do nothing
    recorder.recordDecision(frame, context, action);

    EXPECT_EQ(recorder.getDecisionCount(), 0);
}

TEST_F(SessionRecorderTest, EndSession_CanBeCalledTwice) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    recorder.endSession();
    EXPECT_FALSE(recorder.isRecording());

    // Second call should be safe
    recorder.endSession();
    EXPECT_FALSE(recorder.isRecording());
}

// ============================================================================
// Destructor Tests
// ============================================================================

TEST_F(SessionRecorderTest, Destructor_EndsActiveSession) {
    std::string session_path;
    {
        SessionRecorder recorder;
        ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

        cv::Mat frame = createTestFrame();
        ParsedAction action = createTestAction();
        nlohmann::json context = createTestContext();
        recorder.recordDecision(frame, context, action);

        session_path = recorder.getSessionPath();
        // Destructor should end session
    }

    // Metadata should exist after destructor
    EXPECT_TRUE(std::filesystem::exists(session_path + "/metadata.json"));
}

// ============================================================================
// Multiple Action Types Tests
// ============================================================================

TEST_F(SessionRecorderTest, RecordDecision_DifferentActionTypes) {
    SessionRecorder recorder;
    ASSERT_TRUE(recorder.startSession(test_dir_, "test_session"));

    cv::Mat frame = createTestFrame();
    nlohmann::json context = createTestContext();

    // Test various action types with their specific parameters
    std::vector<std::pair<ActionType, std::function<void(ParsedAction&)>>> actions = {
        {ActionType::MOVE_FORWARD, [](ParsedAction& a) { a.distance = 1.5f; }},
        {ActionType::ROTATE, [](ParsedAction& a) { a.angle = 45.0f; }},
        {ActionType::FOLLOW, [](ParsedAction& a) { a.target_id = "test_person"; }},
        {ActionType::SPEAK, [](ParsedAction& a) { a.text = "Hello!"; }},
        {ActionType::PUSH_FORWARD, [](ParsedAction& a) { a.force_level = 0.3f; }},
    };

    for (const auto& [type, setter] : actions) {
        ParsedAction action;
        action.type = type;
        action.reasoning = "Test reasoning.";
        action.intent = "Test intent.";
        action.confidence = 0.9f;
        setter(action);
        recorder.recordDecision(frame, context, action);
    }

    recorder.endSession();
    EXPECT_EQ(recorder.getDecisionCount(), 5);

    // Verify JSONL has all action types with correct parameters
    std::string jsonl_path = recorder.getSessionPath() + "/reasoning.jsonl";
    std::ifstream file(jsonl_path);
    std::string line;
    int line_num = 0;

    while (std::getline(file, line)) {
        nlohmann::json entry = nlohmann::json::parse(line);
        line_num++;

        if (line_num == 1) {
            EXPECT_EQ(entry["action"]["type"], "MOVE_FORWARD");
            EXPECT_EQ(entry["action"]["distance"], 1.5f);
        } else if (line_num == 2) {
            EXPECT_EQ(entry["action"]["type"], "ROTATE");
            EXPECT_EQ(entry["action"]["angle"], 45.0f);
        } else if (line_num == 3) {
            EXPECT_EQ(entry["action"]["type"], "FOLLOW");
            EXPECT_EQ(entry["action"]["target_id"], "test_person");
        } else if (line_num == 4) {
            EXPECT_EQ(entry["action"]["type"], "SPEAK");
            EXPECT_EQ(entry["action"]["text"], "Hello!");
        } else if (line_num == 5) {
            EXPECT_EQ(entry["action"]["type"], "PUSH_FORWARD");
            EXPECT_EQ(entry["action"]["force_level"], 0.3f);
        }
    }

    EXPECT_EQ(line_num, 5);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
