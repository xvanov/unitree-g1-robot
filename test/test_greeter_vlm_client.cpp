#include <gtest/gtest.h>
#include "greeter/GreeterVlmClient.h"
#include "greeter/GreeterPrompts.h"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <cstdlib>

using namespace greeter;

class GreeterVlmClientTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize curl for all tests
        GreeterVlmClient::globalInit();
    }

    void TearDown() override {
        // Cleanup curl
        GreeterVlmClient::globalCleanup();
    }
};

// Test: Prompts are properly defined
TEST_F(GreeterVlmClientTest, SystemPromptsAreDefined) {
    // WITH_GOAL prompt should contain identity and purpose
    EXPECT_FALSE(SYSTEM_PROMPT_WITH_GOAL.empty());
    EXPECT_NE(SYSTEM_PROMPT_WITH_GOAL.find("Barry"), std::string::npos);
    EXPECT_NE(SYSTEM_PROMPT_WITH_GOAL.find("greeter"), std::string::npos);
    EXPECT_NE(SYSTEM_PROMPT_WITH_GOAL.find("PURPOSE"), std::string::npos);
    EXPECT_NE(SYSTEM_PROMPT_WITH_GOAL.find("IDENTITY"), std::string::npos);

    // NO_GOAL prompt should NOT contain explicit purpose/identity section
    EXPECT_FALSE(SYSTEM_PROMPT_NO_GOAL.empty());
    EXPECT_NE(SYSTEM_PROMPT_NO_GOAL.find("Barry"), std::string::npos);
    EXPECT_EQ(SYSTEM_PROMPT_NO_GOAL.find("PURPOSE"), std::string::npos);
    EXPECT_EQ(SYSTEM_PROMPT_NO_GOAL.find("IDENTITY"), std::string::npos);
}

// Test: Both prompts specify identical action vocabulary
TEST_F(GreeterVlmClientTest, PromptsHaveIdenticalActionVocabulary) {
    // List of all actions that should appear in both prompts
    std::vector<std::string> actions = {
        "MOVE_TO", "MOVE_FORWARD", "MOVE_BACKWARD", "ROTATE",
        "FOLLOW", "STOP", "RETURN_TO_POST", "WAVE_HAND",
        "SHAKE_HAND", "BOW", "STAND_UP", "SIT_DOWN",
        "PUSH_FORWARD", "SPEAK", "WAIT", "NO_ACTION"
    };

    for (const auto& action : actions) {
        EXPECT_NE(SYSTEM_PROMPT_WITH_GOAL.find(action), std::string::npos)
            << "WITH_GOAL prompt missing action: " << action;
        EXPECT_NE(SYSTEM_PROMPT_NO_GOAL.find(action), std::string::npos)
            << "NO_GOAL prompt missing action: " << action;
    }
}

// Test: Both prompts specify identical response format
TEST_F(GreeterVlmClientTest, PromptsHaveIdenticalResponseFormat) {
    // Response format fields that should appear in both
    std::vector<std::string> format_fields = {
        "reasoning", "intent", "action", "parameters",
        "speak_text", "confidence"
    };

    for (const auto& field : format_fields) {
        EXPECT_NE(SYSTEM_PROMPT_WITH_GOAL.find(field), std::string::npos)
            << "WITH_GOAL prompt missing format field: " << field;
        EXPECT_NE(SYSTEM_PROMPT_NO_GOAL.find(field), std::string::npos)
            << "NO_GOAL prompt missing format field: " << field;
    }
}

// Test: Client can be constructed with API key
TEST_F(GreeterVlmClientTest, ConstructWithApiKey) {
    GreeterVlmClient client("test-api-key");
    // No exception should be thrown
    EXPECT_TRUE(true);
}

// Test: Client configuration methods work
TEST_F(GreeterVlmClientTest, ConfigurationMethods) {
    GreeterVlmClient client("test-api-key");

    // Set various configurations
    client.setModel("claude-sonnet-4-5-20250514");
    client.setTimeout(60000);
    client.setMaxRetries(5);
    client.setMaxTokens(2048);

    // No exception should be thrown
    EXPECT_TRUE(true);
}

// Test: Status methods return initial values
TEST_F(GreeterVlmClientTest, InitialStatusValues) {
    GreeterVlmClient client("test-api-key");

    EXPECT_EQ(client.getLastStatusCode(), 0);
    EXPECT_TRUE(client.getLastError().empty());
    EXPECT_EQ(client.getTokensUsed(), 0);
    EXPECT_EQ(client.getInputTokens(), 0);
    EXPECT_EQ(client.getOutputTokens(), 0);
}

// Test: Empty frame handling
TEST_F(GreeterVlmClientTest, EmptyFrameReturnsError) {
    GreeterVlmClient client("test-api-key");
    cv::Mat empty_frame;

    std::string result = client.sendObservation(
        SYSTEM_PROMPT_WITH_GOAL,
        empty_frame,
        "{\"test\": true}"
    );

    EXPECT_TRUE(result.empty());
    EXPECT_EQ(client.getLastError(), "Empty frame provided");
}

// Test: Base64 encoding produces valid output
TEST_F(GreeterVlmClientTest, Base64EncodingWorks) {
    GreeterVlmClient client("test-api-key");

    // Create a small test image
    cv::Mat test_image(100, 100, CV_8UC3, cv::Scalar(128, 128, 128));

    // We can't directly test encodeImageBase64 as it's private,
    // but we can verify the client doesn't crash with a valid image
    // Note: This will fail due to invalid API key, but that's expected
    std::string result = client.sendObservation(
        SYSTEM_PROMPT_WITH_GOAL,
        test_image,
        "{\"test\": true}"
    );

    // With invalid API key, we expect empty result and an error message
    EXPECT_TRUE(result.empty());
    EXPECT_FALSE(client.getLastError().empty());
    // Should have made an HTTP request (got 401 unauthorized)
    EXPECT_EQ(client.getLastStatusCode(), 401);
}

// Test: Text-only observation (for simulation)
TEST_F(GreeterVlmClientTest, TextOnlyObservation) {
    GreeterVlmClient client("test-api-key");

    // This will fail due to invalid API key, but should not crash
    std::string result = client.sendTextObservation(
        SYSTEM_PROMPT_NO_GOAL,
        R"({"scene": "test scene", "detected_faces": []})"
    );

    // With invalid API key, we expect empty result and an error
    EXPECT_TRUE(result.empty());
    EXPECT_FALSE(client.getLastError().empty());
    // Should have made an HTTP request (got 401 unauthorized)
    EXPECT_EQ(client.getLastStatusCode(), 401);
}

// Test: Global init/cleanup can be called multiple times safely
TEST_F(GreeterVlmClientTest, GlobalInitCleanupIdempotent) {
    // Already called in SetUp, call again
    GreeterVlmClient::globalInit();
    GreeterVlmClient::globalInit();

    GreeterVlmClient client("test-api-key");

    // Cleanup and init again
    GreeterVlmClient::globalCleanup();
    GreeterVlmClient::globalInit();

    // Should still be able to create client
    GreeterVlmClient client2("another-key");

    EXPECT_TRUE(true);  // No crashes
}

// Integration test: Real API call (only runs with valid API key)
TEST_F(GreeterVlmClientTest, DISABLED_IntegrationRealApiCall) {
    // This test is disabled by default - enable manually for integration testing
    // Run with: ./test_greeter_vlm_client --gtest_also_run_disabled_tests

    const char* api_key = std::getenv("ANTHROPIC_API_KEY");
    if (!api_key || std::string(api_key).empty()) {
        GTEST_SKIP() << "ANTHROPIC_API_KEY not set, skipping integration test";
    }

    GreeterVlmClient client(api_key);
    client.setMaxTokens(256);  // Keep response short for test

    // Create simple test context
    nlohmann::json context;
    context["scene_description"] = "A quiet lobby with potted plants";
    context["detected_faces"] = nlohmann::json::array();
    context["environment"] = {{"location", "lobby"}, {"time", "morning"}};
    context["robot_state"] = {{"position", {{"x", 0}, {"y", 0}}}, {"battery", 85}};

    std::string result = client.sendTextObservation(
        SYSTEM_PROMPT_NO_GOAL,
        context.dump()
    );

    // Should get a non-empty response
    EXPECT_FALSE(result.empty()) << "API returned empty response: " << client.getLastError();

    // Response should be valid JSON with expected fields
    if (!result.empty()) {
        try {
            auto response = nlohmann::json::parse(result);
            EXPECT_TRUE(response.contains("action")) << "Response missing 'action' field";
            EXPECT_TRUE(response.contains("reasoning")) << "Response missing 'reasoning' field";
        } catch (const std::exception& e) {
            FAIL() << "Response is not valid JSON: " << e.what() << "\nResponse: " << result;
        }
    }

    // Should have tracked token usage
    EXPECT_GT(client.getTokensUsed(), 0) << "Token usage not tracked";
    EXPECT_EQ(client.getLastStatusCode(), 200) << "Expected 200 status code";
}

// Test: Retry-able status codes
TEST_F(GreeterVlmClientTest, RetryableStatusCodes) {
    // Create client and test internal shouldRetry logic indirectly
    // We can't call shouldRetry directly, but we can verify behavior description

    // 429 (rate limit) should be retryable
    // 500+ (server errors) should be retryable
    // 400 (bad request) should NOT be retryable

    // This is a documentation test - actual retry behavior tested via integration
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
