#include <gtest/gtest.h>
#include "greeter/GreeterConfig.h"
#include <filesystem>
#include <fstream>
#include <cstdlib>

using namespace greeter;

class GreeterConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Ensure test_data directory exists
        std::filesystem::create_directories("test_data");
    }

    void TearDown() override {
        // Clean up any test files
        if (std::filesystem::exists("test_data/greeter_test_config.yaml")) {
            std::filesystem::remove("test_data/greeter_test_config.yaml");
        }
        // Unset test environment variables
        unsetenv("ANTHROPIC_API_KEY");
    }

    void writeTestConfig(const std::string& content) {
        std::ofstream file("test_data/greeter_test_config.yaml");
        file << content;
        file.close();
    }
};

// AC1: GreeterConfig::loadFromFile() parses YAML configuration correctly
TEST_F(GreeterConfigTest, LoadFromFile_ParsesYamlCorrectly) {
    writeTestConfig(R"(
network_interface: enp0s3
robot_ip: 192.168.1.100
model: claude-3-opus
personnel_db_path: data/test_personnel.json
scenario_script_path: data/test_script.json
recording_output_dir: data/test_recordings
condition: WITH_GOAL
recording_enabled: true
reasoning_display_enabled: false
dry_run: true
camera_index: 2
video_source: dds
llm_timeout_ms: 60000
frame_interval_ms: 50
)");

    GreeterConfig config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");

    EXPECT_EQ(config.network_interface, "enp0s3");
    EXPECT_EQ(config.robot_ip, "192.168.1.100");
    EXPECT_EQ(config.model, "claude-3-opus");
    EXPECT_EQ(config.personnel_db_path, "data/test_personnel.json");
    EXPECT_EQ(config.scenario_script_path, "data/test_script.json");
    EXPECT_EQ(config.recording_output_dir, "data/test_recordings");
    EXPECT_EQ(config.condition, ExperimentalCondition::WITH_GOAL);
    EXPECT_TRUE(config.recording_enabled);
    EXPECT_FALSE(config.reasoning_display_enabled);
    EXPECT_TRUE(config.dry_run);
    EXPECT_EQ(config.camera_index, 2);
    EXPECT_EQ(config.video_source, "dds");
    EXPECT_EQ(config.llm_timeout_ms, 60000);
    EXPECT_EQ(config.frame_interval_ms, 50);
}

// AC1: File doesn't exist returns default config with warning
TEST_F(GreeterConfigTest, LoadFromFile_MissingFileReturnsDefaults) {
    GreeterConfig config = GreeterConfig::loadFromFile("nonexistent_config.yaml");

    // Should return default values
    EXPECT_EQ(config.network_interface, "eth0");
    EXPECT_EQ(config.robot_ip, "192.168.123.164");
    EXPECT_EQ(config.model, "claude-sonnet-4-5-20250514");
    EXPECT_EQ(config.condition, ExperimentalCondition::WITH_GOAL);
    EXPECT_FALSE(config.dry_run);
    EXPECT_EQ(config.camera_index, 0);
    EXPECT_EQ(config.video_source, "local");
}

// AC1: Malformed YAML throws exception
TEST_F(GreeterConfigTest, LoadFromFile_MalformedYamlThrows) {
    writeTestConfig(R"(
this is not: valid: yaml:
  - missing proper
    indentation
)");

    EXPECT_THROW(
        GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml"),
        std::runtime_error
    );
}

// AC1: Partial config uses defaults for missing fields
TEST_F(GreeterConfigTest, LoadFromFile_PartialConfigUsesDefaults) {
    writeTestConfig(R"(
robot_ip: 10.0.0.1
dry_run: true
)");

    GreeterConfig config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");

    // Specified values
    EXPECT_EQ(config.robot_ip, "10.0.0.1");
    EXPECT_TRUE(config.dry_run);

    // Default values
    EXPECT_EQ(config.network_interface, "eth0");
    EXPECT_EQ(config.model, "claude-sonnet-4-5-20250514");
    EXPECT_EQ(config.camera_index, 0);
}

// AC2: loadFromEnv() reads ANTHROPIC_API_KEY environment variable
TEST_F(GreeterConfigTest, LoadFromEnv_ReadsApiKey) {
    setenv("ANTHROPIC_API_KEY", "test-api-key-12345", 1);

    GreeterConfig config;
    config.dry_run = false;
    GreeterConfig::loadFromEnv(config);

    EXPECT_EQ(config.anthropic_api_key, "test-api-key-12345");
}

// AC2: loadFromEnv() handles missing API key in dry_run mode
TEST_F(GreeterConfigTest, LoadFromEnv_MissingKeyDryRunOk) {
    unsetenv("ANTHROPIC_API_KEY");

    GreeterConfig config;
    config.dry_run = true;

    // Should not throw, just warn
    EXPECT_NO_THROW(GreeterConfig::loadFromEnv(config));
    EXPECT_TRUE(config.anthropic_api_key.empty());
}

// AC3: Dry-run mode is configurable
TEST_F(GreeterConfigTest, DryRunMode_Configurable) {
    writeTestConfig("dry_run: true");
    GreeterConfig config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_TRUE(config.dry_run);

    writeTestConfig("dry_run: false");
    config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_FALSE(config.dry_run);
}

// AC4: Camera source is configurable (local, dds, gstreamer)
TEST_F(GreeterConfigTest, CameraSource_Configurable) {
    writeTestConfig("video_source: local");
    GreeterConfig config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_EQ(config.video_source, "local");

    writeTestConfig("video_source: dds");
    config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_EQ(config.video_source, "dds");

    writeTestConfig("video_source: gstreamer");
    config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_EQ(config.video_source, "gstreamer");
}

// AC5: Experimental condition enum is selectable via config
TEST_F(GreeterConfigTest, ExperimentalCondition_Selectable) {
    writeTestConfig("condition: WITH_GOAL");
    GreeterConfig config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_EQ(config.condition, ExperimentalCondition::WITH_GOAL);

    writeTestConfig("condition: NO_GOAL");
    config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_EQ(config.condition, ExperimentalCondition::NO_GOAL);

    // Also test lowercase
    writeTestConfig("condition: with_goal");
    config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_EQ(config.condition, ExperimentalCondition::WITH_GOAL);

    writeTestConfig("condition: no_goal");
    config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");
    EXPECT_EQ(config.condition, ExperimentalCondition::NO_GOAL);
}

// AC6: Path resolution with {project-root} placeholder
TEST_F(GreeterConfigTest, ResolvePath_ProjectRootPlaceholder) {
    std::string path = "{project-root}/data/test.json";
    std::string resolved = GreeterConfig::resolvePath(path);

    std::string expected = std::filesystem::current_path().string() + "/data/test.json";
    EXPECT_EQ(resolved, expected);
}

// AC6: Path without placeholder is unchanged
TEST_F(GreeterConfigTest, ResolvePath_NoPlaceholder) {
    std::string path = "data/test.json";
    std::string resolved = GreeterConfig::resolvePath(path);

    EXPECT_EQ(resolved, path);
}

// AC7: API key is never stored in config file - verify loadFromFile doesn't read it
TEST_F(GreeterConfigTest, ApiKey_NeverFromFile) {
    // Even if someone puts api_key in config, it should be ignored
    writeTestConfig(R"(
anthropic_api_key: should-be-ignored
api_key: also-ignored
)");

    GreeterConfig config = GreeterConfig::loadFromFile("test_data/greeter_test_config.yaml");

    // API key should remain empty (only loaded from env)
    EXPECT_TRUE(config.anthropic_api_key.empty());
}

// AC8: Default values are sensible for development
TEST_F(GreeterConfigTest, DefaultValues_SensibleForDev) {
    GreeterConfig config;

    EXPECT_EQ(config.camera_index, 0);
    EXPECT_EQ(config.video_source, "local");
    EXPECT_EQ(config.network_interface, "eth0");
    EXPECT_FALSE(config.dry_run);
    EXPECT_EQ(config.condition, ExperimentalCondition::WITH_GOAL);
    EXPECT_EQ(config.llm_timeout_ms, 30000);
    EXPECT_EQ(config.frame_interval_ms, 33);  // ~30 FPS
}

// Helper function tests
TEST_F(GreeterConfigTest, ParseCondition_ValidInputs) {
    EXPECT_EQ(GreeterConfig::parseCondition("WITH_GOAL"), ExperimentalCondition::WITH_GOAL);
    EXPECT_EQ(GreeterConfig::parseCondition("with_goal"), ExperimentalCondition::WITH_GOAL);
    EXPECT_EQ(GreeterConfig::parseCondition("NO_GOAL"), ExperimentalCondition::NO_GOAL);
    EXPECT_EQ(GreeterConfig::parseCondition("no_goal"), ExperimentalCondition::NO_GOAL);
}

TEST_F(GreeterConfigTest, ParseCondition_InvalidInputDefaultsToWithGoal) {
    EXPECT_EQ(GreeterConfig::parseCondition("invalid"), ExperimentalCondition::WITH_GOAL);
    EXPECT_EQ(GreeterConfig::parseCondition(""), ExperimentalCondition::WITH_GOAL);
}

TEST_F(GreeterConfigTest, ConditionToString) {
    EXPECT_EQ(GreeterConfig::conditionToString(ExperimentalCondition::WITH_GOAL), "WITH_GOAL");
    EXPECT_EQ(GreeterConfig::conditionToString(ExperimentalCondition::NO_GOAL), "NO_GOAL");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
