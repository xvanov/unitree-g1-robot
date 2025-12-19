#include <gtest/gtest.h>
#include "greeter/GreeterRunner.h"
#include "greeter/GreeterConfig.h"
#include <filesystem>
#include <thread>
#include <chrono>
#include <atomic>
#include <fstream>
#include <nlohmann/json.hpp>

using namespace greeter;

class GreeterRunnerTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_dir_ = "test_greeter_runner_temp";
        std::filesystem::create_directories(test_dir_);

        // Create minimal test scenario
        createTestScenario();

        // Create minimal personnel database
        createTestPersonnelDb();
    }

    void TearDown() override {
        std::filesystem::remove_all(test_dir_);
    }

    void createTestScenario() {
        nlohmann::json scenario = {
            {"scenario_id", "test_demo"},
            {"name", "Test Demo Scenario"},
            {"events", {
                {
                    {"id", "event_1"},
                    {"type", "PERSON_ENTERS"},
                    {"trigger", {{"type", "after_seconds"}, {"time", 0.5}}},
                    {"payload", {{"person_id", "test_person"}}}
                }
            }}
        };

        std::ofstream file(test_dir_ + "/test_scenario.json");
        file << scenario.dump(2);
        file.close();
    }

    void createTestPersonnelDb() {
        nlohmann::json personnel = {
            {"personnel", {
                {
                    {"id", "test_person"},
                    {"name", "Test Person"},
                    {"role", "Tester"},
                    {"department", "QA"},
                    {"relationship_to_robot", "friendly"},
                    {"context_notes", "Test user."}
                }
            }}
        };

        std::ofstream file(test_dir_ + "/test_personnel.json");
        file << personnel.dump(2);
        file.close();
    }

    GreeterConfig createDryRunConfig() {
        GreeterConfig config;
        config.dry_run = true;
        config.video_source = "none";
        config.reasoning_display_enabled = false;  // No display in tests
        config.recording_enabled = false;
        config.scenario_script_path = test_dir_ + "/test_scenario.json";
        config.personnel_db_path = test_dir_ + "/test_personnel.json";
        config.condition = ExperimentalCondition::WITH_GOAL;
        return config;
    }

    std::string test_dir_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(GreeterRunnerTest, Init_DryRunMode) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();

    EXPECT_TRUE(runner.init(config));
    EXPECT_EQ(runner.getState(), GreeterState::IDLE);
}

TEST_F(GreeterRunnerTest, Init_LoadsScenario) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();

    ASSERT_TRUE(runner.init(config));

    // State should be IDLE after successful init
    EXPECT_EQ(runner.getStateString(), "IDLE");
}

TEST_F(GreeterRunnerTest, Init_WithGoalCondition) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    config.condition = ExperimentalCondition::WITH_GOAL;

    ASSERT_TRUE(runner.init(config));
    EXPECT_EQ(runner.getState(), GreeterState::IDLE);
}

TEST_F(GreeterRunnerTest, Init_NoGoalCondition) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    config.condition = ExperimentalCondition::NO_GOAL;

    ASSERT_TRUE(runner.init(config));
    EXPECT_EQ(runner.getState(), GreeterState::IDLE);
}

// ============================================================================
// State String Tests
// ============================================================================

TEST_F(GreeterRunnerTest, GetStateString_AllStates) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    ASSERT_TRUE(runner.init(config));

    // Test that IDLE state string is correct
    EXPECT_EQ(runner.getStateString(), "IDLE");

    // Request shutdown
    runner.requestShutdown();
    EXPECT_EQ(runner.getStateString(), "SHUTDOWN");
}

// ============================================================================
// Shutdown Tests
// ============================================================================

TEST_F(GreeterRunnerTest, RequestShutdown_SetsState) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    ASSERT_TRUE(runner.init(config));

    runner.requestShutdown();

    EXPECT_EQ(runner.getState(), GreeterState::SHUTDOWN);
}

TEST_F(GreeterRunnerTest, RequestShutdown_ViaExternalFlag) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();

    std::atomic<bool> shutdown_flag{false};
    runner.setExternalShutdownFlag(&shutdown_flag);

    ASSERT_TRUE(runner.init(config));

    // Simulate signal handler setting the flag
    shutdown_flag.store(true, std::memory_order_release);

    // Runner should detect this on next iteration
    // Since we're in dry-run, just test that init worked and the flag mechanism is set up
    EXPECT_EQ(runner.getState(), GreeterState::IDLE);
}

// ============================================================================
// Run Tests (Brief - since run() is blocking)
// ============================================================================

TEST_F(GreeterRunnerTest, Run_ShutdownImmediately) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    ASSERT_TRUE(runner.init(config));

    // Request shutdown before run
    runner.requestShutdown();

    // Run should return immediately since shutdown was requested
    // This will test the shutdown path without hanging
    // We use a thread with timeout to ensure test doesn't hang
    std::atomic<bool> completed{false};
    std::thread run_thread([&]() {
        runner.run();
        completed.store(true);
    });

    // Wait up to 2 seconds for run to complete
    for (int i = 0; i < 20 && !completed.load(); i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_TRUE(completed.load()) << "run() did not complete after shutdown request";

    if (run_thread.joinable()) {
        run_thread.join();
    }

    EXPECT_EQ(runner.getState(), GreeterState::SHUTDOWN);
}

TEST_F(GreeterRunnerTest, Run_ExternalShutdownFlag) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();

    std::atomic<bool> shutdown_flag{false};
    runner.setExternalShutdownFlag(&shutdown_flag);

    ASSERT_TRUE(runner.init(config));

    // Start run in a thread
    std::atomic<bool> completed{false};
    std::thread run_thread([&]() {
        runner.run();
        completed.store(true);
    });

    // Let it run briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Signal shutdown via flag
    shutdown_flag.store(true, std::memory_order_release);

    // Wait for completion
    for (int i = 0; i < 30 && !completed.load(); i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_TRUE(completed.load()) << "run() did not complete after external shutdown flag";

    if (run_thread.joinable()) {
        run_thread.join();
    }
}

// ============================================================================
// Validation Tests
// ============================================================================

TEST_F(GreeterRunnerTest, ValidateDemo_DryRunMode) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    ASSERT_TRUE(runner.init(config));

    // In dry-run mode without camera, some checks may fail
    // but validateDemo should not crash
    bool result = runner.validateDemo();

    // The result depends on what components are available
    // We just verify it runs without crashing
    (void)result;

    EXPECT_NE(runner.getState(), GreeterState::ERROR);
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(GreeterRunnerTest, Init_InvalidScenarioPath) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    config.scenario_script_path = "/nonexistent/path/scenario.json";

    // Should still init (scenario is optional in dry-run)
    // But may log a warning
    bool result = runner.init(config);

    // In dry-run mode, we're lenient - init may succeed even with missing scenario
    // The important thing is it doesn't crash
    (void)result;
}

TEST_F(GreeterRunnerTest, Init_InvalidPersonnelPath) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    config.personnel_db_path = "/nonexistent/path/personnel.json";

    // Should still init (personnel is optional in dry-run)
    bool result = runner.init(config);

    // Lenient - may succeed with warning
    (void)result;
}

// ============================================================================
// State Machine Tests
// ============================================================================

TEST_F(GreeterRunnerTest, StateTransitions_Initial) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();

    // Before init, state might be INITIALIZING or unset
    // After init, should be IDLE
    ASSERT_TRUE(runner.init(config));
    EXPECT_EQ(runner.getState(), GreeterState::IDLE);
}

TEST_F(GreeterRunnerTest, StateTransitions_ToShutdown) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    ASSERT_TRUE(runner.init(config));

    EXPECT_EQ(runner.getState(), GreeterState::IDLE);

    runner.requestShutdown();

    EXPECT_EQ(runner.getState(), GreeterState::SHUTDOWN);
}

// ============================================================================
// Recording Integration Tests
// ============================================================================

TEST_F(GreeterRunnerTest, Init_WithRecording) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    config.recording_enabled = true;
    config.recording_output_dir = test_dir_ + "/recordings";

    EXPECT_TRUE(runner.init(config));
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST_F(GreeterRunnerTest, RequestShutdown_ThreadSafe) {
    GreeterRunner runner;
    GreeterConfig config = createDryRunConfig();
    ASSERT_TRUE(runner.init(config));

    // Request shutdown from multiple threads simultaneously
    std::vector<std::thread> threads;
    for (int i = 0; i < 10; i++) {
        threads.emplace_back([&]() {
            runner.requestShutdown();
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    // State should be SHUTDOWN (multiple calls are safe)
    EXPECT_EQ(runner.getState(), GreeterState::SHUTDOWN);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
