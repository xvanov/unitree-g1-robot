#include <gtest/gtest.h>
#include "greeter/ScenarioScript.h"
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>

using namespace greeter;

class ScenarioScriptTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a minimal test scenario JSON
        test_dir_ = "test_scenario_temp";
        std::filesystem::create_directories(test_dir_);

        createMinimalScenario();
    }

    void TearDown() override {
        std::filesystem::remove_all(test_dir_);
    }

    void createMinimalScenario() {
        nlohmann::json scenario = {
            {"scenario_id", "test_001"},
            {"name", "Test Scenario"},
            {"events", {
                {
                    {"id", "event_1"},
                    {"type", "PERSON_ENTERS"},
                    {"trigger", {{"type", "after_seconds"}, {"time", 1.0}}},
                    {"payload", {{"person_id", "test_person"}}}
                },
                {
                    {"id", "event_2"},
                    {"type", "OVERHEARD_CONVERSATION"},
                    {"trigger", {{"type", "after_seconds"}, {"time", 2.0}}},
                    {"payload", {
                        {"content", "This is a test conversation."},
                        {"speakers", {"Person A", "Person B"}}
                    }}
                },
                {
                    {"id", "event_3"},
                    {"type", "ENVIRONMENT_CHANGE"},
                    {"trigger", {{"type", "on_greet"}, {"person_id", "test_person"}}},
                    {"payload", {{"changes", {{"near_staircase", true}}}}}
                },
                {
                    {"id", "event_4"},
                    {"type", "MANUAL_TRIGGER"},
                    {"trigger", {{"type", "manual"}}},
                    {"payload", {{"action", "end_scenario"}}}
                }
            }}
        };

        std::ofstream file(test_dir_ + "/scenario.json");
        file << scenario.dump(2);
        file.close();
    }

    std::string test_dir_;
};

// ============================================================================
// Loading Tests
// ============================================================================

TEST_F(ScenarioScriptTest, LoadFromFile_ValidFile) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));
    EXPECT_TRUE(script.isLoaded());
    EXPECT_EQ(script.getScenarioId(), "test_001");
    EXPECT_EQ(script.getScenarioName(), "Test Scenario");
    EXPECT_EQ(script.getEventCount(), 4);
}

TEST_F(ScenarioScriptTest, LoadFromFile_InvalidPath) {
    ScenarioScript script;
    ASSERT_FALSE(script.loadFromFile("nonexistent/path/scenario.json"));
    EXPECT_FALSE(script.isLoaded());
}

TEST_F(ScenarioScriptTest, LoadFromFile_InvalidJson) {
    // Create an invalid JSON file
    std::ofstream file(test_dir_ + "/invalid.json");
    file << "{ invalid json }";
    file.close();

    ScenarioScript script;
    ASSERT_FALSE(script.loadFromFile(test_dir_ + "/invalid.json"));
    EXPECT_FALSE(script.isLoaded());
}

TEST_F(ScenarioScriptTest, LoadFromFile_LoadsActualDemoScript) {
    ScenarioScript script;
    std::string demo_path = "data/scripts/barry_demo.json";

    if (std::filesystem::exists(demo_path)) {
        ASSERT_TRUE(script.loadFromFile(demo_path));
        EXPECT_TRUE(script.isLoaded());
        EXPECT_FALSE(script.getScenarioId().empty());
        EXPECT_GT(script.getEventCount(), 0);
    } else {
        GTEST_SKIP() << "Demo script not found at " << demo_path;
    }
}

// ============================================================================
// Time-Based Trigger Tests
// ============================================================================

TEST_F(ScenarioScriptTest, Update_TriggersAfterSeconds) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // At t=0.5s, no events should fire
    script.update(0.5f);
    auto events = script.getPendingEvents();
    EXPECT_EQ(events.size(), 0);

    // At t=1.5s, event_1 should fire (trigger at 1.0s)
    script.update(1.5f);
    events = script.getPendingEvents();
    ASSERT_EQ(events.size(), 1);
    EXPECT_EQ(events[0].id, "event_1");
    EXPECT_EQ(events[0].type, ScenarioEventType::PERSON_ENTERS);
}

TEST_F(ScenarioScriptTest, Update_TriggersMultipleEvents) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Jump to t=3.0s - both event_1 (1.0s) and event_2 (2.0s) should fire
    script.update(3.0f);
    auto events = script.getPendingEvents();
    EXPECT_EQ(events.size(), 2);
}

TEST_F(ScenarioScriptTest, Update_EventTriggersOnlyOnce) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Trigger event_1
    script.update(1.5f);
    auto events = script.getPendingEvents();
    EXPECT_EQ(events.size(), 1);

    // Update again - event_1 should NOT fire again
    script.update(2.0f);
    events = script.getPendingEvents();
    // Only event_2 should be in pending (not event_1 again)
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events[0].id, "event_2");
}

// ============================================================================
// Greet Trigger Tests
// ============================================================================

TEST_F(ScenarioScriptTest, OnGreet_TriggersMatchingEvent) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Greet the test person - should trigger event_3
    script.onGreet("test_person");
    auto events = script.getPendingEvents();
    ASSERT_EQ(events.size(), 1);
    EXPECT_EQ(events[0].id, "event_3");
    EXPECT_EQ(events[0].type, ScenarioEventType::ENVIRONMENT_CHANGE);
}

TEST_F(ScenarioScriptTest, OnGreet_NoMatchingPerson) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Greet a different person - no events should fire
    script.onGreet("unknown_person");
    auto events = script.getPendingEvents();
    EXPECT_EQ(events.size(), 0);
}

// ============================================================================
// Manual Trigger Tests
// ============================================================================

TEST_F(ScenarioScriptTest, OnManualTrigger_TriggersEvent) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Manually trigger event_4
    script.onManualTrigger("event_4");
    auto events = script.getPendingEvents();
    ASSERT_EQ(events.size(), 1);
    EXPECT_EQ(events[0].id, "event_4");
    EXPECT_EQ(events[0].type, ScenarioEventType::MANUAL_TRIGGER);
}

TEST_F(ScenarioScriptTest, OnManualTrigger_UnknownEventId) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Try to trigger non-existent event
    script.onManualTrigger("nonexistent_event");
    auto events = script.getPendingEvents();
    EXPECT_EQ(events.size(), 0);
}

// ============================================================================
// Overheard Conversation Tests
// ============================================================================

TEST_F(ScenarioScriptTest, OverheardConversation_TracksContent) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Initially no conversations
    auto convs = script.getOverheardConversations();
    EXPECT_EQ(convs.size(), 0);

    // Trigger event_2 (conversation event)
    script.update(2.5f);
    script.getPendingEvents();  // Must call to process events

    convs = script.getOverheardConversations();
    ASSERT_EQ(convs.size(), 1);
    EXPECT_EQ(convs[0], "This is a test conversation.");
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(ScenarioScriptTest, Reset_ClearsTriggeredState) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Trigger some events
    script.update(3.0f);
    script.getPendingEvents();

    // Reset
    script.reset();

    // Events should be able to fire again
    script.update(1.5f);
    auto events = script.getPendingEvents();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events[0].id, "event_1");
}

TEST_F(ScenarioScriptTest, Reset_ClearsConversations) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    // Trigger conversation event
    script.update(2.5f);
    script.getPendingEvents();
    EXPECT_EQ(script.getOverheardConversations().size(), 1);

    // Reset
    script.reset();
    EXPECT_EQ(script.getOverheardConversations().size(), 0);
}

TEST_F(ScenarioScriptTest, Reset_ResetsElapsedTime) {
    ScenarioScript script;
    ASSERT_TRUE(script.loadFromFile(test_dir_ + "/scenario.json"));

    script.update(5.0f);
    EXPECT_FLOAT_EQ(script.getElapsedTime(), 5.0f);

    script.reset();
    EXPECT_FLOAT_EQ(script.getElapsedTime(), 0.0f);
}

// ============================================================================
// Event Type Conversion Tests
// ============================================================================

TEST_F(ScenarioScriptTest, EventTypeToString_AllTypes) {
    EXPECT_EQ(ScenarioEvent::typeToString(ScenarioEventType::PERSON_ENTERS), "PERSON_ENTERS");
    EXPECT_EQ(ScenarioEvent::typeToString(ScenarioEventType::PERSON_EXITS), "PERSON_EXITS");
    EXPECT_EQ(ScenarioEvent::typeToString(ScenarioEventType::OVERHEARD_CONVERSATION), "OVERHEARD_CONVERSATION");
    EXPECT_EQ(ScenarioEvent::typeToString(ScenarioEventType::ENVIRONMENT_CHANGE), "ENVIRONMENT_CHANGE");
    EXPECT_EQ(ScenarioEvent::typeToString(ScenarioEventType::MANUAL_TRIGGER), "MANUAL_TRIGGER");
}

TEST_F(ScenarioScriptTest, TriggerTypeToString_AllTypes) {
    EXPECT_EQ(ScenarioEvent::triggerToString(TriggerType::ON_GREET), "on_greet");
    EXPECT_EQ(ScenarioEvent::triggerToString(TriggerType::AFTER_SECONDS), "after_seconds");
    EXPECT_EQ(ScenarioEvent::triggerToString(TriggerType::MANUAL), "manual");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
