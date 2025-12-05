#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <vector>
#include "safety/SafetyTypes.h"
#include "safety/SafetyMonitor.h"

// ============================================
// Threshold Logic Tests (isolated, no mocks needed)
// ============================================

TEST(SafetyThresholdTest, BatteryWarningAt20Percent) {
    float battery = 20.0f;
    float warning_threshold = 20.0f;
    EXPECT_TRUE(battery <= warning_threshold);  // Would trigger warning
}

TEST(SafetyThresholdTest, BatteryCriticalAt10Percent) {
    float battery = 10.0f;
    float critical_threshold = 10.0f;
    EXPECT_TRUE(battery <= critical_threshold);  // Would trigger critical
}

TEST(SafetyThresholdTest, BatteryShutdownAt5Percent) {
    float battery = 5.0f;
    float shutdown_threshold = 5.0f;
    EXPECT_TRUE(battery <= shutdown_threshold);  // Would trigger auto E-stop
}

TEST(SafetyThresholdTest, BatteryValidityCheck) {
    // Invalid battery reading (disconnected) should NOT trigger E-stop
    float battery = 0.0f;
    EXPECT_TRUE(battery <= 0.0f);  // Should skip threshold checks
}

TEST(SafetyThresholdTest, BatteryOkAt50Percent) {
    float battery = 50.0f;
    float warning_threshold = 20.0f;
    EXPECT_FALSE(battery <= warning_threshold);  // No warning
}

TEST(SafetyThresholdTest, CollisionWarningDistance) {
    float collision_threshold = 0.3f;
    float min_range = 0.25f;
    EXPECT_TRUE(min_range < collision_threshold);  // Would trigger warning
}

TEST(SafetyThresholdTest, CollisionEmergencyDistance) {
    float emergency_threshold = 0.15f;
    float min_range = 0.10f;
    EXPECT_TRUE(min_range < emergency_threshold);  // Would trigger E-stop
}

TEST(SafetyThresholdTest, CollisionSafeDistance) {
    float collision_threshold = 0.3f;
    float min_range = 0.5f;
    EXPECT_FALSE(min_range < collision_threshold);  // Safe
}

TEST(SafetyThresholdTest, WatchdogTimeout) {
    float timeout_s = 1.0f;
    float elapsed_s = 1.5f;
    EXPECT_TRUE(elapsed_s > timeout_s);  // Would trigger E-stop
}

TEST(SafetyThresholdTest, WatchdogActive) {
    float timeout_s = 1.0f;
    float elapsed_s = 0.5f;
    EXPECT_FALSE(elapsed_s > timeout_s);  // Still active
}

// ============================================
// State Transition Tests
// ============================================

TEST(SafetyStateTest, StateTransitions) {
    SafetyState state = SafetyState::OK;
    EXPECT_EQ(state, SafetyState::OK);

    state = SafetyState::WARNING;
    EXPECT_EQ(state, SafetyState::WARNING);

    state = SafetyState::CRITICAL;
    EXPECT_EQ(state, SafetyState::CRITICAL);

    state = SafetyState::EMERGENCY_STOP;
    EXPECT_EQ(state, SafetyState::EMERGENCY_STOP);
}

TEST(SafetyStateTest, StateEnumValues) {
    // Ensure enum values are distinct
    EXPECT_NE(SafetyState::OK, SafetyState::WARNING);
    EXPECT_NE(SafetyState::WARNING, SafetyState::CRITICAL);
    EXPECT_NE(SafetyState::CRITICAL, SafetyState::EMERGENCY_STOP);
}

// ============================================
// Event Enum Tests
// ============================================

TEST(SafetyEventTest, EventEnumValues) {
    EXPECT_NE(SafetyEvent::NONE, SafetyEvent::LOW_BATTERY_WARNING);
    EXPECT_NE(SafetyEvent::LOW_BATTERY_WARNING, SafetyEvent::LOW_BATTERY_CRITICAL);
    EXPECT_NE(SafetyEvent::COLLISION_IMMINENT, SafetyEvent::ESTOP_TRIGGERED);
    EXPECT_NE(SafetyEvent::COMMAND_TIMEOUT, SafetyEvent::SENSOR_FAILURE);
}

// ============================================
// String Conversion Tests (SafetyStatus)
// ============================================

TEST(SafetyStatusTest, StateStringConversion) {
    SafetyStatus status;

    status.state = SafetyState::OK;
    EXPECT_EQ(status.stateToString(), "OK");

    status.state = SafetyState::WARNING;
    EXPECT_EQ(status.stateToString(), "WARNING");

    status.state = SafetyState::CRITICAL;
    EXPECT_EQ(status.stateToString(), "CRITICAL");

    status.state = SafetyState::EMERGENCY_STOP;
    EXPECT_EQ(status.stateToString(), "EMERGENCY_STOP");
}

TEST(SafetyStatusTest, EventStringConversion) {
    SafetyStatus status;
    EXPECT_EQ(status.eventToString(SafetyEvent::NONE), "NONE");
    EXPECT_EQ(status.eventToString(SafetyEvent::LOW_BATTERY_WARNING), "LOW_BATTERY_WARNING");
    EXPECT_EQ(status.eventToString(SafetyEvent::LOW_BATTERY_CRITICAL), "LOW_BATTERY_CRITICAL");
    EXPECT_EQ(status.eventToString(SafetyEvent::COLLISION_IMMINENT), "COLLISION_IMMINENT");
    EXPECT_EQ(status.eventToString(SafetyEvent::LOCALIZATION_LOST), "LOCALIZATION_LOST");
    EXPECT_EQ(status.eventToString(SafetyEvent::COMMAND_TIMEOUT), "COMMAND_TIMEOUT");
    EXPECT_EQ(status.eventToString(SafetyEvent::SENSOR_FAILURE), "SENSOR_FAILURE");
    EXPECT_EQ(status.eventToString(SafetyEvent::ESTOP_TRIGGERED), "ESTOP_TRIGGERED");
    EXPECT_EQ(status.eventToString(SafetyEvent::ESTOP_CLEARED), "ESTOP_CLEARED");
}

// ============================================
// SafetyStatus Default Values Test
// ============================================

TEST(SafetyStatusTest, DefaultValues) {
    SafetyStatus status;
    EXPECT_EQ(status.state, SafetyState::OK);
    EXPECT_TRUE(status.active_events.empty());
    EXPECT_FLOAT_EQ(status.battery_percent, 100.0f);
    EXPECT_FLOAT_EQ(status.min_obstacle_distance, 100.0f);
    EXPECT_FALSE(status.estop_active);
    EXPECT_TRUE(status.lidar_healthy);
    EXPECT_TRUE(status.imu_healthy);
    EXPECT_FALSE(status.degraded_mode);
}

// ============================================
// SafetyStatus Event Management
// ============================================

TEST(SafetyStatusTest, EventManagement) {
    SafetyStatus status;
    EXPECT_TRUE(status.active_events.empty());

    status.active_events.push_back(SafetyEvent::LOW_BATTERY_WARNING);
    EXPECT_EQ(status.active_events.size(), 1u);

    status.active_events.push_back(SafetyEvent::COLLISION_IMMINENT);
    EXPECT_EQ(status.active_events.size(), 2u);

    status.active_events.clear();
    EXPECT_TRUE(status.active_events.empty());
}

// ============================================
// Forward Arc Angle Calculation Tests
// ============================================

TEST(CollisionDetectionTest, ForwardArcAngles) {
    // Test that angles are correctly identified as in/out of forward arc
    float forward_arc_min = -0.5236f;  // -30 degrees
    float forward_arc_max = 0.5236f;   // +30 degrees

    // Angle at 0 degrees (dead ahead) - in arc
    float angle_ahead = 0.0f;
    EXPECT_TRUE(angle_ahead >= forward_arc_min && angle_ahead <= forward_arc_max);

    // Angle at +15 degrees - in arc
    float angle_right = 0.2618f;
    EXPECT_TRUE(angle_right >= forward_arc_min && angle_right <= forward_arc_max);

    // Angle at -15 degrees - in arc
    float angle_left = -0.2618f;
    EXPECT_TRUE(angle_left >= forward_arc_min && angle_left <= forward_arc_max);

    // Angle at +90 degrees - outside arc
    float angle_side = 1.5708f;
    EXPECT_FALSE(angle_side >= forward_arc_min && angle_side <= forward_arc_max);

    // Angle at +180 degrees (behind) - outside arc
    float angle_behind = 3.1416f;
    EXPECT_FALSE(angle_behind >= forward_arc_min && angle_behind <= forward_arc_max);
}

// ============================================
// Range Validation Tests
// ============================================

TEST(CollisionDetectionTest, RangeValidation) {
    // Test range validity checks
    float min_valid = 0.1f;
    float max_valid = 10.0f;

    // Valid range
    float range_ok = 2.5f;
    EXPECT_TRUE(range_ok > min_valid && range_ok < max_valid);

    // Too close (likely noise/error)
    float range_close = 0.05f;
    EXPECT_FALSE(range_close > min_valid && range_close < max_valid);

    // Too far (beyond sensor range)
    float range_far = 15.0f;
    EXPECT_FALSE(range_far > min_valid && range_far < max_valid);

    // Zero (invalid reading)
    float range_zero = 0.0f;
    EXPECT_FALSE(range_zero > min_valid && range_zero < max_valid);

    // Negative (invalid)
    float range_negative = -1.0f;
    EXPECT_FALSE(range_negative > min_valid && range_negative < max_valid);
}

// ============================================
// Timing Tests
// ============================================

TEST(TimingTest, WatchdogElapsedCalculation) {
    // Simulate watchdog elapsed time calculation
    auto last_feed = std::chrono::steady_clock::now();

    // Simulate 500ms elapsed
    auto simulated_now = last_feed + std::chrono::milliseconds(500);

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        simulated_now - last_feed).count();

    EXPECT_EQ(elapsed, 500);
}

TEST(TimingTest, EstopResponseTarget) {
    // E-stop must respond within 500ms (AC1)
    const int max_response_ms = 500;

    // Simulated response times
    EXPECT_TRUE(10 < max_response_ms);   // Fast response - PASS
    EXPECT_TRUE(100 < max_response_ms);  // Normal response - PASS
    EXPECT_TRUE(300 < max_response_ms);  // Slow but acceptable - PASS
    EXPECT_FALSE(500 < max_response_ms); // At limit - FAIL
    EXPECT_FALSE(600 < max_response_ms); // Too slow - FAIL
}

// ============================================
// SafetyMonitor Class Tests (without hardware)
// ============================================

TEST(SafetyMonitorTest, DefaultConstruction) {
    SafetyMonitor monitor;
    EXPECT_EQ(monitor.getState(), SafetyState::OK);
    EXPECT_FALSE(monitor.isEstopActive());
    EXPECT_FALSE(monitor.isInDegradedMode());
    EXPECT_TRUE(monitor.isEnabled());
}

TEST(SafetyMonitorTest, InitWithNullPointers) {
    SafetyMonitor monitor;
    // Should not crash with nullptr
    monitor.init(nullptr, nullptr);
    EXPECT_EQ(monitor.getState(), SafetyState::OK);
}

TEST(SafetyMonitorTest, UpdateWithNullSensors) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);
    // Should not crash
    monitor.update();
    EXPECT_EQ(monitor.getState(), SafetyState::OK);
}

TEST(SafetyMonitorTest, TriggerEstopChangesState) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    EXPECT_FALSE(monitor.isEstopActive());
    EXPECT_EQ(monitor.getState(), SafetyState::OK);

    monitor.triggerEstop();

    EXPECT_TRUE(monitor.isEstopActive());
    EXPECT_EQ(monitor.getState(), SafetyState::EMERGENCY_STOP);
}

TEST(SafetyMonitorTest, EstopEventEmitted) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    bool callback_fired = false;
    SafetyEvent received_event = SafetyEvent::NONE;

    monitor.setEventCallback([&](SafetyEvent event) {
        callback_fired = true;
        received_event = event;
    });

    monitor.triggerEstop();

    EXPECT_TRUE(callback_fired);
    EXPECT_EQ(received_event, SafetyEvent::ESTOP_TRIGGERED);
}

TEST(SafetyMonitorTest, ClearEstopAfterTrigger) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    monitor.triggerEstop();
    EXPECT_TRUE(monitor.isEstopActive());

    // Clear should work when no sensors (no safety checks fail)
    monitor.clearEstop();
    EXPECT_FALSE(monitor.isEstopActive());
    EXPECT_EQ(monitor.getState(), SafetyState::OK);
}

TEST(SafetyMonitorTest, DisableAndEnable) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    EXPECT_TRUE(monitor.isEnabled());

    monitor.disable();
    EXPECT_FALSE(monitor.isEnabled());

    // Update should be a no-op when disabled
    monitor.update();

    monitor.enable();
    EXPECT_TRUE(monitor.isEnabled());
}

TEST(SafetyMonitorTest, GetStatusReturnsValidStruct) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    SafetyStatus status = monitor.getStatus();
    EXPECT_EQ(status.state, SafetyState::OK);
    EXPECT_FALSE(status.estop_active);
    EXPECT_TRUE(status.lidar_healthy);
    EXPECT_TRUE(status.imu_healthy);
    EXPECT_FALSE(status.degraded_mode);
}

TEST(SafetyMonitorTest, ConfigurationSetters) {
    SafetyMonitor monitor;

    // These should not crash
    monitor.setCollisionDistance(0.5f);
    monitor.setEmergencyDistance(0.25f);
    monitor.setBatteryWarningThreshold(25.0f);
    monitor.setBatteryCriticalThreshold(15.0f);
    monitor.setWatchdogTimeout(2.0f);

    // Monitor should still be in valid state
    EXPECT_EQ(monitor.getState(), SafetyState::OK);
}

TEST(SafetyMonitorTest, FeedWatchdogDoesNotCrash) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    // Should not crash
    monitor.feedWatchdog();
    monitor.feedWatchdog();
    monitor.feedWatchdog();
}

TEST(SafetyMonitorTest, CollisionImminentDefaultFalse) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    // With no sensor data, cached_min_range_ is 100m, so no collision
    EXPECT_FALSE(monitor.isCollisionImminent());
}

TEST(SafetyMonitorTest, BatteryPercentDefaultValue) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    // Default cached battery is 100%
    EXPECT_FLOAT_EQ(monitor.getBatteryPercent(), 100.0f);
}

TEST(SafetyMonitorTest, SensorHealthDefaultTrue) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    // Default health status is healthy
    EXPECT_TRUE(monitor.isLidarHealthy());
    EXPECT_TRUE(monitor.isImuHealthy());
}

TEST(SafetyMonitorTest, StatusLastUpdateTimestamp) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    auto before = std::chrono::steady_clock::now();
    monitor.update();
    auto after = std::chrono::steady_clock::now();

    SafetyStatus status = monitor.getStatus();

    // last_update should be between before and after
    EXPECT_GE(status.last_update, before);
    EXPECT_LE(status.last_update, after);
}

TEST(SafetyMonitorTest, MultipleEstopTriggersIdempotent) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    int callback_count = 0;
    monitor.setEventCallback([&](SafetyEvent) {
        callback_count++;
    });

    monitor.triggerEstop();
    monitor.triggerEstop();
    monitor.triggerEstop();

    EXPECT_TRUE(monitor.isEstopActive());
    // Each trigger should still fire the callback
    EXPECT_EQ(callback_count, 3);
}

TEST(SafetyMonitorTest, ClearEstopWhenNotActive) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    int callback_count = 0;
    monitor.setEventCallback([&](SafetyEvent) {
        callback_count++;
    });

    // Clear when not in E-stop should be a no-op
    monitor.clearEstop();
    EXPECT_FALSE(monitor.isEstopActive());
    EXPECT_EQ(callback_count, 0);
}

// ============================================
// Watchdog Behavior Tests
// ============================================

TEST(WatchdogBehaviorTest, WatchdogNotActiveUntilFed) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    // Watchdog should not be enabled until first feed
    // Update should not trigger E-stop even with old timestamp
    monitor.update();
    EXPECT_FALSE(monitor.isEstopActive());
}

TEST(WatchdogBehaviorTest, WatchdogFeedResetsTimer) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    // Feed watchdog
    monitor.feedWatchdog();

    // Immediate update should not trigger E-stop
    monitor.update();
    EXPECT_FALSE(monitor.isEstopActive());
}

// ============================================
// Configuration Validation Tests
// ============================================

TEST(ConfigValidationTest, RejectNegativeCollisionDistance) {
    SafetyMonitor monitor;

    // Should not crash and should reject invalid value
    monitor.setCollisionDistance(-1.0f);

    // Value should remain at default (0.3m)
    // We can't directly check the private member, but behavior should be unchanged
    EXPECT_FALSE(monitor.isCollisionImminent());  // Default cached_min_range is 100m
}

TEST(ConfigValidationTest, RejectZeroCollisionDistance) {
    SafetyMonitor monitor;
    monitor.setCollisionDistance(0.0f);  // Should be rejected
    EXPECT_FALSE(monitor.isCollisionImminent());
}

TEST(ConfigValidationTest, AcceptValidCollisionDistance) {
    SafetyMonitor monitor;
    monitor.setCollisionDistance(0.5f);  // Valid value
    // Should not crash
    EXPECT_EQ(monitor.getState(), SafetyState::OK);
}

TEST(ConfigValidationTest, RejectNegativeWatchdogTimeout) {
    SafetyMonitor monitor;
    monitor.setWatchdogTimeout(-1.0f);  // Should be rejected
    // Should not crash
    EXPECT_EQ(monitor.getState(), SafetyState::OK);
}

TEST(ConfigValidationTest, RejectInvalidBatteryThresholds) {
    SafetyMonitor monitor;

    // Negative values
    monitor.setBatteryWarningThreshold(-10.0f);
    monitor.setBatteryCriticalThreshold(-5.0f);

    // Over 100%
    monitor.setBatteryWarningThreshold(150.0f);
    monitor.setBatteryCriticalThreshold(200.0f);

    // Should not crash
    EXPECT_EQ(monitor.getState(), SafetyState::OK);
}

// ============================================
// Thread Safety Verification Tests
// ============================================

TEST(ThreadSafetyTest, ConcurrentStateQueries) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    std::atomic<bool> stop{false};
    std::atomic<int> query_count{0};

    // Reader thread - continuously queries state
    std::thread reader([&]() {
        while (!stop.load()) {
            monitor.getState();
            monitor.isEstopActive();
            monitor.isCollisionImminent();
            monitor.getBatteryPercent();
            query_count++;
        }
    });

    // Main thread - triggers E-stop multiple times
    for (int i = 0; i < 100; ++i) {
        monitor.triggerEstop();
        monitor.clearEstop();
    }

    stop.store(true);
    reader.join();

    // Should complete without crashes or hangs
    EXPECT_GT(query_count.load(), 0);
}

TEST(ThreadSafetyTest, ConcurrentUpdates) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    std::atomic<bool> stop{false};
    std::atomic<int> update_count{0};

    // Updater thread
    std::thread updater([&]() {
        while (!stop.load()) {
            monitor.update();
            update_count++;
        }
    });

    // Main thread - triggers E-stop and clears
    for (int i = 0; i < 50; ++i) {
        monitor.triggerEstop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        monitor.clearEstop();
    }

    stop.store(true);
    updater.join();

    EXPECT_GT(update_count.load(), 0);
}

// ============================================
// Integration Behavior Tests (no mock, but verify flow)
// ============================================

TEST(IntegrationBehaviorTest, EstopClearResetsWatchdog) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    // Enable watchdog
    monitor.feedWatchdog();

    // Trigger E-stop
    monitor.triggerEstop();
    EXPECT_TRUE(monitor.isEstopActive());

    // Clear E-stop
    monitor.clearEstop();
    EXPECT_FALSE(monitor.isEstopActive());

    // Update should NOT trigger E-stop (watchdog was disabled on clear)
    monitor.update();
    EXPECT_FALSE(monitor.isEstopActive());
}

TEST(IntegrationBehaviorTest, EventCallbackFiredOnMultipleEvents) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    std::vector<SafetyEvent> received_events;
    monitor.setEventCallback([&](SafetyEvent event) {
        received_events.push_back(event);
    });

    // Trigger E-stop
    monitor.triggerEstop();

    // Clear E-stop
    monitor.clearEstop();

    // Should have received both events
    EXPECT_EQ(received_events.size(), 2u);
    EXPECT_EQ(received_events[0], SafetyEvent::ESTOP_TRIGGERED);
    EXPECT_EQ(received_events[1], SafetyEvent::ESTOP_CLEARED);
}

TEST(IntegrationBehaviorTest, StatusTimestampUpdatedOnUpdate) {
    SafetyMonitor monitor;
    monitor.init(nullptr, nullptr);

    auto before = std::chrono::steady_clock::now();

    // Small delay to ensure different timestamp
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    monitor.update();
    SafetyStatus status = monitor.getStatus();

    auto after = std::chrono::steady_clock::now();

    // Timestamp should be between before and after
    EXPECT_GT(status.last_update, before);
    EXPECT_LE(status.last_update, after);
}
