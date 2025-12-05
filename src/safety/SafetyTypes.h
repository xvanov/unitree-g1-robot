#pragma once

#include <chrono>
#include <string>
#include <vector>

enum class SafetyState {
    OK,             // Normal operation
    WARNING,        // Degraded but operational
    CRITICAL,       // Immediate action required
    EMERGENCY_STOP  // Robot halted
};

enum class SafetyEvent {
    NONE,
    LOW_BATTERY_WARNING,      // Battery at warning threshold
    LOW_BATTERY_CRITICAL,     // Battery at critical threshold
    COLLISION_IMMINENT,       // Obstacle detected in path
    LOCALIZATION_LOST,        // Position uncertainty too high
    COMMAND_TIMEOUT,          // No velocity commands received
    SENSOR_FAILURE,           // Sensor disconnected or stale
    ESTOP_TRIGGERED,          // E-stop activated
    ESTOP_CLEARED             // E-stop released
};

struct SafetyStatus {
    SafetyState state = SafetyState::OK;
    std::vector<SafetyEvent> active_events;
    float battery_percent = 100.0f;
    float min_obstacle_distance = 100.0f;  // meters
    bool estop_active = false;
    bool lidar_healthy = true;
    bool imu_healthy = true;
    bool degraded_mode = false;
    std::chrono::steady_clock::time_point last_update;

    // String conversion for logging
    std::string stateToString() const {
        switch (state) {
            case SafetyState::OK: return "OK";
            case SafetyState::WARNING: return "WARNING";
            case SafetyState::CRITICAL: return "CRITICAL";
            case SafetyState::EMERGENCY_STOP: return "EMERGENCY_STOP";
            default: return "UNKNOWN";
        }
    }

    std::string eventToString(SafetyEvent event) const {
        switch (event) {
            case SafetyEvent::NONE: return "NONE";
            case SafetyEvent::LOW_BATTERY_WARNING: return "LOW_BATTERY_WARNING";
            case SafetyEvent::LOW_BATTERY_CRITICAL: return "LOW_BATTERY_CRITICAL";
            case SafetyEvent::COLLISION_IMMINENT: return "COLLISION_IMMINENT";
            case SafetyEvent::LOCALIZATION_LOST: return "LOCALIZATION_LOST";
            case SafetyEvent::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
            case SafetyEvent::SENSOR_FAILURE: return "SENSOR_FAILURE";
            case SafetyEvent::ESTOP_TRIGGERED: return "ESTOP_TRIGGERED";
            case SafetyEvent::ESTOP_CLEARED: return "ESTOP_CLEARED";
            default: return "UNKNOWN";
        }
    }
};
