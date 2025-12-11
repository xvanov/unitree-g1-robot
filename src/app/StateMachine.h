#pragma once

#include <string>
#include <atomic>

enum class InspectionState {
    IDLE,
    CALIBRATING,
    INSPECTING,
    PAUSED,
    BLOCKED,
    WAITING_OPERATOR,
    COMPLETE,
    EMERGENCY_STOP,
    RETURNING_HOME
};

class StateMachine {
public:
    StateMachine() = default;

    // State accessors
    InspectionState getState() const { return state_.load(); }
    std::string getStateString() const;

    // State transitions
    bool startInspection();
    bool pause();
    bool resume();
    bool stop();
    void emergencyStop();
    bool clearEstop();
    bool startReturningHome();

    // Condition-based transitions
    bool setBlocked(bool blocked);
    bool setComplete();
    bool setCalibrated();

    // Progress tracking
    void setWaypointProgress(int completed, int total);
    float getCompletionPercent() const;
    int getCompletedWaypoints() const { return completed_waypoints_; }
    int getTotalWaypoints() const { return total_waypoints_; }

    // Update (called each frame)
    void update(float dt);

private:
    bool canTransition(InspectionState from, InspectionState to) const;
    bool setState(InspectionState new_state);

    std::atomic<InspectionState> state_{InspectionState::IDLE};
    int completed_waypoints_ = 0;
    int total_waypoints_ = 0;
    bool was_blocked_ = false;  // Track if we were blocked before going to INSPECTING
};
