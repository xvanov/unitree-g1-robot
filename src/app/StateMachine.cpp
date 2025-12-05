#include "app/StateMachine.h"
#include <iostream>

std::string StateMachine::getStateString() const {
    switch (state_.load()) {
        case InspectionState::IDLE:             return "IDLE";
        case InspectionState::CALIBRATING:      return "CALIBRATING";
        case InspectionState::INSPECTING:       return "INSPECTING";
        case InspectionState::PAUSED:           return "PAUSED";
        case InspectionState::BLOCKED:          return "BLOCKED";
        case InspectionState::WAITING_OPERATOR: return "WAITING_OPERATOR";
        case InspectionState::COMPLETE:         return "COMPLETE";
        case InspectionState::EMERGENCY_STOP:   return "EMERGENCY_STOP";
        case InspectionState::RETURNING_HOME:   return "RETURNING_HOME";
        default:                                return "UNKNOWN";
    }
}

bool StateMachine::canTransition(InspectionState from, InspectionState to) const {
    // E-stop is always allowed from any state
    if (to == InspectionState::EMERGENCY_STOP) {
        return true;
    }

    // Transition rules per state
    switch (from) {
        case InspectionState::IDLE:
            return to == InspectionState::CALIBRATING;

        case InspectionState::CALIBRATING:
            return to == InspectionState::INSPECTING ||
                   to == InspectionState::IDLE;

        case InspectionState::INSPECTING:
            return to == InspectionState::PAUSED ||
                   to == InspectionState::BLOCKED ||
                   to == InspectionState::COMPLETE ||
                   to == InspectionState::RETURNING_HOME;

        case InspectionState::PAUSED:
            return to == InspectionState::INSPECTING ||
                   to == InspectionState::IDLE;

        case InspectionState::BLOCKED:
            return to == InspectionState::INSPECTING ||
                   to == InspectionState::RETURNING_HOME;

        case InspectionState::WAITING_OPERATOR:
            return to == InspectionState::INSPECTING ||
                   to == InspectionState::IDLE ||
                   to == InspectionState::RETURNING_HOME;

        case InspectionState::COMPLETE:
            return to == InspectionState::IDLE ||
                   to == InspectionState::RETURNING_HOME;

        case InspectionState::EMERGENCY_STOP:
            return to == InspectionState::IDLE;

        case InspectionState::RETURNING_HOME:
            return to == InspectionState::IDLE;

        default:
            return false;
    }
}

bool StateMachine::setState(InspectionState new_state) {
    InspectionState current = state_.load();
    if (!canTransition(current, new_state)) {
        std::cerr << "[STATE] Invalid transition: " << getStateString()
                  << " -> " << static_cast<int>(new_state) << std::endl;
        return false;
    }

    state_.store(new_state);
    std::cout << "[STATE] " << getStateString() << std::endl;
    return true;
}

bool StateMachine::startInspection() {
    InspectionState current = state_.load();

    if (current == InspectionState::IDLE) {
        // Start from IDLE -> CALIBRATING
        return setState(InspectionState::CALIBRATING);
    }

    return false;
}

bool StateMachine::setCalibrated() {
    InspectionState current = state_.load();

    if (current == InspectionState::CALIBRATING) {
        return setState(InspectionState::INSPECTING);
    }

    return false;
}

bool StateMachine::pause() {
    InspectionState current = state_.load();

    if (current == InspectionState::INSPECTING) {
        return setState(InspectionState::PAUSED);
    }

    return false;
}

bool StateMachine::resume() {
    InspectionState current = state_.load();

    if (current == InspectionState::PAUSED) {
        return setState(InspectionState::INSPECTING);
    }

    return false;
}

bool StateMachine::stop() {
    InspectionState current = state_.load();

    // Can stop from multiple states - need to bypass canTransition for RETURNING_HOME
    if (current == InspectionState::INSPECTING ||
        current == InspectionState::PAUSED ||
        current == InspectionState::CALIBRATING ||
        current == InspectionState::BLOCKED ||
        current == InspectionState::WAITING_OPERATOR ||
        current == InspectionState::COMPLETE ||
        current == InspectionState::RETURNING_HOME) {
        // Direct state change for stop() since canTransition is strict
        state_.store(InspectionState::IDLE);
        std::cout << "[STATE] IDLE" << std::endl;
        return true;
    }

    return current == InspectionState::IDLE;  // Already stopped
}

void StateMachine::emergencyStop() {
    // E-stop is always allowed
    state_.store(InspectionState::EMERGENCY_STOP);
    std::cout << "[STATE] EMERGENCY_STOP" << std::endl;
}

bool StateMachine::clearEstop() {
    InspectionState current = state_.load();

    if (current == InspectionState::EMERGENCY_STOP) {
        return setState(InspectionState::IDLE);
    }

    return false;
}

bool StateMachine::startReturningHome() {
    InspectionState current = state_.load();

    // Can return home from INSPECTING, BLOCKED, COMPLETE, or WAITING_OPERATOR
    if (current == InspectionState::INSPECTING ||
        current == InspectionState::BLOCKED ||
        current == InspectionState::COMPLETE ||
        current == InspectionState::WAITING_OPERATOR) {
        return setState(InspectionState::RETURNING_HOME);
    }

    return false;
}

bool StateMachine::setBlocked(bool blocked) {
    InspectionState current = state_.load();

    if (blocked) {
        if (current == InspectionState::INSPECTING) {
            was_blocked_ = true;
            return setState(InspectionState::BLOCKED);
        }
    } else {
        if (current == InspectionState::BLOCKED) {
            was_blocked_ = false;
            return setState(InspectionState::INSPECTING);
        }
    }

    return false;
}

bool StateMachine::setComplete() {
    InspectionState current = state_.load();

    if (current == InspectionState::INSPECTING) {
        return setState(InspectionState::COMPLETE);
    }

    return false;
}

void StateMachine::setWaypointProgress(int completed, int total) {
    completed_waypoints_ = completed;
    total_waypoints_ = total;
}

float StateMachine::getCompletionPercent() const {
    if (total_waypoints_ == 0) {
        return 0.0f;
    }
    return 100.0f * static_cast<float>(completed_waypoints_) / static_cast<float>(total_waypoints_);
}

void StateMachine::update(float dt) {
    (void)dt;  // Currently unused, but available for future time-based transitions
}
