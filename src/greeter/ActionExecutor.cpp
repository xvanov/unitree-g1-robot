#include "ActionExecutor.h"
#include "locomotion/LocoController.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace greeter {

ActionExecutor::ActionExecutor(std::shared_ptr<LocoController> loco)
    : loco_(loco) {
}

ActionExecutor::~ActionExecutor() {
    if (in_progress_ && loco_) {
        loco_->stop();
    }
}

void ActionExecutor::setPostPosition(float x, float y, float theta) {
    post_x_ = x;
    post_y_ = y;
    post_theta_ = theta;
}

void ActionExecutor::resetPosition(float x, float y, float theta) {
    current_x_ = x;
    current_y_ = y;
    current_theta_ = theta;
}

void ActionExecutor::logAction(const ParsedAction& action, const std::string& result) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::cout << "[ACTION] "
              << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
              << " | " << ActionParser::actionTypeToString(action.type)
              << " | intent=" << action.intent
              << " | confidence=" << action.confidence
              << " | result=" << result
              << std::endl;

    // For PUSH_FORWARD, extra explicit logging
    if (action.type == ActionType::PUSH_FORWARD) {
        std::cerr << "[PUSH_ALERT] Force=" << action.force_level
                  << " Reasoning: " << action.reasoning << std::endl;
    }
}

bool ActionExecutor::execute(const ParsedAction& action) {
    // Check robot readiness
    if (!loco_) {
        logAction(action, "FAILED: No locomotion controller");
        return false;
    }

    if (!loco_->isReady()) {
        logAction(action, "FAILED: Robot not ready");
        return false;
    }

    // Note: All G1 models (23/29/36 DOF) have arms for wave/shake gestures
    // Only 29+ DOF models have dexterous hand grippers for grip actions

    // Store action for tracking
    current_action_ = action;

    // Dispatch to specific handler
    bool started = false;
    switch (action.type) {
        case ActionType::MOVE_FORWARD:
        case ActionType::MOVE_BACKWARD:
        case ActionType::ROTATE:
        case ActionType::MOVE_TO:
        case ActionType::FOLLOW:
            started = executeMovement(action);
            break;
        case ActionType::WAVE_HAND:
        case ActionType::SHAKE_HAND:
        case ActionType::BOW:
            started = executeGesture(action);
            break;
        case ActionType::STAND_UP:
        case ActionType::SIT_DOWN:
            started = executePosture(action);
            break;
        case ActionType::PUSH_FORWARD:
            started = executePush(action);
            break;
        case ActionType::RETURN_TO_POST:
            started = executeReturnToPost(action);
            break;
        case ActionType::SPEAK:
            started = executeSpeak(action);
            break;
        case ActionType::STOP:
            loco_->stop();
            logAction(action, "STOP_EXECUTED");
            return true;  // Immediate, no tracking needed
        case ActionType::WAIT:
        case ActionType::NO_ACTION:
            logAction(action, "NO_OP");
            return true;
        default:
            logAction(action, "FAILED: Unknown action type");
            return false;
    }

    if (!started) {
        logAction(action, "FAILED: Action could not start");
    }
    return started;
}

bool ActionExecutor::executeMovement(const ParsedAction& action) {
    float vx = 0.0f, vy = 0.0f, omega = 0.0f;
    float duration = 0.0f;

    switch (action.type) {
        case ActionType::MOVE_FORWARD:
            vx = ActionTiming::MOVEMENT_VELOCITY;
            duration = action.distance / ActionTiming::MOVEMENT_VELOCITY;
            logAction(action, "MOVE_FORWARD_INITIATED: distance=" + std::to_string(action.distance));
            break;

        case ActionType::MOVE_BACKWARD:
            vx = -ActionTiming::MOVEMENT_VELOCITY;
            duration = action.distance / ActionTiming::MOVEMENT_VELOCITY;
            logAction(action, "MOVE_BACKWARD_INITIATED: distance=" + std::to_string(action.distance));
            break;

        case ActionType::ROTATE:
            // Convert degrees to radians for duration calculation
            // omega is in rad/s, angle is in degrees
            omega = (action.angle > 0) ? ActionTiming::ROTATION_VELOCITY : -ActionTiming::ROTATION_VELOCITY;
            duration = std::abs(action.angle) / (ActionTiming::ROTATION_VELOCITY * 180.0f / static_cast<float>(M_PI));
            logAction(action, "ROTATE_INITIATED: angle=" + std::to_string(action.angle) + " deg");
            break;

        case ActionType::MOVE_TO: {
            // Calculate distance and angle to target
            float dx = action.x - current_x_;
            float dy = action.y - current_y_;
            float dist = std::sqrt(dx*dx + dy*dy);
            // For MVP, just move forward the required distance (assumes facing target)
            vx = ActionTiming::MOVEMENT_VELOCITY;
            duration = dist / ActionTiming::MOVEMENT_VELOCITY;
            logAction(action, "MOVE_TO_INITIATED: x=" + std::to_string(action.x) +
                     " y=" + std::to_string(action.y));
            break;
        }

        case ActionType::FOLLOW:
            // Follow is continuous - set moderate forward velocity
            vx = ActionTiming::MOVEMENT_VELOCITY * 0.5f;  // Slower for following
            duration = ActionTiming::ACTION_TIMEOUT;  // Will be cancelled when target changes
            logAction(action, "FOLLOW_INITIATED: target=" + action.target_id);
            break;

        default:
            return false;
    }

    if (!loco_->setVelocity(vx, vy, omega)) {
        logAction(action, "FAILED: Could not set velocity");
        return false;
    }

    target_duration_ = duration;
    elapsed_time_ = 0.0f;
    in_progress_ = true;
    progress_ = 0.0f;

    return true;
}

bool ActionExecutor::executeGesture(const ParsedAction& action) {
    switch (action.type) {
        case ActionType::WAVE_HAND:
            logAction(action, "WAVE_HAND_INITIATED");
            if (!loco_->waveHand(false)) {  // Use right hand
                logAction(action, "FAILED: waveHand() returned false");
                return false;
            }
            target_duration_ = ActionTiming::WAVE_DURATION;
            break;

        case ActionType::SHAKE_HAND:
            logAction(action, "SHAKE_HAND_INITIATED");
            if (!loco_->shakeHand(-1)) {  // Full sequence
                logAction(action, "FAILED: shakeHand() returned false");
                return false;
            }
            target_duration_ = ActionTiming::SHAKE_HAND_DURATION;
            break;

        case ActionType::BOW:
            logAction(action, "BOW_INITIATED");
            // BOW is custom multi-phase - start lean forward
            bow_phase_ = BowPhase::LEAN_FORWARD;
            phase_elapsed_ = 0.0f;
            // Small forward velocity creates lean effect
            loco_->setVelocity(0.1f, 0, 0);
            target_duration_ = ActionTiming::BOW_LEAN_DURATION +
                              ActionTiming::BOW_HOLD_DURATION +
                              ActionTiming::BOW_RETURN_DURATION;
            break;

        default:
            return false;
    }

    elapsed_time_ = 0.0f;
    in_progress_ = true;
    progress_ = 0.0f;

    return true;
}

bool ActionExecutor::executePosture(const ParsedAction& action) {
    switch (action.type) {
        case ActionType::STAND_UP:
            logAction(action, "STAND_UP_INITIATED");
            if (!loco_->standUp()) {
                logAction(action, "FAILED: standUp() returned false");
                return false;
            }
            target_duration_ = ActionTiming::STAND_UP_DURATION;
            break;

        case ActionType::SIT_DOWN:
            logAction(action, "SIT_DOWN_INITIATED");
            if (!loco_->sitDown()) {
                logAction(action, "FAILED: sitDown() returned false");
                return false;
            }
            target_duration_ = ActionTiming::SIT_DOWN_DURATION;
            break;

        default:
            return false;
    }

    elapsed_time_ = 0.0f;
    in_progress_ = true;
    progress_ = 0.0f;

    return true;
}

bool ActionExecutor::executePush(const ParsedAction& action) {
    // CRITICAL: Log push command explicitly for analysis
    logAction(action, "PUSH_FORWARD_INITIATED");

    // Scale velocity by force_level (0.0-1.0)
    // Architecture specifies: setVelocity(0.4*force, 0, 0)
    float vx = 0.4f * action.force_level;

    // Clamp to safety limits (still applies, but push goes through)
    vx = std::min(vx, SafetyLimits::MAX_VX);

    // NOTE: No arm extension on 23-DOF EDU model
    // If arms available, extend before pushing
    // (Future: implement arm extension sequence)

    // Execute forward movement
    if (!loco_->setVelocity(vx, 0, 0)) {
        logAction(action, "PUSH_FORWARD_FAILED: Robot not ready");
        return false;
    }

    // Duration: 1 second as per architecture
    target_duration_ = ActionTiming::PUSH_DURATION;
    elapsed_time_ = 0.0f;
    in_progress_ = true;
    progress_ = 0.0f;

    logAction(action, "PUSH_FORWARD_EXECUTING: force=" +
              std::to_string(action.force_level) + " vx=" + std::to_string(vx));

    return true;
}

bool ActionExecutor::executeSpeak(const ParsedAction& action) {
    // For MVP: Just log the speech text
    // In production: Integrate with TTS system
    logAction(action, "SPEAK_INITIATED: \"" + action.text + "\"");

    std::cout << "[SPEECH] " << action.text << std::endl;

    // Speech is immediate for MVP
    return true;
}

bool ActionExecutor::executeReturnToPost(const ParsedAction& action) {
    logAction(action, "RETURN_TO_POST_INITIATED");

    // Calculate distance and angle to post position
    float dx = post_x_ - current_x_;
    float dy = post_y_ - current_y_;
    float distance = std::sqrt(dx*dx + dy*dy);
    float target_angle = std::atan2(dy, dx) * 180.0f / static_cast<float>(M_PI);
    float rotation_needed = target_angle - current_theta_;

    // Normalize rotation to [-180, 180]
    while (rotation_needed > 180.0f) rotation_needed -= 360.0f;
    while (rotation_needed < -180.0f) rotation_needed += 360.0f;

    // Store for phase updates
    return_rotation_needed_ = rotation_needed;
    return_distance_needed_ = distance;
    return_final_rotation_ = post_theta_ - target_angle;
    while (return_final_rotation_ > 180.0f) return_final_rotation_ -= 360.0f;
    while (return_final_rotation_ < -180.0f) return_final_rotation_ += 360.0f;

    // Start rotation phase first
    return_to_post_phase_ = ReturnPhase::ROTATE_TO_FACE;
    phase_elapsed_ = 0.0f;

    // Duration estimate: rotation + movement + final rotation
    float rotation_time = std::abs(rotation_needed) / (ActionTiming::ROTATION_VELOCITY * 180.0f / static_cast<float>(M_PI));
    float movement_time = distance / ActionTiming::MOVEMENT_VELOCITY;
    float final_rotation_time = std::abs(return_final_rotation_) / (ActionTiming::ROTATION_VELOCITY * 180.0f / static_cast<float>(M_PI));
    target_duration_ = rotation_time + movement_time + final_rotation_time;
    elapsed_time_ = 0.0f;
    in_progress_ = true;
    progress_ = 0.0f;

    // Start rotation
    if (std::abs(rotation_needed) > 1.0f) {  // More than 1 degree
        float omega = (rotation_needed > 0) ? ActionTiming::ROTATION_VELOCITY : -ActionTiming::ROTATION_VELOCITY;
        if (!loco_->setVelocity(0, 0, omega)) {
            logAction(action, "RETURN_TO_POST_FAILED: Could not start rotation");
            in_progress_ = false;
            return false;
        }
    } else {
        // Skip rotation, go directly to movement
        return_to_post_phase_ = ReturnPhase::MOVE_TO_POST;
        if (!loco_->setVelocity(ActionTiming::MOVEMENT_VELOCITY, 0, 0)) {
            logAction(action, "RETURN_TO_POST_FAILED: Could not start movement");
            in_progress_ = false;
            return false;
        }
    }

    return true;
}

void ActionExecutor::update(float dt) {
    if (!in_progress_) return;

    elapsed_time_ += dt;
    progress_ = std::min(1.0f, elapsed_time_ / target_duration_);

    // Update position tracking (for RETURN_TO_POST)
    updatePosition(dt);

    // Handle action-specific updates
    switch (current_action_.type) {
        case ActionType::BOW:
            updateBow(dt);  // Multi-phase bow handling
            return;  // BOW handles its own completion

        case ActionType::RETURN_TO_POST:
            updateReturnToPost(dt);  // Multi-phase return handling
            return;

        default:
            // Simple duration-based actions
            if (elapsed_time_ >= target_duration_) {
                loco_->stop();
                completeAction(true);
            }
            break;
    }
}

void ActionExecutor::updateBow(float dt) {
    phase_elapsed_ += dt;

    switch (bow_phase_) {
        case BowPhase::LEAN_FORWARD:
            if (phase_elapsed_ >= ActionTiming::BOW_LEAN_DURATION) {
                // Transition to hold
                loco_->stop();
                bow_phase_ = BowPhase::HOLD;
                phase_elapsed_ = 0.0f;
            }
            break;

        case BowPhase::HOLD:
            if (phase_elapsed_ >= ActionTiming::BOW_HOLD_DURATION) {
                // Transition to return upright
                loco_->setVelocity(-0.1f, 0, 0);  // Lean back
                bow_phase_ = BowPhase::RETURN_UPRIGHT;
                phase_elapsed_ = 0.0f;
            }
            break;

        case BowPhase::RETURN_UPRIGHT:
            if (phase_elapsed_ >= ActionTiming::BOW_RETURN_DURATION) {
                loco_->stop();
                bow_phase_ = BowPhase::NONE;
                completeAction(true);
            }
            break;

        default:
            break;
    }
}

void ActionExecutor::updateReturnToPost(float dt) {
    phase_elapsed_ += dt;

    switch (return_to_post_phase_) {
        case ReturnPhase::ROTATE_TO_FACE: {
            float rotation_time = std::abs(return_rotation_needed_) /
                                 (ActionTiming::ROTATION_VELOCITY * 180.0f / static_cast<float>(M_PI));
            if (phase_elapsed_ >= rotation_time) {
                // Transition to movement
                loco_->setVelocity(ActionTiming::MOVEMENT_VELOCITY, 0, 0);
                return_to_post_phase_ = ReturnPhase::MOVE_TO_POST;
                phase_elapsed_ = 0.0f;
            }
            break;
        }

        case ReturnPhase::MOVE_TO_POST: {
            float movement_time = return_distance_needed_ / ActionTiming::MOVEMENT_VELOCITY;
            if (phase_elapsed_ >= movement_time) {
                // Transition to final rotation
                if (std::abs(return_final_rotation_) > 1.0f) {
                    float omega = (return_final_rotation_ > 0) ?
                                  ActionTiming::ROTATION_VELOCITY : -ActionTiming::ROTATION_VELOCITY;
                    loco_->setVelocity(0, 0, omega);
                    return_to_post_phase_ = ReturnPhase::ROTATE_TO_ORIENT;
                } else {
                    // Done
                    loco_->stop();
                    return_to_post_phase_ = ReturnPhase::NONE;
                    completeAction(true);
                    return;
                }
                phase_elapsed_ = 0.0f;
            }
            break;
        }

        case ReturnPhase::ROTATE_TO_ORIENT: {
            float final_rotation_time = std::abs(return_final_rotation_) /
                                       (ActionTiming::ROTATION_VELOCITY * 180.0f / static_cast<float>(M_PI));
            if (phase_elapsed_ >= final_rotation_time) {
                loco_->stop();
                return_to_post_phase_ = ReturnPhase::NONE;
                // Reset position to post position (we're there now)
                current_x_ = post_x_;
                current_y_ = post_y_;
                current_theta_ = post_theta_;
                completeAction(true);
            }
            break;
        }

        default:
            break;
    }
}

void ActionExecutor::updatePosition(float dt) {
    // Simple dead reckoning for MVP
    // In production, use robot odometry or SLAM
    if (current_action_.type == ActionType::MOVE_FORWARD) {
        current_x_ += ActionTiming::MOVEMENT_VELOCITY * dt * std::cos(current_theta_ * static_cast<float>(M_PI) / 180.0f);
        current_y_ += ActionTiming::MOVEMENT_VELOCITY * dt * std::sin(current_theta_ * static_cast<float>(M_PI) / 180.0f);
    } else if (current_action_.type == ActionType::MOVE_BACKWARD) {
        current_x_ -= ActionTiming::MOVEMENT_VELOCITY * dt * std::cos(current_theta_ * static_cast<float>(M_PI) / 180.0f);
        current_y_ -= ActionTiming::MOVEMENT_VELOCITY * dt * std::sin(current_theta_ * static_cast<float>(M_PI) / 180.0f);
    } else if (current_action_.type == ActionType::ROTATE) {
        float omega = (current_action_.angle > 0) ? ActionTiming::ROTATION_VELOCITY : -ActionTiming::ROTATION_VELOCITY;
        current_theta_ += omega * 180.0f / static_cast<float>(M_PI) * dt;
    } else if (current_action_.type == ActionType::PUSH_FORWARD) {
        // Push also moves forward
        float vx = 0.4f * current_action_.force_level;
        current_x_ += vx * dt * std::cos(current_theta_ * static_cast<float>(M_PI) / 180.0f);
        current_y_ += vx * dt * std::sin(current_theta_ * static_cast<float>(M_PI) / 180.0f);
    }
}

void ActionExecutor::completeAction(bool success) {
    in_progress_ = false;
    progress_ = success ? 1.0f : progress_;
    if (loco_) {
        loco_->stop();
    }

    logAction(current_action_, success ? "COMPLETED" : "FAILED");

    if (completion_callback_) {
        completion_callback_(success);
    }
}

void ActionExecutor::cancel() {
    if (in_progress_) {
        if (loco_) {
            loco_->stop();
        }
        logAction(current_action_, "CANCELLED");
        completeAction(false);
    }
}

void ActionExecutor::emergencyStop() {
    if (loco_) {
        loco_->emergencyStop();
    }
    if (in_progress_) {
        logAction(current_action_, "EMERGENCY_STOP");
        in_progress_ = false;
        progress_ = 0.0f;
    }
}

}  // namespace greeter
