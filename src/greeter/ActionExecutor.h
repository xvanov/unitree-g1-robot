#pragma once

#include "ActionParser.h"
#include <memory>
#include <functional>
#include <cmath>

class LocoController;

namespace greeter {

// Timing constants for action durations
namespace ActionTiming {
    constexpr float WAVE_DURATION = 2.0f;
    constexpr float BOW_DURATION = 3.0f;
    constexpr float SHAKE_HAND_DURATION = 2.0f;
    constexpr float STAND_UP_DURATION = 3.0f;
    constexpr float SIT_DOWN_DURATION = 3.0f;
    constexpr float PUSH_DURATION = 1.0f;
    constexpr float MOVEMENT_VELOCITY = 0.3f;
    constexpr float ROTATION_VELOCITY = 0.3f;
    constexpr float ACTION_TIMEOUT = 30.0f;

    // BOW phases
    constexpr float BOW_LEAN_DURATION = 1.0f;
    constexpr float BOW_HOLD_DURATION = 1.0f;
    constexpr float BOW_RETURN_DURATION = 1.0f;
}

class ActionExecutor {
public:
    explicit ActionExecutor(std::shared_ptr<LocoController> loco);
    ~ActionExecutor();

    // Execute action (non-blocking, starts action)
    bool execute(const ParsedAction& action);

    // Query state
    bool isActionComplete() const { return !in_progress_; }
    float getProgress() const { return progress_; }

    // Control
    void cancel();
    void emergencyStop();

    // Update (call each frame with delta time in seconds)
    void update(float dt);

    // Completion callback
    using CompletionCallback = std::function<void(bool success)>;
    void setCompletionCallback(CompletionCallback cb) { completion_callback_ = cb; }

    // Store post position for RETURN_TO_POST
    void setPostPosition(float x, float y, float theta);

    // Get current action being executed
    const ParsedAction& getCurrentAction() const { return current_action_; }

    // Reset position tracking (call when at known landmark)
    void resetPosition(float x, float y, float theta);

private:
    std::shared_ptr<LocoController> loco_;
    ParsedAction current_action_;
    bool in_progress_ = false;
    float progress_ = 0.0f;
    float elapsed_time_ = 0.0f;
    float target_duration_ = 0.0f;
    CompletionCallback completion_callback_;

    // Post position for RETURN_TO_POST
    float post_x_ = 0.0f, post_y_ = 0.0f, post_theta_ = 0.0f;

    // Current position tracking (dead reckoning)
    float current_x_ = 0.0f, current_y_ = 0.0f, current_theta_ = 0.0f;

    // BOW gesture state machine
    enum class BowPhase { NONE, LEAN_FORWARD, HOLD, RETURN_UPRIGHT };
    BowPhase bow_phase_ = BowPhase::NONE;
    float phase_elapsed_ = 0.0f;

    // RETURN_TO_POST state machine
    enum class ReturnPhase { NONE, ROTATE_TO_FACE, MOVE_TO_POST, ROTATE_TO_ORIENT };
    ReturnPhase return_to_post_phase_ = ReturnPhase::NONE;
    float return_rotation_needed_ = 0.0f;
    float return_distance_needed_ = 0.0f;
    float return_final_rotation_ = 0.0f;

    // Implementation methods
    bool executeMovement(const ParsedAction& action);
    bool executeGesture(const ParsedAction& action);
    bool executePosture(const ParsedAction& action);
    bool executePush(const ParsedAction& action);
    bool executeSpeak(const ParsedAction& action);
    bool executeReturnToPost(const ParsedAction& action);
    void completeAction(bool success);

    // Multi-phase action updates
    void updateBow(float dt);
    void updateReturnToPost(float dt);
    void updatePosition(float dt);

    // Logging
    void logAction(const ParsedAction& action, const std::string& result);
};

}  // namespace greeter
