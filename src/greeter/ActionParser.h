#pragma once

#include <string>
#include <optional>

namespace greeter {

enum class ActionType {
    // Movement
    MOVE_TO,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    ROTATE,
    FOLLOW,
    STOP,
    RETURN_TO_POST,

    // Gestures
    WAVE_HAND,
    SHAKE_HAND,
    BOW,

    // Posture
    STAND_UP,
    SIT_DOWN,

    // Physical interaction
    PUSH_FORWARD,

    // Communication
    SPEAK,

    // Meta
    WAIT,
    NO_ACTION
};

struct ParsedAction {
    ActionType type = ActionType::NO_ACTION;

    // Movement parameters
    float distance = 0.0f;      // MOVE_FORWARD, MOVE_BACKWARD (meters)
    float angle = 0.0f;         // ROTATE (degrees, positive = left)
    float x = 0.0f, y = 0.0f;   // MOVE_TO coordinates

    // Target
    std::string target_id;      // FOLLOW target person ID

    // Physical
    float force_level = 0.5f;   // PUSH_FORWARD (0.0-1.0)

    // Communication
    std::string text;           // SPEAK text

    // LLM metadata (always capture for analysis)
    std::string reasoning;
    std::string intent;
    float confidence = 0.0f;
};

class ActionParser {
public:
    // Parse LLM JSON response into action
    // Returns nullopt on parse failure (never throws)
    std::optional<ParsedAction> parse(const std::string& json_response);

    // Get last parsing error message
    std::string getLastError() const { return last_error_; }

    // Extract reasoning even if action parsing fails (for logging)
    static std::string extractReasoning(const std::string& json_response);

    // Action type <-> string conversion
    static std::string actionTypeToString(ActionType type);
    static ActionType stringToActionType(const std::string& action_str);

private:
    std::string last_error_;
};

}  // namespace greeter
