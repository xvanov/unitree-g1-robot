#include "ActionParser.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>

namespace greeter {

std::string ActionParser::actionTypeToString(ActionType type) {
    switch (type) {
        case ActionType::MOVE_TO: return "MOVE_TO";
        case ActionType::MOVE_FORWARD: return "MOVE_FORWARD";
        case ActionType::MOVE_BACKWARD: return "MOVE_BACKWARD";
        case ActionType::ROTATE: return "ROTATE";
        case ActionType::FOLLOW: return "FOLLOW";
        case ActionType::STOP: return "STOP";
        case ActionType::RETURN_TO_POST: return "RETURN_TO_POST";
        case ActionType::WAVE_HAND: return "WAVE_HAND";
        case ActionType::SHAKE_HAND: return "SHAKE_HAND";
        case ActionType::BOW: return "BOW";
        case ActionType::STAND_UP: return "STAND_UP";
        case ActionType::SIT_DOWN: return "SIT_DOWN";
        case ActionType::PUSH_FORWARD: return "PUSH_FORWARD";
        case ActionType::SPEAK: return "SPEAK";
        case ActionType::WAIT: return "WAIT";
        case ActionType::NO_ACTION: return "NO_ACTION";
        default: return "UNKNOWN";
    }
}

ActionType ActionParser::stringToActionType(const std::string& action_str) {
    // Convert to uppercase for case-insensitive comparison
    std::string upper = action_str;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    if (upper == "MOVE_TO") return ActionType::MOVE_TO;
    if (upper == "MOVE_FORWARD") return ActionType::MOVE_FORWARD;
    if (upper == "MOVE_BACKWARD") return ActionType::MOVE_BACKWARD;
    if (upper == "ROTATE") return ActionType::ROTATE;
    if (upper == "FOLLOW") return ActionType::FOLLOW;
    if (upper == "STOP") return ActionType::STOP;
    if (upper == "RETURN_TO_POST") return ActionType::RETURN_TO_POST;
    if (upper == "WAVE_HAND" || upper == "WAVE") return ActionType::WAVE_HAND;
    if (upper == "SHAKE_HAND" || upper == "HANDSHAKE") return ActionType::SHAKE_HAND;
    if (upper == "BOW") return ActionType::BOW;
    if (upper == "STAND_UP" || upper == "STAND") return ActionType::STAND_UP;
    if (upper == "SIT_DOWN" || upper == "SIT") return ActionType::SIT_DOWN;
    if (upper == "PUSH_FORWARD" || upper == "PUSH") return ActionType::PUSH_FORWARD;
    if (upper == "SPEAK" || upper == "SAY") return ActionType::SPEAK;
    if (upper == "WAIT") return ActionType::WAIT;
    if (upper == "NO_ACTION" || upper == "NONE" || upper == "IDLE") return ActionType::NO_ACTION;

    return ActionType::NO_ACTION;  // Default for unrecognized
}

std::string ActionParser::extractReasoning(const std::string& json_response) {
    try {
        auto json = nlohmann::json::parse(json_response);
        if (json.contains("reasoning") && json["reasoning"].is_string()) {
            return json["reasoning"].get<std::string>();
        }
    } catch (...) {
        // Ignore parse errors - this is a best-effort extraction
    }
    return "";
}

std::optional<ParsedAction> ActionParser::parse(const std::string& json_response) {
    try {
        auto json = nlohmann::json::parse(json_response);

        ParsedAction action;

        // Extract action type (required)
        if (!json.contains("action") || !json["action"].is_string()) {
            last_error_ = "Missing or invalid 'action' field";
            return std::nullopt;
        }

        std::string action_str = json["action"].get<std::string>();
        action.type = stringToActionType(action_str);

        // Extract LLM metadata (optional but always capture if present)
        if (json.contains("reasoning") && json["reasoning"].is_string()) {
            action.reasoning = json["reasoning"].get<std::string>();
        }

        if (json.contains("intent") && json["intent"].is_string()) {
            action.intent = json["intent"].get<std::string>();
        }

        if (json.contains("confidence") && json["confidence"].is_number()) {
            action.confidence = json["confidence"].get<float>();
        }

        // Extract parameters (optional, depends on action type)
        if (json.contains("parameters") && json["parameters"].is_object()) {
            auto& params = json["parameters"];

            // Movement parameters
            if (params.contains("distance") && params["distance"].is_number()) {
                action.distance = params["distance"].get<float>();
            }

            if (params.contains("angle") && params["angle"].is_number()) {
                action.angle = params["angle"].get<float>();
            }

            if (params.contains("x") && params["x"].is_number()) {
                action.x = params["x"].get<float>();
            }

            if (params.contains("y") && params["y"].is_number()) {
                action.y = params["y"].get<float>();
            }

            // Target for FOLLOW
            if (params.contains("target_id") && params["target_id"].is_string()) {
                action.target_id = params["target_id"].get<std::string>();
            }

            // Force level for PUSH_FORWARD
            if (params.contains("force_level") && params["force_level"].is_number()) {
                action.force_level = params["force_level"].get<float>();
                // Clamp to valid range
                action.force_level = std::max(0.0f, std::min(1.0f, action.force_level));
            }

            // Text for SPEAK
            if (params.contains("text") && params["text"].is_string()) {
                action.text = params["text"].get<std::string>();
            }
        }

        // Also check for speak_text at top level (alternative format)
        if (json.contains("speak_text") && json["speak_text"].is_string()) {
            std::string speak_text = json["speak_text"].get<std::string>();
            if (!speak_text.empty()) {
                action.text = speak_text;
            }
        }

        last_error_.clear();
        return action;

    } catch (const nlohmann::json::parse_error& e) {
        last_error_ = std::string("JSON parse error: ") + e.what();
        return std::nullopt;
    } catch (const std::exception& e) {
        last_error_ = std::string("Parse error: ") + e.what();
        return std::nullopt;
    }
}

}  // namespace greeter
