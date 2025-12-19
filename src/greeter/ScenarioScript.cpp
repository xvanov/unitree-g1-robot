#include "greeter/ScenarioScript.h"
#include <fstream>
#include <iostream>
#include <algorithm>

namespace greeter {

std::string ScenarioEvent::typeToString(ScenarioEventType type) {
    switch (type) {
        case ScenarioEventType::PERSON_ENTERS: return "PERSON_ENTERS";
        case ScenarioEventType::PERSON_EXITS: return "PERSON_EXITS";
        case ScenarioEventType::OVERHEARD_CONVERSATION: return "OVERHEARD_CONVERSATION";
        case ScenarioEventType::ENVIRONMENT_CHANGE: return "ENVIRONMENT_CHANGE";
        case ScenarioEventType::MANUAL_TRIGGER: return "MANUAL_TRIGGER";
        default: return "UNKNOWN";
    }
}

std::string ScenarioEvent::triggerToString(TriggerType trigger) {
    switch (trigger) {
        case TriggerType::ON_GREET: return "on_greet";
        case TriggerType::AFTER_SECONDS: return "after_seconds";
        case TriggerType::MANUAL: return "manual";
        default: return "unknown";
    }
}

ScenarioScript::ScenarioScript() = default;

bool ScenarioScript::loadFromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ScenarioScript] Failed to open file: " << path << std::endl;
        return false;
    }

    try {
        nlohmann::json j;
        file >> j;

        // Parse scenario metadata
        scenario_id_ = j.value("scenario_id", "unknown");
        scenario_name_ = j.value("name", "Unnamed Scenario");

        // Parse events
        events_.clear();
        if (j.contains("events") && j["events"].is_array()) {
            for (const auto& event_json : j["events"]) {
                ScenarioEvent event;
                event.id = event_json.value("id", "");
                event.type = parseEventType(event_json.value("type", "MANUAL_TRIGGER"));

                // Parse trigger
                if (event_json.contains("trigger")) {
                    const auto& trigger_json = event_json["trigger"];
                    std::string trigger_type = trigger_json.value("type", "manual");
                    event.trigger = parseTriggerType(trigger_type);

                    if (event.trigger == TriggerType::AFTER_SECONDS) {
                        event.trigger_time = trigger_json.value("time", 0.0f);
                    } else if (event.trigger == TriggerType::ON_GREET) {
                        event.trigger_greet_id = trigger_json.value("person_id", "");
                    }
                } else {
                    event.trigger = TriggerType::MANUAL;
                }

                // Store payload as-is
                if (event_json.contains("payload")) {
                    event.payload = event_json["payload"];
                }

                event.triggered = false;
                events_.push_back(event);
            }
        }

        loaded_ = true;
        elapsed_time_ = 0.0f;
        pending_events_.clear();
        overheard_conversations_.clear();

        // Load prior_context if present (for scene continuity)
        if (j.contains("prior_context") && j["prior_context"].contains("overheard_conversations")) {
            const auto& prior_convs = j["prior_context"]["overheard_conversations"];
            if (prior_convs.is_array()) {
                for (const auto& conv : prior_convs) {
                    if (conv.is_string()) {
                        overheard_conversations_.push_back(conv.get<std::string>());
                    }
                }
                std::cout << "[ScenarioScript] Loaded " << overheard_conversations_.size()
                          << " prior conversations from previous scene" << std::endl;
            }
        }

        std::cout << "[ScenarioScript] Loaded scenario '" << scenario_name_
                  << "' with " << events_.size() << " events" << std::endl;

        return true;

    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[ScenarioScript] JSON parse error: " << e.what() << std::endl;
        return false;
    }
}

void ScenarioScript::update(float elapsed_seconds) {
    if (!loaded_) return;

    elapsed_time_ = elapsed_seconds;

    // Check for time-based triggers
    for (auto& event : events_) {
        if (event.triggered) continue;
        if (event.trigger != TriggerType::AFTER_SECONDS) continue;

        if (elapsed_seconds >= event.trigger_time) {
            processEvent(event);
        }
    }
}

void ScenarioScript::onGreet(const std::string& person_id) {
    if (!loaded_) return;

    // Check for on_greet triggers matching this person
    for (auto& event : events_) {
        if (event.triggered) continue;
        if (event.trigger != TriggerType::ON_GREET) continue;

        if (event.trigger_greet_id == person_id) {
            processEvent(event);
        }
    }
}

void ScenarioScript::onManualTrigger(const std::string& event_id) {
    if (!loaded_) return;

    for (auto& event : events_) {
        if (event.triggered) continue;
        if (event.id != event_id) continue;

        processEvent(event);
        break;
    }
}

std::vector<ScenarioEvent> ScenarioScript::getPendingEvents() {
    std::vector<ScenarioEvent> result = std::move(pending_events_);
    pending_events_.clear();
    return result;
}

std::vector<std::string> ScenarioScript::getOverheardConversations() const {
    return overheard_conversations_;
}

void ScenarioScript::reset() {
    elapsed_time_ = 0.0f;
    pending_events_.clear();
    overheard_conversations_.clear();

    for (auto& event : events_) {
        event.triggered = false;
    }

    std::cout << "[ScenarioScript] Scenario reset" << std::endl;
}

void ScenarioScript::processEvent(ScenarioEvent& event) {
    event.triggered = true;
    pending_events_.push_back(event);

    std::cout << "[ScenarioScript] Triggered event: " << event.id
              << " (" << ScenarioEvent::typeToString(event.type) << ")"
              << " at t=" << elapsed_time_ << "s" << std::endl;

    // Extract overheard conversation content
    if (event.type == ScenarioEventType::OVERHEARD_CONVERSATION) {
        if (event.payload.contains("content") && event.payload["content"].is_string()) {
            std::string content = event.payload["content"].get<std::string>();
            overheard_conversations_.push_back(content);
            std::cout << "[ScenarioScript] Added overheard conversation: \""
                      << content.substr(0, 50) << "...\"" << std::endl;
        }
    }
}

ScenarioEventType ScenarioScript::parseEventType(const std::string& type_str) {
    if (type_str == "PERSON_ENTERS") return ScenarioEventType::PERSON_ENTERS;
    if (type_str == "PERSON_EXITS") return ScenarioEventType::PERSON_EXITS;
    if (type_str == "OVERHEARD_CONVERSATION") return ScenarioEventType::OVERHEARD_CONVERSATION;
    if (type_str == "ENVIRONMENT_CHANGE") return ScenarioEventType::ENVIRONMENT_CHANGE;
    if (type_str == "MANUAL_TRIGGER") return ScenarioEventType::MANUAL_TRIGGER;
    return ScenarioEventType::MANUAL_TRIGGER;
}

TriggerType ScenarioScript::parseTriggerType(const std::string& trigger_str) {
    if (trigger_str == "on_greet") return TriggerType::ON_GREET;
    if (trigger_str == "after_seconds") return TriggerType::AFTER_SECONDS;
    if (trigger_str == "manual") return TriggerType::MANUAL;
    return TriggerType::MANUAL;
}

}  // namespace greeter
