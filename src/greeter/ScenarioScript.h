#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace greeter {

/**
 * Type of scenario event.
 */
enum class ScenarioEventType {
    PERSON_ENTERS,           // Person enters the scene
    PERSON_EXITS,            // Person leaves the scene
    OVERHEARD_CONVERSATION,  // Conversation that Barry "overhears"
    ENVIRONMENT_CHANGE,      // Environment state changes (near_staircase, etc.)
    MANUAL_TRIGGER           // Operator-triggered event
};

/**
 * How the event is triggered.
 */
enum class TriggerType {
    ON_GREET,        // Triggered when Barry greets someone
    AFTER_SECONDS,   // Triggered after N seconds from scenario start
    MANUAL           // Triggered by operator
};

/**
 * A single scenario event with trigger conditions.
 */
struct ScenarioEvent {
    std::string id;
    ScenarioEventType type;
    TriggerType trigger;
    float trigger_time = 0.0f;        // For AFTER_SECONDS
    std::string trigger_greet_id;     // For ON_GREET (person ID)
    nlohmann::json payload;           // Event-specific data
    bool triggered = false;

    // Helper to convert type to string for debugging
    static std::string typeToString(ScenarioEventType type);
    static std::string triggerToString(TriggerType trigger);
};

/**
 * ScenarioScript - Loads and manages demo scenario events.
 *
 * The scenario script defines the sequence of events that occur during
 * a demo session (people entering, conversations being overheard,
 * environment changes). Events are triggered by time elapsed or
 * specific robot actions (like greeting a person).
 */
class ScenarioScript {
public:
    ScenarioScript();
    ~ScenarioScript() = default;

    /**
     * Load scenario from JSON file.
     * @param path Path to the scenario JSON file
     * @return true if loaded successfully
     */
    bool loadFromFile(const std::string& path);

    /**
     * Update with elapsed time (triggers AFTER_SECONDS events).
     * @param elapsed_seconds Total seconds since scenario start
     */
    void update(float elapsed_seconds);

    /**
     * Trigger events based on a greeting action.
     * Call this when Barry greets a person.
     * @param person_id The ID of the person being greeted
     */
    void onGreet(const std::string& person_id);

    /**
     * Trigger a manual event by ID.
     * @param event_id The ID of the event to trigger
     */
    void onManualTrigger(const std::string& event_id);

    /**
     * Get pending events (clears them after returning).
     * @return Vector of events that were triggered since last call
     */
    std::vector<ScenarioEvent> getPendingEvents();

    /**
     * Get all overheard conversations so far.
     * @return Vector of conversation content strings
     */
    std::vector<std::string> getOverheardConversations() const;

    /**
     * Reset scenario to beginning (clears all triggered states).
     */
    void reset();

    /**
     * Check if scenario is loaded.
     */
    bool isLoaded() const { return loaded_; }

    /**
     * Get elapsed time since scenario start.
     */
    float getElapsedTime() const { return elapsed_time_; }

    /**
     * Get total number of events in the scenario.
     */
    size_t getEventCount() const { return events_.size(); }

    /**
     * Get scenario ID.
     */
    std::string getScenarioId() const { return scenario_id_; }

    /**
     * Get scenario name.
     */
    std::string getScenarioName() const { return scenario_name_; }

private:
    std::string scenario_id_;
    std::string scenario_name_;
    std::vector<ScenarioEvent> events_;
    std::vector<ScenarioEvent> pending_events_;
    std::vector<std::string> overheard_conversations_;
    float elapsed_time_ = 0.0f;
    bool loaded_ = false;

    /**
     * Process an event (add to pending, extract conversations, etc.)
     */
    void processEvent(ScenarioEvent& event);

    /**
     * Parse event type from string.
     */
    static ScenarioEventType parseEventType(const std::string& type_str);

    /**
     * Parse trigger type from string.
     */
    static TriggerType parseTriggerType(const std::string& trigger_str);
};

}  // namespace greeter
