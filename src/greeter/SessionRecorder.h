#pragma once

#include "greeter/ActionParser.h"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <fstream>
#include <filesystem>

namespace greeter {

/**
 * SessionRecorder - Records demo sessions for analysis.
 *
 * Saves:
 * - Camera frames at decision points (JPEG)
 * - reasoning.jsonl - JSON Lines with context and action for each decision
 * - actions.csv - CSV format action log
 * - metadata.json - Session metadata (start/end time, config, counts)
 *
 * Directory structure:
 *   data/recordings/<session_id>/
 *     frames/
 *       frame_001.jpg
 *       frame_002.jpg
 *       ...
 *     reasoning.jsonl
 *     actions.csv
 *     metadata.json
 */
class SessionRecorder {
public:
    SessionRecorder();
    ~SessionRecorder();

    /**
     * Start a new recording session.
     * @param output_dir Base directory for recordings (e.g., "data/recordings")
     * @param session_id Optional session ID (auto-generated if empty)
     * @return true if session started successfully
     */
    bool startSession(const std::string& output_dir, const std::string& session_id = "");

    /**
     * Record a decision point (frame + context + action).
     * @param frame Camera frame at decision time
     * @param context Context JSON sent to LLM
     * @param action Parsed action from LLM
     */
    void recordDecision(const cv::Mat& frame,
                       const nlohmann::json& context,
                       const ParsedAction& action);

    /**
     * Record an action result (success/failure).
     * @param action_id ID of the action (decision_id)
     * @param success Whether the action succeeded
     * @param notes Optional notes about the result
     */
    void recordActionResult(const std::string& action_id, bool success,
                           const std::string& notes = "");

    /**
     * End session and finalize files.
     */
    void endSession();

    /**
     * Get current session ID.
     */
    std::string getSessionId() const { return session_id_; }

    /**
     * Get session directory path.
     */
    std::string getSessionPath() const { return session_path_.string(); }

    /**
     * Get number of decisions recorded.
     */
    int getDecisionCount() const { return decision_count_; }

    /**
     * Check if currently recording.
     */
    bool isRecording() const { return recording_; }

    /**
     * Set experimental condition for metadata.
     */
    void setExperimentalCondition(const std::string& condition) {
        experimental_condition_ = condition;
    }

    /**
     * Set model name for metadata.
     */
    void setModelName(const std::string& model) { model_name_ = model; }

private:
    std::filesystem::path session_path_;
    std::string session_id_;
    std::string experimental_condition_ = "unknown";
    std::string model_name_ = "unknown";
    std::ofstream reasoning_file_;   // reasoning.jsonl
    std::ofstream actions_file_;     // actions.csv
    nlohmann::json metadata_;
    int decision_count_ = 0;
    int frame_count_ = 0;
    bool recording_ = false;
    std::chrono::system_clock::time_point start_time_;

    /**
     * Generate a session ID based on timestamp.
     */
    std::string generateSessionId() const;

    /**
     * Save a frame to disk.
     */
    void saveFrame(const cv::Mat& frame, int decision_id);

    /**
     * Write metadata.json file.
     */
    void writeMetadata();

    /**
     * Get current ISO timestamp.
     */
    std::string getTimestamp() const;
};

}  // namespace greeter
