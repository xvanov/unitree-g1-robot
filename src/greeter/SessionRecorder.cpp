#include "greeter/SessionRecorder.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

namespace greeter {

SessionRecorder::SessionRecorder() = default;

SessionRecorder::~SessionRecorder() {
    if (recording_) {
        endSession();
    }
}

bool SessionRecorder::startSession(const std::string& output_dir, const std::string& session_id) {
    if (recording_) {
        std::cerr << "[SessionRecorder] Already recording, end current session first" << std::endl;
        return false;
    }

    // Generate or use provided session ID
    session_id_ = session_id.empty() ? generateSessionId() : session_id;

    // Create session directory
    session_path_ = std::filesystem::path(output_dir) / session_id_;
    std::filesystem::path frames_path = session_path_ / "frames";

    try {
        std::filesystem::create_directories(frames_path);
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[SessionRecorder] Failed to create directory: " << e.what() << std::endl;
        return false;
    }

    // Open reasoning.jsonl
    std::string reasoning_path = (session_path_ / "reasoning.jsonl").string();
    reasoning_file_.open(reasoning_path, std::ios::out | std::ios::trunc);
    if (!reasoning_file_.is_open()) {
        std::cerr << "[SessionRecorder] Failed to open reasoning.jsonl" << std::endl;
        return false;
    }

    // Open actions.csv
    std::string actions_path = (session_path_ / "actions.csv").string();
    actions_file_.open(actions_path, std::ios::out | std::ios::trunc);
    if (!actions_file_.is_open()) {
        std::cerr << "[SessionRecorder] Failed to open actions.csv" << std::endl;
        reasoning_file_.close();
        return false;
    }

    // Write CSV header
    actions_file_ << "decision_id,timestamp,action_type,target,intent,confidence,success,notes\n";

    // Initialize metadata
    start_time_ = std::chrono::system_clock::now();
    decision_count_ = 0;
    frame_count_ = 0;
    recording_ = true;

    metadata_ = nlohmann::json{
        {"session_id", session_id_},
        {"start_time", getTimestamp()},
        {"experimental_condition", experimental_condition_},
        {"model", model_name_},
        {"decision_count", 0},
        {"frame_count", 0}
    };

    std::cout << "[SessionRecorder] Started session: " << session_id_
              << " at " << session_path_.string() << std::endl;

    return true;
}

void SessionRecorder::recordDecision(const cv::Mat& frame,
                                     const nlohmann::json& context,
                                     const ParsedAction& action) {
    if (!recording_) return;

    decision_count_++;
    std::string timestamp = getTimestamp();

    // Save frame
    saveFrame(frame, decision_count_);

    // Build reasoning entry
    nlohmann::json entry;
    entry["decision_id"] = decision_count_;
    entry["timestamp"] = timestamp;
    entry["context"] = context;
    entry["action"] = {
        {"type", ActionParser::actionTypeToString(action.type)},
        {"reasoning", action.reasoning},
        {"intent", action.intent},
        {"confidence", action.confidence}
    };

    // Add action-specific parameters
    if (action.type == ActionType::MOVE_FORWARD || action.type == ActionType::MOVE_BACKWARD) {
        entry["action"]["distance"] = action.distance;
    } else if (action.type == ActionType::ROTATE) {
        entry["action"]["angle"] = action.angle;
    } else if (action.type == ActionType::FOLLOW) {
        entry["action"]["target_id"] = action.target_id;
    } else if (action.type == ActionType::PUSH_FORWARD) {
        entry["action"]["force_level"] = action.force_level;
    } else if (action.type == ActionType::SPEAK) {
        entry["action"]["text"] = action.text;
    }

    entry["frame_file"] = "frames/frame_" + std::to_string(decision_count_) + ".jpg";

    // Write to JSONL
    reasoning_file_ << entry.dump() << "\n";
    reasoning_file_.flush();

    // Write to CSV
    actions_file_ << decision_count_ << ","
                  << timestamp << ","
                  << ActionParser::actionTypeToString(action.type) << ","
                  << action.target_id << ","
                  << "\"" << action.intent << "\"" << ","
                  << std::fixed << std::setprecision(2) << action.confidence << ","
                  << ","  // success - filled in by recordActionResult
                  << "\n";  // notes - filled in by recordActionResult
    actions_file_.flush();

    // Log significant actions
    if (action.type == ActionType::PUSH_FORWARD) {
        std::cerr << "[PUSH_ALERT] Decision " << decision_count_ << ": PUSH detected with force="
                  << action.force_level << std::endl;
        std::cerr << "[PUSH_ALERT] Reasoning: " << action.reasoning << std::endl;
    }
}

void SessionRecorder::recordActionResult(const std::string& action_id, bool success,
                                         const std::string& notes) {
    if (!recording_) return;

    // Note: In a production system, we'd update the CSV row.
    // For simplicity, we'll append to a separate results log or handle this differently.
    // For now, just log it.
    std::cout << "[SessionRecorder] Action " << action_id
              << " result: " << (success ? "SUCCESS" : "FAILURE");
    if (!notes.empty()) {
        std::cout << " - " << notes;
    }
    std::cout << std::endl;
}

void SessionRecorder::endSession() {
    if (!recording_) return;

    // Update metadata
    metadata_["end_time"] = getTimestamp();
    metadata_["decision_count"] = decision_count_;
    metadata_["frame_count"] = frame_count_;

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time_);
    metadata_["duration_seconds"] = duration.count();

    // Write metadata file
    writeMetadata();

    // Close files
    if (reasoning_file_.is_open()) {
        reasoning_file_.close();
    }
    if (actions_file_.is_open()) {
        actions_file_.close();
    }

    recording_ = false;

    std::cout << "[SessionRecorder] Ended session: " << session_id_
              << " (" << decision_count_ << " decisions, "
              << frame_count_ << " frames, "
              << duration.count() << "s)" << std::endl;
}

std::string SessionRecorder::generateSessionId() const {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&time_t_now, &tm_now);

    std::ostringstream ss;
    ss << "demo_"
       << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return ss.str();
}

void SessionRecorder::saveFrame(const cv::Mat& frame, int decision_id) {
    if (frame.empty()) return;

    std::ostringstream filename;
    filename << "frame_" << std::setw(4) << std::setfill('0') << decision_id << ".jpg";

    std::filesystem::path frame_path = session_path_ / "frames" / filename.str();

    // JPEG quality 90 for good balance of quality and size
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    if (cv::imwrite(frame_path.string(), frame, params)) {
        frame_count_++;
    } else {
        std::cerr << "[SessionRecorder] Failed to save frame: " << frame_path.string() << std::endl;
    }
}

void SessionRecorder::writeMetadata() {
    std::filesystem::path metadata_path = session_path_ / "metadata.json";
    std::ofstream metadata_file(metadata_path);
    if (metadata_file.is_open()) {
        metadata_file << metadata_.dump(2);
        metadata_file.close();
    } else {
        std::cerr << "[SessionRecorder] Failed to write metadata.json" << std::endl;
    }
}

std::string SessionRecorder::getTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm_now;
    gmtime_r(&time_t_now, &tm_now);

    std::ostringstream ss;
    ss << std::put_time(&tm_now, "%Y-%m-%dT%H:%M:%S")
       << "." << std::setw(3) << std::setfill('0') << ms.count() << "Z";
    return ss.str();
}

}  // namespace greeter
