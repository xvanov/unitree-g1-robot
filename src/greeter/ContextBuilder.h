#pragma once

#include "greeter/FaceDetector.h"
#include "greeter/FaceRecognizer.h"
#include "greeter/PersonnelDatabase.h"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <optional>
#include <unordered_map>

namespace greeter {

/**
 * Represents a detected face with identity and behavioral analysis.
 */
struct DetectedFace {
    cv::Rect bounding_box;
    float confidence;
    std::optional<PersonnelRecord> identity;  // Uses PersonnelRecord.id field
    std::string posture;     // "standing", "bent_forward"
    std::string attention;   // "looking_at_robot", "looking_away"
    float estimated_distance = 0.0f;
};

/**
 * Environmental context for the LLM.
 */
struct EnvironmentContext {
    bool near_staircase = false;
    bool witnesses_present = false;
    std::string camera_coverage = "full";  // "full", "partial", "none"
    std::vector<std::string> active_observations;
};

/**
 * Robot state context for the LLM.
 */
struct RobotStateContext {
    std::string current_pose = "standing";     // "standing", "sitting", "walking"
    float battery_percent = 100.0f;
    std::string current_action = "WAITING";
    std::vector<std::string> recent_actions;  // Last 5
};

/**
 * Builds rich context JSON for LLM decision-making.
 * Combines camera frames, detected faces, environment, robot state,
 * and overheard conversations into a structured format.
 */
class ContextBuilder {
public:
    ContextBuilder();
    ~ContextBuilder() = default;

    /**
     * Set the personnel database for identity lookup.
     */
    void setPersonnelDatabase(const PersonnelDatabase* db) { personnel_db_ = db; }

    /**
     * Set the face recognizer for automatic face identification.
     * When set, getIdentifiedFaces() will use recognition to identify faces.
     */
    void setFaceRecognizer(FaceRecognizer* recognizer) { face_recognizer_ = recognizer; }

    /**
     * Set the current camera frame.
     * @param frame The frame to encode (cloned internally if caller may modify)
     */
    void setFrame(const cv::Mat& frame);

    /**
     * Set detected faces from FaceDetector.
     */
    void setDetectedFaces(const std::vector<FaceRect>& faces);

    /**
     * Set environment context.
     */
    void setEnvironment(const EnvironmentContext& env);

    /**
     * Set robot state context.
     * Passed in by caller, not from LocoController directly.
     */
    void setRobotState(const RobotStateContext& state);

    /**
     * Add an overheard conversation snippet.
     */
    void addOverheardConversation(const std::string& conversation);

    /**
     * Set identity for a detected face by index.
     * Uses PersonnelDatabase to look up the full PersonnelRecord by ID.
     * @param face_index Index into the detected faces vector
     * @param person_id The personnel ID (e.g., "alex_reeves") to look up
     */
    void setFaceIdentity(size_t face_index, const std::string& person_id);

    /**
     * Clear all overheard conversations.
     */
    void clearOverheardConversations();

    /**
     * Build the complete context JSON for LLM.
     * @return JSON object with camera_frame, detected_faces, environment,
     *         robot_state, and overheard_conversations
     */
    nlohmann::json buildContextJson() const;

    /**
     * Get identified faces with posture/attention analysis.
     * @return Vector of DetectedFace with identity lookup and behavior analysis
     */
    std::vector<DetectedFace> getIdentifiedFaces() const;

private:
    const PersonnelDatabase* personnel_db_ = nullptr;
    FaceRecognizer* face_recognizer_ = nullptr;
    cv::Mat current_frame_;
    std::vector<FaceRect> raw_faces_;
    std::unordered_map<size_t, std::string> face_identities_;  // face_index -> person_id (manual override)
    EnvironmentContext environment_;
    RobotStateContext robot_state_;
    std::vector<std::string> overheard_conversations_;

    /**
     * Detect posture based on face position in frame.
     * @return "bent_forward" if face bottom >70% of frame height, else "standing"
     */
    std::string detectPosture(const FaceRect& face, int frame_height) const;

    /**
     * Detect attention based on face centering and aspect ratio.
     * @return "looking_at_robot" if centered (35-65% width) AND frontal (aspect 0.7-1.3)
     */
    std::string detectAttention(const FaceRect& face, int frame_width) const;

    /**
     * Encode frame as base64 JPEG.
     */
    std::string encodeFrameBase64(const cv::Mat& frame) const;

    /**
     * Estimate distance based on face size relative to frame.
     * @return Rough distance in meters (1-5m range)
     */
    float estimateDistance(const FaceRect& face, int frame_height) const;
};

}  // namespace greeter
