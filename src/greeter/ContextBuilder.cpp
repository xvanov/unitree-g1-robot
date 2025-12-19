#include "greeter/ContextBuilder.h"
#include <iostream>
#include <sstream>

namespace greeter {

ContextBuilder::ContextBuilder() = default;

void ContextBuilder::setFrame(const cv::Mat& frame) {
    // Clone to ensure we own the data
    current_frame_ = frame.clone();
}

void ContextBuilder::setDetectedFaces(const std::vector<FaceRect>& faces) {
    raw_faces_ = faces;
    face_identities_.clear();  // Clear old identities when new faces are set
}

void ContextBuilder::setEnvironment(const EnvironmentContext& env) {
    environment_ = env;
}

void ContextBuilder::setRobotState(const RobotStateContext& state) {
    robot_state_ = state;
}

void ContextBuilder::addOverheardConversation(const std::string& conversation) {
    overheard_conversations_.push_back(conversation);
}

void ContextBuilder::clearOverheardConversations() {
    overheard_conversations_.clear();
}

void ContextBuilder::setFaceIdentity(size_t face_index, const std::string& person_id) {
    face_identities_[face_index] = person_id;
}

std::string ContextBuilder::detectPosture(const FaceRect& face, int frame_height) const {
    if (frame_height <= 0) return "standing";

    float face_bottom_ratio = static_cast<float>(face.bottom()) / frame_height;
    return (face_bottom_ratio > 0.70f) ? "bent_forward" : "standing";
}

std::string ContextBuilder::detectAttention(const FaceRect& face, int frame_width) const {
    if (frame_width <= 0) return "looking_away";

    float center_x = static_cast<float>(face.centerX()) / frame_width;
    float aspect = static_cast<float>(face.bounding_box.width) /
                   std::max(1, face.bounding_box.height);

    bool centered = (center_x > 0.35f && center_x < 0.65f);
    bool frontal = (aspect > 0.7f && aspect < 1.3f);

    return (centered && frontal) ? "looking_at_robot" : "looking_away";
}

float ContextBuilder::estimateDistance(const FaceRect& face, int frame_height) const {
    if (frame_height <= 0) return 5.0f;

    float face_ratio = static_cast<float>(face.bounding_box.height) / frame_height;

    if (face_ratio > 0.25f) return 1.0f;
    if (face_ratio > 0.15f) return 2.0f;
    if (face_ratio > 0.10f) return 3.0f;
    if (face_ratio > 0.05f) return 4.0f;
    return 5.0f;
}

std::string ContextBuilder::encodeFrameBase64(const cv::Mat& frame) const {
    if (frame.empty()) {
        return "";
    }

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};

    try {
        cv::imencode(".jpg", frame, buf, params);
    } catch (const cv::Exception& e) {
        std::cerr << "ContextBuilder: Failed to encode frame: " << e.what() << std::endl;
        return "";
    }

    // Base64 encoding
    static const char* base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    std::string result;
    result.reserve(((buf.size() + 2) / 3) * 4);

    size_t i = 0;
    while (i < buf.size()) {
        uint32_t octet_a = buf[i++];
        uint32_t octet_b = i < buf.size() ? buf[i++] : 0;
        uint32_t octet_c = i < buf.size() ? buf[i++] : 0;

        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        result += base64_chars[(triple >> 18) & 0x3F];
        result += base64_chars[(triple >> 12) & 0x3F];
        result += (i > buf.size() + 1) ? '=' : base64_chars[(triple >> 6) & 0x3F];
        result += (i > buf.size()) ? '=' : base64_chars[triple & 0x3F];
    }

    return result;
}

std::vector<DetectedFace> ContextBuilder::getIdentifiedFaces() const {
    std::vector<DetectedFace> identified;

    int frame_height = current_frame_.rows;
    int frame_width = current_frame_.cols;

    for (size_t i = 0; i < raw_faces_.size(); ++i) {
        const auto& face = raw_faces_[i];
        DetectedFace df;
        df.bounding_box = face.bounding_box;
        df.confidence = face.confidence;
        df.posture = detectPosture(face, frame_height);
        df.attention = detectAttention(face, frame_width);
        df.estimated_distance = estimateDistance(face, frame_height);

        // Identity lookup (AC3)
        // Priority: 1) Manual override via setFaceIdentity()
        //           2) Automatic recognition via FaceRecognizer
        //           3) Unknown (nullopt)
        auto identity_it = face_identities_.find(i);
        if (identity_it != face_identities_.end() && personnel_db_ != nullptr) {
            // Manual identity set - look up PersonnelRecord
            df.identity = personnel_db_->findById(identity_it->second);
        } else if (face_recognizer_ != nullptr && face_recognizer_->isInitialized() &&
                   face_recognizer_->enrolledCount() > 0 && !current_frame_.empty()) {
            // Automatic face recognition
            RecognitionResult result = face_recognizer_->recognize(current_frame_, face);
            if (result.matched && personnel_db_ != nullptr) {
                df.identity = personnel_db_->findById(result.person_id);
            } else {
                df.identity = std::nullopt;
            }
        } else {
            df.identity = std::nullopt;
        }

        identified.push_back(df);
    }

    return identified;
}

nlohmann::json ContextBuilder::buildContextJson() const {
    nlohmann::json context;

    // Camera frame (base64 encoded JPEG)
    context["camera_frame"] = encodeFrameBase64(current_frame_);

    // Detected faces with analysis
    nlohmann::json faces_json = nlohmann::json::array();
    auto identified = getIdentifiedFaces();

    for (const auto& face : identified) {
        nlohmann::json face_json;

        if (face.identity.has_value()) {
            const auto& person = face.identity.value();
            face_json["id"] = person.id;
            face_json["name"] = person.name;
            face_json["role"] = person.role;
            face_json["relationship"] = person.relationship_to_robot;

            // Parse context_notes as array if it contains newlines, otherwise single item
            nlohmann::json notes = nlohmann::json::array();
            if (!person.context_notes.empty()) {
                std::istringstream iss(person.context_notes);
                std::string line;
                while (std::getline(iss, line)) {
                    if (!line.empty()) {
                        notes.push_back(line);
                    }
                }
                if (notes.empty()) {
                    notes.push_back(person.context_notes);
                }
            }
            face_json["context_notes"] = notes;
        } else {
            face_json["id"] = nullptr;
            face_json["name"] = "Unknown";
            face_json["role"] = nullptr;
            face_json["relationship"] = nullptr;
            face_json["context_notes"] = nlohmann::json::array();
        }

        face_json["posture"] = face.posture;
        face_json["attention"] = face.attention;
        face_json["estimated_distance"] = face.estimated_distance;

        // Include bounding box for debugging
        face_json["bounding_box"] = {
            {"x", face.bounding_box.x},
            {"y", face.bounding_box.y},
            {"width", face.bounding_box.width},
            {"height", face.bounding_box.height}
        };
        face_json["confidence"] = face.confidence;

        faces_json.push_back(face_json);
    }
    context["detected_faces"] = faces_json;

    // Environment context
    nlohmann::json env_json;
    env_json["near_staircase"] = environment_.near_staircase;
    env_json["witnesses_present"] = environment_.witnesses_present;
    env_json["camera_coverage"] = environment_.camera_coverage;
    env_json["observations"] = environment_.active_observations;
    context["environment"] = env_json;

    // Robot state context
    nlohmann::json robot_json;
    robot_json["current_pose"] = robot_state_.current_pose;
    robot_json["battery_percent"] = robot_state_.battery_percent;
    robot_json["current_action"] = robot_state_.current_action;
    robot_json["recent_actions"] = robot_state_.recent_actions;
    context["robot_state"] = robot_json;

    // Overheard conversations
    context["overheard_conversations"] = overheard_conversations_;

    return context;
}

}  // namespace greeter
