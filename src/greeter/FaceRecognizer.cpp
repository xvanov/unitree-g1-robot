#include "greeter/FaceRecognizer.h"
#include "greeter/GreeterConfig.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace greeter {

FaceRecognizer::FaceRecognizer() = default;

bool FaceRecognizer::init() {
    // Use GreeterConfig path discovery (Story 1-6)
    std::string model = GreeterConfig::findModelPath("face_recognition_sface_2021dec.onnx");

    if (model.empty()) {
        std::cerr << "FaceRecognizer: SFace model not found in search paths" << std::endl;
        return false;
    }

    std::cout << "FaceRecognizer: Using model: " << model << std::endl;
    return init(model);
}

bool FaceRecognizer::init(const std::string& model_path) {
    try {
        recognizer_ = cv::FaceRecognizerSF::create(model_path, "");
        if (recognizer_.empty()) {
            std::cerr << "FaceRecognizer: Failed to create recognizer from " << model_path << std::endl;
            initialized_ = false;
            return false;
        }

        initialized_ = true;
        std::cout << "FaceRecognizer: Model loaded successfully" << std::endl;
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "FaceRecognizer: OpenCV exception during init: " << e.what() << std::endl;
        initialized_ = false;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "FaceRecognizer: Exception during init: " << e.what() << std::endl;
        initialized_ = false;
        return false;
    }
}

cv::Mat FaceRecognizer::alignFace(const cv::Mat& frame, const FaceRect& face) {
    // Extract face region with some padding for better alignment
    int pad = static_cast<int>(face.bounding_box.width * 0.1);
    int x = std::max(0, face.bounding_box.x - pad);
    int y = std::max(0, face.bounding_box.y - pad);
    int w = std::min(frame.cols - x, face.bounding_box.width + 2 * pad);
    int h = std::min(frame.rows - y, face.bounding_box.height + 2 * pad);

    cv::Rect roi(x, y, w, h);
    cv::Mat face_img = frame(roi).clone();

    // Resize to expected input size (112x112 for SFace)
    cv::Mat aligned;
    cv::resize(face_img, aligned, cv::Size(112, 112));

    return aligned;
}

FaceEmbedding FaceRecognizer::extractEmbedding(const cv::Mat& frame, const FaceRect& face) {
    FaceEmbedding embedding;

    if (!initialized_) {
        std::cerr << "FaceRecognizer: Not initialized" << std::endl;
        return embedding;
    }

    if (frame.empty()) {
        return embedding;
    }

    try {
        // Align face
        cv::Mat aligned = alignFace(frame, face);

        // Extract features
        cv::Mat feature;
        recognizer_->feature(aligned, feature);

        // Convert to vector
        embedding.data.resize(feature.total());
        if (feature.isContinuous()) {
            std::memcpy(embedding.data.data(), feature.ptr<float>(), feature.total() * sizeof(float));
        } else {
            for (int i = 0; i < feature.rows; i++) {
                const float* row = feature.ptr<float>(i);
                for (int j = 0; j < feature.cols; j++) {
                    embedding.data[i * feature.cols + j] = row[j];
                }
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "FaceRecognizer: OpenCV exception during extraction: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "FaceRecognizer: Exception during extraction: " << e.what() << std::endl;
    }

    return embedding;
}

void FaceRecognizer::enrollFace(const std::string& person_id, const FaceEmbedding& embedding) {
    if (embedding.empty()) {
        std::cerr << "FaceRecognizer: Cannot enroll empty embedding for " << person_id << std::endl;
        return;
    }
    enrolled_faces_[person_id] = embedding;
    std::cout << "FaceRecognizer: Enrolled face for " << person_id << std::endl;
}

bool FaceRecognizer::enrollFace(const std::string& person_id, const cv::Mat& frame, const FaceRect& face) {
    FaceEmbedding embedding = extractEmbedding(frame, face);
    if (embedding.empty()) {
        return false;
    }
    enrollFace(person_id, embedding);
    return true;
}

bool FaceRecognizer::unenrollFace(const std::string& person_id) {
    auto it = enrolled_faces_.find(person_id);
    if (it != enrolled_faces_.end()) {
        enrolled_faces_.erase(it);
        std::cout << "FaceRecognizer: Unenrolled " << person_id << std::endl;
        return true;
    }
    return false;
}

float FaceRecognizer::cosineSimilarity(const FaceEmbedding& a, const FaceEmbedding& b) {
    if (a.empty() || b.empty() || a.size() != b.size()) {
        return 0.0f;
    }

    float dot = 0.0f;
    float norm_a = 0.0f;
    float norm_b = 0.0f;

    for (size_t i = 0; i < a.size(); i++) {
        dot += a.data[i] * b.data[i];
        norm_a += a.data[i] * a.data[i];
        norm_b += b.data[i] * b.data[i];
    }

    if (norm_a == 0.0f || norm_b == 0.0f) {
        return 0.0f;
    }

    return dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
}

RecognitionResult FaceRecognizer::recognize(const FaceEmbedding& embedding, float threshold) const {
    RecognitionResult result;
    result.similarity = 0.0f;
    result.matched = false;

    if (embedding.empty() || enrolled_faces_.empty()) {
        return result;
    }

    // Find best match
    float best_similarity = -1.0f;
    std::string best_match;

    for (const auto& [person_id, enrolled_embedding] : enrolled_faces_) {
        float similarity = cosineSimilarity(embedding, enrolled_embedding);
        if (similarity > best_similarity) {
            best_similarity = similarity;
            best_match = person_id;
        }
    }

    result.similarity = best_similarity;
    if (best_similarity >= threshold) {
        result.person_id = best_match;
        result.matched = true;
    }

    return result;
}

RecognitionResult FaceRecognizer::recognize(const cv::Mat& frame, const FaceRect& face, float threshold) {
    FaceEmbedding embedding = extractEmbedding(frame, face);
    return recognize(embedding, threshold);
}

bool FaceRecognizer::isEnrolled(const std::string& person_id) const {
    return enrolled_faces_.find(person_id) != enrolled_faces_.end();
}

std::vector<std::string> FaceRecognizer::getEnrolledIds() const {
    std::vector<std::string> ids;
    ids.reserve(enrolled_faces_.size());
    for (const auto& [id, _] : enrolled_faces_) {
        ids.push_back(id);
    }
    return ids;
}

FaceEmbedding FaceRecognizer::getEnrolledEmbedding(const std::string& person_id) const {
    auto it = enrolled_faces_.find(person_id);
    if (it != enrolled_faces_.end()) {
        return it->second;
    }
    return FaceEmbedding{};
}

bool FaceRecognizer::loadEnrollments(const std::string& path) {
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "FaceRecognizer: Cannot open enrollments file: " << path << std::endl;
            return false;
        }

        nlohmann::json j;
        file >> j;

        enrolled_faces_.clear();
        for (auto& [person_id, embedding_array] : j.items()) {
            FaceEmbedding embedding;
            embedding.data = embedding_array.get<std::vector<float>>();
            enrolled_faces_[person_id] = embedding;
        }

        std::cout << "FaceRecognizer: Loaded " << enrolled_faces_.size()
                  << " enrollments from " << path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "FaceRecognizer: Error loading enrollments: " << e.what() << std::endl;
        return false;
    }
}

bool FaceRecognizer::saveEnrollments(const std::string& path) const {
    try {
        nlohmann::json j;
        for (const auto& [person_id, embedding] : enrolled_faces_) {
            j[person_id] = embedding.data;
        }

        std::ofstream file(path);
        if (!file.is_open()) {
            std::cerr << "FaceRecognizer: Cannot create enrollments file: " << path << std::endl;
            return false;
        }

        file << j.dump(2);
        std::cout << "FaceRecognizer: Saved " << enrolled_faces_.size()
                  << " enrollments to " << path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "FaceRecognizer: Error saving enrollments: " << e.what() << std::endl;
        return false;
    }
}

void FaceRecognizer::clearEnrollments() {
    enrolled_faces_.clear();
}

}  // namespace greeter
