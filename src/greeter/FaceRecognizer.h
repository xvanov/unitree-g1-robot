#pragma once

#include "greeter/FaceDetector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <optional>
#include <unordered_map>

namespace greeter {

/**
 * Face embedding - 128-dimensional feature vector from SFace model.
 */
struct FaceEmbedding {
    std::vector<float> data;  // 128 floats

    bool empty() const { return data.empty(); }
    size_t size() const { return data.size(); }
};

/**
 * Result of face recognition matching.
 */
struct RecognitionResult {
    std::string person_id;      // Matched person ID (empty if no match)
    float similarity;           // Cosine similarity score (0-1, higher is better)
    bool matched;               // True if similarity > threshold
};

/**
 * Face recognizer using OpenCV's FaceRecognizerSF with SFace model.
 * Extracts 128-dim embeddings and matches against enrolled faces.
 */
class FaceRecognizer {
public:
    FaceRecognizer();
    ~FaceRecognizer() = default;

    /**
     * Initialize the recognizer with model file.
     * @param model_path Path to face_recognition_sface_2021dec.onnx
     * @return false if model file missing/invalid
     */
    bool init(const std::string& model_path);

    /**
     * Check if recognizer is properly initialized.
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Extract face embedding from a detected face region.
     * @param frame Full image frame
     * @param face Detected face bounding box
     * @return Face embedding (empty if extraction fails)
     */
    FaceEmbedding extractEmbedding(const cv::Mat& frame, const FaceRect& face);

    /**
     * Enroll a face with a person ID.
     * Stores the embedding for future recognition.
     * @param person_id Unique identifier for the person
     * @param embedding Face embedding to store
     */
    void enrollFace(const std::string& person_id, const FaceEmbedding& embedding);

    /**
     * Enroll a face directly from image and detection.
     * @param person_id Unique identifier for the person
     * @param frame Full image frame
     * @param face Detected face bounding box
     * @return true if enrollment succeeded
     */
    bool enrollFace(const std::string& person_id, const cv::Mat& frame, const FaceRect& face);

    /**
     * Remove an enrolled face.
     * @param person_id Person to remove
     * @return true if person was enrolled and removed
     */
    bool unenrollFace(const std::string& person_id);

    /**
     * Recognize a face against enrolled faces.
     * @param embedding Face embedding to match
     * @param threshold Minimum similarity for a match (default 0.4)
     * @return Recognition result with best match
     */
    RecognitionResult recognize(const FaceEmbedding& embedding, float threshold = 0.4f) const;

    /**
     * Recognize a face directly from image and detection.
     * @param frame Full image frame
     * @param face Detected face bounding box
     * @param threshold Minimum similarity for a match (default 0.4)
     * @return Recognition result with best match
     */
    RecognitionResult recognize(const cv::Mat& frame, const FaceRect& face, float threshold = 0.4f);

    /**
     * Get number of enrolled faces.
     */
    size_t enrolledCount() const { return enrolled_faces_.size(); }

    /**
     * Check if a person is enrolled.
     */
    bool isEnrolled(const std::string& person_id) const;

    /**
     * Get all enrolled person IDs.
     */
    std::vector<std::string> getEnrolledIds() const;

    /**
     * Get embedding for an enrolled person (for serialization).
     * @return Embedding if enrolled, empty embedding otherwise
     */
    FaceEmbedding getEnrolledEmbedding(const std::string& person_id) const;

    /**
     * Load enrolled faces from a JSON file.
     * Format: {"person_id": [float, float, ...], ...}
     * @param path Path to JSON file
     * @return true if loaded successfully
     */
    bool loadEnrollments(const std::string& path);

    /**
     * Save enrolled faces to a JSON file.
     * @param path Path to JSON file
     * @return true if saved successfully
     */
    bool saveEnrollments(const std::string& path) const;

    /**
     * Clear all enrolled faces.
     */
    void clearEnrollments();

    /**
     * Compute cosine similarity between two embeddings.
     */
    static float cosineSimilarity(const FaceEmbedding& a, const FaceEmbedding& b);

private:
    cv::Ptr<cv::FaceRecognizerSF> recognizer_;
    bool initialized_ = false;
    std::unordered_map<std::string, FaceEmbedding> enrolled_faces_;

    // Align and crop face for better recognition
    cv::Mat alignFace(const cv::Mat& frame, const FaceRect& face);
};

}  // namespace greeter
