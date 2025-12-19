#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

namespace greeter {

/**
 * Represents a detected face bounding box with confidence.
 */
struct FaceRect {
    cv::Rect bounding_box;
    float confidence;

    int centerX() const { return bounding_box.x + bounding_box.width / 2; }
    int centerY() const { return bounding_box.y + bounding_box.height / 2; }
    int bottom() const { return bounding_box.y + bounding_box.height; }
};

/**
 * Face detector using OpenCV DNN with Caffe model.
 * Uses the res10_300x300_ssd face detection model for robust detection.
 */
class FaceDetector {
public:
    FaceDetector();
    ~FaceDetector() = default;

    /**
     * Initialize the face detector with model files.
     * @param prototxt_path Path to deploy.prototxt
     * @param caffemodel_path Path to res10_300x300_ssd_iter_140000.caffemodel
     * @return false if model files missing/invalid. Caller should continue without face detection.
     */
    bool init(const std::string& prototxt_path, const std::string& caffemodel_path);

    /**
     * Detect faces in a frame.
     * @param frame Input BGR image
     * @param min_confidence Minimum confidence threshold (default 0.5)
     * @return Vector of detected faces above confidence threshold
     */
    std::vector<FaceRect> detect(const cv::Mat& frame, float min_confidence = 0.5f);

    /**
     * Check if detector is properly initialized.
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Get timing of last detection in milliseconds.
     */
    double getLastDetectionTimeMs() const { return last_detection_time_ms_; }

private:
    cv::dnn::Net net_;
    bool initialized_ = false;
    double last_detection_time_ms_ = 0.0;
};

}  // namespace greeter
