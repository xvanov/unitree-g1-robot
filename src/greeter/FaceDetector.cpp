#include "greeter/FaceDetector.h"
#include "greeter/GreeterConfig.h"
#include <iostream>
#include <chrono>

namespace greeter {

FaceDetector::FaceDetector() = default;

bool FaceDetector::init() {
    // Use GreeterConfig path discovery (Story 1-6)
    std::string proto = GreeterConfig::findModelPath("deploy.prototxt");
    std::string model = GreeterConfig::findModelPath("res10_300x300_ssd_iter_140000.caffemodel");

    if (proto.empty()) {
        std::cerr << "FaceDetector: deploy.prototxt not found in search paths" << std::endl;
        return false;
    }
    if (model.empty()) {
        std::cerr << "FaceDetector: caffemodel not found in search paths" << std::endl;
        return false;
    }

    std::cout << "FaceDetector: Using prototxt: " << proto << std::endl;
    std::cout << "FaceDetector: Using model: " << model << std::endl;
    return init(proto, model);
}

bool FaceDetector::init(const std::string& prototxt_path, const std::string& caffemodel_path) {
    try {
        net_ = cv::dnn::readNetFromCaffe(prototxt_path, caffemodel_path);
        if (net_.empty()) {
            std::cerr << "FaceDetector: Failed to load network from "
                      << prototxt_path << " and " << caffemodel_path << std::endl;
            initialized_ = false;
            return false;
        }

        // Use OpenCV backend with CPU target for compatibility
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        initialized_ = true;
        std::cout << "FaceDetector: Model loaded successfully" << std::endl;
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "FaceDetector: OpenCV exception during init: " << e.what() << std::endl;
        initialized_ = false;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "FaceDetector: Exception during init: " << e.what() << std::endl;
        initialized_ = false;
        return false;
    }
}

std::vector<FaceRect> FaceDetector::detect(const cv::Mat& frame, float min_confidence) {
    std::vector<FaceRect> faces;

    if (!initialized_) {
        return faces;
    }

    if (frame.empty()) {
        return faces;
    }

    auto start = std::chrono::high_resolution_clock::now();

    try {
        // Create blob from image - res10 model expects 300x300 input
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300),
            cv::Scalar(104.0, 177.0, 123.0), false, false);

        net_.setInput(blob);
        cv::Mat detection = net_.forward();

        // Output shape: [1, 1, N, 7]
        // Each detection: [batchId, classId, confidence, x1, y1, x2, y2]
        // x1, y1, x2, y2 are normalized (0-1)
        cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

        for (int i = 0; i < detectionMat.rows; i++) {
            float confidence = detectionMat.at<float>(i, 2);

            if (confidence > min_confidence) {
                // Convert normalized coordinates to pixel coordinates
                int x1 = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
                int y1 = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
                int x2 = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
                int y2 = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);

                // Clamp coordinates to image bounds
                x1 = std::max(0, std::min(x1, frame.cols - 1));
                y1 = std::max(0, std::min(y1, frame.rows - 1));
                x2 = std::max(0, std::min(x2, frame.cols - 1));
                y2 = std::max(0, std::min(y2, frame.rows - 1));

                // Ensure valid rectangle (width and height > 0)
                if (x2 > x1 && y2 > y1) {
                    FaceRect face;
                    face.bounding_box = cv::Rect(x1, y1, x2 - x1, y2 - y1);
                    face.confidence = confidence;
                    faces.push_back(face);
                }
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "FaceDetector: OpenCV exception during detection: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "FaceDetector: Exception during detection: " << e.what() << std::endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    last_detection_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();

    return faces;
}

}  // namespace greeter
