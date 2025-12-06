#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "util/Types.h"

enum class DefectType {
    LOCATION_ERROR,    // Element in wrong location
    QUALITY_ISSUE,     // Scratch, crack, stain, poor workmanship
    SAFETY_HAZARD,     // Exposed wiring, missing covers
    MISSING_ELEMENT    // Expected element not present
};

struct Defect {
    std::string id;              // Unique ID (e.g., "def_001")
    DefectType type;             // Defect category
    std::string description;     // Human-readable description
    Point2D image_loc;           // Pixel coordinates in source image
    int bbox_x = 0;              // Bounding box top-left x
    int bbox_y = 0;              // Bounding box top-left y
    int bbox_width = 0;          // Bounding box width
    int bbox_height = 0;         // Bounding box height
    Point2D plan_loc;            // World coordinates on plan (meters)
    float confidence = 0.0f;     // 0.0 - 1.0
    std::string severity;        // "high", "medium", "low"
    std::string trade;           // "finishes", "mep", "structural"

    // Helper to get bounding box as cv::Rect for OpenCV operations
    cv::Rect getBoundingBox() const { return cv::Rect(bbox_x, bbox_y, bbox_width, bbox_height); }
};

// JSON serialization helpers
std::string defectTypeToString(DefectType type);
DefectType stringToDefectType(const std::string& str);

// nlohmann/json serialization
void to_json(nlohmann::json& j, const Defect& d);
void from_json(const nlohmann::json& j, Defect& d);
