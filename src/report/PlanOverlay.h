#pragma once

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "util/Types.h"
#include "detection/DefectTypes.h"

class PlanOverlay {
public:
    PlanOverlay() = default;

    // Add individual defect marker
    void addDefectMarker(const Point2D& plan_loc, int number, const std::string& severity);

    // Add markers from defect list (assigns numbers sequentially)
    void addDefectMarkers(const std::vector<Defect>& defects);

    // Clear all markers
    void clearMarkers();

    // Generate overlay image with markers drawn on plan
    // scale: multiplier for marker sizes (useful for different plan resolutions)
    // meters_per_pixel: conversion from plan coordinates (meters) to image pixels
    //                   Default 0.02 = 2cm per pixel. Use PlanInfo::resolution for accuracy.
    cv::Mat generateOverlay(const cv::Mat& plan_image, float scale = 1.0f,
                            float meters_per_pixel = 0.02f) const;

    // Configuration
    void setMarkerRadius(int radius) { marker_radius_ = radius; }
    void setFontScale(float scale) { font_scale_ = scale; }
    void setHighColor(cv::Scalar color) { color_high_ = color; }
    void setMediumColor(cv::Scalar color) { color_medium_ = color; }
    void setLowColor(cv::Scalar color) { color_low_ = color; }

    // Add legend to image showing severity colors
    void addLegend(cv::Mat& image) const;

    // Get marker count
    size_t getMarkerCount() const { return markers_.size(); }

private:
    struct Marker {
        Point2D location;   // Plan coordinates in meters
        int number;         // Display number (1, 2, 3...)
        std::string severity;  // "high", "medium", "low"
    };

    std::vector<Marker> markers_;

    // Visual configuration
    int marker_radius_ = 15;
    float font_scale_ = 0.5f;
    cv::Scalar color_high_ = cv::Scalar(0, 0, 255);      // Red (BGR)
    cv::Scalar color_medium_ = cv::Scalar(0, 165, 255);  // Orange (BGR)
    cv::Scalar color_low_ = cv::Scalar(0, 255, 255);     // Yellow (BGR)
    cv::Scalar color_text_ = cv::Scalar(255, 255, 255);  // White text

    // Helper to get color for severity
    cv::Scalar getColorForSeverity(const std::string& severity) const;
};
