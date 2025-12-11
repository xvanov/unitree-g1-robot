#pragma once

#include <opencv2/opencv.hpp>
#include "detection/DefectTypes.h"

class ImageAnnotator {
public:
    ImageAnnotator() = default;

    // Annotate image with defect bounding boxes and labels
    void annotateImage(cv::Mat& image, const std::vector<Defect>& defects);

    // Configuration
    void setFontScale(float scale) { font_scale_ = scale; }
    void setLineThickness(int thickness) { line_thickness_ = thickness; }

    // Save annotated image
    static bool saveAnnotatedImage(const cv::Mat& image, const std::string& path);

private:
    cv::Scalar getColorForSeverity(const std::string& severity) const;

    float font_scale_ = 0.6f;
    int line_thickness_ = 2;
};
