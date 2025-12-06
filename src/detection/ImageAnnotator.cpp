#include "detection/ImageAnnotator.h"
#include <sstream>
#include <iomanip>

cv::Scalar ImageAnnotator::getColorForSeverity(const std::string& severity) const {
    // BGR colors (OpenCV convention)
    if (severity == "high") {
        return cv::Scalar(0, 0, 255);    // Red
    } else if (severity == "medium") {
        return cv::Scalar(0, 165, 255);  // Orange
    } else {
        return cv::Scalar(0, 255, 0);    // Green
    }
}

void ImageAnnotator::annotateImage(cv::Mat& image, const std::vector<Defect>& defects) {
    if (image.empty()) return;

    for (const auto& defect : defects) {
        cv::Scalar color = getColorForSeverity(defect.severity);
        cv::Rect bbox = defect.getBoundingBox();

        // Ensure bounding box is within image bounds
        bbox.x = std::max(0, bbox.x);
        bbox.y = std::max(0, bbox.y);
        bbox.width = std::min(bbox.width, image.cols - bbox.x);
        bbox.height = std::min(bbox.height, image.rows - bbox.y);

        if (bbox.width <= 0 || bbox.height <= 0) {
            // If no bounding box, draw a circle at image_loc
            cv::Point center(static_cast<int>(defect.image_loc.x),
                           static_cast<int>(defect.image_loc.y));
            cv::circle(image, center, 20, color, line_thickness_);
        } else {
            // Draw rectangle
            cv::rectangle(image, bbox, color, line_thickness_);
        }

        // Build label text
        std::ostringstream label;
        label << defectTypeToString(defect.type);
        label << " (" << std::fixed << std::setprecision(0) << (defect.confidence * 100) << "%)";

        // Calculate text size for background
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX,
                                              font_scale_, 1, &baseline);

        // Position label above bounding box
        cv::Point text_org;
        if (bbox.width > 0 && bbox.height > 0) {
            text_org = cv::Point(bbox.x, bbox.y - 5);
        } else {
            text_org = cv::Point(static_cast<int>(defect.image_loc.x) - text_size.width / 2,
                                static_cast<int>(defect.image_loc.y) - 25);
        }

        // Ensure text stays within image bounds
        text_org.x = std::max(0, text_org.x);
        text_org.y = std::max(text_size.height + 2, text_org.y);

        // Draw background rectangle for text
        cv::rectangle(image,
                     cv::Point(text_org.x - 2, text_org.y - text_size.height - 2),
                     cv::Point(text_org.x + text_size.width + 2, text_org.y + baseline + 2),
                     color, cv::FILLED);

        // Draw text in white
        cv::putText(image, label.str(), text_org, cv::FONT_HERSHEY_SIMPLEX,
                   font_scale_, cv::Scalar(255, 255, 255), 1);

        // Draw description below if there's room
        if (!defect.description.empty() && bbox.height > 0) {
            std::string desc = defect.description;
            if (desc.length() > 40) {
                desc = desc.substr(0, 37) + "...";
            }
            cv::Point desc_org(bbox.x, bbox.y + bbox.height + text_size.height + 5);
            if (desc_org.y < image.rows - 5) {
                cv::putText(image, desc, desc_org, cv::FONT_HERSHEY_SIMPLEX,
                           font_scale_ * 0.8f, color, 1);
            }
        }
    }
}

bool ImageAnnotator::saveAnnotatedImage(const cv::Mat& image, const std::string& path) {
    if (image.empty()) return false;
    return cv::imwrite(path, image);
}
