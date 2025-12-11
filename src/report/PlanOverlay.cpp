#include "report/PlanOverlay.h"
#include <iostream>

void PlanOverlay::addDefectMarker(const Point2D& plan_loc, int number, const std::string& severity) {
    markers_.push_back({plan_loc, number, severity});
}

void PlanOverlay::addDefectMarkers(const std::vector<Defect>& defects) {
    int number = 1;
    for (const auto& d : defects) {
        addDefectMarker(d.plan_loc, number++, d.severity);
    }
}

void PlanOverlay::clearMarkers() {
    markers_.clear();
}

cv::Scalar PlanOverlay::getColorForSeverity(const std::string& severity) const {
    if (severity == "high") return color_high_;
    if (severity == "medium") return color_medium_;
    if (severity == "low") return color_low_;
    return color_low_;  // Default to low severity color
}

cv::Mat PlanOverlay::generateOverlay(const cv::Mat& plan_image, float scale,
                                      float meters_per_pixel) const {
    if (plan_image.empty()) {
        std::cerr << "[OVERLAY] Error: Plan image is empty" << std::endl;
        return cv::Mat();
    }

    // Create a copy to draw on
    cv::Mat overlay;
    if (plan_image.channels() == 1) {
        cv::cvtColor(plan_image, overlay, cv::COLOR_GRAY2BGR);
    } else {
        overlay = plan_image.clone();
    }

    // Calculate scale factor for converting meters to pixels
    // Use provided meters_per_pixel (from PlanInfo::resolution) for accurate positioning
    float pixels_per_meter = 1.0f / meters_per_pixel;

    // Adjust marker size based on image resolution and scale parameter
    int radius = static_cast<int>(marker_radius_ * scale);
    float font_scale = font_scale_ * scale;
    int thickness = std::max(1, static_cast<int>(2 * scale));

    // Draw each marker
    for (const auto& marker : markers_) {
        // Convert plan coordinates (meters) to pixel coordinates
        // Note: Plan coordinate (0,0) is typically top-left in image coordinates
        int px = static_cast<int>(marker.location.x * pixels_per_meter);
        int py = static_cast<int>(marker.location.y * pixels_per_meter);

        // Skip if outside image bounds
        if (px < 0 || px >= overlay.cols || py < 0 || py >= overlay.rows) {
            continue;
        }

        cv::Point center(px, py);
        cv::Scalar color = getColorForSeverity(marker.severity);

        // Draw filled circle with border
        cv::circle(overlay, center, radius, color, cv::FILLED);
        cv::circle(overlay, center, radius, cv::Scalar(0, 0, 0), thickness);  // Black border

        // Draw number text centered in circle
        std::string text = std::to_string(marker.number);
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);

        cv::Point text_pos(
            center.x - text_size.width / 2,
            center.y + text_size.height / 2
        );

        // Draw text with outline for visibility
        cv::putText(overlay, text, text_pos, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                    cv::Scalar(0, 0, 0), thickness + 1);  // Black outline
        cv::putText(overlay, text, text_pos, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                    color_text_, thickness);  // White text
    }

    return overlay;
}

void PlanOverlay::addLegend(cv::Mat& image) const {
    if (image.empty()) return;

    int legend_width = 120;
    int legend_height = 80;
    int legend_x = image.cols - legend_width - 10;
    int legend_y = 10;

    // Draw semi-transparent background
    cv::Mat roi = image(cv::Rect(legend_x, legend_y, legend_width, legend_height));
    cv::Mat overlay = cv::Mat::zeros(roi.size(), roi.type());
    overlay.setTo(cv::Scalar(255, 255, 255));
    cv::addWeighted(roi, 0.3, overlay, 0.7, 0, roi);

    // Draw border
    cv::rectangle(image, cv::Rect(legend_x, legend_y, legend_width, legend_height),
                  cv::Scalar(0, 0, 0), 1);

    // Draw legend items
    int y_offset = legend_y + 20;
    int circle_x = legend_x + 15;
    int text_x = legend_x + 35;

    // High severity
    cv::circle(image, cv::Point(circle_x, y_offset), 8, color_high_, cv::FILLED);
    cv::putText(image, "High", cv::Point(text_x, y_offset + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    y_offset += 20;

    // Medium severity
    cv::circle(image, cv::Point(circle_x, y_offset), 8, color_medium_, cv::FILLED);
    cv::putText(image, "Medium", cv::Point(text_x, y_offset + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    y_offset += 20;

    // Low severity
    cv::circle(image, cv::Point(circle_x, y_offset), 8, color_low_, cv::FILLED);
    cv::putText(image, "Low", cv::Point(text_x, y_offset + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
}
