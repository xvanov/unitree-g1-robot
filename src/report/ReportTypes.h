#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include "detection/DefectTypes.h"
#include "capture/ImageCapture.h"
#include "plan/PlanManager.h"

// Summary of defects by severity and type
struct InspectionSummary {
    int total_defects = 0;
    int high_severity = 0;
    int medium_severity = 0;
    int low_severity = 0;
    int location_errors = 0;
    int quality_issues = 0;
    int safety_hazards = 0;
    int missing_elements = 0;
};

// HIGH-3 FIX: Removed unused ReportConfig struct - configuration is handled
// directly by ReportGenerator class via setPageSize(), setMargins(), etc.

// Complete inspection report data
struct InspectionReport {
    // Metadata (AC8)
    std::string inspection_id;
    std::string date;               // Format: YYYY-MM-DD
    std::string time;               // Format: HH:MM:SS
    std::string plan_name;
    std::string trade_type;

    // Coverage (AC8)
    float coverage_percent = 0.0f;
    int images_captured = 0;
    int waypoints_visited = 0;
    int total_waypoints = 0;

    // Plan data (AC2)
    cv::Mat plan_image;
    PlanInfo plan_info;

    // Defects sorted by severity (AC7)
    std::vector<Defect> defects;
    InspectionSummary summary;

    // Photo references - paths relative to session dir (AC6)
    std::vector<ImageMetadata> captured_images;

    // Robot info (AC8)
    std::string robot_id;
    float battery_at_start = 0.0f;
    float battery_at_end = 0.0f;
    float duration_minutes = 0.0f;

    // Helper to sort defects by severity (high > medium > low)
    void sortDefectsBySeverity() {
        std::sort(defects.begin(), defects.end(), [](const Defect& a, const Defect& b) {
            auto severityRank = [](const std::string& s) {
                if (s == "high") return 0;
                if (s == "medium") return 1;
                if (s == "low") return 2;
                return 3;
            };
            return severityRank(a.severity) < severityRank(b.severity);
        });
    }

    // Helper to compute summary from defects
    void computeSummary() {
        summary = InspectionSummary{};
        summary.total_defects = static_cast<int>(defects.size());

        for (const auto& d : defects) {
            // Count by severity
            if (d.severity == "high") summary.high_severity++;
            else if (d.severity == "medium") summary.medium_severity++;
            else if (d.severity == "low") summary.low_severity++;

            // Count by type
            switch (d.type) {
                case DefectType::LOCATION_ERROR:
                    summary.location_errors++;
                    break;
                case DefectType::QUALITY_ISSUE:
                    summary.quality_issues++;
                    break;
                case DefectType::SAFETY_HAZARD:
                    summary.safety_hazards++;
                    break;
                case DefectType::MISSING_ELEMENT:
                    summary.missing_elements++;
                    break;
            }
        }
    }
};

// Load inspection report data from session directory
// Uses from_json() from DefectTypes.cpp for defect parsing
// IMPORTANT: DO NOT call VlmClient or detection functions here (they require curl init)
inline bool loadInspectionReport(const std::string& session_dir,
                                  InspectionReport& report,
                                  PlanManager* plan_manager = nullptr) {
    namespace fs = std::filesystem;

    // Extract inspection ID from directory name
    report.inspection_id = fs::path(session_dir).filename().string();

    // Load analysis results (defects from Story 1-8)
    std::string analysis_path = session_dir + "/analysis_results.json";
    if (fs::exists(analysis_path)) {
        std::ifstream file(analysis_path);
        if (!file.is_open()) {
            std::cerr << "[REPORT] Error: Cannot open " << analysis_path << std::endl;
            return false;
        }

        try {
            nlohmann::json j;
            file >> j;

            // Parse defects using from_json() from DefectTypes.cpp
            // JSON format matches Story 1-8 output (see story dev notes)
            if (j.contains("defects_by_image")) {
                for (const auto& img_entry : j["defects_by_image"]) {
                    if (!img_entry.contains("defects")) continue;

                    // Get the source image for all defects in this entry
                    std::string source_img = img_entry.value("image", "");

                    for (const auto& d_json : img_entry["defects"]) {
                        Defect d;
                        // Use the existing from_json() function from DefectTypes.cpp
                        // MEDIUM-1 FIX: Removed duplicate plan_location parsing - from_json() already handles it
                        from_json(d_json, d);

                        // Set source image from parent entry if not already set
                        if (d.source_image.empty()) {
                            d.source_image = source_img;
                        }

                        report.defects.push_back(d);
                    }
                }
            }

            report.images_captured = j.value("images_processed", 0);

            // Handle empty defects case gracefully
            if (report.defects.empty()) {
                std::cout << "[REPORT] No defects found in session - generating clean report" << std::endl;
            }
        } catch (const nlohmann::json::exception& e) {
            std::cerr << "[REPORT] JSON parse error: " << e.what() << std::endl;
            return false;
        }
    } else {
        std::cerr << "[REPORT] Warning: analysis_results.json not found at " << analysis_path << std::endl;
        std::cout << "[REPORT] Generating report with no defect data" << std::endl;
    }

    // Load captured images metadata
    std::string images_dir = session_dir + "/images";
    if (fs::exists(images_dir)) {
        for (const auto& entry : fs::directory_iterator(images_dir)) {
            if (entry.path().extension() == ".json") {
                ImageMetadata meta;
                if (ImageCapture::loadMetadata(entry.path().string(), meta)) {
                    report.captured_images.push_back(meta);
                }
            }
        }
    }

    // Load plan info if available
    if (plan_manager && plan_manager->isLoaded()) {
        report.plan_info = plan_manager->getPlanInfo();
        report.plan_image = plan_manager->getPlanImage().clone();
        report.plan_name = report.plan_info.path;
        report.trade_type = report.plan_info.trade_type;
    } else {
        // Try to load plan from session directory
        std::string plan_path = session_dir + "/plan.png";
        if (fs::exists(plan_path)) {
            report.plan_image = cv::imread(plan_path);
            report.plan_name = "plan.png";
        }
    }

    // Set date/time from session directory name or current time
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time_t);
    char date_buf[32], time_buf[32];
    std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", &tm);
    std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &tm);
    report.date = date_buf;
    report.time = time_buf;

    // Compute summary
    report.computeSummary();
    report.sortDefectsBySeverity();

    return true;
}
