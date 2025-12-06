#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "report/ReportGenerator.h"
#include "report/ReportTypes.h"
#include "report/PlanOverlay.h"

namespace fs = std::filesystem;

class ReportTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test directory
        test_dir_ = "test_data/report_test_" + std::to_string(std::time(nullptr));
        fs::create_directories(test_dir_);
        fs::create_directories(test_dir_ + "/images");

        // Create a simple test plan image
        test_plan_ = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::rectangle(test_plan_, cv::Point(50, 50), cv::Point(750, 550), cv::Scalar(200, 200, 200), cv::FILLED);
    }

    void TearDown() override {
        // Cleanup test directory
        if (fs::exists(test_dir_)) {
            fs::remove_all(test_dir_);
        }
    }

    // Create mock defects for testing
    std::vector<Defect> createMockDefects() {
        std::vector<Defect> defects;

        // High severity defect
        Defect d1;
        d1.id = "def_001";
        d1.type = DefectType::SAFETY_HAZARD;
        d1.description = "Exposed electrical wiring near entrance";
        d1.severity = "high";
        d1.trade = "mep";
        d1.plan_loc = {2.5f, 1.2f};
        d1.image_loc = {320, 240};
        d1.confidence = 0.95f;
        d1.source_image = "img_00000001.jpg";
        defects.push_back(d1);

        // Medium severity defect
        Defect d2;
        d2.id = "def_002";
        d2.type = DefectType::LOCATION_ERROR;
        d2.description = "Outlet misaligned by 5cm from plan";
        d2.severity = "medium";
        d2.trade = "mep";
        d2.plan_loc = {4.1f, 3.8f};
        d2.image_loc = {150, 300};
        d2.confidence = 0.87f;
        d2.source_image = "img_00000002.jpg";
        defects.push_back(d2);

        // Low severity defect
        Defect d3;
        d3.id = "def_003";
        d3.type = DefectType::QUALITY_ISSUE;
        d3.description = "Minor scratch on tile surface";
        d3.severity = "low";
        d3.trade = "finishes";
        d3.plan_loc = {6.0f, 2.5f};
        d3.image_loc = {400, 180};
        d3.confidence = 0.72f;
        d3.source_image = "img_00000003.jpg";
        defects.push_back(d3);

        return defects;
    }

    // Create mock InspectionReport
    InspectionReport createMockReport() {
        InspectionReport report;
        report.inspection_id = "test_insp_001";
        report.date = "2025-12-05";
        report.time = "10:30:00";
        report.plan_name = "floor_plan.png";
        report.trade_type = "finishes";
        report.coverage_percent = 87.5f;
        report.images_captured = 25;
        report.waypoints_visited = 15;
        report.total_waypoints = 18;
        report.plan_image = test_plan_.clone();
        report.defects = createMockDefects();
        report.robot_id = "G1-001";
        report.battery_at_start = 95.0f;
        report.battery_at_end = 72.0f;
        report.duration_minutes = 45.5f;
        report.computeSummary();
        return report;
    }

    std::string test_dir_;
    cv::Mat test_plan_;
};

// ============================================
// ReportTypes Tests
// ============================================

TEST_F(ReportTest, InspectionSummary_DefaultValues) {
    InspectionSummary summary;
    EXPECT_EQ(summary.total_defects, 0);
    EXPECT_EQ(summary.high_severity, 0);
    EXPECT_EQ(summary.medium_severity, 0);
    EXPECT_EQ(summary.low_severity, 0);
}

TEST_F(ReportTest, InspectionReport_ComputeSummary) {
    InspectionReport report = createMockReport();

    EXPECT_EQ(report.summary.total_defects, 3);
    EXPECT_EQ(report.summary.high_severity, 1);
    EXPECT_EQ(report.summary.medium_severity, 1);
    EXPECT_EQ(report.summary.low_severity, 1);
    EXPECT_EQ(report.summary.safety_hazards, 1);
    EXPECT_EQ(report.summary.location_errors, 1);
    EXPECT_EQ(report.summary.quality_issues, 1);
}

TEST_F(ReportTest, InspectionReport_SortDefectsBySeverity) {
    InspectionReport report = createMockReport();

    // Shuffle defects
    std::swap(report.defects[0], report.defects[2]);

    report.sortDefectsBySeverity();

    EXPECT_EQ(report.defects[0].severity, "high");
    EXPECT_EQ(report.defects[1].severity, "medium");
    EXPECT_EQ(report.defects[2].severity, "low");
}

// ============================================
// PlanOverlay Tests
// ============================================

TEST_F(ReportTest, PlanOverlay_AddDefectMarker) {
    PlanOverlay overlay;

    overlay.addDefectMarker({2.5f, 1.2f}, 1, "high");
    overlay.addDefectMarker({4.0f, 3.0f}, 2, "medium");

    EXPECT_EQ(overlay.getMarkerCount(), 2);
}

TEST_F(ReportTest, PlanOverlay_AddDefectMarkersFromList) {
    PlanOverlay overlay;
    auto defects = createMockDefects();

    overlay.addDefectMarkers(defects);

    EXPECT_EQ(overlay.getMarkerCount(), 3);
}

TEST_F(ReportTest, PlanOverlay_ClearMarkers) {
    PlanOverlay overlay;
    overlay.addDefectMarker({2.5f, 1.2f}, 1, "high");
    overlay.addDefectMarker({4.0f, 3.0f}, 2, "medium");

    overlay.clearMarkers();

    EXPECT_EQ(overlay.getMarkerCount(), 0);
}

TEST_F(ReportTest, PlanOverlay_GenerateOverlay_WithMarkers) {
    PlanOverlay overlay;
    overlay.addDefectMarker({2.5f, 1.2f}, 1, "high");
    overlay.addDefectMarker({4.0f, 3.0f}, 2, "medium");

    cv::Mat result = overlay.generateOverlay(test_plan_);

    EXPECT_FALSE(result.empty());
    EXPECT_EQ(result.rows, test_plan_.rows);
    EXPECT_EQ(result.cols, test_plan_.cols);
    EXPECT_EQ(result.channels(), 3);
}

TEST_F(ReportTest, PlanOverlay_GenerateOverlay_EmptyImage) {
    PlanOverlay overlay;
    cv::Mat empty_img;

    cv::Mat result = overlay.generateOverlay(empty_img);

    EXPECT_TRUE(result.empty());
}

TEST_F(ReportTest, PlanOverlay_GenerateOverlay_GrayscaleInput) {
    PlanOverlay overlay;
    cv::Mat gray_plan;
    cv::cvtColor(test_plan_, gray_plan, cv::COLOR_BGR2GRAY);

    overlay.addDefectMarker({2.5f, 1.2f}, 1, "high");
    cv::Mat result = overlay.generateOverlay(gray_plan);

    EXPECT_FALSE(result.empty());
    EXPECT_EQ(result.channels(), 3);  // Should convert to BGR
}

TEST_F(ReportTest, PlanOverlay_AddLegend) {
    PlanOverlay overlay;
    cv::Mat image = test_plan_.clone();

    overlay.addLegend(image);

    // Legend should have been added (can't easily verify visually in unit test)
    EXPECT_FALSE(image.empty());
}

TEST_F(ReportTest, PlanOverlay_MarkerPositionAccuracy) {
    // Test that markers appear at correct pixel positions based on plan coordinates
    // Using meters_per_pixel = 0.01 (1cm per pixel = 100 pixels per meter)
    float meters_per_pixel = 0.01f;

    // Create a 500x500 image
    cv::Mat plan(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    PlanOverlay overlay;
    // Add marker at (1.0m, 2.0m) which should appear at pixel (100, 200) with 0.01 m/pixel
    overlay.addDefectMarker({1.0f, 2.0f}, 1, "high");

    cv::Mat result = overlay.generateOverlay(plan, 1.0f, meters_per_pixel);
    EXPECT_FALSE(result.empty());

    // Check that the marker was drawn at approximately the correct position
    // The marker is a red circle (BGR: 0, 0, 255) at pixel position (100, 200)
    // Check a pixel slightly offset from center (center has white text on it)
    // Marker radius is 15, so check at +10 pixels offset
    cv::Vec3b pixel_at_marker = result.at<cv::Vec3b>(200 + 10, 100);  // Note: Mat access is (row, col) = (y, x)

    // The pixel should not be pure white (255, 255, 255) since marker was drawn there
    bool is_marked = (pixel_at_marker[0] != 255 || pixel_at_marker[1] != 255 || pixel_at_marker[2] != 255);
    EXPECT_TRUE(is_marked) << "Marker should be visible near expected pixel position (100, 200)";

    // Verify a pixel far from the marker is still white (untouched)
    cv::Vec3b pixel_far_away = result.at<cv::Vec3b>(400, 400);
    bool is_white = (pixel_far_away[0] == 255 && pixel_far_away[1] == 255 && pixel_far_away[2] == 255);
    EXPECT_TRUE(is_white) << "Pixel far from marker should remain white";
}

TEST_F(ReportTest, PlanOverlay_DifferentResolutions) {
    // Test that different resolutions produce markers at different pixel positions
    cv::Mat plan(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    PlanOverlay overlay;
    overlay.addDefectMarker({2.0f, 2.0f}, 1, "high");

    // At 0.01 m/pixel (100 px/m): marker at (2m, 2m) -> pixel (200, 200)
    // Check at +10 offset to avoid white text at center
    cv::Mat result1 = overlay.generateOverlay(plan, 1.0f, 0.01f);
    cv::Vec3b px1 = result1.at<cv::Vec3b>(200 + 10, 200);
    bool marked1 = (px1[0] != 255 || px1[1] != 255 || px1[2] != 255);
    EXPECT_TRUE(marked1) << "At 0.01 m/pixel, marker should be near (200, 200)";

    // At 0.02 m/pixel (50 px/m): marker at (2m, 2m) -> pixel (100, 100)
    cv::Mat result2 = overlay.generateOverlay(plan, 1.0f, 0.02f);
    cv::Vec3b px2 = result2.at<cv::Vec3b>(100 + 10, 100);
    bool marked2 = (px2[0] != 255 || px2[1] != 255 || px2[2] != 255);
    EXPECT_TRUE(marked2) << "At 0.02 m/pixel, marker should be near (100, 100)";
}

// ============================================
// ReportGenerator Tests
// ============================================

TEST_F(ReportTest, ReportGenerator_GenerateFilename) {
    std::string filename = ReportGenerator::generateFilename("insp_001", "floor_plan.png", "2025-12-05");

    EXPECT_EQ(filename, "inspection_2025-12-05_floor_plan.pdf");
}

TEST_F(ReportTest, ReportGenerator_GenerateFilename_EmptyPlanName) {
    std::string filename = ReportGenerator::generateFilename("insp_001", "", "2025-12-05");

    EXPECT_EQ(filename, "inspection_2025-12-05_inspection.pdf");
}

TEST_F(ReportTest, ReportGenerator_GenerateFilename_PathInPlanName) {
    std::string filename = ReportGenerator::generateFilename("insp_001", "/path/to/floor_plan.png", "2025-12-05");

    EXPECT_EQ(filename, "inspection_2025-12-05_floor_plan.pdf");
}

TEST_F(ReportTest, ReportGenerator_ExportPunchListCsv) {
    InspectionReport report = createMockReport();
    std::string csv_path = test_dir_ + "/punch_list.csv";

    bool result = ReportGenerator::exportPunchListCsv(report, csv_path);

    EXPECT_TRUE(result);
    EXPECT_TRUE(fs::exists(csv_path));

    // Verify CSV content
    std::ifstream file(csv_path);
    std::string line;

    // Check header
    std::getline(file, line);
    EXPECT_TRUE(line.find("\"ID\"") != std::string::npos);
    EXPECT_TRUE(line.find("\"Type\"") != std::string::npos);
    EXPECT_TRUE(line.find("\"Severity\"") != std::string::npos);
    EXPECT_TRUE(line.find("\"Description\"") != std::string::npos);

    // Check data rows exist
    int row_count = 0;
    while (std::getline(file, line)) {
        row_count++;
    }
    EXPECT_EQ(row_count, 3);  // 3 defects
}

TEST_F(ReportTest, ReportGenerator_ExportPunchListCsv_EmptyDefects) {
    InspectionReport report = createMockReport();
    report.defects.clear();
    std::string csv_path = test_dir_ + "/empty_punch_list.csv";

    bool result = ReportGenerator::exportPunchListCsv(report, csv_path);

    EXPECT_TRUE(result);
    EXPECT_TRUE(fs::exists(csv_path));

    // Verify only header exists
    std::ifstream file(csv_path);
    std::string line;
    int line_count = 0;
    while (std::getline(file, line)) {
        line_count++;
    }
    EXPECT_EQ(line_count, 1);  // Header only
}

TEST_F(ReportTest, ReportGenerator_ExportPunchListCsv_SpecialCharacters) {
    InspectionReport report;
    report.inspection_id = "test";

    Defect d;
    d.id = "def_001";
    d.type = DefectType::QUALITY_ISSUE;
    d.description = "Quote test \"with quotes\" and commas, here";
    d.severity = "high";
    d.trade = "test";
    d.plan_loc = {1.0f, 1.0f};
    report.defects.push_back(d);

    std::string csv_path = test_dir_ + "/special_chars.csv";
    bool result = ReportGenerator::exportPunchListCsv(report, csv_path);

    EXPECT_TRUE(result);

    // Verify quotes are escaped
    std::ifstream file(csv_path);
    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
    EXPECT_TRUE(content.find("\"\"with quotes\"\"") != std::string::npos);
}

TEST_F(ReportTest, ReportGenerator_ExportPunchListCsv_SourceImage) {
    InspectionReport report = createMockReport();
    std::string csv_path = test_dir_ + "/punch_list_images.csv";

    bool result = ReportGenerator::exportPunchListCsv(report, csv_path);
    EXPECT_TRUE(result);

    // Verify source_image is in the CSV output
    std::ifstream file(csv_path);
    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

    // Should contain the actual source image filenames, not defect IDs
    EXPECT_TRUE(content.find("img_00000001.jpg") != std::string::npos)
        << "CSV should contain source image filename";
    EXPECT_TRUE(content.find("img_00000002.jpg") != std::string::npos);
    EXPECT_TRUE(content.find("img_00000003.jpg") != std::string::npos);
}

TEST_F(ReportTest, ReportGenerator_ExportPunchListCsv_MissingSourceImage) {
    // Test that missing source_image falls back to "N/A" not fake filename
    InspectionReport report;
    report.inspection_id = "test";

    Defect d;
    d.id = "def_001";
    d.type = DefectType::QUALITY_ISSUE;
    d.description = "Test defect";
    d.severity = "high";
    d.trade = "test";
    d.plan_loc = {1.0f, 1.0f};
    d.source_image = "";  // Empty - should become "N/A"
    report.defects.push_back(d);

    std::string csv_path = test_dir_ + "/missing_source.csv";
    bool result = ReportGenerator::exportPunchListCsv(report, csv_path);
    EXPECT_TRUE(result);

    std::ifstream file(csv_path);
    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

    // Should contain "N/A" not "def_001.jpg"
    EXPECT_TRUE(content.find("\"N/A\"") != std::string::npos)
        << "CSV should use N/A for missing source_image";
    EXPECT_TRUE(content.find("def_001.jpg") == std::string::npos)
        << "CSV should NOT contain fake filename";
}

TEST_F(ReportTest, ReportGenerator_Generate_CreatesPdf) {
    InspectionReport report = createMockReport();
    std::string output_dir = test_dir_ + "/output";

    ReportGenerator generator;
    bool result = generator.generate(report, output_dir);

    EXPECT_TRUE(result);

    // Check PDF file exists
    std::string expected_pdf = output_dir + "/inspection_2025-12-05_floor_plan.pdf";
    EXPECT_TRUE(fs::exists(expected_pdf));

    // Verify file has content
    EXPECT_GT(fs::file_size(expected_pdf), 0);
}

TEST_F(ReportTest, ReportGenerator_Generate_NoPlanImage) {
    InspectionReport report = createMockReport();
    report.plan_image = cv::Mat();  // Clear plan image
    std::string output_dir = test_dir_ + "/output_no_plan";

    ReportGenerator generator;
    bool result = generator.generate(report, output_dir);

    EXPECT_TRUE(result);  // Should still generate without plan overlay

    std::string expected_pdf = output_dir + "/inspection_2025-12-05_floor_plan.pdf";
    EXPECT_TRUE(fs::exists(expected_pdf));
}

TEST_F(ReportTest, ReportGenerator_Generate_NoDefects) {
    InspectionReport report = createMockReport();
    report.defects.clear();
    report.computeSummary();
    std::string output_dir = test_dir_ + "/output_no_defects";

    ReportGenerator generator;
    bool result = generator.generate(report, output_dir);

    EXPECT_TRUE(result);

    std::string expected_pdf = output_dir + "/inspection_2025-12-05_floor_plan.pdf";
    EXPECT_TRUE(fs::exists(expected_pdf));
}

TEST_F(ReportTest, ReportGenerator_Configuration) {
    ReportGenerator generator;

    // Test configuration methods don't crash
    generator.setPageSize(612.0f, 792.0f);  // US Letter
    generator.setMargins(72, 72, 72, 72);   // 1 inch margins
    generator.setLogoPath("/path/to/logo.png");
    generator.setIncludePhotos(false);
    generator.setMaxPhotosPerPage(6);

    InspectionReport report = createMockReport();
    std::string output_dir = test_dir_ + "/output_configured";

    bool result = generator.generate(report, output_dir);
    EXPECT_TRUE(result);
}

// ============================================
// Integration Tests
// ============================================

TEST_F(ReportTest, FullWorkflow_MockSession) {
    // Create mock analysis_results.json
    std::string analysis_json = R"({
        "session_id": "test_session",
        "images_processed": 5,
        "total_defects": 2,
        "defects_by_image": [
            {
                "image": "img_00000001.jpg",
                "defects": [
                    {
                        "id": "def_001",
                        "type": "QUALITY_ISSUE",
                        "description": "Test defect 1",
                        "confidence": 0.85,
                        "severity": "high",
                        "trade": "finishes",
                        "image_location": {"x": 100, "y": 200},
                        "plan_location": {"x": 2.0, "y": 1.5}
                    }
                ]
            },
            {
                "image": "img_00000002.jpg",
                "defects": [
                    {
                        "id": "def_002",
                        "type": "LOCATION_ERROR",
                        "description": "Test defect 2",
                        "confidence": 0.92,
                        "severity": "medium",
                        "trade": "mep",
                        "image_location": {"x": 300, "y": 400},
                        "plan_location": {"x": 4.0, "y": 3.0}
                    }
                ]
            }
        ]
    })";

    std::ofstream analysis_file(test_dir_ + "/analysis_results.json");
    analysis_file << analysis_json;
    analysis_file.close();

    // Save plan image
    cv::imwrite(test_dir_ + "/plan.png", test_plan_);

    // Load and generate report
    InspectionReport report;
    bool loaded = loadInspectionReport(test_dir_, report);
    EXPECT_TRUE(loaded);
    EXPECT_EQ(report.defects.size(), 2);
    EXPECT_EQ(report.summary.total_defects, 2);

    // Generate PDF
    std::string output_dir = test_dir_ + "/output";
    ReportGenerator generator;
    bool generated = generator.generate(report, output_dir);
    EXPECT_TRUE(generated);

    // Export CSV
    std::string csv_path = output_dir + "/punch_list.csv";
    bool csv_exported = ReportGenerator::exportPunchListCsv(report, csv_path);
    EXPECT_TRUE(csv_exported);

    // Verify both files exist
    EXPECT_TRUE(fs::exists(output_dir));
    EXPECT_TRUE(fs::exists(csv_path));
}

TEST_F(ReportTest, LoadInspectionReport_MissingAnalysisFile) {
    // Create directory without analysis_results.json
    std::string empty_session = test_dir_ + "/empty_session";
    fs::create_directories(empty_session);

    InspectionReport report;
    bool loaded = loadInspectionReport(empty_session, report);

    EXPECT_TRUE(loaded);  // Should succeed but with no defects
    EXPECT_EQ(report.defects.size(), 0);
}

TEST_F(ReportTest, LoadInspectionReport_InvalidJson) {
    // Create invalid JSON file
    std::ofstream file(test_dir_ + "/analysis_results.json");
    file << "{ invalid json }";
    file.close();

    InspectionReport report;
    bool loaded = loadInspectionReport(test_dir_, report);

    EXPECT_FALSE(loaded);  // Should fail on invalid JSON
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
