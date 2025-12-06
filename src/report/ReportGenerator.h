#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <hpdf.h>
#include <opencv2/opencv.hpp>
#include "report/ReportTypes.h"
#include "report/PlanOverlay.h"
#include "detection/DefectTypes.h"

// Error state for tracking libharu failures
struct HpdfErrorState {
    bool has_error = false;
    HPDF_STATUS error_no = 0;
    HPDF_STATUS detail_no = 0;

    void clear() { has_error = false; error_no = 0; detail_no = 0; }
    std::string toString() const {
        std::ostringstream ss;
        ss << "HPDF Error 0x" << std::hex << error_no << ", detail: 0x" << detail_no;
        return ss.str();
    }
};

// Thread Safety: ReportGenerator is NOT thread-safe.
// Each thread should use its own instance. Do not share HPDF_Doc handles between threads.
class ReportGenerator {
public:
    ReportGenerator();
    ~ReportGenerator();

    // Main generation function - creates PDF with all sections (AC1)
    // output_dir: where to save the PDF (e.g., "data/reports/insp_001")
    // session_dir: inspection session directory for loading photos (e.g., "data/inspections/insp_001")
    //              If empty, photos won't be loaded for gallery
    // Returns true on success, false on failure
    bool generate(const InspectionReport& report, const std::string& output_dir,
                  const std::string& session_dir = "");

    // Configuration
    void setPageSize(float width, float height);  // Default A4
    void setMargins(float top, float bottom, float left, float right);
    void setLogoPath(const std::string& path) { logo_path_ = path; }
    void setIncludePhotos(bool include) { include_photos_ = include; }
    void setMaxPhotosPerPage(int max) { max_photos_per_page_ = max; }

    // CSV export (AC4) - static for standalone use
    static bool exportPunchListCsv(const InspectionReport& report, const std::string& path);

    // Helper to generate output filename following pattern: inspection_<date>_<plan>.pdf (AC5)
    static std::string generateFilename(const std::string& inspection_id,
                                         const std::string& plan_name,
                                         const std::string& date);

    // Get last error message
    std::string getLastError() const { return last_error_; }

private:
    // PDF generation helpers
    bool initDocument();
    void cleanup();

    // Section generation
    void addTitlePage(const InspectionReport& report);
    void addSummarySection(const InspectionReport& report);
    void addPlanOverlayPage(const InspectionReport& report);
    void addPhotoGallery(const InspectionReport& report);
    void addPunchListTable(const InspectionReport& report);
    void addPageFooter(int page_num, int total_pages);

    // Image helpers
    bool addImageToPage(const cv::Mat& image, float x, float y, float max_width, float max_height);
    HPDF_Image loadImageToPdf(const cv::Mat& image);

    // Table helpers
    float drawTableHeader(const std::vector<std::string>& headers,
                          const std::vector<float>& col_widths,
                          float start_y);
    float drawTableRow(const std::vector<std::string>& cells,
                       const std::vector<float>& col_widths,
                       float y, bool alternating);

    // Text helpers
    void drawText(float x, float y, const std::string& text, HPDF_Font font, float size);
    void drawTextCentered(float x, float y, float width, const std::string& text, HPDF_Font font, float size);

    // New page helper
    HPDF_Page addNewPage();

    // CSV helper
    static std::string escapeCSV(const std::string& s);

    // libharu document and current page
    HPDF_Doc pdf_ = nullptr;
    HPDF_Page current_page_ = nullptr;
    HpdfErrorState error_state_;

    // Fonts
    HPDF_Font font_regular_ = nullptr;
    HPDF_Font font_bold_ = nullptr;

    // Page configuration
    float page_width_ = 595.276f;   // A4 width in points
    float page_height_ = 841.890f;  // A4 height in points
    float margin_top_ = 50.0f;
    float margin_bottom_ = 50.0f;
    float margin_left_ = 50.0f;
    float margin_right_ = 50.0f;

    // Options
    std::string logo_path_;
    bool include_photos_ = true;
    int max_photos_per_page_ = 4;

    // Page tracking
    int total_pages_ = 0;
    std::vector<HPDF_Page> pages_;

    // Error tracking
    std::string last_error_;

    // Session directory for loading images
    std::string session_dir_;
};
