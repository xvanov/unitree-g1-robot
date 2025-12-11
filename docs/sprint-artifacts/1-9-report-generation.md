# Story 1.9: Report Generation

**Status:** Done

---

## Quick Reference

**New files to create:**
- `src/report/ReportGenerator.h` / `.cpp` - PDF report generation with libharu
- `src/report/ReportTypes.h` - Report data structures and enums
- `src/report/PlanOverlay.h` / `.cpp` - Draw defect markers on plan image
- `test/test_report.cpp` - Unit tests for report generation
- `scripts/create_test_report_data.sh` - Generate mock inspection data for testing

**Files to modify:**
- `CMakeLists.txt` - Add `report` library and `test_report`
- `src/main.cpp` - Add `--generate-report` CLI option

**Key classes:**
| Class | Purpose |
|-------|---------|
| `ReportGenerator` | Create PDF with summary, plan overlay, photos, defect list, punch list |
| `PlanOverlay` | Draw numbered markers on plan image at defect locations |
| `InspectionReport` | Data struct containing all inspection results for report |

**Key function signatures:**
```cpp
// ReportGenerator - main entry point
bool ReportGenerator::generate(const InspectionReport& report, const std::string& output_dir);
static bool ReportGenerator::exportPunchListCsv(const InspectionReport& report, const std::string& path);

// PlanOverlay - marker drawing
cv::Mat PlanOverlay::generateOverlay(const cv::Mat& plan_image, float scale = 1.0f) const;

// Data loading from Story 1-8 output
bool loadInspectionReport(const std::string& session_dir, InspectionReport& report, PlanManager* plan_manager);
```

**Primary acceptance criteria:** AC1 (PDF generated with all sections), AC2 (plan overlay with markers), AC3 (punch list table), AC4 (CSV export)

**Prerequisites:** Story 1-8 (VLM Defect Detection) must be complete for defect data

**Thread Safety:** ReportGenerator is NOT thread-safe. Each thread should use its own instance. Do not share `HPDF_Doc` handles between threads.

**Curl Note:** This story does NOT require curl initialization - it's pure offline PDF generation. Do NOT call VlmClient or detection module functions during report generation.

---

## Story

As an **operator**,
I want **PDF reports with punch lists**,
So that **I can share findings with crews**.

---

## Acceptance Criteria

1. **AC1:** PDF generated with all sections: summary, plan overlay, photos, defect list, punch list
2. **AC2:** Plan overlay shows defect markers with numbered callouts at defect locations
3. **AC3:** Punch list in table format with defect details
4. **AC4:** CSV export works for punch list (importable to spreadsheets)
5. **AC5:** Report filename follows pattern: `inspection_<date>_<plan>.pdf`
6. **AC6:** Photo thumbnails included with annotations showing defect locations
7. **AC7:** Defects grouped and sorted by severity (high, medium, low)
8. **AC8:** Report metadata includes inspection ID, date, robot info, coverage percentage

---

## Tasks / Subtasks

- [x] **Task 1: Define Report Data Structures** (AC: 1, 8)
  - [x] 1.1 Create `src/report/ReportTypes.h`
    - `InspectionReport` struct with all report data
    - `ReportConfig` struct for customization options
    - Helper types for report sections
  - [x] 1.2 Include fields for: inspection_id, date, plan_name, coverage_percent, defects, images, robot_info

- [x] **Task 2: Implement PlanOverlay Class** (AC: 2)
  - [x] 2.1 Create `src/report/PlanOverlay.h` - overlay interface
    - `addDefectMarker(Point2D location, int number, const std::string& severity)`
    - `generateOverlay(const cv::Mat& plan_image)` -> cv::Mat with markers
    - `setMarkerSize(int pixels)` - size of numbered circles
    - `setColors(high, medium, low)` - severity colors
  - [x] 2.2 Create `src/report/PlanOverlay.cpp` - implementation
    - Draw numbered circles at defect plan coordinates
    - Color-code by severity (red=high, orange=medium, yellow=low)
    - Scale markers appropriately for plan resolution
    - Add legend showing severity colors

- [x] **Task 3: Implement ReportGenerator Class** (AC: 1, 2, 3, 5, 6, 7, 8)
  - [x] 3.1 Create `src/report/ReportGenerator.h` - main report interface
    - `ReportGenerator()` constructor
    - `generate(const InspectionReport& report, const std::string& output_path)` -> bool
    - `setPageSize(width, height)` - A4 default
    - `setMargins(top, bottom, left, right)`
    - `setLogoPath(path)` - optional company logo
  - [x] 3.2 Create `src/report/ReportGenerator.cpp` - libharu implementation
    - Initialize PDF document with libharu (HPDF_New)
    - Add title page with inspection metadata
    - Add executive summary section
    - Add plan overlay page (convert cv::Mat to PDF image)
    - Add defect gallery with thumbnails
    - Add punch list table
    - Save PDF to output path
  - [x] 3.3 Implement helper methods:
    - `addTitlePage()` - inspection ID, date, plan name, coverage
    - `addSummarySection()` - defect counts by severity and type
    - `addPlanOverlayPage()` - annotated floor plan
    - `addPhotoGallery()` - thumbnails with defect info
    - `addPunchListTable()` - sortable defect list
    - `addFooter()` - page numbers

- [x] **Task 4: Implement CSV Export** (AC: 4)
  - [x] 4.1 Add `exportPunchListCsv(const InspectionReport& report, const std::string& path)`
    - CSV columns: ID, Type, Severity, Location (x,y), Description, Image, Trade
    - Proper escaping for CSV special characters
    - UTF-8 encoding
  - [x] 4.2 Optionally export alongside PDF automatically

- [x] **Task 5: libharu PDF Implementation Details** (AC: 1, 3)
  - [x] 5.1 Initialize libharu correctly
    - `HPDF_New()` with error handler
    - Set compression mode for smaller files
    - Set UTF-8 encoding for text
  - [x] 5.2 Add images to PDF
    - Convert cv::Mat to PNG buffer
    - Load PNG into PDF with `HPDF_LoadPngImageFromMem()`
    - Scale images to fit page
  - [x] 5.3 Create tables with libharu
    - Manual table drawing with lines and text cells
    - Alternating row colors for readability
    - Header row styling

- [x] **Task 6: CMake Integration** (AC: 1-8)
  - [x] 6.1 Update `CMakeLists.txt`
    - Add `report` library with ReportGenerator.cpp, PlanOverlay.cpp
    - Link libharu (`${HPDF_LIBRARY}`)
    - Link OpenCV for image handling
    - Link nlohmann_json for data serialization
    - Add `test_report` unit test

- [x] **Task 7: CLI Integration** (AC: 1, 5)
  - [x] 7.1 Update `src/main.cpp`
    - Add `--generate-report --inspection <id>` CLI option
    - Load defects from inspection session directory
    - Generate PDF to `data/reports/<inspection_id>/`
    - Print output path on success

- [x] **Task 8: Unit Tests** (AC: 1-4)
  - [x] 8.1 Create `test/test_report.cpp`
    - Test ReportGenerator with mock data
    - Test PlanOverlay marker placement
    - Test CSV export format
    - Test filename generation pattern
    - Test defect sorting by severity
    - Test PDF file creation (verify file exists and has content)

- [x] **Task 9: Integration with Inspection Session** (AC: 1-8)
  - [x] 9.1 Create helper to load inspection session data
    - Load defects from `analysis_results.json` (from Story 1-8)
    - Load captured images metadata from `images/*.json`
    - Load plan info from PlanManager
    - Calculate coverage from PlanCorrelator
  - [x] 9.2 Wire up in main.cpp for `--generate-report` command

---

## Previous Story Intelligence

### From Story 1-8 (VLM Defect Detection)

**Key learnings:**
- Defects stored in `analysis_results.json` with image references
- DefectTypes.h provides `Defect` struct and JSON serialization via `from_json()`
- ImageAnnotator shows pattern for color-coded annotations by severity
- VlmClient uses libcurl - DO NOT mix with report generation

**Critical: analysis_results.json Format (from Story 1-8 line 780-804):**
```json
{
  "session_id": "insp_001",
  "images_processed": 30,
  "total_defects": 5,
  "defects_by_image": [
    {
      "image": "img_00000003.jpg",
      "defects": [
        {
          "id": "def_001",
          "type": "QUALITY_ISSUE",
          "description": "Scratch on tile surface",
          "confidence": 0.87,
          "severity": "high",
          "trade": "finishes",
          "image_location": {"x": 320, "y": 240},
          "bounding_box": {"x": 280, "y": 200, "width": 80, "height": 80},
          "plan_location": {"x": 2.5, "y": 1.2}
        }
      ]
    }
  ]
}
```

**Files to reference:**
- `src/detection/DefectTypes.h` - Defect struct definition with `from_json()`
- `src/detection/ImageAnnotator.cpp` - Severity color mapping pattern (red/orange/green)

**Code patterns established:**
- `#pragma once` for headers
- PascalCase classes, camelCase methods
- nlohmann/json serialization with to_json/from_json
- OpenCV for image manipulation

### From Story 1-7 (Visual Capture)

**Key learnings:**
- ImageCapture saves images to `data/inspections/{session_id}/images/`
- Each image has JSON metadata sidecar with pose information
- `ImageCapture::loadMetadata()` static method for loading metadata
- Session directory structure established

**Files to reference:**
- `src/capture/ImageCapture.h` - ImageMetadata struct

### From Story 1-6 (State Machine + CLI)

**Key learnings:**
- CLI argument parsing pattern in main.cpp
- PlanManager provides plan image and info
- PlanInfo struct has all plan metadata

**Files to reference:**
- `src/plan/PlanManager.h` - PlanInfo struct

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Use any Python or Python bindings - this is pure C++17
- Use third-party PDF libraries other than libharu (already in project)
- Block main thread during report generation - consider async if >5 seconds
- Create new types that duplicate existing Point2D, Pose2D from Types.h
- Hardcode report dimensions - make configurable
- Call VlmClient or detection module functions (they require curl init)
- Share HPDF_Doc handles between threads (libharu is NOT thread-safe)

**MUST USE:**
- C++17 standard (`-std=c++17`)
- libharu (HPDF) for PDF generation (already linked in CMakeLists.txt)
- OpenCV for image manipulation (plan overlay, thumbnails)
- nlohmann/json for loading inspection data
- Existing types: `Point2D`, `Pose2D` from `src/util/Types.h`
- Existing `Defect` struct from `src/detection/DefectTypes.h`
- Existing `from_json()` function from `src/detection/DefectTypes.cpp` for parsing defects
- Existing `ImageMetadata` struct from `src/capture/ImageCapture.h`
- Existing `PlanInfo` from `src/plan/PlanManager.h`

### libharu (HPDF) API Reference

**Basic Document Creation with Robust Error Handling:**
```cpp
#include <hpdf.h>

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

// Error handler that stores state for caller to check
void hpdf_error_handler(HPDF_STATUS error_no, HPDF_STATUS detail_no, void* user_data) {
    auto* state = static_cast<HpdfErrorState*>(user_data);
    if (state) {
        state->has_error = true;
        state->error_no = error_no;
        state->detail_no = detail_no;
    }
    std::cerr << "HPDF Error: 0x" << std::hex << error_no << ", detail: 0x" << detail_no << std::endl;
}

// Create document with error state tracking
HpdfErrorState error_state;
HPDF_Doc pdf = HPDF_New(hpdf_error_handler, &error_state);
if (!pdf || error_state.has_error) {
    std::cerr << "Failed to create PDF document" << std::endl;
    return false;
}

// Enable compression
HPDF_SetCompressionMode(pdf, HPDF_COMP_ALL);

// Add page (A4 size)
HPDF_Page page = HPDF_AddPage(pdf);
HPDF_Page_SetSize(page, HPDF_PAGE_SIZE_A4, HPDF_PAGE_PORTRAIT);

// Get page dimensions
float page_width = HPDF_Page_GetWidth(page);    // 595.276 for A4
float page_height = HPDF_Page_GetHeight(page);  // 841.890 for A4

// Draw text
HPDF_Font font = HPDF_GetFont(pdf, "Helvetica", nullptr);
HPDF_Page_SetFontAndSize(page, font, 12);
HPDF_Page_BeginText(page);
HPDF_Page_TextOut(page, 50, page_height - 50, "Hello World");
HPDF_Page_EndText(page);

// Draw line
HPDF_Page_SetLineWidth(page, 1.0f);
HPDF_Page_MoveTo(page, 50, 700);
HPDF_Page_LineTo(page, 545, 700);
HPDF_Page_Stroke(page);

// Save document
HPDF_SaveToFile(pdf, "output.pdf");
HPDF_Free(pdf);
```

**Adding Images from cv::Mat:**
```cpp
// Convert cv::Mat to PNG buffer
std::vector<uchar> png_buffer;
cv::imencode(".png", image, png_buffer);

// Load PNG from memory
HPDF_Image pdf_image = HPDF_LoadPngImageFromMem(pdf, png_buffer.data(), png_buffer.size());

// Draw image on page (x, y, width, height)
float img_width = HPDF_Image_GetWidth(pdf_image);
float img_height = HPDF_Image_GetHeight(pdf_image);
float scale = std::min(500.0f / img_width, 400.0f / img_height);  // Fit in 500x400 box
HPDF_Page_DrawImage(page, pdf_image, 50, 300, img_width * scale, img_height * scale);
```

**Creating Tables:**
```cpp
// Table helper - draw cell borders and text
void drawTableCell(HPDF_Page page, float x, float y, float width, float height,
                   const std::string& text, HPDF_Font font) {
    // Draw cell border
    HPDF_Page_Rectangle(page, x, y, width, height);
    HPDF_Page_Stroke(page);

    // Draw text centered in cell
    HPDF_Page_BeginText(page);
    HPDF_Page_SetFontAndSize(page, font, 10);
    float text_width = HPDF_Page_TextWidth(page, text.c_str());
    float text_x = x + (width - text_width) / 2;
    float text_y = y + (height - 10) / 2;  // Approx vertical center
    HPDF_Page_TextOut(page, text_x, text_y, text.c_str());
    HPDF_Page_EndText(page);
}
```

### ReportGenerator Design

```cpp
// src/report/ReportGenerator.h
#pragma once

#include <string>
#include <vector>
#include <hpdf.h>
#include <opencv2/opencv.hpp>
#include "report/ReportTypes.h"
#include "detection/DefectTypes.h"

class ReportGenerator {
public:
    ReportGenerator();
    ~ReportGenerator();

    // Main generation function
    bool generate(const InspectionReport& report, const std::string& output_dir);

    // Configuration
    void setPageSize(float width, float height);  // Default A4
    void setMargins(float top, float bottom, float left, float right);
    void setLogoPath(const std::string& path) { logo_path_ = path; }
    void setIncludePhotos(bool include) { include_photos_ = include; }
    void setMaxPhotosPerPage(int max) { max_photos_per_page_ = max; }

    // CSV export
    static bool exportPunchListCsv(const InspectionReport& report, const std::string& path);

    // Helper to generate output filename
    static std::string generateFilename(const std::string& inspection_id,
                                        const std::string& plan_name,
                                        const std::string& date);

private:
    // PDF generation helpers
    bool initDocument();
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
    void drawTableHeader(const std::vector<std::string>& headers,
                         const std::vector<float>& col_widths,
                         float start_y);
    void drawTableRow(const std::vector<std::string>& cells,
                      const std::vector<float>& col_widths,
                      float y, bool alternating);

    HPDF_Doc pdf_ = nullptr;
    HPDF_Page current_page_ = nullptr;
    HPDF_Font font_regular_ = nullptr;
    HPDF_Font font_bold_ = nullptr;

    float page_width_ = 595.276f;   // A4 width in points
    float page_height_ = 841.890f;  // A4 height in points
    float margin_top_ = 50.0f;
    float margin_bottom_ = 50.0f;
    float margin_left_ = 50.0f;
    float margin_right_ = 50.0f;

    std::string logo_path_;
    bool include_photos_ = true;
    int max_photos_per_page_ = 4;
};
```

### ReportTypes Design

```cpp
// src/report/ReportTypes.h
#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "detection/DefectTypes.h"
#include "capture/ImageCapture.h"
#include "plan/PlanManager.h"

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

struct InspectionReport {
    // Metadata
    std::string inspection_id;
    std::string date;               // Format: YYYY-MM-DD
    std::string time;               // Format: HH:MM:SS
    std::string plan_name;
    std::string trade_type;

    // Coverage
    float coverage_percent = 0.0f;
    int images_captured = 0;
    int waypoints_visited = 0;
    int total_waypoints = 0;

    // Plan data
    cv::Mat plan_image;
    PlanInfo plan_info;

    // Defects (sorted by severity)
    std::vector<Defect> defects;
    InspectionSummary summary;

    // Photo references (paths relative to session dir)
    std::vector<ImageMetadata> captured_images;

    // Robot info
    std::string robot_id;
    float battery_at_start = 0.0f;
    float battery_at_end = 0.0f;
    float duration_minutes = 0.0f;

    // Helper to sort defects by severity
    void sortDefectsBySeverity();

    // Helper to compute summary from defects
    void computeSummary();
};

// Load inspection report data from session directory
bool loadInspectionReport(const std::string& session_dir,
                          InspectionReport& report,
                          PlanManager* plan_manager = nullptr);
```

### PlanOverlay Design

```cpp
// src/report/PlanOverlay.h
#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "util/Types.h"
#include "detection/DefectTypes.h"

class PlanOverlay {
public:
    PlanOverlay() = default;

    // Add defect markers
    void addDefectMarker(const Point2D& plan_loc, int number, const std::string& severity);
    void addDefectMarkers(const std::vector<Defect>& defects);

    // Clear all markers
    void clearMarkers();

    // Generate overlay image
    cv::Mat generateOverlay(const cv::Mat& plan_image, float scale = 1.0f) const;

    // Configuration
    void setMarkerRadius(int radius) { marker_radius_ = radius; }
    void setFontScale(float scale) { font_scale_ = scale; }
    void setHighColor(cv::Scalar color) { color_high_ = color; }
    void setMediumColor(cv::Scalar color) { color_medium_ = color; }
    void setLowColor(cv::Scalar color) { color_low_ = color; }

    // Add legend to image
    void addLegend(cv::Mat& image) const;

private:
    struct Marker {
        Point2D location;
        int number;
        std::string severity;
    };

    std::vector<Marker> markers_;

    int marker_radius_ = 15;
    float font_scale_ = 0.5f;
    cv::Scalar color_high_ = cv::Scalar(0, 0, 255);      // Red
    cv::Scalar color_medium_ = cv::Scalar(0, 165, 255);  // Orange
    cv::Scalar color_low_ = cv::Scalar(0, 255, 255);     // Yellow
    cv::Scalar color_text_ = cv::Scalar(255, 255, 255);  // White text
};
```

### CSV Export Format

```cpp
// CSV columns: ID, Type, Severity, Location_X, Location_Y, Description, Image, Trade
// Example output:
// "ID","Type","Severity","Location_X","Location_Y","Description","Image","Trade"
// "def_001","QUALITY_ISSUE","high","2.5","1.2","Scratch on tile surface","img_00000003.jpg","finishes"
// "def_002","LOCATION_ERROR","medium","4.1","3.8","Outlet misaligned by 5cm","img_00000007.jpg","mep"

bool ReportGenerator::exportPunchListCsv(const InspectionReport& report, const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) return false;

    // Header
    file << "\"ID\",\"Type\",\"Severity\",\"Location_X\",\"Location_Y\","
         << "\"Description\",\"Image\",\"Trade\"\n";

    // Rows
    for (const auto& d : report.defects) {
        file << "\"" << escapeCSV(d.id) << "\","
             << "\"" << defectTypeToString(d.type) << "\","
             << "\"" << escapeCSV(d.severity) << "\","
             << "\"" << d.plan_loc.x << "\","
             << "\"" << d.plan_loc.y << "\","
             << "\"" << escapeCSV(d.description) << "\","
             << "\"" << d.id << ".jpg\","  // Associated image
             << "\"" << escapeCSV(d.trade) << "\"\n";
    }

    return true;
}

std::string escapeCSV(const std::string& s) {
    std::string result;
    for (char c : s) {
        if (c == '"') result += "\"\"";  // Escape quotes by doubling
        else result += c;
    }
    return result;
}
```

### Directory Structure

```
src/report/              # NEW DIRECTORY
├── ReportTypes.h        # Report data structures
├── PlanOverlay.h        # Plan overlay interface
├── PlanOverlay.cpp      # Marker drawing implementation
├── ReportGenerator.h    # Main report generator interface
└── ReportGenerator.cpp  # libharu PDF implementation

test/
└── test_report.cpp      # NEW: Unit tests for report generation

data/reports/            # Output directory (created at runtime)
└── insp_001/
    ├── inspection_2025-12-04_floor.pdf
    └── punch_list.csv
```

### CMake Additions

```cmake
# ============================================
# Report Generation (Story 1-9)
# ============================================

# Report library (ReportGenerator, PlanOverlay)
add_library(report
    src/report/ReportGenerator.cpp
    src/report/PlanOverlay.cpp
)
target_include_directories(report PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(report
    detection    # For Defect types
    capture      # For ImageMetadata
    plan         # For PlanInfo
    ${OpenCV_LIBS}
    ${HPDF_LIBRARY}
    nlohmann_json::nlohmann_json
)

# Update g1_inspector to include report
target_link_libraries(g1_inspector
    # ... existing libraries ...
    report
)

# Report unit tests
if(GTest_FOUND)
    add_executable(test_report test/test_report.cpp)
    target_link_libraries(test_report
        report
        GTest::gtest_main
    )
    add_test(NAME test_report COMMAND test_report
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # End-to-end test for Story 1-9 (Report Generation)
    add_test(NAME test_e2e_story_1_9
        COMMAND ${CMAKE_SOURCE_DIR}/test/test_e2e_story_1_9.sh
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()
```

### Test Data Generation Script

Create `scripts/create_test_report_data.sh` to generate mock inspection data for testing:

```bash
#!/bin/bash
# Create mock inspection session data for testing report generation
# Usage: ./scripts/create_test_report_data.sh [session_id]

SESSION_ID="${1:-test_insp}"
BASE_DIR="data/inspections/${SESSION_ID}"

echo "Creating mock inspection data in ${BASE_DIR}..."

# Create directories
mkdir -p "${BASE_DIR}/images"
mkdir -p "${BASE_DIR}/annotated"

# Create mock analysis_results.json (matches Story 1-8 output format)
cat > "${BASE_DIR}/analysis_results.json" << 'EOF'
{
  "session_id": "test_insp",
  "images_processed": 5,
  "total_defects": 3,
  "defects_by_image": [
    {
      "image": "img_00000001.jpg",
      "defects": [
        {
          "id": "def_001",
          "type": "QUALITY_ISSUE",
          "description": "Scratch on tile surface near entrance",
          "confidence": 0.87,
          "severity": "high",
          "trade": "finishes",
          "image_location": {"x": 320, "y": 240},
          "bounding_box": {"x": 280, "y": 200, "width": 80, "height": 80},
          "plan_location": {"x": 2.5, "y": 1.2}
        }
      ]
    },
    {
      "image": "img_00000003.jpg",
      "defects": [
        {
          "id": "def_002",
          "type": "LOCATION_ERROR",
          "description": "Outlet misaligned by 5cm from plan",
          "confidence": 0.92,
          "severity": "medium",
          "trade": "mep",
          "image_location": {"x": 150, "y": 300},
          "bounding_box": {"x": 120, "y": 270, "width": 60, "height": 60},
          "plan_location": {"x": 4.1, "y": 3.8}
        },
        {
          "id": "def_003",
          "type": "SAFETY_HAZARD",
          "description": "Exposed electrical wiring",
          "confidence": 0.95,
          "severity": "high",
          "trade": "mep",
          "image_location": {"x": 400, "y": 180},
          "bounding_box": {"x": 370, "y": 150, "width": 60, "height": 60},
          "plan_location": {"x": 4.5, "y": 3.5}
        }
      ]
    }
  ]
}
EOF

# Create a simple test plan image (gray rectangle)
# Uses ImageMagick if available, otherwise creates empty placeholder
if command -v convert &> /dev/null; then
    convert -size 800x600 xc:white \
        -fill lightgray -draw "rectangle 50,50 750,550" \
        -fill black -draw "rectangle 100,100 200,200" \
        -fill black -draw "rectangle 600,400 700,500" \
        "${BASE_DIR}/plan.png"
    echo "Created plan.png with ImageMagick"
else
    # Create minimal valid PNG (1x1 white pixel) as placeholder
    echo -n -e '\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde\x00\x00\x00\x0cIDATx\x9cc\xf8\xff\xff?\x00\x05\xfe\x02\xfe\xa7V\xcf\x00\x00\x00\x00IEND\xaeB`\x82' > "${BASE_DIR}/plan.png"
    echo "Created placeholder plan.png (ImageMagick not available)"
fi

# Create mock image metadata files
for i in 1 2 3 4 5; do
    PADDED=$(printf "%08d" $i)
    # Create placeholder image (or use convert if available)
    if command -v convert &> /dev/null; then
        convert -size 640x480 xc:gray "${BASE_DIR}/images/img_${PADDED}.jpg"
    else
        touch "${BASE_DIR}/images/img_${PADDED}.jpg"
    fi

    # Create metadata JSON
    cat > "${BASE_DIR}/images/img_${PADDED}.json" << EOF
{
  "image_path": "images/img_${PADDED}.jpg",
  "timestamp_ms": $((1701705600000 + i * 1000)),
  "robot_pose": {"x": $((i * 2)).0, "y": 1.5, "theta": 0.0},
  "plan_coords": {"x": $((i * 2)).0, "y": 1.5},
  "camera_yaw": 0.0,
  "sequence_number": $i
}
EOF
done

echo "Created ${BASE_DIR}:"
echo "  - analysis_results.json (3 defects)"
echo "  - plan.png"
echo "  - images/img_*.jpg + .json (5 images)"
echo ""
echo "Run report generation with:"
echo "  ./build/g1_inspector --generate-report --inspection ${SESSION_ID}"
```

### Integration with Inspection Session

```cpp
// Load all data needed for report from inspection session
// IMPORTANT: Uses from_json() from DefectTypes.cpp - include "detection/DefectTypes.h"
bool loadInspectionReport(const std::string& session_dir,
                          InspectionReport& report,
                          PlanManager* plan_manager) {
    namespace fs = std::filesystem;

    // Extract inspection ID from directory name
    report.inspection_id = fs::path(session_dir).filename().string();

    // Load analysis results (defects from Story 1-8)
    std::string analysis_path = session_dir + "/analysis_results.json";
    if (fs::exists(analysis_path)) {
        std::ifstream file(analysis_path);
        nlohmann::json j;
        file >> j;

        // Parse defects using from_json() from DefectTypes.cpp
        // JSON format matches Story 1-8 output (see "Previous Story Intelligence" section)
        if (j.contains("defects_by_image")) {
            for (const auto& img_entry : j["defects_by_image"]) {
                if (!img_entry.contains("defects")) continue;

                for (const auto& d_json : img_entry["defects"]) {
                    Defect d;
                    // Use the existing from_json() function from DefectTypes.cpp
                    // This properly handles all fields including plan_loc
                    from_json(d_json, d);

                    // Handle plan_location field (Story 1-8 uses "plan_location" key)
                    if (d_json.contains("plan_location")) {
                        d.plan_loc.x = d_json["plan_location"].value("x", 0.0f);
                        d.plan_loc.y = d_json["plan_location"].value("y", 0.0f);
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
```

### CLI Integration for --generate-report

```cpp
// In main.cpp, add to argument parsing and command handling

// Argument parsing
bool generate_report = false;
std::string report_inspection_id;

for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--generate-report") {
        generate_report = true;
    } else if (arg == "--inspection" && i + 1 < argc) {
        report_inspection_id = argv[++i];
    }
}

// Report generation command
if (generate_report) {
    if (report_inspection_id.empty()) {
        std::cerr << "Error: --generate-report requires --inspection <id>" << std::endl;
        return 1;
    }

    std::string session_dir = "data/inspections/" + report_inspection_id;
    if (!std::filesystem::exists(session_dir)) {
        std::cerr << "Error: Inspection session not found: " << session_dir << std::endl;
        return 1;
    }

    // Load inspection data
    InspectionReport report;
    PlanManager plan_mgr;
    // Try to load plan if available
    std::string plan_path = session_dir + "/plan.png";
    if (std::filesystem::exists(plan_path)) {
        plan_mgr.loadPlan(plan_path);
    }

    if (!loadInspectionReport(session_dir, report, &plan_mgr)) {
        std::cerr << "Error: Failed to load inspection data" << std::endl;
        return 1;
    }

    // Generate report
    ReportGenerator generator;
    std::string output_dir = "data/reports/" + report_inspection_id;
    std::filesystem::create_directories(output_dir);

    if (generator.generate(report, output_dir)) {
        std::string pdf_name = ReportGenerator::generateFilename(
            report.inspection_id, report.plan_name, report.date);
        std::cout << "Report generated: " << output_dir << "/" << pdf_name << std::endl;

        // Also export CSV
        std::string csv_path = output_dir + "/punch_list.csv";
        if (ReportGenerator::exportPunchListCsv(report, csv_path)) {
            std::cout << "Punch list: " << csv_path << std::endl;
        }
    } else {
        std::cerr << "Error: Report generation failed" << std::endl;
        return 1;
    }

    return 0;
}
```

---

## Verification Commands

```bash
# Build (inside Docker container)
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests
./test_report               # All tests pass

# Generate report from an inspection session
./g1_inspector --generate-report --inspection insp_001

# Output:
# Report generated: data/reports/insp_001/inspection_2025-12-04_floor.pdf
# Punch list: data/reports/insp_001/punch_list.csv

# Verify PDF exists and has content
ls -la data/reports/insp_001/
# inspection_2025-12-04_floor.pdf   punch_list.csv

# Check CSV format
head -5 data/reports/insp_001/punch_list.csv
# "ID","Type","Severity","Location_X","Location_Y","Description","Image","Trade"
# "def_001","QUALITY_ISSUE","high","2.5","1.2","Scratch on tile",...

# Open PDF to verify all sections present
# - Title page with inspection ID, date, coverage
# - Executive summary with defect counts
# - Plan overlay with numbered markers
# - Photo gallery with thumbnails
# - Punch list table
```

### Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| Unit tests | `./test_report` | Exit 0, all pass |
| PDF creation | `--generate-report --inspection insp_001` | PDF file created |
| CSV export | Check punch_list.csv | Valid CSV with header and data |
| Filename pattern | ls data/reports/ | `inspection_<date>_<plan>.pdf` |
| Plan overlay | Open PDF | Numbered markers on plan |
| Photo gallery | Open PDF | Thumbnails with annotations |
| Punch list table | Open PDF | Defects in table format |
| Severity sorting | Check PDF/CSV | High > Medium > Low order |
| No crash on empty | Empty session dir | Graceful error message |

---

## Dependencies on Previous Stories

**Story 1-1 (Project Setup):**
- CMakeLists.txt base configuration
- libharu (HPDF) dependency already configured and linked

**Story 1-6 (State Machine + CLI):**
- PlanManager for plan image and info
- CLI argument parsing pattern

**Story 1-7 (Visual Capture):**
- ImageCapture for captured images metadata
- Session directory structure

**Story 1-8 (VLM Defect Detection):**
- Defect struct and JSON serialization
- analysis_results.json format
- Annotated images in session directory

---

## Test Data Requirements

For unit testing, create mock inspection data:
- Mock `InspectionReport` with sample defects
- Test plan image (small PNG)
- Sample defects covering all types and severities

Integration testing requires:
- Run a complete inspection to create session data
- Or create test session directory with sample data

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **ReportGenerator class** | PDF created with all sections |
| **PlanOverlay class** | Numbered markers on plan image |
| **CSV export** | punch_list.csv with correct format |
| **CLI integration** | `--generate-report` command works |
| **Unit tests** | test_report passes |

### Demo Script (Run This When Done)

```bash
# Inside Docker container
cd /workspace
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests
./test_report
# All tests pass

# Create test data (if no real inspection available)
mkdir -p ../data/inspections/test_insp/images
mkdir -p ../data/inspections/test_insp/annotated
# Copy test images and create analysis_results.json

# Generate report
./g1_inspector --generate-report --inspection test_insp

# Verify outputs
ls ../data/reports/test_insp/
# inspection_2025-12-05_plan.pdf  punch_list.csv

# Verify CSV
cat ../data/reports/test_insp/punch_list.csv

# Open PDF and verify sections:
# 1. Title page with inspection ID, date
# 2. Summary with defect counts by severity
# 3. Plan overlay with numbered markers
# 4. Photo gallery (if photos available)
# 5. Punch list table
```

**SUCCESS CRITERIA:** Story 1.9 is DONE when:
1. `./test_report` exits with code 0 (all tests pass)
2. PDF generated with all required sections
3. Plan overlay shows numbered defect markers
4. Punch list table in PDF with defects sorted by severity
5. CSV export works with correct format
6. Filename follows pattern: `inspection_<date>_<plan>.pdf`
7. `--generate-report` CLI command works
8. Graceful handling of missing data (empty inspections)
9. Full project builds successfully

---

## References

- [Source: docs/architecture.md#4.8-Report] - ReportGenerator design
- [Source: docs/epics.md#story-9] - Original story requirements
- [Source: docs/sprint-artifacts/1-8-vlm-defect-detection.md] - Defect types and patterns
- [Source: docs/sprint-artifacts/1-7-visual-capture.md] - ImageCapture patterns
- [External: libharu documentation](http://libharu.org/wiki/Documentation) - PDF API reference

---

## Dev Agent Record

### Context Reference

This story file serves as the complete implementation context.

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

None

### Completion Notes List

- Story created by create-story workflow with comprehensive PDF generation guide
- 2025-12-05: Validation improvements applied (validate-create-story):
  - CRITICAL-1 FIXED: Added curl initialization warning - story does NOT require curl, must not mix with detection module
  - CRITICAL-2 FIXED: Fixed loadInspectionReport to properly use from_json() and handle plan_location field
  - ENHANCEMENT-1 FIXED: Added thread safety warning for libharu HPDF_Doc handles
  - ENHANCEMENT-2 FIXED: Added empty defects handling in loadInspectionReport()
  - ENHANCEMENT-3 FIXED: Added robust HpdfErrorState error handler pattern
  - ENHANCEMENT-4 FIXED: Added explicit analysis_results.json format reference from Story 1-8
  - ENHANCEMENT-5 FIXED: Added test_e2e_story_1_9 to CMake configuration
  - OPT-1 FIXED: Added Key Function Signatures in Quick Reference
  - OPT-2 FIXED: Consolidated Previous Story Intelligence section, removed duplicate
  - OPT-3 FIXED: Added scripts/create_test_report_data.sh for mock test data generation
- 2025-12-06: Implementation completed:
  - All 9 tasks implemented and verified
  - 23 unit tests passing (test_report)
  - 8 E2E tests passing (test_e2e_story_1_9.sh)
  - PDF generation with title page, summary, plan overlay, photo gallery, punch list table
  - CSV export with proper escaping and formatting
  - CLI integration with --generate-report --inspection <id>
  - Graceful handling of missing plan images and empty defect lists

### Change Log

- 2025-12-06: Implemented Story 1-9 Report Generation
  - Created ReportTypes.h with InspectionReport, InspectionSummary, ReportConfig structs
  - Created PlanOverlay class for defect marker visualization on plan images
  - Created ReportGenerator class with libharu PDF generation
  - Added CSV punch list export with proper escaping
  - Integrated --generate-report CLI command in main.cpp
  - Added 23 unit tests covering all report functionality
  - Added E2E test script for full workflow validation
  - Added test data generation script

- 2025-12-06: Code Review Fixes Round 1 (6 issues resolved)
  - HIGH-1 FIXED: PlanOverlay.generateOverlay() now accepts meters_per_pixel parameter
    - Uses PlanInfo::resolution for accurate coordinate conversion
    - Default 0.02 m/pixel maintained for backwards compatibility
  - HIGH-2 FIXED: Added source_image field to Defect struct
    - CSV export now uses actual source image filename instead of defect ID
    - loadInspectionReport() captures source image from JSON "image" key
  - MEDIUM-1 FIXED: ReportGenerator.generate() now accepts explicit session_dir
    - Eliminates fragile path inference for photo loading
  - MEDIUM-3 FIXED: E2E test 7 logic corrected
    - Both branches were passing; now properly tests error handling
  - MEDIUM-4 FIXED: Added unit tests for marker position accuracy
    - Tests verify markers appear at correct pixel coordinates for different resolutions
  - Added 4 new unit tests: PlanOverlay_MarkerPositionAccuracy,
    PlanOverlay_DifferentResolutions, ReportGenerator_ExportPunchListCsv_SourceImage

- 2025-12-06: Code Review Fixes Round 2 (6 issues resolved)
  - HIGH-1 FIXED: addPlanOverlayPage() now has proper error recovery with fallback and placeholder
  - HIGH-2 FIXED: generate() checks error_state after each section and cleans up on failure
  - HIGH-3 FIXED: Removed dead ReportConfig struct (config handled by ReportGenerator methods)
  - MEDIUM-1 FIXED: Removed duplicate plan_location parsing in loadInspectionReport()
  - MEDIUM-2 FIXED: addNewPage() now has 500-page limit to prevent runaway creation
  - MEDIUM-3 FIXED: CSV export now uses "N/A" for missing source_image (not fake filename)
  - FIXED: PlanInfo.resolution -> PlanInfo.scale (correct field name)
  - FIXED: Marker position tests now check offset from center (center has white text)
  - Added 1 new unit test: ReportGenerator_ExportPunchListCsv_MissingSourceImage
  - All 27 unit tests passing, all 8 E2E tests passing

### File List

**New files created:**
- `src/report/ReportTypes.h` - Report data structures (InspectionReport, InspectionSummary, ReportConfig, loadInspectionReport)
- `src/report/PlanOverlay.h` - Plan overlay interface
- `src/report/PlanOverlay.cpp` - Marker drawing implementation with severity colors and legend
- `src/report/ReportGenerator.h` - Main report generator interface with HpdfErrorState
- `src/report/ReportGenerator.cpp` - libharu PDF implementation (title page, summary, overlay, gallery, punch list)
- `test/test_report.cpp` - 28 unit tests for report generation (including 5 from code reviews)
- `test/test_e2e_story_1_9.sh` - End-to-end test script (8 tests)
- `scripts/create_test_report_data.sh` - Mock test data generation script

**Files modified:**
- `CMakeLists.txt` - Added `report` library, `test_report` executable, `test_e2e_story_1_9` test
- `src/main.cpp` - Added `--generate-report --inspection <id>` CLI option and runGenerateReport function
- `src/detection/DefectTypes.h` - Added `source_image` field to Defect struct (code review fix)
- `src/detection/DefectTypes.cpp` - Updated JSON serialization for source_image field
