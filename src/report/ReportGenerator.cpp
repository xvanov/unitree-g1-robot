#include "report/ReportGenerator.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>

// libharu error handler that stores state for caller to check
static void hpdf_error_handler(HPDF_STATUS error_no, HPDF_STATUS detail_no, void* user_data) {
    auto* state = static_cast<HpdfErrorState*>(user_data);
    if (state) {
        state->has_error = true;
        state->error_no = error_no;
        state->detail_no = detail_no;
    }
    std::cerr << "[REPORT] HPDF Error: 0x" << std::hex << error_no
              << ", detail: 0x" << detail_no << std::dec << std::endl;
}

ReportGenerator::ReportGenerator() {
    error_state_.clear();
}

ReportGenerator::~ReportGenerator() {
    cleanup();
}

void ReportGenerator::cleanup() {
    if (pdf_) {
        HPDF_Free(pdf_);
        pdf_ = nullptr;
    }
    current_page_ = nullptr;
    font_regular_ = nullptr;
    font_bold_ = nullptr;
    pages_.clear();
    total_pages_ = 0;
}

bool ReportGenerator::initDocument() {
    cleanup();
    error_state_.clear();

    pdf_ = HPDF_New(hpdf_error_handler, &error_state_);
    if (!pdf_ || error_state_.has_error) {
        last_error_ = "Failed to create PDF document";
        return false;
    }

    // Enable compression for smaller files
    HPDF_SetCompressionMode(pdf_, HPDF_COMP_ALL);

    // Set up fonts
    font_regular_ = HPDF_GetFont(pdf_, "Helvetica", nullptr);
    font_bold_ = HPDF_GetFont(pdf_, "Helvetica-Bold", nullptr);

    if (!font_regular_ || !font_bold_) {
        last_error_ = "Failed to load fonts";
        return false;
    }

    return true;
}

HPDF_Page ReportGenerator::addNewPage() {
    // MEDIUM-2 FIX: Add bounds check to prevent runaway page creation
    constexpr int MAX_PAGES = 500;  // Reasonable limit for inspection reports
    if (total_pages_ >= MAX_PAGES) {
        last_error_ = "Maximum page limit reached (" + std::to_string(MAX_PAGES) + ")";
        std::cerr << "[REPORT] " << last_error_ << std::endl;
        return nullptr;
    }

    HPDF_Page page = HPDF_AddPage(pdf_);
    if (!page) {
        last_error_ = "Failed to add new page";
        return nullptr;
    }

    HPDF_Page_SetSize(page, HPDF_PAGE_SIZE_A4, HPDF_PAGE_PORTRAIT);
    pages_.push_back(page);
    total_pages_++;
    current_page_ = page;

    return page;
}

void ReportGenerator::setPageSize(float width, float height) {
    page_width_ = width;
    page_height_ = height;
}

void ReportGenerator::setMargins(float top, float bottom, float left, float right) {
    margin_top_ = top;
    margin_bottom_ = bottom;
    margin_left_ = left;
    margin_right_ = right;
}

void ReportGenerator::drawText(float x, float y, const std::string& text, HPDF_Font font, float size) {
    if (!current_page_) return;

    HPDF_Page_BeginText(current_page_);
    HPDF_Page_SetFontAndSize(current_page_, font, size);
    HPDF_Page_TextOut(current_page_, x, y, text.c_str());
    HPDF_Page_EndText(current_page_);
}

void ReportGenerator::drawTextCentered(float x, float y, float width, const std::string& text,
                                        HPDF_Font font, float size) {
    if (!current_page_) return;

    HPDF_Page_BeginText(current_page_);
    HPDF_Page_SetFontAndSize(current_page_, font, size);
    float text_width = HPDF_Page_TextWidth(current_page_, text.c_str());
    float text_x = x + (width - text_width) / 2;
    HPDF_Page_TextOut(current_page_, text_x, y, text.c_str());
    HPDF_Page_EndText(current_page_);
}

HPDF_Image ReportGenerator::loadImageToPdf(const cv::Mat& image) {
    if (image.empty() || !pdf_) return nullptr;

    // Convert to BGR if grayscale
    cv::Mat img_bgr;
    if (image.channels() == 1) {
        cv::cvtColor(image, img_bgr, cv::COLOR_GRAY2BGR);
    } else {
        img_bgr = image;
    }

    // Encode as PNG
    std::vector<uchar> png_buffer;
    if (!cv::imencode(".png", img_bgr, png_buffer)) {
        std::cerr << "[REPORT] Failed to encode image to PNG" << std::endl;
        return nullptr;
    }

    // Load PNG from memory
    HPDF_Image pdf_image = HPDF_LoadPngImageFromMem(pdf_, png_buffer.data(),
                                                     static_cast<HPDF_UINT>(png_buffer.size()));
    return pdf_image;
}

bool ReportGenerator::addImageToPage(const cv::Mat& image, float x, float y,
                                      float max_width, float max_height) {
    HPDF_Image pdf_image = loadImageToPdf(image);
    if (!pdf_image) return false;

    float img_width = static_cast<float>(HPDF_Image_GetWidth(pdf_image));
    float img_height = static_cast<float>(HPDF_Image_GetHeight(pdf_image));

    // Calculate scale to fit in max dimensions while preserving aspect ratio
    float scale = std::min(max_width / img_width, max_height / img_height);
    float draw_width = img_width * scale;
    float draw_height = img_height * scale;

    HPDF_Page_DrawImage(current_page_, pdf_image, x, y, draw_width, draw_height);
    return true;
}

float ReportGenerator::drawTableHeader(const std::vector<std::string>& headers,
                                         const std::vector<float>& col_widths,
                                         float start_y) {
    if (!current_page_ || headers.size() != col_widths.size()) return start_y;

    float row_height = 20.0f;
    float x = margin_left_;
    float y = start_y;

    // Draw header background
    HPDF_Page_SetRGBFill(current_page_, 0.2f, 0.2f, 0.2f);
    float total_width = 0;
    for (float w : col_widths) total_width += w;
    HPDF_Page_Rectangle(current_page_, x, y - row_height, total_width, row_height);
    HPDF_Page_Fill(current_page_);

    // Draw header text (white)
    HPDF_Page_SetRGBFill(current_page_, 1.0f, 1.0f, 1.0f);
    for (size_t i = 0; i < headers.size(); i++) {
        HPDF_Page_BeginText(current_page_);
        HPDF_Page_SetFontAndSize(current_page_, font_bold_, 9);
        HPDF_Page_TextOut(current_page_, x + 3, y - 14, headers[i].c_str());
        HPDF_Page_EndText(current_page_);
        x += col_widths[i];
    }

    // Reset fill color
    HPDF_Page_SetRGBFill(current_page_, 0.0f, 0.0f, 0.0f);

    return y - row_height;
}

float ReportGenerator::drawTableRow(const std::vector<std::string>& cells,
                                      const std::vector<float>& col_widths,
                                      float y, bool alternating) {
    if (!current_page_ || cells.size() != col_widths.size()) return y;

    float row_height = 18.0f;
    float x = margin_left_;

    // Draw alternating background
    if (alternating) {
        HPDF_Page_SetRGBFill(current_page_, 0.95f, 0.95f, 0.95f);
        float total_width = 0;
        for (float w : col_widths) total_width += w;
        HPDF_Page_Rectangle(current_page_, x, y - row_height, total_width, row_height);
        HPDF_Page_Fill(current_page_);
    }

    // Draw cell text
    HPDF_Page_SetRGBFill(current_page_, 0.0f, 0.0f, 0.0f);
    for (size_t i = 0; i < cells.size(); i++) {
        HPDF_Page_BeginText(current_page_);
        HPDF_Page_SetFontAndSize(current_page_, font_regular_, 8);

        // Truncate text if too long for column
        std::string text = cells[i];
        float max_chars = col_widths[i] / 4.5f;  // Approximate chars that fit
        if (text.length() > static_cast<size_t>(max_chars)) {
            text = text.substr(0, static_cast<size_t>(max_chars) - 3) + "...";
        }

        HPDF_Page_TextOut(current_page_, x + 3, y - 13, text.c_str());
        HPDF_Page_EndText(current_page_);
        x += col_widths[i];
    }

    // Draw row border
    HPDF_Page_SetLineWidth(current_page_, 0.5f);
    float total_width = 0;
    for (float w : col_widths) total_width += w;
    HPDF_Page_Rectangle(current_page_, margin_left_, y - row_height, total_width, row_height);
    HPDF_Page_Stroke(current_page_);

    return y - row_height;
}

void ReportGenerator::addTitlePage(const InspectionReport& report) {
    addNewPage();

    float y = page_height_ - margin_top_;
    float content_width = page_width_ - margin_left_ - margin_right_;

    // Title
    drawTextCentered(margin_left_, y - 40, content_width, "INSPECTION REPORT", font_bold_, 24);
    y -= 80;

    // Horizontal line
    HPDF_Page_SetLineWidth(current_page_, 2.0f);
    HPDF_Page_MoveTo(current_page_, margin_left_, y);
    HPDF_Page_LineTo(current_page_, page_width_ - margin_right_, y);
    HPDF_Page_Stroke(current_page_);
    y -= 40;

    // Inspection details
    drawText(margin_left_, y, "Inspection ID:", font_bold_, 12);
    drawText(margin_left_ + 120, y, report.inspection_id, font_regular_, 12);
    y -= 25;

    drawText(margin_left_, y, "Date:", font_bold_, 12);
    drawText(margin_left_ + 120, y, report.date + " " + report.time, font_regular_, 12);
    y -= 25;

    drawText(margin_left_, y, "Plan:", font_bold_, 12);
    drawText(margin_left_ + 120, y, report.plan_name, font_regular_, 12);
    y -= 25;

    drawText(margin_left_, y, "Trade:", font_bold_, 12);
    drawText(margin_left_ + 120, y, report.trade_type.empty() ? "General" : report.trade_type, font_regular_, 12);
    y -= 25;

    drawText(margin_left_, y, "Coverage:", font_bold_, 12);
    std::ostringstream coverage_ss;
    coverage_ss << std::fixed << std::setprecision(1) << report.coverage_percent << "%";
    drawText(margin_left_ + 120, y, coverage_ss.str(), font_regular_, 12);
    y -= 25;

    drawText(margin_left_, y, "Images Captured:", font_bold_, 12);
    drawText(margin_left_ + 120, y, std::to_string(report.images_captured), font_regular_, 12);
    y -= 40;

    // Summary box
    HPDF_Page_SetRGBFill(current_page_, 0.95f, 0.95f, 0.95f);
    HPDF_Page_Rectangle(current_page_, margin_left_, y - 120, content_width, 120);
    HPDF_Page_Fill(current_page_);
    HPDF_Page_SetRGBFill(current_page_, 0.0f, 0.0f, 0.0f);
    HPDF_Page_Rectangle(current_page_, margin_left_, y - 120, content_width, 120);
    HPDF_Page_Stroke(current_page_);

    float box_y = y - 25;
    drawTextCentered(margin_left_, box_y, content_width, "DEFECT SUMMARY", font_bold_, 14);
    box_y -= 30;

    // Defect counts
    std::ostringstream total_ss;
    total_ss << "Total Defects: " << report.summary.total_defects;
    drawTextCentered(margin_left_, box_y, content_width, total_ss.str(), font_regular_, 12);
    box_y -= 25;

    std::ostringstream severity_ss;
    severity_ss << "High: " << report.summary.high_severity
                << "  |  Medium: " << report.summary.medium_severity
                << "  |  Low: " << report.summary.low_severity;
    drawTextCentered(margin_left_, box_y, content_width, severity_ss.str(), font_regular_, 11);
}

void ReportGenerator::addSummarySection(const InspectionReport& report) {
    addNewPage();

    float y = page_height_ - margin_top_;
    float content_width = page_width_ - margin_left_ - margin_right_;

    // Section title
    drawText(margin_left_, y, "EXECUTIVE SUMMARY", font_bold_, 16);
    y -= 30;

    // Horizontal line
    HPDF_Page_SetLineWidth(current_page_, 1.0f);
    HPDF_Page_MoveTo(current_page_, margin_left_, y);
    HPDF_Page_LineTo(current_page_, page_width_ - margin_right_, y);
    HPDF_Page_Stroke(current_page_);
    y -= 30;

    // Summary by type
    drawText(margin_left_, y, "Defects by Type:", font_bold_, 12);
    y -= 20;

    std::vector<std::pair<std::string, int>> type_counts = {
        {"Location Errors", report.summary.location_errors},
        {"Quality Issues", report.summary.quality_issues},
        {"Safety Hazards", report.summary.safety_hazards},
        {"Missing Elements", report.summary.missing_elements}
    };

    for (const auto& tc : type_counts) {
        std::ostringstream ss;
        ss << "  - " << tc.first << ": " << tc.second;
        drawText(margin_left_ + 20, y, ss.str(), font_regular_, 11);
        y -= 18;
    }
    y -= 20;

    // Summary by severity with color indicators
    drawText(margin_left_, y, "Defects by Severity:", font_bold_, 12);
    y -= 25;

    // High severity (red)
    HPDF_Page_SetRGBFill(current_page_, 1.0f, 0.0f, 0.0f);
    HPDF_Page_Circle(current_page_, margin_left_ + 30, y + 5, 6);
    HPDF_Page_Fill(current_page_);
    HPDF_Page_SetRGBFill(current_page_, 0.0f, 0.0f, 0.0f);
    std::ostringstream high_ss;
    high_ss << "High Severity: " << report.summary.high_severity;
    drawText(margin_left_ + 50, y, high_ss.str(), font_regular_, 11);
    y -= 22;

    // Medium severity (orange)
    HPDF_Page_SetRGBFill(current_page_, 1.0f, 0.65f, 0.0f);
    HPDF_Page_Circle(current_page_, margin_left_ + 30, y + 5, 6);
    HPDF_Page_Fill(current_page_);
    HPDF_Page_SetRGBFill(current_page_, 0.0f, 0.0f, 0.0f);
    std::ostringstream med_ss;
    med_ss << "Medium Severity: " << report.summary.medium_severity;
    drawText(margin_left_ + 50, y, med_ss.str(), font_regular_, 11);
    y -= 22;

    // Low severity (yellow)
    HPDF_Page_SetRGBFill(current_page_, 1.0f, 1.0f, 0.0f);
    HPDF_Page_Circle(current_page_, margin_left_ + 30, y + 5, 6);
    HPDF_Page_Fill(current_page_);
    HPDF_Page_SetRGBFill(current_page_, 0.0f, 0.0f, 0.0f);
    std::ostringstream low_ss;
    low_ss << "Low Severity: " << report.summary.low_severity;
    drawText(margin_left_ + 50, y, low_ss.str(), font_regular_, 11);
    y -= 40;

    // Robot info if available
    if (!report.robot_id.empty() || report.battery_at_start > 0) {
        drawText(margin_left_, y, "Robot Information:", font_bold_, 12);
        y -= 20;

        if (!report.robot_id.empty()) {
            drawText(margin_left_ + 20, y, "Robot ID: " + report.robot_id, font_regular_, 11);
            y -= 18;
        }

        if (report.duration_minutes > 0) {
            std::ostringstream dur_ss;
            dur_ss << "Duration: " << std::fixed << std::setprecision(1) << report.duration_minutes << " minutes";
            drawText(margin_left_ + 20, y, dur_ss.str(), font_regular_, 11);
            y -= 18;
        }

        if (report.battery_at_start > 0) {
            std::ostringstream bat_ss;
            bat_ss << "Battery: " << std::fixed << std::setprecision(0)
                   << report.battery_at_start << "% -> " << report.battery_at_end << "%";
            drawText(margin_left_ + 20, y, bat_ss.str(), font_regular_, 11);
        }
    }
}

void ReportGenerator::addPlanOverlayPage(const InspectionReport& report) {
    if (report.plan_image.empty()) {
        std::cout << "[REPORT] No plan image available - skipping plan overlay page" << std::endl;
        return;
    }

    addNewPage();

    float y = page_height_ - margin_top_;
    float content_width = page_width_ - margin_left_ - margin_right_;
    float content_height = page_height_ - margin_top_ - margin_bottom_ - 60;  // Leave room for title

    // Section title
    drawText(margin_left_, y, "PLAN OVERLAY - DEFECT LOCATIONS", font_bold_, 16);
    y -= 30;

    // Create overlay with markers
    // Use plan scale for accurate coordinate conversion (default 0.02 = 2cm/pixel)
    float meters_per_pixel = report.plan_info.scale > 0 ? report.plan_info.scale : 0.02f;
    PlanOverlay overlay;
    overlay.addDefectMarkers(report.defects);
    cv::Mat overlay_image = overlay.generateOverlay(report.plan_image, 1.0f, meters_per_pixel);

    // HIGH-1 FIX: Proper error handling for image rendering
    bool image_added = false;
    if (!overlay_image.empty()) {
        // Add legend
        overlay.addLegend(overlay_image);

        // Draw the overlay image
        image_added = addImageToPage(overlay_image, margin_left_, margin_bottom_ + 20, content_width, content_height);
    }

    // Fallback: try raw plan if overlay failed
    if (!image_added && !report.plan_image.empty()) {
        image_added = addImageToPage(report.plan_image, margin_left_, margin_bottom_ + 20, content_width, content_height);
    }

    // If all image attempts failed, add placeholder text
    if (!image_added) {
        drawText(margin_left_, page_height_ / 2, "[Plan image could not be rendered]", font_regular_, 12);
        std::cerr << "[REPORT] Warning: Failed to render plan overlay image" << std::endl;
    }
}

void ReportGenerator::addPhotoGallery(const InspectionReport& report) {
    if (!include_photos_ || report.captured_images.empty()) {
        return;
    }

    addNewPage();

    float y = page_height_ - margin_top_;
    float content_width = page_width_ - margin_left_ - margin_right_;

    // Section title
    drawText(margin_left_, y, "PHOTO GALLERY", font_bold_, 16);
    y -= 30;

    // Calculate grid layout
    int photos_per_row = 2;
    float photo_width = (content_width - 20) / photos_per_row;
    float photo_height = photo_width * 0.75f;  // 4:3 aspect ratio
    int photos_per_page = max_photos_per_page_;

    int photo_count = 0;
    int row = 0;
    int col = 0;

    for (const auto& img_meta : report.captured_images) {
        if (photo_count >= photos_per_page * 3) break;  // Limit total photos

        // Load image from session directory
        std::string img_path = session_dir_ + "/" + img_meta.image_path;
        cv::Mat img = cv::imread(img_path);

        if (img.empty()) {
            // Try annotated directory
            std::filesystem::path p(img_meta.image_path);
            img_path = session_dir_ + "/annotated/" + p.filename().string();
            img = cv::imread(img_path);
        }

        if (img.empty()) continue;

        // Check if we need a new page
        if (photo_count > 0 && photo_count % photos_per_page == 0) {
            addNewPage();
            y = page_height_ - margin_top_;
            drawText(margin_left_, y, "PHOTO GALLERY (continued)", font_bold_, 16);
            y -= 30;
            row = 0;
            col = 0;
        }

        // Calculate position
        float x = margin_left_ + col * (photo_width + 10);
        float img_y = y - (row + 1) * (photo_height + 30);

        // Draw photo
        addImageToPage(img, x, img_y, photo_width, photo_height);

        // Draw caption
        std::filesystem::path p(img_meta.image_path);
        std::string caption = p.filename().string();
        HPDF_Page_BeginText(current_page_);
        HPDF_Page_SetFontAndSize(current_page_, font_regular_, 8);
        HPDF_Page_TextOut(current_page_, x, img_y - 10, caption.c_str());
        HPDF_Page_EndText(current_page_);

        col++;
        if (col >= photos_per_row) {
            col = 0;
            row++;
        }
        photo_count++;
    }
}

void ReportGenerator::addPunchListTable(const InspectionReport& report) {
    addNewPage();

    float y = page_height_ - margin_top_;
    float content_width = page_width_ - margin_left_ - margin_right_;

    // Section title
    drawText(margin_left_, y, "PUNCH LIST", font_bold_, 16);
    y -= 30;

    // Horizontal line
    HPDF_Page_SetLineWidth(current_page_, 1.0f);
    HPDF_Page_MoveTo(current_page_, margin_left_, y);
    HPDF_Page_LineTo(current_page_, page_width_ - margin_right_, y);
    HPDF_Page_Stroke(current_page_);
    y -= 20;

    if (report.defects.empty()) {
        drawText(margin_left_, y, "No defects found during inspection.", font_regular_, 12);
        return;
    }

    // Table column widths
    std::vector<float> col_widths = {30, 50, 45, 40, 40, 180, 60, 50};  // Total ~495
    std::vector<std::string> headers = {"#", "ID", "Severity", "Loc X", "Loc Y", "Description", "Type", "Trade"};

    y = drawTableHeader(headers, col_widths, y);

    // Draw rows
    int row_num = 1;
    for (const auto& d : report.defects) {
        // Check if we need a new page
        if (y < margin_bottom_ + 30) {
            addNewPage();
            y = page_height_ - margin_top_;
            drawText(margin_left_, y, "PUNCH LIST (continued)", font_bold_, 16);
            y -= 40;
            y = drawTableHeader(headers, col_widths, y);
        }

        std::ostringstream loc_x, loc_y;
        loc_x << std::fixed << std::setprecision(1) << d.plan_loc.x;
        loc_y << std::fixed << std::setprecision(1) << d.plan_loc.y;

        std::vector<std::string> cells = {
            std::to_string(row_num),
            d.id,
            d.severity,
            loc_x.str(),
            loc_y.str(),
            d.description,
            defectTypeToString(d.type),
            d.trade
        };

        y = drawTableRow(cells, col_widths, y, row_num % 2 == 0);
        row_num++;
    }
}

void ReportGenerator::addPageFooter(int page_num, int total_pages) {
    if (!current_page_) return;

    std::ostringstream ss;
    ss << "Page " << page_num << " of " << total_pages;

    float y = margin_bottom_ / 2;
    float x = page_width_ / 2;

    HPDF_Page_BeginText(current_page_);
    HPDF_Page_SetFontAndSize(current_page_, font_regular_, 9);
    float text_width = HPDF_Page_TextWidth(current_page_, ss.str().c_str());
    HPDF_Page_TextOut(current_page_, x - text_width / 2, y, ss.str().c_str());
    HPDF_Page_EndText(current_page_);
}

bool ReportGenerator::generate(const InspectionReport& report, const std::string& output_dir,
                                const std::string& session_dir) {
    // Store session directory for image loading
    // Use explicit session_dir if provided, otherwise try to infer from output_dir
    if (!session_dir.empty()) {
        session_dir_ = session_dir;
    } else if (output_dir.find("/reports/") != std::string::npos) {
        // Fallback: infer from output_dir if it follows standard pattern
        std::filesystem::path p(output_dir);
        std::string insp_id = p.filename().string();
        session_dir_ = p.parent_path().parent_path().string() + "/inspections/" + insp_id;
    } else {
        session_dir_ = output_dir;  // Last resort - may not have photos
    }

    if (!initDocument()) {
        return false;
    }

    // HIGH-2 FIX: Check for libharu errors after each section
    // Generate all sections with error checking
    addTitlePage(report);
    if (error_state_.has_error) {
        last_error_ = "Failed generating title page: " + error_state_.toString();
        cleanup();
        return false;
    }

    addSummarySection(report);
    if (error_state_.has_error) {
        last_error_ = "Failed generating summary: " + error_state_.toString();
        cleanup();
        return false;
    }

    addPlanOverlayPage(report);
    if (error_state_.has_error) {
        last_error_ = "Failed generating plan overlay: " + error_state_.toString();
        cleanup();
        return false;
    }

    addPhotoGallery(report);
    if (error_state_.has_error) {
        last_error_ = "Failed generating photo gallery: " + error_state_.toString();
        cleanup();
        return false;
    }

    addPunchListTable(report);
    if (error_state_.has_error) {
        last_error_ = "Failed generating punch list: " + error_state_.toString();
        cleanup();
        return false;
    }

    // Add page footers to all pages
    for (size_t i = 0; i < pages_.size(); i++) {
        current_page_ = pages_[i];
        addPageFooter(static_cast<int>(i + 1), static_cast<int>(pages_.size()));
    }

    // Generate filename and save
    std::string filename = generateFilename(report.inspection_id, report.plan_name, report.date);
    std::string output_path = output_dir + "/" + filename;

    // Create output directory if needed
    std::filesystem::create_directories(output_dir);

    // Save PDF
    HPDF_STATUS status = HPDF_SaveToFile(pdf_, output_path.c_str());
    if (status != HPDF_OK || error_state_.has_error) {
        last_error_ = "Failed to save PDF: " + error_state_.toString();
        cleanup();
        return false;
    }

    std::cout << "[REPORT] PDF generated: " << output_path << std::endl;
    cleanup();
    return true;
}

std::string ReportGenerator::generateFilename(const std::string& inspection_id,
                                               const std::string& plan_name,
                                               const std::string& date) {
    // Extract base plan name without path and extension
    std::filesystem::path p(plan_name);
    std::string base_name = p.stem().string();
    if (base_name.empty()) base_name = "inspection";

    // Clean up plan name for filename
    for (char& c : base_name) {
        if (!std::isalnum(c) && c != '_' && c != '-') {
            c = '_';
        }
    }

    // Format: inspection_<date>_<plan>.pdf (AC5)
    return "inspection_" + date + "_" + base_name + ".pdf";
}

std::string ReportGenerator::escapeCSV(const std::string& s) {
    std::string result;
    for (char c : s) {
        if (c == '"') {
            result += "\"\"";  // Escape quotes by doubling
        } else {
            result += c;
        }
    }
    return result;
}

bool ReportGenerator::exportPunchListCsv(const InspectionReport& report, const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "[REPORT] Failed to open CSV file: " << path << std::endl;
        return false;
    }

    // Write header
    file << "\"ID\",\"Type\",\"Severity\",\"Location_X\",\"Location_Y\","
         << "\"Description\",\"Image\",\"Trade\"\n";

    // Write rows
    for (const auto& d : report.defects) {
        // MEDIUM-3 FIX: Use source_image if available; fallback to "N/A" (not fake filename)
        // The source_image field should be populated by loadInspectionReport from analysis_results.json
        std::string image_ref = d.source_image.empty() ? "N/A" : d.source_image;
        file << "\"" << escapeCSV(d.id) << "\","
             << "\"" << defectTypeToString(d.type) << "\","
             << "\"" << escapeCSV(d.severity) << "\","
             << "\"" << std::fixed << std::setprecision(2) << d.plan_loc.x << "\","
             << "\"" << std::fixed << std::setprecision(2) << d.plan_loc.y << "\","
             << "\"" << escapeCSV(d.description) << "\","
             << "\"" << escapeCSV(image_ref) << "\","
             << "\"" << escapeCSV(d.trade) << "\"\n";
    }

    file.close();
    std::cout << "[REPORT] CSV exported: " << path << std::endl;
    return true;
}
