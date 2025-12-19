#pragma once

#include "greeter/ActionParser.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <deque>

namespace greeter {

/**
 * Entry in the action history for display.
 */
struct ActionHistoryEntry {
    std::string action_type;
    std::string intent;
    float confidence;
    std::string timestamp;
};

/**
 * ReasoningDisplay - Renders Barry's decision-making process for observation.
 *
 * Displays a composite view showing:
 * - Camera frame (left side)
 * - LLM reasoning panel (right side top)
 * - Action history (right side bottom)
 * - Status bar (bottom)
 *
 * This provides visibility into the LLM's decision-making for the demo.
 */
class ReasoningDisplay {
public:
    ReasoningDisplay();
    ~ReasoningDisplay();

    /**
     * Initialize the display window.
     * @param window_name Name for the OpenCV window
     * @return true if initialization succeeded
     */
    bool init(const std::string& window_name = "Barry - Reasoning Display");

    /**
     * Update display with current frame and LLM response.
     * @param frame Current camera frame
     * @param action Parsed action from LLM
     * @param history Recent action history
     */
    void update(const cv::Mat& frame, const ParsedAction& action,
                const std::deque<ActionHistoryEntry>& history);

    /**
     * Show just the frame (when no LLM response available).
     * @param frame Current camera frame
     */
    void showFrame(const cv::Mat& frame);

    /**
     * Set status text to display.
     * @param status Status message
     */
    void setStatus(const std::string& status);

    /**
     * Check if window is still open.
     */
    bool isOpen() const;

    /**
     * Process window events (call in main loop).
     * @param wait_ms Milliseconds to wait (default 1)
     * @return Key pressed, or -1 if none
     */
    int processEvents(int wait_ms = 1);

    /**
     * Set maximum history entries to display.
     */
    void setMaxHistoryDisplay(int max_entries) { max_history_display_ = max_entries; }

    /**
     * Close the display window.
     */
    void close();

private:
    std::string window_name_;
    bool initialized_ = false;
    std::string status_text_;
    int max_history_display_ = 5;

    // Display dimensions
    static constexpr int DISPLAY_WIDTH = 1280;
    static constexpr int DISPLAY_HEIGHT = 720;
    static constexpr int CAMERA_WIDTH = 640;
    static constexpr int CAMERA_HEIGHT = 480;
    static constexpr int PANEL_WIDTH = 640;
    static constexpr int STATUS_HEIGHT = 40;

    /**
     * Create the composite display frame.
     */
    cv::Mat createDisplayFrame(const cv::Mat& camera_frame,
                               const ParsedAction& action,
                               const std::deque<ActionHistoryEntry>& history) const;

    /**
     * Draw the reasoning panel.
     */
    void drawReasoningPanel(cv::Mat& display, const ParsedAction& action,
                           int x, int y, int width, int height) const;

    /**
     * Draw the history panel.
     */
    void drawHistoryPanel(cv::Mat& display, const std::deque<ActionHistoryEntry>& history,
                         int x, int y, int width, int height) const;

    /**
     * Draw the status bar.
     */
    void drawStatusBar(cv::Mat& display, int y) const;

    /**
     * Draw wrapped text (helper).
     */
    void drawWrappedText(cv::Mat& display, const std::string& text,
                        int x, int y, int max_width, int line_height,
                        const cv::Scalar& color, double font_scale) const;
};

}  // namespace greeter
