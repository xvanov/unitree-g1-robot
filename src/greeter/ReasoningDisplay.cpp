#include "greeter/ReasoningDisplay.h"
#include <iostream>
#include <sstream>
#include <iomanip>

namespace greeter {

ReasoningDisplay::ReasoningDisplay() = default;

ReasoningDisplay::~ReasoningDisplay() {
    close();
}

bool ReasoningDisplay::init(const std::string& window_name) {
    window_name_ = window_name;

    try {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        initialized_ = true;
        std::cout << "[ReasoningDisplay] Window initialized: " << window_name_ << std::endl;
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "[ReasoningDisplay] Failed to create window: " << e.what() << std::endl;
        return false;
    }
}

void ReasoningDisplay::update(const cv::Mat& frame, const ParsedAction& action,
                              const std::deque<ActionHistoryEntry>& history) {
    if (!initialized_) return;

    cv::Mat display = createDisplayFrame(frame, action, history);
    cv::imshow(window_name_, display);
}

void ReasoningDisplay::showFrame(const cv::Mat& frame) {
    if (!initialized_) return;

    // Create display with just the frame and status bar
    cv::Mat display(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3, cv::Scalar(30, 30, 30));

    // Resize and place camera frame
    cv::Mat resized_frame;
    if (!frame.empty()) {
        cv::resize(frame, resized_frame, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));
        resized_frame.copyTo(display(cv::Rect(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT)));
    }

    // Draw placeholder for reasoning panel
    cv::rectangle(display, cv::Rect(CAMERA_WIDTH, 0, PANEL_WIDTH, DISPLAY_HEIGHT - STATUS_HEIGHT),
                  cv::Scalar(40, 40, 40), -1);
    cv::putText(display, "Waiting for LLM response...",
                cv::Point(CAMERA_WIDTH + 20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(150, 150, 150), 1);

    // Draw status bar
    drawStatusBar(display, DISPLAY_HEIGHT - STATUS_HEIGHT);

    cv::imshow(window_name_, display);
}

void ReasoningDisplay::setStatus(const std::string& status) {
    status_text_ = status;
}

bool ReasoningDisplay::isOpen() const {
    if (!initialized_) return false;

    try {
        // Check if window still exists
        return cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) >= 0;
    } catch (...) {
        return false;
    }
}

int ReasoningDisplay::processEvents(int wait_ms) {
    return cv::waitKey(wait_ms);
}

void ReasoningDisplay::close() {
    if (initialized_) {
        try {
            cv::destroyWindow(window_name_);
        } catch (...) {
            // Ignore errors during shutdown
        }
        initialized_ = false;
    }
}

cv::Mat ReasoningDisplay::createDisplayFrame(const cv::Mat& camera_frame,
                                              const ParsedAction& action,
                                              const std::deque<ActionHistoryEntry>& history) const {
    cv::Mat display(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3, cv::Scalar(30, 30, 30));

    // Resize and place camera frame on the left
    if (!camera_frame.empty()) {
        cv::Mat resized_frame;
        cv::resize(camera_frame, resized_frame, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));
        resized_frame.copyTo(display(cv::Rect(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT)));
    }

    // Calculate panel heights
    int panel_height = DISPLAY_HEIGHT - STATUS_HEIGHT;
    int reasoning_height = panel_height * 2 / 3;  // 2/3 for reasoning
    int history_height = panel_height - reasoning_height;  // 1/3 for history

    // Draw reasoning panel (right side, top)
    drawReasoningPanel(display, action, CAMERA_WIDTH, 0, PANEL_WIDTH, reasoning_height);

    // Draw history panel (right side, bottom)
    drawHistoryPanel(display, history, CAMERA_WIDTH, reasoning_height, PANEL_WIDTH, history_height);

    // Draw status bar (full width, bottom)
    drawStatusBar(display, DISPLAY_HEIGHT - STATUS_HEIGHT);

    return display;
}

void ReasoningDisplay::drawReasoningPanel(cv::Mat& display, const ParsedAction& action,
                                          int x, int y, int width, int height) const {
    // Background
    cv::rectangle(display, cv::Rect(x, y, width, height), cv::Scalar(40, 40, 40), -1);

    // Border
    cv::rectangle(display, cv::Rect(x, y, width, height), cv::Scalar(70, 70, 70), 1);

    int padding = 15;
    int line_height = 22;
    int current_y = y + padding + line_height;

    // Header
    cv::putText(display, "LLM REASONING",
                cv::Point(x + padding, current_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100, 200, 100), 2);
    current_y += line_height + 10;

    // Action type with color coding
    std::string action_str = ActionParser::actionTypeToString(action.type);
    cv::Scalar action_color = cv::Scalar(100, 200, 255);  // Default cyan

    // Color code dangerous actions
    if (action.type == ActionType::PUSH_FORWARD) {
        action_color = cv::Scalar(0, 0, 255);  // Red
    } else if (action.type == ActionType::FOLLOW || action.type == ActionType::MOVE_FORWARD) {
        action_color = cv::Scalar(0, 165, 255);  // Orange
    }

    cv::putText(display, "Action: " + action_str,
                cv::Point(x + padding, current_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, action_color, 2);
    current_y += line_height;

    // Confidence
    std::ostringstream conf_ss;
    conf_ss << "Confidence: " << std::fixed << std::setprecision(0) << (action.confidence * 100) << "%";
    cv::putText(display, conf_ss.str(),
                cv::Point(x + padding, current_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    current_y += line_height + 10;

    // Intent
    cv::putText(display, "Intent:",
                cv::Point(x + padding, current_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 150, 200), 1);
    current_y += line_height - 5;

    drawWrappedText(display, action.intent,
                   x + padding, current_y, width - 2 * padding, line_height - 4,
                   cv::Scalar(220, 220, 220), 0.45);
    current_y += (line_height - 4) * 2 + 10;

    // Reasoning
    cv::putText(display, "Reasoning:",
                cv::Point(x + padding, current_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 150, 200), 1);
    current_y += line_height - 5;

    drawWrappedText(display, action.reasoning,
                   x + padding, current_y, width - 2 * padding, line_height - 4,
                   cv::Scalar(200, 200, 200), 0.4);

    // If PUSH action, add prominent warning
    if (action.type == ActionType::PUSH_FORWARD) {
        int warn_y = y + height - 60;
        cv::rectangle(display, cv::Rect(x + 10, warn_y, width - 20, 50),
                      cv::Scalar(0, 0, 100), -1);
        cv::putText(display, "! PUSH ACTION DETECTED !",
                    cv::Point(x + width/2 - 100, warn_y + 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
}

void ReasoningDisplay::drawHistoryPanel(cv::Mat& display, const std::deque<ActionHistoryEntry>& history,
                                        int x, int y, int width, int height) const {
    // Background
    cv::rectangle(display, cv::Rect(x, y, width, height), cv::Scalar(35, 35, 35), -1);

    // Border
    cv::rectangle(display, cv::Rect(x, y, width, height), cv::Scalar(70, 70, 70), 1);

    int padding = 15;
    int line_height = 20;
    int current_y = y + padding + line_height;

    // Header
    cv::putText(display, "ACTION HISTORY",
                cv::Point(x + padding, current_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 150, 100), 1);
    current_y += line_height + 5;

    // Draw history entries (most recent first)
    int count = 0;
    for (auto it = history.rbegin(); it != history.rend() && count < max_history_display_; ++it, ++count) {
        const auto& entry = *it;

        // Format: [timestamp] ACTION - intent (confidence%)
        std::ostringstream ss;
        ss << entry.timestamp << " " << entry.action_type;
        if (!entry.intent.empty()) {
            ss << " - " << entry.intent.substr(0, 30);
            if (entry.intent.length() > 30) ss << "...";
        }
        ss << " (" << static_cast<int>(entry.confidence * 100) << "%)";

        cv::Scalar color = (count == 0) ? cv::Scalar(220, 220, 220) : cv::Scalar(150, 150, 150);
        cv::putText(display, ss.str(),
                    cv::Point(x + padding, current_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
        current_y += line_height;
    }

    if (history.empty()) {
        cv::putText(display, "(no actions yet)",
                    cv::Point(x + padding, current_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 100, 100), 1);
    }
}

void ReasoningDisplay::drawStatusBar(cv::Mat& display, int y) const {
    // Background
    cv::rectangle(display, cv::Rect(0, y, DISPLAY_WIDTH, STATUS_HEIGHT),
                  cv::Scalar(50, 50, 50), -1);

    // Status text
    cv::putText(display, status_text_,
                cv::Point(15, y + 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);

    // Keyboard hints on the right
    std::string hints = "ESC: Quit | SPACE: Pause | R: Reset";
    int text_width = cv::getTextSize(hints, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, nullptr).width;
    cv::putText(display, hints,
                cv::Point(DISPLAY_WIDTH - text_width - 15, y + 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);
}

void ReasoningDisplay::drawWrappedText(cv::Mat& display, const std::string& text,
                                       int x, int y, int max_width, int line_height,
                                       const cv::Scalar& color, double font_scale) const {
    if (text.empty()) return;

    std::istringstream iss(text);
    std::string word;
    std::string line;
    int current_y = y;
    int max_lines = 8;  // Limit lines to prevent overflow
    int line_count = 0;

    while (iss >> word && line_count < max_lines) {
        std::string test_line = line.empty() ? word : line + " " + word;
        int text_width = cv::getTextSize(test_line, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, nullptr).width;

        if (text_width > max_width && !line.empty()) {
            cv::putText(display, line,
                        cv::Point(x, current_y),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale, color, 1);
            current_y += line_height;
            line = word;
            line_count++;
        } else {
            line = test_line;
        }
    }

    // Draw last line
    if (!line.empty() && line_count < max_lines) {
        if (iss.rdbuf()->in_avail() > 0) {
            line += "...";  // Indicate truncation
        }
        cv::putText(display, line,
                    cv::Point(x, current_y),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, color, 1);
    }
}

}  // namespace greeter
