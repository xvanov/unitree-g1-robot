#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <mutex>

#include "replay/SensorReplayer.h"
#include "replay/ReplaySensorSource.h"
#include "replay/ReplayController.h"

namespace replay {

/**
 * ReplayRunner - High-level replay orchestrator with progress display
 *
 * Provides CLI integration and terminal-based progress display.
 */
class ReplayRunner {
public:
    ReplayRunner();
    ~ReplayRunner();

    // Configuration
    void setSessionId(const std::string& session_id);
    void setRecordingPath(const std::string& path);
    void setSpeed(float speed) { initial_speed_ = speed; }
    void setLoop(bool enabled) { loop_enabled_ = enabled; }
    void setVisualize(bool enabled) { visualize_ = enabled; }

    // Run replay (blocking)
    int run();

    // Get components for external access
    ReplaySensorSource* getSensorSource() { return source_.get(); }
    ReplayController* getController() { return controller_.get(); }

private:
    // Display progress in terminal
    void displayProgress();

    // Handle keyboard input for controls
    void handleKeyboard();

    // Print progress bar
    void printProgressBar(float progress, int width);

    // Format time as MM:SS.s
    std::string formatTime(float seconds);

    // Configuration
    std::string session_id_;
    std::string recording_path_;
    float initial_speed_ = 1.0f;
    bool loop_enabled_ = false;
    bool visualize_ = false;

    // Components
    std::unique_ptr<SensorReplayer> replayer_;
    std::unique_ptr<ReplaySensorSource> source_;
    std::unique_ptr<ReplayController> controller_;

    // State
    std::atomic<bool> running_{false};

    // Output synchronization to prevent garbled console output
    static std::mutex output_mutex_;
};

} // namespace replay
