#include "replay/ReplayRunner.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <opencv2/opencv.hpp>

// Reference to global running flag from main.cpp
extern std::atomic<bool> g_running;

namespace replay {

// Static mutex for output synchronization
std::mutex ReplayRunner::output_mutex_;

// Non-blocking keyboard input helper
static int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        return ch;
    }

    return 0;
}

ReplayRunner::ReplayRunner() = default;

ReplayRunner::~ReplayRunner() {
    if (source_) {
        source_->stop();
    }
}

void ReplayRunner::setSessionId(const std::string& session_id) {
    session_id_ = session_id;
    recording_path_ = "data/recordings/" + session_id;
}

void ReplayRunner::setRecordingPath(const std::string& path) {
    recording_path_ = path;
    // Extract session_id from path if not set
    if (session_id_.empty()) {
        std::filesystem::path p(path);
        session_id_ = p.filename().string();
    }
}

int ReplayRunner::run() {
    // Validate recording path
    if (recording_path_.empty()) {
        std::cerr << "[REPLAY] No recording path specified" << std::endl;
        return 1;
    }

    if (!std::filesystem::exists(recording_path_)) {
        std::cerr << "[REPLAY] Recording not found: " << recording_path_ << std::endl;
        return 1;
    }

    // Create components
    replayer_ = std::make_unique<SensorReplayer>();
    controller_ = std::make_unique<ReplayController>();
    source_ = std::make_unique<ReplaySensorSource>();

    // Open recording
    if (!replayer_->open(recording_path_)) {
        std::cerr << "[REPLAY] Failed to open recording" << std::endl;
        return 1;
    }

    // Initialize source
    if (!source_->init(replayer_.get(), controller_.get())) {
        std::cerr << "[REPLAY] Failed to initialize replay source" << std::endl;
        return 1;
    }

    // Configure controller
    controller_->setSpeed(initial_speed_);
    controller_->setLoop(loop_enabled_);

    // Start playback
    if (!source_->start()) {
        std::cerr << "[REPLAY] Failed to start playback" << std::endl;
        return 1;
    }

    running_.store(true);

    // Print header
    auto metadata = replayer_->getMetadata();
    std::cout << "\n[REPLAY] " << metadata.session_id << std::endl;
    std::cout << "Duration: " << formatTime(metadata.duration_seconds)
              << " | Messages: " << (metadata.lidar_count + metadata.imu_count +
                                     metadata.pose_count + metadata.image_count)
              << std::endl;
    std::cout << "Controls: [SPACE] pause  [+/-] speed  [L] loop  [Q] quit" << std::endl;
    std::cout << std::endl;

    // Optional visualization window
    cv::Mat viz_frame;
    if (visualize_) {
        cv::namedWindow("Replay", cv::WINDOW_AUTOSIZE);
    }

    // Main loop
    while (running_.load() && g_running.load() && !controller_->isFinished()) {
        // Handle keyboard input
        handleKeyboard();

        // Display progress
        displayProgress();

        // Update visualization if enabled
        if (visualize_ && source_->hasNewImage()) {
            viz_frame = source_->getImage();
            if (!viz_frame.empty()) {
                // Add overlay text
                std::stringstream ss;
                ss << formatTime(controller_->getElapsedTime()) << " / "
                   << formatTime(controller_->getTotalTime())
                   << " @ " << std::fixed << std::setprecision(1)
                   << controller_->getSpeed() << "x";

                cv::putText(viz_frame, ss.str(), cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                cv::imshow("Replay", viz_frame);
            }
        }

        // Handle OpenCV window events
        if (visualize_) {
            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q' || key == 27) {  // q, Q, or ESC
                running_.store(false);
            } else if (key == ' ') {
                if (controller_->isPaused()) {
                    controller_->resume();
                } else {
                    controller_->pause();
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Final status
    std::cout << "\r\033[K";  // Clear line
    if (controller_->isFinished()) {
        std::cout << "[REPLAY] Playback complete" << std::endl;
    } else {
        std::cout << "[REPLAY] Playback stopped" << std::endl;
    }

    // Print summary
    std::cout << "Replayed: LiDAR=" << controller_->getLidarReplayed()
              << " IMU=" << controller_->getImuReplayed()
              << " Pose=" << controller_->getPoseReplayed()
              << " Image=" << controller_->getImageReplayed()
              << " Video=" << controller_->getVideoFrameReplayed() << std::endl;

    // Cleanup
    source_->stop();

    if (visualize_) {
        cv::destroyAllWindows();
    }

    return 0;
}

void ReplayRunner::displayProgress() {
    float progress = controller_->getProgress();
    float elapsed = controller_->getElapsedTime();
    float total = controller_->getTotalTime();
    float speed = controller_->getSpeed();

    // Build status line
    std::stringstream ss;
    ss << "\rTime: " << formatTime(elapsed) << " / " << formatTime(total) << "  ";

    // Progress bar
    ss << "[";
    printProgressBar(progress, 20);
    ss << "] " << std::fixed << std::setprecision(0) << (progress * 100) << "%  ";

    // Speed and counts
    ss << "Speed: " << std::fixed << std::setprecision(1) << speed << "x  | ";
    ss << "L:" << controller_->getLidarReplayed() << " ";
    ss << "I:" << controller_->getImuReplayed() << " ";
    ss << "P:" << controller_->getPoseReplayed();

    // Status indicator
    if (controller_->isPaused()) {
        ss << " [PAUSED]";
    } else if (controller_->isLoopEnabled()) {
        ss << " [LOOP]";
    }

    ss << "   ";  // Extra space to clear previous content

    // Synchronized output to prevent garbled display
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << ss.str() << std::flush;
    }
}

void ReplayRunner::printProgressBar(float progress, int width) {
    int filled = static_cast<int>(progress * width);
    for (int i = 0; i < width; ++i) {
        if (i < filled) {
            std::cout << "=";
        } else if (i == filled) {
            std::cout << ">";
        } else {
            std::cout << " ";
        }
    }
}

void ReplayRunner::handleKeyboard() {
    int ch = kbhit();
    if (ch == 0) return;

    switch (ch) {
        case ' ':  // Space - pause/resume
            if (controller_->isPaused()) {
                controller_->resume();
            } else {
                controller_->pause();
            }
            break;

        case '+':
        case '=':  // Increase speed
            {
                float speed = controller_->getSpeed();
                if (speed < 4.0f) {
                    speed = std::min(4.0f, speed * 2.0f);
                    controller_->setSpeed(speed);
                }
            }
            break;

        case '-':
        case '_':  // Decrease speed
            {
                float speed = controller_->getSpeed();
                if (speed > 0.25f) {
                    speed = std::max(0.25f, speed / 2.0f);
                    controller_->setSpeed(speed);
                }
            }
            break;

        case 'l':
        case 'L':  // Toggle loop
            controller_->setLoop(!controller_->isLoopEnabled());
            break;

        case 'r':
        case 'R':  // Restart
            controller_->seekTo(0.0f);
            break;

        case 'q':
        case 'Q':  // Quit
            running_.store(false);
            break;

        case '[':  // Seek back 10%
            {
                float progress = controller_->getProgress();
                controller_->seekTo(std::max(0.0f, progress - 0.1f));
            }
            break;

        case ']':  // Seek forward 10%
            {
                float progress = controller_->getProgress();
                controller_->seekTo(std::min(1.0f, progress + 0.1f));
            }
            break;
    }
}

std::string ReplayRunner::formatTime(float seconds) {
    int mins = static_cast<int>(seconds) / 60;
    float secs = seconds - (mins * 60);

    std::stringstream ss;
    ss << std::setfill('0') << std::setw(2) << mins << ":"
       << std::setfill('0') << std::setw(4) << std::fixed << std::setprecision(1) << secs;
    return ss.str();
}

} // namespace replay
