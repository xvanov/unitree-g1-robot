// Stream viewer - receives all streams from robot OR replays from recordings
// Usage: ./stream_viewer [--no-depth] [--no-webcam]
//        ./stream_viewer --replay <session> [--replay-speed <n>] [--replay-loop]
//
// Live mode: Receives and displays:
//   - RealSense RGB + Depth on port 5001
//   - USB Webcam on port 5002
//
// Replay mode: Plays back recorded sessions with RGB + Depth

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <atomic>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "depth/DepthStreamClient.h"
#include "webcam/WebcamStreamServer.h"
#include "replay/SensorReplayer.h"
#include "replay/ReplayController.h"

static volatile bool g_running = true;
std::atomic<bool> g_running_atomic{true};

// Required for replay library
std::atomic<bool>& getGlobalRunning() {
    return g_running_atomic;
}

void signalHandler(int) {
    g_running = false;
    g_running_atomic.store(false);
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Receives all streams from robot OR replays recorded sessions." << std::endl;
    std::cout << std::endl;
    std::cout << "Live Mode Options:" << std::endl;
    std::cout << "  --no-depth           Disable depth stream (port 5001)" << std::endl;
    std::cout << "  --no-webcam          Disable webcam stream (port 5002)" << std::endl;
    std::cout << "  --depth-port PORT    Depth stream port (default: 5001)" << std::endl;
    std::cout << "  --webcam-port PORT   Webcam stream port (default: 5002)" << std::endl;
    std::cout << std::endl;
    std::cout << "Replay Mode Options:" << std::endl;
    std::cout << "  --replay <session>   Replay a recorded session (e.g., --replay 008)" << std::endl;
    std::cout << "  --replay-speed <n>   Playback speed multiplier (default: 1.0)" << std::endl;
    std::cout << "  --replay-loop        Loop playback continuously" << std::endl;
    std::cout << std::endl;
    std::cout << "  -h, --help           Show this help" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  q, ESC   Quit" << std::endl;
    std::cout << "  s        Save snapshot of all streams" << std::endl;
    std::cout << "  SPACE    Pause/Resume (replay mode)" << std::endl;
    std::cout << "  +/-      Speed up/down (replay mode)" << std::endl;
    std::cout << "  [/]      Seek back/forward 10% (replay mode)" << std::endl;
    std::cout << "  r        Restart (replay mode)" << std::endl;
    std::cout << "  l        Toggle loop (replay mode)" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << prog << "                          # Live streams" << std::endl;
    std::cout << "  " << prog << " --replay 008             # Replay session 008" << std::endl;
    std::cout << "  " << prog << " --replay 008 --replay-speed 0.5  # Half speed" << std::endl;
}

std::string formatTime(float seconds) {
    int mins = static_cast<int>(seconds) / 60;
    float secs = seconds - (mins * 60);
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(2) << mins << ":"
       << std::setfill('0') << std::setw(4) << std::fixed << std::setprecision(1) << secs;
    return ss.str();
}

// Simple frame-to-frame timing for replay
struct ReplayTiming {
    int64_t last_frame_recording_us = 0;  // Recording timestamp of last displayed frame

    void reset() {
        last_frame_recording_us = 0;
    }

    // Calculate how long to wait before showing this frame (based on gap from previous frame)
    int64_t getWaitMs(int64_t recording_time_us, float speed) {
        if (last_frame_recording_us == 0) {
            // First frame - no wait
            last_frame_recording_us = recording_time_us;
            return 0;
        }

        // Time gap between this frame and previous in recording
        int64_t recording_gap_us = recording_time_us - last_frame_recording_us;
        last_frame_recording_us = recording_time_us;

        // Adjust for playback speed
        int64_t wait_us = static_cast<int64_t>(recording_gap_us / speed);
        int64_t wait_ms = wait_us / 1000;

        // Clamp to reasonable range
        if (wait_ms < 0) wait_ms = 0;
        if (wait_ms > 500) wait_ms = 500;  // Cap at 500ms

        return wait_ms;
    }
};

// Helper to process keyboard input and return true if should exit
bool handleReplayKey(int key, replay::ReplayController& controller, replay::SensorReplayer& replayer,
                     ReplayTiming& timing, const cv::Mat& color_frame, const cv::Mat& depth_vis) {
    if (key == -1) return false;  // No key pressed

    if (key == 'q' || key == 'Q' || key == 27) {
        return true;  // Signal exit
    }
    else if (key == ' ') {
        if (controller.isPaused()) {
            controller.resume();
        } else {
            controller.pause();
        }
    }
    else if (key == '+' || key == '=') {
        float new_speed = std::min(4.0f, controller.getSpeed() * 2.0f);
        controller.setSpeed(new_speed);
    }
    else if (key == '-' || key == '_') {
        float new_speed = std::max(0.25f, controller.getSpeed() / 2.0f);
        controller.setSpeed(new_speed);
    }
    else if (key == 'l' || key == 'L') {
        controller.setLoop(!controller.isLoopEnabled());
    }
    else if (key == 'r' || key == 'R') {
        // Restart - seek to beginning
        replayer.seek(0.0f);
        timing.reset();
    }
    else if (key == '[') {
        float target = std::max(0.0f, replayer.getProgress() - 0.1f);
        replayer.seek(target);
        timing.reset();
    }
    else if (key == ']') {
        float target = std::min(1.0f, replayer.getProgress() + 0.1f);
        replayer.seek(target);
        timing.reset();
    }
    else if (key == 's') {
        // Save snapshot
        auto now_t = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now_t);
        std::stringstream ts;
        ts << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
        if (!color_frame.empty()) {
            cv::imwrite("snapshot_rgb_" + ts.str() + ".jpg", color_frame);
            std::cout << "\nSaved: snapshot_rgb_" << ts.str() << ".jpg" << std::endl;
        }
        if (!depth_vis.empty()) {
            cv::imwrite("snapshot_depth_" + ts.str() + ".png", depth_vis);
            std::cout << "Saved: snapshot_depth_" << ts.str() << ".png" << std::endl;
        }
    }
    return false;
}

// Run replay mode
int runReplayMode(const std::string& session_id, float speed, bool loop) {
    std::string recording_path = "data/recordings/" + session_id;

    if (!std::filesystem::exists(recording_path)) {
        std::cerr << "Recording not found: " << recording_path << std::endl;
        return 1;
    }

    // Create replayer and controller
    replay::SensorReplayer replayer;
    replay::ReplayController controller;

    if (!replayer.open(recording_path)) {
        std::cerr << "Failed to open recording" << std::endl;
        return 1;
    }

    auto metadata = replayer.getMetadata();
    controller.setTotalTime(metadata.duration_seconds);
    controller.setSpeed(speed);
    controller.setLoop(loop);

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Replay: " << metadata.session_id << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Duration: " << formatTime(metadata.duration_seconds) << std::endl;
    std::cout << "Frames: Video=" << metadata.video_frame_count
              << " Depth=" << metadata.depth_frame_count << std::endl;
    std::cout << std::endl;
    std::cout << "Controls: [SPACE] pause  [+/-] speed  [L] loop  [R] restart" << std::endl;
    std::cout << "          [/] seek  [S] snapshot  [Q] quit" << std::endl;
    std::cout << std::endl;

    cv::namedWindow("Replay", cv::WINDOW_AUTOSIZE);

    // Frame data
    cv::Mat color_frame, depth_vis;
    uint32_t frame_count = 0;
    uint32_t depth_count = 0;

    // Timing - simple frame-to-frame
    ReplayTiming timing;
    timing.reset();

    controller.play();
    bool finished = false;

    // Only wait/display on visual frames, skip IMU/lidar for speed
    int64_t last_display_time_us = 0;

    replay::ReplayMessage msg;
    while (g_running && !finished) {
        // Handle pause - just wait and process keys
        while (controller.isPaused() && g_running) {
            int key = cv::waitKey(50);
            if (handleReplayKey(key, controller, replayer, timing, color_frame, depth_vis)) {
                g_running = false;
                break;
            }
        }
        if (!g_running) break;

        // Read next message
        if (!replayer.readNext(msg)) {
            if (controller.isLoopEnabled()) {
                replayer.seek(0.0f);
                timing.reset();
                continue;
            }
            finished = true;
            break;
        }

        // Only process DEPTH_FRAME - it has both color and depth
        // Skip IMU, LiDAR, VIDEO_FRAME (webcam) for now
        if (msg.type != recording::MessageType::DEPTH_FRAME) {
            continue;  // Skip non-depth messages entirely
        }

        // Calculate wait time
        float current_speed = controller.getSpeed();
        int64_t wait_ms = timing.getWaitMs(msg.timestamp_us, current_speed);

        // Decode depth frame (do this during wait if possible)
        auto decode_start = std::chrono::steady_clock::now();

        replay::DecodedDepthFrame decoded;
        if (!replay::SensorReplayer::decodeDepthFrame(msg, decoded)) {
            continue;
        }

        // Decode color JPEG
        if (!decoded.color_jpeg.empty()) {
            color_frame = cv::imdecode(decoded.color_jpeg, cv::IMREAD_COLOR);
        }
        // Decode depth PNG
        if (!decoded.depth_png.empty()) {
            cv::Mat depth_raw = cv::imdecode(decoded.depth_png, cv::IMREAD_UNCHANGED);
            if (!depth_raw.empty()) {
                depth_raw.convertTo(depth_vis, CV_8U, 255.0 / 10000.0);
                cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
            }
        }
        depth_count++;

        // Subtract decode time from wait
        auto decode_end = std::chrono::steady_clock::now();
        int64_t decode_ms = std::chrono::duration_cast<std::chrono::milliseconds>(decode_end - decode_start).count();
        int64_t remaining_wait_ms = wait_ms - decode_ms;

        // Wait remaining time (use waitKey which also handles keyboard)
        int key = -1;
        if (remaining_wait_ms > 1) {
            key = cv::waitKey(static_cast<int>(remaining_wait_ms));
        } else {
            key = cv::waitKey(1);  // Minimum 1ms for event processing
        }

        if (handleReplayKey(key, controller, replayer, timing, color_frame, depth_vis)) {
            break;
        }

        // Update progress
        float elapsed_s = static_cast<float>(msg.timestamp_us - metadata.start_time_us) / 1000000.0f;
        controller.setElapsedTime(elapsed_s);
        controller.setProgress(replayer.getProgress());

        // Build and show display
        if (!color_frame.empty() || !depth_vis.empty()) {
            std::vector<cv::Mat> images;
            std::vector<std::string> labels;

            if (!color_frame.empty()) {
                images.push_back(color_frame);
                labels.push_back("RGB");
            }
            if (!depth_vis.empty()) {
                images.push_back(depth_vis);
                labels.push_back("Depth");
            }

            // Resize all to same height
            int target_height = 480;
            std::vector<cv::Mat> resized;
            int total_width = 0;

            for (size_t i = 0; i < images.size(); i++) {
                cv::Mat img = images[i];
                float scale = static_cast<float>(target_height) / img.rows;
                int new_width = static_cast<int>(img.cols * scale);

                cv::Mat r;
                cv::resize(img, r, cv::Size(new_width, target_height));

                // Add label
                cv::putText(r, labels[i], cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

                resized.push_back(r);
                total_width += new_width;
            }

            // Concatenate horizontally
            cv::Mat display(target_height, total_width, CV_8UC3);
            int x_offset = 0;
            for (const auto& img : resized) {
                img.copyTo(display(cv::Rect(x_offset, 0, img.cols, img.rows)));
                x_offset += img.cols;
            }

            // Add status bar at bottom
            std::stringstream ss;
            ss << formatTime(controller.getElapsedTime()) << " / "
               << formatTime(controller.getTotalTime())
               << "  [" << std::fixed << std::setprecision(0)
               << (replayer.getProgress() * 100) << "%]"
               << "  Speed: " << std::setprecision(1) << controller.getSpeed() << "x";
            if (controller.isPaused()) ss << "  [PAUSED]";
            if (controller.isLoopEnabled()) ss << "  [LOOP]";

            cv::putText(display, ss.str(), cv::Point(10, target_height - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            cv::imshow("Replay", display);
        }
    }

    cv::destroyAllWindows();

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Replay Complete" << std::endl;
    std::cout << "  Frames: " << frame_count << " video, " << depth_count << " depth" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}

// Run live stream mode
int runLiveMode(bool enable_depth, bool enable_webcam, int depth_port, int webcam_port) {
    std::cout << "========================================" << std::endl;
    std::cout << "  Robot Stream Viewer" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Start clients
    std::unique_ptr<DepthStreamClient> depth_client;
    std::unique_ptr<WebcamStreamClient> webcam_client;

    if (enable_depth) {
        depth_client = std::make_unique<DepthStreamClient>();
        if (!depth_client->start(depth_port)) {
            std::cerr << "Warning: Failed to start depth client on port " << depth_port << std::endl;
            depth_client.reset();
        } else {
            std::cout << "Depth stream: listening on port " << depth_port << std::endl;
        }
    }

    if (enable_webcam) {
        webcam_client = std::make_unique<WebcamStreamClient>();
        if (!webcam_client->start(webcam_port)) {
            std::cerr << "Warning: Failed to start webcam client on port " << webcam_port << std::endl;
            webcam_client.reset();
        } else {
            std::cout << "Webcam stream: listening on port " << webcam_port << std::endl;
        }
    }

    if (!depth_client && !webcam_client) {
        std::cerr << "No streams available!" << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "Waiting for streams... Press 'q' to quit, 's' to save snapshot." << std::endl;
    std::cout << std::endl;

    cv::namedWindow("Robot Streams", cv::WINDOW_AUTOSIZE);

    auto start_time = std::chrono::steady_clock::now();
    auto last_status_time = start_time;

    // Latest frames
    cv::Mat depth_color, depth_vis, webcam_frame;
    uint32_t last_depth_frame = 0;
    uint32_t last_webcam_frame = 0;

    while (g_running) {
        bool updated = false;

        // Get depth frame
        if (depth_client) {
            auto frame = depth_client->getLatestFrame();
            if (frame.valid && frame.frame_number != last_depth_frame) {
                last_depth_frame = frame.frame_number;
                depth_color = frame.color.clone();

                if (!frame.depth.empty()) {
                    // Normalize depth for display (0-10m -> 0-255)
                    frame.depth.convertTo(depth_vis, CV_8U, 255.0 / 10000.0);
                    cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
                }
                updated = true;
            }
        }

        // Get webcam frame
        if (webcam_client) {
            auto frame = webcam_client->getLatestFrame();
            if (frame.valid && frame.frame_number != last_webcam_frame) {
                last_webcam_frame = frame.frame_number;
                webcam_frame = frame.image.clone();
                updated = true;
            }
        }

        // Build composite display
        if (updated || (!depth_color.empty() || !webcam_frame.empty())) {
            // Determine layout - all images side by side
            std::vector<cv::Mat> images;
            std::vector<std::string> labels;

            if (!depth_color.empty()) {
                images.push_back(depth_color);
                labels.push_back("RGB (RealSense)");
            }
            if (!depth_vis.empty()) {
                images.push_back(depth_vis);
                labels.push_back("Depth");
            }
            if (!webcam_frame.empty()) {
                images.push_back(webcam_frame);
                labels.push_back("Webcam");
            }

            if (!images.empty()) {
                // Resize all to same height
                int target_height = 480;
                std::vector<cv::Mat> resized;
                int total_width = 0;

                for (size_t i = 0; i < images.size(); i++) {
                    cv::Mat img = images[i];
                    float scale = static_cast<float>(target_height) / img.rows;
                    int new_width = static_cast<int>(img.cols * scale);

                    cv::Mat r;
                    cv::resize(img, r, cv::Size(new_width, target_height));

                    // Add label
                    cv::putText(r, labels[i], cv::Point(10, 30),
                               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

                    resized.push_back(r);
                    total_width += new_width;
                }

                // Concatenate horizontally
                cv::Mat display(target_height, total_width, CV_8UC3);
                int x_offset = 0;
                for (const auto& img : resized) {
                    img.copyTo(display(cv::Rect(x_offset, 0, img.cols, img.rows)));
                    x_offset += img.cols;
                }

                // Add FPS info at bottom
                std::stringstream ss;
                ss << "Depth FPS: " << std::fixed << std::setprecision(1)
                   << (depth_client ? depth_client->getFps() : 0.0f)
                   << " | Webcam FPS: "
                   << (webcam_client ? webcam_client->getFps() : 0.0f);
                cv::putText(display, ss.str(), cv::Point(10, target_height - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

                cv::imshow("Robot Streams", display);
            }
        }

        // Handle key presses
        int key = cv::waitKey(16);  // ~60 Hz
        if (key == 'q' || key == 27) {  // q or ESC
            break;
        } else if (key == 's') {
            // Save snapshot
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);
            std::stringstream ts;
            ts << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");

            if (!depth_color.empty()) {
                cv::imwrite("snapshot_rgb_" + ts.str() + ".jpg", depth_color);
                std::cout << "Saved: snapshot_rgb_" << ts.str() << ".jpg" << std::endl;
            }
            if (!depth_vis.empty()) {
                cv::imwrite("snapshot_depth_" + ts.str() + ".png", depth_vis);
                std::cout << "Saved: snapshot_depth_" << ts.str() << ".png" << std::endl;
            }
            if (!webcam_frame.empty()) {
                cv::imwrite("snapshot_webcam_" + ts.str() + ".jpg", webcam_frame);
                std::cout << "Saved: snapshot_webcam_" << ts.str() << ".jpg" << std::endl;
            }
        }

        // Print status every 2 seconds
        auto now = std::chrono::steady_clock::now();
        auto status_elapsed = now - last_status_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(status_elapsed).count() >= 2) {
            auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

            std::cout << "\r[" << total_elapsed << "s]";
            if (depth_client) {
                std::cout << " Depth: " << depth_client->getFrameCount() << " frames, "
                          << std::fixed << std::setprecision(1) << depth_client->getFps() << " fps";
            }
            if (webcam_client) {
                std::cout << " | Webcam: " << webcam_client->getFrameCount() << " frames, "
                          << std::fixed << std::setprecision(1) << webcam_client->getFps() << " fps";
                if (!webcam_frame.empty()) {
                    std::cout << " (" << webcam_frame.cols << "x" << webcam_frame.rows << ")";
                }
            }
            std::cout << "     " << std::flush;

            last_status_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << std::endl << std::endl;
    std::cout << "Stopping..." << std::endl;

    cv::destroyAllWindows();

    if (depth_client) depth_client->stop();
    if (webcam_client) webcam_client->stop();

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Done" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse arguments
    bool enable_depth = true;
    bool enable_webcam = true;
    int depth_port = 5001;
    int webcam_port = 5002;

    // Replay options
    std::string replay_session;
    float replay_speed = 1.0f;
    bool replay_loop = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--no-depth") {
            enable_depth = false;
        } else if (arg == "--no-webcam") {
            enable_webcam = false;
        } else if (arg == "--depth-port" && i + 1 < argc) {
            depth_port = std::stoi(argv[++i]);
        } else if (arg == "--webcam-port" && i + 1 < argc) {
            webcam_port = std::stoi(argv[++i]);
        } else if (arg == "--replay" && i + 1 < argc) {
            replay_session = argv[++i];
        } else if (arg == "--replay-speed" && i + 1 < argc) {
            replay_speed = std::stof(argv[++i]);
            if (replay_speed < 0.25f || replay_speed > 4.0f) {
                std::cerr << "Error: --replay-speed must be between 0.25 and 4.0" << std::endl;
                return 1;
            }
        } else if (arg == "--replay-loop") {
            replay_loop = true;
        }
    }

    // Replay mode or live mode?
    if (!replay_session.empty()) {
        return runReplayMode(replay_session, replay_speed, replay_loop);
    } else {
        return runLiveMode(enable_depth, enable_webcam, depth_port, webcam_port);
    }
}
