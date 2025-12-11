// Test tool for RealSense D435i depth capture
// Usage: ./test_realsense [--save <output_dir>]
//
// Tests basic depth capture functionality and optionally saves sample frames.

#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <signal.h>

#ifdef HAS_REALSENSE
#include "capture/DepthCapture.h"
#endif

static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

int main(int argc, char** argv) {
#ifndef HAS_REALSENSE
    std::cerr << "Error: Built without RealSense support" << std::endl;
    std::cerr << "Make sure librealsense2 is installed and rebuild" << std::endl;
    return 1;
#else
    signal(SIGINT, signalHandler);

    std::string output_dir;
    bool save_frames = false;

    // Parse args
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--save" && i + 1 < argc) {
            output_dir = argv[++i];
            save_frames = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [--save <output_dir>]" << std::endl;
            std::cout << std::endl;
            std::cout << "Tests RealSense D435i depth capture." << std::endl;
            std::cout << "Press Ctrl+C to stop." << std::endl;
            return 0;
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  RealSense D435i Test Tool" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    DepthCapture capture;

    // Initialize
    std::cout << "[TEST] Initializing RealSense..." << std::endl;
    if (!capture.init()) {
        std::cerr << "[TEST] Failed to initialize RealSense" << std::endl;
        return 1;
    }
    std::cout << "[TEST] RealSense initialized successfully" << std::endl;

    // Start capture
    std::cout << "[TEST] Starting capture..." << std::endl;
    if (!capture.startCapture()) {
        std::cerr << "[TEST] Failed to start capture" << std::endl;
        return 1;
    }

    // Start recording if requested
    if (save_frames) {
        std::filesystem::create_directories(output_dir);
        if (!capture.startRecording(output_dir)) {
            std::cerr << "[TEST] Failed to start recording" << std::endl;
            return 1;
        }
        std::cout << "[TEST] Recording to: " << output_dir << std::endl;
    }

    std::cout << std::endl;
    std::cout << "[TEST] Streaming depth data. Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    uint64_t last_frame_count = 0;
    auto last_fps_time = start_time;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
        uint64_t current_count = capture.getFrameCount();
        double fps = (current_count - last_frame_count) / elapsed;

        DepthFrame frame = capture.getLatestFrame();

        std::cout << "[TEST] Frames: " << current_count
                  << " | FPS: " << std::fixed << std::setprecision(1) << fps;

        if (!frame.depth.empty()) {
            // Get some depth statistics
            double min_depth = 0, max_depth = 0;
            cv::minMaxLoc(frame.depth, &min_depth, &max_depth);

            std::cout << " | Depth range: " << (int)min_depth << "-" << (int)max_depth << " mm";
            std::cout << " | Size: " << frame.depth.cols << "x" << frame.depth.rows;
        }

        if (save_frames) {
            std::cout << " | Recorded: " << capture.getRecordedFrames();
        }

        std::cout << std::endl;

        last_frame_count = current_count;
        last_fps_time = now;
    }

    std::cout << std::endl;
    std::cout << "[TEST] Stopping..." << std::endl;

    if (save_frames) {
        capture.stopRecording();
    }
    capture.stopCapture();

    auto total_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Test Complete" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total frames: " << capture.getFrameCount() << std::endl;
    std::cout << "Total time: " << std::fixed << std::setprecision(1) << total_time << " seconds" << std::endl;
    std::cout << "Average FPS: " << std::fixed << std::setprecision(1)
              << (capture.getFrameCount() / total_time) << std::endl;

    if (save_frames) {
        std::cout << "Recorded frames: " << capture.getRecordedFrames() << std::endl;
        std::cout << "Output directory: " << output_dir << std::endl;
    }

    return 0;
#endif
}
