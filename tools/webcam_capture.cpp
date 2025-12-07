// Webcam capture tool - runs on dev machine
// Usage: ./webcam_capture [port] [output_dir]
//
// Receives webcam frames from robot via UDP and displays/saves them.

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "webcam/WebcamStreamServer.h"
#include <opencv2/opencv.hpp>

static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

void printHelp(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [port] [output_dir]" << std::endl;
    std::cout << std::endl;
    std::cout << "Receives webcam frames from robot and displays/saves them." << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  port        UDP port to listen on (default: 5002)" << std::endl;
    std::cout << "  output_dir  Directory to save frames (optional)" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  s     Save current frame" << std::endl;
    std::cout << "  r     Toggle recording (save all frames)" << std::endl;
    std::cout << "  q     Quit" << std::endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse arguments
    int port = WebcamStreamServer::DEFAULT_PORT;
    std::string output_dir;

    if (argc >= 2) {
        std::string arg1(argv[1]);
        if (arg1 == "--help" || arg1 == "-h") {
            printHelp(argv[0]);
            return 0;
        }
        port = std::stoi(arg1);
    }
    if (argc >= 3) {
        output_dir = argv[2];
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  Webcam Capture" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Port: " << port << std::endl;
    if (!output_dir.empty()) {
        std::cout << "Output: " << output_dir << std::endl;
        std::filesystem::create_directories(output_dir);
    }
    std::cout << std::endl;

    WebcamStreamClient client;

    std::cout << "Starting client..." << std::endl;
    if (!client.start(port)) {
        std::cerr << "Failed to start client" << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "Waiting for frames. Press 'q' to quit, 's' to save, 'r' to toggle recording." << std::endl;
    std::cout << std::endl;

    cv::namedWindow("Webcam Stream", cv::WINDOW_AUTOSIZE);

    bool recording = false;
    int saved_count = 0;
    auto last_fps_print = std::chrono::steady_clock::now();

    while (g_running) {
        auto frame = client.getLatestFrame();

        if (frame.valid && !frame.image.empty()) {
            // Add overlay info
            cv::Mat display = frame.image.clone();
            std::stringstream ss;
            ss << "Frame: " << frame.frame_number
               << " | FPS: " << std::fixed << std::setprecision(1) << client.getFps()
               << " | " << (recording ? "REC" : "");
            cv::putText(display, ss.str(), cv::Point(10, 25),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            cv::imshow("Webcam Stream", display);

            // Save if recording
            if (recording && !output_dir.empty()) {
                std::stringstream filename;
                filename << output_dir << "/webcam_"
                         << std::setfill('0') << std::setw(6) << frame.frame_number
                         << ".jpg";
                cv::imwrite(filename.str(), frame.image);
                saved_count++;
            }
        }

        // Handle key presses
        int key = cv::waitKey(16);  // ~60 Hz display refresh
        if (key == 'q' || key == 27) {  // q or ESC
            break;
        } else if (key == 's' && frame.valid) {
            // Save single frame
            std::string save_dir = output_dir.empty() ? "." : output_dir;
            std::filesystem::create_directories(save_dir);
            std::stringstream filename;
            filename << save_dir << "/webcam_snapshot_"
                     << std::setfill('0') << std::setw(6) << frame.frame_number
                     << ".jpg";
            cv::imwrite(filename.str(), frame.image);
            std::cout << "Saved: " << filename.str() << std::endl;
        } else if (key == 'r') {
            recording = !recording;
            if (recording && output_dir.empty()) {
                output_dir = "webcam_recording";
                std::filesystem::create_directories(output_dir);
            }
            std::cout << (recording ? "Recording started" : "Recording stopped")
                      << " (" << saved_count << " frames saved)" << std::endl;
        }

        // Print FPS periodically
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_fps_print).count() >= 2.0) {
            std::cout << "FPS: " << std::fixed << std::setprecision(1) << client.getFps()
                      << " | Frames: " << client.getFrameCount()
                      << " | Received: " << client.getBytesReceived() / 1024 / 1024 << " MB"
                      << std::endl;
            last_fps_print = now;
        }
    }

    std::cout << std::endl;
    std::cout << "Stopping..." << std::endl;
    cv::destroyAllWindows();
    client.stop();

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Done" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total frames: " << client.getFrameCount() << std::endl;
    std::cout << "Data received: " << client.getBytesReceived() / 1024 / 1024 << " MB" << std::endl;
    if (saved_count > 0) {
        std::cout << "Saved: " << saved_count << " frames" << std::endl;
    }

    return 0;
}
