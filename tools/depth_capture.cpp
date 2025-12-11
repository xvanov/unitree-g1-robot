// Depth capture tool - receives depth stream from robot and saves locally
// Usage: ./depth_capture [port] [--output dir] [--duration secs] [--preview]
//
// Receives RealSense depth+color frames over UDP from depth_stream_server
// running on the robot.

#include <iostream>
#include <fstream>
#include <csignal>
#include <thread>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "depth/DepthStreamClient.h"

static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    return ss.str();
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Receives depth stream from robot and saves frames locally." << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -p, --port PORT      UDP port to listen on (default: 5001)" << std::endl;
    std::cout << "  -o, --output DIR     Output directory (default: data/depth_capture/<timestamp>)" << std::endl;
    std::cout << "  -d, --duration SECS  Capture duration in seconds (default: unlimited)" << std::endl;
    std::cout << "  --preview            Show live preview window" << std::endl;
    std::cout << "  --no-save            Don't save frames, just preview" << std::endl;
    std::cout << "  -h, --help           Show this help" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << prog << " --port 5001 --preview" << std::endl;
    std::cout << "  " << prog << " -o data/my_capture -d 30" << std::endl;
    std::cout << std::endl;
    std::cout << "On robot, run: ./depth_stream_server <your_computer_ip> 5001" << std::endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse arguments
    int port = DepthStreamClient::DEFAULT_PORT;
    std::string output_dir;
    int duration = 0;  // 0 = unlimited
    bool preview = false;
    bool save_frames = true;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "-p" || arg == "--port") {
            if (i + 1 < argc) port = std::stoi(argv[++i]);
        } else if (arg == "-o" || arg == "--output") {
            if (i + 1 < argc) output_dir = argv[++i];
        } else if (arg == "-d" || arg == "--duration") {
            if (i + 1 < argc) duration = std::stoi(argv[++i]);
        } else if (arg == "--preview") {
            preview = true;
        } else if (arg == "--no-save") {
            save_frames = false;
        }
    }

    // Set default output directory
    if (output_dir.empty() && save_frames) {
        output_dir = "data/depth_capture/" + getTimestamp();
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  Depth Capture Tool" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Listening on port: " << port << std::endl;
    if (save_frames) {
        std::cout << "Output directory: " << output_dir << std::endl;
    } else {
        std::cout << "Output: disabled (preview only)" << std::endl;
    }
    if (duration > 0) {
        std::cout << "Duration: " << duration << " seconds" << std::endl;
    }
    if (preview) {
        std::cout << "Preview: enabled" << std::endl;
    }
    std::cout << std::endl;

    // Create output directories
    std::string color_dir, depth_dir;
    if (save_frames) {
        color_dir = output_dir + "/color";
        depth_dir = output_dir + "/depth";
        std::filesystem::create_directories(color_dir);
        std::filesystem::create_directories(depth_dir);
        std::cout << "Created output directories." << std::endl;
    }

    // Start depth client
    DepthStreamClient client;

    std::cout << "Starting UDP listener..." << std::endl;
    if (!client.start(port)) {
        std::cerr << "Failed to start depth client on port " << port << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "Waiting for depth stream from robot..." << std::endl;
    std::cout << "(Run on robot: ./depth_stream_server <your_ip> " << port << ")" << std::endl;
    std::cout << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    uint64_t last_frame_count = 0;
    uint64_t saved_count = 0;
    auto last_status_time = start_time;

    // Intrinsics file (write once)
    bool intrinsics_written = false;

    while (g_running) {
        // Check duration
        if (duration > 0) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= duration) {
                std::cout << std::endl << "Duration reached, stopping..." << std::endl;
                break;
            }
        }

        // Get latest frame
        auto frame = client.getLatestFrame();

        if (frame.valid && frame.frame_number != last_frame_count) {
            last_frame_count = frame.frame_number;

            // Save intrinsics once
            if (!intrinsics_written && save_frames) {
                std::ofstream intrinsics_file(output_dir + "/intrinsics.txt");
                intrinsics_file << "# RealSense D435i Camera Intrinsics" << std::endl;
                intrinsics_file << "width: " << frame.width << std::endl;
                intrinsics_file << "height: " << frame.height << std::endl;
                intrinsics_file << "fx: " << frame.fx << std::endl;
                intrinsics_file << "fy: " << frame.fy << std::endl;
                intrinsics_file << "cx: " << frame.cx << std::endl;
                intrinsics_file << "cy: " << frame.cy << std::endl;
                intrinsics_file << "depth_scale: " << frame.depth_scale << std::endl;
                intrinsics_file.close();
                intrinsics_written = true;
                std::cout << "Saved camera intrinsics." << std::endl;
            }

            // Save frames
            if (save_frames && !frame.color.empty() && !frame.depth.empty()) {
                std::stringstream ss;
                ss << std::setfill('0') << std::setw(6) << saved_count;
                std::string frame_id = ss.str();

                cv::imwrite(color_dir + "/" + frame_id + ".jpg", frame.color);
                cv::imwrite(depth_dir + "/" + frame_id + ".png", frame.depth);
                saved_count++;
            }

            // Preview
            if (preview && !frame.color.empty()) {
                cv::imshow("Color", frame.color);

                if (!frame.depth.empty()) {
                    // Normalize depth for display (0-10m -> 0-255)
                    cv::Mat depth_vis;
                    frame.depth.convertTo(depth_vis, CV_8U, 255.0 / 10000.0);
                    cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
                    cv::imshow("Depth", depth_vis);
                }

                int key = cv::waitKey(1);
                if (key == 27 || key == 'q') {  // ESC or 'q'
                    g_running = false;
                }
            }
        }

        // Print status every second
        auto now = std::chrono::steady_clock::now();
        auto status_elapsed = now - last_status_time;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(status_elapsed).count() >= 1000) {
            auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            float fps = client.getFps();
            uint64_t bytes = client.getBytesReceived();
            float mbps = bytes / (1024.0f * 1024.0f);

            std::cout << "\r[" << total_elapsed << "s] Frames: " << client.getFrameCount()
                      << " | Saved: " << saved_count
                      << " | FPS: " << std::fixed << std::setprecision(1) << fps
                      << " | Data: " << std::setprecision(1) << mbps << " MB"
                      << "     " << std::flush;

            last_status_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << std::endl << std::endl;
    std::cout << "Stopping..." << std::endl;
    client.stop();

    if (preview) {
        cv::destroyAllWindows();
    }

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Capture Complete" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total frames received: " << client.getFrameCount() << std::endl;
    std::cout << "Frames saved: " << saved_count << std::endl;
    std::cout << "Data received: " << client.getBytesReceived() / (1024 * 1024) << " MB" << std::endl;
    if (save_frames && saved_count > 0) {
        std::cout << "Output: " << output_dir << std::endl;
    }

    return 0;
}
