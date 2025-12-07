// Stream viewer - receives all streams from robot and displays in unified window
// Usage: ./stream_viewer [--no-depth] [--no-webcam]
//
// Receives and displays:
//   - RealSense RGB + Depth on port 5001
//   - USB Webcam on port 5002

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>

#include <opencv2/opencv.hpp>

#include "depth/DepthStreamClient.h"
#include "webcam/WebcamStreamServer.h"

static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Receives all streams from robot and displays in unified window." << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --no-depth           Disable depth stream (port 5001)" << std::endl;
    std::cout << "  --no-webcam          Disable webcam stream (port 5002)" << std::endl;
    std::cout << "  --depth-port PORT    Depth stream port (default: 5001)" << std::endl;
    std::cout << "  --webcam-port PORT   Webcam stream port (default: 5002)" << std::endl;
    std::cout << "  -h, --help           Show this help" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  q, ESC   Quit" << std::endl;
    std::cout << "  s        Save snapshot of all streams" << std::endl;
    std::cout << std::endl;
    std::cout << "Streams are broadcast from robot services:" << std::endl;
    std::cout << "  - depth-stream.service (port 5001)" << std::endl;
    std::cout << "  - webcam-stream.service (port 5002)" << std::endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse arguments
    bool enable_depth = true;
    bool enable_webcam = true;
    int depth_port = 5001;
    int webcam_port = 5002;

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
        }
    }

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
