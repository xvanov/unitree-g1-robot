// Test depth stream client - receives and decodes frames
#include "depth/DepthStreamClient.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

std::atomic<bool> running{true};

void sigHandler(int) {
    running.store(false);
}

int main(int argc, char** argv) {
    int port = 5001;
    int duration_sec = 15;

    if (argc > 1) port = std::stoi(argv[1]);
    if (argc > 2) duration_sec = std::stoi(argv[2]);

    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    std::cout << "========================================" << std::endl;
    std::cout << "  Depth Stream Client Test" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Listening on port: " << port << std::endl;
    std::cout << "Duration: " << duration_sec << " seconds" << std::endl;
    std::cout << std::endl;

    DepthStreamClient client;

    // Set callback for new frames
    client.setFrameCallback([](const DepthStreamClient::Frame& frame) {
        std::cout << "[FRAME] #" << frame.frame_number
                  << " (" << frame.width << "x" << frame.height << ")"
                  << " color: " << frame.color.cols << "x" << frame.color.rows
                  << " depth: " << frame.depth.cols << "x" << frame.depth.rows
                  << " fx=" << frame.fx << " scale=" << frame.depth_scale
                  << std::endl;
    });

    if (!client.start(port)) {
        std::cerr << "Failed to start client" << std::endl;
        return 1;
    }

    auto start = std::chrono::steady_clock::now();
    auto last_status = start;

    while (running.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        if (elapsed >= duration_sec) {
            break;
        }

        // Print status every second
        auto status_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_status).count();
        if (status_elapsed >= 1) {
            auto frame = client.getLatestFrame();
            std::cout << "[STATUS] Frames: " << client.getFrameCount()
                      << " | FPS: " << client.getFps()
                      << " | Bytes: " << client.getBytesReceived() / 1024 << " KB"
                      << " | Latest valid: " << (frame.valid ? "yes" : "no")
                      << std::endl;
            last_status = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    client.stop();

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Done" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total frames: " << client.getFrameCount() << std::endl;
    std::cout << "Total bytes: " << client.getBytesReceived() / 1024 << " KB" << std::endl;

    return 0;
}
