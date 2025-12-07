// Depth streaming server - runs on robot
// Usage: ./depth_stream_server <target_ip> [port] [fps]
//
// Streams RealSense depth+color frames over UDP.
// Use broadcast address (e.g., 192.168.123.255) to allow any client to receive.

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#ifdef HAS_REALSENSE
#include "depth/DepthStreamServer.h"
#endif

static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

int main(int argc, char** argv) {
#ifndef HAS_REALSENSE
    std::cerr << "Error: Built without RealSense support" << std::endl;
    return 1;
#else
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Default to broadcast if no IP specified
    std::string client_ip = "192.168.123.255";

    if (argc >= 2 && std::string(argv[1]) != "--help" && std::string(argv[1]) != "-h") {
        client_ip = argv[1];
    } else if (argc >= 2) {
        std::cout << "Usage: " << argv[0] << " [target_ip] [port] [fps]" << std::endl;
        std::cout << std::endl;
        std::cout << "Streams RealSense depth+color frames over UDP." << std::endl;
        std::cout << std::endl;
        std::cout << "Arguments:" << std::endl;
        std::cout << "  target_ip  IP or broadcast address (default: 192.168.123.255)" << std::endl;
        std::cout << "  port       UDP port (default: 5001)" << std::endl;
        std::cout << "  fps        Target framerate (default: 15)" << std::endl;
        std::cout << std::endl;
        std::cout << "Examples:" << std::endl;
        std::cout << "  # Broadcast (default - any client can receive):" << std::endl;
        std::cout << "  " << argv[0] << std::endl;
        std::cout << std::endl;
        std::cout << "  # Unicast to specific client:" << std::endl;
        std::cout << "  " << argv[0] << " 192.168.123.222 5001 15" << std::endl;
        return 1;
    }
    int port = (argc > 2) ? std::stoi(argv[2]) : DepthStreamServer::DEFAULT_PORT;
    int fps = (argc > 3) ? std::stoi(argv[3]) : 15;

    std::cout << "========================================" << std::endl;
    std::cout << "  Depth Stream Server" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Target: " << client_ip << ":" << port << std::endl;
    std::cout << "FPS: " << fps << std::endl;
    std::cout << std::endl;

    DepthStreamServer server;
    server.setFramerate(fps);

    std::cout << "Initializing RealSense..." << std::endl;
    if (!server.init()) {
        std::cerr << "Failed to initialize" << std::endl;
        return 1;
    }

    std::cout << "Starting stream..." << std::endl;
    if (!server.start(client_ip, port)) {
        std::cerr << "Failed to start streaming" << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "Streaming depth data. Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << std::endl;
    std::cout << "Stopping..." << std::endl;
    server.stop();

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Done" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total frames: " << server.getFrameCount() << std::endl;
    std::cout << "Data sent: " << server.getBytesSent() / 1024 / 1024 << " MB" << std::endl;

    return 0;
#endif
}
