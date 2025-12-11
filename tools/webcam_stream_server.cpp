// Webcam streaming server - runs on robot
// Usage: ./webcam_stream_server [device] [target_ip] [port] [fps]
//
// Streams USB webcam frames over UDP.
// Use broadcast address (e.g., 192.168.123.255) to allow any client to receive.

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include "webcam/WebcamStreamServer.h"

static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

void printHelp(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [device] [target_ip] [port] [fps]" << std::endl;
    std::cout << std::endl;
    std::cout << "Streams USB webcam frames over UDP." << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  device     Video device (default: /dev/video0)" << std::endl;
    std::cout << "  target_ip  IP or broadcast address (default: 192.168.123.255)" << std::endl;
    std::cout << "  port       UDP port (default: 5002)" << std::endl;
    std::cout << "  fps        Target framerate (default: 30)" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  # Default (broadcast /dev/video0):" << std::endl;
    std::cout << "  " << argv0 << std::endl;
    std::cout << std::endl;
    std::cout << "  # Specific device:" << std::endl;
    std::cout << "  " << argv0 << " /dev/video2" << std::endl;
    std::cout << std::endl;
    std::cout << "  # Unicast to specific client:" << std::endl;
    std::cout << "  " << argv0 << " /dev/video0 192.168.123.222 5002 30" << std::endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse arguments
    std::string device = "/dev/video0";
    std::string client_ip = "192.168.123.255";
    int port = WebcamStreamServer::DEFAULT_PORT;
    int fps = 30;

    if (argc >= 2) {
        std::string arg1(argv[1]);
        if (arg1 == "--help" || arg1 == "-h") {
            printHelp(argv[0]);
            return 0;
        }
        device = arg1;
    }
    if (argc >= 3) {
        client_ip = argv[2];
    }
    if (argc >= 4) {
        port = std::stoi(argv[3]);
    }
    if (argc >= 5) {
        fps = std::stoi(argv[4]);
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  Webcam Stream Server" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Device: " << device << std::endl;
    std::cout << "Target: " << client_ip << ":" << port << std::endl;
    std::cout << "FPS: " << fps << std::endl;
    std::cout << std::endl;

    WebcamStreamServer server;
    server.setFramerate(fps);

    std::cout << "Initializing webcam..." << std::endl;
    if (!server.init(device)) {
        std::cerr << "Failed to initialize webcam" << std::endl;
        return 1;
    }

    std::cout << "Starting stream..." << std::endl;
    if (!server.start(client_ip, port)) {
        std::cerr << "Failed to start streaming" << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "Streaming webcam. Press Ctrl+C to stop." << std::endl;
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
}
