#include "depth/DepthStreamer.h"
#include <iostream>
#include <chrono>

#ifdef HAS_UNITREE_SDK2
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#endif

// Define a simple depth message structure for DDS
// This matches what we'll subscribe to on the dev machine
namespace unitree_depth {
namespace msg {
namespace dds_ {

struct DepthFrame_ {
    uint64_t timestamp_us = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t frame_number = 0;
    float fx = 0, fy = 0, cx = 0, cy = 0;  // Camera intrinsics
    float depth_scale = 0.001f;            // Depth units (meters per unit)
    std::vector<uint8_t> color_jpeg;       // JPEG-encoded color image
    std::vector<uint8_t> depth_png;        // PNG-encoded 16-bit depth
};

}}}

DepthStreamer::DepthStreamer() = default;

DepthStreamer::~DepthStreamer() {
    stop();
}

bool DepthStreamer::init(const std::string& network_interface) {
#ifndef HAS_REALSENSE
    std::cerr << "[DEPTH_STREAM] RealSense not available" << std::endl;
    return false;
#else
    network_interface_ = network_interface;

    // Initialize RealSense
    depth_capture_ = std::make_unique<DepthCapture>();
    depth_capture_->setResolution(width_, height_);
    depth_capture_->setFramerate(fps_);

    if (!depth_capture_->init()) {
        std::cerr << "[DEPTH_STREAM] Failed to initialize RealSense" << std::endl;
        return false;
    }

#ifdef HAS_UNITREE_SDK2
    try {
        // Initialize DDS
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
        std::cout << "[DEPTH_STREAM] DDS initialized on " << network_interface << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[DEPTH_STREAM] Failed to initialize DDS: " << e.what() << std::endl;
        return false;
    }
#else
    std::cerr << "[DEPTH_STREAM] unitree_sdk2 not available - DDS disabled" << std::endl;
    return false;
#endif

    initialized_.store(true);
    std::cout << "[DEPTH_STREAM] Initialized (" << width_ << "x" << height_
              << " @ " << fps_ << " fps)" << std::endl;
    return true;
#endif
}

bool DepthStreamer::start() {
#ifndef HAS_REALSENSE
    return false;
#else
    if (!initialized_.load()) {
        std::cerr << "[DEPTH_STREAM] Not initialized" << std::endl;
        return false;
    }

    if (running_.load()) {
        return true;  // Already running
    }

    // Start RealSense capture
    if (!depth_capture_->startCapture()) {
        std::cerr << "[DEPTH_STREAM] Failed to start capture" << std::endl;
        return false;
    }

    running_.store(true);
    stream_thread_ = std::thread(&DepthStreamer::streamLoop, this);

    std::cout << "[DEPTH_STREAM] Streaming started" << std::endl;
    return true;
#endif
}

void DepthStreamer::stop() {
    if (!running_.load()) return;

    running_.store(false);

    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }

#ifdef HAS_REALSENSE
    if (depth_capture_) {
        depth_capture_->stopCapture();
    }
#endif

    std::cout << "[DEPTH_STREAM] Stopped (frames: " << frame_count_.load() << ")" << std::endl;
}

void DepthStreamer::streamLoop() {
#if defined(HAS_REALSENSE) && defined(HAS_UNITREE_SDK2)
    // Create DDS publisher for depth topic
    // Note: This uses a raw topic since there's no predefined depth IDL
    // The subscriber will need to decode the same format

    auto start_time = std::chrono::steady_clock::now();
    int frames_since_fps = 0;
    auto last_fps_time = start_time;
    uint64_t last_frame_number = 0;

    std::cout << "[DEPTH_STREAM] Publishing depth on topic 'rt/depth_frame'" << std::endl;

    while (running_.load()) {
        // Get latest depth frame
        DepthFrame frame = depth_capture_->getLatestFrame();

        // Skip if no new frame
        if (frame.frame_number == last_frame_number) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        last_frame_number = frame.frame_number;

        if (frame.depth.empty() || frame.color.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Encode color as JPEG
        std::vector<uint8_t> color_jpeg;
        std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 85};
        cv::imencode(".jpg", frame.color, color_jpeg, jpeg_params);

        // Encode depth as PNG (lossless 16-bit)
        std::vector<uint8_t> depth_png;
        std::vector<int> png_params = {cv::IMWRITE_PNG_COMPRESSION, 3};  // Fast compression
        cv::imencode(".png", frame.depth, depth_png, png_params);

        // Build message
        unitree_depth::msg::dds_::DepthFrame_ msg;
        msg.timestamp_us = static_cast<uint64_t>(frame.timestamp * 1e6);
        msg.width = frame.depth.cols;
        msg.height = frame.depth.rows;
        msg.frame_number = frame.frame_number;
        msg.fx = frame.fx;
        msg.fy = frame.fy;
        msg.cx = frame.cx;
        msg.cy = frame.cy;
        msg.depth_scale = frame.depth_scale;
        msg.color_jpeg = std::move(color_jpeg);
        msg.depth_png = std::move(depth_png);

        // TODO: Publish via DDS
        // For now, we'll just update stats - need to implement custom publisher
        // unitree_robot::ChannelPublisher doesn't support custom types easily

        frame_count_.fetch_add(1);
        frames_since_fps++;

        // Update FPS
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
        if (elapsed >= 1.0) {
            current_fps_.store(static_cast<float>(frames_since_fps / elapsed));
            frames_since_fps = 0;
            last_fps_time = now;

            // Log stats
            std::cout << "[DEPTH_STREAM] Frames: " << frame_count_.load()
                      << " | FPS: " << current_fps_.load()
                      << " | Color: " << msg.color_jpeg.size() / 1024 << " KB"
                      << " | Depth: " << msg.depth_png.size() / 1024 << " KB"
                      << std::endl;
        }
    }
#endif
}

void DepthStreamer::setResolution(int width, int height) {
    if (!running_.load()) {
        width_ = width;
        height_ = height;
    }
}

void DepthStreamer::setFramerate(int fps) {
    if (!running_.load()) {
        fps_ = fps;
    }
}
