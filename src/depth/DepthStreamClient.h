#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <cstdint>
#include <functional>
#include <opencv2/opencv.hpp>

/**
 * UDP-based depth stream client.
 *
 * Runs on the dev machine to receive depth frames from the robot.
 * Receives frames sent by DepthStreamServer.
 *
 * Protocol:
 *   - Header: magic (4) + frame_num (4) + chunk_idx (2) + chunk_count (2) + chunk_size (4)
 *   - Payload: msgpack-encoded frame data (chunked for UDP)
 *
 * Frame format (msgpack):
 *   - ts: timestamp_us (uint64)
 *   - w, h: width, height (uint32)
 *   - n: frame_number (uint32)
 *   - fx, fy, cx, cy: camera intrinsics (float)
 *   - s: depth_scale (float)
 *   - c: color_jpeg (bytes)
 *   - d: depth_png (bytes)
 */
class DepthStreamClient {
public:
    static constexpr uint32_t MAGIC = 0x44455054;  // "DEPT"
    static constexpr int DEFAULT_PORT = 5001;
    static constexpr int MAX_CHUNK_SIZE = 60000;   // UDP-safe size

    // Received depth frame
    struct Frame {
        uint64_t timestamp_us = 0;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t frame_number = 0;
        float fx = 0, fy = 0, cx = 0, cy = 0;
        float depth_scale = 0.001f;
        cv::Mat color;      // Decoded BGR image
        cv::Mat depth;      // Decoded 16-bit depth (mm)
        bool valid = false;
    };

    using FrameCallback = std::function<void(const Frame&)>;

    DepthStreamClient();
    ~DepthStreamClient();

    // Start listening on port
    bool start(int port = DEFAULT_PORT);

    // Stop receiving
    void stop();

    // Check if running
    bool isRunning() const { return running_.load(); }

    // Get latest frame (thread-safe copy)
    Frame getLatestFrame();

    // Set callback for new frames
    void setFrameCallback(FrameCallback callback) { frame_callback_ = callback; }

    // Statistics
    uint64_t getFrameCount() const { return frame_count_.load(); }
    float getFps() const { return current_fps_.load(); }
    uint64_t getBytesReceived() const { return bytes_received_.load(); }

private:
    void receiveLoop();

    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    int socket_fd_ = -1;

    // Frame reassembly
    std::mutex chunks_mutex_;
    uint32_t current_frame_num_ = 0;
    uint16_t expected_chunks_ = 0;
    std::vector<std::vector<uint8_t>> chunks_;
    std::vector<bool> chunks_received_;

    // Latest frame
    std::mutex frame_mutex_;
    Frame latest_frame_;

    // Callback
    FrameCallback frame_callback_;

    // Statistics
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<float> current_fps_{0.0f};
    std::atomic<uint64_t> bytes_received_{0};
};
