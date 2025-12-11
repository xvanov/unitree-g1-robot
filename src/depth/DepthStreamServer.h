#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <cstdint>

#ifdef HAS_REALSENSE
#include "capture/DepthCapture.h"
#endif

/**
 * UDP-based depth streaming server for RealSense cameras.
 *
 * Runs on the robot and streams depth frames over UDP to the dev machine.
 * Uses a simple protocol:
 *   - Header: magic (4) + frame_num (4) + chunk_idx (2) + chunk_count (2) + chunk_size (4)
 *   - Payload: compressed frame data (chunked for UDP)
 *
 * Frame format (msgpack):
 *   - timestamp_us: uint64
 *   - width, height: uint32
 *   - frame_number: uint32
 *   - fx, fy, cx, cy, depth_scale: float
 *   - color_jpeg: bytes
 *   - depth_png: bytes
 */
class DepthStreamServer {
public:
    static constexpr uint32_t MAGIC = 0x44455054;  // "DEPT"
    static constexpr int DEFAULT_PORT = 5001;
    static constexpr int MAX_CHUNK_SIZE = 60000;   // UDP-safe size

    DepthStreamServer();
    ~DepthStreamServer();

    // Initialize RealSense camera
    bool init();

    // Start streaming to specified client
    bool start(const std::string& client_ip, int port = DEFAULT_PORT);

    // Stop streaming
    void stop();

    // Check if running
    bool isRunning() const { return running_.load(); }

    // Statistics
    uint64_t getFrameCount() const { return frame_count_.load(); }
    float getFps() const { return current_fps_.load(); }
    uint64_t getBytesSent() const { return bytes_sent_.load(); }

    // Configuration
    void setResolution(int width, int height);
    void setFramerate(int fps);
    void setJpegQuality(int quality) { jpeg_quality_ = quality; }

private:
    void streamLoop();
    bool sendFrame(const std::vector<uint8_t>& data);

    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::thread stream_thread_;

#ifdef HAS_REALSENSE
    std::unique_ptr<DepthCapture> depth_capture_;
#endif

    // Network
    int socket_fd_ = -1;
    std::string client_ip_;
    int client_port_ = DEFAULT_PORT;

    // Configuration
    int width_ = 640;
    int height_ = 480;
    int fps_ = 15;  // Lower FPS for bandwidth
    int jpeg_quality_ = 80;

    // Statistics
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<float> current_fps_{0.0f};
    std::atomic<uint64_t> bytes_sent_{0};
};


/**
 * UDP-based depth stream client.
 *
 * Runs on the dev machine to receive depth frames from the robot.
 */
class DepthStreamClient {
public:
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
    bool start(int port = DepthStreamServer::DEFAULT_PORT);

    // Stop receiving
    void stop();

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
    bool reassembleFrame(std::vector<uint8_t>& out_data);

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
