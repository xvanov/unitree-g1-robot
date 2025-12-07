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
 * UDP-based webcam streaming server for regular USB cameras.
 *
 * Runs on the robot and streams webcam frames over UDP to the dev machine.
 * Uses a simple protocol:
 *   - Header: magic (4) + frame_num (4) + chunk_idx (2) + chunk_count (2) + chunk_size (4)
 *   - Payload: JPEG-compressed frame data (chunked for UDP)
 */
class WebcamStreamServer {
public:
    static constexpr uint32_t MAGIC = 0x57454243;  // "WEBC"
    static constexpr int DEFAULT_PORT = 5002;
    static constexpr int MAX_CHUNK_SIZE = 60000;   // UDP-safe size

    WebcamStreamServer();
    ~WebcamStreamServer();

    // Initialize webcam by device path or index
    bool init(const std::string& device = "/dev/video0");
    bool init(int device_index);

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
    bool sendFrame(const std::vector<uint8_t>& data, uint32_t frame_num);

    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::thread stream_thread_;

    cv::VideoCapture capture_;

    // Network
    int socket_fd_ = -1;
    std::string client_ip_;
    int client_port_ = DEFAULT_PORT;

    // Configuration
    int width_ = 640;
    int height_ = 480;
    int fps_ = 30;
    int jpeg_quality_ = 80;

    // Statistics
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<float> current_fps_{0.0f};
    std::atomic<uint64_t> bytes_sent_{0};
};


/**
 * UDP-based webcam stream client.
 *
 * Runs on the dev machine to receive webcam frames from the robot.
 */
class WebcamStreamClient {
public:
    // Received webcam frame
    struct Frame {
        uint64_t timestamp_us = 0;
        uint32_t frame_number = 0;
        cv::Mat image;      // Decoded BGR image
        bool valid = false;
    };

    using FrameCallback = std::function<void(const Frame&)>;

    WebcamStreamClient();
    ~WebcamStreamClient();

    // Start listening on port
    bool start(int port = WebcamStreamServer::DEFAULT_PORT);

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
