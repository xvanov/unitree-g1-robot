#include "webcam/WebcamStreamServer.h"
#include <iostream>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// ============================================================================
// WebcamStreamServer (runs on robot)
// ============================================================================

WebcamStreamServer::WebcamStreamServer() = default;

WebcamStreamServer::~WebcamStreamServer() {
    stop();
}

bool WebcamStreamServer::init(const std::string& device) {
    // Try to open device by path
    capture_.open(device, cv::CAP_V4L2);
    if (!capture_.isOpened()) {
        std::cerr << "[WEBCAM_SERVER] Failed to open device: " << device << std::endl;
        return false;
    }

    // Use MJPEG format first (required for higher resolutions on many cameras)
    capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // Set resolution and framerate
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture_.set(cv::CAP_PROP_FPS, fps_);

    initialized_.store(true);

    int actual_w = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_h = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = capture_.get(cv::CAP_PROP_FPS);

    std::cout << "[WEBCAM_SERVER] Initialized: " << device
              << " (" << actual_w << "x" << actual_h
              << " @ " << actual_fps << " fps)" << std::endl;
    return true;
}

bool WebcamStreamServer::init(int device_index) {
    capture_.open(device_index, cv::CAP_V4L2);
    if (!capture_.isOpened()) {
        std::cerr << "[WEBCAM_SERVER] Failed to open device index: " << device_index << std::endl;
        return false;
    }

    // Use MJPEG format first (required for higher resolutions on many cameras)
    capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // Set resolution and framerate
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture_.set(cv::CAP_PROP_FPS, fps_);

    initialized_.store(true);

    int actual_w = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_h = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = capture_.get(cv::CAP_PROP_FPS);

    std::cout << "[WEBCAM_SERVER] Initialized: index " << device_index
              << " (" << actual_w << "x" << actual_h
              << " @ " << actual_fps << " fps)" << std::endl;
    return true;
}

bool WebcamStreamServer::start(const std::string& client_ip, int port) {
    if (!initialized_.load()) {
        std::cerr << "[WEBCAM_SERVER] Not initialized" << std::endl;
        return false;
    }

    if (running_.load()) {
        return true;
    }

    client_ip_ = client_ip;
    client_port_ = port;

    // Create UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "[WEBCAM_SERVER] Failed to create socket" << std::endl;
        return false;
    }

    // Enable broadcast if using broadcast address
    if (client_ip == "255.255.255.255" || client_ip.find(".255") != std::string::npos) {
        int broadcast = 1;
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
            std::cerr << "[WEBCAM_SERVER] Warning: Failed to enable broadcast" << std::endl;
        } else {
            std::cout << "[WEBCAM_SERVER] Broadcast mode enabled" << std::endl;
        }
    }

    running_.store(true);
    stream_thread_ = std::thread(&WebcamStreamServer::streamLoop, this);

    if (client_ip == "255.255.255.255" || client_ip.find(".255") != std::string::npos) {
        std::cout << "[WEBCAM_SERVER] Broadcasting to port " << port << " (any client can receive)" << std::endl;
    } else {
        std::cout << "[WEBCAM_SERVER] Streaming to " << client_ip << ":" << port << std::endl;
    }
    return true;
}

void WebcamStreamServer::stop() {
    if (!running_.load()) return;

    running_.store(false);

    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    if (capture_.isOpened()) {
        capture_.release();
    }

    std::cout << "[WEBCAM_SERVER] Stopped (frames: " << frame_count_.load()
              << ", sent: " << bytes_sent_.load() / 1024 / 1024 << " MB)" << std::endl;
}

void WebcamStreamServer::streamLoop() {
    cv::Mat frame;
    int frames_since_fps = 0;
    auto last_fps_time = std::chrono::steady_clock::now();
    uint32_t frame_num = 0;

    // Target frame interval
    auto target_interval = std::chrono::milliseconds(1000 / fps_);
    auto last_send_time = std::chrono::steady_clock::now();

    while (running_.load()) {
        // Rate limiting
        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_send_time;
        if (elapsed < target_interval) {
            std::this_thread::sleep_for(target_interval - elapsed);
            continue;
        }
        last_send_time = std::chrono::steady_clock::now();

        // Capture frame
        if (!capture_.read(frame)) {
            std::cerr << "[WEBCAM_SERVER] Failed to capture frame" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (frame.empty()) {
            continue;
        }

        // Encode as JPEG
        std::vector<uint8_t> jpeg_data;
        std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        if (!cv::imencode(".jpg", frame, jpeg_data, jpeg_params)) {
            std::cerr << "[WEBCAM_SERVER] Failed to encode frame" << std::endl;
            continue;
        }

        // Send frame
        if (sendFrame(jpeg_data, frame_num)) {
            frame_count_.fetch_add(1);
            frame_num++;
            frames_since_fps++;
        }

        // Update FPS
        now = std::chrono::steady_clock::now();
        double fps_elapsed = std::chrono::duration<double>(now - last_fps_time).count();
        if (fps_elapsed >= 1.0) {
            current_fps_.store(static_cast<float>(frames_since_fps / fps_elapsed));
            frames_since_fps = 0;
            last_fps_time = now;

            std::cout << "[WEBCAM_SERVER] FPS: " << current_fps_.load()
                      << " | Size: " << jpeg_data.size() / 1024 << " KB"
                      << " | Total: " << bytes_sent_.load() / 1024 / 1024 << " MB"
                      << std::endl;
        }
    }
}

bool WebcamStreamServer::sendFrame(const std::vector<uint8_t>& data, uint32_t frame_num) {
    if (socket_fd_ < 0) return false;

    // Calculate number of chunks needed
    uint16_t chunk_count = (data.size() + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(client_port_);
    inet_pton(AF_INET, client_ip_.c_str(), &addr.sin_addr);

    // Send each chunk
    for (uint16_t i = 0; i < chunk_count; i++) {
        size_t offset = i * MAX_CHUNK_SIZE;
        size_t chunk_size = std::min<size_t>(MAX_CHUNK_SIZE, data.size() - offset);

        // Build packet: header + payload
        std::vector<uint8_t> packet;
        packet.reserve(16 + chunk_size);

        // Header: magic(4) + frame_num(4) + chunk_idx(2) + chunk_count(2) + chunk_size(4)
        uint32_t magic = MAGIC;
        packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&magic),
                      reinterpret_cast<uint8_t*>(&magic) + 4);
        packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&frame_num),
                      reinterpret_cast<uint8_t*>(&frame_num) + 4);
        packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&i),
                      reinterpret_cast<uint8_t*>(&i) + 2);
        packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&chunk_count),
                      reinterpret_cast<uint8_t*>(&chunk_count) + 2);
        uint32_t cs = static_cast<uint32_t>(chunk_size);
        packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&cs),
                      reinterpret_cast<uint8_t*>(&cs) + 4);

        // Payload
        packet.insert(packet.end(), data.begin() + offset, data.begin() + offset + chunk_size);

        // Send
        ssize_t sent = sendto(socket_fd_, packet.data(), packet.size(), 0,
                              (struct sockaddr*)&addr, sizeof(addr));
        if (sent < 0) {
            std::cerr << "[WEBCAM_SERVER] Send error" << std::endl;
            return false;
        }

        bytes_sent_.fetch_add(sent);
    }

    return true;
}

void WebcamStreamServer::setResolution(int width, int height) {
    if (!running_.load()) {
        width_ = width;
        height_ = height;
    }
}

void WebcamStreamServer::setFramerate(int fps) {
    if (!running_.load()) {
        fps_ = fps;
    }
}


// ============================================================================
// WebcamStreamClient (runs on dev machine)
// ============================================================================

WebcamStreamClient::WebcamStreamClient() = default;

WebcamStreamClient::~WebcamStreamClient() {
    stop();
}

bool WebcamStreamClient::start(int port) {
    if (running_.load()) return true;

    // Create UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "[WEBCAM_CLIENT] Failed to create socket" << std::endl;
        return false;
    }

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Increase receive buffer
    int rcvbuf = 4 * 1024 * 1024;  // 4 MB
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    // Bind to port
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[WEBCAM_CLIENT] Failed to bind to port " << port << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    running_.store(true);
    receive_thread_ = std::thread(&WebcamStreamClient::receiveLoop, this);

    std::cout << "[WEBCAM_CLIENT] Listening on port " << port << std::endl;
    return true;
}

void WebcamStreamClient::stop() {
    if (!running_.load()) return;

    running_.store(false);

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    std::cout << "[WEBCAM_CLIENT] Stopped (frames: " << frame_count_.load() << ")" << std::endl;
}

void WebcamStreamClient::receiveLoop() {
    std::vector<uint8_t> recv_buf(WebcamStreamServer::MAX_CHUNK_SIZE + 16);

    int frames_since_fps = 0;
    auto last_fps_time = std::chrono::steady_clock::now();

    while (running_.load()) {
        ssize_t len = recv(socket_fd_, recv_buf.data(), recv_buf.size(), 0);
        if (len < 16) continue;  // Too short or timeout

        // Parse header
        uint32_t magic = *reinterpret_cast<uint32_t*>(recv_buf.data());
        if (magic != WebcamStreamServer::MAGIC) continue;

        uint32_t frame_num = *reinterpret_cast<uint32_t*>(recv_buf.data() + 4);
        uint16_t chunk_idx = *reinterpret_cast<uint16_t*>(recv_buf.data() + 8);
        uint16_t chunk_count = *reinterpret_cast<uint16_t*>(recv_buf.data() + 10);
        uint32_t chunk_size = *reinterpret_cast<uint32_t*>(recv_buf.data() + 12);

        if (chunk_size > len - 16) continue;  // Invalid

        bytes_received_.fetch_add(len);

        // Handle new frame
        {
            std::lock_guard<std::mutex> lock(chunks_mutex_);

            if (frame_num != current_frame_num_) {
                // New frame, reset state
                current_frame_num_ = frame_num;
                expected_chunks_ = chunk_count;
                chunks_.clear();
                chunks_.resize(chunk_count);
                chunks_received_.clear();
                chunks_received_.resize(chunk_count, false);
            }

            // Store chunk
            if (chunk_idx < chunks_.size() && !chunks_received_[chunk_idx]) {
                chunks_[chunk_idx].assign(recv_buf.begin() + 16,
                                          recv_buf.begin() + 16 + chunk_size);
                chunks_received_[chunk_idx] = true;
            }

            // Check if frame complete
            bool complete = true;
            for (bool received : chunks_received_) {
                if (!received) { complete = false; break; }
            }

            if (complete && !chunks_.empty()) {
                // Reassemble frame
                std::vector<uint8_t> frame_data;
                for (const auto& chunk : chunks_) {
                    frame_data.insert(frame_data.end(), chunk.begin(), chunk.end());
                }

                // Decode JPEG
                cv::Mat image = cv::imdecode(frame_data, cv::IMREAD_COLOR);

                if (!image.empty()) {
                    Frame f;
                    f.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    f.frame_number = frame_num;
                    f.image = image;
                    f.valid = true;

                    // Update latest frame
                    {
                        std::lock_guard<std::mutex> flock(frame_mutex_);
                        latest_frame_ = f;
                    }

                    // Call callback
                    if (frame_callback_) {
                        frame_callback_(f);
                    }

                    frame_count_.fetch_add(1);
                    frames_since_fps++;
                }

                // Clear for next frame
                chunks_.clear();
                chunks_received_.clear();
            }
        }

        // Update FPS
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
        if (elapsed >= 1.0) {
            current_fps_.store(static_cast<float>(frames_since_fps / elapsed));
            frames_since_fps = 0;
            last_fps_time = now;
        }
    }
}

WebcamStreamClient::Frame WebcamStreamClient::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_;
}
