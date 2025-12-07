#include "depth/DepthStreamServer.h"
#include <iostream>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <msgpack.hpp>
#include <opencv2/opencv.hpp>

// ============================================================================
// DepthStreamServer (runs on robot)
// ============================================================================

DepthStreamServer::DepthStreamServer() = default;

DepthStreamServer::~DepthStreamServer() {
    stop();
}

bool DepthStreamServer::init() {
#ifndef HAS_REALSENSE
    std::cerr << "[DEPTH_SERVER] RealSense not available" << std::endl;
    return false;
#else
    depth_capture_ = std::make_unique<DepthCapture>();
    depth_capture_->setResolution(width_, height_);
    depth_capture_->setFramerate(fps_);

    if (!depth_capture_->init()) {
        std::cerr << "[DEPTH_SERVER] Failed to initialize RealSense" << std::endl;
        return false;
    }

    initialized_.store(true);
    std::cout << "[DEPTH_SERVER] Initialized (" << width_ << "x" << height_
              << " @ " << fps_ << " fps)" << std::endl;
    return true;
#endif
}

bool DepthStreamServer::start(const std::string& client_ip, int port) {
#ifndef HAS_REALSENSE
    return false;
#else
    if (!initialized_.load()) {
        std::cerr << "[DEPTH_SERVER] Not initialized" << std::endl;
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
        std::cerr << "[DEPTH_SERVER] Failed to create socket" << std::endl;
        return false;
    }

    // Start RealSense
    if (!depth_capture_->startCapture()) {
        std::cerr << "[DEPTH_SERVER] Failed to start capture" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    running_.store(true);
    stream_thread_ = std::thread(&DepthStreamServer::streamLoop, this);

    std::cout << "[DEPTH_SERVER] Streaming to " << client_ip << ":" << port << std::endl;
    return true;
#endif
}

void DepthStreamServer::stop() {
    if (!running_.load()) return;

    running_.store(false);

    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

#ifdef HAS_REALSENSE
    if (depth_capture_) {
        depth_capture_->stopCapture();
    }
#endif

    std::cout << "[DEPTH_SERVER] Stopped (frames: " << frame_count_.load()
              << ", sent: " << bytes_sent_.load() / 1024 / 1024 << " MB)" << std::endl;
}

void DepthStreamServer::streamLoop() {
#ifdef HAS_REALSENSE
    auto start_time = std::chrono::steady_clock::now();
    int frames_since_fps = 0;
    auto last_fps_time = start_time;
    uint64_t last_frame_number = 0;

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

        // Get latest frame
        DepthFrame frame = depth_capture_->getLatestFrame();

        if (frame.depth.empty() || frame.color.empty()) {
            continue;
        }

        // Skip duplicate frames
        if (frame.frame_number == last_frame_number) {
            continue;
        }
        last_frame_number = frame.frame_number;

        // Encode color as JPEG
        std::vector<uint8_t> color_jpeg;
        std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        cv::imencode(".jpg", frame.color, color_jpeg, jpeg_params);

        // Encode depth as PNG (lossless)
        std::vector<uint8_t> depth_png;
        std::vector<int> png_params = {cv::IMWRITE_PNG_COMPRESSION, 1};  // Fast
        cv::imencode(".png", frame.depth, depth_png, png_params);

        // Pack frame data with msgpack
        msgpack::sbuffer sbuf;
        msgpack::packer<msgpack::sbuffer> pk(&sbuf);
        pk.pack_map(10);
        pk.pack("ts"); pk.pack(static_cast<uint64_t>(frame.timestamp * 1e6));
        pk.pack("w"); pk.pack(static_cast<uint32_t>(frame.depth.cols));
        pk.pack("h"); pk.pack(static_cast<uint32_t>(frame.depth.rows));
        pk.pack("n"); pk.pack(static_cast<uint32_t>(frame.frame_number));
        pk.pack("fx"); pk.pack(frame.fx);
        pk.pack("fy"); pk.pack(frame.fy);
        pk.pack("cx"); pk.pack(frame.cx);
        pk.pack("cy"); pk.pack(frame.cy);
        pk.pack("s"); pk.pack(frame.depth_scale);
        pk.pack("c"); pk.pack_bin(color_jpeg.size());
        pk.pack_bin_body(reinterpret_cast<const char*>(color_jpeg.data()), color_jpeg.size());
        pk.pack("d"); pk.pack_bin(depth_png.size());
        pk.pack_bin_body(reinterpret_cast<const char*>(depth_png.data()), depth_png.size());

        // Send frame
        std::vector<uint8_t> data(sbuf.data(), sbuf.data() + sbuf.size());
        if (sendFrame(data)) {
            frame_count_.fetch_add(1);
            frames_since_fps++;
        }

        // Update FPS
        now = std::chrono::steady_clock::now();
        double fps_elapsed = std::chrono::duration<double>(now - last_fps_time).count();
        if (fps_elapsed >= 1.0) {
            current_fps_.store(static_cast<float>(frames_since_fps / fps_elapsed));
            frames_since_fps = 0;
            last_fps_time = now;

            std::cout << "[DEPTH_SERVER] FPS: " << current_fps_.load()
                      << " | Color: " << color_jpeg.size() / 1024 << " KB"
                      << " | Depth: " << depth_png.size() / 1024 << " KB"
                      << " | Total: " << bytes_sent_.load() / 1024 / 1024 << " MB"
                      << std::endl;
        }
    }
#endif
}

bool DepthStreamServer::sendFrame(const std::vector<uint8_t>& data) {
    if (socket_fd_ < 0) return false;

    // Calculate number of chunks needed
    uint16_t chunk_count = (data.size() + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;
    uint32_t frame_num = frame_count_.load();

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
            std::cerr << "[DEPTH_SERVER] Send error" << std::endl;
            return false;
        }

        bytes_sent_.fetch_add(sent);
    }

    return true;
}

void DepthStreamServer::setResolution(int width, int height) {
    if (!running_.load()) {
        width_ = width;
        height_ = height;
    }
}

void DepthStreamServer::setFramerate(int fps) {
    if (!running_.load()) {
        fps_ = fps;
    }
}


// ============================================================================
// DepthStreamClient (runs on dev machine)
// ============================================================================

DepthStreamClient::DepthStreamClient() = default;

DepthStreamClient::~DepthStreamClient() {
    stop();
}

bool DepthStreamClient::start(int port) {
    if (running_.load()) return true;

    // Create UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "[DEPTH_CLIENT] Failed to create socket" << std::endl;
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
        std::cerr << "[DEPTH_CLIENT] Failed to bind to port " << port << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    running_.store(true);
    receive_thread_ = std::thread(&DepthStreamClient::receiveLoop, this);

    std::cout << "[DEPTH_CLIENT] Listening on port " << port << std::endl;
    return true;
}

void DepthStreamClient::stop() {
    if (!running_.load()) return;

    running_.store(false);

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    std::cout << "[DEPTH_CLIENT] Stopped (frames: " << frame_count_.load() << ")" << std::endl;
}

void DepthStreamClient::receiveLoop() {
    std::vector<uint8_t> recv_buf(DepthStreamServer::MAX_CHUNK_SIZE + 16);

    auto start_time = std::chrono::steady_clock::now();
    int frames_since_fps = 0;
    auto last_fps_time = start_time;

    while (running_.load()) {
        ssize_t len = recv(socket_fd_, recv_buf.data(), recv_buf.size(), 0);
        if (len < 16) continue;  // Too short or timeout

        // Parse header
        uint32_t magic = *reinterpret_cast<uint32_t*>(recv_buf.data());
        if (magic != DepthStreamServer::MAGIC) continue;

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

                // Parse with msgpack
                try {
                    msgpack::object_handle oh = msgpack::unpack(
                        reinterpret_cast<const char*>(frame_data.data()), frame_data.size());
                    msgpack::object obj = oh.get();

                    if (obj.type == msgpack::type::MAP) {
                        Frame f;
                        std::vector<uint8_t> color_data, depth_data;

                        for (size_t i = 0; i < obj.via.map.size; i++) {
                            auto& kv = obj.via.map.ptr[i];
                            std::string key = kv.key.as<std::string>();

                            if (key == "ts") f.timestamp_us = kv.val.as<uint64_t>();
                            else if (key == "w") f.width = kv.val.as<uint32_t>();
                            else if (key == "h") f.height = kv.val.as<uint32_t>();
                            else if (key == "n") f.frame_number = kv.val.as<uint32_t>();
                            else if (key == "fx") f.fx = kv.val.as<float>();
                            else if (key == "fy") f.fy = kv.val.as<float>();
                            else if (key == "cx") f.cx = kv.val.as<float>();
                            else if (key == "cy") f.cy = kv.val.as<float>();
                            else if (key == "s") f.depth_scale = kv.val.as<float>();
                            else if (key == "c") {
                                auto bin = kv.val.as<msgpack::type::raw_ref>();
                                color_data.assign(bin.ptr, bin.ptr + bin.size);
                            }
                            else if (key == "d") {
                                auto bin = kv.val.as<msgpack::type::raw_ref>();
                                depth_data.assign(bin.ptr, bin.ptr + bin.size);
                            }
                        }

                        // Decode images
                        if (!color_data.empty()) {
                            f.color = cv::imdecode(color_data, cv::IMREAD_COLOR);
                        }
                        if (!depth_data.empty()) {
                            f.depth = cv::imdecode(depth_data, cv::IMREAD_UNCHANGED);
                        }

                        f.valid = !f.color.empty() && !f.depth.empty();

                        if (f.valid) {
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
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[DEPTH_CLIENT] Parse error: " << e.what() << std::endl;
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

DepthStreamClient::Frame DepthStreamClient::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_;
}
