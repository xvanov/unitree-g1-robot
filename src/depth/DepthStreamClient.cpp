#include "depth/DepthStreamClient.h"
#include <iostream>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <msgpack.hpp>

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
    std::vector<uint8_t> recv_buf(MAX_CHUNK_SIZE + 16);

    auto start_time = std::chrono::steady_clock::now();
    int frames_since_fps = 0;
    auto last_fps_time = start_time;
    int recv_count = 0;
    int timeout_count = 0;

    std::cout << "[DEPTH_CLIENT] Receive loop started, waiting for UDP packets..." << std::endl;

    while (running_.load()) {
        ssize_t len = recv(socket_fd_, recv_buf.data(), recv_buf.size(), 0);
        if (len < 0) {
            timeout_count++;
            if (timeout_count % 5 == 1) {  // Every 5 timeouts (~5 sec)
                std::cout << "[DEPTH_CLIENT] Waiting for packets... (timeouts: " << timeout_count << ")" << std::endl;
            }
            continue;
        }
        if (len < 16) {
            std::cout << "[DEPTH_CLIENT] Short packet: " << len << " bytes" << std::endl;
            continue;  // Too short
        }

        recv_count++;

        // Parse header
        uint32_t magic = *reinterpret_cast<uint32_t*>(recv_buf.data());
        if (magic != MAGIC) {
            continue;
        }

        uint32_t frame_num = *reinterpret_cast<uint32_t*>(recv_buf.data() + 4);
        uint16_t chunk_idx = *reinterpret_cast<uint16_t*>(recv_buf.data() + 8);
        uint16_t chunk_count = *reinterpret_cast<uint16_t*>(recv_buf.data() + 10);
        uint32_t chunk_size = *reinterpret_cast<uint32_t*>(recv_buf.data() + 12);

        if (chunk_size > static_cast<uint32_t>(len - 16)) continue;  // Invalid

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
                                // Handle BIN type for color data
                                if (kv.val.type == msgpack::type::BIN) {
                                    color_data.assign(
                                        kv.val.via.bin.ptr,
                                        kv.val.via.bin.ptr + kv.val.via.bin.size);
                                } else if (kv.val.type == msgpack::type::STR) {
                                    auto str = kv.val.as<msgpack::type::raw_ref>();
                                    color_data.assign(str.ptr, str.ptr + str.size);
                                }
                            }
                            else if (key == "d") {
                                // Handle BIN type for depth data
                                if (kv.val.type == msgpack::type::BIN) {
                                    depth_data.assign(
                                        kv.val.via.bin.ptr,
                                        kv.val.via.bin.ptr + kv.val.via.bin.size);
                                } else if (kv.val.type == msgpack::type::STR) {
                                    auto str = kv.val.as<msgpack::type::raw_ref>();
                                    depth_data.assign(str.ptr, str.ptr + str.size);
                                }
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
