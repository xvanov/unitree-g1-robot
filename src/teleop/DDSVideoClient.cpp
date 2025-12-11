#include "teleop/DDSVideoClient.h"
#include <iostream>
#include <chrono>

#ifdef HAS_UNITREE_SDK2
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>
#endif

DDSVideoClient::DDSVideoClient() = default;

DDSVideoClient::~DDSVideoClient() {
    stop();
}

bool DDSVideoClient::init(const std::string& network_interface) {
#ifndef HAS_UNITREE_SDK2
    std::cerr << "[DDS_VIDEO] unitree_sdk2 not available" << std::endl;
    return false;
#else
    network_interface_ = network_interface;

    try {
        // Initialize DDS channel factory (only once per process)
        // ChannelFactory is a singleton, so this is safe to call multiple times
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
        std::cout << "[DDS_VIDEO] DDS initialized on interface: " << network_interface << std::endl;
        initialized_.store(true);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[DDS_VIDEO] Failed to initialize DDS: " << e.what() << std::endl;
        return false;
    }
#endif
}

void DDSVideoClient::start() {
    if (!initialized_.load()) {
        std::cerr << "[DDS_VIDEO] Not initialized, call init() first" << std::endl;
        return;
    }

    if (running_.load()) {
        std::cerr << "[DDS_VIDEO] Already running" << std::endl;
        return;
    }

    running_.store(true);
    fetch_thread_ = std::thread(&DDSVideoClient::fetchLoop, this);
    std::cout << "[DDS_VIDEO] Video fetch thread started" << std::endl;
}

void DDSVideoClient::stop() {
    if (!running_.load()) return;

    running_.store(false);
    if (fetch_thread_.joinable()) {
        fetch_thread_.join();
    }
    std::cout << "[DDS_VIDEO] Video fetch thread stopped" << std::endl;
}

cv::Mat DDSVideoClient::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_.clone();
}

std::vector<uint8_t> DDSVideoClient::getLatestJpegData() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_jpeg_;
}

void DDSVideoClient::fetchLoop() {
#ifdef HAS_UNITREE_SDK2
    try {
        // Create video client (connects to "videohub" service)
        unitree::robot::go2::VideoClient video_client;
        video_client.SetTimeout(2.0f);  // 2 second timeout
        video_client.Init();

        std::cout << "[DDS_VIDEO] Connected to videohub service" << std::endl;

        std::vector<uint8_t> jpeg_data;
        auto start_time = std::chrono::steady_clock::now();
        int frames_since_fps_update = 0;
        auto last_fps_time = start_time;

        while (running_.load()) {
            auto frame_start = std::chrono::steady_clock::now();

            // Fetch frame from robot
            int ret = video_client.GetImageSample(jpeg_data);

            if (ret == 0 && !jpeg_data.empty()) {
                // Decode JPEG
                cv::Mat frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

                if (!frame.empty()) {
                    // Update latest frame
                    {
                        std::lock_guard<std::mutex> lock(frame_mutex_);
                        latest_frame_ = frame;
                        latest_jpeg_ = jpeg_data;
                    }

                    frame_count_.fetch_add(1);
                    frames_since_fps_update++;

                    // Call callback if set
                    if (frame_callback_) {
                        frame_callback_(frame, jpeg_data);
                    }

                    // Update FPS every second
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
                    if (elapsed >= 1.0) {
                        current_fps_.store(static_cast<float>(frames_since_fps_update / elapsed));
                        frames_since_fps_update = 0;
                        last_fps_time = now;
                    }
                } else {
                    std::cerr << "[DDS_VIDEO] Failed to decode JPEG (" << jpeg_data.size() << " bytes)" << std::endl;
                    error_count_.fetch_add(1);
                }
            } else {
                error_count_.fetch_add(1);
                if (error_count_.load() % 10 == 1) {
                    std::cerr << "[DDS_VIDEO] Failed to get frame (ret=" << ret << ")" << std::endl;
                }
                // Wait a bit before retrying on error
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // Minimal delay to avoid hammering the service
            // The service itself throttles to ~15-20 FPS
            auto frame_end = std::chrono::steady_clock::now();
            auto frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start);
            if (frame_time.count() < 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10 - frame_time.count()));
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[DDS_VIDEO] Exception in fetch loop: " << e.what() << std::endl;
    }
#endif
}
