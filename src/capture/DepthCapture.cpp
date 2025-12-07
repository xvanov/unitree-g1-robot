#include "capture/DepthCapture.h"
#include <librealsense2/rs.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// RealSense implementation details
struct DepthCapture::RealSenseImpl {
    rs2::pipeline pipeline;
    rs2::config config;
    rs2::align align_to_color{RS2_STREAM_COLOR};
    bool pipeline_started = false;
};

DepthCapture::DepthCapture()
    : rs_impl_(std::make_unique<RealSenseImpl>())
{
}

DepthCapture::~DepthCapture() {
    stopRecording();
    stopCapture();
}

bool DepthCapture::init() {
    try {
        // Check for RealSense devices
        rs2::context ctx;
        auto devices = ctx.query_devices();

        if (devices.size() == 0) {
            std::cerr << "[DEPTH] No RealSense devices found" << std::endl;
            return false;
        }

        std::cout << "[DEPTH] Found " << devices.size() << " RealSense device(s)" << std::endl;
        for (auto&& dev : devices) {
            std::cout << "[DEPTH]   " << dev.get_info(RS2_CAMERA_INFO_NAME)
                     << " (S/N: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ")" << std::endl;
        }

        // Configure streams
        rs_impl_->config.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
        rs_impl_->config.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);

        initialized_.store(true);
        std::cout << "[DEPTH] RealSense initialized (" << width_ << "x" << height_
                  << " @ " << fps_ << " fps)" << std::endl;
        return true;

    } catch (const rs2::error& e) {
        std::cerr << "[DEPTH] RealSense error: " << e.what() << std::endl;
        return false;
    }
}

bool DepthCapture::startCapture() {
    if (!initialized_.load()) {
        std::cerr << "[DEPTH] Not initialized" << std::endl;
        return false;
    }

    if (capturing_.load()) {
        return true;  // Already capturing
    }

    try {
        // Start the pipeline
        auto profile = rs_impl_->pipeline.start(rs_impl_->config);
        rs_impl_->pipeline_started = true;

        // Get camera intrinsics
        auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto intrinsics = depth_stream.get_intrinsics();
        fx_ = intrinsics.fx;
        fy_ = intrinsics.fy;
        cx_ = intrinsics.ppx;
        cy_ = intrinsics.ppy;

        // Get depth scale
        auto sensor = profile.get_device().first<rs2::depth_sensor>();
        depth_scale_ = sensor.get_depth_scale();

        std::cout << "[DEPTH] Intrinsics: fx=" << fx_ << ", fy=" << fy_
                  << ", cx=" << cx_ << ", cy=" << cy_
                  << ", scale=" << depth_scale_ << std::endl;

        capturing_.store(true);
        capture_thread_ = std::thread(&DepthCapture::captureLoop, this);

        std::cout << "[DEPTH] Capture started" << std::endl;
        return true;

    } catch (const rs2::error& e) {
        std::cerr << "[DEPTH] Failed to start capture: " << e.what() << std::endl;
        return false;
    }
}

void DepthCapture::stopCapture() {
    if (!capturing_.load()) return;

    capturing_.store(false);

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    if (rs_impl_->pipeline_started) {
        try {
            rs_impl_->pipeline.stop();
            rs_impl_->pipeline_started = false;
        } catch (...) {}
    }

    std::cout << "[DEPTH] Capture stopped (total frames: " << frame_count_.load() << ")" << std::endl;
}

void DepthCapture::captureLoop() {
    rs2::colorizer color_map;  // For depth visualization

    while (capturing_.load()) {
        try {
            // Wait for frames with timeout
            rs2::frameset frames;
            if (!rs_impl_->pipeline.poll_for_frames(&frames)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Align depth to color if requested
            if (align_to_color_) {
                frames = rs_impl_->align_to_color.process(frames);
            }

            auto color_frame = frames.get_color_frame();
            auto depth_frame = frames.get_depth_frame();

            if (!color_frame || !depth_frame) {
                continue;
            }

            // Create DepthFrame
            DepthFrame df;
            df.timestamp = frames.get_timestamp() / 1000.0;  // Convert to seconds
            df.frame_number = frame_count_.fetch_add(1);
            df.fx = fx_;
            df.fy = fy_;
            df.cx = cx_;
            df.cy = cy_;
            df.depth_scale = depth_scale_;

            // Convert to OpenCV Mat
            df.color = cv::Mat(cv::Size(width_, height_), CV_8UC3,
                              (void*)color_frame.get_data(), cv::Mat::AUTO_STEP).clone();

            df.depth = cv::Mat(cv::Size(width_, height_), CV_16UC1,
                              (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP).clone();

            // Colorize depth for visualization
            auto colorized = color_map.colorize(depth_frame);
            df.depth_colorized = cv::Mat(cv::Size(width_, height_), CV_8UC3,
                                        (void*)colorized.get_data(), cv::Mat::AUTO_STEP).clone();

            // Update latest frame
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = df;
            }

            // Call callback
            if (frame_callback_) {
                frame_callback_(df);
            }

            // Record if enabled
            if (recording_.load()) {
                saveFrame(df);
            }

        } catch (const rs2::error& e) {
            std::cerr << "[DEPTH] Frame error: " << e.what() << std::endl;
        }
    }
}

bool DepthCapture::startRecording(const std::string& session_dir) {
    std::lock_guard<std::mutex> lock(recording_mutex_);

    session_dir_ = session_dir;

    // Create directories
    try {
        std::filesystem::create_directories(session_dir_ + "/color");
        std::filesystem::create_directories(session_dir_ + "/depth");
    } catch (const std::exception& e) {
        std::cerr << "[DEPTH] Failed to create directories: " << e.what() << std::endl;
        return false;
    }

    // Write camera intrinsics
    json meta;
    meta["width"] = width_;
    meta["height"] = height_;
    meta["fps"] = fps_;
    meta["fx"] = fx_;
    meta["fy"] = fy_;
    meta["cx"] = cx_;
    meta["cy"] = cy_;
    meta["depth_scale"] = depth_scale_;
    meta["aligned"] = align_to_color_;

    std::ofstream meta_file(session_dir_ + "/depth_meta.json");
    if (meta_file.is_open()) {
        meta_file << meta.dump(2);
        meta_file.close();
    }

    recorded_frames_.store(0);
    recording_.store(true);

    std::cout << "[DEPTH] Recording to " << session_dir_ << std::endl;
    return true;
}

void DepthCapture::stopRecording() {
    if (!recording_.load()) return;

    recording_.store(false);

    // Update metadata with frame count
    std::lock_guard<std::mutex> lock(recording_mutex_);

    json meta;
    std::ifstream meta_in(session_dir_ + "/depth_meta.json");
    if (meta_in.is_open()) {
        meta_in >> meta;
        meta_in.close();
    }

    meta["total_frames"] = recorded_frames_.load();

    std::ofstream meta_out(session_dir_ + "/depth_meta.json");
    if (meta_out.is_open()) {
        meta_out << meta.dump(2);
        meta_out.close();
    }

    std::cout << "[DEPTH] Recording stopped (" << recorded_frames_.load() << " frames)" << std::endl;
}

void DepthCapture::saveFrame(const DepthFrame& frame) {
    std::lock_guard<std::mutex> lock(recording_mutex_);

    uint64_t idx = recorded_frames_.fetch_add(1);

    // Generate filename
    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(8) << idx;
    std::string base = ss.str();

    // Save color as JPEG
    std::string color_path = session_dir_ + "/color/" + base + ".jpg";
    cv::imwrite(color_path, frame.color, {cv::IMWRITE_JPEG_QUALITY, 90});

    // Save depth as PNG (lossless 16-bit)
    std::string depth_path = session_dir_ + "/depth/" + base + ".png";
    cv::imwrite(depth_path, frame.depth);

    // Save timestamp to a simple index file (append)
    static std::ofstream index_file;
    if (!index_file.is_open()) {
        index_file.open(session_dir_ + "/timestamps.txt");
    }
    index_file << idx << " " << std::fixed << std::setprecision(6) << frame.timestamp << "\n";
    index_file.flush();
}

DepthFrame DepthCapture::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_;
}

void DepthCapture::setResolution(int width, int height) {
    if (!capturing_.load()) {
        width_ = width;
        height_ = height;
    }
}

void DepthCapture::setFramerate(int fps) {
    if (!capturing_.load()) {
        fps_ = fps;
    }
}
