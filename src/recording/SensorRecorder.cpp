#include "recording/SensorRecorder.h"

#include <msgpack.hpp>
#include <zstd.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cstring>

// ============================================
// RingBuffer Implementation
// ============================================

SensorRecorder::RingBuffer::RingBuffer(size_t capacity)
    : buffer_(capacity)
    , capacity_(capacity)
{
}

bool SensorRecorder::RingBuffer::push(const uint8_t* data, size_t size) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (used_.load() + size > capacity_) {
        // Buffer full - drop data and count overflow
        overflow_count_++;
        return false;
    }

    // Copy data to buffer (handle wrap-around)
    size_t first_part = std::min(size, capacity_ - write_pos_);
    std::memcpy(buffer_.data() + write_pos_, data, first_part);

    if (first_part < size) {
        // Wrap around to beginning
        std::memcpy(buffer_.data(), data + first_part, size - first_part);
    }

    write_pos_ = (write_pos_ + size) % capacity_;
    used_ += size;

    cv_.notify_one();
    return true;
}

size_t SensorRecorder::RingBuffer::pop(uint8_t* out, size_t max_size) {
    std::unique_lock<std::mutex> lock(mutex_);

    // Wait for data or shutdown
    cv_.wait(lock, [this] { return used_.load() > 0 || shutdown_.load(); });

    if (shutdown_.load() && used_.load() == 0) {
        return 0;  // Shutdown with no more data
    }

    size_t to_read = std::min(max_size, used_.load());

    // Copy data (handle wrap-around)
    size_t first_part = std::min(to_read, capacity_ - read_pos_);
    std::memcpy(out, buffer_.data() + read_pos_, first_part);

    if (first_part < to_read) {
        std::memcpy(out + first_part, buffer_.data(), to_read - first_part);
    }

    read_pos_ = (read_pos_ + to_read) % capacity_;
    used_ -= to_read;

    return to_read;
}

void SensorRecorder::RingBuffer::shutdown() {
    shutdown_.store(true);
    cv_.notify_all();
}

// ============================================
// SensorRecorder Implementation
// ============================================

SensorRecorder::SensorRecorder() = default;

SensorRecorder::~SensorRecorder() {
    if (recording_.load()) {
        stopRecording();
    }
}

bool SensorRecorder::startRecording(const std::string& session_id) {
    if (recording_.load()) {
        std::cerr << "[RECORDER] Already recording - stop first" << std::endl;
        return false;
    }

    // Reset state
    shutting_down_.store(false);
    bytes_raw_.store(0);
    bytes_compressed_.store(0);
    messages_recorded_.store(0);
    lidar_count_.store(0);
    pointcloud_count_.store(0);
    imu_count_.store(0);
    pose_count_.store(0);
    motor_state_count_.store(0);
    image_count_.store(0);
    teleop_count_.store(0);
    video_frame_count_.store(0);

    // Create session directory
    session_dir_ = output_dir_ + "/" + session_id;
    try {
        std::filesystem::create_directories(session_dir_ + "/images");
        std::cout << "[RECORDER] Created session directory: " << session_dir_ << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[RECORDER] Failed to create directory: " << e.what() << std::endl;
        return false;
    }

    // Initialize ring buffer
    ring_buffer_ = std::make_unique<RingBuffer>(RING_BUFFER_SIZE);

    // Open output file
    std::string bin_path = session_dir_ + "/sensors.bin.zst";
    output_file_.open(bin_path, std::ios::binary);
    if (!output_file_.is_open()) {
        std::cerr << "[RECORDER] Failed to open output file: " << bin_path << std::endl;
        return false;
    }

    // Initialize zstd compressor
    cstream_ = ZSTD_createCStream();
    if (!cstream_) {
        std::cerr << "[RECORDER] Failed to create zstd compressor" << std::endl;
        output_file_.close();
        return false;
    }
    ZSTD_initCStream(static_cast<ZSTD_CStream*>(cstream_), compression_level_);

    // Allocate compression output buffer
    compress_buffer_.resize(ZSTD_CStreamOutSize());

    // Initialize metadata
    metadata_ = recording::RecordingMetadata();
    metadata_.session_id = session_id;
    metadata_.start_time_us = recording::getCurrentTimestampUs();
    metadata_.plan_path = plan_path_;

    start_time_ = std::chrono::steady_clock::now();

    // Start writer thread
    recording_.store(true);
    writer_thread_ = std::thread(&SensorRecorder::writerThreadFunc, this);

    std::cout << "[RECORDER] Recording started: " << session_id << std::endl;
    return true;
}

void SensorRecorder::stopRecording() {
    if (!recording_.load()) {
        return;
    }

    std::cout << "[RECORDER] Stopping recording..." << std::endl;

    // Signal shutdown
    shutting_down_.store(true);
    recording_.store(false);

    // Signal ring buffer to unblock writer thread
    if (ring_buffer_) {
        ring_buffer_->shutdown();
    }

    // Wait for writer thread to finish
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }

    // Flush compressor and close file
    flushCompressor();

    {
        std::lock_guard<std::mutex> lock(file_mutex_);
        if (output_file_.is_open()) {
            output_file_.close();
        }
    }

    // Clean up compressor
    if (cstream_) {
        ZSTD_freeCStream(static_cast<ZSTD_CStream*>(cstream_));
        cstream_ = nullptr;
    }

    // Wait for pending image saves
    waitForPendingImages();

    // Calculate final metadata
    auto end_time = std::chrono::steady_clock::now();
    float duration = std::chrono::duration<float>(end_time - start_time_).count();

    metadata_.end_time_us = recording::getCurrentTimestampUs();
    metadata_.duration_seconds = duration;
    metadata_.lidar_count = lidar_count_.load();
    metadata_.pointcloud_count = pointcloud_count_.load();
    metadata_.imu_count = imu_count_.load();
    metadata_.pose_count = pose_count_.load();
    metadata_.motor_state_count = motor_state_count_.load();
    metadata_.image_count = image_count_.load();
    metadata_.teleop_count = teleop_count_.load();
    metadata_.video_frame_count = video_frame_count_.load();
    metadata_.depth_frame_count = depth_frame_count_.load();
    metadata_.total_bytes_raw = bytes_raw_.load();
    metadata_.total_bytes_compressed = bytes_compressed_.load();
    metadata_.buffer_overflow_count = ring_buffer_ ? ring_buffer_->getOverflowCount() : 0;

    if (metadata_.total_bytes_compressed > 0) {
        metadata_.compression_ratio =
            static_cast<float>(metadata_.total_bytes_raw) / metadata_.total_bytes_compressed;
    }

    // Write metadata JSON
    writeMetadata();

    ring_buffer_.reset();

    std::cout << "[RECORDER] Recording stopped. "
              << messages_recorded_.load() << " messages, "
              << std::fixed << std::setprecision(1)
              << (bytes_compressed_.load() / (1024.0 * 1024.0)) << " MB compressed"
              << std::endl;
}

void SensorRecorder::recordLidarScan(const LidarScan& scan) {
    if (!recording_.load()) return;

    // Pack LiDAR data with msgpack
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    pk.pack_map(4);
    pk.pack("ranges");
    pk.pack(scan.ranges);
    pk.pack("angle_min");
    pk.pack(scan.angle_min);
    pk.pack("angle_max");
    pk.pack(scan.angle_max);
    pk.pack("count");
    pk.pack(static_cast<uint32_t>(scan.ranges.size()));

    queueMessage(recording::MessageType::LIDAR_SCAN,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    lidar_count_++;
}

void SensorRecorder::recordPointCloud3D(const PointCloud3D& cloud) {
    if (!recording_.load()) return;
    if (cloud.size() == 0) return;

    // Pack 3D point cloud with msgpack
    // Format: {count, x[], y[], z[], intensity[]}
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    pk.pack_map(5);
    pk.pack("count");
    pk.pack(static_cast<uint32_t>(cloud.size()));
    pk.pack("x");
    pk.pack(cloud.x);
    pk.pack("y");
    pk.pack(cloud.y);
    pk.pack("z");
    pk.pack(cloud.z);
    pk.pack("intensity");
    pk.pack(cloud.intensity);

    queueMessage(recording::MessageType::POINT_CLOUD_3D,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    pointcloud_count_++;
}

void SensorRecorder::recordImu(const ImuData& imu) {
    if (!recording_.load()) return;

    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    pk.pack_map(4);

    // Compute quaternion from RPY (roll, pitch, yaw) using ZYX Euler convention
    // This is the standard aerospace convention also used by Unitree
    float cr = std::cos(imu.roll * 0.5f);
    float sr = std::sin(imu.roll * 0.5f);
    float cp = std::cos(imu.pitch * 0.5f);
    float sp = std::sin(imu.pitch * 0.5f);
    float cy = std::cos(imu.yaw * 0.5f);
    float sy = std::sin(imu.yaw * 0.5f);

    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;

    pk.pack("quat");
    pk.pack_array(4);
    pk.pack(qw); pk.pack(qx); pk.pack(qy); pk.pack(qz);

    pk.pack("gyro");
    pk.pack_array(3);
    pk.pack(imu.gyro_x); pk.pack(imu.gyro_y); pk.pack(imu.gyro_z);

    pk.pack("accel");
    pk.pack_array(3);
    pk.pack(imu.accel_x); pk.pack(imu.accel_y); pk.pack(imu.accel_z);

    pk.pack("rpy");
    pk.pack_array(3);
    pk.pack(imu.roll); pk.pack(imu.pitch); pk.pack(imu.yaw);

    queueMessage(recording::MessageType::IMU_DATA,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    imu_count_++;
}

void SensorRecorder::recordPose(const Pose2D& pose) {
    if (!recording_.load()) return;

    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    pk.pack_map(3);
    pk.pack("x");
    pk.pack(pose.x);
    pk.pack("y");
    pk.pack(pose.y);
    pk.pack("theta");
    pk.pack(pose.theta);

    queueMessage(recording::MessageType::POSE,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    pose_count_++;
}

void SensorRecorder::recordMotorState(const MotorState& motors) {
    if (!recording_.load()) return;

    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    pk.pack_map(2);

    // Pack joint positions
    pk.pack("q");
    pk.pack_array(G1_NUM_MOTORS);
    for (int i = 0; i < G1_NUM_MOTORS; ++i) {
        pk.pack(motors.q[i]);
    }

    // Pack joint velocities
    pk.pack("dq");
    pk.pack_array(G1_NUM_MOTORS);
    for (int i = 0; i < G1_NUM_MOTORS; ++i) {
        pk.pack(motors.dq[i]);
    }

    queueMessage(recording::MessageType::MOTOR_STATE,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    motor_state_count_++;
}

void SensorRecorder::recordImage(const cv::Mat& image, int sequence_num) {
    if (!recording_.load()) return;

    // Save image asynchronously
    saveImageAsync(image.clone(), sequence_num);

    // Record reference in binary stream
    char filename[32];
    std::snprintf(filename, sizeof(filename), "img_%08d.jpg", sequence_num);

    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    pk.pack_map(4);
    pk.pack("sequence");
    pk.pack(sequence_num);
    pk.pack("filename");
    pk.pack(std::string(filename));
    pk.pack("width");
    pk.pack(image.cols);
    pk.pack("height");
    pk.pack(image.rows);

    queueMessage(recording::MessageType::IMAGE,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    image_count_++;
}

void SensorRecorder::recordTeleopCommand(const recording::TeleopCommandRecord& cmd) {
    if (!recording_.load()) return;

    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    pk.pack_map(6);
    pk.pack("vx");
    pk.pack(cmd.vx);
    pk.pack("vy");
    pk.pack(cmd.vy);
    pk.pack("omega");
    pk.pack(cmd.omega);
    pk.pack("stand");
    pk.pack(cmd.stand_cmd);
    pk.pack("sit");
    pk.pack(cmd.sit_cmd);
    pk.pack("estop");
    pk.pack(cmd.estop_cmd);

    queueMessage(recording::MessageType::TELEOP_CMD,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    teleop_count_++;
}

void SensorRecorder::recordVideoFrame(const std::vector<uint8_t>& jpeg_data, const cv::Mat& image) {
    if (!recording_.load()) return;

    std::vector<uint8_t> frame_data;

    if (!jpeg_data.empty()) {
        // Use provided JPEG data directly (from DDS)
        frame_data = jpeg_data;
    } else if (!image.empty()) {
        // Encode image to JPEG
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        cv::imencode(".jpg", image, frame_data, params);
    } else {
        return;  // No data to record
    }

    // Pack video frame: frame_num + jpeg_size + jpeg_data
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    uint32_t frame_num = video_frame_count_.load();

    pk.pack_map(2);
    pk.pack("frame");
    pk.pack(frame_num);
    pk.pack("jpeg");
    pk.pack_bin(frame_data.size());
    pk.pack_bin_body(reinterpret_cast<const char*>(frame_data.data()), frame_data.size());

    queueMessage(recording::MessageType::VIDEO_FRAME,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    video_frame_count_++;
}

void SensorRecorder::recordDepthFrame(const std::vector<uint8_t>& color_jpeg,
                                       const std::vector<uint8_t>& depth_png,
                                       float fx, float fy, float cx, float cy,
                                       float depth_scale) {
    if (!recording_.load()) return;

    if (color_jpeg.empty() || depth_png.empty()) {
        return;  // No data to record
    }

    // Pack depth frame with msgpack
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);

    uint32_t frame_num = depth_frame_count_.load();

    pk.pack_map(8);
    pk.pack("frame");
    pk.pack(frame_num);
    pk.pack("fx");
    pk.pack(fx);
    pk.pack("fy");
    pk.pack(fy);
    pk.pack("cx");
    pk.pack(cx);
    pk.pack("cy");
    pk.pack(cy);
    pk.pack("scale");
    pk.pack(depth_scale);
    pk.pack("color");
    pk.pack_bin(color_jpeg.size());
    pk.pack_bin_body(reinterpret_cast<const char*>(color_jpeg.data()), color_jpeg.size());
    pk.pack("depth");
    pk.pack_bin(depth_png.size());
    pk.pack_bin_body(reinterpret_cast<const char*>(depth_png.data()), depth_png.size());

    queueMessage(recording::MessageType::DEPTH_FRAME,
                 reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());

    depth_frame_count_++;
}

recording::RecordingStats SensorRecorder::getStats() const {
    recording::RecordingStats stats;

    stats.messages_recorded = messages_recorded_.load();
    stats.bytes_written = bytes_raw_.load();
    stats.bytes_compressed = bytes_compressed_.load();
    stats.lidar_count = lidar_count_.load();
    stats.imu_count = imu_count_.load();
    stats.pose_count = pose_count_.load();
    stats.image_count = image_count_.load();
    stats.video_frame_count = video_frame_count_.load();
    stats.depth_frame_count = depth_frame_count_.load();

    if (stats.bytes_compressed > 0) {
        stats.compression_ratio = static_cast<float>(stats.bytes_written) / stats.bytes_compressed;
    }

    auto now = std::chrono::steady_clock::now();
    stats.duration_seconds = std::chrono::duration<float>(now - start_time_).count();

    if (stats.duration_seconds > 0) {
        stats.disk_rate_mbps = (stats.bytes_compressed / (1024.0f * 1024.0f)) / stats.duration_seconds;
    }

    if (ring_buffer_) {
        stats.buffer_usage_percent = ring_buffer_->getUsagePercent();
        stats.overflow_count = ring_buffer_->getOverflowCount();
    }

    return stats;
}

void SensorRecorder::queueMessage(recording::MessageType type,
                                   const uint8_t* payload, size_t payload_size) {
    // Build message: [timestamp_us: int64][type: uint8][size: uint32][payload]
    int64_t timestamp = recording::getCurrentTimestampUs();
    uint8_t type_byte = static_cast<uint8_t>(type);
    uint32_t size = static_cast<uint32_t>(payload_size);

    // Calculate total message size
    size_t header_size = sizeof(timestamp) + sizeof(type_byte) + sizeof(size);
    size_t total_size = header_size + payload_size;

    // Build message in temporary buffer
    std::vector<uint8_t> msg(total_size);
    size_t offset = 0;

    std::memcpy(msg.data() + offset, &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);

    msg[offset++] = type_byte;

    std::memcpy(msg.data() + offset, &size, sizeof(size));
    offset += sizeof(size);

    std::memcpy(msg.data() + offset, payload, payload_size);

    // Push to ring buffer
    if (ring_buffer_) {
        bool success = ring_buffer_->push(msg.data(), msg.size());

        if (!success) {
            // Log overflow warning (rate limited)
            static auto last_warning = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<float>(now - last_warning).count() > 1.0f) {
                std::cerr << "[RECORDER] Warning: Ring buffer overflow, dropping data" << std::endl;
                last_warning = now;
            }
        }

        // Warn if buffer usage is high
        float usage = ring_buffer_->getUsagePercent();
        static bool warned_high_usage = false;
        if (usage > BUFFER_WARNING_PERCENT && !warned_high_usage) {
            std::cerr << "[RECORDER] Warning: Buffer usage at "
                      << static_cast<int>(usage) << "%" << std::endl;
            warned_high_usage = true;
        } else if (usage < 50.0f) {
            warned_high_usage = false;
        }
    }

    bytes_raw_ += total_size;
    messages_recorded_++;
}

void SensorRecorder::writerThreadFunc() {
    std::vector<uint8_t> read_buffer(64 * 1024);  // 64KB read chunks

    while (!shutting_down_.load() || (ring_buffer_ && ring_buffer_->getUsedBytes() > 0)) {
        size_t bytes_read = ring_buffer_->pop(read_buffer.data(), read_buffer.size());

        if (bytes_read == 0) {
            continue;  // Shutdown signaled or spurious wakeup
        }

        // Compress and write to file
        writeCompressed(read_buffer.data(), bytes_read);
    }
}

void SensorRecorder::writeCompressed(const uint8_t* data, size_t size) {
    ZSTD_CStream* cstream = static_cast<ZSTD_CStream*>(cstream_);

    ZSTD_inBuffer input = {data, size, 0};

    while (input.pos < input.size) {
        ZSTD_outBuffer output = {compress_buffer_.data(), compress_buffer_.size(), 0};

        size_t ret = ZSTD_compressStream(cstream, &output, &input);
        if (ZSTD_isError(ret)) {
            std::cerr << "[RECORDER] Compression error: " << ZSTD_getErrorName(ret) << std::endl;
            return;
        }

        if (output.pos > 0) {
            std::lock_guard<std::mutex> lock(file_mutex_);
            output_file_.write(reinterpret_cast<const char*>(compress_buffer_.data()), output.pos);
            bytes_compressed_ += output.pos;
        }
    }
}

void SensorRecorder::flushCompressor() {
    if (!cstream_) return;

    ZSTD_CStream* cstream = static_cast<ZSTD_CStream*>(cstream_);
    ZSTD_outBuffer output = {compress_buffer_.data(), compress_buffer_.size(), 0};

    size_t remaining = ZSTD_endStream(cstream, &output);
    if (ZSTD_isError(remaining)) {
        std::cerr << "[RECORDER] Error flushing compressor: " << ZSTD_getErrorName(remaining) << std::endl;
        return;
    }

    if (output.pos > 0) {
        std::lock_guard<std::mutex> lock(file_mutex_);
        output_file_.write(reinterpret_cast<const char*>(compress_buffer_.data()), output.pos);
        bytes_compressed_ += output.pos;
    }
}

void SensorRecorder::saveImageAsync(const cv::Mat& image, int sequence_num) {
    // Clean up completed futures periodically
    {
        std::lock_guard<std::mutex> lock(pending_images_mutex_);
        if (pending_images_.size() > 5) {
            cleanupPendingImages();
        }
    }

    auto future = std::async(std::launch::async, [this, image, sequence_num]() {
        if (shutting_down_.load()) return;

        char filename[32];
        std::snprintf(filename, sizeof(filename), "img_%08d.jpg", sequence_num);
        std::string path = session_dir_ + "/images/" + filename;

        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        cv::imwrite(path, image, params);
    });

    std::lock_guard<std::mutex> lock(pending_images_mutex_);
    pending_images_.push_back(std::move(future));
}

void SensorRecorder::waitForPendingImages() {
    std::vector<std::future<void>> futures;
    {
        std::lock_guard<std::mutex> lock(pending_images_mutex_);
        futures = std::move(pending_images_);
        pending_images_.clear();
    }

    for (auto& f : futures) {
        if (f.valid()) {
            f.wait();
        }
    }
}

void SensorRecorder::cleanupPendingImages() {
    // Already holding pending_images_mutex_
    pending_images_.erase(
        std::remove_if(pending_images_.begin(), pending_images_.end(),
            [](std::future<void>& f) {
                return f.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
            }),
        pending_images_.end()
    );
}

void SensorRecorder::writeMetadata() {
    nlohmann::json j;

    j["session_id"] = metadata_.session_id;
    j["start_time_us"] = metadata_.start_time_us;
    j["end_time_us"] = metadata_.end_time_us;
    j["duration_seconds"] = metadata_.duration_seconds;
    j["robot_id"] = metadata_.robot_id;
    j["plan_path"] = metadata_.plan_path;

    j["message_counts"] = {
        {"lidar", metadata_.lidar_count},
        {"pointcloud_3d", metadata_.pointcloud_count},
        {"imu", metadata_.imu_count},
        {"pose", metadata_.pose_count},
        {"motor_state", metadata_.motor_state_count},
        {"image", metadata_.image_count},
        {"teleop", metadata_.teleop_count},
        {"video_frame", metadata_.video_frame_count},
        {"depth_frame", metadata_.depth_frame_count}
    };

    j["file_stats"] = {
        {"bytes_raw", metadata_.total_bytes_raw},
        {"bytes_compressed", metadata_.total_bytes_compressed},
        {"compression_ratio", metadata_.compression_ratio},
        {"buffer_overflow_count", metadata_.buffer_overflow_count}
    };

    std::string meta_path = session_dir_ + "/metadata.json";
    std::ofstream file(meta_path);
    if (file.is_open()) {
        file << j.dump(2);
        file.close();
        std::cout << "[RECORDER] Metadata saved: " << meta_path << std::endl;
    } else {
        std::cerr << "[RECORDER] Failed to save metadata: " << meta_path << std::endl;
    }
}
