#include "replay/SensorReplayer.h"

#include <msgpack.hpp>
#include <zstd.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <iostream>
#include <cstring>
#include <algorithm>

namespace replay {

// Message header size: timestamp(8) + type(1) + size(4) = 13 bytes
static constexpr size_t MESSAGE_HEADER_SIZE = 13;

// Buffer sizes
static constexpr size_t COMPRESSED_BUFFER_SIZE = 128 * 1024;     // 128KB read chunks
static constexpr size_t DECOMPRESSED_BUFFER_SIZE = 1024 * 1024;  // 1MB decompressed buffer (video frames ~330KB)

SensorReplayer::SensorReplayer()
    : compressed_buffer_(COMPRESSED_BUFFER_SIZE)
    , decompressed_buffer_(DECOMPRESSED_BUFFER_SIZE)
{
}

SensorReplayer::~SensorReplayer() {
    close();
}

bool SensorReplayer::open(const std::string& recording_path) {
    if (is_open_) {
        close();
    }

    // Determine if path is a directory or file
    std::filesystem::path path(recording_path);
    if (std::filesystem::is_directory(path)) {
        recording_dir_ = recording_path;
    } else {
        // Assume it's the sensors.bin.zst file
        recording_dir_ = path.parent_path().string();
    }

    // Check required files exist
    std::string metadata_path = recording_dir_ + "/metadata.json";
    std::string bin_path = recording_dir_ + "/sensors.bin.zst";

    if (!std::filesystem::exists(metadata_path)) {
        std::cerr << "[REPLAYER] Metadata file not found: " << metadata_path << std::endl;
        return false;
    }

    if (!std::filesystem::exists(bin_path)) {
        std::cerr << "[REPLAYER] Binary file not found: " << bin_path << std::endl;
        return false;
    }

    // Load metadata
    if (!loadMetadata(metadata_path)) {
        std::cerr << "[REPLAYER] Failed to load metadata" << std::endl;
        return false;
    }

    // Initialize decompressor
    if (!initDecompressor(bin_path)) {
        std::cerr << "[REPLAYER] Failed to initialize decompressor" << std::endl;
        return false;
    }

    is_open_ = true;
    at_end_ = false;
    current_timestamp_us_ = metadata_.start_time_us;
    start_timestamp_us_ = metadata_.start_time_us;
    messages_read_ = 0;

    std::cout << "[REPLAYER] Opened recording: " << metadata_.session_id << std::endl;
    std::cout << "[REPLAYER] Duration: " << metadata_.duration_seconds << "s, "
              << "Messages: LiDAR=" << metadata_.lidar_count
              << " IMU=" << metadata_.imu_count
              << " Pose=" << metadata_.pose_count
              << " Image=" << metadata_.image_count
              << " Video=" << metadata_.video_frame_count << std::endl;

    return true;
}

void SensorReplayer::close() {
    if (dstream_) {
        ZSTD_freeDStream(static_cast<ZSTD_DStream*>(dstream_));
        dstream_ = nullptr;
    }

    if (compressed_file_.is_open()) {
        compressed_file_.close();
    }

    is_open_ = false;
    at_end_ = true;
    decompressed_pos_ = 0;
    decompressed_available_ = 0;
}

bool SensorReplayer::loadMetadata(const std::string& metadata_path) {
    try {
        std::ifstream file(metadata_path);
        if (!file.is_open()) {
            return false;
        }

        nlohmann::json j;
        file >> j;

        metadata_.session_id = j.value("session_id", "unknown");
        metadata_.start_time_us = j.value("start_time_us", int64_t(0));
        metadata_.end_time_us = j.value("end_time_us", int64_t(0));
        metadata_.duration_seconds = j.value("duration_seconds", 0.0f);
        metadata_.robot_id = j.value("robot_id", "g1_inspector");
        metadata_.plan_path = j.value("plan_path", "");

        // Message counts
        if (j.contains("message_counts")) {
            auto& counts = j["message_counts"];
            metadata_.lidar_count = counts.value("lidar", uint32_t(0));
            metadata_.imu_count = counts.value("imu", uint32_t(0));
            metadata_.pose_count = counts.value("pose", uint32_t(0));
            metadata_.image_count = counts.value("image", uint32_t(0));
            metadata_.teleop_count = counts.value("teleop", uint32_t(0));
            metadata_.video_frame_count = counts.value("video_frame", uint32_t(0));
        }

        // File stats
        if (j.contains("file_stats")) {
            auto& stats = j["file_stats"];
            metadata_.total_bytes_raw = stats.value("bytes_raw", uint64_t(0));
            metadata_.total_bytes_compressed = stats.value("bytes_compressed", uint64_t(0));
            metadata_.compression_ratio = stats.value("compression_ratio", 0.0f);
            metadata_.buffer_overflow_count = stats.value("buffer_overflow_count", uint32_t(0));
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[REPLAYER] Error parsing metadata: " << e.what() << std::endl;
        return false;
    }
}

bool SensorReplayer::initDecompressor(const std::string& bin_path) {
    // Open compressed file
    compressed_file_.open(bin_path, std::ios::binary);
    if (!compressed_file_.is_open()) {
        return false;
    }

    // Get file size
    compressed_file_.seekg(0, std::ios::end);
    compressed_file_size_ = compressed_file_.tellg();
    compressed_file_.seekg(0, std::ios::beg);

    // Create decompression stream
    dstream_ = ZSTD_createDStream();
    if (!dstream_) {
        return false;
    }

    size_t ret = ZSTD_initDStream(static_cast<ZSTD_DStream*>(dstream_));
    if (ZSTD_isError(ret)) {
        std::cerr << "[REPLAYER] ZSTD init error: " << ZSTD_getErrorName(ret) << std::endl;
        ZSTD_freeDStream(static_cast<ZSTD_DStream*>(dstream_));
        dstream_ = nullptr;
        return false;
    }

    decompressed_pos_ = 0;
    decompressed_available_ = 0;

    return true;
}

bool SensorReplayer::fillBuffer() {
    if (!dstream_ || !compressed_file_.is_open()) {
        return false;
    }

    ZSTD_outBuffer output = {decompressed_buffer_.data(), decompressed_buffer_.size(), 0};

    // Keep decompressing until we have output or run out of input
    while (output.pos == 0) {
        // If we've consumed all compressed data, read more from file
        if (compressed_pos_ >= compressed_available_) {
            compressed_file_.read(reinterpret_cast<char*>(compressed_buffer_.data()),
                                  compressed_buffer_.size());
            compressed_available_ = compressed_file_.gcount();
            compressed_pos_ = 0;

            if (compressed_available_ == 0) {
                at_end_ = true;
                return false;
            }
        }

        // Decompress from current position in compressed buffer
        ZSTD_inBuffer input = {
            compressed_buffer_.data() + compressed_pos_,
            compressed_available_ - compressed_pos_,
            0
        };

        size_t ret = ZSTD_decompressStream(static_cast<ZSTD_DStream*>(dstream_), &output, &input);
        if (ZSTD_isError(ret)) {
            std::cerr << "[REPLAYER] Decompression error: " << ZSTD_getErrorName(ret) << std::endl;
            at_end_ = true;
            return false;
        }

        // Update compressed position based on how much was consumed
        compressed_pos_ += input.pos;

        // If output buffer is full, stop (even if more input available)
        if (output.pos == output.size) {
            break;
        }
    }

    decompressed_pos_ = 0;
    decompressed_available_ = output.pos;

    return decompressed_available_ > 0;
}

bool SensorReplayer::readBytes(void* dst, size_t size) {
    uint8_t* out = static_cast<uint8_t*>(dst);
    size_t remaining = size;

    while (remaining > 0) {
        // Check if we need more data
        if (decompressed_pos_ >= decompressed_available_) {
            if (!fillBuffer()) {
                return false;
            }
        }

        // Copy available data
        size_t available = decompressed_available_ - decompressed_pos_;
        size_t to_copy = std::min(remaining, available);

        std::memcpy(out, decompressed_buffer_.data() + decompressed_pos_, to_copy);

        out += to_copy;
        remaining -= to_copy;
        decompressed_pos_ += to_copy;
    }

    return true;
}

bool SensorReplayer::readNext(ReplayMessage& msg) {
    if (!is_open_ || at_end_) {
        return false;
    }

    // Read message header: [timestamp_us: int64][type: uint8][size: uint32]
    int64_t timestamp;
    uint8_t type;
    uint32_t payload_size;

    if (!readBytes(&timestamp, sizeof(timestamp))) {
        at_end_ = true;
        return false;
    }

    if (!readBytes(&type, sizeof(type))) {
        at_end_ = true;
        return false;
    }

    if (!readBytes(&payload_size, sizeof(payload_size))) {
        at_end_ = true;
        return false;
    }

    // Sanity check payload size (max 10MB)
    if (payload_size > 10 * 1024 * 1024) {
        std::cerr << "[REPLAYER] Invalid payload size: " << payload_size << std::endl;
        at_end_ = true;
        return false;
    }

    // Read payload
    msg.timestamp_us = timestamp;
    msg.type = static_cast<recording::MessageType>(type);
    msg.payload.resize(payload_size);

    if (payload_size > 0) {
        if (!readBytes(msg.payload.data(), payload_size)) {
            at_end_ = true;
            return false;
        }
    }

    current_timestamp_us_ = timestamp;
    messages_read_++;

    return true;
}

bool SensorReplayer::hasMore() const {
    return is_open_ && !at_end_;
}

void SensorReplayer::resetDecompressor() {
    // Reset file position
    compressed_file_.clear();
    compressed_file_.seekg(0, std::ios::beg);

    // Reset decompressor
    if (dstream_) {
        ZSTD_initDStream(static_cast<ZSTD_DStream*>(dstream_));
    }

    // Reset all buffer state
    compressed_pos_ = 0;
    compressed_available_ = 0;
    decompressed_pos_ = 0;
    decompressed_available_ = 0;
    at_end_ = false;
    current_timestamp_us_ = start_timestamp_us_;
    messages_read_ = 0;
}

// Note: buildIndex() was originally planned for optimized seeking
// but MVP uses re-decompression from start. This is simple but slower
// for long recordings. Future optimization could add seekable zstd frames.

bool SensorReplayer::seek(float progress) {
    if (!is_open_) {
        return false;
    }

    // Clamp progress
    progress = std::max(0.0f, std::min(1.0f, progress));

    // Calculate target timestamp
    int64_t duration_us = getDurationUs();
    int64_t target_us = start_timestamp_us_ + static_cast<int64_t>(progress * duration_us);

    return seekToTime(target_us);
}

bool SensorReplayer::seekToTime(int64_t target_us) {
    if (!is_open_) {
        return false;
    }

    // MVP implementation: re-decompress from start
    // This is simple but slow for long recordings
    resetDecompressor();

    // Skip messages until we reach target time
    ReplayMessage msg;
    while (readNext(msg)) {
        if (msg.timestamp_us >= target_us) {
            // Found target position, back up current timestamp
            current_timestamp_us_ = msg.timestamp_us;
            return true;
        }
    }

    // Reached end of file
    return false;
}

int64_t SensorReplayer::getDurationUs() const {
    return metadata_.end_time_us - metadata_.start_time_us;
}

float SensorReplayer::getProgress() const {
    if (!is_open_ || metadata_.duration_seconds <= 0) {
        return 0.0f;
    }

    int64_t elapsed = current_timestamp_us_ - start_timestamp_us_;
    int64_t total = getDurationUs();

    if (total <= 0) {
        return 0.0f;
    }

    return static_cast<float>(elapsed) / static_cast<float>(total);
}

// ============================================
// Decode Helpers
// ============================================

bool SensorReplayer::decodeLidarScan(const ReplayMessage& msg, DecodedLidarScan& out) {
    if (msg.type != recording::MessageType::LIDAR_SCAN || msg.payload.empty()) {
        return false;
    }

    try {
        msgpack::object_handle oh = msgpack::unpack(
            reinterpret_cast<const char*>(msg.payload.data()), msg.payload.size());
        msgpack::object obj = oh.get();

        auto map = obj.as<std::map<std::string, msgpack::object>>();

        // Decode ranges
        if (map.count("ranges")) {
            map["ranges"].convert(out.scan.ranges);
        }

        // Decode angle bounds
        if (map.count("angle_min")) {
            out.scan.angle_min = map["angle_min"].as<float>();
        }
        if (map.count("angle_max")) {
            out.scan.angle_max = map["angle_max"].as<float>();
        }

        out.timestamp_us = msg.timestamp_us;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[REPLAYER] LiDAR decode error: " << e.what() << std::endl;
        return false;
    }
}

bool SensorReplayer::decodeImuData(const ReplayMessage& msg, DecodedImuData& out) {
    if (msg.type != recording::MessageType::IMU_DATA || msg.payload.empty()) {
        return false;
    }

    try {
        msgpack::object_handle oh = msgpack::unpack(
            reinterpret_cast<const char*>(msg.payload.data()), msg.payload.size());
        msgpack::object obj = oh.get();

        auto map = obj.as<std::map<std::string, msgpack::object>>();

        // Decode RPY (primary source)
        if (map.count("rpy")) {
            std::vector<float> rpy;
            map["rpy"].convert(rpy);
            if (rpy.size() >= 3) {
                out.imu.roll = rpy[0];
                out.imu.pitch = rpy[1];
                out.imu.yaw = rpy[2];
            }
        }

        // Decode gyro
        if (map.count("gyro")) {
            std::vector<float> gyro;
            map["gyro"].convert(gyro);
            if (gyro.size() >= 3) {
                out.imu.gyro_x = gyro[0];
                out.imu.gyro_y = gyro[1];
                out.imu.gyro_z = gyro[2];
            }
        }

        // Decode accel
        if (map.count("accel")) {
            std::vector<float> accel;
            map["accel"].convert(accel);
            if (accel.size() >= 3) {
                out.imu.accel_x = accel[0];
                out.imu.accel_y = accel[1];
                out.imu.accel_z = accel[2];
            }
        }

        out.timestamp_us = msg.timestamp_us;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[REPLAYER] IMU decode error: " << e.what() << std::endl;
        return false;
    }
}

bool SensorReplayer::decodePose(const ReplayMessage& msg, DecodedPose& out) {
    if (msg.type != recording::MessageType::POSE || msg.payload.empty()) {
        return false;
    }

    try {
        msgpack::object_handle oh = msgpack::unpack(
            reinterpret_cast<const char*>(msg.payload.data()), msg.payload.size());
        msgpack::object obj = oh.get();

        auto map = obj.as<std::map<std::string, float>>();

        out.pose.x = map.count("x") ? map["x"] : 0.0f;
        out.pose.y = map.count("y") ? map["y"] : 0.0f;
        out.pose.theta = map.count("theta") ? map["theta"] : 0.0f;

        out.timestamp_us = msg.timestamp_us;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[REPLAYER] Pose decode error: " << e.what() << std::endl;
        return false;
    }
}

bool SensorReplayer::decodeImage(const ReplayMessage& msg, DecodedImage& out) {
    if (msg.type != recording::MessageType::IMAGE || msg.payload.empty()) {
        return false;
    }

    try {
        msgpack::object_handle oh = msgpack::unpack(
            reinterpret_cast<const char*>(msg.payload.data()), msg.payload.size());
        msgpack::object obj = oh.get();

        auto map = obj.as<std::map<std::string, msgpack::object>>();

        if (map.count("sequence")) {
            out.sequence_number = map["sequence"].as<int>();
        }
        if (map.count("filename")) {
            out.filename = map["filename"].as<std::string>();
        }
        if (map.count("width")) {
            out.width = map["width"].as<int>();
        }
        if (map.count("height")) {
            out.height = map["height"].as<int>();
        }

        out.timestamp_us = msg.timestamp_us;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[REPLAYER] Image decode error: " << e.what() << std::endl;
        return false;
    }
}

bool SensorReplayer::decodeTeleopCmd(const ReplayMessage& msg, DecodedTeleopCmd& out) {
    if (msg.type != recording::MessageType::TELEOP_CMD || msg.payload.empty()) {
        return false;
    }

    try {
        msgpack::object_handle oh = msgpack::unpack(
            reinterpret_cast<const char*>(msg.payload.data()), msg.payload.size());
        msgpack::object obj = oh.get();

        auto map = obj.as<std::map<std::string, msgpack::object>>();

        if (map.count("vx")) {
            out.cmd.vx = map["vx"].as<float>();
        }
        if (map.count("vy")) {
            out.cmd.vy = map["vy"].as<float>();
        }
        if (map.count("omega")) {
            out.cmd.omega = map["omega"].as<float>();
        }
        if (map.count("stand")) {
            out.cmd.stand_cmd = map["stand"].as<bool>();
        }
        if (map.count("sit")) {
            out.cmd.sit_cmd = map["sit"].as<bool>();
        }
        if (map.count("estop")) {
            out.cmd.estop_cmd = map["estop"].as<bool>();
        }

        out.timestamp_us = msg.timestamp_us;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[REPLAYER] Teleop decode error: " << e.what() << std::endl;
        return false;
    }
}

bool SensorReplayer::decodeVideoFrame(const ReplayMessage& msg, DecodedVideoFrame& out) {
    if (msg.type != recording::MessageType::VIDEO_FRAME || msg.payload.empty()) {
        return false;
    }

    try {
        msgpack::object_handle oh = msgpack::unpack(
            reinterpret_cast<const char*>(msg.payload.data()), msg.payload.size());
        msgpack::object obj = oh.get();

        auto map = obj.as<std::map<std::string, msgpack::object>>();

        if (map.count("frame")) {
            out.frame_number = map["frame"].as<uint32_t>();
        }
        if (map.count("jpeg")) {
            // Extract binary data
            auto bin = map["jpeg"].as<msgpack::type::raw_ref>();
            out.jpeg_data.assign(bin.ptr, bin.ptr + bin.size);
        }

        out.timestamp_us = msg.timestamp_us;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[REPLAYER] VideoFrame decode error: " << e.what() << std::endl;
        return false;
    }
}

} // namespace replay
