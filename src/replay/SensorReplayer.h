#pragma once

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <cstdint>

#include "recording/RecordingTypes.h"
#include "sensors/ISensorSource.h"
#include "util/Types.h"

namespace replay {

// Index entry for seeking support
struct MessageIndex {
    int64_t timestamp_us;
    size_t decompressed_offset;  // Offset in decompressed stream
};

// Extended recorded message with payload
struct ReplayMessage {
    int64_t timestamp_us = 0;
    recording::MessageType type = recording::MessageType::METADATA;
    std::vector<uint8_t> payload;
};

// Decoded sensor data variants
struct DecodedLidarScan {
    LidarScan scan;
    int64_t timestamp_us = 0;
};

struct DecodedImuData {
    ImuData imu;
    int64_t timestamp_us = 0;
};

struct DecodedPose {
    Pose2D pose;
    int64_t timestamp_us = 0;
};

struct DecodedImage {
    int sequence_number = 0;
    std::string filename;
    int width = 0;
    int height = 0;
    int64_t timestamp_us = 0;
};

struct DecodedTeleopCmd {
    recording::TeleopCommandRecord cmd;
    int64_t timestamp_us = 0;
};

struct DecodedVideoFrame {
    uint32_t frame_number = 0;
    std::vector<uint8_t> jpeg_data;
    int64_t timestamp_us = 0;
};

struct DecodedDepthFrame {
    uint32_t frame_number = 0;
    std::vector<uint8_t> color_jpeg;
    std::vector<uint8_t> depth_png;
    float fx = 0, fy = 0, cx = 0, cy = 0;
    float depth_scale = 0.001f;
    int64_t timestamp_us = 0;
};

/**
 * SensorReplayer - Low-level decoder for recorded sensor streams
 *
 * Reads zstd-compressed msgpack messages from recordings created by SensorRecorder.
 * Provides sequential reading and basic seeking support.
 */
class SensorReplayer {
public:
    SensorReplayer();
    ~SensorReplayer();

    // Session management
    bool open(const std::string& recording_path);
    void close();
    bool isOpen() const { return is_open_; }

    // Sequential reading
    bool readNext(ReplayMessage& msg);
    bool hasMore() const;

    // Seeking (MVP: re-decompresses from start)
    bool seek(float progress);  // 0.0 to 1.0
    bool seekToTime(int64_t timestamp_us);

    // Metadata access
    recording::RecordingMetadata getMetadata() const { return metadata_; }
    float getDuration() const { return metadata_.duration_seconds; }
    int64_t getDurationUs() const;
    int64_t getCurrentTime() const { return current_timestamp_us_; }
    float getProgress() const;

    // Message counts
    uint32_t getLidarCount() const { return metadata_.lidar_count; }
    uint32_t getImuCount() const { return metadata_.imu_count; }
    uint32_t getPoseCount() const { return metadata_.pose_count; }
    uint32_t getImageCount() const { return metadata_.image_count; }
    uint32_t getVideoFrameCount() const { return metadata_.video_frame_count; }
    uint32_t getDepthFrameCount() const { return metadata_.depth_frame_count; }

    // Get recording directory (for image loading)
    std::string getRecordingDir() const { return recording_dir_; }

    // Decode helpers - convert payload to typed data
    static bool decodeLidarScan(const ReplayMessage& msg, DecodedLidarScan& out);
    static bool decodeImuData(const ReplayMessage& msg, DecodedImuData& out);
    static bool decodePose(const ReplayMessage& msg, DecodedPose& out);
    static bool decodeImage(const ReplayMessage& msg, DecodedImage& out);
    static bool decodeTeleopCmd(const ReplayMessage& msg, DecodedTeleopCmd& out);
    static bool decodeVideoFrame(const ReplayMessage& msg, DecodedVideoFrame& out);
    static bool decodeDepthFrame(const ReplayMessage& msg, DecodedDepthFrame& out);

private:
    // Load metadata from JSON file
    bool loadMetadata(const std::string& metadata_path);

    // Initialize decompressor and open binary file
    bool initDecompressor(const std::string& bin_path);

    // Read and decompress more data into buffer
    bool fillBuffer();

    // Read raw bytes from decompressed buffer
    bool readBytes(void* dst, size_t size);

    // Reset decompressor for seeking
    void resetDecompressor();

    // Note: Index-based seeking is a future optimization
    // MVP uses re-decompression from start (simple but slower for long recordings)

    // State
    bool is_open_ = false;
    bool at_end_ = false;
    std::string recording_dir_;
    recording::RecordingMetadata metadata_;

    // Current position
    int64_t current_timestamp_us_ = 0;
    int64_t start_timestamp_us_ = 0;
    size_t messages_read_ = 0;

    // Compressed file
    std::ifstream compressed_file_;
    std::vector<uint8_t> compressed_buffer_;
    size_t compressed_file_size_ = 0;
    size_t compressed_available_ = 0;  // Bytes available in compressed_buffer_
    size_t compressed_pos_ = 0;        // Current position in compressed_buffer_

    // Decompression
    void* dstream_ = nullptr;  // ZSTD_DStream*
    std::vector<uint8_t> decompressed_buffer_;
    size_t decompressed_pos_ = 0;
    size_t decompressed_available_ = 0;

    // Future: Seeking index (sparse - one entry per ~1 second)
    // Not implemented in MVP - seeking uses re-decompression
    // std::vector<MessageIndex> index_;
};

} // namespace replay
