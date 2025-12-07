#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>
#include "util/Types.h"
#include "sensors/ISensorSource.h"

namespace recording {

// Message types for the binary recording format
enum class MessageType : uint8_t {
    LIDAR_SCAN = 1,
    IMU_DATA = 2,
    POSE = 3,
    IMAGE = 4,
    MOTOR_STATE = 5,
    TELEOP_CMD = 6,    // Teleop commands for replay
    VIDEO_FRAME = 7,   // Continuous video frames for playback
    DEPTH_FRAME = 8,   // Depth camera frames (color + depth)
    METADATA = 255
};

// Header for each recorded message in the binary stream
struct RecordedMessage {
    int64_t timestamp_us;  // Microseconds since epoch
    MessageType type;
    // Payload follows (msgpack encoded)
};

// Session metadata - written to metadata.json
struct RecordingMetadata {
    std::string session_id;
    int64_t start_time_us = 0;
    int64_t end_time_us = 0;
    float duration_seconds = 0.0f;
    std::string robot_id = "g1_inspector";
    std::string plan_path;  // Empty if no plan loaded

    // Message counts
    uint32_t lidar_count = 0;
    uint32_t imu_count = 0;
    uint32_t pose_count = 0;
    uint32_t image_count = 0;
    uint32_t teleop_count = 0;
    uint32_t video_frame_count = 0;
    uint32_t depth_frame_count = 0;

    // File statistics
    uint64_t total_bytes_raw = 0;
    uint64_t total_bytes_compressed = 0;
    float compression_ratio = 0.0f;

    // Overflow tracking
    uint32_t buffer_overflow_count = 0;
};

// Live recording statistics
struct RecordingStats {
    uint32_t messages_recorded = 0;
    uint64_t bytes_written = 0;        // Raw bytes before compression
    uint64_t bytes_compressed = 0;     // Bytes written to disk (compressed)
    float compression_ratio = 0.0f;
    float duration_seconds = 0.0f;
    float disk_rate_mbps = 0.0f;

    // Per-type counts
    uint32_t lidar_count = 0;
    uint32_t imu_count = 0;
    uint32_t pose_count = 0;
    uint32_t image_count = 0;
    uint32_t video_frame_count = 0;
    uint32_t depth_frame_count = 0;

    // Buffer status
    float buffer_usage_percent = 0.0f;
    uint32_t overflow_count = 0;
};

// Teleop command for recording/replay
struct TeleopCommandRecord {
    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
    bool stand_cmd = false;
    bool sit_cmd = false;
    bool estop_cmd = false;
};

// Image reference (stored in binary, actual image in separate file)
struct ImageReference {
    int sequence_number = 0;
    std::string filename;
    int width = 0;
    int height = 0;
};

// Helper to get current timestamp in microseconds
inline int64_t getCurrentTimestampUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// Helper to get timestamp in milliseconds
inline int64_t getCurrentTimestampMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

} // namespace recording
