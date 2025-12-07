#pragma once

#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <future>
#include <vector>
#include <fstream>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include "recording/RecordingTypes.h"
#include "sensors/ISensorSource.h"
#include "util/Types.h"

class SensorRecorder {
public:
    SensorRecorder();
    ~SensorRecorder();

    // Session management
    bool startRecording(const std::string& session_id);
    void stopRecording();
    bool isRecording() const { return recording_.load(); }

    // Data recording methods - call from sensor callbacks
    void recordLidarScan(const LidarScan& scan);
    void recordImu(const ImuData& imu);
    void recordPose(const Pose2D& pose);
    void recordImage(const cv::Mat& image, int sequence_num);
    void recordTeleopCommand(const recording::TeleopCommandRecord& cmd);

    // Record video frame (continuous recording for playback)
    // jpeg_data: raw JPEG bytes from DDS video stream
    // If jpeg_data is empty but image is provided, will encode to JPEG
    void recordVideoFrame(const std::vector<uint8_t>& jpeg_data, const cv::Mat& image = cv::Mat());

    // Record depth frame (color + depth from RealSense)
    // color_jpeg: JPEG-encoded color image
    // depth_png: PNG-encoded 16-bit depth map
    // intrinsics: fx, fy, cx, cy, depth_scale
    void recordDepthFrame(const std::vector<uint8_t>& color_jpeg,
                          const std::vector<uint8_t>& depth_png,
                          float fx, float fy, float cx, float cy,
                          float depth_scale);

    // Get live statistics
    recording::RecordingStats getStats() const;

    // Get metadata (valid after stopRecording)
    recording::RecordingMetadata getMetadata() const { return metadata_; }

    // Get session directory
    std::string getSessionDir() const { return session_dir_; }

    // Configuration
    void setOutputDir(const std::string& dir) { output_dir_ = dir; }
    void setPlanPath(const std::string& path) { plan_path_ = path; }
    void setCompressionLevel(int level) { compression_level_ = level; }
    void setJpegQuality(int quality) { jpeg_quality_ = quality; }

    // Ring buffer constants
    static constexpr size_t RING_BUFFER_SIZE = 10 * 1024 * 1024;  // 10MB
    static constexpr size_t BUFFER_WARNING_PERCENT = 90;

private:
    // Ring buffer for sensor data
    // Design note: Uses mutex rather than lock-free for simplicity at 10-100Hz sensor rates.
    // At these frequencies, mutex overhead (~100ns) is negligible compared to message
    // serialization (~1μs) and disk I/O (~10μs). Lock-free would add complexity without
    // measurable benefit. The async writer thread ensures sensor callbacks never block on disk.
    class RingBuffer {
    public:
        explicit RingBuffer(size_t capacity);
        ~RingBuffer() = default;

        // Producer: push data, returns false if buffer full (overflow)
        bool push(const uint8_t* data, size_t size);

        // Consumer: pop data, blocks until data available or shutdown
        // Returns bytes read, 0 on shutdown
        size_t pop(uint8_t* out, size_t max_size);

        // Signal shutdown to unblock consumer
        void shutdown();

        // Statistics
        size_t getUsedBytes() const { return used_.load(); }
        float getUsagePercent() const { return 100.0f * used_.load() / capacity_; }
        size_t getOverflowCount() const { return overflow_count_.load(); }
        void clearOverflowCount() { overflow_count_.store(0); }

    private:
        std::vector<uint8_t> buffer_;
        size_t capacity_;
        size_t write_pos_ = 0;
        size_t read_pos_ = 0;
        std::atomic<size_t> used_{0};
        std::atomic<size_t> overflow_count_{0};
        std::atomic<bool> shutdown_{false};

        std::mutex mutex_;
        std::condition_variable cv_;
    };

    // Pack and queue a message for writing
    void queueMessage(recording::MessageType type, const uint8_t* payload, size_t payload_size);

    // Writer thread function
    void writerThreadFunc();

    // Write compressed data to file
    void writeCompressed(const uint8_t* data, size_t size);

    // Flush compression stream
    void flushCompressor();

    // Save image asynchronously (separate from sensor stream)
    void saveImageAsync(const cv::Mat& image, int sequence_num);

    // Wait for pending image saves
    void waitForPendingImages();

    // Cleanup completed image save futures
    void cleanupPendingImages();

    // Write metadata JSON
    void writeMetadata();

    // Configuration
    std::string output_dir_ = "data/recordings";
    std::string session_dir_;
    std::string plan_path_;
    int compression_level_ = 3;   // zstd level (1-22, 3 is good for real-time)
    int jpeg_quality_ = 85;

    // State
    std::atomic<bool> recording_{false};
    std::atomic<bool> shutting_down_{false};

    // Ring buffer and writer thread
    std::unique_ptr<RingBuffer> ring_buffer_;
    std::thread writer_thread_;

    // zstd compression context
    void* cstream_ = nullptr;  // ZSTD_CStream*
    std::vector<uint8_t> compress_buffer_;

    // Output file
    std::ofstream output_file_;
    std::mutex file_mutex_;

    // Statistics
    std::atomic<uint64_t> bytes_raw_{0};
    std::atomic<uint64_t> bytes_compressed_{0};
    std::atomic<uint32_t> messages_recorded_{0};
    std::atomic<uint32_t> lidar_count_{0};
    std::atomic<uint32_t> imu_count_{0};
    std::atomic<uint32_t> pose_count_{0};
    std::atomic<uint32_t> image_count_{0};
    std::atomic<uint32_t> teleop_count_{0};
    std::atomic<uint32_t> video_frame_count_{0};
    std::atomic<uint32_t> depth_frame_count_{0};
    std::chrono::steady_clock::time_point start_time_;

    // Metadata (populated on stop)
    recording::RecordingMetadata metadata_;

    // Async image saves (like ImageCapture pattern)
    std::mutex pending_images_mutex_;
    std::vector<std::future<void>> pending_images_;
};
