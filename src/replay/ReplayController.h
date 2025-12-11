#pragma once

#include <atomic>
#include <mutex>
#include <chrono>

namespace replay {

// Playback states
enum class PlaybackState {
    STOPPED,
    PLAYING,
    PAUSED,
    SEEKING,
    FINISHED
};

/**
 * ReplayController - Playback control for sensor replay
 *
 * Manages playback speed, pause/resume, seeking, and loop mode.
 * Thread-safe for use with background replay thread.
 */
class ReplayController {
public:
    ReplayController();
    ~ReplayController() = default;

    // Playback control
    void play();
    void pause();
    void resume();
    void stop();

    // Speed control
    void setSpeed(float multiplier);  // 0.25x to 4.0x
    float getSpeed() const { return speed_.load(); }

    // Seeking
    void seekTo(float progress);      // 0.0 to 1.0
    void seekToTime(int64_t timestamp_us);
    bool isSeekPending() const { return seek_pending_.load(); }
    float getSeekTarget() const { return seek_target_progress_.load(); }
    int64_t getSeekTargetTime() const { return seek_target_time_.load(); }
    void clearSeekPending();

    // Loop mode
    void setLoop(bool enabled) { loop_enabled_.store(enabled); }
    bool isLoopEnabled() const { return loop_enabled_.load(); }

    // State queries
    PlaybackState getState() const { return state_.load(); }
    bool isPlaying() const { return state_.load() == PlaybackState::PLAYING; }
    bool isPaused() const { return state_.load() == PlaybackState::PAUSED; }
    bool isFinished() const { return state_.load() == PlaybackState::FINISHED; }
    bool isStopped() const { return state_.load() == PlaybackState::STOPPED; }

    // Progress tracking (set by ReplaySensorSource)
    void setProgress(float progress);
    float getProgress() const { return progress_.load(); }

    void setElapsedTime(float seconds);
    float getElapsedTime() const { return elapsed_time_.load(); }

    void setTotalTime(float seconds) { total_time_.store(seconds); }
    float getTotalTime() const { return total_time_.load(); }

    // Message counters (set by ReplaySensorSource)
    void incrementLidarCount() { lidar_replayed_++; }
    void incrementImuCount() { imu_replayed_++; }
    void incrementPoseCount() { pose_replayed_++; }
    void incrementImageCount() { image_replayed_++; }
    void incrementVideoFrameCount() { video_frame_replayed_++; }

    uint32_t getLidarReplayed() const { return lidar_replayed_.load(); }
    uint32_t getImuReplayed() const { return imu_replayed_.load(); }
    uint32_t getPoseReplayed() const { return pose_replayed_.load(); }
    uint32_t getImageReplayed() const { return image_replayed_.load(); }
    uint32_t getVideoFrameReplayed() const { return video_frame_replayed_.load(); }

    // Reset counters
    void resetCounters();

    // Mark playback as finished
    void setFinished();

    // For timing calculations
    std::chrono::steady_clock::time_point getPlaybackStartTime() const;
    void setPlaybackStartTime(std::chrono::steady_clock::time_point time);

private:
    std::atomic<PlaybackState> state_{PlaybackState::STOPPED};
    std::atomic<float> speed_{1.0f};
    std::atomic<bool> loop_enabled_{false};

    // Seeking
    std::atomic<bool> seek_pending_{false};
    std::atomic<float> seek_target_progress_{0.0f};
    std::atomic<int64_t> seek_target_time_{0};
    std::atomic<bool> seek_by_time_{false};

    // Progress
    std::atomic<float> progress_{0.0f};
    std::atomic<float> elapsed_time_{0.0f};
    std::atomic<float> total_time_{0.0f};

    // Message counters
    std::atomic<uint32_t> lidar_replayed_{0};
    std::atomic<uint32_t> imu_replayed_{0};
    std::atomic<uint32_t> pose_replayed_{0};
    std::atomic<uint32_t> image_replayed_{0};
    std::atomic<uint32_t> video_frame_replayed_{0};

    // Timing
    mutable std::mutex time_mutex_;
    std::chrono::steady_clock::time_point playback_start_time_;
};

} // namespace replay
