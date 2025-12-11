#include "replay/ReplayController.h"

#include <algorithm>

namespace replay {

ReplayController::ReplayController() {
    playback_start_time_ = std::chrono::steady_clock::now();
}

void ReplayController::play() {
    state_.store(PlaybackState::PLAYING);
    setPlaybackStartTime(std::chrono::steady_clock::now());
}

void ReplayController::pause() {
    if (state_.load() == PlaybackState::PLAYING) {
        state_.store(PlaybackState::PAUSED);
    }
}

void ReplayController::resume() {
    PlaybackState current = state_.load();
    if (current == PlaybackState::PAUSED || current == PlaybackState::SEEKING) {
        state_.store(PlaybackState::PLAYING);
        // Adjust playback start time to account for pause duration
        setPlaybackStartTime(std::chrono::steady_clock::now());
    }
}

void ReplayController::stop() {
    state_.store(PlaybackState::STOPPED);
    resetCounters();
    progress_.store(0.0f);
    elapsed_time_.store(0.0f);
}

void ReplayController::setSpeed(float multiplier) {
    // Clamp to valid range
    multiplier = std::max(0.25f, std::min(4.0f, multiplier));
    speed_.store(multiplier);
}

void ReplayController::seekTo(float progress) {
    progress = std::max(0.0f, std::min(1.0f, progress));
    seek_target_progress_.store(progress);
    seek_by_time_.store(false);
    seek_pending_.store(true);

    // Set state to seeking if playing
    if (state_.load() == PlaybackState::PLAYING) {
        state_.store(PlaybackState::SEEKING);
    }
}

void ReplayController::seekToTime(int64_t timestamp_us) {
    seek_target_time_.store(timestamp_us);
    seek_by_time_.store(true);
    seek_pending_.store(true);

    if (state_.load() == PlaybackState::PLAYING) {
        state_.store(PlaybackState::SEEKING);
    }
}

void ReplayController::clearSeekPending() {
    seek_pending_.store(false);

    // Return to playing if we were seeking
    if (state_.load() == PlaybackState::SEEKING) {
        state_.store(PlaybackState::PLAYING);
    }
}

void ReplayController::setProgress(float progress) {
    progress_.store(progress);
}

void ReplayController::setElapsedTime(float seconds) {
    elapsed_time_.store(seconds);
}

void ReplayController::resetCounters() {
    lidar_replayed_.store(0);
    imu_replayed_.store(0);
    pose_replayed_.store(0);
    image_replayed_.store(0);
    video_frame_replayed_.store(0);
}

void ReplayController::setFinished() {
    if (loop_enabled_.load()) {
        // Reset for loop
        resetCounters();
        progress_.store(0.0f);
        elapsed_time_.store(0.0f);
        seek_target_progress_.store(0.0f);
        seek_pending_.store(true);  // Signal to seek back to start
        seek_by_time_.store(false);
    } else {
        state_.store(PlaybackState::FINISHED);
    }
}

std::chrono::steady_clock::time_point ReplayController::getPlaybackStartTime() const {
    std::lock_guard<std::mutex> lock(time_mutex_);
    return playback_start_time_;
}

void ReplayController::setPlaybackStartTime(std::chrono::steady_clock::time_point time) {
    std::lock_guard<std::mutex> lock(time_mutex_);
    playback_start_time_ = time;
}

} // namespace replay
