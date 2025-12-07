#pragma once

#include <string>
#include <memory>
#include <atomic>
#include "teleop/KeyboardTeleop.h"  // For VideoSource enum

class SensorManager;
class LocoController;
class SensorRecorder;
class TeleopController;

// Teleop mode types
enum class TeleopMode {
    GAMEPAD,
    KEYBOARD
};

// Teleop runner - manages teleop session with optional recording
class TeleopRunner {
public:
    TeleopRunner();
    ~TeleopRunner();

    // Configure before running
    void setMode(TeleopMode mode) { mode_ = mode; }
    void setNetworkInterface(const std::string& iface) { network_interface_ = iface; }
    void setRecordingEnabled(bool enable, const std::string& session_id = "") {
        recording_enabled_ = enable;
        session_id_ = session_id;
    }
    void setPlanPath(const std::string& path) { plan_path_ = path; }
    void setDryRun(bool dry_run) { dry_run_ = dry_run; }
    void setRobotIP(const std::string& ip) { robot_ip_ = ip; }
    void setAutoStream(bool auto_stream) { auto_stream_ = auto_stream; }
    void setVideoSource(VideoSource source) { video_source_ = source; }

    // Run teleop (blocking - returns when user quits or signal received)
    // Returns 0 on success, non-zero on error
    int run();

    // Stop teleop (can be called from signal handler)
    void stop();

    // Check if running
    bool isRunning() const { return running_.load(); }

private:
    // Initialize hardware components
    bool initHardware();

    // Run gamepad teleop mode
    int runGamepadMode();

    // Run keyboard teleop mode
    int runKeyboardMode();

    // Wire sensor callbacks to recorder
    void wireRecorderCallbacks();

    // Start video stream on robot via SSH (returns child PID, -1 on failure)
    pid_t startRobotStream();

    // Stop the robot stream process
    void stopRobotStream();

    // Get local IP address on the robot's subnet
    std::string getLocalIP();

    // Configuration
    TeleopMode mode_ = TeleopMode::KEYBOARD;
    std::string network_interface_;
    bool recording_enabled_ = false;
    std::string session_id_;
    std::string plan_path_;
    std::string robot_ip_;  // Robot IP for video streaming
    bool dry_run_ = false;  // Skip robot connection, just test camera/UI
    bool auto_stream_ = true;  // Auto-start video stream on robot via SSH (only for GStreamer)
    VideoSource video_source_ = VideoSource::DDS;  // Default to DDS video
    pid_t stream_pid_ = -1;  // PID of the stream process

    // State
    std::atomic<bool> running_{false};

    // Components
    std::shared_ptr<SensorManager> sensor_manager_;
    std::shared_ptr<LocoController> loco_controller_;
    std::unique_ptr<SensorRecorder> recorder_;
    std::unique_ptr<TeleopController> gamepad_controller_;
    std::unique_ptr<KeyboardTeleop> keyboard_teleop_;
};
