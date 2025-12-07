#include "teleop/TeleopRunner.h"
#include "teleop/TeleopController.h"
#include "teleop/KeyboardTeleop.h"
#include "recording/SensorRecorder.h"
#include "sensors/SensorManager.h"
#include "locomotion/LocoController.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <cstring>
#include <fcntl.h>

// External shutdown flag (shared with main.cpp signal handler)
// Defined in main.cpp - if linking without main.cpp (e.g., unit tests),
// the linker will use the weak definition below
#if defined(__GNUC__) || defined(__clang__)
__attribute__((weak))
#endif
std::atomic<bool> g_running{true};  // Fallback definition for standalone builds

TeleopRunner::TeleopRunner() = default;

TeleopRunner::~TeleopRunner() {
    stop();
}

int TeleopRunner::run() {
    running_.store(true);

    // Initialize hardware (skip in dry-run mode)
    if (dry_run_) {
        std::cout << "[TELEOP] Dry-run mode: skipping robot connection" << std::endl;
        std::cout << "[TELEOP] Camera and keyboard controls will work, but no robot movement" << std::endl;
    } else if (!initHardware()) {
        std::cerr << "[TELEOP] Failed to initialize hardware" << std::endl;
        running_.store(false);
        return 1;
    }

    // Initialize recorder if recording enabled
    if (recording_enabled_) {
        recorder_ = std::make_unique<SensorRecorder>();
        recorder_->setPlanPath(plan_path_);

        if (session_id_.empty()) {
            session_id_ = "teleop_" + std::to_string(std::time(nullptr));
        }

        if (!recorder_->startRecording(session_id_)) {
            std::cerr << "[TELEOP] Failed to start recording" << std::endl;
            // Continue without recording
            recorder_.reset();
        } else {
            // Wire sensor callbacks to recorder
            wireRecorderCallbacks();
        }
    }

    int result = 0;

    // Run appropriate mode
    switch (mode_) {
        case TeleopMode::GAMEPAD:
            result = runGamepadMode();
            break;

        case TeleopMode::KEYBOARD:
            result = runKeyboardMode();
            break;
    }

    // Stop recording
    if (recorder_ && recorder_->isRecording()) {
        std::cout << "[TELEOP] Flushing recording..." << std::endl;
        recorder_->stopRecording();
    }

    // Stop robot
    if (loco_controller_) {
        std::cout << "[TELEOP] Stopping robot..." << std::endl;
        loco_controller_->stop();
    }

    running_.store(false);
    return result;
}

void TeleopRunner::stop() {
    running_.store(false);

    // Signal keyboard teleop to stop if running
    if (keyboard_teleop_) {
        keyboard_teleop_->stop();
    }
}

bool TeleopRunner::initHardware() {
#ifdef HAS_UNITREE_SDK2
    // Initialize sensor manager (optional - may fail if low-level not accessible)
    sensor_manager_ = std::make_shared<SensorManager>();
    if (!sensor_manager_->init(network_interface_)) {
        std::cerr << "[TELEOP] Warning: Failed to initialize sensors (low-level not accessible)" << std::endl;
        std::cerr << "[TELEOP] Continuing with high-level control only..." << std::endl;
        // Don't fail - allow keyboard teleop with just camera + high-level control
        sensor_manager_.reset();
    }

    // Initialize locomotion controller (required for movement)
    loco_controller_ = std::make_shared<LocoController>();
    if (!loco_controller_->init(network_interface_)) {
        std::cerr << "[TELEOP] Failed to initialize locomotion" << std::endl;
        return false;
    }

    std::cout << "[TELEOP] Hardware initialized" << std::endl;
    if (!sensor_manager_) {
        std::cout << "[TELEOP] Note: Running without sensor streaming (keyboard + camera only)" << std::endl;
    }
    return true;
#else
    std::cerr << "[TELEOP] unitree_sdk2 not available - hardware disabled" << std::endl;
    // Create mock objects for testing without hardware
    sensor_manager_ = std::make_shared<SensorManager>();
    loco_controller_ = std::make_shared<LocoController>();
    return true;
#endif
}

int TeleopRunner::runGamepadMode() {
    std::cout << "[TELEOP] Starting gamepad mode..." << std::endl;
    std::cout << "[TELEOP] Controls:" << std::endl;
    std::cout << "  Left stick Y  - Forward/Backward" << std::endl;
    std::cout << "  Left stick X  - Strafe Left/Right" << std::endl;
    std::cout << "  Right stick X - Rotate" << std::endl;
    std::cout << "  A button      - Stand up" << std::endl;
    std::cout << "  B button      - Sit down" << std::endl;
    std::cout << "  Start         - Emergency stop" << std::endl;
    std::cout << "\nPress Ctrl+C to quit\n" << std::endl;

    gamepad_controller_ = std::make_unique<TeleopController>();

    // Main teleop loop
    auto last_cmd_time = std::chrono::steady_clock::now();
    const auto cmd_interval = std::chrono::milliseconds(20);  // 50 Hz

    Pose2D current_pose{0, 0, 0};
    int pose_record_counter = 0;

    while (running_.load() && g_running.load()) {
        // Get wireless remote data
        uint8_t remote_data[40] = {0};
        bool has_gamepad = sensor_manager_->getRawWirelessRemote(remote_data);

        if (has_gamepad) {
            gamepad_controller_->update(remote_data);
            TeleopCommand cmd = gamepad_controller_->getCommand();

            // Handle button actions
            if (cmd.estop_pressed) {
                std::cout << "[TELEOP] Emergency stop!" << std::endl;
                loco_controller_->emergencyStop();
                continue;
            }

            if (cmd.stand_pressed) {
                std::cout << "[TELEOP] Standing up..." << std::endl;
                loco_controller_->standUp();
            }

            if (cmd.sit_pressed) {
                std::cout << "[TELEOP] Sitting down..." << std::endl;
                loco_controller_->sitDown();
            }

            // Send velocity command at fixed rate
            auto now = std::chrono::steady_clock::now();
            if (now - last_cmd_time >= cmd_interval) {
                loco_controller_->setVelocity(cmd.vx, cmd.vy, cmd.omega);
                last_cmd_time = now;

                // Record teleop command if recording
                if (recorder_ && recorder_->isRecording()) {
                    recording::TeleopCommandRecord rec;
                    rec.vx = cmd.vx;
                    rec.vy = cmd.vy;
                    rec.omega = cmd.omega;
                    rec.stand_cmd = cmd.stand;
                    rec.sit_cmd = cmd.sit;
                    rec.estop_cmd = cmd.emergency_stop;
                    recorder_->recordTeleopCommand(rec);

                    // Record pose at ~50Hz using IMU-derived orientation
                    pose_record_counter++;
                    if (pose_record_counter >= 1) {
                        // Get pose estimate from SensorManager (IMU yaw-based)
                        // Full SLAM would provide x,y position; this gives orientation only
                        current_pose = sensor_manager_->getEstimatedPose();
                        recorder_->recordPose(current_pose);
                        pose_record_counter = 0;
                    }
                }
            }
        } else {
            // No gamepad data - zero velocity for safety
            loco_controller_->setVelocity(0, 0, 0);
        }

        // Small sleep to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "[TELEOP] Gamepad mode ended" << std::endl;
    return 0;
}

int TeleopRunner::runKeyboardMode() {
    std::cout << "[TELEOP] Starting keyboard mode..." << std::endl;

    // Only start SSH GStreamer stream for legacy mode
    if (video_source_ == VideoSource::GSTREAMER && !robot_ip_.empty() && auto_stream_) {
        std::cout << "[TELEOP] Using GStreamer video (legacy mode with SSH)" << std::endl;
        if (startRobotStream() < 0) {
            std::cerr << "[TELEOP] Warning: Could not auto-start video stream" << std::endl;
            std::cerr << "[TELEOP] You can start it manually on robot:" << std::endl;
            std::cerr << "[TELEOP]   gst-launch-1.0 v4l2src device=/dev/video2 ! \\" << std::endl;
            std::cerr << "[TELEOP]     \"video/x-raw,format=UYVY,width=640,height=480,framerate=30/1\" ! \\" << std::endl;
            std::cerr << "[TELEOP]     videoconvert ! x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast ! \\" << std::endl;
            std::cerr << "[TELEOP]     rtph264pay ! udpsink host=<YOUR_PC_IP> port=5000" << std::endl;
        }
        // Give the stream a moment to start before opening receiver
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else if (video_source_ == VideoSource::DDS) {
        std::cout << "[TELEOP] Using DDS video (no SSH required)" << std::endl;
    }

    keyboard_teleop_ = std::make_unique<KeyboardTeleop>(
        sensor_manager_.get(),
        loco_controller_.get(),
        recorder_.get()
    );

    // Configure video source
    keyboard_teleop_->setVideoSource(video_source_);
    keyboard_teleop_->setNetworkInterface(network_interface_);

    // Configure depth streaming if enabled
    if (depth_port_ > 0) {
        keyboard_teleop_->setDepthPort(depth_port_);
    }

    // Set robot IP for GStreamer fallback if specified
    if (!robot_ip_.empty()) {
        keyboard_teleop_->setRobotIP(robot_ip_);
    }

    // Run blocking teleop loop
    keyboard_teleop_->run();

    // Stop the robot stream (only needed for GStreamer mode)
    if (video_source_ == VideoSource::GSTREAMER) {
        stopRobotStream();
    }

    std::cout << "[TELEOP] Keyboard mode ended" << std::endl;
    return 0;
}

void TeleopRunner::wireRecorderCallbacks() {
    if (!recorder_ || !sensor_manager_) return;

    // Wire LiDAR callback
    sensor_manager_->setLidarCallback([this](const LidarScan& scan) {
        if (recorder_ && recorder_->isRecording()) {
            recorder_->recordLidarScan(scan);
        }
    });

    // Wire IMU callback
    sensor_manager_->setImuCallback([this](const ImuData& imu) {
        if (recorder_ && recorder_->isRecording()) {
            recorder_->recordImu(imu);
        }
    });

    std::cout << "[TELEOP] Sensor callbacks wired to recorder" << std::endl;
}

std::string TeleopRunner::getLocalIP() {
    // Find local IP on the same subnet as the robot
    struct ifaddrs *ifaddr, *ifa;
    char ip[INET_ADDRSTRLEN];
    std::string local_ip;

    if (getifaddrs(&ifaddr) == -1) {
        return "";
    }

    // Parse robot IP to get subnet (assumes /24)
    unsigned long robot_subnet = 0;
    if (!robot_ip_.empty()) {
        struct in_addr addr;
        if (inet_pton(AF_INET, robot_ip_.c_str(), &addr) == 1) {
            robot_subnet = ntohl(addr.s_addr) & 0xFFFFFF00;  // /24 mask
        }
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;

        struct sockaddr_in *sa = (struct sockaddr_in *)ifa->ifa_addr;
        unsigned long this_subnet = ntohl(sa->sin_addr.s_addr) & 0xFFFFFF00;

        if (this_subnet == robot_subnet) {
            inet_ntop(AF_INET, &sa->sin_addr, ip, INET_ADDRSTRLEN);
            local_ip = ip;
            break;
        }
    }

    freeifaddrs(ifaddr);
    return local_ip;
}

pid_t TeleopRunner::startRobotStream() {
    if (robot_ip_.empty()) {
        std::cerr << "[STREAM] No robot IP specified" << std::endl;
        return -1;
    }

    std::string local_ip = getLocalIP();
    if (local_ip.empty()) {
        std::cerr << "[STREAM] Could not determine local IP on robot subnet" << std::endl;
        std::cerr << "[STREAM] Make sure you're connected to the robot network" << std::endl;
        return -1;
    }

    std::cout << "[STREAM] Local IP: " << local_ip << std::endl;
    std::cout << "[STREAM] Starting video stream from robot " << robot_ip_ << "..." << std::endl;

    pid_t pid = fork();
    if (pid == -1) {
        std::cerr << "[STREAM] Fork failed" << std::endl;
        return -1;
    }

    if (pid == 0) {
        // Child process - exec SSH command
        // NOTE: Don't redirect stdout/stderr so SSH can prompt for password if needed
        // (this helps diagnose auth issues)

        // Build GStreamer command
        std::string gst_cmd =
            "gst-launch-1.0 v4l2src device=/dev/video2 ! "
            "\"video/x-raw,format=UYVY,width=640,height=480,framerate=30/1\" ! "
            "videoconvert ! x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast ! "
            "rtph264pay ! udpsink host=" + local_ip + " port=5000";

        execlp("ssh", "ssh",
               "-o", "StrictHostKeyChecking=no",
               "-o", "UserKnownHostsFile=/dev/null",
               "-o", "ConnectTimeout=5",
               "-o", "BatchMode=yes",  // Fail immediately if password is required
               "-o", "ServerAliveInterval=30",
               (std::string("unitree@") + robot_ip_).c_str(),
               gst_cmd.c_str(),
               nullptr);

        // If exec fails
        _exit(1);
    }

    // Parent process
    stream_pid_ = pid;
    std::cout << "[STREAM] Started stream process (PID: " << pid << ")" << std::endl;

    // Give SSH time to connect and start streaming
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    // Check if child is still running
    int status;
    pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == pid) {
        // Child exited already - connection failed
        if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
            std::cerr << "[STREAM] SSH connection failed (exit code: " << WEXITSTATUS(status) << ")" << std::endl;
            std::cerr << "[STREAM] Make sure SSH key is set up:" << std::endl;
            std::cerr << "[STREAM]   ssh-copy-id unitree@" << robot_ip_ << std::endl;
            std::cerr << "[STREAM] Or test manually:" << std::endl;
            std::cerr << "[STREAM]   ssh unitree@" << robot_ip_ << " 'echo success'" << std::endl;
        } else {
            std::cerr << "[STREAM] Failed to start stream" << std::endl;
        }
        stream_pid_ = -1;
        return -1;
    }

    std::cout << "[STREAM] Stream process running" << std::endl;
    return pid;
}

void TeleopRunner::stopRobotStream() {
    if (stream_pid_ <= 0) return;

    std::cout << "[STREAM] Stopping video stream..." << std::endl;

    // Send SIGTERM to the SSH process
    kill(stream_pid_, SIGTERM);

    // Wait for it to exit (with timeout)
    for (int i = 0; i < 20; i++) {  // 2 second timeout
        int status;
        pid_t result = waitpid(stream_pid_, &status, WNOHANG);
        if (result == stream_pid_) {
            std::cout << "[STREAM] Stream stopped" << std::endl;
            stream_pid_ = -1;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Force kill if still running
    std::cout << "[STREAM] Force killing stream process..." << std::endl;
    kill(stream_pid_, SIGKILL);
    waitpid(stream_pid_, nullptr, 0);
    stream_pid_ = -1;
}
