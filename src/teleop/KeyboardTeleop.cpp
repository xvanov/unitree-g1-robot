#include "teleop/KeyboardTeleop.h"
#include "teleop/DDSVideoClient.h"
#include "depth/DepthStreamServer.h"  // For DepthStreamClient
#include "webcam/WebcamStreamServer.h"  // For WebcamStreamClient
#include "sensors/SensorManager.h"
#include "locomotion/LocoController.h"
#include "recording/SensorRecorder.h"
#include "slam/SlamVisualizer.h"
#include "slam/GridMapper.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>

KeyboardTeleop::KeyboardTeleop(SensorManager* sensors, LocoController* loco,
                               SensorRecorder* recorder)
    : sensors_(sensors)
    , loco_(loco)
    , recorder_(recorder)
{
}

KeyboardTeleop::~KeyboardTeleop() {
    if (webcam_client_) {
        webcam_client_->stop();
    }
    if (depth_client_) {
        depth_client_->stop();
    }
    if (dds_video_) {
        dds_video_->stop();
    }
    if (camera_.isOpened()) {
        camera_.release();
    }
    cv::destroyAllWindows();
}

void KeyboardTeleop::run() {
    running_ = true;

    // Initialize depth stream FIRST if enabled (it provides both color + depth)
    // This must happen before DDS video since they compete for the same camera
    if (depth_port_ > 0) {
        initDepthStream();
    }

    // Initialize webcam stream if enabled
    if (webcam_port_ > 0) {
        initWebcamStream();
    }

    // Initialize video source (will be used as fallback if depth not available)
    if (!depth_available_) {
        initVideoSource();
    }

    // Create window
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

    // Initialize SLAM visualizer if enabled (Story 3-1)
    if (visualize_slam_ && grid_mapper_) {
        slam_viz_ = std::make_unique<SlamVisualizer>(800, 600);
        std::cout << "[TELEOP] SLAM visualizer initialized" << std::endl;

        // Wire LiDAR callback to:
        // 1. Update GridMapper with new scan
        // 2. Cache scan for visualizer rendering
        // Note: This is the ONLY place we set the LiDAR callback to avoid overwrites
        if (sensors_) {
            sensors_->setLidarCallback([this](const LidarScan& scan) {
                // Update GridMapper with new scan data
                if (grid_mapper_) {
                    Pose2D pose = sensors_->getEstimatedPose();
                    grid_mapper_->update(pose, scan);
                }
                // Cache scan for visualizer
                latest_scan_ = scan;
                has_new_scan_ = true;
            });
        }
    }

    std::cout << "[TELEOP] Keyboard teleop started" << std::endl;
    std::cout << "[TELEOP] Controls:" << std::endl;
    std::cout << "  U       - Stand up" << std::endl;
    std::cout << "  J       - Sit down (damp)" << std::endl;
    std::cout << "  W/S     - Forward/Backward" << std::endl;
    std::cout << "  A/D     - Strafe Left/Right" << std::endl;
    std::cout << "  Z/E     - Rotate Left/Right" << std::endl;
    std::cout << "  SPACE   - Emergency stop (zero velocity)" << std::endl;
    std::cout << "  R       - Toggle recording" << std::endl;
    std::cout << "  +/-     - Adjust max speed" << std::endl;
    std::cout << "  ESC/Q   - Quit" << std::endl;

    auto last_update = std::chrono::steady_clock::now();
    const auto target_frame_time = std::chrono::milliseconds(33);  // ~30 FPS

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update);

        // Update camera and display
        cv::Mat frame;
        if (!updateCamera()) {
            // Create test pattern if no camera
            frame = cv::Mat(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(frame, "No Camera", cv::Point(240, 240),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 200), 2);
        } else {
            frame = last_frame_.clone();
        }

        // Draw overlay
        drawOverlay(frame);

        // Create display frame - side by side for all available streams
        cv::Mat display_frame;
        std::vector<cv::Mat> frames_to_concat;

        // Add RGB frame (with overlay already applied)
        frames_to_concat.push_back(frame);

        // Add depth if available
        if (depth_available_ && !last_depth_color_.empty()) {
            cv::Mat depth_resized;
            if (last_depth_color_.rows != frame.rows) {
                double scale = static_cast<double>(frame.rows) / last_depth_color_.rows;
                cv::resize(last_depth_color_, depth_resized, cv::Size(), scale, scale);
            } else {
                depth_resized = last_depth_color_.clone();
            }
            cv::putText(depth_resized, "DEPTH", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            frames_to_concat.push_back(depth_resized);
        }

        // Add webcam if available
        if (webcam_available_ && !last_webcam_.empty()) {
            cv::Mat webcam_resized;
            if (last_webcam_.rows != frame.rows) {
                double scale = static_cast<double>(frame.rows) / last_webcam_.rows;
                cv::resize(last_webcam_, webcam_resized, cv::Size(), scale, scale);
            } else {
                webcam_resized = last_webcam_.clone();
            }
            cv::putText(webcam_resized, "WEBCAM", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            frames_to_concat.push_back(webcam_resized);
        }

        // Concatenate all frames side by side
        if (frames_to_concat.size() == 1) {
            display_frame = frames_to_concat[0];
        } else {
            cv::hconcat(frames_to_concat, display_frame);
        }

        // Show frame
        cv::imshow(WINDOW_NAME, display_frame);

        // Update SLAM visualizer if enabled (Story 3-1)
        if (slam_viz_ && grid_mapper_) {
            Pose2D pose = sensors_ ? sensors_->getEstimatedPose() : Pose2D{0, 0, 0};
            slam_viz_->update(*grid_mapper_, pose, has_new_scan_ ? &latest_scan_ : nullptr);
            has_new_scan_ = false;
            slam_viz_->render();

            // Check if SLAM visualizer was closed
            if (slam_viz_->shouldClose()) {
                break;  // User quit via SLAM window
            }
        }

        // Process keyboard input (wait 1ms for key)
        int key = cv::waitKey(1);
        if (key != -1 && !processKey(key)) {
            break;  // User quit
        }

        // Also let SLAM visualizer process keys
        if (slam_viz_) {
            slam_viz_->processKey(key);
        }

        // Send velocity command at ~50Hz
        if (elapsed > target_frame_time) {
            sendVelocity();
            last_update = now;
        }
    }

    // Stop robot on exit
    if (loco_) {
        std::cout << "[TELEOP] Stopping robot..." << std::endl;
        loco_->stop();
    }

    // Stop recording if active
    if (recorder_ && is_recording_) {
        std::cout << "[TELEOP] Stopping recording..." << std::endl;
        recorder_->stopRecording();
        is_recording_ = false;
    }

    cv::destroyAllWindows();
    std::cout << "[TELEOP] Keyboard teleop ended" << std::endl;
}

bool KeyboardTeleop::processKey(int key) {
    // Handle special keys (different keycodes on different platforms)
    key = key & 0xFF;  // Mask to handle extended keycodes

    switch (key) {
        case 27:  // ESC
        case 'q':
        case 'Q':
            return false;  // Quit

        case 'w':
        case 'W':
            cmd_vx_ = std::min(cmd_vx_ + VEL_INCREMENT, SafetyLimits::MAX_VX);
            break;

        case 's':
        case 'S':
            cmd_vx_ = std::max(cmd_vx_ - VEL_INCREMENT, -SafetyLimits::MAX_VX);
            break;

        case 'a':
        case 'A':
            cmd_vy_ = std::min(cmd_vy_ + VEL_INCREMENT, SafetyLimits::MAX_VY);
            break;

        case 'd':
        case 'D':
            cmd_vy_ = std::max(cmd_vy_ - VEL_INCREMENT, -SafetyLimits::MAX_VY);
            break;

        // Note: 'q' is quit, so we use 'z' for rotate left
        case 'z':
        case 'Z':  // Rotate left (counter-clockwise)
            cmd_omega_ = std::min(cmd_omega_ + OMEGA_INCREMENT, SafetyLimits::MAX_OMEGA);
            break;

        case 'e':
        case 'E':  // Rotate right (clockwise)
            cmd_omega_ = std::max(cmd_omega_ - OMEGA_INCREMENT, -SafetyLimits::MAX_OMEGA);
            break;

        case ' ':  // Space - emergency stop
            cmd_vx_ = 0.0f;
            cmd_vy_ = 0.0f;
            cmd_omega_ = 0.0f;
            if (loco_) {
                loco_->stop();
            }
            std::cout << "[TELEOP] Emergency stop - velocities zeroed" << std::endl;
            break;

        case 'r':
        case 'R':  // Toggle recording
            if (recorder_) {
                if (!is_recording_) {
                    std::string session_id = "teleop_" + std::to_string(std::time(nullptr));
                    if (recorder_->startRecording(session_id)) {
                        is_recording_ = true;
                        recording_start_ = std::chrono::steady_clock::now();
                        std::cout << "[TELEOP] Recording started: " << session_id << std::endl;
                    } else {
                        std::cerr << "[TELEOP] Failed to start recording" << std::endl;
                    }
                } else {
                    recorder_->stopRecording();
                    is_recording_ = false;
                    std::cout << "[TELEOP] Recording stopped" << std::endl;
                }
            } else {
                std::cout << "[TELEOP] Recording not available (no recorder)" << std::endl;
            }
            break;

        case '+':
        case '=':  // Increase speed scale
            velocity_scale_ = std::min(velocity_scale_ + 0.1f, 1.0f);
            std::cout << "[TELEOP] Speed scale: " << static_cast<int>(velocity_scale_ * 100) << "%" << std::endl;
            break;

        case '-':
        case '_':  // Decrease speed scale
            velocity_scale_ = std::max(velocity_scale_ - 0.1f, 0.1f);
            std::cout << "[TELEOP] Speed scale: " << static_cast<int>(velocity_scale_ * 100) << "%" << std::endl;
            break;

        case 'u':
        case 'U':  // Stand up
            if (loco_) {
                std::cout << "[TELEOP] Standing up..." << std::endl;
                loco_->standUp();
            }
            break;

        case 'j':
        case 'J':  // Sit down (damp)
            if (loco_) {
                std::cout << "[TELEOP] Sitting down..." << std::endl;
                loco_->sitDown();
            }
            break;

        default:
            // Ignore other keys
            break;
    }

    return true;  // Continue
}

bool KeyboardTeleop::initVideoSource() {
    std::cout << "[TELEOP] Video source: ";

    switch (video_source_) {
        case VideoSource::DDS:
            std::cout << "DDS (recommended)" << std::endl;
            return initDDSVideo();

        case VideoSource::GSTREAMER:
            std::cout << "GStreamer/SSH (legacy)" << std::endl;
            return initGStreamerVideo();

        case VideoSource::LOCAL:
            std::cout << "Local camera" << std::endl;
            return initLocalCamera();

        case VideoSource::NONE:
            std::cout << "None (no video)" << std::endl;
            camera_available_ = false;
            return true;
    }

    return false;
}

bool KeyboardTeleop::initDDSVideo() {
    if (network_interface_.empty()) {
        std::cerr << "[TELEOP] No network interface specified for DDS video" << std::endl;
        std::cerr << "[TELEOP] Falling back to GStreamer..." << std::endl;
        return initGStreamerVideo();
    }

    dds_video_ = std::make_unique<DDSVideoClient>();
    if (!dds_video_->init(network_interface_)) {
        std::cerr << "[TELEOP] Failed to initialize DDS video client" << std::endl;
        std::cerr << "[TELEOP] Falling back to GStreamer..." << std::endl;
        dds_video_.reset();
        return initGStreamerVideo();
    }

    dds_video_->start();

    // Wait briefly for first frame
    std::cout << "[TELEOP] Waiting for DDS video stream..." << std::endl;
    for (int i = 0; i < 30; i++) {  // Wait up to 3 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cv::Mat frame = dds_video_->getLatestFrame();
        if (!frame.empty()) {
            camera_available_ = true;
            std::cout << "[TELEOP] DDS video connected: " << frame.cols << "x" << frame.rows << std::endl;
            return true;
        }
    }

    std::cerr << "[TELEOP] Timeout waiting for DDS video" << std::endl;
    std::cerr << "[TELEOP] Is videohub service running on robot?" << std::endl;
    // Keep DDS client running - it might connect later
    camera_available_ = false;
    return true;  // Not a fatal error
}

bool KeyboardTeleop::initGStreamerVideo() {
    if (!gstreamer_pipeline_.empty()) {
        // Use custom GStreamer pipeline
        std::cout << "[TELEOP] Opening GStreamer pipeline: " << gstreamer_pipeline_ << std::endl;
        camera_.open(gstreamer_pipeline_, cv::CAP_GSTREAMER);
        if (camera_.isOpened()) {
            camera_available_ = true;
            std::cout << "[TELEOP] GStreamer pipeline opened successfully" << std::endl;
            return true;
        }
        std::cerr << "[TELEOP] Failed to open GStreamer pipeline" << std::endl;
    } else if (!robot_ip_.empty()) {
        // Use default UDP stream from robot (H264 on port 5000)
        std::string pipeline = "udpsrc port=5000 timeout=5000000000 ! "
                              "application/x-rtp,encoding-name=H264,payload=96 ! "
                              "rtph264depay ! h264parse ! avdec_h264 ! "
                              "videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false";
        std::cout << "[TELEOP] Opening UDP stream from robot at " << robot_ip_ << std::endl;
        camera_.open(pipeline, cv::CAP_GSTREAMER);
        if (camera_.isOpened()) {
            camera_available_ = true;
            std::cout << "[TELEOP] UDP stream opened successfully" << std::endl;
            return true;
        }
        std::cerr << "[TELEOP] Failed to open UDP stream" << std::endl;
    }

    // Fall back to local camera
    return initLocalCamera();
}

bool KeyboardTeleop::initLocalCamera() {
    camera_.open(camera_index_);
    if (camera_.isOpened()) {
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        camera_available_ = true;
        std::cout << "[TELEOP] Camera opened at index " << camera_index_ << std::endl;
        return true;
    }

    // Try alternative indices
    for (int alt_idx : {1, 2, 0}) {
        if (alt_idx == camera_index_) continue;
        camera_.open(alt_idx);
        if (camera_.isOpened()) {
            camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            camera_available_ = true;
            std::cout << "[TELEOP] Camera opened at alternate index " << alt_idx << std::endl;
            return true;
        }
    }

    std::cout << "[TELEOP] Warning: No camera found, using test pattern" << std::endl;
    camera_available_ = false;
    return true;  // Not a fatal error
}

bool KeyboardTeleop::initDepthStream() {
    if (depth_port_ <= 0) {
        return false;
    }

    std::cout << "[TELEOP] Initializing depth stream on port " << depth_port_ << std::endl;

    depth_client_ = std::make_unique<DepthStreamClient>();
    if (!depth_client_->start(depth_port_)) {
        std::cerr << "[TELEOP] Failed to start depth client on port " << depth_port_ << std::endl;
        depth_client_.reset();
        return false;
    }

    // Wait briefly for first frame
    std::cout << "[TELEOP] Waiting for depth stream..." << std::endl;
    for (int i = 0; i < 30; i++) {  // Wait up to 3 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto frame = depth_client_->getLatestFrame();
        if (frame.valid) {
            depth_available_ = true;
            std::cout << "[TELEOP] Depth stream connected: " << frame.width << "x" << frame.height
                      << " (fx=" << frame.fx << ", scale=" << frame.depth_scale << ")" << std::endl;
            return true;
        }
    }

    std::cout << "[TELEOP] Timeout waiting for depth stream" << std::endl;
    std::cout << "[TELEOP] Is depth_stream_server running on robot?" << std::endl;
    std::cout << "[TELEOP]   ssh unitree@<robot_ip> 'cd ~/g1_inspector/build && ./depth_stream_server <your_ip> " << depth_port_ << "'" << std::endl;
    // Keep client running - it might connect later
    depth_available_ = false;
    return true;  // Not a fatal error
}

bool KeyboardTeleop::initWebcamStream() {
    if (webcam_port_ <= 0) {
        return false;
    }

    std::cout << "[TELEOP] Initializing webcam stream on port " << webcam_port_ << std::endl;

    webcam_client_ = std::make_unique<WebcamStreamClient>();
    if (!webcam_client_->start(webcam_port_)) {
        std::cerr << "[TELEOP] Failed to start webcam client on port " << webcam_port_ << std::endl;
        webcam_client_.reset();
        return false;
    }

    // Wait briefly for first frame
    std::cout << "[TELEOP] Waiting for webcam stream..." << std::endl;
    for (int i = 0; i < 30; i++) {  // Wait up to 3 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto frame = webcam_client_->getLatestFrame();
        if (frame.valid) {
            webcam_available_ = true;
            std::cout << "[TELEOP] Webcam stream connected: " << frame.image.cols << "x" << frame.image.rows << std::endl;
            return true;
        }
    }

    std::cout << "[TELEOP] Timeout waiting for webcam stream" << std::endl;
    std::cout << "[TELEOP] Is webcam-stream.service running on robot?" << std::endl;
    // Keep client running - it might connect later
    webcam_available_ = false;
    return true;  // Not a fatal error
}

bool KeyboardTeleop::updateCamera() {
    bool got_video = false;

    // Try depth client first if available (provides both color + depth)
    if (depth_client_) {
        auto depth_frame = depth_client_->getLatestFrame();
        if (depth_frame.valid) {
            // Use color from depth stream as primary video
            last_frame_ = depth_frame.color.clone();
            last_depth_ = depth_frame.depth.clone();

            // Store intrinsics
            depth_fx_ = depth_frame.fx;
            depth_fy_ = depth_frame.fy;
            depth_cx_ = depth_frame.cx;
            depth_cy_ = depth_frame.cy;
            depth_scale_ = depth_frame.depth_scale;

            // Colorize depth for display (0-5m range)
            cv::Mat depth_normalized;
            double max_depth_mm = 5000.0;  // 5 meters
            last_depth_.convertTo(depth_normalized, CV_8U, 255.0 / max_depth_mm);
            cv::applyColorMap(depth_normalized, last_depth_color_, cv::COLORMAP_JET);

            camera_available_ = true;
            depth_available_ = true;
            got_video = true;

            // Record depth frame if recording is active
            if (recorder_ && recorder_->isRecording()) {
                // Encode color to JPEG
                std::vector<uint8_t> color_jpeg;
                std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 85};
                cv::imencode(".jpg", depth_frame.color, color_jpeg, jpeg_params);

                // Encode depth to PNG (lossless 16-bit)
                std::vector<uint8_t> depth_png;
                std::vector<int> png_params = {cv::IMWRITE_PNG_COMPRESSION, 3};
                cv::imencode(".png", depth_frame.depth, depth_png, png_params);

                recorder_->recordDepthFrame(color_jpeg, depth_png,
                                           depth_fx_, depth_fy_, depth_cx_, depth_cy_,
                                           depth_scale_);

                // Also record as video frame for compatibility
                recorder_->recordVideoFrame(color_jpeg);
            }
        }
    }

    // Try DDS video if we didn't get video from depth stream
    if (!got_video && dds_video_ && dds_video_->isRunning()) {
        cv::Mat frame = dds_video_->getLatestFrame();
        if (!frame.empty()) {
            last_frame_ = frame;
            last_jpeg_ = dds_video_->getLatestJpegData();
            camera_available_ = true;

            // Record video frame if recording is active
            if (recorder_ && recorder_->isRecording() && !last_jpeg_.empty()) {
                recorder_->recordVideoFrame(last_jpeg_);
            }

            got_video = true;
        }
    }

    // Fall back to OpenCV camera
    if (!got_video && camera_.isOpened()) {
        cv::Mat frame;
        camera_ >> frame;
        if (!frame.empty()) {
            last_frame_ = frame;
            last_jpeg_.clear();  // No JPEG data from GStreamer

            // Record video frame if recording is active (will encode to JPEG)
            if (recorder_ && recorder_->isRecording()) {
                recorder_->recordVideoFrame({}, frame);
            }

            got_video = true;
        }
    }

    // Update webcam stream (independent of other video sources)
    if (webcam_client_) {
        auto webcam_frame = webcam_client_->getLatestFrame();
        if (webcam_frame.valid && !webcam_frame.image.empty()) {
            last_webcam_ = webcam_frame.image.clone();
            webcam_available_ = true;
        }
    }

    return got_video;
}

void KeyboardTeleop::drawOverlay(cv::Mat& frame) {
    // Colors
    const cv::Scalar white(255, 255, 255);
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar red(0, 0, 255);
    const cv::Scalar yellow(0, 255, 255);

    // Semi-transparent overlay background for text areas
    cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.cols, 40),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(frame, cv::Point(0, frame.rows - 80), cv::Point(frame.cols, frame.rows),
                  cv::Scalar(0, 0, 0), cv::FILLED);

    // Top bar: Recording status
    int y_top = 25;
    if (is_recording_) {
        // Recording indicator
        cv::circle(frame, cv::Point(20, y_top - 5), 8, red, cv::FILLED);

        auto now = std::chrono::steady_clock::now();
        float duration = std::chrono::duration<float>(now - recording_start_).count();
        std::string duration_str = formatDuration(duration);

        std::string size_str = "0 B";
        std::string frames_str = "";
        if (recorder_) {
            auto stats = recorder_->getStats();
            size_str = formatSize(stats.bytes_compressed);
            if (stats.video_frame_count > 0) {
                frames_str = "  " + std::to_string(stats.video_frame_count) + " frames";
            }
        }

        std::string recording_text = "[RECORDING] " + duration_str + "  " + size_str + frames_str;
        cv::putText(frame, recording_text, cv::Point(40, y_top),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, red, 2);
    } else {
        cv::putText(frame, "Press R to record", cv::Point(10, y_top),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, white, 1);
    }

    // Bottom section: Velocity commands and status
    int y_bottom = frame.rows - 55;

    // Velocity display
    std::ostringstream vel_ss;
    vel_ss << std::fixed << std::setprecision(2)
           << "vx: " << cmd_vx_ * velocity_scale_
           << "   vy: " << cmd_vy_ * velocity_scale_
           << "   w: " << cmd_omega_ * velocity_scale_;
    cv::putText(frame, vel_ss.str(), cv::Point(10, y_bottom),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, green, 2);

    // Battery and FSM state
    // NOTE: G1 LowState_ doesn't include battery - requires separate BMS topic subscription
    // getBatteryPercent() returns 0 on G1 until BMS topic is implemented
    y_bottom += 25;
    float battery = sensors_ ? sensors_->getBatteryPercent() : 0.0f;
    int fsm_state = loco_ ? loco_->getState() : 0;

    std::string fsm_name;
    switch (fsm_state) {
        case G1FSM::ZERO_TORQUE: fsm_name = "ZERO_TORQUE"; break;
        case G1FSM::DAMP: fsm_name = "DAMP"; break;
        case G1FSM::SQUAT: fsm_name = "SQUAT"; break;
        case G1FSM::SIT: fsm_name = "SIT"; break;
        case G1FSM::STAND_UP: fsm_name = "STAND_UP"; break;
        case G1FSM::START: fsm_name = "START"; break;
        case G1FSM::WALKING: fsm_name = "WALKING"; break;
        case G1FSM::AI_MODE: fsm_name = "AI_MODE"; break;
        default: fsm_name = "UNKNOWN(" + std::to_string(fsm_state) + ")"; break;
    }

    std::ostringstream status_ss;
    status_ss << "Battery: " << static_cast<int>(battery) << "%    "
              << "FSM: " << fsm_name << "    "
              << "Scale: " << static_cast<int>(velocity_scale_ * 100) << "%";
    cv::putText(frame, status_ss.str(), cv::Point(10, y_bottom),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, white, 1);

    // Help text
    y_bottom += 22;
    cv::putText(frame, "Press Q to quit, R to record, SPACE to stop",
               cv::Point(10, y_bottom), cv::FONT_HERSHEY_SIMPLEX, 0.45, yellow, 1);
}

void KeyboardTeleop::sendVelocity() {
    if (!loco_) {
        return;
    }

    // Apply velocity scale
    float vx = cmd_vx_ * velocity_scale_;
    float vy = cmd_vy_ * velocity_scale_;
    float omega = cmd_omega_ * velocity_scale_;

    // Send to robot
    loco_->setVelocity(vx, vy, omega);
}

cv::Mat KeyboardTeleop::getLastFrame() const {
    return last_frame_.clone();
}

std::string KeyboardTeleop::formatDuration(float seconds) const {
    int total_seconds = static_cast<int>(seconds);
    int hours = total_seconds / 3600;
    int minutes = (total_seconds % 3600) / 60;
    int secs = total_seconds % 60;

    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(2) << hours << ":"
       << std::setfill('0') << std::setw(2) << minutes << ":"
       << std::setfill('0') << std::setw(2) << secs;
    return ss.str();
}

std::string KeyboardTeleop::formatSize(uint64_t bytes) const {
    if (bytes < 1024) {
        return std::to_string(bytes) + " B";
    } else if (bytes < 1024 * 1024) {
        return std::to_string(bytes / 1024) + " KB";
    } else if (bytes < 1024 * 1024 * 1024) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << (bytes / (1024.0 * 1024.0)) << " MB";
        return ss.str();
    } else {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << (bytes / (1024.0 * 1024.0 * 1024.0)) << " GB";
        return ss.str();
    }
}
