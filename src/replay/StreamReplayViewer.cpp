#include "replay/StreamReplayViewer.h"

#include <iostream>
#include <cmath>
#include <algorithm>

namespace replay {

StreamReplayViewer::StreamReplayViewer() {
    // Default configuration set in header
}

StreamReplayViewer::~StreamReplayViewer() {
    cv::destroyAllWindows();
}

bool StreamReplayViewer::init(const std::string& recording_path) {
    // Create replayer
    replayer_ = std::make_unique<SensorReplayer>();
    if (!replayer_->open(recording_path)) {
        std::cerr << "[STREAM REPLAY] Failed to open: " << recording_path << std::endl;
        return false;
    }

    // Create controller
    controller_ = std::make_unique<ReplayController>();
    controller_->setTotalTime(replayer_->getDuration());
    controller_->setSpeed(initial_speed_);

    // GridMapper: 200x200 at 0.05m = 10m x 10m
    mapper_ = std::make_unique<GridMapper>(0.05f, 200, 200);

    // SlamVisualizer: 640x480 window
    slam_viz_ = std::make_unique<SlamVisualizer>(DISPLAY_WIDTH, DISPLAY_HEIGHT);

    // Pre-allocate display buffers (avoid per-frame allocation)
    rgb_display_ = cv::Mat(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);
    webcam_display_ = cv::Mat(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);
    depth_display_ = cv::Mat(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);
    status_canvas_ = cv::Mat(STATUS_HEIGHT, DISPLAY_WIDTH, CV_8UC3);

    // Load and validate metadata
    auto meta = replayer_->getMetadata();
    std::cout << "[STREAM REPLAY] Loaded: " << meta.session_id << std::endl;
    std::cout << "  Duration: " << meta.duration_seconds << "s" << std::endl;
    std::cout << "  Video frames: " << meta.video_frame_count << std::endl;
    std::cout << "  Webcam images: " << meta.image_count << std::endl;
    std::cout << "  Depth frames: " << meta.depth_frame_count << std::endl;
    std::cout << "  LiDAR scans: " << meta.lidar_count << std::endl;
    std::cout << "  3D Point clouds: " << meta.pointcloud_count << std::endl;

    // Check for webcam data (IMAGE messages stored in images/ folder)
    if (meta.image_count > 0) {
        has_webcam_data_ = true;
        std::cout << "[STREAM REPLAY] Webcam data available (" << meta.image_count << " frames)" << std::endl;
    }

    // Validate recording has expected data types
    if (meta.video_frame_count == 0 && meta.depth_frame_count == 0) {
        std::cerr << "[STREAM REPLAY] Warning: No video/depth frames in recording" << std::endl;
        std::cerr << "  RGB window will show placeholder" << std::endl;
    }
    if (meta.lidar_count == 0) {
        std::cerr << "[STREAM REPLAY] Warning: No LiDAR data - SLAM map will not build" << std::endl;
    }

    // Decide whether to use DEPTH_FRAME color for RGB
    if (meta.video_frame_count == 0 && meta.depth_frame_count > 0) {
        use_depth_for_rgb_ = true;
        std::cout << "[STREAM REPLAY] Using DEPTH_FRAME color for RGB window" << std::endl;
    }

    return true;
}

void StreamReplayViewer::createWindows() {
    cv::namedWindow(WINDOW_RGB, cv::WINDOW_AUTOSIZE);
    if (has_webcam_data_) {
        cv::namedWindow(WINDOW_WEBCAM, cv::WINDOW_AUTOSIZE);
    }
    cv::namedWindow(WINDOW_DEPTH, cv::WINDOW_AUTOSIZE);
    // Note: SlamVisualizer creates its own window "SLAM Visualizer"
    cv::namedWindow(WINDOW_STATUS, cv::WINDOW_AUTOSIZE);

    // Position windows in 3x2 or 2x2 grid layout
    // Note: cv::moveWindow may not work on all window managers (e.g., Wayland)
    // Users on Wayland may need to manually arrange windows
    if (layout_ == WindowLayout::GRID_2X2) {
        // With webcam: 3 columns top row, 2 columns bottom row
        // Without webcam: 2x2 grid
        cv::moveWindow(WINDOW_RGB, 0, 0);
        if (has_webcam_data_) {
            cv::moveWindow(WINDOW_WEBCAM, DISPLAY_WIDTH + 20, 0);
            cv::moveWindow(WINDOW_DEPTH, 2 * (DISPLAY_WIDTH + 20), 0);
            cv::moveWindow("SLAM Visualizer", 0, DISPLAY_HEIGHT + 40);
            cv::moveWindow(WINDOW_STATUS, DISPLAY_WIDTH + 20, DISPLAY_HEIGHT + 40);
        } else {
            cv::moveWindow(WINDOW_DEPTH, DISPLAY_WIDTH + 20, 0);
            cv::moveWindow("SLAM Visualizer", 0, DISPLAY_HEIGHT + 40);
            cv::moveWindow(WINDOW_STATUS, DISPLAY_WIDTH + 20, DISPLAY_HEIGHT + 40);
        }
    } else {
        // Horizontal strip layout
        cv::moveWindow(WINDOW_RGB, 0, 0);
        int offset = DISPLAY_WIDTH + 10;
        if (has_webcam_data_) {
            cv::moveWindow(WINDOW_WEBCAM, offset, 0);
            offset += DISPLAY_WIDTH + 10;
        }
        cv::moveWindow(WINDOW_DEPTH, offset, 0);
        cv::moveWindow("SLAM Visualizer", offset + DISPLAY_WIDTH + 10, 0);
        cv::moveWindow(WINDOW_STATUS, 0, DISPLAY_HEIGHT + 40);
    }
}

int StreamReplayViewer::run() {
    createWindows();
    controller_->play();

    // Get starting timestamp
    ReplayMessage first_msg;
    if (replayer_->readNext(first_msg)) {
        start_timestamp_ = first_msg.timestamp_us;
        processMessage(first_msg);
    }

    while (true) {
        // Process messages while respecting playback timing
        ReplayMessage msg;
        while (replayer_->hasMore()) {
            if (!replayer_->readNext(msg)) {
                break;
            }

            processMessage(msg);

            // Check if we've processed enough based on playback time
            float playback_time = controller_->getElapsedTime();
            float msg_time = (msg.timestamp_us - start_timestamp_) / 1000000.0f;
            if (msg_time > playback_time * controller_->getSpeed()) {
                break;
            }
        }

        updateAllDisplays();

        int key = cv::waitKey(33);  // ~30 FPS display
        if (!processKey(key)) {
            break;
        }

        // Check if replay finished
        if (!replayer_->hasMore()) {
            if (controller_->isLoopEnabled()) {
                replayer_->seek(0.0f);
                mapper_ = std::make_unique<GridMapper>(0.05f, 200, 200);
                rgb_frame_count_ = 0;
                webcam_frame_count_ = 0;
                depth_frame_count_ = 0;
                lidar_count_ = 0;
                pose_count_ = 0;
                controller_->setElapsedTime(0.0f);
                controller_->play();
            } else {
                // Show final frame, wait for user to quit
                controller_->setFinished();
                updateAllDisplays();
                while (true) {
                    int k = cv::waitKey(100);
                    if (k == 'q' || k == 'Q' || k == 27) {
                        break;
                    }
                }
                break;
            }
        }

        // Update elapsed time based on actual wall clock if playing
        if (controller_->isPlaying()) {
            static auto last_update = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - last_update).count();
            last_update = now;
            float new_time = controller_->getElapsedTime() + dt * controller_->getSpeed();
            if (new_time <= controller_->getTotalTime()) {
                controller_->setElapsedTime(new_time);
            }
        }
    }

    cv::destroyAllWindows();
    return 0;
}

void StreamReplayViewer::processMessage(const ReplayMessage& msg) {
    using recording::MessageType;

    switch (msg.type) {
        case MessageType::VIDEO_FRAME: {
            DecodedVideoFrame decoded;
            if (!SensorReplayer::decodeVideoFrame(msg, decoded)) {
                std::cerr << "[STREAM REPLAY] Failed to decode video frame" << std::endl;
                break;
            }
            latest_rgb_ = cv::imdecode(decoded.jpeg_data, cv::IMREAD_COLOR);
            if (latest_rgb_.empty()) {
                std::cerr << "[STREAM REPLAY] Failed to decode JPEG from video frame" << std::endl;
            }
            latest_rgb_timestamp_ = msg.timestamp_us;
            rgb_frame_count_++;
            break;
        }

        case MessageType::DEPTH_FRAME: {
            DecodedDepthFrame decoded;
            if (!SensorReplayer::decodeDepthFrame(msg, decoded)) {
                std::cerr << "[STREAM REPLAY] Failed to decode depth frame" << std::endl;
                break;
            }
            // Use color_jpeg for RGB window if no VIDEO_FRAME
            if (latest_rgb_.empty() || use_depth_for_rgb_) {
                latest_rgb_ = cv::imdecode(decoded.color_jpeg, cv::IMREAD_COLOR);
                latest_rgb_timestamp_ = msg.timestamp_us;
                if (!use_depth_for_rgb_) {
                    rgb_frame_count_++;  // Count as RGB if we don't have VIDEO_FRAME
                }
            }
            // Decode depth PNG (16-bit grayscale, values in mm)
            cv::Mat depth_16 = cv::imdecode(decoded.depth_png, cv::IMREAD_UNCHANGED);
            if (!depth_16.empty()) {
                latest_depth_raw_ = depth_16;
            }
            latest_depth_timestamp_ = msg.timestamp_us;
            depth_frame_count_++;
            break;
        }

        case MessageType::LIDAR_SCAN: {
            DecodedLidarScan decoded;
            if (!SensorReplayer::decodeLidarScan(msg, decoded)) {
                std::cerr << "[STREAM REPLAY] Failed to decode LiDAR scan" << std::endl;
                break;
            }
            latest_scan_ = decoded.scan;
            if (mapper_) {
                mapper_->update(latest_pose_, latest_scan_);
            }
            latest_lidar_timestamp_ = msg.timestamp_us;
            lidar_count_++;
            break;
        }

        case MessageType::POSE: {
            DecodedPose decoded;
            if (!SensorReplayer::decodePose(msg, decoded)) {
                break;  // Pose decode failures are less critical
            }
            latest_pose_ = decoded.pose;
            latest_pose_timestamp_ = msg.timestamp_us;
            pose_count_++;
            break;
        }

        case MessageType::IMU_DATA: {
            DecodedImuData decoded;
            if (SensorReplayer::decodeImuData(msg, decoded)) {
                latest_imu_ = decoded.imu;
                // IMU could be used for pose interpolation if needed
            }
            break;
        }

        case MessageType::POINT_CLOUD_3D:
            // Store for potential future 3D visualization (Story D-2)
            // For now, skip - SLAM uses 2D LIDAR_SCAN
            break;

        case MessageType::IMAGE: {
            // Webcam frames stored as separate image files
            DecodedImage decoded;
            if (!SensorReplayer::decodeImage(msg, decoded)) {
                break;  // Image decode failures are non-critical
            }
            // Load the image from the images/ subdirectory
            cv::Mat img = loadImageFile(decoded.filename);
            if (!img.empty()) {
                latest_webcam_ = img;
                latest_webcam_timestamp_ = msg.timestamp_us;
                webcam_frame_count_++;
            }
            break;
        }

        case MessageType::TELEOP_CMD:
        case MessageType::MOTOR_STATE:
        case MessageType::METADATA:
            // Ignore these message types for visualization
            break;
    }
}

void StreamReplayViewer::renderRgbWindow() {
    if (latest_rgb_.empty()) {
        // Create placeholder
        cv::Mat placeholder(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3, cv::Scalar(40, 40, 40));
        cv::putText(placeholder, "No RGB Data", cv::Point(220, 240),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 200), 2);
        cv::imshow(WINDOW_RGB, placeholder);
        return;
    }

    // Clone and resize if needed
    cv::Mat display;
    if (latest_rgb_.cols != DISPLAY_WIDTH || latest_rgb_.rows != DISPLAY_HEIGHT) {
        cv::resize(latest_rgb_, display, cv::Size(DISPLAY_WIDTH, DISPLAY_HEIGHT));
    } else {
        display = latest_rgb_.clone();
    }

    // Draw timestamp overlay
    std::string ts = formatTimestamp(controller_->getElapsedTime());
    cv::putText(display, ts, cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    cv::imshow(WINDOW_RGB, display);
}

void StreamReplayViewer::renderWebcamWindow() {
    if (!has_webcam_data_) {
        return;  // No webcam window to render
    }

    if (latest_webcam_.empty()) {
        // Create placeholder
        webcam_display_.setTo(cv::Scalar(40, 40, 40));
        cv::putText(webcam_display_, "No Webcam Data", cv::Point(200, 240),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 200), 2);
        cv::imshow(WINDOW_WEBCAM, webcam_display_);
        return;
    }

    // Clone and resize if needed
    cv::Mat display;
    if (latest_webcam_.cols != DISPLAY_WIDTH || latest_webcam_.rows != DISPLAY_HEIGHT) {
        cv::resize(latest_webcam_, display, cv::Size(DISPLAY_WIDTH, DISPLAY_HEIGHT));
    } else {
        display = latest_webcam_.clone();
    }

    // Draw "Webcam" label
    cv::putText(display, "Webcam", cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

    cv::imshow(WINDOW_WEBCAM, display);
}

cv::Mat StreamReplayViewer::loadImageFile(const std::string& filename) {
    // Images are stored in <recording_dir>/images/<filename>
    std::string path = replayer_->getRecordingDir() + "/images/" + filename;
    cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
    if (img.empty()) {
        // Try without images/ subdirectory (legacy format)
        path = replayer_->getRecordingDir() + "/" + filename;
        img = cv::imread(path, cv::IMREAD_COLOR);
    }
    return img;
}

void StreamReplayViewer::renderDepthWindow() {
    if (latest_depth_raw_.empty()) {
        // Reuse pre-allocated buffer for placeholder
        depth_display_.setTo(cv::Scalar(40, 40, 40));
        cv::putText(depth_display_, "No Depth Data", cv::Point(200, 240),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 200), 2);
        cv::imshow(WINDOW_DEPTH, depth_display_);
        return;
    }

    // Resize depth if needed
    cv::Mat depth_resized;
    if (latest_depth_raw_.cols != DISPLAY_WIDTH || latest_depth_raw_.rows != DISPLAY_HEIGHT) {
        cv::resize(latest_depth_raw_, depth_resized, cv::Size(DISPLAY_WIDTH, DISPLAY_HEIGHT), 0, 0, cv::INTER_NEAREST);
    } else {
        depth_resized = latest_depth_raw_;
    }

    // Convert 16-bit depth to 8-bit for colormap
    // Typical depth range: 0-10000mm (0-10m)
    cv::Mat depth_8bit;
    depth_resized.convertTo(depth_8bit, CV_8UC1, 255.0 / MAX_DEPTH_MM);

    // Apply TURBO colormap (blue=near, red=far)
    cv::applyColorMap(depth_8bit, depth_display_, cv::COLORMAP_TURBO);

    // Mask out invalid pixels (depth == 0) using optimized OpenCV operation
    cv::Mat valid_mask = (depth_resized > 0);
    depth_display_.setTo(cv::Scalar(0, 0, 0), ~valid_mask);

    cv::imshow(WINDOW_DEPTH, depth_display_);
}

void StreamReplayViewer::renderSlamWindow() {
    if (slam_viz_ && mapper_) {
        slam_viz_->update(*mapper_, latest_pose_, &latest_scan_);
        slam_viz_->render();
    }
}

void StreamReplayViewer::renderStatusOverlay() {
    // Reuse pre-allocated status canvas (avoid per-frame allocation)
    status_canvas_.setTo(cv::Scalar(30, 30, 30));

    // Playback state
    std::string state_str;
    auto state = controller_->getState();
    switch (state) {
        case PlaybackState::PLAYING: state_str = "PLAYING"; break;
        case PlaybackState::PAUSED: state_str = "PAUSED"; break;
        case PlaybackState::STOPPED: state_str = "STOPPED"; break;
        case PlaybackState::FINISHED: state_str = "FINISHED"; break;
        case PlaybackState::SEEKING: state_str = "SEEKING"; break;
    }

    // Time display
    float elapsed = controller_->getElapsedTime();
    float total = controller_->getTotalTime();
    std::string time_str = formatTimestamp(elapsed) + " / " + formatTimestamp(total);

    // Speed display
    char speed_str[32];
    snprintf(speed_str, sizeof(speed_str), "Speed: %.2fx", controller_->getSpeed());

    // Frame counts
    char counts_str[160];
    if (has_webcam_data_) {
        snprintf(counts_str, sizeof(counts_str),
                "RGB: %d | Webcam: %d | Depth: %d | LiDAR: %d",
                rgb_frame_count_, webcam_frame_count_, depth_frame_count_, lidar_count_);
    } else {
        snprintf(counts_str, sizeof(counts_str),
                "RGB: %d | Depth: %d | LiDAR: %d | Pose: %d",
                rgb_frame_count_, depth_frame_count_, lidar_count_, pose_count_);
    }

    // Progress bar
    float progress = (total > 0) ? (elapsed / total) : 0.0f;
    cv::rectangle(status_canvas_, cv::Point(BAR_X, BAR_Y),
                 cv::Point(BAR_X + BAR_WIDTH, BAR_Y + BAR_HEIGHT),
                 cv::Scalar(60, 60, 60), -1);
    cv::rectangle(status_canvas_, cv::Point(BAR_X, BAR_Y),
                 cv::Point(BAR_X + static_cast<int>(BAR_WIDTH * progress), BAR_Y + BAR_HEIGHT),
                 cv::Scalar(0, 200, 0), -1);

    // Draw text
    cv::putText(status_canvas_, state_str, cv::Point(20, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    cv::putText(status_canvas_, time_str, cv::Point(200, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(status_canvas_, speed_str, cv::Point(450, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 2);
    cv::putText(status_canvas_, counts_str, cv::Point(20, 70),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(180, 180, 180), 1);

    // Controls help
    cv::putText(status_canvas_, "SPACE: Pause | +/-: Speed | Q: Quit | Arrow: Seek",
               cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5,
               cv::Scalar(150, 150, 150), 1);

    cv::imshow(WINDOW_STATUS, status_canvas_);
}

void StreamReplayViewer::updateAllDisplays() {
    renderRgbWindow();
    renderWebcamWindow();
    renderDepthWindow();
    renderSlamWindow();
    renderStatusOverlay();
}

bool StreamReplayViewer::processKey(int key) {
    if (key == -1) return true;  // No key pressed

    switch (key) {
        case 'q':
        case 'Q':
        case 27:  // ESC
            return false;  // Quit

        case ' ':  // SPACE - pause/resume
            if (controller_->getState() == PlaybackState::PAUSED) {
                controller_->resume();
            } else {
                controller_->pause();
            }
            break;

        case '+':
        case '=':  // Speed up
            {
                float speed = controller_->getSpeed();
                if (speed < 4.0f) {
                    controller_->setSpeed(speed * 2.0f);
                    std::cout << "[REPLAY] Speed: " << controller_->getSpeed() << "x" << std::endl;
                }
            }
            break;

        case '-':
        case '_':  // Slow down
            {
                float speed = controller_->getSpeed();
                if (speed > 0.25f) {
                    controller_->setSpeed(speed / 2.0f);
                    std::cout << "[REPLAY] Speed: " << controller_->getSpeed() << "x" << std::endl;
                }
            }
            break;

        case 0x250000:  // LEFT arrow (OpenCV)
        case 2:         // LEFT arrow (some systems)
            {
                float current = controller_->getElapsedTime();
                float target = std::max(0.0f, current - 5.0f);
                seekTo(target);
            }
            break;

        case 0x270000:  // RIGHT arrow (OpenCV)
        case 3:         // RIGHT arrow (some systems)
            {
                float current = controller_->getElapsedTime();
                float target = std::min(controller_->getTotalTime(), current + 5.0f);
                seekTo(target);
            }
            break;

        case 'l':
        case 'L':  // Toggle LiDAR rays in SLAM visualizer
            if (slam_viz_) {
                slam_viz_->processKey(key);
            }
            break;

        case 'r':
        case 'R':  // Reset view (SLAM) - center on robot
            if (slam_viz_) {
                slam_viz_->centerOnRobot();
            }
            break;
    }
    return true;
}

void StreamReplayViewer::seekTo(float target_time) {
    std::cout << "[REPLAY] Seeking to " << formatTimestamp(target_time) << std::endl;
    controller_->pause();

    // Convert time to progress (0.0-1.0)
    float progress = target_time / controller_->getTotalTime();
    replayer_->seek(progress);

    // Clear map and rebuild
    if (mapper_) {
        mapper_ = std::make_unique<GridMapper>(0.05f, 200, 200);
    }

    // Reset counters since we're re-reading
    rgb_frame_count_ = 0;
    webcam_frame_count_ = 0;
    depth_frame_count_ = 0;
    lidar_count_ = 0;
    pose_count_ = 0;

    // Clear latest data
    latest_rgb_ = cv::Mat();
    latest_webcam_ = cv::Mat();
    latest_depth_raw_ = cv::Mat();
    latest_scan_ = LidarScan();
    latest_pose_ = Pose2D();

    // Update elapsed time
    controller_->setElapsedTime(target_time);
    controller_->resume();

    // Process messages up to the seek target
    ReplayMessage msg;
    start_timestamp_ = 0;
    while (replayer_->hasMore()) {
        if (!replayer_->readNext(msg)) break;
        if (start_timestamp_ == 0) {
            start_timestamp_ = msg.timestamp_us;
        }
        float msg_time = (msg.timestamp_us - start_timestamp_) / 1000000.0f;
        if (msg_time >= target_time) break;
        processMessage(msg);
    }
}

std::string StreamReplayViewer::formatTimestamp(float seconds) const {
    int mins = static_cast<int>(seconds / 60);
    int secs = static_cast<int>(seconds) % 60;
    int ms = static_cast<int>((seconds - static_cast<int>(seconds)) * 1000);
    char buf[32];
    snprintf(buf, sizeof(buf), "%02d:%02d.%03d", mins, secs, ms);
    return std::string(buf);
}

} // namespace replay
