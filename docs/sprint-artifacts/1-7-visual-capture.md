# Story 1.7: Visual Capture

**Status:** Ready for Review

---

## Quick Reference

**New files to create:**
- `src/capture/ImageCapture.h` / `.cpp` - Core image capture class with 1fps capture, pose tagging, and storage
- `src/capture/PlanCorrelator.h` / `.cpp` - Coordinate transforms and FOV coverage tracking
- `test/test_capture.cpp` - Unit tests for image capture and correlation

**Files to modify:**
- `src/main.cpp` - Initialize ImageCapture, wire to inspection loop via background thread
- `CMakeLists.txt` - Add `capture` library with OpenCV and nlohmann_json
- Note: `src/app/StateMachine.cpp` was NOT modified - capture uses main.cpp polling instead

**Key classes:**
| Class | Purpose |
|-------|---------|
| `ImageCapture` | Capture RGB images at 1fps during INSPECTING state, tag with pose metadata, save to disk |
| `PlanCorrelator` | Transform robot pose to plan coordinates, calculate FOV coverage percentage |

**Primary acceptance criteria:** AC1 (1fps capture), AC2 (pose metadata per image), AC3 (images saved to inspection directory), AC4 (coverage tracking)

**Prerequisites:** Story 1-6 (StateMachine, PlanManager) must be complete for state integration and coordinate transforms

---

## Story

As the **robot**,
I want **to capture geotagged images during inspection**,
So that **defects can be detected from photos**.

---

## Acceptance Criteria

1. **AC1:** Images captured at 1fps during INSPECTING state
2. **AC2:** Each image has pose metadata (timestamp, robot pose, camera orientation)
3. **AC3:** Images saved to inspection directory with structured naming
4. **AC4:** Coverage tracked (percentage of plan area photographed)
5. **AC5:** Storage management - one directory per inspection session
6. **AC6:** Graceful handling of camera unavailability
7. **AC7:** Aligned depth capture (optional for MVP - can defer)

---

## Tasks / Subtasks

- [x] **Task 1: Implement ImageCapture Class** (AC: 1, 2, 3, 5, 6)
  - [x] 1.1 Create `src/capture/ImageCapture.h` - image capture interface
    - Constructor: `ImageCapture(ISensorSource*, PlanManager*)`
    - `startCapture(session_id)` - Begin capture session, create directory
    - `stopCapture()` - End capture session, finalize
    - `captureFrame(robot_pose)` - Capture single frame if interval elapsed
    - `setOutputDir(path)` - Set base output directory (default: `data/inspections/`)
    - `setCaptureInterval(seconds)` - Set capture rate (default: 1.0s)
    - `getImageCount()` - Return captured image count
    - `getCapturedImages()` - Return list of captured image metadata
  - [x] 1.2 Create `src/capture/ImageCapture.cpp` - implementation
    - Use OpenCV VideoCapture for camera access OR ISensorSource if camera available via SDK
    - Save JPEG images with quality 85
    - Save JSON metadata sidecar per image
    - Directory structure: `data/inspections/{session_id}/images/`
    - File naming: `img_{NNNN}.jpg`, `img_{NNNN}.json`
    - Handle camera not available gracefully (log warning, continue without capture)

- [x] **Task 2: Implement PlanCorrelator Class** (AC: 4)
  - [x] 2.1 Create `src/capture/PlanCorrelator.h` - coordinate transforms and coverage
    - Constructor: `PlanCorrelator(PlanManager*)`
    - `robotToPlanCoords(robot_pose)` - Transform robot pose to plan coordinates (delegate to PlanManager)
    - `calculateFOV(pose, camera_params)` - Calculate camera field of view polygon on plan
    - `updateCoverage(pose)` - Mark FOV area as covered in coverage map
    - `getCoveragePercent()` - Return percentage of plan area covered
    - `saveCoverageMap(path)` - Save coverage visualization as PNG
  - [x] 2.2 Create `src/capture/PlanCorrelator.cpp` - implementation
    - FOV calculation: assume 60° horizontal, 45° vertical, range 5m
    - Coverage map: same resolution as plan image, mark pixels as visited
    - Coverage percent = visited pixels / free space pixels

- [x] **Task 3: Define Image Metadata Struct** (AC: 2)
  - [x] 3.1 Add `ImageMetadata` struct to `src/capture/ImageCapture.h`
    ```cpp
    struct ImageMetadata {
        std::string image_path;      // Relative path to image file
        int64_t timestamp_ms;        // Unix timestamp in milliseconds
        Pose2D robot_pose;           // Robot position when captured
        Point2D plan_coords;         // Position on plan in meters
        float camera_yaw;            // Camera orientation (radians)
        int sequence_number;         // Image sequence number in session
    };
    ```
  - [x] 3.2 Implement JSON serialization using nlohmann/json macros

- [x] **Task 4: Integrate with StateMachine** (AC: 1)
  - [x] 4.1 Integrate capture with state machine via `src/main.cpp` background thread
    - Note: StateMachine.cpp was NOT modified - capture integration is done via a background
      thread in main.cpp that monitors state transitions and calls ImageCapture accordingly.
    - Start capture when entering INSPECTING state
    - Stop capture when leaving INSPECTING state (PAUSED, BLOCKED, COMPLETE, EMERGENCY_STOP)
    - Resume capture when transitioning from PAUSED to INSPECTING
  - [x] 4.2 Add capture callbacks to StateMachine or wire in main loop

- [x] **Task 5: Main Loop Integration** (AC: 1, 4)
  - [x] 5.1 Modify `src/main.cpp`
    - Initialize ImageCapture with sensor source and plan manager
    - Call `captureFrame()` in main inspection loop (automatic interval enforcement)
    - Pass current robot pose to capture function

- [x] **Task 6: CMake Integration** (AC: 1-7)
  - [x] 6.1 Update `CMakeLists.txt`
    - Add `capture` library with ImageCapture.cpp, PlanCorrelator.cpp
    - Link OpenCV for VideoCapture and image I/O
    - Link nlohmann_json for metadata serialization
    - Add `test_capture` unit test executable

- [x] **Task 7: Unit Tests** (AC: 1, 2, 4)
  - [x] 7.1 Create `test/test_capture.cpp`
    - Test capture interval enforcement (1fps)
    - Test metadata generation and JSON serialization
    - Test coverage calculation
    - Test session directory creation
    - Mock sensor source for testing without camera

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Use ROS2, image_transport, or any ROS dependencies - this is pure C++
- Block the main inspection loop during image save - use async I/O if needed
- Hardcode camera parameters - make configurable
- Create new Types.h structs - use existing Point2D, Pose2D from `src/util/Types.h`
- Ignore state machine state - only capture during INSPECTING

**MUST USE:**
- C++17 standard (`-std=c++17`)
- `<filesystem>` for directory creation (`std::filesystem::create_directories`)
- OpenCV for camera access via `cv::VideoCapture` and image I/O (`cv::imwrite`)
- nlohmann/json for metadata serialization (header-only, already in project)
- Existing types: `Point2D`, `Pose2D` from `src/util/Types.h`
- Existing interfaces: `ISensorSource` from `src/sensors/ISensorSource.h`
- Existing `PlanManager` from Story 1-6 for coordinate transforms
- Existing `StateMachine` from Story 1-6 for state-based capture control
- Async I/O for image saves (std::async or thread queue) - main loop MUST NOT block

### Camera API Decision (CRITICAL)

**For MVP:** Use `cv::VideoCapture(0)` for development/testing with webcam or USB camera.

**For Real G1 Robot:** The G1 uses Intel RealSense D435i. Two options:
1. **If RealSense appears as V4L2 device:** cv::VideoCapture may work directly
2. **If RealSense SDK required:** Add librealsense2 dependency in future story

**Decision for this story:** Implement with cv::VideoCapture. Add `camera_available_` flag for graceful fallback. Real robot camera integration can be refined in Story 1-8+ if cv::VideoCapture doesn't work with G1's camera setup.

**DO NOT:**
- Assume cv::VideoCapture(0) will work on G1 without testing
- Block on librealsense integration for MVP

### ImageCapture Design

```cpp
// src/capture/ImageCapture.h
#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "util/Types.h"
#include "sensors/ISensorSource.h"

class PlanManager;  // Forward declaration

struct ImageMetadata {
    std::string image_path;      // Relative path to image file
    int64_t timestamp_ms;        // Unix timestamp in milliseconds
    Pose2D robot_pose;           // Robot position when captured
    Point2D plan_coords;         // Position on plan in meters
    float camera_yaw;            // Camera orientation (radians)
    int sequence_number;         // Image sequence in session
};

// NOTE: Do NOT use NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE here - it won't work
// because Point2D/Pose2D are nested types. Use manual serialization instead.
// See "JSON Serialization Implementation" section below for the correct pattern.

class ImageCapture {
public:
    ImageCapture(ISensorSource* sensors = nullptr, PlanManager* plan = nullptr);
    ~ImageCapture();

    // Session management
    bool startCapture(const std::string& session_id);
    void stopCapture();
    bool isCapturing() const { return capturing_; }

    // Frame capture (call each loop iteration - enforces interval internally)
    bool captureFrame(const Pose2D& robot_pose);

    // Configuration
    void setOutputDir(const std::string& path) { output_dir_ = path; }
    void setCaptureInterval(float seconds) { capture_interval_s_ = seconds; }
    void setCameraIndex(int index) { camera_index_ = index; }

    // Status
    int getImageCount() const { return image_count_; }
    const std::vector<ImageMetadata>& getCapturedImages() const { return captured_images_; }
    std::string getCurrentSessionDir() const { return session_dir_; }

private:
    bool initCamera();
    void closeCamera();
    void saveImageAsync(const cv::Mat& frame, const Pose2D& pose);  // Non-blocking
    void saveMetadata(const ImageMetadata& meta);

    ISensorSource* sensors_;
    PlanManager* plan_manager_;
    cv::VideoCapture camera_;

    std::string output_dir_ = "data/inspections";
    std::string session_dir_;
    std::string session_id_;
    float capture_interval_s_ = 1.0f;
    int camera_index_ = 0;
    int jpeg_quality_ = 85;

    bool capturing_ = false;
    bool camera_available_ = false;
    int image_count_ = 0;
    std::chrono::steady_clock::time_point last_capture_time_;
    std::vector<ImageMetadata> captured_images_;

    // Async save tracking
    std::vector<std::future<void>> pending_saves_;
};
```

### Camera Access Pattern

```cpp
// In ImageCapture.cpp
#include <filesystem>
#include <future>

bool ImageCapture::startCapture(const std::string& session_id) {
    session_id_ = session_id;
    session_dir_ = output_dir_ + "/" + session_id;

    // CRITICAL: Create directory structure before saving any files
    try {
        std::filesystem::create_directories(session_dir_ + "/images");
        std::cout << "[CAPTURE] Created session directory: " << session_dir_ << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[CAPTURE] Failed to create directory: " << e.what() << std::endl;
        return false;
    }

    if (!initCamera()) {
        std::cout << "[CAPTURE] Warning: Camera not available, capture disabled" << std::endl;
        // Continue anyway - graceful degradation
    }

    capturing_ = true;
    image_count_ = 0;
    captured_images_.clear();
    last_capture_time_ = std::chrono::steady_clock::now();
    return true;
}

bool ImageCapture::initCamera() {
    // Try to open camera
    camera_.open(camera_index_);

    if (!camera_.isOpened()) {
        std::cerr << "[CAPTURE] Warning: Camera " << camera_index_
                  << " not available. Capture disabled." << std::endl;
        camera_available_ = false;
        return false;
    }

    // Set resolution (1080p if supported, fallback to default)
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    // Verify actual resolution
    int width = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "[CAPTURE] Camera initialized: " << width << "x" << height << std::endl;
    camera_available_ = true;
    return true;
}

bool ImageCapture::captureFrame(const Pose2D& robot_pose) {
    if (!capturing_ || !camera_available_) return false;

    // Check capture interval
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - last_capture_time_).count();
    if (elapsed < capture_interval_s_) {
        return false;  // Not time yet
    }

    // Capture frame
    cv::Mat frame;
    camera_ >> frame;
    if (frame.empty()) {
        std::cerr << "[CAPTURE] Warning: Empty frame captured" << std::endl;
        return false;
    }

    // CRITICAL: Save asynchronously to avoid blocking main loop
    // cv::imwrite takes 10-50ms for JPEG encoding - main loop runs at 20Hz (50ms)
    saveImageAsync(frame.clone(), robot_pose);  // Clone to avoid race condition
    last_capture_time_ = now;
    return true;
}

// CRITICAL: Non-blocking save using std::async
void ImageCapture::saveImageAsync(const cv::Mat& frame, const Pose2D& pose) {
    image_count_++;
    int seq = image_count_;

    // Launch async save
    auto future = std::async(std::launch::async, [this, frame, pose, seq]() {
        // Build file paths
        char filename[32];
        std::snprintf(filename, sizeof(filename), "img_%04d.jpg", seq);
        std::string img_path = session_dir_ + "/images/" + filename;

        // Save JPEG with quality setting
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        if (!cv::imwrite(img_path, frame, params)) {
            std::cerr << "[CAPTURE] Failed to save: " << img_path << std::endl;
            return;
        }

        // Build and save metadata
        ImageMetadata meta;
        meta.image_path = std::string("images/") + filename;
        meta.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        meta.robot_pose = pose;
        meta.plan_coords = plan_manager_ ? plan_manager_->robotToPlanCoords(pose) : Point2D{0, 0};
        meta.camera_yaw = pose.theta;
        meta.sequence_number = seq;

        saveMetadata(meta);

        std::cout << "[CAPTURE] Saved: " << filename << std::endl;
    });

    // Track pending saves (optional: cleanup completed futures periodically)
    pending_saves_.push_back(std::move(future));
}

void ImageCapture::stopCapture() {
    capturing_ = false;

    // Wait for all pending saves to complete
    for (auto& future : pending_saves_) {
        if (future.valid()) {
            future.wait();
        }
    }
    pending_saves_.clear();

    closeCamera();
    std::cout << "[CAPTURE] Session stopped. " << image_count_ << " images captured." << std::endl;
}
```

### Directory Structure

```
data/
└── inspections/
    └── insp_001/               # Session directory (session_id)
        ├── images/
        │   ├── img_0001.jpg    # JPEG image
        │   ├── img_0001.json   # Metadata sidecar
        │   ├── img_0002.jpg
        │   ├── img_0002.json
        │   └── ...
        ├── coverage.png        # Coverage visualization
        └── session.json        # Session summary (optional)
```

### Metadata JSON Format

```json
{
    "image_path": "images/img_0001.jpg",
    "timestamp_ms": 1733356800000,
    "robot_pose": {
        "x": 1.234,
        "y": 2.567,
        "theta": 0.785
    },
    "plan_coords": {
        "x": 5.234,
        "y": 6.567
    },
    "camera_yaw": 0.785,
    "sequence_number": 1
}
```

### JSON Serialization Implementation

```cpp
// In ImageCapture.cpp or a shared header
#include <nlohmann/json.hpp>

// Manual to_json/from_json for existing types (add to Types.h or locally)
namespace nlohmann {
    template <>
    struct adl_serializer<Point2D> {
        static void to_json(json& j, const Point2D& p) {
            j = json{{"x", p.x}, {"y", p.y}};
        }
        static void from_json(const json& j, Point2D& p) {
            j.at("x").get_to(p.x);
            j.at("y").get_to(p.y);
        }
    };

    template <>
    struct adl_serializer<Pose2D> {
        static void to_json(json& j, const Pose2D& p) {
            j = json{{"x", p.x}, {"y", p.y}, {"theta", p.theta}};
        }
        static void from_json(const json& j, Pose2D& p) {
            j.at("x").get_to(p.x);
            j.at("y").get_to(p.y);
            j.at("theta").get_to(p.theta);
        }
    };
}

void ImageCapture::saveMetadata(const ImageMetadata& meta) {
    nlohmann::json j;
    j["image_path"] = meta.image_path;
    j["timestamp_ms"] = meta.timestamp_ms;
    j["robot_pose"] = meta.robot_pose;
    j["plan_coords"] = meta.plan_coords;
    j["camera_yaw"] = meta.camera_yaw;
    j["sequence_number"] = meta.sequence_number;

    // Build metadata file path
    std::string meta_path = session_dir_ + "/" +
        meta.image_path.substr(0, meta.image_path.find_last_of('.')) + ".json";

    std::ofstream file(meta_path);
    if (file.is_open()) {
        file << j.dump(2);  // Pretty print with 2-space indent
    } else {
        std::cerr << "[CAPTURE] Failed to save metadata: " << meta_path << std::endl;
    }
}
```

### PlanCorrelator Coverage Tracking

```cpp
// src/capture/PlanCorrelator.h
#pragma once

#include <opencv2/opencv.hpp>
#include "util/Types.h"

class PlanManager;

struct CameraParams {
    float hfov_deg = 60.0f;    // Horizontal field of view
    float vfov_deg = 45.0f;    // Vertical field of view
    float max_range_m = 5.0f;  // Maximum effective range
};

class PlanCorrelator {
public:
    explicit PlanCorrelator(PlanManager* plan);

    // CRITICAL: Must call after plan is loaded to initialize coverage map
    void initFromPlan();

    // Coordinate transforms (delegates to PlanManager)
    Point2D robotToPlanCoords(const Pose2D& robot_pose) const;

    // FOV and coverage
    void setCameraParams(const CameraParams& params) { camera_params_ = params; }
    std::vector<Point2D> calculateFOV(const Pose2D& pose) const;
    void updateCoverage(const Pose2D& pose);
    float getCoveragePercent() const;

    // Visualization
    void saveCoverageMap(const std::string& path) const;
    void resetCoverage();

private:
    PlanManager* plan_manager_;
    CameraParams camera_params_;
    cv::Mat coverage_map_;  // Binary mask: 255=covered, 0=not covered
    int total_free_pixels_ = 0;
    bool initialized_ = false;
};
```

### Coverage Calculation

```cpp
// In PlanCorrelator.cpp

// CRITICAL: Must call this after plan is loaded, before any coverage updates
void PlanCorrelator::initFromPlan() {
    if (!plan_manager_ || !plan_manager_->isLoaded()) {
        std::cerr << "[CORRELATOR] Cannot init: plan not loaded" << std::endl;
        return;
    }

    int w = plan_manager_->getGridWidth();
    int h = plan_manager_->getGridHeight();

    // Create empty coverage map (same size as plan)
    coverage_map_ = cv::Mat::zeros(h, w, CV_8UC1);

    // Count free pixels from plan (white = free, black = wall)
    const cv::Mat& plan = plan_manager_->getPlanImage();
    total_free_pixels_ = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (plan.at<uint8_t>(y, x) >= 128) {
                total_free_pixels_++;
            }
        }
    }

    initialized_ = true;
    std::cout << "[CORRELATOR] Initialized coverage map: " << w << "x" << h
              << ", " << total_free_pixels_ << " free pixels" << std::endl;
}

void PlanCorrelator::updateCoverage(const Pose2D& pose) {
    if (!plan_manager_ || !initialized_ || coverage_map_.empty()) return;

    // Calculate FOV polygon on plan
    std::vector<Point2D> fov = calculateFOV(pose);
    if (fov.size() < 3) return;

    // Convert to pixel coordinates
    std::vector<cv::Point> fov_pixels;
    float scale = plan_manager_->getResolution();
    for (const auto& pt : fov) {
        int px = static_cast<int>(pt.x / scale);
        int py = static_cast<int>(pt.y / scale);
        fov_pixels.emplace_back(px, py);
    }

    // Fill FOV polygon on coverage map
    cv::fillConvexPoly(coverage_map_, fov_pixels, cv::Scalar(255));
}

float PlanCorrelator::getCoveragePercent() const {
    if (coverage_map_.empty() || total_free_pixels_ == 0) return 0.0f;

    int covered = cv::countNonZero(coverage_map_);
    return 100.0f * static_cast<float>(covered) / static_cast<float>(total_free_pixels_);
}

std::vector<Point2D> PlanCorrelator::calculateFOV(const Pose2D& pose) const {
    // Simple triangular FOV approximation
    float range = camera_params_.max_range_m;
    float half_angle = camera_params_.hfov_deg * M_PI / 360.0f;  // Convert to radians

    Point2D plan_pos = robotToPlanCoords(pose);

    // Camera forward direction
    float cam_angle = pose.theta;  // Assume camera faces forward

    // FOV triangle: robot position + two far corners
    std::vector<Point2D> fov;
    fov.push_back(plan_pos);  // Apex at robot

    // Left corner
    float left_angle = cam_angle + half_angle;
    fov.push_back({
        plan_pos.x + range * std::cos(left_angle),
        plan_pos.y + range * std::sin(left_angle)
    });

    // Right corner
    float right_angle = cam_angle - half_angle;
    fov.push_back({
        plan_pos.x + range * std::cos(right_angle),
        plan_pos.y + range * std::sin(right_angle)
    });

    return fov;
}
```

### StateMachine Integration Pattern

```cpp
// Add to main.cpp inspection loop or StateMachine callbacks

// Option 1: Main loop polling
void runInspectionLoop(StateMachine& sm, ImageCapture& capture, ISensorSource& sensors) {
    std::string session_id = "insp_" + std::to_string(std::time(nullptr));

    while (g_running) {
        InspectionState state = sm.getState();

        // Start/stop capture based on state
        if (state == InspectionState::INSPECTING && !capture.isCapturing()) {
            capture.startCapture(session_id);
        } else if (state != InspectionState::INSPECTING && capture.isCapturing()) {
            capture.stopCapture();
        }

        // Capture frame if in INSPECTING state (interval enforced internally)
        if (state == InspectionState::INSPECTING) {
            Pose2D pose = sensors.getPose();
            capture.captureFrame(pose);
        }

        // Main loop rate limiting
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz loop
    }
}

// Option 2: State transition callbacks (if StateMachine supports them)
// Add callbacks for onEnterState/onExitState that start/stop capture
```

### CMake Additions

```cmake
# ============================================
# Capture (Story 1-7)
# ============================================

# Capture library
add_library(capture
    src/capture/ImageCapture.cpp
    src/capture/PlanCorrelator.cpp
)
target_include_directories(capture PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(capture
    plan
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

# Update g1_inspector to include capture
target_link_libraries(g1_inspector
    navigation
    slam
    sensors
    loco_hw
    plan
    app
    capture    # NEW
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    ${HPDF_LIBRARY}
    nlohmann_json::nlohmann_json
)

# Capture unit tests
if(GTest_FOUND)
    add_executable(test_capture test/test_capture.cpp)
    target_link_libraries(test_capture
        capture
        GTest::gtest_main
    )
    add_test(NAME test_capture COMMAND test_capture)
endif()
```

### Project Structure Notes

Files to create:
```
src/capture/                   # NEW DIRECTORY
├── ImageCapture.h             # NEW
├── ImageCapture.cpp           # NEW
├── PlanCorrelator.h           # NEW
└── PlanCorrelator.cpp         # NEW

test/
└── test_capture.cpp           # NEW

data/                          # NEW DIRECTORY (runtime)
└── inspections/               # Created at runtime
```

Files to modify:
```
src/main.cpp                   # Add capture integration
CMakeLists.txt                 # Add capture library
```

### Dependencies on Previous Stories

**Story 1-1 (Project Setup):**
- `src/util/Types.h` - Point2D, Pose2D structs (add JSON serialization)
- CMakeLists.txt base configuration
- OpenCV and nlohmann_json dependencies

**Story 1-4 (Hardware Integration):**
- `src/sensors/ISensorSource.h` - Interface for sensor data (LiDAR, IMU, pose, battery)
- **Note on Camera:** ISensorSource does NOT currently include camera access. Camera is accessed separately via cv::VideoCapture. This is intentional - camera is a different data path than the real-time sensor topics.

**Story 1-6 (State Machine + CLI + Plan Management):**
- `src/app/StateMachine.h` - State-based capture control
- `src/plan/PlanManager.h` - Coordinate transforms for geotagging

### ISensorSource and Camera Integration Note

The current `ISensorSource` interface provides:
```cpp
virtual LidarScan getLidarScan() = 0;
virtual Pose2D getPose() = 0;
virtual ImuData getImu() = 0;
virtual float getBatteryPercent() = 0;
```

Camera is intentionally **separate** because:
1. LiDAR/IMU are high-frequency real-time data (10-100Hz)
2. Camera capture is low-frequency (1fps) and high-latency
3. Different access patterns - sensor data is polled, camera is grabbed

**For this story:** Use `cv::VideoCapture` directly. The `ISensorSource*` parameter in ImageCapture constructor is for getting pose data, not camera frames.

**Future consideration:** If G1's camera requires SDK access, add optional `getCameraFrame()` method to ISensorSource in a future story.

### Test Data

Create test images for unit testing:
- Use placeholder images or capture test frames
- Mock ISensorSource for unit tests without camera hardware

---

## Verification Commands

```bash
# Build (inside Docker container)
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests
./test_capture               # All tests pass

# Test capture without robot (requires camera or will skip gracefully)
./g1_inspector --interactive
g1> upload --plan ../test_data/office.png --trade finishes
g1> calibrate --position 0,0,0
g1> start
# Wait 10 seconds (should capture ~10 images if camera available)
g1> stop

# Check captured images
ls data/inspections/insp_*/images/
# Should show: img_0001.jpg  img_0001.json  img_0002.jpg  img_0002.json  ...

# Verify metadata
cat data/inspections/insp_*/images/img_0001.json
# {"timestamp_ms": 1733356800000, "robot_pose": {"x": 0.0, "y": 0.0, "theta": 0.0}, ...}
```

### Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| Unit tests | `./test_capture` | Exit 0, all pass |
| Camera init | Start inspection | Log shows camera status |
| 1fps rate | 10s inspection | ~10 images captured |
| Metadata saved | Check img_NNNN.json | Valid JSON with pose |
| Directory created | ls data/inspections/ | Session directory exists |
| Graceful fallback | No camera | Warning logged, no crash |

---

## Previous Story Intelligence

### From Story 1-6 (State Machine + CLI + Plan Management)

**Key learnings:**
- StateMachine uses atomic state for thread safety
- PlanManager provides coordinate transforms via `robotToPlanCoords()`
- CLI integration pattern for main.cpp
- State transition validation pattern

**Files to reference:**
- `src/app/StateMachine.h` - State enum and transition methods
- `src/plan/PlanManager.h` - Coordinate transform API
- `src/app/CliHandler.cpp` - Example of component integration

**Code patterns established:**
- `#pragma once` for headers
- PascalCase classes, camelCase methods
- OpenCV for image I/O
- JSON serialization with nlohmann/json

### From Story 1-4 (Hardware Integration)

**Key learnings:**
- SensorManager provides thread-safe data access
- `HAS_UNITREE_SDK2` conditional compilation pattern
- Network interface configuration

**Integration notes:**
- Camera may be accessed via SDK or direct OpenCV VideoCapture
- For simulation, use ISensorSource mock

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **ImageCapture class** | 1fps capture with pose metadata |
| **PlanCorrelator class** | FOV coverage tracking |
| **JSON metadata** | Each image has sidecar JSON |
| **State integration** | Capture starts/stops with INSPECTING state |
| **Coverage tracking** | Percentage of plan area photographed |
| **Unit tests** | test_capture passes |

### Demo Script (Run This When Done)

```bash
# Inside Docker container
cd /workspace
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests
./test_capture

# Test with interactive CLI
./g1_inspector --interactive

# In interactive mode:
g1> upload --plan ../test_data/office.png
g1> calibrate --position 1,1,0
g1> start
# Wait 30 seconds
g1> stop

# Verify captures
ls data/inspections/*/images/
# Should show ~30 images (1fps for 30s)

cat data/inspections/*/images/img_0001.json
# Should show pose metadata
```

**SUCCESS CRITERIA:** Story 1.7 is DONE when:
1. `./test_capture` exits with code 0 (all tests pass)
2. Images captured at 1fps during INSPECTING state
3. Each image has JSON metadata with timestamp and pose
4. Images saved to session directory structure
5. Coverage percentage tracked and queryable
6. Graceful handling when camera unavailable

---

## References

- [Source: docs/architecture.md#4.7-Capture] - ImageCapture design
- [Source: docs/epics.md#story-7] - Original story requirements
- [Source: docs/sprint-artifacts/1-6-state-machine-cli-plan-management.md] - StateMachine and PlanManager patterns
- [External: OpenCV VideoCapture](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html) - Camera access
- [External: nlohmann/json](https://github.com/nlohmann/json) - JSON serialization

---

## Dev Agent Record

### Context Reference

This story file serves as the complete implementation context.

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

None

### Completion Notes List

- Story created by create-story workflow with comprehensive implementation guide
- ✅ Implementation completed: All 7 tasks implemented and tested
- ✅ Unit tests: 17 tests passing in test_capture.cpp
- ✅ Build verified: All components compile and link successfully
- ✅ No regressions: Existing tests continue to pass (test_capture passes 100%)

### File List

**Created:**
- src/capture/ImageCapture.h - Image capture interface with ImageMetadata struct
- src/capture/ImageCapture.cpp - Async image capture with JSON metadata serialization
- src/capture/PlanCorrelator.h - Coverage tracking and FOV calculation interface
- src/capture/PlanCorrelator.cpp - Coverage map with visualization export
- test/test_capture.cpp - 41 unit tests for capture functionality
- test/test_e2e_story_1_7.sh - End-to-end test script for capture system

**Modified:**
- src/main.cpp - Added capture integration with background inspection loop
- CMakeLists.txt - Added capture library, test_capture executable, and test_e2e_story_1_7

**NOT part of Story 1-7 (found in git but unrelated):**
- docker/Dockerfile - libpoppler-cpp-dev for PDF plan loading (Story 1-6)
- test/test_e2e_story_1_6.sh - Path fixes for e2e test script (Story 1-6)

### Change Log

- 2025-12-05: Story 1-7 created by create-story workflow - comprehensive Visual Capture guide with ImageCapture design, PlanCorrelator coverage tracking, JSON metadata serialization, and state machine integration patterns.
- 2025-12-05: Validation improvements applied (validate-create-story):
  - CRITICAL-1 FIXED: Replaced NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE macro with manual serialization for nested types (Point2D, Pose2D)
  - CRITICAL-2 FIXED: Added "Camera API Decision" section clarifying cv::VideoCapture for MVP, RealSense for future
  - CRITICAL-3 FIXED: Replaced blocking saveImage() with saveImageAsync() using std::async
  - ENHANCEMENT-1 FIXED: Added std::filesystem::create_directories() in startCapture()
  - ENHANCEMENT-2 FIXED: Added initFromPlan() method to PlanCorrelator for coverage map initialization
  - ENHANCEMENT-3 FIXED: Added "ISensorSource and Camera Integration Note" clarifying camera is separate from sensor interface
  - OPTIMIZATION: Removed duplicate Technical Reference section (consolidated into Camera API Decision)
  - OPTIMIZATION: Added std::future tracking for async saves with cleanup in stopCapture()
- 2025-12-05: Story 1-7 implementation completed by dev-story workflow:
  - Implemented ImageCapture class with async JPEG saving and JSON metadata
  - Implemented PlanCorrelator class with FOV calculation and coverage tracking
  - Integrated capture with StateMachine via main loop state monitoring
  - Added background thread for capture during INSPECTING state
  - Created 17 comprehensive unit tests (all passing)
  - All acceptance criteria satisfied (AC1-AC6, AC7 deferred per story)
- 2025-12-05: Code review fixes applied (code-review workflow):
  - HIGH-1 FIXED: getCapturedImages() now properly populates with thread-safe mutex protection
  - HIGH-2 FIXED: Added isLoaded() check before calling robotToPlanCoords()
  - HIGH-3 FIXED: Added error feedback in main.cpp when capture fails to start
  - HIGH-4 FIXED: Added 5 JSON serialization tests (Point2D, Pose2D, ImageMetadata roundtrip, file I/O)
  - MEDIUM-1 FIXED: Changed filename format from img_%04d to img_%08d for long inspections
  - MEDIUM-2 FIXED: Coverage calculation now uses free_space_mask_ to exclude wall pixels
  - MEDIUM-3 FIXED: Added frame validation (size check, corruption detection)
  - MEDIUM-4 FIXED: Added clarifying comment explaining PlanCorrelator vs direct PlanManager usage
  - LOW-1 FIXED: Replaced magic number 360.0f with explicit (PI/180)/2 for degrees-to-radians conversion
  - LOW-2 FIXED: Changed [CAPTURE] to [CORRELATOR] for coverage messages in main.cpp
- 2025-12-05: Code review fixes applied (code-review workflow - round 2):
  - HIGH-1 FIXED: Corrected story documentation - StateMachine.cpp was never modified, capture uses main.cpp background thread
  - HIGH-2 FIXED: Made image_count_ std::atomic<int> for thread safety in saveImageAsync
  - HIGH-3 FIXED: Added isLoaded() check to PlanCorrelator::robotToPlanCoords
  - MEDIUM-1 FIXED: Added try/catch exception handling to capture background thread in main.cpp
  - MEDIUM-2 FIXED: Coverage tracking now updates independently of camera availability
  - MEDIUM-3 FIXED: Added session management functions (listSessions, deleteSession, getTotalDiskUsage)
  - MEDIUM-4 FIXED: Added shrink_to_fit() for pending_saves_ vector to prevent memory bloat
  - Added 7 new unit tests for session management functions (24 total tests)
- 2025-12-05: Code review fixes applied (code-review workflow - round 3):
  - HIGH-1 FIXED: Race condition in saveImageAsync - ++image_count_ now correctly returns atomic value
  - HIGH-2 FIXED: Added ImageMetadata JSON deserialization (adl_serializer) and loadMetadata() static method
  - HIGH-3 FIXED: Updated story File List - Dockerfile changes are unrelated to Story 1-7 (poppler for PDF plans is Story 1-6)
  - HIGH-4 FIXED: Added shutting_down_ atomic flag to prevent use-after-free in async lambda
  - MEDIUM-1 FIXED: cleanupPendingSaves() now only called when pending_saves_.size() > 5
  - MEDIUM-2 FIXED: Added disk space check in startCapture() with getAvailableDiskSpace() (50MB minimum)
  - MEDIUM-3 FIXED: Added documentation comment explaining FOV coverage limitation (no wall occlusion)
  - LOW-1 FIXED: Changed [CAPTURE] to [CORRELATOR] for coverage messages in main.cpp
  - LOW-3 FIXED: Added named constants for frame validation (MIN_FRAME_WIDTH, MIN_FRAME_HEIGHT, MIN/MAX_INTENSITY)
  - Added 5 new unit tests for loadMetadata and disk space checking (29 total tests)
- 2025-12-05: Code review fixes applied (code-review workflow - round 4):
  - HIGH-1 FIXED: Added waitForPendingSaves() public method and documented getCapturedImages() async behavior
  - HIGH-2 FIXED: Added captureTestFrame() method and 5 mock camera tests to verify actual capture functionality
  - HIGH-3 FIXED: Added detailed comments explaining coverage coordinate system (meters to pixels conversion)
  - HIGH-4 FIXED: Improved stopCapture() to wait for futures without holding mutex (prevents potential deadlock)
  - MEDIUM-2 FIXED: Added test_e2e_story_1_7.sh end-to-end test script and registered in CMakeLists.txt
  - MEDIUM-3 FIXED: Rate-limited frame intensity warnings to max 5 per session (reduces log noise in dark sites)
  - Added 2 new coordinate system tests for PlanCorrelator (CoverageCoordinateSystem, CoverageMultiplePositions)
  - Added 5 new mock camera tests (CaptureTestFrameSavesImage, MetadataContent, IntervalEnforced, MultipleImages, WaitForPendingSaves)
  - Total: 41 unit tests in test_capture.cpp
  - Fixed captureTestFrame interval logic to allow first capture to always succeed
