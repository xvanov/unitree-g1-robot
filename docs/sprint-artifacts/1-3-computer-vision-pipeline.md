# Story 1.3: Computer Vision Pipeline

Status: ready-for-dev

## Story

As the **robot**,
I want **face detection, personnel identification, and context building**,
So that **the LLM receives rich observations about who is present and their state**.

## File List

| File | Action |
|------|--------|
| `src/greeter/FaceDetector.h` | CREATE |
| `src/greeter/FaceDetector.cpp` | CREATE |
| `src/greeter/ContextBuilder.h` | CREATE |
| `src/greeter/ContextBuilder.cpp` | CREATE |
| `test/test_face_detector.cpp` | CREATE |
| `test/test_context_builder.cpp` | CREATE |
| `CMakeLists.txt` | MODIFY |
| `src/main.cpp` | MODIFY |

## Acceptance Criteria

1. Face detector loads OpenCV DNN Caffe model and detects faces with >0.5 confidence
2. Performance: <100ms per frame on CPU (run detection every 3rd frame for 10 FPS at 30 FPS capture)
3. Personnel lookup returns correct `PersonnelRecord` using `id` field from PersonnelDatabase
4. Posture detection: `"bent_forward"` when face bottom >70% of frame height, else `"standing"`
5. Attention detection: `"looking_at_robot"` when face centered (35-65% width) AND frontal (aspect 0.7-1.3), else `"looking_away"`
6. Context JSON includes: `camera_frame` (base64), `detected_faces`, `environment`, `robot_state`, `overheard_conversations`

## Tasks

- [ ] Create `src/greeter/FaceDetector.h` with FaceRect struct and FaceDetector class
- [ ] Create `src/greeter/FaceDetector.cpp` with OpenCV DNN implementation
- [ ] Create `src/greeter/ContextBuilder.h` with DetectedFace, EnvironmentContext, RobotStateContext structs
- [ ] Create `src/greeter/ContextBuilder.cpp` with context assembly and JSON building
- [ ] Modify CMakeLists.txt line 16: add DNN component to OpenCV find_package
- [ ] Add FaceDetector and ContextBuilder to greeter library
- [ ] Create unit test `test/test_face_detector.cpp`
- [ ] Create unit test `test/test_context_builder.cpp`
- [ ] Add `--test-face-detection` CLI flag to main.cpp
- [ ] Verify face detection model files exist in `models/face_detection/`

## Prerequisites

**Story 1.1 (DONE):** Provides foundation code
- `src/greeter/PersonnelDatabase.h/cpp` - Uses `id` field (not `person_id`)
- `src/greeter/GreeterConfig.h/cpp` - Model paths from config
- `models/face_detection/*.caffemodel` and `deploy.prototxt`
- Greeter library already in CMakeLists.txt

**Story 1.2 (ready-for-dev):** Should complete first for shared library structure, but FaceDetector/ContextBuilder can be developed independently.

## Dev Notes

### CMakeLists.txt Changes (EXACT)

**Step 1: Modify line 16** - Change existing find_package:
```cmake
# FROM:
find_package(OpenCV REQUIRED)

# TO:
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui dnn imgcodecs)
```

**Step 2: Update greeter library** (around line 274):
```cmake
add_library(greeter
    src/greeter/GreeterConfig.cpp
    src/greeter/PersonnelDatabase.cpp
    src/greeter/FaceDetector.cpp      # NEW
    src/greeter/ContextBuilder.cpp    # NEW
)
target_include_directories(greeter PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(greeter
    yaml-cpp
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
)
```

**Step 3: Add tests:**
```cmake
add_executable(test_face_detector test/test_face_detector.cpp)
target_link_libraries(test_face_detector greeter ${OpenCV_LIBS})

add_executable(test_context_builder test/test_context_builder.cpp)
target_link_libraries(test_context_builder greeter ${OpenCV_LIBS})
```

### FaceDetector Interface

```cpp
// src/greeter/FaceDetector.h
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

namespace greeter {

struct FaceRect {
    cv::Rect bounding_box;
    float confidence;
    int centerX() const { return bounding_box.x + bounding_box.width / 2; }
    int centerY() const { return bounding_box.y + bounding_box.height / 2; }
    int bottom() const { return bounding_box.y + bounding_box.height; }
};

class FaceDetector {
public:
    FaceDetector();
    ~FaceDetector() = default;

    // Returns false if model files missing/invalid. Caller should continue without face detection.
    bool init(const std::string& prototxt_path, const std::string& caffemodel_path);

    std::vector<FaceRect> detect(const cv::Mat& frame, float min_confidence = 0.5f);
    bool isInitialized() const { return initialized_; }
    double getLastDetectionTimeMs() const { return last_detection_time_ms_; }

private:
    cv::dnn::Net net_;
    bool initialized_ = false;
    double last_detection_time_ms_ = 0.0;
};

}  // namespace greeter
```

### ContextBuilder Interface

```cpp
// src/greeter/ContextBuilder.h
#pragma once
#include "FaceDetector.h"
#include "PersonnelDatabase.h"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <optional>

namespace greeter {

struct DetectedFace {
    cv::Rect bounding_box;
    float confidence;
    std::optional<PersonnelRecord> identity;  // Uses PersonnelRecord.id field
    std::string posture;     // "standing", "bent_forward"
    std::string attention;   // "looking_at_robot", "looking_away"
    float estimated_distance = 0.0f;
};

struct EnvironmentContext {
    bool near_staircase = false;
    bool witnesses_present = false;
    std::string camera_coverage;  // "full", "partial", "none"
    std::vector<std::string> active_observations;
};

struct RobotStateContext {
    std::string current_pose;     // "standing", "sitting", "walking"
    float battery_percent = 100.0f;
    std::string current_action;
    std::vector<std::string> recent_actions;  // Last 5
};

class ContextBuilder {
public:
    ContextBuilder();
    ~ContextBuilder() = default;

    void setPersonnelDatabase(const PersonnelDatabase* db) { personnel_db_ = db; }
    void setFrame(const cv::Mat& frame);  // Use frame.clone() if caller may modify
    void setDetectedFaces(const std::vector<FaceRect>& faces);
    void setEnvironment(const EnvironmentContext& env);
    void setRobotState(const RobotStateContext& state);  // Passed in by caller, not from LocoController directly
    void addOverheardConversation(const std::string& conversation);
    void clearOverheardConversations();

    nlohmann::json buildContextJson() const;
    std::vector<DetectedFace> getIdentifiedFaces() const;

private:
    const PersonnelDatabase* personnel_db_ = nullptr;
    cv::Mat current_frame_;
    std::vector<FaceRect> raw_faces_;
    EnvironmentContext environment_;
    RobotStateContext robot_state_;
    std::vector<std::string> overheard_conversations_;

    std::string detectPosture(const FaceRect& face, int frame_height) const;
    std::string detectAttention(const FaceRect& face, int frame_width) const;
    std::string encodeFrameBase64(const cv::Mat& frame) const;
    float estimateDistance(const FaceRect& face, int frame_height) const;
};

}  // namespace greeter
```

### Key Implementation Notes

**OpenCV DNN Face Detection:**
```cpp
// Load model once in init()
net_ = cv::dnn::readNetFromCaffe(prototxt_path, caffemodel_path);
net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

// In detect():
cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300),
    cv::Scalar(104.0, 177.0, 123.0), false, false);
net_.setInput(blob);
cv::Mat detection = net_.forward();
// Output shape: [1, 1, N, 7] where each detection = [batchId, classId, conf, x1, y1, x2, y2]
// CRITICAL: Clamp coordinates to [0, frame.cols-1] and [0, frame.rows-1]
```

**Posture Heuristic:**
```cpp
float face_bottom_ratio = static_cast<float>(face.bottom()) / frame_height;
return (face_bottom_ratio > 0.70f) ? "bent_forward" : "standing";
```

**Attention Heuristic:**
```cpp
float center_x = static_cast<float>(face.centerX()) / frame_width;
float aspect = static_cast<float>(face.bounding_box.width) / face.bounding_box.height;
bool centered = (center_x > 0.35f && center_x < 0.65f);
bool frontal = (aspect > 0.7f && aspect < 1.3f);
return (centered && frontal) ? "looking_at_robot" : "looking_away";
```

**Distance Estimation (rough):**
```cpp
float face_ratio = static_cast<float>(face.bounding_box.height) / frame_height;
if (face_ratio > 0.25f) return 1.0f;
if (face_ratio > 0.15f) return 2.0f;
if (face_ratio > 0.10f) return 3.0f;
if (face_ratio > 0.05f) return 4.0f;
return 5.0f;
```

**Base64 Encoding:** Use cv::imencode(".jpg", frame, buf) then standard base64 encoding loop.

### Context JSON Schema

```json
{
  "camera_frame": "<base64-jpg>",
  "detected_faces": [{
    "id": "alex_reeves",
    "name": "Alex Reeves",
    "role": "Head of Robotics Infrastructure",
    "relationship": "...",
    "context_notes": ["..."],
    "posture": "bent_forward",
    "attention": "looking_away",
    "estimated_distance": 1.2
  }],
  "environment": {
    "near_staircase": false,
    "witnesses_present": false,
    "camera_coverage": "full",
    "observations": ["..."]
  },
  "robot_state": {
    "current_pose": "standing",
    "battery_percent": 87.5,
    "current_action": "WAITING",
    "recent_actions": ["WAVE_HAND", "FOLLOW"]
  },
  "overheard_conversations": ["..."]
}
```

### Error Handling

**Model Load Failure:**
- If `FaceDetector::init()` returns false, log warning and continue
- GreeterRunner should support vision-disabled mode for dry-run testing
- Detection calls on uninitialized detector return empty vector

**Frame Processing:**
- Empty frame → return empty detection vector
- Invalid Mat → log error, return empty vector

### Testing

**Test Assets:** Create `test/assets/test_face.jpg` (640x480 with single centered face). Any permissively-licensed face image works, or use synthetic cv::Mat with drawn rectangle for unit tests.

**test_face_detector.cpp tests:**
1. Model loads from valid paths
2. Model fails gracefully with invalid paths (returns false, no crash)
3. Detects face in test image
4. Returns empty for image with no faces
5. Detection time measurement works
6. Confidence filtering (only >threshold)
7. Bounding box within image bounds (clamp verification)

**test_context_builder.cpp tests:**
1. Build context JSON with all fields
2. Posture: face at bottom → bent_forward
3. Posture: face in middle → standing
4. Attention: centered face → looking_at_robot
5. Attention: off-center → looking_away
6. Base64 produces valid string
7. Personnel matching uses `id` field correctly
8. Multiple faces handled
9. Empty frame handled
10. Overheard conversations in context

### CLI Integration

Add to `src/main.cpp`:
```cpp
if (arg == "--test-face-detection") {
    test_face_detection = true;
}
if (arg == "--camera" && i + 1 < argc) {
    test_camera_index = std::stoi(argv[++i]);
}

// Test mode: open camera, run detection, display bounding boxes
// Exit on ESC key
```

### Verification

```bash
# Build
mkdir -p build && cd build && cmake .. && make greeter test_face_detector test_context_builder

# Unit tests
./test_face_detector
./test_context_builder

# Integration (manual)
./g1_inspector --test-face-detection --camera 0
# Shows camera with green bounding boxes, "Faces: N | Time: Xms"
```

### Model Files

Already in `models/face_detection/` from Story 1.1:
- `deploy.prototxt` (28KB)
- `res10_300x300_ssd_iter_140000.caffemodel` (10.4MB)

If missing:
```bash
./scripts/download_models.sh
```

## References

- [Tech Spec: FaceDetector](docs/agentic-misalignment-tech-spec.md)
- [Story 1.1: Configuration Infrastructure](docs/sprint-artifacts/1-1-configuration-infrastructure.md)
- [PersonnelDatabase Interface](src/greeter/PersonnelDatabase.h) - Uses `id` field

## Dev Agent Record

### Agent Model Used

{{agent_model_name_version}}

### Completion Notes List

### File List

| File | Action |
|------|--------|
| `src/greeter/FaceDetector.h` | CREATE |
| `src/greeter/FaceDetector.cpp` | CREATE |
| `src/greeter/ContextBuilder.h` | CREATE |
| `src/greeter/ContextBuilder.cpp` | CREATE |
| `test/test_face_detector.cpp` | CREATE |
| `test/test_context_builder.cpp` | CREATE |
| `CMakeLists.txt` | MODIFY (line 16 + greeter lib + tests) |
| `src/main.cpp` | MODIFY (--test-face-detection, --camera) |
