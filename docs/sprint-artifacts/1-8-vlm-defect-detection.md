# Story 1.8: VLM Defect Detection

**Status:** Ready for Review

---

## Quick Reference

**New files to create:**
- `src/detection/VlmClient.h` / `.cpp` - HTTP client for Anthropic Claude API with vision
- `src/detection/DefectTypes.h` / `.cpp` - Defect data structures, enums, and JSON serialization
- `src/detection/ImageAnnotator.h` / `.cpp` - Draw bounding boxes on images for defects
- `sim/detection_sim/DetectionSim.h` / `.cpp` - Simulation/test harness for VLM detection
- `sim/detection_sim/main.cpp` - detection_sim executable entry point
- `test/test_detection.cpp` - Unit tests for VlmClient, parsing, retry logic
- `test/MockVlmClient.h` - Mock client for unit testing without API calls

**Files to modify:**
- `CMakeLists.txt` - Add `detection` library, `detection_sim` executable, `test_detection`
- `src/main.cpp` - Add `--test-detection` CLI option (optional for hardware integration)

**Key classes:**
| Class | Purpose |
|-------|---------|
| `VlmClient` | HTTP POST to Claude API with base64 images, structured prompts, JSON response parsing |
| `Defect` | Data struct: id, type, description, image_loc, plan_loc, confidence |
| `ImageAnnotator` | Draw bounding boxes and labels on captured images |
| `DetectionSim` | Test harness to run VLM detection on test images and evaluate results |

**Primary acceptance criteria:** AC1 (images sent to VLM), AC2 (defects returned with structured data), AC3 (annotated images generated), AC4 (API errors handled gracefully)

**Prerequisites:** Story 1-7 (ImageCapture) must be complete for captured images and metadata

---

## Story

As the **system**,
I want **to analyze images via VLM API**,
So that **defects are detected automatically**.

---

## Acceptance Criteria

1. **AC1:** Images sent to VLM API (Claude Anthropic) with base64 encoding
2. **AC2:** Defects returned with type, description, location (image coords + plan coords), confidence
3. **AC3:** Annotated images generated with bounding boxes around detected defects
4. **AC4:** API errors handled gracefully with retry and exponential backoff
5. **AC5:** Structured prompts guide VLM to detect construction-specific defects
6. **AC6:** Confidence scoring for each detected defect (0.0-1.0)
7. **AC7:** Support both real-time single image analysis and batch processing

---

## Tasks / Subtasks

- [x] **Task 1: Define Defect Data Structures** (AC: 2, 6)
  - [x] 1.1 Create `src/detection/DefectTypes.h`
    - `DefectType` enum: `LOCATION_ERROR`, `QUALITY_ISSUE`, `SAFETY_HAZARD`, `MISSING_ELEMENT`
    - `Defect` struct with all required fields
    - JSON serialization using nlohmann/json
  - [x] 1.2 Implement JSON to_json/from_json for Defect and DefectType

- [x] **Task 2: Implement VlmClient Class** (AC: 1, 2, 4, 5)
  - [x] 2.1 Create `src/detection/VlmClient.h` - VLM client interface
    - Constructor: `VlmClient(const std::string& api_key)`
    - `analyzeImage(const cv::Mat& image, const std::string& plan_context, const Pose2D& pose)` -> `std::vector<Defect>`
    - `setApiUrl(url)` - Override default API endpoint
    - `setModel(model)` - Set Claude model (default: claude-sonnet-4-5)
    - `setMaxRetries(n)` - Set retry count (default: 3)
    - `setTimeout(ms)` - Set HTTP timeout (default: 30000)
  - [x] 2.2 Create `src/detection/VlmClient.cpp` - implementation
    - Base64 encode image using OpenCV imencode + custom base64 encoder
    - Build JSON request body per Claude Messages API format
    - Use libcurl for HTTP POST to `https://api.anthropic.com/v1/messages`
    - Set required headers: `x-api-key`, `anthropic-version: 2023-06-01`, `content-type: application/json`
    - Parse JSON response to extract defect information
    - Retry with exponential backoff on 429 (rate limit) and 5xx errors
    - Log errors but don't crash - return empty vector on failure

- [x] **Task 3: Implement Structured Prompts** (AC: 5)
  - [x] 3.1 Create defect detection prompt template in VlmClient
    - Context about construction site inspection
    - Trade-specific guidance (finishes, MEP, structural)
    - Expected defect types with examples
    - Output format specification (JSON with defect array)
    - Include robot pose for spatial context
  - [x] 3.2 Parse VLM response to extract structured defect data
    - Handle JSON parsing errors gracefully
    - Validate required fields exist
    - Filter low-confidence detections (threshold configurable)

- [x] **Task 4: Implement ImageAnnotator Class** (AC: 3)
  - [x] 4.1 Create `src/detection/ImageAnnotator.h` - annotation interface
    - `annotateImage(cv::Mat& image, const std::vector<Defect>& defects)` - draw boxes
    - `setBoxColor(cv::Scalar)` - color for bounding boxes
    - `setFontScale(float)` - text size for labels
    - `saveAnnotatedImage(path)` - save with annotations
  - [x] 4.2 Create `src/detection/ImageAnnotator.cpp` - implementation
    - Draw rectangles around defect locations
    - Add text labels with defect type and confidence
    - Color-code by severity (red=high, yellow=medium, green=low)

- [x] **Task 5: Implement Base64 Encoding** (AC: 1)
  - [x] 5.1 Create base64 encode function (or use existing library)
    - Encode cv::Mat to JPEG buffer, then to base64 string
    - Optimize for image sizes (resize if >4096px to reduce tokens)
    - JPEG quality 85 for balance of quality and size

- [x] **Task 6: Implement DetectionSim** (AC: 1, 2, 3, 7)
  - [x] 6.1 Create `sim/detection_sim/DetectionSim.h` - test harness
    - Constructor: `DetectionSim(const std::string& test_images_dir)`
    - `run(VlmClient& client)` - process all test images
    - `evaluate(const std::string& ground_truth_json)` - compare results
    - `saveResults(const std::string& path)` - output defects.json
  - [x] 6.2 Create `sim/detection_sim/DetectionSim.cpp` - implementation
    - Load all .jpg/.png from test_images_dir
    - Call VlmClient for each image
    - Aggregate results
    - Generate summary statistics
  - [x] 6.3 Create `sim/detection_sim/main.cpp` - CLI entry point
    - `--images <dir>` - input images directory
    - `--plan <file>` - plan image for context
    - `--output <dir>` - output directory for results
    - `--key <api_key>` or use ANTHROPIC_API_KEY env var
    - `--model <model>` - Claude model to use
    - `--dry-run` - Test without API calls (use mock responses)
    - `--session <dir>` - Load images from ImageCapture session directory

- [x] **Task 7: CMake Integration** (AC: 1-7)
  - [x] 7.1 Update `CMakeLists.txt`
    - Add `detection` library with VlmClient.cpp, ImageAnnotator.cpp
    - Link curl, OpenCV, nlohmann_json
    - Add `detection_sim` executable
    - Add `test_detection` unit test

- [x] **Task 8: Unit Tests** (AC: 2, 4)
  - [x] 8.1 Create `test/test_detection.cpp`
    - Test Defect JSON serialization/deserialization
    - Test VlmClient request building (mock HTTP)
    - Test retry logic with simulated failures
    - Test ImageAnnotator drawing functions
    - Test base64 encoding correctness
    - Test confidence filtering
  - [x] 8.2 Create `test/MockVlmClient.h` - Mock client for unit tests
    - Inherits from VlmClient or wraps interface
    - Returns predefined defects without API calls
    - Allows testing detection pipeline without API costs

- [x] **Task 9: ImageCapture Integration** (AC: 7)
  - [x] 9.1 Add integration pattern for processing captured images
    - Load images from ImageCapture session directory
    - Parse ImageMetadata JSON for pose information
    - Batch process session images through VlmClient
    - Save detection results alongside captured images

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Use any Python or Python bindings - this is pure C++17
- Block the main inspection loop during VLM API calls - use async or threading
- Hardcode API keys - always read from environment or config
- Ignore HTTP errors - implement proper retry with backoff
- Create new types that duplicate existing Point2D, Pose2D from Types.h

**MUST USE:**
- C++17 standard (`-std=c++17`)
- libcurl for HTTP requests (already a project dependency)
- nlohmann/json for JSON handling (already in project)
- OpenCV for image encoding (`cv::imencode`) and annotation (`cv::rectangle`, `cv::putText`)
- Existing types: `Point2D`, `Pose2D` from `src/util/Types.h`
- Environment variable `ANTHROPIC_API_KEY` for API authentication
- `curl_global_init(CURL_GLOBAL_DEFAULT)` at startup and `curl_global_cleanup()` at shutdown

### Anthropic Claude API Specification

**Endpoint:** `https://api.anthropic.com/v1/messages`

**Required Headers:**
```
x-api-key: {ANTHROPIC_API_KEY}
anthropic-version: 2023-06-01
content-type: application/json
```

**Model Names:** Use full model ID format: `claude-sonnet-4-5-20250514` (recommended) or short form `claude-sonnet-4-5` (works but may change).

**Request Body Format:**
```json
{
  "model": "claude-sonnet-4-5-20250514",
  "max_tokens": 4096,
  "messages": [
    {
      "role": "user",
      "content": [
        {
          "type": "image",
          "source": {
            "type": "base64",
            "media_type": "image/jpeg",
            "data": "BASE64_ENCODED_IMAGE_DATA"
          }
        },
        {
          "type": "text",
          "text": "STRUCTURED_PROMPT_HERE"
        }
      ]
    }
  ]
}
```

**Response Format:**
```json
{
  "id": "msg_...",
  "type": "message",
  "role": "assistant",
  "content": [
    {
      "type": "text",
      "text": "JSON defect analysis result"
    }
  ],
  "model": "claude-sonnet-4-5-20250514",
  "stop_reason": "end_turn",
  "usage": {
    "input_tokens": 1234,
    "output_tokens": 567
  }
}
```

### VlmClient Design

**Thread Safety Note:** VlmClient is NOT thread-safe. Each thread should use its own VlmClient instance, or access must be serialized with a mutex. The underlying libcurl handles are not shared between instances.

```cpp
// src/detection/VlmClient.h
#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "util/Types.h"
#include "detection/DefectTypes.h"

class VlmClient {
public:
    explicit VlmClient(const std::string& api_key);
    virtual ~VlmClient();  // Virtual for MockVlmClient inheritance

    // CRITICAL: Call once at application startup before creating any VlmClient
    static void globalInit();
    // CRITICAL: Call once at application shutdown after all VlmClients destroyed
    static void globalCleanup();

    // Main analysis function (virtual for mocking)
    virtual std::vector<Defect> analyzeImage(
        const cv::Mat& image,
        const std::string& plan_context,
        const Pose2D& pose
    );

    // Configuration
    void setApiUrl(const std::string& url) { api_url_ = url; }
    void setModel(const std::string& model) { model_ = model; }
    void setMaxRetries(int n) { max_retries_ = n; }
    void setTimeout(long ms) { timeout_ms_ = ms; }
    void setConfidenceThreshold(float t) { confidence_threshold_ = t; }

    // Status
    int getLastStatusCode() const { return last_status_code_; }
    std::string getLastError() const { return last_error_; }
    int getTokensUsed() const { return tokens_used_; }

protected:
    std::string buildPrompt(const std::string& plan_context, const Pose2D& pose) const;
    std::string encodeImageBase64(const cv::Mat& image) const;
    std::string httpPost(const std::string& body);
    std::vector<Defect> parseResponse(const std::string& json_response);
    std::vector<Defect> filterByConfidence(std::vector<Defect>& defects) const;
    bool shouldRetry(int status_code) const;

    std::string api_key_;
    std::string api_url_ = "https://api.anthropic.com/v1/messages";
    std::string model_ = "claude-sonnet-4-5-20250514";
    int max_retries_ = 3;
    long timeout_ms_ = 30000;
    float confidence_threshold_ = 0.5f;

    // Last request status
    int last_status_code_ = 0;
    std::string last_error_;
    int tokens_used_ = 0;

private:
    static bool curl_initialized_;
};
```

### DefectTypes Design

```cpp
// src/detection/DefectTypes.h
#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "util/Types.h"

enum class DefectType {
    LOCATION_ERROR,    // Element in wrong location
    QUALITY_ISSUE,     // Scratch, crack, stain, poor workmanship
    SAFETY_HAZARD,     // Exposed wiring, missing covers
    MISSING_ELEMENT    // Expected element not present
};

struct Defect {
    std::string id;              // Unique ID (e.g., "def_001")
    DefectType type;             // Defect category
    std::string description;     // Human-readable description
    Point2D image_loc;           // Pixel coordinates in source image (uses existing type)
    int bbox_x = 0;              // Bounding box top-left x
    int bbox_y = 0;              // Bounding box top-left y
    int bbox_width = 0;          // Bounding box width
    int bbox_height = 0;         // Bounding box height
    Point2D plan_loc;            // World coordinates on plan (meters)
    float confidence = 0.0f;     // 0.0 - 1.0
    std::string severity;        // "high", "medium", "low"
    std::string trade;           // "finishes", "mep", "structural"

    // Helper to get bounding box as cv::Rect for OpenCV operations
    cv::Rect getBoundingBox() const { return cv::Rect(bbox_x, bbox_y, bbox_width, bbox_height); }
};

// JSON serialization helpers
std::string defectTypeToString(DefectType type);
DefectType stringToDefectType(const std::string& str);

// nlohmann/json serialization
void to_json(nlohmann::json& j, const Defect& d);
void from_json(const nlohmann::json& j, Defect& d);
```

### Structured Prompt Template

```cpp
std::string VlmClient::buildPrompt(const std::string& plan_context, const Pose2D& pose) const {
    std::ostringstream prompt;
    prompt << R"(You are an expert construction site inspector analyzing an image for defects.

CONTEXT:
- This is a construction site inspection photo
- Trade type: )" << (plan_context.empty() ? "general finishes" : plan_context) << R"(
- Robot position: ()" << pose.x << ", " << pose.y << R"() meters, facing )" << pose.theta << R"( radians

TASK:
Analyze this image for construction defects. Look for:
1. LOCATION_ERROR - Elements installed in wrong positions
2. QUALITY_ISSUE - Scratches, cracks, stains, poor workmanship, misalignment
3. SAFETY_HAZARD - Exposed wiring, missing covers, trip hazards
4. MISSING_ELEMENT - Expected elements not present

OUTPUT FORMAT:
Respond with ONLY valid JSON in this exact format:
{
  "defects": [
    {
      "id": "def_001",
      "type": "QUALITY_ISSUE",
      "description": "Scratch visible on tile surface",
      "image_location": {"x": 320, "y": 240},
      "bounding_box": {"x": 280, "y": 200, "width": 80, "height": 80},
      "confidence": 0.87,
      "severity": "medium",
      "trade": "finishes"
    }
  ],
  "summary": "Brief overall assessment"
}

If no defects found, return: {"defects": [], "summary": "No defects detected"}

IMPORTANT:
- Only report defects you are confident about (confidence > 0.5)
- Provide accurate pixel coordinates for bounding boxes
- Be specific in descriptions
)";
    return prompt.str();
}
```

### HTTP Implementation with libcurl

```cpp
// In VlmClient.cpp
#include <curl/curl.h>
#include <thread>
#include <chrono>

// Static initialization flag
bool VlmClient::curl_initialized_ = false;

// CRITICAL: Call once at application startup
void VlmClient::globalInit() {
    if (!curl_initialized_) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl_initialized_ = true;
    }
}

// CRITICAL: Call once at application shutdown
void VlmClient::globalCleanup() {
    if (curl_initialized_) {
        curl_global_cleanup();
        curl_initialized_ = false;
    }
}

// Callback for curl response
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append(static_cast<char*>(contents), size * nmemb);
    return size * nmemb;
}

std::string VlmClient::httpPost(const std::string& body) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        last_error_ = "Failed to initialize curl";
        last_status_code_ = 0;
        return "";
    }

    std::string response;
    struct curl_slist* headers = nullptr;

    // Set headers
    headers = curl_slist_append(headers, ("x-api-key: " + api_key_).c_str());
    headers = curl_slist_append(headers, "anthropic-version: 2023-06-01");
    headers = curl_slist_append(headers, "content-type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, api_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body.size());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms_);

    // Retry loop with exponential backoff
    int retries = 0;
    while (retries <= max_retries_) {
        response.clear();
        CURLcode res = curl_easy_perform(curl);

        if (res != CURLE_OK) {
            last_error_ = curl_easy_strerror(res);
            last_status_code_ = 0;
        } else {
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &last_status_code_);

            if (last_status_code_ == 200) {
                break;  // Success
            }

            if (!shouldRetry(last_status_code_)) {
                break;  // Non-retryable error
            }
        }

        // Exponential backoff: 1s, 2s, 4s...
        if (retries < max_retries_) {
            int delay_ms = 1000 * (1 << retries);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
        retries++;
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return response;
}

bool VlmClient::shouldRetry(int status_code) const {
    // Retry on rate limit (429) and server errors (5xx)
    return status_code == 429 || (status_code >= 500 && status_code < 600);
}
```

### Base64 Encoding

```cpp
// Base64 encoding table
static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string VlmClient::encodeImageBase64(const cv::Mat& image) const {
    // Resize if too large (reduce API token cost)
    cv::Mat resized = image;
    int max_dim = 2048;  // Max dimension to keep token usage reasonable
    if (image.cols > max_dim || image.rows > max_dim) {
        float scale = static_cast<float>(max_dim) / std::max(image.cols, image.rows);
        cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    }

    // Encode to JPEG
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    cv::imencode(".jpg", resized, buffer, params);

    // Base64 encode
    std::string base64;
    base64.reserve((buffer.size() * 4) / 3 + 4);

    size_t i = 0;
    while (i < buffer.size()) {
        uint32_t octet_a = i < buffer.size() ? buffer[i++] : 0;
        uint32_t octet_b = i < buffer.size() ? buffer[i++] : 0;
        uint32_t octet_c = i < buffer.size() ? buffer[i++] : 0;

        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        base64 += base64_chars[(triple >> 18) & 0x3F];
        base64 += base64_chars[(triple >> 12) & 0x3F];
        base64 += (i > buffer.size() + 1) ? '=' : base64_chars[(triple >> 6) & 0x3F];
        base64 += (i > buffer.size()) ? '=' : base64_chars[triple & 0x3F];
    }

    return base64;
}
```

### ImageAnnotator Design

```cpp
// src/detection/ImageAnnotator.h
#pragma once

#include <opencv2/opencv.hpp>
#include "detection/DefectTypes.h"

class ImageAnnotator {
public:
    ImageAnnotator() = default;

    // Annotate image with defect bounding boxes and labels
    void annotateImage(cv::Mat& image, const std::vector<Defect>& defects);

    // Configuration
    void setFontScale(float scale) { font_scale_ = scale; }
    void setLineThickness(int thickness) { line_thickness_ = thickness; }

    // Save annotated image
    static bool saveAnnotatedImage(const cv::Mat& image, const std::string& path);

private:
    cv::Scalar getColorForSeverity(const std::string& severity) const;

    float font_scale_ = 0.6f;
    int line_thickness_ = 2;
};
```

### CMake Additions

```cmake
# ============================================
# Detection (Story 1-8)
# ============================================

# Detection library (VlmClient, ImageAnnotator, DefectTypes)
add_library(detection
    src/detection/VlmClient.cpp
    src/detection/ImageAnnotator.cpp
    src/detection/DefectTypes.cpp
)
target_include_directories(detection PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(detection
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# DetectionSim executable
add_executable(detection_sim
    sim/detection_sim/DetectionSim.cpp
    sim/detection_sim/main.cpp
)
target_link_libraries(detection_sim
    detection
    plan
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

# Update g1_inspector to include detection
target_link_libraries(g1_inspector
    # ... existing libraries ...
    detection
)

# Detection unit tests
if(GTest_FOUND)
    add_executable(test_detection test/test_detection.cpp)
    target_link_libraries(test_detection
        detection
        GTest::gtest_main
    )
    add_test(NAME test_detection COMMAND test_detection)
endif()
```

### Directory Structure

```
src/detection/              # NEW DIRECTORY
├── DefectTypes.h           # Defect struct and enum
├── DefectTypes.cpp         # JSON serialization implementation
├── VlmClient.h             # VLM client interface
├── VlmClient.cpp           # HTTP implementation
├── ImageAnnotator.h        # Annotation interface
└── ImageAnnotator.cpp      # Drawing implementation

sim/detection_sim/          # NEW DIRECTORY
├── DetectionSim.h          # Test harness interface
├── DetectionSim.cpp        # Test harness implementation
└── main.cpp                # CLI entry point

test/
├── test_detection.cpp      # NEW: Unit tests
└── MockVlmClient.h         # NEW: Mock client for testing
```

### MockVlmClient for Unit Testing

```cpp
// test/MockVlmClient.h
#pragma once

#include "detection/VlmClient.h"

class MockVlmClient : public VlmClient {
public:
    MockVlmClient() : VlmClient("mock-api-key") {}

    std::vector<Defect> analyzeImage(
        const cv::Mat& image,
        const std::string& plan_context,
        const Pose2D& pose
    ) override {
        call_count_++;
        last_image_size_ = {image.cols, image.rows};
        last_plan_context_ = plan_context;
        last_pose_ = pose;
        return mock_defects_;
    }

    // Test setup
    void setMockDefects(const std::vector<Defect>& defects) { mock_defects_ = defects; }
    void clearMockDefects() { mock_defects_.clear(); }

    // Test verification
    int getCallCount() const { return call_count_; }
    cv::Size getLastImageSize() const { return last_image_size_; }
    std::string getLastPlanContext() const { return last_plan_context_; }
    Pose2D getLastPose() const { return last_pose_; }
    void resetCallCount() { call_count_ = 0; }

private:
    std::vector<Defect> mock_defects_;
    int call_count_ = 0;
    cv::Size last_image_size_;
    std::string last_plan_context_;
    Pose2D last_pose_;
};
```

### ImageCapture Integration Pattern

This section shows how to integrate VLM detection with captured images from Story 1-7.

```cpp
// Example: Processing a captured inspection session
#include "capture/ImageCapture.h"
#include "detection/VlmClient.h"
#include "detection/ImageAnnotator.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>

struct SessionAnalysisResult {
    std::string session_id;
    int images_processed = 0;
    int total_defects = 0;
    std::vector<std::pair<std::string, std::vector<Defect>>> image_defects;
};

// Load ImageMetadata from JSON file (matches Story 1-7 format)
ImageMetadata loadImageMetadata(const std::string& json_path) {
    std::ifstream file(json_path);
    nlohmann::json j;
    file >> j;

    ImageMetadata meta;
    meta.image_path = j.value("image_path", "");
    meta.timestamp_ms = j.value("timestamp_ms", 0LL);
    meta.robot_pose.x = j["robot_pose"].value("x", 0.0f);
    meta.robot_pose.y = j["robot_pose"].value("y", 0.0f);
    meta.robot_pose.theta = j["robot_pose"].value("theta", 0.0f);
    meta.plan_coords.x = j["plan_coords"].value("x", 0.0f);
    meta.plan_coords.y = j["plan_coords"].value("y", 0.0f);
    return meta;
}

// Analyze all images from an ImageCapture session
SessionAnalysisResult analyzeInspectionSession(
    const std::string& session_dir,
    VlmClient& vlm,
    const std::string& plan_context = "finishes"
) {
    SessionAnalysisResult result;
    result.session_id = std::filesystem::path(session_dir).filename().string();

    std::string images_dir = session_dir + "/images";
    ImageAnnotator annotator;

    // Create output directory for annotated images
    std::filesystem::create_directories(session_dir + "/annotated");

    // Process each image in the session
    for (const auto& entry : std::filesystem::directory_iterator(images_dir)) {
        if (entry.path().extension() != ".jpg") continue;

        std::string img_path = entry.path().string();
        std::string json_path = img_path.substr(0, img_path.size() - 4) + ".json";

        // Load image and metadata
        cv::Mat image = cv::imread(img_path);
        if (image.empty()) continue;

        ImageMetadata meta = loadImageMetadata(json_path);

        // Analyze with VLM
        auto defects = vlm.analyzeImage(image, plan_context, meta.robot_pose);

        if (!defects.empty()) {
            // Annotate image with defects
            annotator.annotateImage(image, defects);

            // Save annotated image
            std::string annotated_path = session_dir + "/annotated/" +
                entry.path().filename().string();
            cv::imwrite(annotated_path, image);

            result.image_defects.push_back({meta.image_path, defects});
            result.total_defects += defects.size();
        }

        result.images_processed++;

        // Rate limit: 1 second between API calls
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return result;
}

// Save analysis results to JSON
void saveAnalysisResults(const SessionAnalysisResult& result, const std::string& output_path) {
    nlohmann::json j;
    j["session_id"] = result.session_id;
    j["images_processed"] = result.images_processed;
    j["total_defects"] = result.total_defects;

    j["defects_by_image"] = nlohmann::json::array();
    for (const auto& [img_path, defects] : result.image_defects) {
        nlohmann::json img_entry;
        img_entry["image"] = img_path;
        img_entry["defects"] = nlohmann::json::array();
        for (const auto& d : defects) {
            img_entry["defects"].push_back({
                {"id", d.id},
                {"type", defectTypeToString(d.type)},
                {"description", d.description},
                {"confidence", d.confidence},
                {"severity", d.severity}
            });
        }
        j["defects_by_image"].push_back(img_entry);
    }

    std::ofstream file(output_path);
    file << j.dump(2);
}

// Usage in main.cpp or detection_sim
void processInspectionSession(const std::string& session_path) {
    VlmClient::globalInit();

    std::string api_key = std::getenv("ANTHROPIC_API_KEY") ? std::getenv("ANTHROPIC_API_KEY") : "";
    if (api_key.empty()) {
        std::cerr << "ANTHROPIC_API_KEY not set" << std::endl;
        return;
    }

    VlmClient vlm(api_key);
    auto result = analyzeInspectionSession(session_path, vlm, "finishes");

    std::cout << "Processed " << result.images_processed << " images" << std::endl;
    std::cout << "Found " << result.total_defects << " defects" << std::endl;

    saveAnalysisResults(result, session_path + "/analysis_results.json");

    VlmClient::globalCleanup();
}
```

### Token Usage Estimation

**Formula:** `tokens = (width px × height px) / 750`

| Image Size | Approx Tokens |
|------------|---------------|
| 640×480    | ~410 |
| 1280×720   | ~1,230 |
| 1920×1080  | ~2,765 |
| 2048×2048 (max recommended) | ~5,592 |

**Recommendation:** Resize images to max 2048px on longest side to balance quality vs cost.

### API Rate Limits

Anthropic API has rate limits that vary by tier. Handle gracefully:
- **Rate limit (429):** Retry with exponential backoff (handled in httpPost)
- **Tier 1:** ~60 requests/minute, ~60K tokens/minute
- **Tier 2+:** Higher limits

For large inspections with many images:
- Process images sequentially, not in parallel
- Add 1-2 second delay between requests to stay under limits
- Consider batching multiple images if session has >50 images

### Confidence Filtering Implementation

```cpp
std::vector<Defect> VlmClient::filterByConfidence(std::vector<Defect>& defects) const {
    defects.erase(
        std::remove_if(defects.begin(), defects.end(),
            [this](const Defect& d) { return d.confidence < confidence_threshold_; }),
        defects.end()
    );
    return defects;
}

std::vector<Defect> VlmClient::parseResponse(const std::string& json_response) {
    std::vector<Defect> defects;
    try {
        auto j = nlohmann::json::parse(json_response);

        // Extract content text from response
        if (!j.contains("content") || j["content"].empty()) {
            last_error_ = "No content in response";
            return {};
        }

        std::string text = j["content"][0]["text"].get<std::string>();

        // Parse the JSON from the text
        auto result = nlohmann::json::parse(text);

        if (result.contains("defects")) {
            for (const auto& d : result["defects"]) {
                Defect defect;
                defect.id = d.value("id", "unknown");
                defect.type = stringToDefectType(d.value("type", "QUALITY_ISSUE"));
                defect.description = d.value("description", "");
                defect.confidence = d.value("confidence", 0.0f);
                defect.severity = d.value("severity", "medium");
                defect.trade = d.value("trade", "finishes");

                if (d.contains("image_location")) {
                    defect.image_loc.x = d["image_location"].value("x", 0.0f);
                    defect.image_loc.y = d["image_location"].value("y", 0.0f);
                }

                if (d.contains("bounding_box")) {
                    defect.bbox_x = d["bounding_box"].value("x", 0);
                    defect.bbox_y = d["bounding_box"].value("y", 0);
                    defect.bbox_width = d["bounding_box"].value("width", 0);
                    defect.bbox_height = d["bounding_box"].value("height", 0);
                }

                defects.push_back(defect);
            }
        }

        // Track token usage
        if (j.contains("usage")) {
            tokens_used_ = j["usage"].value("input_tokens", 0) + j["usage"].value("output_tokens", 0);
        }

    } catch (const nlohmann::json::exception& e) {
        last_error_ = std::string("JSON parse error: ") + e.what();
        return {};
    }

    // Apply confidence filtering
    return filterByConfidence(defects);
}
```

### Error Handling Strategy

```cpp
std::vector<Defect> VlmClient::analyzeImage(
    const cv::Mat& image,
    const std::string& plan_context,
    const Pose2D& pose
) {
    if (image.empty()) {
        last_error_ = "Empty image provided";
        return {};
    }

    if (api_key_.empty()) {
        last_error_ = "API key not set";
        return {};
    }

    // Build request
    std::string base64_img = encodeImageBase64(image);
    std::string prompt = buildPrompt(plan_context, pose);

    nlohmann::json request = {
        {"model", model_},
        {"max_tokens", 4096},
        {"messages", {{
            {"role", "user"},
            {"content", {
                {{"type", "image"}, {"source", {{"type", "base64"}, {"media_type", "image/jpeg"}, {"data", base64_img}}}},
                {{"type", "text"}, {"text", prompt}}
            }}
        }}}
    };

    // Send request
    std::string response = httpPost(request.dump());

    if (response.empty()) {
        std::cerr << "[VLM] Request failed: " << last_error_ << std::endl;
        return {};
    }

    // Parse response
    return parseResponse(response);
}
```

---

## Verification Commands

```bash
# Set API key
export ANTHROPIC_API_KEY="your-api-key-here"

# Build (inside Docker container)
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests (works without API key - uses mocks)
./test_detection               # All tests pass

# Test with dry-run (no API calls)
./detection_sim --images ../test_data/defect_samples/ --output outputs/ --dry-run
# Uses mock responses to test pipeline without API costs

# Run detection simulation on test images (requires API key)
./detection_sim --images ../test_data/defect_samples/ --output outputs/

# Process an ImageCapture session from Story 1-7
./detection_sim --session ../data/inspections/insp_001/ --output outputs/

# Check results
cat outputs/defects.json
# [
#   {"id": "def_001", "type": "QUALITY_ISSUE", "description": "Scratch on tile",
#    "image_location": {"x": 320, "y": 240}, "confidence": 0.87},
#   ...
# ]

ls outputs/annotated/
# img_00000001_annotated.jpg  img_00000003_annotated.jpg
```

### Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| Unit tests | `./test_detection` | Exit 0, all pass |
| Dry-run mode | `detection_sim --dry-run` | Mock defects returned, no API calls |
| API connectivity | `detection_sim --images test_data/` | Defects returned |
| Session processing | `detection_sim --session data/inspections/insp_001/` | ImageCapture images processed |
| Base64 encoding | Unit test | Valid base64 output |
| Retry logic | Unit test with mock | Retries on 429/5xx |
| JSON parsing | Unit test | Defects parsed correctly |
| Confidence filtering | Unit test | Low confidence defects removed |
| Annotation | Check output images | Bounding boxes visible |
| No API key | Unset ANTHROPIC_API_KEY | Graceful error, no crash |
| Curl lifecycle | Multiple VlmClient instances | No memory leaks |

---

## Previous Story Intelligence

### From Story 1-7 (Visual Capture)

**Key learnings:**
- ImageCapture saves images to `data/inspections/{session_id}/images/`
- Each image has JSON metadata sidecar with pose information
- PlanCorrelator provides coordinate transforms
- Async I/O pattern using `std::async` for non-blocking operations

**Files to reference:**
- `src/capture/ImageCapture.h` - ImageMetadata struct with pose data
- `src/capture/PlanCorrelator.h` - Coordinate transform API

**Code patterns established:**
- `#pragma once` for headers
- PascalCase classes, camelCase methods
- OpenCV for image I/O
- nlohmann/json serialization with manual to_json/from_json for nested types
- Async operations to avoid blocking main loop

### From Story 1-6 (State Machine + CLI)

**Key learnings:**
- CLI argument parsing pattern in main.cpp
- PlanManager provides plan context for trade types
- State machine integration pattern

### From Architecture Document

**VLM Integration Pattern (Section 4.7):**
```cpp
// Simple HTTP POST to Anthropic API
std::string analyze_image(const cv::Mat& image) {
    std::string base64_img = base64_encode(image);
    // ... build request, send to API, parse response
}
```

**No ML server needed.** The foundation model runs on Anthropic's infrastructure.

---

## Dependencies on Previous Stories

**Story 1-1 (Project Setup):**
- CMakeLists.txt base configuration
- curl dependency already configured
- nlohmann_json dependency

**Story 1-7 (Visual Capture):**
- ImageMetadata struct for pose information
- Image storage directory structure
- Integration point for analyzing captured images

---

## Test Data Requirements

Create test images for detection_sim:
- `test_data/defect_samples/` directory with sample construction images
- Include images with known defects for validation
- Include clean images to verify false positive handling

Ground truth JSON for evaluation (optional):
```json
{
  "images": [
    {
      "filename": "img_001.jpg",
      "expected_defects": [
        {"type": "QUALITY_ISSUE", "region": {"x": 300, "y": 200, "w": 100, "h": 100}}
      ]
    }
  ]
}
```

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **VlmClient class** | Images sent to Claude API, defects returned |
| **DefectTypes** | JSON serialization working |
| **ImageAnnotator** | Bounding boxes drawn on images |
| **DetectionSim** | Batch processing test images |
| **Unit tests** | test_detection passes |

### Demo Script (Run This When Done)

```bash
# Inside Docker container
cd /workspace
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests (no API key needed - uses mocks)
./test_detection

# Test dry-run mode (no API calls)
mkdir -p test_data/defect_samples
# Add some test images to test_data/defect_samples/
./detection_sim --images ../test_data/defect_samples/ --output outputs/ --dry-run

# Test with real API (requires API key)
export ANTHROPIC_API_KEY="sk-..."
./detection_sim --images ../test_data/defect_samples/ --output outputs/

# Process an ImageCapture session from Story 1-7
# (First run an inspection with g1_inspector to create session)
./detection_sim --session ../data/inspections/insp_001/ --output outputs/

# Verify results
cat outputs/defects.json
ls outputs/annotated/
cat outputs/analysis_results.json  # If processing a session
```

**SUCCESS CRITERIA:** Story 1.8 is DONE when:
1. `./test_detection` exits with code 0 (all tests pass)
2. Images successfully sent to Claude API
3. Defects returned with type, description, location, confidence
4. Annotated images generated with bounding boxes
5. API errors handled gracefully (no crashes on failure)
6. detection_sim produces outputs/defects.json and annotated images
7. `--dry-run` mode works for testing without API calls
8. `--session` option processes ImageCapture session directories
9. Confidence filtering removes low-confidence defects

---

## References

- [Source: docs/architecture.md#4.7-Detection] - VlmClient design
- [Source: docs/epics.md#story-8] - Original story requirements
- [Source: docs/sprint-artifacts/1-7-visual-capture.md] - ImageCapture patterns
- [External: Anthropic Claude API Vision](https://platform.claude.com/docs/en/build-with-claude/vision) - API specification
- [External: libcurl C API](https://curl.se/libcurl/c/) - HTTP client
- [External: nlohmann/json](https://github.com/nlohmann/json) - JSON serialization

---

## Dev Agent Record

### Context Reference

This story file serves as the complete implementation context.

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

None

### Completion Notes List

- Story created by create-story workflow with comprehensive VLM integration guide
- 2025-12-05: Validation improvements applied (validate-create-story):
  - CRITICAL-1 FIXED: Added ImageCapture integration pattern with full example code
  - CRITICAL-2 FIXED: Updated model name to full format `claude-sonnet-4-5-20250514`
  - CRITICAL-3 FIXED: Added `curl_global_init()`/`curl_global_cleanup()` lifecycle management
  - CRITICAL-4 FIXED: Changed `cv::Point2f` to `Point2D` for type consistency
  - ENHANCEMENT-1 FIXED: Added `--dry-run` and `--session` CLI options
  - ENHANCEMENT-2 FIXED: Added MockVlmClient class for unit testing
  - ENHANCEMENT-3 FIXED: Added API rate limits documentation section
  - ENHANCEMENT-4 FIXED: Added DefectTypes.cpp to file list and CMake
  - ENHANCEMENT-5 FIXED: Added thread safety notes for VlmClient
  - ENHANCEMENT-6 FIXED: Added confidence filtering implementation
  - OPT: Made VlmClient methods protected for MockVlmClient inheritance
  - OPT: Added virtual destructor and analyzeImage for mocking support
- 2025-12-06: Implementation completed (dev-story workflow):
  - All 9 tasks implemented with 25 unit tests
  - VlmClient: Full Claude API integration with base64 encoding, structured prompts, retry logic
  - ImageAnnotator: Severity-based color coding (red/orange/green), bounding boxes, labels
  - DetectionSim: CLI tool with --dry-run, --session options, JSON output
  - All tests pass: 10/10 test suites (100% pass rate)
  - detection_sim verified with dry-run mode producing correct defects.json and annotated images
  - Full project builds successfully with no regressions
- 2025-12-05: Code review fixes applied (code-review workflow):
  - HIGH-1 FIXED: Implemented full `evaluate()` function with precision/recall/F1/IoU metrics
  - HIGH-2 FIXED: Added E2E test script `test/test_e2e_story_1_8.sh` + registered in CMake
  - MEDIUM-1 FIXED: Added 4 proper confidence filtering tests via TestableVlmClient helper
  - MEDIUM-2 FIXED: Corrected base64 padding logic for all input sizes
  - MEDIUM-3 FIXED: Added mutex-protected thread-safe curl initialization
  - MEDIUM-4 FIXED: Added VLM response logging for debugging non-JSON outputs

### File List

**New files created:**
- `src/detection/DefectTypes.h` - Defect struct and enum definitions
- `src/detection/DefectTypes.cpp` - JSON serialization implementation
- `src/detection/VlmClient.h` - VLM client interface with Claude API
- `src/detection/VlmClient.cpp` - HTTP/curl implementation with retry logic
- `src/detection/ImageAnnotator.h` - Image annotation interface
- `src/detection/ImageAnnotator.cpp` - Bounding box drawing implementation
- `sim/detection_sim/DetectionSim.h` - Test harness interface
- `sim/detection_sim/DetectionSim.cpp` - Batch processing implementation with evaluate()
- `sim/detection_sim/main.cpp` - CLI entry point with --dry-run support
- `test/test_detection.cpp` - 29 unit tests for detection module (25 original + 4 confidence filtering)
- `test/MockVlmClient.h` - Mock client for unit testing
- `test/test_e2e_story_1_8.sh` - End-to-end test script for detection pipeline

**Modified files:**
- `CMakeLists.txt` - Added detection library, detection_sim executable, test_detection, test_e2e_story_1_8
