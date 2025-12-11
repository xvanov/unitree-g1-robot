# Story 2.3: End-to-End Replay Test

**Status:** ready-for-dev

---

## Quick Reference

**New files to create:**
- `test/e2e/E2EReplayTest.h` / `.cpp` - E2E test framework orchestrator
- `test/e2e/TestScenario.h` / `.cpp` - Test scenario definitions and JSON parsing
- `test/e2e/GroundTruthValidator.h` / `.cpp` - Compare estimated vs actual results
- `test/e2e/MockVlmClient.h` / `.cpp` - VLM mock for CI testing without API calls
- `test/test_e2e_replay.cpp` - GTest-based E2E test suite
- `test/test_e2e_story_2_3.sh` - Shell script E2E tests
- `data/test_scenarios/sample_scenario.json` - Sample test scenario
- `data/test_scenarios/README.md` - Documentation for creating test scenarios

**Files to modify:**
- `CMakeLists.txt` - Add `e2e_test` library, link replay/slam/detection, add test executable
- `src/slam/Localizer.h` - Extend with LocalizationMode enum and scan matching mode
- `src/slam/Localizer.cpp` - Add actual scan matching algorithm
- `src/main.cpp` - Add `--e2e-test <scenario.json>` CLI option

**Key classes:**
| Class | Purpose | Location |
|-------|---------|----------|
| `E2EReplayTest` | Orchestrates full pipeline test with recorded data | `test/e2e/` |
| `TestScenario` | Defines expected results (defects, positions, coverage) | `test/e2e/` |
| `GroundTruthValidator` | Compares pipeline outputs against ground truth | `test/e2e/` |
| `MockVlmClient` | VLM mock for fast CI testing (inherits from `VlmClient`) | `test/e2e/` |
| `ScanMatchingLocalizer` | Real localization with scan correlation | `src/slam/` |

**Key function signatures:**
```cpp
// E2EReplayTest - main test orchestrator
struct E2ETestResult {
    bool passed;
    float mean_position_error;
    float max_position_error;
    float detection_rate;
    float precision;
    std::string report_markdown;
};

E2ETestResult E2EReplayTest::run(const TestScenario& scenario);
void E2EReplayTest::setLocalizationMode(LocalizationMode mode);
void E2EReplayTest::setMockVlm(bool use_mock);
void E2EReplayTest::setHeadless(bool headless);

// GroundTruthValidator - result comparison
ValidationResult GroundTruthValidator::validateLocalization(
    const std::vector<PoseEstimate>& estimated,
    const std::vector<Pose2D>& ground_truth,
    float tolerance_meters, float tolerance_radians);

ValidationResult GroundTruthValidator::validateDefects(
    const std::vector<Defect>& detected,
    const std::vector<ExpectedDefect>& expected,
    float position_tolerance);

// Localizer with scan matching
enum class LocalizationMode {
    PASS_THROUGH,    // Just use odometry (current behavior)
    SCAN_MATCHING    // Correlate scans against map
};

Pose2D Localizer::localize(const LidarScan& scan, const Pose2D& odometry_delta);
void Localizer::setMode(LocalizationMode mode);
void Localizer::setMap(const OccupancyGrid& map);
```

**Primary acceptance criteria:** AC1 (E2E test runs), AC2 (localization validated), AC3 (defect detection validated), AC4 (deterministic with seed)

**Prerequisites:**
- Story 2-1 (Teleop + Sensor Recording) for recording infrastructure
- Story 2-2 (Replay System) for `ReplaySensorSource`, `SensorReplayer`, `ReplayController`
- Story 1-3 (SLAM Core) for `GridMapper`, `OccupancyGrid`
- Story 1-8 (VLM Detection) for `VlmClient`, `Defect` types

**Quick Start Implementation Order:**
1. Extend Localizer with scan matching (Task 1)
2. Define TestScenario format (Task 2)
3. Implement GroundTruthValidator (Task 3)
4. Implement E2EReplayTest orchestrator (Task 4)
5. Add MockVlmClient for CI (Task 5)
6. Wire up CLI and GTest (Tasks 6-7)
7. Add E2E test script (Task 8)

---

## Story

As a **developer**,
I want **an end-to-end test using real recorded data**,
So that **I can validate the full inspection pipeline before deploying to the robot**.

---

## Acceptance Criteria

1. **AC1:** E2E test runs full pipeline: replay -> localization -> capture -> detection -> report
2. **AC2:** Localization accuracy validated: estimated position within 0.3m of ground truth
3. **AC3:** Defect detection validated: >80% recall on known defects in test scenario
4. **AC4:** Deterministic results with seeded random (same seed = same results)
5. **AC5:** Test scenarios defined in JSON configuration files (not hardcoded)
6. **AC6:** Pass/fail criteria clearly defined with tolerance thresholds
7. **AC7:** Test report generated showing: accuracy metrics, detected vs expected defects, coverage
8. **AC8:** Works with recordings from Story 2-1 format (`sensors.bin.zst`, `metadata.json`)
9. **AC9:** Can run in CI/CD (no GUI required, headless mode, mock VLM)
10. **AC10:** Failing tests provide actionable diagnostics (what went wrong, where)

---

## Technical Design

### Integration Points

| Component | Uses | Provides |
|-----------|------|----------|
| `E2EReplayTest` | ReplaySensorSource, Localizer, VlmClient, TestScenario | E2ETestResult with pass/fail |
| `GroundTruthValidator` | PoseEstimate, Defect, TestScenario | ValidationResult with metrics |
| `ScanMatchingLocalizer` | OccupancyGrid, LidarScan | Corrected Pose2D |
| `MockVlmClient` | TestScenario.expected_defects | Defects at expected locations |

### E2E Test Flow

```
                    TestScenario (JSON)
                           |
                           v
┌──────────────────────────────────────────────────────────────┐
│                     E2EReplayTest                             │
│                                                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ Load        │  │ Load        │  │ Load Recording      │  │
│  │ Scenario    │  │ Plan Map    │  │ (SensorReplayer)    │  │
│  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘  │
│         │                │                    │              │
│         └────────────────┼────────────────────┘              │
│                          v                                   │
│                 ┌─────────────────┐                          │
│                 │ ReplaySensor-   │                          │
│                 │ Source          │                          │
│                 │ (ISensorSource) │                          │
│                 └────────┬────────┘                          │
│                          │                                   │
│         ┌────────────────┼────────────────┐                  │
│         v                v                v                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ Localizer   │  │ VlmClient/  │  │ Coverage    │         │
│  │ (Scan Match)│  │ MockVlm     │  │ Tracker     │         │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘         │
│         │                │                │                  │
│         v                v                v                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ Pose        │  │ Detected    │  │ Coverage    │         │
│  │ Estimates   │  │ Defects     │  │ Percent     │         │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘         │
│         │                │                │                  │
│         └────────────────┼────────────────┘                  │
│                          v                                   │
│                 ┌─────────────────┐                          │
│                 │ GroundTruth-    │                          │
│                 │ Validator       │                          │
│                 └────────┬────────┘                          │
│                          v                                   │
│                 ┌─────────────────┐                          │
│                 │ E2ETestResult   │                          │
│                 │ (pass/fail)     │                          │
│                 └─────────────────┘                          │
└──────────────────────────────────────────────────────────────┘
```

### Scan Matching Localization

Grid-based scan matching against built map:

```cpp
class ScanMatchingLocalizer {
    OccupancyGrid map_;
    Pose2D current_pose_;

public:
    void setMap(const OccupancyGrid& map) { map_ = map; }

    Pose2D localize(const LidarScan& scan, const Pose2D& odometry_delta) {
        // Predict pose from odometry
        Pose2D predicted = applyDelta(current_pose_, odometry_delta);

        // Search around predicted pose for best match
        float best_score = 0;
        Pose2D best_pose = predicted;

        // Grid search (coarse)
        for (float dx = -0.2f; dx <= 0.2f; dx += 0.05f) {
            for (float dy = -0.2f; dy <= 0.2f; dy += 0.05f) {
                for (float dth = -0.1f; dth <= 0.1f; dth += 0.025f) {
                    Pose2D test_pose{predicted.x + dx, predicted.y + dy, predicted.theta + dth};
                    float score = computeMatchScore(scan, test_pose);
                    if (score > best_score) {
                        best_score = score;
                        best_pose = test_pose;
                    }
                }
            }
        }

        current_pose_ = best_pose;
        return current_pose_;
    }

private:
    float computeMatchScore(const LidarScan& scan, const Pose2D& pose) {
        // Transform scan to world frame at pose
        // Count how many endpoints hit occupied cells in map
        float hits = 0;
        float total = 0;

        float angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges.size();

        for (size_t i = 0; i < scan.ranges.size(); i++) {
            if (scan.ranges[i] <= 0 || scan.ranges[i] > 10.0f) continue;

            float angle = scan.angle_min + i * angle_increment + pose.theta;
            float x = pose.x + scan.ranges[i] * cos(angle);
            float y = pose.y + scan.ranges[i] * sin(angle);

            if (isOccupied(x, y)) {
                hits += 1.0f;
            }
            total += 1.0f;
        }

        return (total > 0) ? (hits / total) : 0.0f;
    }

    bool isOccupied(float x, float y) const {
        int gx = static_cast<int>((x - map_.origin.x) / map_.resolution);
        int gy = static_cast<int>((y - map_.origin.y) / map_.resolution);
        if (gx < 0 || gx >= map_.width || gy < 0 || gy >= map_.height) return false;
        // Log-odds > 0 means occupied
        return map_.log_odds[gy * map_.width + gx] > 0.0f;
    }
};
```

### Test Scenario JSON Format

```json
{
  "name": "test_room_001",
  "recording_path": "data/recordings/room_001",
  "plan_path": "data/plans/room_001.png",
  "random_seed": 42,

  "localization": {
    "checkpoints": [
      {"time_s": 10.0, "pose": {"x": 1.0, "y": 0.5, "theta": 0.0}, "tolerance_m": 0.2},
      {"time_s": 30.0, "pose": {"x": 3.0, "y": 2.0, "theta": 1.57}, "tolerance_m": 0.3}
    ],
    "max_position_error_m": 0.3,
    "max_orientation_error_rad": 0.1
  },

  "defects": {
    "expected": [
      {
        "location": {"x": 2.5, "y": 1.2},
        "type": "QUALITY_ISSUE",
        "severity": "high",
        "tolerance_m": 0.5
      },
      {
        "location": {"x": 4.0, "y": 3.0},
        "type": "SAFETY_HAZARD",
        "severity": "medium",
        "tolerance_m": 0.5
      }
    ],
    "min_detection_rate": 0.8,
    "max_false_positive_rate": 0.2
  },

  "coverage": {
    "expected_percent": 85.0,
    "tolerance_percent": 5.0
  }
}
```

### Validation Result Structure

```cpp
struct LocalizationMetrics {
    float mean_position_error;      // Average over all checkpoints
    float max_position_error;       // Worst case
    float rmse_position;            // Root mean square error
    float mean_orientation_error;   // Average angular error
    float max_orientation_error;    // Worst angular error
    std::vector<std::string> checkpoint_failures;  // Details on failed checkpoints
};

struct DefectMetrics {
    int true_positives;
    int false_negatives;
    int false_positives;
    float detection_rate;     // TP / (TP + FN)
    float precision;          // TP / (TP + FP)
    std::vector<std::string> missed_defects;    // Description of FN
    std::vector<std::string> extra_detections;  // Description of FP
};

struct ValidationResult {
    bool passed;
    std::string summary;

    LocalizationMetrics localization;
    DefectMetrics defects;

    float actual_coverage;
    float expected_coverage;
};
```

### MockVlmClient for CI

Inherits from `VlmClient` (virtual methods support inheritance):

```cpp
class MockVlmClient : public VlmClient {
public:
    MockVlmClient() : VlmClient("") {}  // No API key needed

    void setExpectedDefects(const std::vector<ExpectedDefect>& defects) {
        expected_ = defects;
    }

    std::vector<Defect> analyzeImage(
        const cv::Mat& image,
        const std::string& plan_context,
        const Pose2D& pose
    ) override {
        // Return defects that would be visible from this location
        std::vector<Defect> visible;
        int seq = 0;
        for (const auto& exp : expected_) {
            float dist = std::sqrt(
                (pose.x - exp.location.x) * (pose.x - exp.location.x) +
                (pose.y - exp.location.y) * (pose.y - exp.location.y)
            );
            if (dist < 2.0f) {  // Within 2m viewing range
                Defect d;
                d.id = "mock_" + std::to_string(seq++);
                d.type = stringToDefectType(exp.type);
                d.description = "Mock defect for testing";
                d.plan_loc = exp.location;
                d.confidence = 0.9f;
                d.severity = exp.severity;
                visible.push_back(d);
            }
        }
        return visible;
    }

private:
    std::vector<ExpectedDefect> expected_;
};
```

---

## Tasks / Subtasks

- [ ] **Task 1: Implement Real Localization Algorithm** (AC: 2)
  - [ ] 1.1 Update `src/slam/Localizer.h`:
    - Add `LocalizationMode` enum (PASS_THROUGH, SCAN_MATCHING)
    - Add `setMode()`, `setMap()` methods
    - Add `localize(scan, odometry_delta)` method
    - Keep backward compatibility with existing `setOdometry()`/`getPose()` for PASS_THROUGH mode
  - [ ] 1.2 Implement scan matching in `src/slam/Localizer.cpp`:
    - Grid search around predicted pose (±0.2m position, ±0.1rad orientation)
    - Score = percentage of scan endpoints hitting occupied cells
    - Use `OccupancyGrid.log_odds > 0` for occupied test (see `src/slam/OccupancyGrid.h:19`)
    - Resolution matches OccupancyGrid (0.05m, see line 29)
  - [ ] 1.3 Add unit tests in `test/test_slam.cpp`:
    - Test scan matching finds correct pose offset
    - Test handles edge cases (empty scan, pose outside map)

- [ ] **Task 2: Define Test Scenario Format** (AC: 5, 6)
  - [ ] 2.1 Create `test/e2e/TestScenario.h`:
    - `ExpectedDefect` struct (location, type, severity, tolerance)
    - `LocalizationCheckpoint` struct (time_s, pose, tolerance)
    - `TestScenario` struct (name, paths, checkpoints, expected_defects, thresholds)
    - `loadScenario(path)` function to parse JSON
  - [ ] 2.2 Create `test/e2e/TestScenario.cpp`:
    - Use nlohmann/json for parsing (already in project, see `DefectTypes.h:5`)
    - Validate required fields, provide sensible defaults
    - Error on invalid/missing required fields
  - [ ] 2.3 Create `data/test_scenarios/sample_scenario.json`:
    - Minimal valid scenario that can run without real recording
    - Document all fields with comments

- [ ] **Task 3: Implement GroundTruthValidator** (AC: 2, 3, 6, 10)
  - [ ] 3.1 Create `test/e2e/GroundTruthValidator.h`:
    - `ValidationResult` struct with detailed metrics
    - `validateLocalization()` - compare poses at checkpoints
    - `validateDefects()` - match detected to expected
    - `validateCoverage()` - compare actual vs expected
    - `generateReport()` - markdown summary
  - [ ] 3.2 Create `test/e2e/GroundTruthValidator.cpp`:
    - Localization: compute position error as Euclidean distance
    - Orientation: handle wrap-around with `normalizeAngle()`
    - Defect matching: greedy nearest-neighbor within tolerance
    - Track TP, FN, FP with detailed descriptions for diagnostics
  - [ ] 3.3 Defect matching algorithm:
    - For each expected, find closest detected within tolerance
    - Type must match exactly
    - Severity can be same or higher (detected high matches expected medium)
    - Unmatched expected = false negative
    - Unmatched detected = false positive

- [ ] **Task 4: Implement E2EReplayTest Orchestrator** (AC: 1, 4, 8)
  - [ ] 4.1 Create `test/e2e/E2EReplayTest.h`:
    - `E2ETestResult` struct (passed, metrics, report_markdown)
    - `run(scenario)` - main test execution
    - `setLocalizationMode()`, `setMockVlm()`, `setHeadless()`
    - `setRandomSeed()` for determinism
  - [ ] 4.2 Create `test/e2e/E2EReplayTest.cpp`:
    - Load scenario, recording, and plan
    - Use `ReplaySensorSource` from `src/replay/` (ISensorSource interface)
    - Run pipeline loop until replay finished
    - Collect pose estimates at checkpoint times
    - Run detection on images (real or mock VLM)
    - Call validator at end
  - [ ] 4.3 Ensure determinism:
    - Set `srand(scenario.random_seed)` at start
    - Use same seed for any stochastic components
    - Document any sources of non-determinism

- [ ] **Task 5: Implement MockVlmClient** (AC: 9)
  - [ ] 5.1 Create `test/e2e/MockVlmClient.h`:
    - Inherit from `VlmClient` (virtual destructor and virtual `analyzeImage` support inheritance)
    - `setExpectedDefects()` - configure what to return
    - Override `analyzeImage()` to return expected defects near pose
  - [ ] 5.2 Create `test/e2e/MockVlmClient.cpp`:
    - Return defects within 2m of current pose
    - Add slight position noise for realism (seeded)
    - Track calls for test verification

- [ ] **Task 6: Create Test Report Generator** (AC: 7, 10)
  - [ ] 6.1 Add `generateMarkdownReport()` to `GroundTruthValidator`:
    ```markdown
    # E2E Test Report: test_room_001

    ## Summary
    - **Status:** PASSED / FAILED
    - **Recording:** data/recordings/room_001
    - **Duration:** 5:23
    - **Random Seed:** 42

    ## Localization Accuracy
    - Mean Position Error: 0.15m (threshold: 0.30m) OK
    - Max Position Error: 0.28m (threshold: 0.30m) OK
    - Orientation Error: 3.2deg (threshold: 6.0deg) OK

    ### Checkpoints
    | Time | Expected | Actual | Error | Status |
    |------|----------|--------|-------|--------|
    | 10.0s | (1.0, 0.5) | (1.12, 0.48) | 0.12m | OK |

    ## Defect Detection
    - Detection Rate: 85% (threshold: 80%) OK
    - Precision: 90%
    - True Positives: 17/20
    - False Positives: 2

    ### Defect Details
    | Expected | Detected | Distance | Status |
    |----------|----------|----------|--------|
    | Crack @ (2.5, 1.2) | Crack @ (2.4, 1.3) | 0.14m | OK |
    | Missing outlet @ (4.0, 3.0) | Not detected | - | FAIL |

    ## Coverage
    - Actual: 87%
    - Expected: 85%
    - Status: OK
    ```
  - [ ] 6.2 Write report to file if `--report-output` specified

- [ ] **Task 7: GTest Integration** (AC: 9)
  - [ ] 7.1 Create `test/test_e2e_replay.cpp`:
    ```cpp
    #include <gtest/gtest.h>
    #include "e2e/E2EReplayTest.h"
    #include "e2e/TestScenario.h"

    class E2EReplayTestFixture : public ::testing::Test {
    protected:
        void SetUp() override {
            test_.setLocalizationMode(LocalizationMode::SCAN_MATCHING);
            test_.setMockVlm(true);  // Use mock for CI speed
            test_.setHeadless(true);
        }
        E2EReplayTest test_;
    };

    TEST_F(E2EReplayTestFixture, SampleScenario_Passes) {
        auto scenario = loadScenario("data/test_scenarios/sample_scenario.json");
        auto result = test_.run(scenario);
        EXPECT_TRUE(result.passed) << result.report_markdown;
    }

    TEST_F(E2EReplayTestFixture, LocalizationAccuracy) {
        auto scenario = loadScenario("data/test_scenarios/sample_scenario.json");
        auto result = test_.run(scenario);
        EXPECT_LE(result.mean_position_error, scenario.localization.max_position_error_m)
            << "Position error too high: " << result.mean_position_error;
    }

    TEST_F(E2EReplayTestFixture, Determinism) {
        auto scenario = loadScenario("data/test_scenarios/sample_scenario.json");
        auto result1 = test_.run(scenario);
        auto result2 = test_.run(scenario);
        EXPECT_EQ(result1.detection_rate, result2.detection_rate)
            << "Results should be deterministic";
    }
    ```

- [ ] **Task 8: CMake Integration** (AC: all)
  - [ ] 8.1 Update `CMakeLists.txt`:
    ```cmake
    # E2E test library
    add_library(e2e_test
        test/e2e/E2EReplayTest.cpp
        test/e2e/TestScenario.cpp
        test/e2e/GroundTruthValidator.cpp
        test/e2e/MockVlmClient.cpp
    )
    target_include_directories(e2e_test PUBLIC ${CMAKE_SOURCE_DIR}/test)
    target_link_libraries(e2e_test
        replay
        slam
        detection
        nlohmann_json::nlohmann_json
    )

    # E2E test executable
    add_executable(test_e2e_replay test/test_e2e_replay.cpp)
    target_link_libraries(test_e2e_replay
        e2e_test
        GTest::gtest_main
    )
    add_test(NAME test_e2e_replay COMMAND test_e2e_replay)
    ```

- [ ] **Task 9: Shell Script E2E Tests** (AC: 9)
  - [ ] 9.1 Create `test/test_e2e_story_2_3.sh`:
    - Test 1: E2E test executable builds
    - Test 2: Can load sample test scenario JSON
    - Test 3: Runs with sample scenario (mock VLM)
    - Test 4: Generates test report
    - Test 5: Exit code reflects pass/fail

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Hardcode expected results in test code (use scenario JSON files)
- Require GUI for CI tests (must run headless with `--headless`)
- Skip localization testing (this is the core purpose of E2E tests)
- Use real VLM API in fast unit tests (use `MockVlmClient` for CI)
- Block on VLM calls in tight loop (async if needed for real API)

**MUST USE:**
- `VlmClient` virtual methods for inheritance (see `src/detection/VlmClient.h:20`)
- `ISensorSource` interface for sensor abstraction (see `src/sensors/ISensorSource.h`)
- `ReplaySensorSource` from Story 2-2 for feeding recorded data
- `OccupancyGrid` log-odds convention (>0 = occupied, see `src/slam/OccupancyGrid.h`)
- nlohmann/json for scenario parsing (already in project)
- GTest for test framework (already in project)

### Existing Code Patterns to Follow

**From Story 2-2 (Replay System):**
- `ReplaySensorSource` implements `ISensorSource` - use as sensor backend
- `getGroundTruthPose()` returns recorded pose for validation
- `PoseMode::ESTIMATED` allows external pose injection

**From Story 1-8 (VLM Detection):**
- `VlmClient` has virtual destructor and virtual `analyzeImage()` - safe to inherit
- `VlmClient::globalInit()` / `globalCleanup()` must bracket all usage
- `DefectType` enum: `LOCATION_ERROR`, `QUALITY_ISSUE`, `SAFETY_HAZARD`, `MISSING_ELEMENT`
- `Defect.plan_loc` is world coordinates in meters

**From Story 1-3 (SLAM Core):**
- `OccupancyGrid.log_odds` uses log-odds representation
- `log_odds > 0` means occupied (probability > 0.5)
- `resolution = 0.05f` (5cm per cell)
- `origin` is world position of grid (0,0)

### Signal Handler Integration

`main.cpp` uses `g_running` atomic for graceful shutdown (line 26 area):
```cpp
extern std::atomic<bool> g_running;  // Use this for E2E test loop control
```

### Running Without VLM API (CI Mode)

```bash
# Fast CI mode - mock VLM, headless
./test_e2e_replay --mock-vlm --headless

# With real VLM (requires API key)
ANTHROPIC_API_KEY="sk-..." ./test_e2e_replay
```

### Localizer Mode Selection

The current `Localizer` in `src/slam/Localizer.h` is pass-through only. Extend it:

```cpp
// Current (Story 1-3):
class Localizer {
    void setOdometry(const Pose2D& pose);
    Pose2D getPose() const;
};

// Extended (Story 2-3):
class Localizer {
    // Existing (backward compatible)
    void setOdometry(const Pose2D& pose);
    Pose2D getPose() const;

    // New for E2E testing
    void setMode(LocalizationMode mode);
    void setMap(const OccupancyGrid& map);
    Pose2D localize(const LidarScan& scan, const Pose2D& odometry_delta);

private:
    LocalizationMode mode_ = LocalizationMode::PASS_THROUGH;
    OccupancyGrid map_;
    Pose2D current_pose_;
};
```

### Creating Test Scenarios

**Step-by-step guide for users:**

1. **Record a session** (Story 2-1):
   ```bash
   ./g1_inspector --teleop gamepad --record test_room --plan room.png
   ```

2. **Walk through the room**, covering areas with known defects

3. **Note ground truth**:
   - Mark where you were at specific times (checkpoints)
   - Document actual defect locations on the plan

4. **Create scenario JSON** in `data/test_scenarios/`:
   ```json
   {
     "name": "test_room",
     "recording_path": "data/recordings/test_room",
     "plan_path": "data/plans/room.png",
     "random_seed": 42,
     "localization": {
       "checkpoints": [
         {"time_s": 30.0, "pose": {"x": 2.0, "y": 1.5, "theta": 1.57}, "tolerance_m": 0.3}
       ],
       "max_position_error_m": 0.3
     },
     "defects": {
       "expected": [
         {"location": {"x": 3.5, "y": 2.0}, "type": "QUALITY_ISSUE", "severity": "high", "tolerance_m": 0.5}
       ],
       "min_detection_rate": 0.8
     }
   }
   ```

5. **Run E2E test**:
   ```bash
   ./test_e2e_replay --scenario data/test_scenarios/test_room.json
   ```

---

## Directory Structure

```
test/e2e/
├── E2EReplayTest.h          # Main orchestrator interface
├── E2EReplayTest.cpp        # Orchestrator implementation
├── TestScenario.h           # Scenario struct and JSON parsing
├── TestScenario.cpp         # JSON loading implementation
├── GroundTruthValidator.h   # Validation interface
├── GroundTruthValidator.cpp # Validation and report generation
├── MockVlmClient.h          # VLM mock interface
└── MockVlmClient.cpp        # Mock implementation

test/
├── test_e2e_replay.cpp      # GTest-based E2E tests
└── test_e2e_story_2_3.sh    # Shell script E2E tests

data/test_scenarios/
├── sample_scenario.json     # Minimal valid scenario
└── README.md                # Documentation for scenario creation
```

---

## Verification Commands

```bash
# Build
cd build && cmake .. && make -j

# Run unit tests (including new Localizer tests)
./test_slam

# Run E2E tests (with mock VLM)
./test_e2e_replay

# Run with specific scenario
./test_e2e_replay --gtest_filter=*SampleScenario*

# Run shell E2E tests
../test/test_e2e_story_2_3.sh

# Generate detailed test report
./test_e2e_replay --report-output test_report.md

# Run in CI mode (no GUI, mock VLM, headless)
./test_e2e_replay --mock-vlm --headless
```

---

## Dependencies on Previous Stories

**Story 2-1 (Teleop + Sensor Recording):**
- Recording file format (`sensors.bin.zst`, `metadata.json`, `images/`)
- `RecordingTypes.h` for message types

**Story 2-2 (Replay System):**
- `SensorReplayer` for decoding recorded data
- `ReplaySensorSource` for ISensorSource implementation
- `ReplayController` for playback control
- Ground truth pose access via `getGroundTruthPose()`

**Story 1-3 (SLAM Core):**
- `GridMapper` for map building
- `OccupancyGrid` for scan matching
- `Localizer` base class (to be extended)

**Story 1-8 (VLM Defect Detection):**
- `VlmClient` base class with virtual methods
- `Defect` and `DefectType` types
- `VlmClient::globalInit()` / `globalCleanup()` pattern

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **Real localization algorithm** | Position error <0.3m on test scenario |
| **E2EReplayTest framework** | Full pipeline runs with recorded data |
| **GroundTruthValidator** | Compares results against expected with metrics |
| **MockVlmClient** | Fast CI testing without API calls |
| **Test scenario format** | JSON scenarios define expected results |
| **Test reports** | Clear pass/fail with actionable diagnostics |
| **CI-compatible tests** | Runs headless in automation |

---

## Success Criteria

**Story 2-3 is DONE when:**

1. E2E test runs full pipeline: replay -> localization -> detection -> report
2. Scan matching localization achieves <0.3m position error on sample scenario
3. Mock VLM detection achieves >80% recall on known defects
4. Tests are deterministic (same seed = same results)
5. Test report generated with clear pass/fail and metrics
6. Can run in CI without GUI or real hardware (`./test_e2e_replay --mock-vlm --headless`)
7. At least one sample test scenario created and passing

---

## References

- [Story 2-1](docs/sprint-artifacts/2-1-teleop-sensor-recording.md) - Recording infrastructure
- [Story 2-2](docs/sprint-artifacts/2-2-replay-system.md) - Replay system (ReplaySensorSource, SensorReplayer)
- [Story 1-3](docs/sprint-artifacts/1-3-slam-core.md) - SLAM and mapping (OccupancyGrid, Localizer)
- [Story 1-8](docs/sprint-artifacts/1-8-vlm-defect-detection.md) - Defect detection (VlmClient, Defect)
- [src/replay/ReplaySensorSource.h](src/replay/ReplaySensorSource.h) - ISensorSource implementation
- [src/slam/OccupancyGrid.h](src/slam/OccupancyGrid.h) - Log-odds grid format
- [src/detection/VlmClient.h](src/detection/VlmClient.h) - Virtual methods for inheritance

---

## Dev Agent Record

### Context Reference
This story file serves as the complete implementation context.

### Agent Model Used
Claude Opus 4.5

### Completion Notes List
- Story created by PM agent as final story in 3-story E2E testing arc
- This is the capstone that validates the entire inspection pipeline
- Enhanced 2025-12-06 by create-story workflow with:
  - Detailed implementation patterns from actual codebase
  - Integration points table with component relationships
  - Quick Start implementation order
  - Specific file references (e.g., `VlmClient.h:20` for virtual methods)
  - Code patterns from Stories 1-3, 1-8, 2-1, 2-2
  - MockVlmClient inherits from VlmClient for CI testing
  - Localizer extended with scan matching (not replacing pass-through mode)
  - Signal handler integration notes for graceful shutdown

### File List
*To be populated during implementation*
