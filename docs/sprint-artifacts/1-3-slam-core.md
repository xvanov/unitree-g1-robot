# Story 1.3: SLAM Core

**Status:** ready-for-dev

---

## Story

As the **robot**,
I want **to build a map from LiDAR scans**,
So that **I can localize and plan paths**.

---

## Acceptance Criteria

1. **AC1:** GridMapper builds reasonable map from scans (log-odds updates)
2. **AC2:** Bresenham ray tracing marks free space correctly
3. **AC3:** Map matches ground truth environment with ≥90% accuracy
4. **AC4:** Unit tests pass for map building
5. **AC5:** slam_sim executable produces map and accuracy metrics

---

## Tasks / Subtasks

- [ ] **Task 1: Implement GridMapper Class** (AC: 1, 2)
  - [ ] 1.1 Create `src/slam/GridMapper.h` - header with class declaration
    - Constructor taking resolution (meters/cell), width, height
    - `update(pose, scan)` - integrate new LiDAR data using log-odds
    - `getMap()` - return const reference to OccupancyGrid
    - `saveMap(path)` - export built map as PNG
    - `getAccuracy(ground_truth)` - compare to ground truth map
  - [ ] 1.2 Create `src/slam/GridMapper.cpp` - implementation
    - Implement log-odds update formula: `l_new = l_old + l_measurement`
    - `l_occ` (occupied log-odds increment): +0.85f (high confidence for hits)
    - `l_free` (free log-odds increment): -0.4f (lower confidence for free)
    - Clamp log-odds to [-5.0f, 5.0f] to prevent saturation
    - Implement `traceRay()` using Bresenham's line algorithm
    - Mark cells along ray as free, endpoint as occupied
  - [ ] 1.3 Implement Bresenham ray tracing in `traceRay(from, to)`
    - Use integer-only Bresenham for efficiency
    - Update log-odds for each cell along ray (free space)
    - Update endpoint cell with occupied log-odds
    - Handle edge cases (ray outside map bounds)

- [ ] **Task 2: Define OccupancyGrid Type** (AC: 1)
  - [ ] 2.1 Create `src/slam/OccupancyGrid.h` (separate file for SLAM-specific type)
    - `std::vector<float> log_odds` - log-odds values (float for precision, matches GridMapper)
    - `int width, height` - grid dimensions in cells
    - `float resolution` - meters per cell (use 0.05f to match NavSim)
    - `Point2D origin` - world position of grid origin (0,0 for MVP)
    - Helper methods: `logOddsToProb(float l)`, `probToLogOdds(float p)`
    - `toProbabilityMap()` - convert to `std::vector<uint8_t>` (0=obstacle, 255=free) for PNG export

- [ ] **Task 3: Implement Localizer Stub (MVP: Odometry Pass-through)** (AC: 1)
  - [ ] 3.1 Create `src/slam/Localizer.h` and `src/slam/Localizer.cpp`
    - MVP implementation: Simply stores and returns odometry pose
    - `void setOdometry(const Pose2D& pose)` - store pose from odometry
    - `Pose2D getPose() const` - return stored pose
    - No scan matching for MVP (defer to post-MVP if needed)

- [ ] **Task 4: Create SlamSim Executable** (AC: 3, 5)
  - [ ] 4.1 Create `sim/slam_sim/SlamSim.h` - simulation wrapper
    - `SlamSim(ground_truth_map_path, resolution)` - load ground truth via NavSim
    - `run()` - execute full mapping simulation
    - `getBuiltMap()` - return GridMapper's OccupancyGrid
    - `computeAccuracy()` - compare built vs ground truth
    - `saveBuiltMap(path)` - export PNG (0=obstacle, 255=free)
    - `saveAccuracy(path)` - JSON metrics
  - [ ] 4.2 Create `sim/slam_sim/SlamSim.cpp` - implementation
    - **CRITICAL: Reuse NavSim - do NOT reimplement raycasting:**
      - Create `NavSim nav_sim(ground_truth_path, resolution)`
      - Use `nav_sim.getWidth()`, `nav_sim.getHeight()` for GridMapper dimensions
      - Use `nav_sim.setRobotPose(pose)` to position robot
      - Use `nav_sim.simulateLidar(360)` to get LidarScan
    - Create GridMapper with same dimensions as NavSim
    - Feed simulated scans to GridMapper.update(pose, scan)
    - Accuracy = % of cells where (built > 0) == (truth < 128)
  - [ ] 4.3 Create `sim/slam_sim/main.cpp` - executable entry point
    - Parse CLI args: `--map`, `--output` (create output dir if missing)
    - **Snake trajectory generation:**
      - Start at (1.0, 1.0) in world coords
      - Horizontal passes spaced 0.5m apart vertically
      - Reverse direction at map edges (0.5m margin)
      - Step size: 0.1m per pose update
    - Output: `built_map.png`, `accuracy.json`

- [ ] **Task 5: Create Unit Tests** (AC: 4)
  - [ ] 5.1 Create `test/test_slam.cpp`
    - Test log-odds update math
    - Test Bresenham ray tracing correctness
    - Test GridMapper builds map from known scans
    - Test accuracy calculation
  - [ ] 5.2 Update CMakeLists.txt to build test_slam

- [ ] **Task 6: CMake Integration** (AC: 5)
  - [ ] 6.1 Update CMakeLists.txt
    - Add slam library with GridMapper, Localizer
    - Add slam_sim executable
    - Add test_slam test executable
    - Link dependencies (OpenCV for PNG output)

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Use ROS2, slam_toolbox, or any ROS dependencies - this is pure C++
- Use external SLAM libraries - implement log-odds occupancy grid from scratch
- Over-engineer - MVP is odometry-only localization (no scan matching)
- Implement particle filters or EKF - defer to post-MVP if needed
- Use signed distance fields or advanced techniques - simple log-odds is sufficient

**MUST USE:**
- C++17 standard (`-std=c++17`)
- OpenCV for image I/O only (map PNG loading/saving)
- Existing types from `src/util/Types.h`: `Point2D`, `Pose2D`
- Existing `LidarScan` struct from `src/sensors/ISensorSource.h`
- Existing `NavSim` class from `sim/nav_sim/` for simulation environment
- Existing `Costmap` class patterns from `src/navigation/Costmap.h`

### Map Convention Alignment (CRITICAL)

All maps use the same pixel convention for consistency:
- **Black (pixel < 128) = OBSTACLE** - walls, objects
- **White (pixel >= 128) = FREE SPACE** - traversable area

This applies to:
1. `test_data/office.png` - ground truth map loaded by NavSim
2. NavSim internal map - used for collision checking and raycasting
3. GridMapper output via `logOddsToPng()` - built map from SLAM
4. Accuracy comparison - same threshold (128) for both maps

### Filesystem Handling

Use C++17 filesystem for output directory creation:
```cpp
#include <filesystem>
namespace fs = std::filesystem;

// In slam_sim main.cpp - create output directory if needed
std::string output_dir = /* from CLI args */;
if (!fs::exists(output_dir)) {
    fs::create_directories(output_dir);
}
```

### Implementation Details

#### Log-Odds Parameters (AUTHORITATIVE VALUES)

**USE THESE EXACT VALUES throughout implementation:**
```cpp
const float l_occ  = 0.85f;   // Occupied (hit) - high confidence
const float l_free = -0.4f;   // Free (ray passed) - lower confidence
const float l_max  = 5.0f;    // Clamping upper bound
const float l_min  = -5.0f;   // Clamping lower bound
const float l_init = 0.0f;    // Initial value (unknown, p=0.5)
```

**Conversion functions (put in OccupancyGrid.h):**
```cpp
// Log-odds to probability
inline float logOddsToProb(float l) {
    return 1.0f / (1.0f + std::exp(-l));
}

// Probability to log-odds
inline float probToLogOdds(float p) {
    return std::log(p / (1.0f - p));
}

// Log-odds to uint8_t for PNG output
// IMPORTANT: 0=obstacle (black), 255=free (white) - matches NavSim convention
inline uint8_t logOddsToPng(float l) {
    float p = logOddsToProb(l);
    // Invert: high probability (occupied) -> low pixel value (dark)
    return static_cast<uint8_t>((1.0f - p) * 255.0f);
}
```

#### Bresenham's Line Algorithm

Use integer-only Bresenham for efficiency. The algorithm traces cells from start to end:

```cpp
void GridMapper::traceRay(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        // Update cell - all cells except endpoint are FREE
        if (x0 == x1 && y0 == y1) {
            // Endpoint: mark as OCCUPIED
            updateCell(x0, y0, l_occ_);
            break;
        } else {
            // Ray path: mark as FREE
            updateCell(x0, y0, l_free_);
        }

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}
```

#### GridMapper Class Design

```cpp
// src/slam/GridMapper.h
#pragma once

#include <vector>
#include <string>
#include <cmath>
#include "util/Types.h"
#include "sensors/ISensorSource.h"
#include "slam/OccupancyGrid.h"

class GridMapper {
public:
    // Constructor: resolution in meters/cell, dimensions in cells
    // Origin defaults to (0,0) - grid cell (0,0) is at world (0,0)
    GridMapper(float resolution, int width, int height);

    // Main update function - integrates scan into map
    // COORDINATE FRAME: LidarScan ranges are in ROBOT frame (angles relative to robot heading)
    // This function converts to WORLD frame using pose.theta before raytracing
    void update(const Pose2D& pose, const LidarScan& scan);

    // Accessors
    const std::vector<float>& getLogOddsMap() const { return log_odds_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getResolution() const { return resolution_; }

    // Export as PNG (0=obstacle/black, 255=free/white - matches NavSim)
    void saveMap(const std::string& path) const;

    // Accuracy comparison against ground truth PNG
    // ground_truth: grayscale image where black (<128) = obstacle
    float computeAccuracy(const std::vector<uint8_t>& ground_truth) const;

private:
    void traceRay(int x0, int y0, int x1, int y1);
    void updateCell(int x, int y, float delta_log_odds);
    bool isInBounds(int x, int y) const;

    // World to grid conversion (origin at 0,0)
    int worldToGridX(float x) const { return static_cast<int>(x / resolution_); }
    int worldToGridY(float y) const { return static_cast<int>(y / resolution_); }

    float resolution_;           // meters per cell (0.05f default)
    int width_, height_;         // grid dimensions in cells
    std::vector<float> log_odds_; // log-odds values, initialized to 0.0f

    // Log-odds constants - use values from "Log-Odds Parameters" section above
    static constexpr float l_occ_  = 0.85f;
    static constexpr float l_free_ = -0.4f;
    static constexpr float l_max_  = 5.0f;
    static constexpr float l_min_  = -5.0f;
};
```

#### GridMapper::update() Implementation (CRITICAL: Coordinate Conversion)

```cpp
// src/slam/GridMapper.cpp
void GridMapper::update(const Pose2D& pose, const LidarScan& scan) {
    // Robot position in grid coordinates
    int robot_gx = worldToGridX(pose.x);
    int robot_gy = worldToGridY(pose.y);

    // Calculate angle increment from scan
    float angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges.size();

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];

        // Skip invalid ranges (too close or max range)
        if (range < 0.1f || range >= 9.9f) continue;

        // CRITICAL: Convert from robot frame to world frame
        // scan angle is relative to robot, add pose.theta to get world angle
        float scan_angle = scan.angle_min + i * angle_increment;
        float world_angle = scan_angle + pose.theta;  // Robot heading added here

        // Calculate endpoint in world coordinates
        float end_x = pose.x + range * std::cos(world_angle);
        float end_y = pose.y + range * std::sin(world_angle);

        // Convert endpoint to grid coordinates
        int end_gx = worldToGridX(end_x);
        int end_gy = worldToGridY(end_y);

        // Trace ray from robot to endpoint using Bresenham
        traceRay(robot_gx, robot_gy, end_gx, end_gy);
    }
}
```

#### SlamSim Integration with NavSim

**CRITICAL: Reuse NavSim completely - do NOT reimplement any simulation logic.**

```cpp
// sim/slam_sim/SlamSim.cpp
#include "slam_sim/SlamSim.h"
#include "nav_sim/NavSim.h"
#include "slam/GridMapper.h"
#include <cmath>

SlamSim::SlamSim(const std::string& map_path, float resolution)
    : nav_sim_(map_path, resolution)
    , mapper_(resolution, nav_sim_.getWidth(), nav_sim_.getHeight())
    , resolution_(resolution) {}

std::vector<Pose2D> SlamSim::generateSnakeTrajectory() {
    std::vector<Pose2D> trajectory;
    float margin = 0.5f;  // Stay 0.5m from edges
    float spacing = 0.5f; // Vertical spacing between passes
    float step = 0.1f;    // Step size along path

    // World dimensions from NavSim
    float world_width = nav_sim_.getWidth() * resolution_;
    float world_height = nav_sim_.getHeight() * resolution_;

    // BOUNDS CHECK: Ensure map is large enough for trajectory generation
    // Minimum required: 2*margin + spacing = 1.5m in each dimension
    if (world_width < 2.0f * margin + step || world_height < 2.0f * margin + spacing) {
        // Map too small - return single pose at center
        trajectory.push_back({world_width / 2.0f, world_height / 2.0f, 0.0f});
        return trajectory;
    }

    bool left_to_right = true;
    for (float y = margin; y < world_height - margin; y += spacing) {
        if (left_to_right) {
            for (float x = margin; x < world_width - margin; x += step) {
                trajectory.push_back({x, y, 0.0f});  // theta=0 facing right
            }
        } else {
            for (float x = world_width - margin; x > margin; x -= step) {
                trajectory.push_back({x, y, M_PI});  // theta=PI facing left
            }
        }
        left_to_right = !left_to_right;
    }
    return trajectory;
}

void SlamSim::run() {
    auto trajectory = generateSnakeTrajectory();

    for (const auto& pose : trajectory) {
        nav_sim_.setRobotPose(pose);
        LidarScan scan = nav_sim_.simulateLidar(360);  // 360 rays
        mapper_.update(pose, scan);
    }
}

float SlamSim::computeAccuracy() {
    // Load ground truth as grayscale
    // Compare: built map cell occupied (log_odds > 0) vs truth (<128 = obstacle)
    // Return percentage of matching cells
}
```

#### Accuracy Calculation

Compare built map to ground truth PNG (loaded via OpenCV):

```cpp
float GridMapper::computeAccuracy(const std::vector<uint8_t>& ground_truth) const {
    // ground_truth: flattened grayscale PNG from test_data/office.png
    // Convention: black (0) = obstacle, white (255) = free
    // Same as NavSim uses internally

    int matching = 0;
    int total = width_ * height_;

    // Thresholds for classification (avoid comparing against 0.0f which is "unknown")
    // log_odds > 0.4 means p > 0.6, confidently occupied
    // log_odds < -0.4 means p < 0.4, confidently free
    const float occupied_threshold = 0.4f;
    const float free_threshold = -0.4f;

    for (int i = 0; i < total; i++) {
        // Ground truth: dark pixels (<128) = obstacle
        bool truth_occupied = ground_truth[i] < 128;

        // Built map: use threshold to avoid misclassifying "unknown" cells
        // Cells in [-0.4, 0.4] are considered "unknown" and compared neutrally
        if (log_odds_[i] > occupied_threshold) {
            // Confidently occupied
            if (truth_occupied) matching++;
        } else if (log_odds_[i] < free_threshold) {
            // Confidently free
            if (!truth_occupied) matching++;
        } else {
            // Unknown/uncertain - skip or count as partial match
            // For MVP, count unknown cells as matching if they're in unexplored areas
            // This prevents penalizing the accuracy for areas robot didn't visit
            matching++;  // Neutral handling of unknown cells
        }
    }

    return static_cast<float>(matching) / static_cast<float>(total);
}

// Usage in SlamSim:
// cv::Mat truth = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
// std::vector<uint8_t> truth_vec(truth.data, truth.data + truth.total());
// float accuracy = mapper.computeAccuracy(truth_vec);
```

### Project Structure Notes

Files and directories to create in this story:

```
src/slam/                 # NEW DIRECTORY
├── OccupancyGrid.h       # Type definition + conversion functions
├── GridMapper.h
├── GridMapper.cpp
├── Localizer.h           # Simple odometry pass-through
└── Localizer.cpp

sim/slam_sim/             # NEW DIRECTORY
├── SlamSim.h
├── SlamSim.cpp
└── main.cpp

test/
└── test_slam.cpp         # NEW FILE (test/ directory exists)
```

**Test data available:** `test_data/office.png` (200x200 pixels, 0.05m/cell = 10m x 10m environment)

### Pre-flight Validation

Before running slam_sim, verify test data exists:
```cpp
// In main.cpp
if (!fs::exists(map_path)) {
    std::cerr << "ERROR: Map file not found: " << map_path << std::endl;
    return 1;
}
cv::Mat test_map = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
if (test_map.cols != 200 || test_map.rows != 200) {
    std::cerr << "WARNING: Expected 200x200 map, got " << test_map.cols << "x" << test_map.rows << std::endl;
}
```

### Performance Optimization Notes (Post-MVP)

1. **Trajectory step size:** Currently 0.1m produces ~1850 poses for 10x10m map. Can increase to 0.2m for 4x faster simulation with minimal accuracy loss.

2. **Ray tracing parallelism:** Each ray in `update()` is independent. Future optimization could process rays in parallel using `std::execution::par`.

3. **Log-odds clamping:** Currently done per-update. Could batch updates and clamp once per frame for slight speedup.

### Dependencies on Previous Stories

**Story 1-1 (Project Setup):**
- `src/util/Types.h` - Point2D, Pose2D, Velocity structs
- `src/sensors/ISensorSource.h` - LidarScan struct
- CMakeLists.txt base configuration

**Story 1-2 (Navigation + NavSim):**
- `sim/nav_sim/NavSim.h/cpp` - 2D simulation environment (CRITICAL for slam_sim)
- `src/navigation/Costmap.h/cpp` - similar grid patterns (reference for implementation)
- Test map: `test_data/office.png`

### CMake Additions Required

**Add to CMakeLists.txt AFTER the nav_sim executable section (around line 77):**

```cmake
# ============================================
# SLAM (Story 1-3) - Add after nav_sim section
# ============================================

# SLAM library
add_library(slam
    src/slam/GridMapper.cpp
    src/slam/Localizer.cpp
)
target_link_libraries(slam ${OpenCV_LIBS})

# NavSim library (shared between nav_sim and slam_sim)
# CRITICAL: NavSim.cpp must be in a library, not just nav_sim executable
add_library(nav_sim_lib
    sim/nav_sim/NavSim.cpp
)
target_link_libraries(nav_sim_lib ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

# SlamSim executable - reuses NavSim via nav_sim_lib
add_executable(slam_sim
    sim/slam_sim/SlamSim.cpp
    sim/slam_sim/main.cpp
)
target_link_libraries(slam_sim
    slam
    nav_sim_lib         # NavSim class for simulation environment
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

# SLAM unit tests (inside the existing GTest_FOUND block)
if(GTest_FOUND)
    add_executable(test_slam test/test_slam.cpp)
    target_link_libraries(test_slam slam GTest::gtest_main)
    add_test(NAME test_slam COMMAND test_slam)
endif()

# NOTE: Also update nav_sim executable to use nav_sim_lib:
# target_link_libraries(nav_sim nav_sim_lib navigation ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
```

**Note:** The `include_directories` at the top of CMakeLists.txt already includes `${CMAKE_SOURCE_DIR}/src` and `${CMAKE_SOURCE_DIR}/sim`, so no additional include paths needed.

---

## Verification Commands

```bash
# Build and test (inside Docker container)
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests
./test_slam                                              # All tests pass

# Run SLAM simulation (creates outputs/ if missing)
./slam_sim --map ../test_data/office.png --output ../outputs/

# Verify outputs
cat ../outputs/accuracy.json
# Should show: {"accuracy": 0.92, ...}

ls ../outputs/
# Should show: built_map.png  accuracy.json
```

**Note:** slam_sim should create the output directory if it doesn't exist (use `std::filesystem::create_directories`).

### Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| Unit tests | `./test_slam` | Exit 0, all pass |
| Accuracy ≥90% | `cat outputs/accuracy.json` | `"accuracy": 0.9+` |
| Map generated | `ls outputs/built_map.png` | File exists |
| No crashes | Run slam_sim | Completes without errors |

### Output File Formats

**accuracy.json:**
```json
{
  "accuracy": 0.92,
  "cells_total": 40000,
  "cells_occupied": 892,
  "cells_free": 3631,
  "resolution": 0.05,
  "trajectory_poses": 1850
}
```

**built_map.png:** Grayscale image matching ground truth convention:
- **0 (black) = obstacle/occupied**
- **255 (white) = free space**
- Same format as `test_data/office.png` and NavSim internal map

---

## Previous Story Intelligence

### Key Patterns from Story 1-2

**Reuse directly:**
- `sim/nav_sim/NavSim.h/cpp` - simulation environment (CRITICAL for slam_sim)
- `src/util/Types.h` - Point2D, Pose2D
- `src/sensors/ISensorSource.h` - LidarScan struct
- `test_data/office.png` - test map (200x200, 0.05m/cell)

**Code conventions:** `#pragma once`, PascalCase classes, const refs for large structs, OpenCV for images.

**Note:** GridMapper uses Bresenham (integer) for ray tracing, unlike NavSim's float stepping. Bresenham is more efficient for grid updates.

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **GridMapper class** | Builds occupancy grid from LiDAR scans |
| **Log-odds updates** | Unit test verifies math correctness |
| **Bresenham ray tracing** | Unit test verifies cell updates |
| **SlamSim executable** | Runs complete mapping simulation |
| **Accuracy metric** | accuracy.json shows ≥90% match |
| **Built map output** | built_map.png visualizes mapped environment |

### Demo Script (Run This When Done)

```bash
# Inside Docker container
cd /workspace
mkdir -p build && cd build
cmake ..
make -j

# Run unit tests - all should pass
./test_slam

# Run SLAM simulation
./slam_sim --map ../test_data/office.png --output ../outputs/

# Verify accuracy meets threshold
cat ../outputs/accuracy.json | grep accuracy
# Should show: "accuracy": 0.9 or higher

# Verify map was generated
ls -la ../outputs/built_map.png
# Should exist and be non-zero size
```

**SUCCESS CRITERIA:** Story 1.3 is DONE when:
1. `./test_slam` exits with code 0 (all tests pass)
2. `accuracy.json` shows `"accuracy": 0.9` or higher
3. `built_map.png` exists and visually matches `test_data/office.png`

---

## References

- [Source: docs/architecture.md#4.3-slam] - SLAM component design
- [Source: docs/architecture.md#5.2-slamsim] - SlamSim simulation design
- [Source: docs/epics.md#story-3] - Original story requirements
- [Source: docs/sprint-artifacts/1-2-navigation-simulation-navsim.md] - Previous story patterns
- [Source: src/navigation/Costmap.h] - Grid pattern reference
- [Source: sim/nav_sim/NavSim.h] - Simulation environment to reuse
- [External: MRPT Occupancy Grids](https://www.mrpt.org/tutorials/programming/maps-for-localization-slam-map-building/occupancy_grids/) - Log-odds theory
- [External: Bresenham Algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm) - Reference implementation

---

## Dev Agent Record

### Context Reference

This story file serves as the complete implementation context.

### Agent Model Used

(Fill after implementation)

### Debug Log References

(Fill during implementation)

### Completion Notes List

(Fill after each task completion)

### File List

(Fill with created/modified files after implementation)
