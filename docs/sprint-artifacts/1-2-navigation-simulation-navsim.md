# Story 1.2: Navigation + Simulation (NavSim)

**Status:** Done

---

## Story

As the **robot**,
I want **path planning, obstacle avoidance, and a 2D simulation environment**,
So that **I can develop and test navigation without hardware**.

---

## Acceptance Criteria

1. **AC1:** A* finds shortest path around obstacles
2. **AC2:** Costmap updates from simulated scan data
3. **AC3:** PathFollower produces smooth velocity commands
4. **AC4:** NavSim loads test map from PNG
5. **AC5:** Robot navigates from start to goal without collisions
6. **AC6:** PNG shows trajectory, JSON shows metrics
7. **AC7:** Unit tests pass

---

## Tasks / Subtasks

- [x] **Task 1: Implement Navigation Core Classes** (AC: 1, 2, 3)
  - [x] 1.1 Create `src/navigation/Costmap.h/cpp` - 2D occupancy grid
    - Constructor with resolution (meters/cell) and dimensions
    - `updateFromScan(scan, robot_pose)` - integrate LiDAR data
    - `getCost(x, y)` - get cell cost (0=free, 255=obstacle)
    - `setCost(x, y, cost)` - set cell cost
    - `inflate(robot_radius)` - inflate obstacles for safe planning
    - `worldToGrid(x, y)` - convert world to grid coords
    - `gridToWorld(gx, gy)` - convert grid to world coords
    - `savePng(path)` - export map as PNG
  - [x] 1.2 Create `src/navigation/Planner.h/cpp` - A* path planning
    - Constructor taking reference to Costmap
    - `planPath(start, goal)` - return vector of Point2D waypoints
    - `isPathValid(path)` - check if path still traversable
    - Private: `heuristic(a, b)` - Euclidean distance
    - Private: `getNeighbors(node)` - 8-connected grid neighbors
    - Private: `reconstructPath(came_from, current)` - backtrack to build path
  - [x] 1.3 Create `src/navigation/PathFollower.h/cpp` - velocity from path
    - `setPath(path)` - load path to follow
    - `getVelocityCommand(current_pose)` - return Velocity toward next waypoint
    - `isComplete()` - return true if goal reached
    - `getCurrentWaypointIndex()` - for progress tracking
    - Private: pure pursuit or simple lookahead algorithm

- [x] **Task 2: Implement NavSim 2D Simulation** (AC: 4, 5, 6)
  - [x] 2.0 Create `test_data/` directory in project root
  - [x] 2.1 Create `sim/nav_sim/NavSim.h/cpp` - 2D simulation environment
    - `NavSim(map_png_path)` - load map from PNG (black=obstacle, white=free)
    - `setRobotPose(pose)` - place robot in simulation
    - `applyVelocity(vx, vy, omega, dt)` - integrate velocity to update pose
    - `simulateLidar(num_rays)` - raycast to generate simulated LiDAR scan
    - `checkCollision()` - test if robot overlaps obstacle
    - `checkGoalReached(goal, tolerance)` - test if robot at goal
    - `saveSnapshot(path)` - save current state as PNG with trajectory
    - `saveMetrics(path)` - save JSON with metrics
    - Private: `raycast(origin, angle)` - single ray to obstacle distance
  - [x] 2.2 Create `sim/nav_sim/SimLocomotion.h/cpp` - implements ILocomotion
    - Wraps NavSim for velocity commands
    - Integrates with simulation timestep
  - [x] 2.3 Create `sim/nav_sim/SimSensorSource.h/cpp` - implements ISensorSource
    - Wraps NavSim for simulated sensor data
    - Returns simulated LiDAR and pose
  - [x] 2.4 Create `sim/nav_sim/main.cpp` - nav_sim executable
    - Parse CLI args using manual parsing (same pattern as `src/main.cpp`)
    - Args: `--map`, `--start x,y,theta`, `--goal x,y`, `--output`
    - Create output directory if it doesn't exist
    - Run navigation loop until goal reached or collision
    - Output trajectory.png, metrics.json, nav.log
  - [x] 2.5 Create test map: `test_data/office.png` (simple office floor plan)
  - [x] 2.6 Create `outputs/` directory (or ensure main.cpp creates it if missing)

- [x] **Task 3: Create Unit Tests** (AC: 7)
  - [x] 3.1 Create `test/test_navigation.cpp`
    - Test A* correctness on known grid
    - Test Costmap cell updates
    - Test PathFollower velocity generation
    - Test obstacle inflation
  - [x] 3.2 Update CMakeLists.txt to build tests
    - Add Google Test: `find_package(GTest REQUIRED)`
    - Note: GTest installed in Docker via `apt-get install -y libgtest-dev`
    - Create test_navigation target

- [x] **Task 4: CMake Integration** (AC: 6, 7)
  - [x] 4.1 Update CMakeLists.txt
    - Add navigation sources to library
    - Add nav_sim executable target
    - Add test_navigation executable
    - Link OpenCV for image processing

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Use ROS2, Nav2, or any ROS dependencies - this is a pure C++ project
- Use external navigation libraries - implement A* from scratch
- Over-engineer - this is for simulation only, not production navigation
- Use MuJoCo or physics engines - simple 2D kinematics is sufficient

**MUST USE:**
- C++17 standard (`-std=c++17`)
- OpenCV for image loading and PNG output
- Existing interfaces: `ILocomotion`, `ISensorSource` from Story 1.1
- Existing types: `Point2D`, `Pose2D`, `Velocity`, `LidarScan` from Story 1.1

### Implementation Details

#### GridCell Struct (Define in Costmap.h or Types.h)

```cpp
struct GridCell {
    int x, y;

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template<>
    struct hash<GridCell> {
        size_t operator()(const GridCell& c) const {
            return hash<int>()(c.x) ^ (hash<int>()(c.y) << 16);
        }
    };
}
```

#### A* Algorithm Reference

```cpp
// A* implementation with corrected syntax
std::vector<Point2D> Planner::planPath(Point2D start, Point2D goal) {
    // Priority queue: (f_score, node) - min-heap
    std::priority_queue<std::pair<float, GridCell>,
                        std::vector<std::pair<float, GridCell>>,
                        std::greater<>> open_set;

    std::unordered_map<GridCell, GridCell> came_from;
    std::unordered_map<GridCell, float> g_score;  // Cost from start

    auto start_cell = costmap_.worldToGrid(start.x, start.y);
    auto goal_cell = costmap_.worldToGrid(goal.x, goal.y);

    g_score[start_cell] = 0;
    open_set.push({heuristic(start_cell, goal_cell), start_cell});

    while (!open_set.empty()) {
        auto current = open_set.top().second;
        open_set.pop();

        if (current == goal_cell) {
            return reconstructPath(came_from, current);
        }

        for (auto& neighbor : getNeighbors(current)) {
            if (costmap_.getCost(neighbor.x, neighbor.y) >= 250) {
                continue;  // Skip obstacles
            }

            float tentative_g = g_score[current] + distance(current, neighbor);

            // Corrected: use find() instead of value_or()
            auto it = g_score.find(neighbor);
            float current_g = (it != g_score.end()) ? it->second : INFINITY;

            if (tentative_g < current_g) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                float f = tentative_g + heuristic(neighbor, goal_cell);
                open_set.push({f, neighbor});
            }
        }
    }

    return {};  // No path found
}
```

#### Costmap Resolution & Memory Layout

- Resolution: 0.05m per cell (5cm) - good balance of precision and memory
- Inflation radius: 0.3m - robot footprint clearance
- Free space: cost 0
- Unknown: cost 128
- Obstacle: cost 254-255
- **Memory layout:** Row-major `std::vector<uint8_t>` matching OpenCV Mat for efficient PNG export
- **Coordinate convention:** World origin at map top-left. X increases right, Y increases down (matches OpenCV image coordinates)

#### Simulated LiDAR

**Angle increment calculation** (LidarScan doesn't store angle_increment):
```cpp
float angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges.size();
```

**Raycast implementation** (naive stepping - acceptable for MVP, Bresenham optimization optional):
```cpp
// Raycast in 2D grid
float NavSim::raycast(Point2D origin, float angle) {
    float max_range = 10.0f;  // meters
    float step = 0.01f;       // 1cm step

    for (float r = 0; r < max_range; r += step) {
        float x = origin.x + r * cos(angle);
        float y = origin.y + r * sin(angle);

        int gx = static_cast<int>(x / resolution_);
        int gy = static_cast<int>(y / resolution_);

        if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
            return r;  // Out of bounds
        }

        if (map_.at<uint8_t>(gy, gx) < 128) {
            return r;  // Hit obstacle (black = obstacle)
        }
    }

    return max_range;
}
```

#### Path Following

Pure pursuit algorithm for smooth following:
- Lookahead distance: 0.5m
- Max linear velocity: 0.3 m/s
- Max angular velocity: 0.5 rad/s
- Goal tolerance: 0.2m

**Optional optimization:** Path simplification (Douglas-Peucker) to reduce waypoints from A* grid output. Not required for MVP.

#### Test Map Format

- PNG grayscale image
- Black (0): obstacle/wall
- White (255): free space
- Gray (128): unknown (optional)
- Scale: 1 pixel = 5cm (0.05m)

Example 200x200 pixel map = 10m x 10m environment

### Project Structure Notes

Files and directories to create in this story:

```
src/navigation/
├── Costmap.h
├── Costmap.cpp
├── Planner.h
├── Planner.cpp
├── PathFollower.h
└── PathFollower.cpp

sim/nav_sim/
├── NavSim.h
├── NavSim.cpp
├── SimLocomotion.h
├── SimLocomotion.cpp
├── SimSensorSource.h
├── SimSensorSource.cpp
└── main.cpp

test/
└── test_navigation.cpp

test_data/           # CREATE THIS DIRECTORY
└── office.png

outputs/             # CREATE THIS DIRECTORY (or create programmatically)
└── (simulation outputs go here)
```

### Dependencies on Story 1.1

Use these existing files:
- `src/util/Types.h` - Point2D, Pose2D, Velocity structs
- `src/locomotion/ILocomotion.h` - interface for locomotion abstraction
- `src/sensors/ISensorSource.h` - interface with LidarScan struct
- `CMakeLists.txt` - add to existing build configuration

### CMake Additions Required

```cmake
# Add to existing CMakeLists.txt

# Navigation library
add_library(navigation
    src/navigation/Costmap.cpp
    src/navigation/Planner.cpp
    src/navigation/PathFollower.cpp
)
target_link_libraries(navigation ${OpenCV_LIBS})

# NavSim executable
add_executable(nav_sim
    sim/nav_sim/NavSim.cpp
    sim/nav_sim/SimLocomotion.cpp
    sim/nav_sim/SimSensorSource.cpp
    sim/nav_sim/main.cpp
)
target_link_libraries(nav_sim navigation ${OpenCV_LIBS})

# Unit tests (requires Google Test)
find_package(GTest REQUIRED)
add_executable(test_navigation test/test_navigation.cpp)
target_link_libraries(test_navigation navigation GTest::gtest_main)
```

---

## Verification Commands

```bash
# Build and test (inside Docker container)
mkdir -p build && cd build
cmake .. && make -j
./test_navigation                                                    # All tests pass
./nav_sim --map ../test_data/office.png --start 1,1,0 --goal 8,5 --output ../outputs/
```

### Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| Unit tests | `./test_navigation` | Exit 0, all pass |
| Goal reached | `cat outputs/metrics.json` | `"goal_reached": true` |
| No collisions | `cat outputs/metrics.json` | `"collisions": 0` |
| Outputs exist | `ls outputs/` | `trajectory.png`, `metrics.json`, `nav.log` |

### Output File Formats

**metrics.json:** `{"goal_reached": true, "path_length": 12.3, "collisions": 0, "time_s": 4.2, "final_pose": {"x": 8.0, "y": 5.0, "theta": 0.0}}`

**trajectory.png:** Map with path (blue), start (green circle), goal (red circle)

**nav.log:** Timestamped execution log with `[INFO]` entries

---

## Previous Story Intelligence

### Learnings from Story 1.1

**What worked well:**
- Docker environment builds successfully on multiple platforms
- CMakeLists.txt structure with optional unitree_sdk2 handling
- Clean interface abstractions (ILocomotion, ISensorSource)
- Simple, focused main.cpp with CLI parsing

**Patterns to follow:**
- Use `#pragma once` for headers
- Include `util/Types.h` for common types
- Follow existing naming conventions (PascalCase classes, camelCase methods)
- Keep implementation simple - no over-engineering

**Files that exist and should be used:**
- `src/util/Types.h` - Point2D, Pose2D, Velocity
- `src/locomotion/ILocomotion.h` - interface for sim/real abstraction
- `src/sensors/ISensorSource.h` - interface with LidarScan

**Docker notes:**
- Development happens inside container
- OpenCV already available via apt package
- Build with `cmake .. && make -j`

---

## Technical Reference Information

### Key Libraries (See Story 1.1 for setup details)

| Library | Include | Key Functions |
|---------|---------|---------------|
| OpenCV | `#include <opencv2/opencv.hpp>` | `cv::imread()`, `cv::imwrite()`, `cv::polylines()` |
| nlohmann/json | `#include <nlohmann/json.hpp>` | `json::dump()`, object/array syntax |
| GTest | `#include <gtest/gtest.h>` | `TEST()`, `EXPECT_*` macros |

### OpenCV Coordinate Gotcha

OpenCV uses (row, col) = (y, x) for pixel access:
```cpp
uint8_t pixel = map.at<uint8_t>(y, x);  // Note: y first!
```

### Error Handling for Map Loading

```cpp
cv::Mat map = cv::imread(map_png_path, cv::IMREAD_GRAYSCALE);
if (map.empty()) {
    throw std::runtime_error("Failed to load map: " + map_png_path);
}
```

### JSON Metrics Output

```cpp
#include <nlohmann/json.hpp>
#include <fstream>

nlohmann::json metrics;
metrics["goal_reached"] = true;
metrics["path_length"] = 12.3;
metrics["collisions"] = 0;
metrics["time_s"] = 4.2;
metrics["final_pose"] = {{"x", 8.0}, {"y", 5.0}, {"theta", 0.0}};

std::ofstream file("metrics.json");
file << metrics.dump(2);
```

### Thread Safety Note

For MVP, single-threaded execution is sufficient. Thread safety for Costmap access deferred to Story 3 (SLAM) or Story 4 (Hardware Integration).

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **A* path planner** | Unit test finds correct paths, avoids obstacles |
| **2D costmap** | Updates from scans, exports as PNG |
| **Path follower** | Generates smooth velocity commands |
| **NavSim simulation** | Runs complete navigation loop |
| **Trajectory output** | trajectory.png shows robot path |
| **Metrics output** | metrics.json shows goal_reached: true, collisions: 0 |

### Demo Script (Run This When Done)

```bash
# Inside Docker container
cd /workspace
mkdir -p build && cd build
cmake ..
make -j

# Run unit tests
./test_navigation
# All tests should pass

# Run navigation simulation
./nav_sim --map ../test_data/office.png --start 1,1,0 --goal 8,5 --output ../outputs/

# Verify outputs
cat ../outputs/metrics.json
# Should show: {"goal_reached": true, ...}

ls ../outputs/
# Should show: trajectory.png  metrics.json  nav.log
```

**If you see `goal_reached: true` and `collisions: 0` in metrics.json, Story 1.2 is DONE.**

---

## References

- [Source: docs/architecture.md#4.2-navigation] - Navigation component design
- [Source: docs/architecture.md#5.1-navsim] - NavSim simulation design
- [Source: docs/epics.md#story-2] - Original story requirements
- [Source: docs/sprint-artifacts/1-1-project-setup-docker-environment.md] - Previous story patterns
- [External: OpenCV imread](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html) - Image loading
- [External: A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) - Reference implementation

---

## Dev Agent Record

### Context Reference

This story file serves as the complete implementation context.

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Fixed unitree_sdk2 target name: Changed `unitree_sdk2::unitree_sdk2` to `unitree_sdk2` in CMakeLists.txt
- Added missing `#include <cstddef>` to PathFollower.h for `size_t` type
- Added missing `#include <unordered_map>` to Planner.h
- Fixed type mismatch in `std::min()` call with proper float casts
- Added `operator<` to GridCell struct for priority queue compatibility
- Updated Dockerfile to include GTest installation and build

### Completion Notes List

- **Task 1:** Implemented Costmap (2D occupancy grid with inflation), Planner (A* pathfinding with 8-connected neighbors), and PathFollower (pure pursuit algorithm with lookahead). All follow the existing code patterns from Story 1.1.
- **Task 2:** Created NavSim 2D simulation with raycast-based LiDAR simulation, SimLocomotion and SimSensorSource implementing the existing interfaces. Main executable parses CLI args and outputs trajectory.png, metrics.json, and nav.log.
- **Task 3:** Created comprehensive unit tests (20 tests) covering Costmap operations, A* planning, PathFollower behavior, and integration testing.
- **Task 4:** Updated CMakeLists.txt with navigation library, nav_sim executable, and test_navigation targets.

### File List

**New Files:**
- `src/navigation/Costmap.h`
- `src/navigation/Costmap.cpp`
- `src/navigation/Planner.h`
- `src/navigation/Planner.cpp`
- `src/navigation/PathFollower.h`
- `src/navigation/PathFollower.cpp`
- `sim/nav_sim/NavSim.h`
- `sim/nav_sim/NavSim.cpp`
- `sim/nav_sim/SimLocomotion.h`
- `sim/nav_sim/SimLocomotion.cpp`
- `sim/nav_sim/SimSensorSource.h`
- `sim/nav_sim/SimSensorSource.cpp`
- `sim/nav_sim/main.cpp`
- `test/test_navigation.cpp`
- `test_data/office.png`
- `scripts/create_test_map.py`

**Modified Files:**
- `CMakeLists.txt` - Added navigation library, nav_sim and test_navigation targets
- `docker/Dockerfile` - Added GTest installation
- `src/navigation/Costmap.h` - Added cost threshold constants (Code Review #2)
- `src/navigation/Costmap.cpp` - Use named constants (Code Review #2)
- `src/navigation/Planner.cpp` - Use OBSTACLE_THRESHOLD constant (Code Review #2)
- `sim/nav_sim/NavSim.cpp` - Fixed integer truncation with std::floor() (Code Review #2)
- `test/test_navigation.cpp` - Use named constants (Code Review #2)

**Directories Created:**
- `src/navigation/`
- `sim/nav_sim/`
- `test/`
- `test_data/`
- `outputs/`

### Change Log

- 2025-12-05: Story 1.2 implementation complete. All acceptance criteria met. 20/20 unit tests pass. NavSim successfully navigates test map with goal_reached=true and collisions=0.
- 2025-12-05: **Code Review #1 Fixes Applied:**
  - HIGH-1/2: Added closed_set to A* algorithm to prevent processing duplicate nodes and improve performance
  - HIGH-3: Added `Costmap::fromImage()` factory method; updated main.cpp to use cleaner initialization
  - MEDIUM-1: Renamed `findLookaheadPoint()` to `advanceToLookaheadPoint()` with documentation clarifying state mutation
  - MEDIUM-2: Changed raycast step from 1cm to resolution (5cm) for 5x performance improvement
  - MEDIUM-3: Added explicit GOAL_TOLERANCE constant in main.cpp matching PathFollower
  - MEDIUM-4: Added bounds validation for start/goal positions before simulation
  - LOW-1: Fixed log file double-newline issue from ctime()
- 2025-12-05: **Code Review #2 Fixes Applied:**
  - HIGH-1: Verified log format fix is correct in code; outputs/ files are stale and need regeneration
  - MEDIUM-2: Fixed integer truncation in NavSim collision check and raycast - use std::floor() instead of static_cast<int>() for correct negative coordinate handling
  - LOW-2: Added named constants (COST_FREE, COST_UNKNOWN, OBSTACLE_THRESHOLD, COST_OBSTACLE, COST_LETHAL) in Costmap.h, replacing magic number 250/254/255 throughout codebase

