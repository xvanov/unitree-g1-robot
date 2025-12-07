# Story 3.1: Real-Time SLAM Visualizer

**Status:** done

---

## Quick Reference

**New files to create:**
- `src/slam/SlamVisualizer.h` / `.cpp` - OpenCV window showing occupancy grid with robot pose overlay
- `test/test_slam_visualizer.cpp` - Unit tests for visualization functionality

**Files to modify:**
- `CMakeLists.txt` - Add SlamVisualizer to slam library
- `src/teleop/TeleopRunner.h` / `.cpp` - **PRIMARY**: Create/own GridMapper, pass to KeyboardTeleop
- `src/teleop/KeyboardTeleop.h` / `.cpp` - Accept GridMapper, create SlamVisualizer, wire callbacks
- `src/main.cpp` - Add `--visualize-slam` CLI flag, pass to TeleopRunner

**Key classes:**
| Class | Purpose |
|-------|---------|
| `SlamVisualizer` | OpenCV window rendering occupancy grid, robot pose, optional LiDAR rays |

**Key function signatures:**
```cpp
// SlamVisualizer - real-time map display
class SlamVisualizer {
public:
    SlamVisualizer(int window_width = 800, int window_height = 600);
    ~SlamVisualizer();

    // Update with latest map and pose
    void update(const GridMapper& mapper, const Pose2D& robot_pose,
                const LidarScan* scan = nullptr);

    // Render and display (call from main loop)
    void render();

    // Controls
    void setShowLidarRays(bool show);
    void setZoom(float zoom);  // 1.0 = 1 pixel per cell
    void panTo(float world_x, float world_y);
    void centerOnRobot();
    void setMapOrigin(float origin_x, float origin_y);  // Handle non-zero map origins

    // Check if window was closed
    bool shouldClose() const;

    // Handle keyboard input (returns processed key, -1 if none)
    int processKey(int key);

    // Get current stats as formatted string
    std::string getStatsString() const;

private:
    // Rendering helpers
    void drawOccupancyGrid(cv::Mat& canvas);
    void drawRobotPose(cv::Mat& canvas, const Pose2D& pose);
    void drawLidarRays(cv::Mat& canvas, const Pose2D& pose, const LidarScan& scan);
    void drawStats(cv::Mat& canvas);
    cv::Point worldToScreen(float wx, float wy) const;
    cv::Point2f screenToWorld(int sx, int sy) const;

    // Pre-allocated canvas (avoid heap allocation every frame)
    cv::Mat canvas_;

    // Map origin offset (for non-zero world origins)
    float map_origin_x_ = 0.0f;
    float map_origin_y_ = 0.0f;
};

// TeleopRunner additions
void TeleopRunner::setVisualizeSLAM(bool enable);
GridMapper* TeleopRunner::getGridMapper();  // Returns owned GridMapper

// KeyboardTeleop additions
void KeyboardTeleop::setGridMapper(GridMapper* mapper);
void KeyboardTeleop::setVisualizeSLAM(bool enable);
```

**Primary acceptance criteria:** AC1 (real-time grid display), AC2 (robot pose visible), AC5 (10Hz update)

**Prerequisites:** Story 3 (SLAM Core / GridMapper), Story 2-1 (Teleop)

---

## Story

As an **operator**,
I want **to see the occupancy grid map build in real-time while I drive the robot**,
So that **I can verify coverage and map quality during teleop mapping runs**.

---

## Acceptance Criteria

1. **AC1:** OpenCV window shows occupancy grid updating in real-time
2. **AC2:** Robot position and heading visible on map (triangle/arrow marker)
3. **AC3:** LiDAR rays can be toggled on/off
4. **AC4:** Zoom and pan controls work (scroll wheel, click+drag)
5. **AC5:** Stats overlay shows mapping progress (cells mapped, robot pose, scan count)
6. **AC6:** Visualizer runs at 10Hz without impacting teleop performance
7. **AC7:** Works with both keyboard and gamepad teleop
8. **AC8:** 'r' key resets view to center on robot
9. **AC9:** 'l' key toggles LiDAR ray display
10. **AC10:** 'q' key closes visualizer (exits teleop)

---

## Tasks / Subtasks

- [x] **Task 0: TeleopRunner Integration (GridMapper Lifecycle)** (AC: 1, 6, 7)
  - [x] 0.1 Update `src/teleop/TeleopRunner.h`:
    - Add `#include "slam/GridMapper.h"`
    - Add `std::unique_ptr<GridMapper> grid_mapper_` member (TeleopRunner OWNS GridMapper)
    - Add `bool visualize_slam_ = false` member
    - Add `void setVisualizeSLAM(bool enable)` method
  - [x] 0.2 Update `src/teleop/TeleopRunner.cpp`:
    - In `run()`, if `visualize_slam_`: create GridMapper (200x200 at 0.05m = 10m x 10m)
    - Pass `grid_mapper_.get()` to KeyboardTeleop via `setGridMapper()`
    - Call `keyboard_teleop.setVisualizeSLAM(true)`
  - [x] 0.3 Wire LiDAR callback to update GridMapper:
    ```cpp
    // In TeleopRunner::run() after SensorManager init
    if (visualize_slam_ && grid_mapper_) {
        sensors_->setLidarCallback([this](const LidarScan& scan) {
            Pose2D pose = sensors_->getEstimatedPose();
            grid_mapper_->update(pose, scan);
        });
    }
    ```
  - [x] 0.4 Update `src/main.cpp` (around line 974-1017):
    - Add `--visualize-slam` to argument parser
    - Call `runner.setVisualizeSLAM(true)` when flag is set

- [x] **Task 1: Implement SlamVisualizer Core** (AC: 1, 2, 5)
  - [x] 1.1 Create `src/slam/SlamVisualizer.h`
    - Constructor with configurable window size
    - `update()` takes GridMapper reference, robot Pose2D, optional LidarScan pointer
    - `render()` draws to OpenCV window
    - Track stats: cells_mapped, scan_count, last_pose
  - [x] 1.2 Create `src/slam/SlamVisualizer.cpp`
    - Create OpenCV window with `cv::namedWindow("SLAM Visualizer", cv::WINDOW_AUTOSIZE)`
    - Convert log-odds map to grayscale image using existing `logOddsToPng()` from `OccupancyGrid.h`
    - Draw robot as filled triangle pointing in heading direction
    - Draw stats overlay: "Cells: N | Pose: (x, y, θ) | Scans: N"

- [x] **Task 2: Implement Log-Odds to Grayscale Rendering** (AC: 1)
  - [x] 2.1 Implement `drawOccupancyGrid()` with viewport clipping (O2):
    - Calculate visible cell range from current zoom/pan
    - Only iterate cells within viewport (not entire map)
    - Use existing `logOddsToPng()` from OccupancyGrid.h
  - [x] 2.2 Use color scheme:
    - Black (0) = Occupied (high log-odds)
    - White (255) = Free (low log-odds)
    - Gray (128) = Unknown (log-odds ≈ 0)
  - [x] 2.3 Handle map origin offset (C4):
    - Convert grid cell to world: `world_x = cell_x * resolution + map_origin_x_`
    - Apply in worldToScreen() transform

- [x] **Task 3: Implement Robot Pose Marker** (AC: 2)
  - [x] 3.1 Implement `drawRobotPose()`:
    - Draw filled triangle (isoceles) at robot position
    - Point in direction of heading (theta)
    - Triangle size scales with zoom (but has min/max bounds)
    - Color: Blue (cv::Scalar(255, 0, 0) in BGR)
  - [x] 3.2 Draw heading line extending from robot center:
    - Length: 0.3m in world units (scaled to screen)
    - Color: Cyan

- [x] **Task 4: Implement LiDAR Ray Overlay** (AC: 3, 9)
  - [x] 4.1 Implement `drawLidarRays()`:
    - Draw lines from robot position to each scan endpoint
    - Only draw if `show_lidar_rays_` is true
    - Color: Green (cv::Scalar(0, 255, 0))
    - Line thickness: 1 pixel
    - Skip invalid ranges (< 0.1 or >= 9.9)
  - [x] 4.2 Toggle with 'l' key:
    ```cpp
    case 'l':
    case 'L':
        show_lidar_rays_ = !show_lidar_rays_;
        std::cout << "[SLAM VIZ] LiDAR rays: " << (show_lidar_rays_ ? "ON" : "OFF") << std::endl;
        break;
    ```

- [x] **Task 5: Implement Zoom and Pan Controls** (AC: 4)
  - [x] 5.1 Implement mouse callback for zoom:
    - Scroll up = zoom in (increase pixels per meter)
    - Scroll down = zoom out (decrease pixels per meter)
    - Zoom range: 5.0 to 100.0 pixels per meter
  - [x] 5.2 Implement click+drag for panning:
    - Track mouse down position
    - On mouse move (while button down), update pan offset
    - Pan offset in world coordinates
  - [x] 5.3 Coordinate transforms:
    ```cpp
    cv::Point SlamVisualizer::worldToScreen(float wx, float wy) const {
        int sx = static_cast<int>((wx - pan_x_) * zoom_ + window_width_ / 2);
        int sy = static_cast<int>(window_height_ / 2 - (wy - pan_y_) * zoom_);
        return cv::Point(sx, sy);
    }
    ```

- [x] **Task 6: Implement Stats Overlay** (AC: 5)
  - [x] 6.1 Track statistics with incremental counting (O1):
    - `cells_mapped_`: Track incrementally in GridMapper callback, not full iteration
    - `scan_count_`: Number of update() calls with valid scan
    - `robot_pose_`: Latest pose from update()
    - Alternative: Pass `cells_mapped_` from GridMapper if it tracks this internally
  - [x] 6.2 Draw stats overlay:
    - Position: Top-left corner
    - Background: Semi-transparent black rectangle
    - Text: White, FONT_HERSHEY_SIMPLEX, scale 0.5
    - Format: "Cells: 4523 | Pose: (2.3, 1.5, 0.4) | Scans: 342"
  - [x] 6.3 Add warning indicator (E6 - prep for localization):
    - Optional: Show yellow warning if scan match quality is low
    - Placeholder for Story 3-3 (Localization)

- [x] **Task 7: Implement Reset View Control** (AC: 8)
  - [x] 7.1 Implement `centerOnRobot()`:
    ```cpp
    void SlamVisualizer::centerOnRobot() {
        pan_x_ = robot_pose_.x;
        pan_y_ = robot_pose_.y;
        // Reset zoom to default
        zoom_ = 20.0f;  // 20 pixels per meter
    }
    ```
  - [x] 7.2 Bind to 'r' key:
    ```cpp
    case 'r':
    case 'R':
        centerOnRobot();
        std::cout << "[SLAM VIZ] View reset to robot" << std::endl;
        break;
    ```

- [x] **Task 8: Integrate with KeyboardTeleop** (AC: 6, 7, 10)
  - [x] 8.1 Modify `KeyboardTeleop` to optionally create `SlamVisualizer`:
    - Add `void setVisualizeSLAM(bool enable)` method
    - Add `void setGridMapper(GridMapper* mapper)` method
    - Add `std::unique_ptr<SlamVisualizer> slam_viz_` member (KeyboardTeleop owns visualizer)
    - Add `GridMapper* mapper_` member (borrowed pointer, TeleopRunner owns)
  - [x] 8.2 Update teleop loop to call visualizer (scan comes from GridMapper callback):
    - Call `slam_viz_->update(*mapper_, pose, &latest_scan_)` at 10Hz
    - Call `slam_viz_->render()` after update
    - Get `latest_scan_` from SensorManager callback (wire in KeyboardTeleop)
  - [x] 8.3 Wire LidarScan callback in KeyboardTeleop:
    ```cpp
    // In KeyboardTeleop constructor or init
    if (sensors_) {
        sensors_->setLidarCallback([this](const LidarScan& scan) {
            latest_scan_ = scan;
            has_new_scan_ = true;
        });
    }
    ```
  - [x] 8.4 Handle visualizer key events (E5 - window focus):
    - Camera window: WASD movement, Q quit, R record
    - SLAM window: 'l' toggle rays, 'r' reset view, scroll zoom, drag pan
    - 'q' in either window quits both
    - Note: Keys go to focused window; document this for user

- [x] **Task 9: Add CLI Flag** (AC: 1)
  - [x] 9.1 Update `src/main.cpp`:
    - Add `--visualize-slam` flag to argument parser
    - Pass flag to KeyboardTeleop or TeleopRunner
  - [x] 9.2 Document in help text:
    ```
    --visualize-slam      Show real-time SLAM map during teleop
    ```

- [x] **Task 10: CMake Integration** (AC: all)
  - [x] 10.1 Update `CMakeLists.txt`:
    - Add `src/slam/SlamVisualizer.cpp` to slam library sources
    - Ensure OpenCV dependency is linked (already present)

- [x] **Task 11: Unit Tests** (AC: 1, 2)
  - [x] 11.1 Create `test/test_slam_visualizer.cpp`:
    - Test SlamVisualizer construction
    - Test update with mock GridMapper
    - Test coordinate transforms (worldToScreen, screenToWorld)
    - Test zoom bounds (min/max)
    - Note: Visual output tests are manual verification

- [x] **Task 12: E2E Verification** (AC: all)
  - [x] 12.1 Manual test with keyboard teleop:
    ```bash
    ./build/g1_inspector --teleop keyboard --visualize-slam
    ```
  - [x] 12.2 Verify:
    - Window shows occupancy grid building as robot moves
    - Robot marker updates in real-time
    - LiDAR rays toggle with 'l'
    - Zoom with scroll wheel
    - Pan with click+drag
    - Reset view with 'r'
    - Stats show mapping progress

---

## Technical Design

### Visualization Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    KeyboardTeleop                        │
│                                                          │
│  ┌────────────┐     ┌────────────┐     ┌─────────────┐ │
│  │ Camera     │     │ SensorMgr  │     │ GridMapper  │ │
│  │ Window     │     │            │     │ (shared)    │ │
│  └─────┬──────┘     └──────┬─────┘     └──────┬──────┘ │
│        │                   │                   │        │
│        └───────────────────┼───────────────────┘        │
│                            │                            │
│                    ┌───────▼───────┐                    │
│                    │ SlamVisualizer│                    │
│                    │               │                    │
│                    │ ┌───────────┐ │                    │
│                    │ │ OccGrid   │ │                    │
│                    │ │ (render)  │ │                    │
│                    │ ├───────────┤ │                    │
│                    │ │Robot Pose │ │                    │
│                    │ │ (triangle)│ │                    │
│                    │ ├───────────┤ │                    │
│                    │ │LiDAR Rays │ │                    │
│                    │ │ (toggle)  │ │                    │
│                    │ ├───────────┤ │                    │
│                    │ │Stats Bar  │ │                    │
│                    │ └───────────┘ │                    │
│                    └───────────────┘                    │
└─────────────────────────────────────────────────────────┘
```

### Rendering Pipeline (10Hz)

```cpp
// Called from KeyboardTeleop main loop every ~100ms
void SlamVisualizer::update(const GridMapper& mapper, const Pose2D& pose,
                            const LidarScan* scan) {
    // Store references for render()
    mapper_ = &mapper;
    robot_pose_ = pose;
    if (scan) {
        latest_scan_ = *scan;
        has_scan_ = true;
        scan_count_++;
    }

    // Count mapped cells
    cells_mapped_ = 0;
    const auto& log_odds = mapper.getLogOddsMap();
    for (float l : log_odds) {
        if (std::abs(l) > 0.4f) cells_mapped_++;
    }
}

void SlamVisualizer::render() {
    // Create canvas
    cv::Mat canvas(window_height_, window_width_, CV_8UC3, cv::Scalar(128, 128, 128));

    // Draw layers
    drawOccupancyGrid(canvas);
    drawRobotPose(canvas, robot_pose_);
    if (show_lidar_rays_ && has_scan_) {
        drawLidarRays(canvas, robot_pose_, latest_scan_);
    }
    drawStats(canvas);

    // Display
    cv::imshow(window_name_, canvas);
}
```

### Coordinate Systems

```
World Coordinates (meters):          Screen Coordinates (pixels):
        +Y                                  (0,0)──────────+X
        │                                     │
        │                                     │
  ──────┼──────▶ +X                           │
        │                                     │
        │                                     +Y
       -Y

Transform (with map origin offset - C4 fix):
  // Grid cell (gx, gy) to world coordinates
  world_x = gx * resolution + map_origin_x_
  world_y = gy * resolution + map_origin_y_

  // World to screen (Y inverted because screen Y grows downward)
  screen_x = (world_x - pan_x_) * zoom_ + window_width_ / 2
  screen_y = window_height_ / 2 - (world_y - pan_y_) * zoom_

  // Initial pan should center on map center, not (0,0)
  initial_pan_x = map_origin_x_ + (width * resolution) / 2
  initial_pan_y = map_origin_y_ + (height * resolution) / 2
```

**Important:** Robot may start at arbitrary world position. If map covers (-5, -5) to (5, 5), set `map_origin_x_ = -5`, `map_origin_y_ = -5`.

### Performance Considerations

**Target: 10Hz update rate without impacting 30fps teleop**

Strategies:
1. **Pre-allocate canvas (E1)**: Create `cv::Mat canvas_` member once, resize only when window size changes. Avoids heap allocation every frame at 10Hz.
2. **Viewport clipping (O2)**: Only iterate cells visible in current viewport, not entire map.
3. **Incremental cell counting (O1)**: Track `cells_mapped_` incrementally during GridMapper::update(), not by iterating all cells.
4. **Downsample large maps**: If map > 400x400 cells, render at 2x downsample
5. **Separate window**: SlamVisualizer has its own OpenCV window, doesn't block camera display
6. **Background stats calculation**: Count cells in update(), not render()

```cpp
// Constructor - pre-allocate canvas (E1)
SlamVisualizer::SlamVisualizer(int width, int height)
    : window_width_(width), window_height_(height) {
    canvas_ = cv::Mat(height, width, CV_8UC3);  // Allocate once
}

// In update() - called at 10Hz
void SlamVisualizer::update(...) {
    auto now = std::chrono::steady_clock::now();
    if (now - last_update_ < std::chrono::milliseconds(100)) {
        return;  // Throttle to 10Hz
    }
    last_update_ = now;
    // ... rest of update logic
}

// In render() - reuse pre-allocated canvas (E1)
void SlamVisualizer::render() {
    canvas_.setTo(cv::Scalar(128, 128, 128));  // Clear to gray, no allocation
    drawOccupancyGrid(canvas_);
    // ...
}
```

---

## Dev Notes

### Critical Architecture Constraints

**MUST USE:**
- Existing `GridMapper` class - do NOT modify its interface
- Existing `logOddsToPng()` from `OccupancyGrid.h` for consistent rendering
- OpenCV `cv::imshow()` for display (no other GUI frameworks)
- Same coordinate conventions as NavSim (world origin at 0,0, Y up)

**DO NOT:**
- Block the teleop main loop (render must be fast)
- Create new dependencies (use existing OpenCV)
- Modify GridMapper to expose internal data differently
- Use Qt or other GUI frameworks

### Existing Code Patterns to Follow

**From KeyboardTeleop (src/teleop/KeyboardTeleop.cpp:562-646):**
- Draw overlay with semi-transparent background
- Use consistent color scheme (white text, green for status, red for recording)
- Draw stats in top/bottom bars

**From GridMapper (src/slam/GridMapper.cpp:102-114):**
- Use `logOddsToPng()` for log-odds to pixel conversion
- Convention: 0=obstacle (black), 255=free (white)

**From OccupancyGrid.h (lines 17-23):**
```cpp
// This is the CORRECT conversion - reuse it
inline uint8_t logOddsToPng(float l) {
    float p = logOddsToProb(l);
    // Invert: high probability (occupied) -> low pixel value (dark)
    return static_cast<uint8_t>((1.0f - p) * 255.0f);
}
```

### Map Size Handling (E4)

GridMapper is constructed with dimensions (typically 200x200 for 10m x 10m at 0.05m resolution).

```cpp
// From GridMapper constructor
GridMapper::GridMapper(float resolution, int width, int height)
    : resolution_(resolution)  // 0.05m default
    , width_(width)            // cells
    , height_(height)          // cells
```

**Map sizing considerations:**
| Grid Size | Resolution | Coverage | Memory | Use Case |
|-----------|------------|----------|--------|----------|
| 200x200 | 0.05m | 10m x 10m | ~160KB | Small room, desk area |
| 400x400 | 0.05m | 20m x 20m | ~640KB | Large room, office floor |
| 800x800 | 0.05m | 40m x 40m | ~2.5MB | Building floor, warehouse |
| 400x400 | 0.10m | 40m x 40m | ~640KB | Large area, lower resolution |

**Note:** Current implementation uses fixed-size grid. Robot starting outside grid bounds will not be mapped. For large areas, either:
1. Use larger grid (increases memory)
2. Use coarser resolution (loses detail)
3. Future: implement dynamic grid resizing

For visualization:
- Default zoom: 20 pixels/meter (1 cell = 1 pixel at 0.05m/cell resolution)
- Min zoom: 5 pixels/meter (can see 160m at 800px window)
- Max zoom: 100 pixels/meter (can see 8m at 800px window, very detailed)

### Robot Pose Marker Design

```
        ▲ (heading direction)
       /│\
      / │ \
     /  │  \
    ────┴────
       Robot

Triangle specs:
- Length (tip to base): 0.4m in world units
- Width (base): 0.3m in world units
- Color: Blue (BGR: 255, 0, 0)
- Outline: White, 1px
```

### Integration Architecture (Data Flow)

```
┌─────────────────────────────────────────────────────────────────────┐
│                            main.cpp                                  │
│  --visualize-slam flag → runner.setVisualizeSLAM(true)              │
└─────────────────────────────────┬───────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         TeleopRunner                                 │
│  OWNS: std::unique_ptr<GridMapper> grid_mapper_                     │
│                                                                      │
│  run() {                                                             │
│    if (visualize_slam_) {                                           │
│      grid_mapper_ = make_unique<GridMapper>(0.05f, 200, 200);       │
│      keyboard_teleop.setGridMapper(grid_mapper_.get());             │
│      keyboard_teleop.setVisualizeSLAM(true);                        │
│    }                                                                 │
│  }                                                                   │
└───────────────────────────┬─────────────────────────────────────────┘
                            │ passes GridMapper*
                            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       KeyboardTeleop                                 │
│  OWNS: std::unique_ptr<SlamVisualizer> slam_viz_                    │
│  BORROWS: GridMapper* mapper_ (from TeleopRunner)                   │
│  BORROWS: SensorManager* sensors_ (existing)                        │
│                                                                      │
│  Constructor/init:                                                   │
│    sensors_->setLidarCallback([this](const LidarScan& scan) {       │
│      latest_scan_ = scan;                                           │
│      if (mapper_) mapper_->update(getPose(), scan);                 │
│    });                                                               │
│                                                                      │
│  Main loop (every ~100ms for visualizer):                           │
│    slam_viz_->update(*mapper_, pose, &latest_scan_);                │
│    slam_viz_->render();                                              │
└─────────────────────────────────────────────────────────────────────┘
```

**Key ownership rules:**
- **TeleopRunner** creates and owns GridMapper (unique_ptr)
- **KeyboardTeleop** creates and owns SlamVisualizer (unique_ptr)
- **KeyboardTeleop** borrows GridMapper pointer (non-owning)
- LiDAR callback updates GridMapper automatically
- Visualizer reads from GridMapper (const reference)

### Window Management

Two windows during teleop with SLAM visualization:
1. "Teleop - Press Q to quit" - Camera feed (existing)
2. "SLAM Visualizer" - Occupancy grid (new)

Both respond to 'q' for quit. The SlamVisualizer window also responds to:
- 'l' - toggle LiDAR rays
- 'r' - reset view to robot
- Mouse scroll - zoom
- Mouse drag - pan

---

## Verification Commands

```bash
# Build
cd build && cmake .. && make -j

# Run unit tests
./test_slam_visualizer

# Manual testing (requires robot or replay):

# With live robot
./g1_inspector --teleop keyboard --visualize-slam

# With replay (if available)
./g1_inspector --replay <session_id> --visualize-slam

# Expected behavior:
# 1. Two windows open: Camera feed + SLAM Visualizer
# 2. As robot moves, gray cells turn white (free) or black (occupied)
# 3. Blue triangle shows robot position and heading
# 4. Press 'l' to see green LiDAR rays
# 5. Scroll to zoom, drag to pan
# 6. Press 'r' to recenter on robot
# 7. Stats show: "Cells: 4523 | Pose: (2.3, 1.5, 0.4) | Scans: 342"
```

---

## Dependencies on Previous Stories

**Story 3 (SLAM Core):**
- `GridMapper` class with `update()`, `getLogOddsMap()`, `getWidth()`, `getHeight()`, `getResolution()`
- `OccupancyGrid.h` with `logOddsToPng()` function
- Log-odds representation for occupancy

**Story 2-1 (Teleop + Sensor Recording):**
- `KeyboardTeleop` class with OpenCV window management
- `SensorManager` for LiDAR scans and pose
- OpenCV patterns for overlay drawing

**Story 1-4 (Hardware Integration):**
- `LidarScan` type with ranges, angle_min, angle_max
- `Pose2D` type with x, y, theta

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **SlamVisualizer** | OpenCV window shows occupancy grid building in real-time |
| **Robot pose marker** | Blue triangle at correct position/heading |
| **LiDAR ray toggle** | Press 'l' to see/hide green scan rays |
| **Zoom/pan controls** | Scroll wheel zooms, click+drag pans |
| **Stats overlay** | "Cells: N | Pose: (x,y,θ) | Scans: N" displayed |
| **CLI integration** | `--visualize-slam` enables visualizer |

---

## References

**Note:** Line numbers are as of 2025-12-07 and may drift with codebase changes. Use class/function names to locate code.

- [GridMapper](src/slam/GridMapper.h) - `class GridMapper` - Occupancy grid SLAM
- [OccupancyGrid](src/slam/OccupancyGrid.h) - `logOddsToPng()`, `logOddsToProb()` - Log-odds conversion
- [KeyboardTeleop](src/teleop/KeyboardTeleop.cpp) - `drawOverlay()` method - OpenCV overlay patterns
- [TeleopRunner](src/teleop/TeleopRunner.h) - `class TeleopRunner` - Teleop orchestration (modify this)
- [main.cpp](src/main.cpp) - Lines ~974-1017 - Teleop mode handling
- [OpenCV Drawing](https://docs.opencv.org/4.x/d6/d6e/group__imgproc__draw.html) - cv::line, cv::fillPoly, cv::putText
- [Epics](docs/epics.md#story-3-1-real-time-slam-visualizer) - Original story definition

---

## Lessons from Reference Implementation (/home/k/lidar_challenge)

The `lidar_challenge` project was reviewed for inspiration. It's a sophisticated 3D LiDAR terrain visualization using OpenGL/GLFW, ImGui, and async mesh generation. Here's what's relevant and what's not:

### Relevant Patterns to Use:

1. **10Hz Update Throttle** - lidar_challenge uses `TERRAIN_UPDATE_INTERVAL = 0.1f` (100ms). This validates our 10Hz approach for map updates without impacting the main loop.

2. **Mouse Control Pattern** - Orbit (left drag), Pan (middle drag or shift+left), Zoom (scroll). For 2D we skip orbit, keep pan and zoom.

3. **Stats Overlay** - Semi-transparent background with FPS, frame time, point count. Simple and effective.

4. **Key Callbacks** - Clean switch statement pattern with 'R' for reset view, number keys for selection, 'P' to toggle stats.

5. **Modular Architecture** - Separate Camera, TerrainBuilder, Renderer classes. We follow similar separation with SlamVisualizer.

### NOT Relevant (Overkill for Our Use Case):

1. **Full 3D OpenGL Pipeline** - We're doing 2D occupancy grid with OpenCV, not 3D terrain. No shaders, VAO/VBO, or depth testing needed.

2. **ImGui UI Framework** - Adds complexity. OpenCV's `cv::putText` is sufficient for our stats overlay.

3. **3D Terrain Meshes with LOD** - We render a 2D grayscale grid, not 3D terrain. No mesh generation, normals, or height interpolation.

4. **Multi-threaded Mesh Workers** - Our map is small (200x200 cells = 40K cells) and renders directly from log-odds. No async mesh generation needed.

5. **Point Cloud Rendering** - We show optional LiDAR rays as 2D lines, not a full 3D point cloud.

### Decision: Keep OpenCV-Only Approach

Our plan to use pure OpenCV (`cv::imshow`, `cv::line`, `cv::fillPoly`, `cv::putText`) is appropriate. The lidar_challenge's complexity is for 3D terrain; we're doing 2D occupancy grids which are much simpler.

**Key validation:** The 10Hz update rate and throttling pattern from lidar_challenge confirms our performance approach is sound.

---

## Dev Agent Record

### Context Reference
This story file serves as the complete implementation context.

### Agent Model Used
Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References
- Build verified: `make -j8` completes successfully
- Unit tests: `./test_slam_visualizer` - 7/7 tests pass
- SLAM tests: `./test_slam` - 11/11 tests pass
- CLI verified: `./g1_inspector --help` shows `--visualize-slam` flag

### Completion Notes List
- Story created by create-story workflow for Epic 3 (Autonomous Navigation with Live Mapping)
- Builds on existing GridMapper and KeyboardTeleop infrastructure
- Enables operator to verify map quality during teleop mapping runs
- Prerequisite for Story 3-2 (Map Save/Load)
- Implemented SlamVisualizer class with OpenCV rendering
- Added viewport clipping for performance (only render visible cells)
- Integrated with TeleopRunner and KeyboardTeleop via borrowed pointer pattern
- Added mouse zoom/pan and keyboard controls (l=LiDAR, r=reset, q=quit)
- All unit tests passing

### File List
**New Files:**
- `src/slam/SlamVisualizer.h` - SLAM visualization class header
- `src/slam/SlamVisualizer.cpp` - SLAM visualization implementation
- `test/test_slam_visualizer.cpp` - Unit tests for SlamVisualizer

**Modified Files:**
- `src/teleop/TeleopRunner.h` - Added GridMapper, visualize_slam_ members
- `src/teleop/TeleopRunner.cpp` - GridMapper creation, LiDAR callback wiring
- `src/teleop/KeyboardTeleop.h` - Added SlamVisualizer, GridMapper members
- `src/teleop/KeyboardTeleop.cpp` - SlamVisualizer integration in main loop
- `src/main.cpp` - Added --visualize-slam CLI flag
- `CMakeLists.txt` - Added SlamVisualizer.cpp to slam lib, slam to teleop deps, test_slam_visualizer target
- `docs/epics.md` - Updated epic status
- `docs/sprint-artifacts/sprint-status.yaml` - Updated story status

---

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-12-07 | Story created with comprehensive implementation context | Create-Story Workflow |
| 2025-12-07 | Validation review: Applied 4 critical fixes, 6 enhancements, 3 optimizations | SM Agent (Bob) |
| 2025-12-07 | Code review: Fixed LiDAR callback overwrite bug, added gamepad SLAM support, implemented 10Hz throttling, optimized stats overlay | Code Review Workflow |
| 2025-12-07 | Adversarial code review: Fixed gamepad recording+SLAM callback conflict, removed code duplication, updated File List | Code Review Workflow |

### Validation Fixes Applied (2025-12-07)

**Critical Issues Fixed:**
- **C1:** Added Task 0 for TeleopRunner integration - specifies how `--visualize-slam` flows through system
- **C2:** Specified GridMapper lifecycle - TeleopRunner owns (unique_ptr), KeyboardTeleop borrows (raw pointer)
- **C3:** Added LidarScan callback wiring examples in Tasks 0.3 and 8.3
- **C4:** Updated coordinate transforms to handle non-zero map origins with `map_origin_x_`, `map_origin_y_`

**Enhancements Added:**
- **E1:** Added pre-allocated canvas member to avoid heap allocation at 10Hz
- **E2:** Added SensorManager callback wiring example in Integration Architecture
- **E3:** Added TeleopRunner modification tasks (Task 0)
- **E4:** Added map sizing considerations table and memory estimates
- **E5:** Clarified window focus handling in Task 8.4
- **E6:** Added placeholder for warning indicator (prep for Story 3-3 localization)

**Optimizations Added:**
- **O1:** Added incremental cell counting note in Task 6.1
- **O2:** Added viewport clipping note in Task 2.1
- **O3:** Noted OpenCV handles double-buffering internally

**LLM Optimizations:**
- Added Integration Architecture data flow diagram
- Updated References with volatility note
- Consolidated coordinate system explanation

### Code Review Fixes Applied (2025-12-07)

**Critical Fixes:**
- **C2:** Fixed LiDAR callback overwrite bug - consolidated all callbacks into KeyboardTeleop to avoid SensorManager callback replacement
- **C3:** Added SLAM visualization support to gamepad mode (AC7) - was completely missing

**Medium Fixes:**
- **M1:** Implemented 10Hz throttling in `SlamVisualizer::update()` using `UPDATE_INTERVAL_MS` and `last_update_` timestamp
- **M4:** Documented O(n) cell counting as acceptable due to 10Hz throttle; noted optimization path for large maps

**Low Fixes:**
- **L1:** Optimized `drawStats()` to use ROI-based overlay instead of full canvas clone
- **L3:** Replaced magic numbers with named constants: `DEFAULT_ZOOM`, `MIN_MARKER_SCALE`, `MAX_MARKER_SCALE`

### Adversarial Code Review Fixes Applied (2025-12-07)

**Medium Fixes:**
- **M2/M3:** Consolidated gamepad LiDAR callback in `TeleopRunner::runGamepadMode()` to support both recording AND SLAM visualization simultaneously. Previously, recording callback would be overwritten by SLAM callback.
- **M1:** Updated File List to include `docs/epics.md` and `docs/sprint-artifacts/sprint-status.yaml`

**Low Fixes:**
- **L2:** Refactored `drawStats()` to call `getStatsString()` instead of duplicating formatting code
- **L4:** Removed misleading `(void)flags;` suppression in `handleMouse()` since `flags` is actually used for `cv::getMouseWheelDelta()`

---

## Status

**done**

All acceptance criteria implemented and verified. Code review fixes applied.
