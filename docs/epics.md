# Epic Breakdown - Lightweight C++ Stack

**Project:** unitree-g1-robot
**Author:** BMAD
**Date:** 2025-12-04
**Architecture:** v2.0 - C++ with unitree_sdk2 (no ROS2)

---

## Overview

This document defines the epic and story breakdown for the Unitree G1 construction site inspector, built on the **lightweight C++ architecture**.

### Key Principles

1. **Single C++ Binary** - No ROS2, no Python, no middleware overhead
2. **Unitree SDK Direct** - SDK handles DDS, we use high-level API
3. **Shared Core, Different I/O** - Same algorithms for sim and real robot
4. **Component Simulations** - Fast verification without hardware
5. **Agentic Development** - Each story has measurable verification criteria

### Technology Stack

| Component | Technology |
|-----------|------------|
| Language | C++17 |
| Robot SDK | unitree_sdk2 (DDS abstracted) |
| Image Processing | OpenCV |
| HTTP Client | curl |
| JSON | nlohmann/json |
| PDF | libharu |
| Build | CMake |
| Container | Docker |

---

## Epic 1: Construction Site Inspector MVP

**Goal:** Autonomous robot that navigates a construction site, captures images, detects defects via VLM API, and generates PDF reports.

**Total Stories:** 9

---

## Story 1: Project Setup + Docker Environment

As a developer,
I want a standardized, containerized development environment with all dependencies configured,
So that I can build and develop the system reproducibly on any platform.

**Scope:**
- Create multi-arch Dockerfile (supports Mac, Ubuntu 22.04 amd64, Ubuntu 20.04 arm64)
- Create compose.yaml for development (Docker Compose v2)
- Create .env.example for configuration
- Create CMakeLists.txt with all targets
- Set up directory structure (src/, sim/, test/, config/)
- Configure unitree_sdk2 as external dependency
- Add OpenCV, curl, nlohmann/json, libharu dependencies
- Create basic types (Point2D, Pose2D, Velocity)
- Create ILocomotion/ISensorSource interface stubs
- Create placeholder main.cpp that compiles
- Create GitHub Actions CI skeleton (.github/workflows/ci.yml)

**Acceptance Criteria:**
- `docker build` succeeds on Mac, Ubuntu 22.04 amd64, Ubuntu 20.04 arm64
- `docker compose up` starts development environment
- Inside container: `cmake ..` configures without errors
- Inside container: `make` builds successfully
- `./g1_inspector --help` shows usage
- All dependencies found and linked
- CI pipeline runs on push (build only for now)

**Verification:**
```bash
# Build and start dev environment
docker compose up -d --build

# Inside container
docker compose exec dev bash
mkdir build && cd build
cmake ..
make -j
./g1_inspector --help
# Should show: "G1 Inspector v1.0 - Usage: ..."
```

**Prerequisites:** None

---

## Story 2: Navigation + Simulation (NavSim)

As the robot,
I want path planning, obstacle avoidance, and a 2D simulation environment,
So that I can develop and test navigation without hardware.

**Scope:**
- Implement `Planner` class with A* algorithm
- Implement `Costmap` class (2D occupancy grid)
- Implement `PathFollower` class (velocity from path)
- Define `ILocomotion` interface for abstraction
- Implement `NavSim` class:
  - Load map from PNG (black=obstacle, white=free)
  - Robot as 2D point with pose (x, y, theta)
  - `applyVelocity()` integrates velocity to update pose
  - `simulateLidar()` raycasts to generate scan
  - `checkCollision()` tests if robot in obstacle
- Implement `SimLocomotion` (implements ILocomotion)
- Implement `SimSensorSource` (implements ISensorSource)
- Create `nav_sim` executable
- Output: trajectory.png, metrics.json, nav.log
- Unit tests for A* correctness

**Acceptance Criteria:**
- A* finds shortest path around obstacles
- Costmap updates from simulated scan data
- PathFollower produces smooth velocity commands
- NavSim loads test map
- Robot navigates from start to goal
- No collisions occur
- PNG shows trajectory, JSON shows metrics
- Unit tests pass

**Verification:**
```bash
./build/test_navigation
# All tests pass

./build/nav_sim --map test_data/office.png --start 1,1,0 --goal 8,5 --output outputs/

cat outputs/metrics.json
# {"goal_reached": true, "path_length": 12.3, "collisions": 0, "time_s": 4.2}

ls outputs/
# trajectory.png  metrics.json  nav.log
```

**Prerequisites:** Story 1

---

## Story 3: SLAM Core

As the robot,
I want to build a map from LiDAR scans,
So that I can localize and plan paths.

**Scope:**
- Implement `GridMapper` class:
  - Occupancy grid with log-odds updates
  - Bresenham ray tracing for free space
  - `update(pose, scan)` integrates new data
  - `saveMap(path)` exports as PNG
- Implement `Localizer` class (optional, odometry-only for MVP)
- Define `ISensorSource` interface
- Unit tests for map building
- Create `slam_sim` executable for testing with NavSim

**Acceptance Criteria:**
- GridMapper builds reasonable map from scans
- Map matches ground truth environment
- Unit tests pass

**Verification:**
```bash
./build/test_slam
# All tests pass

./build/slam_sim --map test_data/office.png --output outputs/
cat outputs/accuracy.json
# {"accuracy": 0.92, "cells_mapped": 4523}
```

**Prerequisites:** Story 2

---

## Story 4: Hardware Integration

As the system,
I want to connect to the robot hardware via SDK,
So that I can receive sensor data and send movement commands.

**Scope:**
- Implement `SensorManager` class:
  - Initialize SDK channel subscribers
  - Subscribe to LiDAR topic (`rt/utlidar/cloud`)
  - Subscribe to IMU topic (`rt/lowstate`)
  - Thread-safe data access with mutex or lock-free buffer
  - Callbacks for new data
- Implement `RealSensorSource` (implements ISensorSource)
- Parse SDK message types to internal types
- Implement `LocoController` class:
  - Initialize SDK LocoClient
  - `setVelocity(vx, vy, omega)` sends command
  - `stop()`, `standUp()`, `sitDown()`
  - `emergencyStop()` immediate halt
  - Connection status monitoring
- Implement `RealLocomotion` (implements ILocomotion)
- Safety limits on velocity commands
- Create hardware hello world integration test:
  - Connect to robot
  - Read battery level
  - Command stand up
  - Command simple motion (forward 0.5m)
  - Command stop
  - Read LiDAR scan

**Acceptance Criteria:**
- SensorManager connects to SDK topics
- LiDAR data parsed correctly
- IMU data parsed correctly
- LocoController connects to robot
- Velocity commands sent successfully
- Emergency stop works
- Velocity limits enforced
- Robot physically responds to commands (hello world test)

**Verification:**
```bash
# Requires real robot connected via ethernet
./scripts/check_g1_network.sh  # Verify connectivity first

# Test sensors
./build/g1_inspector --robot 192.168.123.164 --test-sensors
# [SENSORS] LiDAR: 3421 points received
# [SENSORS] IMU: orientation (0.01, 0.02, 0.00)
# [SENSORS] Battery: 85%

# Test locomotion
./build/g1_inspector --robot 192.168.123.164 --test-loco
# [LOCO] Connected to robot
# [LOCO] Standing up... OK
# [LOCO] Moving forward 0.2 m/s for 2s... OK
# [LOCO] Stopping... OK

# Full hardware hello world
./build/g1_inspector --robot 192.168.123.164 --hello-world
# Robot should: stand up, walk forward ~0.5m, stop
# Console shows sensor data
```

**Prerequisites:** Story 3

---

## Story 5: Safety System

As an operator,
I want safety monitoring and emergency stop,
So that operation is safe.

**Scope:**
- Implement `SafetyMonitor` class:
  - E-stop service (immediate halt <500ms)
  - Battery monitoring (warn 20%, return 10%)
  - Collision proximity check from costmap
  - Localization confidence monitoring
  - Publishes safety state
- Watchdog for command timeout
- Graceful degradation on sensor failure

**Acceptance Criteria:**
- E-stop halts robot within 500ms
- Low battery triggers warning/return
- Collision proximity detected
- Safety state queryable

**Verification:**
```bash
./build/g1_inspector --robot 192.168.123.164 --test-safety

# Tests:
# [SAFETY] E-stop test: PASS (320ms response)
# [SAFETY] Battery monitor: 85% (OK)
# [SAFETY] Collision check: No obstacles
# [SAFETY] All safety checks passed
```

**Prerequisites:** Story 4

---

## Story 6: State Machine + CLI + Plan Management

As an operator,
I want to control inspections via CLI and upload construction plans,
So that I can manage the robot and define inspection areas easily.

**Scope:**
- Implement `PlanManager` class:
  - `loadPlan(path, trade_type)` - Load PDF or PNG construction plan
  - `parsePdf(path)` - Render PDF to image using poppler
  - `parsePng(path)` - Direct OpenCV load
  - `extractWalls()` - Threshold to binary, create occupancy grid
  - `generateWaypoints()` - Grid-based coverage waypoints
  - `setStartPosition(position, orientation)` - Set robot origin on plan
  - `robotToPlanCoords()` / `planToRobotCoords()` - Coordinate transforms
- Implement `StateMachine` class:
  - States: IDLE, CALIBRATING, INSPECTING, PAUSED, BLOCKED, COMPLETE, EMERGENCY_STOP, RETURNING_HOME
  - Transitions per architecture diagram
  - Status publishing (state, battery, location, completion %)
- Implement CLI commands:
  - `status`, `start`, `pause`, `resume`, `stop`, `estop`
  - `upload --plan <file> [--trade finishes]` - Load and parse plan
  - `calibrate --position x,y,theta` - Set robot position on plan
  - `waypoints` - Show generated inspection waypoints
  - `report`, `help`
- Interactive mode and single-command mode

**Acceptance Criteria:**
- All CLI commands work
- State transitions correct
- Status shows real-time info
- E-stop always available
- PDF and PNG plans load successfully
- Waypoints generated from plan
- Coordinate transforms work correctly

**Verification:**
```bash
./build/g1_inspector --robot 192.168.123.164

> status
State: IDLE
Battery: 85%
Location: (0.0, 0.0, 0.0)

> upload --plan test_data/floor.png --trade finishes
Plan loaded: floor.png
  Size: 1200x800 px (24.0 x 16.0 m)
  Trade: finishes
  Waypoints: 47 generated

> waypoints
Waypoint 1: (2.0, 1.5)
Waypoint 2: (4.0, 1.5)
...
Total: 47 waypoints

> calibrate --position 0,0,0
Calibration complete
Robot origin set at plan position (0, 0)

> start
Inspection started
State: INSPECTING

> pause
Inspection paused

> stop
Inspection stopped
State: IDLE
```

**Prerequisites:** Story 5

---

## Story 7: Visual Capture

As the robot,
I want to capture geotagged images during inspection,
So that defects can be detected from photos.

**Scope:**
- Implement `ImageCapture` class:
  - Capture RGB at 1fps during INSPECTING
  - Tag with timestamp, robot pose, camera orientation
  - Save to disk with metadata JSON
  - Aligned depth capture (optional for MVP)
- Implement `PlanCorrelator`:
  - Transform pose to plan coordinates
  - Calculate FOV coverage
- Storage management (directory per inspection)

**Acceptance Criteria:**
- Images captured at 1fps
- Each image has pose metadata
- Images saved to inspection directory
- Coverage tracked

**Verification:**
```bash
# Run short inspection
./build/g1_inspector --robot 192.168.123.164
> start
# Wait 30 seconds
> stop

# Check captures
ls data/inspections/insp_001/images/
# img_0001.jpg  img_0001.json  img_0002.jpg  img_0002.json  ...

cat data/inspections/insp_001/images/img_0001.json
# {"timestamp": 1701705600, "pose": {"x": 1.2, "y": 0.5, "theta": 0.1}, ...}
```

**Prerequisites:** Story 6

---

## Story 8: VLM Defect Detection

As the system,
I want to analyze images via VLM API,
So that defects are detected automatically.

**Scope:**
- Implement `VlmClient` class:
  - HTTP POST to Anthropic/OpenAI API
  - Base64 encode images
  - Structured prompt for defect detection
  - Parse JSON response to Defect structs
  - Retry with backoff on failure
- Defect types: location_error, quality_issue
- Confidence scoring
- Annotated image generation (bounding boxes)

**Acceptance Criteria:**
- Images sent to VLM API
- Defects returned with type, description, location, confidence
- Annotated images generated
- API errors handled gracefully

**Verification:**
```bash
export ANTHROPIC_API_KEY="your-key"

./build/detection_sim --images test_data/defect_samples/ --plan test_data/floor.png --output outputs/

cat outputs/defects.json
# [
#   {"id": "def_001", "type": "quality_issue", "description": "Scratch on tile",
#    "location": {"x": 2.5, "y": 1.2}, "confidence": 0.87},
#   ...
# ]

ls outputs/annotated/
# img_001_annotated.jpg  img_003_annotated.jpg
```

**Prerequisites:** Story 7

---

## Story 9: Report Generation

As an operator,
I want PDF reports with punch lists,
So that I can share findings with crews.

**Scope:**
- Implement `ReportGenerator` class:
  - PDF with: summary, plan overlay, photos, defect list, punch list
  - Plan overlay with numbered markers at defect locations
  - Photo thumbnails with annotations
  - CSV export for punch list
- Template-based layout
- Naming: `inspection_<date>_<plan>.pdf`

**Acceptance Criteria:**
- PDF generated with all sections
- Plan overlay shows defect markers
- Punch list in table format
- CSV export works

**Verification:**
```bash
./build/g1_inspector --generate-report --inspection insp_001

# Output:
# Report generated: data/reports/insp_001/inspection_2025-12-04_floor.pdf
# Punch list: data/reports/insp_001/punch_list.csv

# Open and verify PDF has all sections
```

**Prerequisites:** Story 8

---

## Story Dependency Graph

```
Story 1 (Setup + Docker)
    │
    ▼
Story 2 (Navigation + NavSim) ◄─────────────┐
    │                                        │
    ▼                                        │ (uses for testing)
Story 3 (SLAM)                               │
    │                                        │
    ▼                                        │
Story 4 (Hardware Integration)               │
    │                                        │
    ▼                                        │
Story 5 (Safety)                             │
    │                                        │
    ▼                                        │
Story 6 (State Machine + CLI)                │
    │                                        │
    ▼                                        │
Story 7 (Visual Capture)                     │
    │                                        │
    ▼                                        │
Story 8 (VLM Detection)                      │
    │                                        │
    ▼                                        │
Story 9 (Report Gen) ────────────────────────┘
```

---

## Agentic Development Verification

Each story has verification commands that can be run automatically by Claude Code:

| Story | Verification Method |
|-------|---------------------|
| 1 | Docker builds, binary runs |
| 2 | Unit tests pass, NavSim outputs trajectory.png + metrics.json |
| 3 | Unit tests pass, slam_sim accuracy |
| 4 | Sensor data received, loco commands work, robot moves |
| 5 | Safety tests pass |
| 6 | CLI commands work, plan loads, waypoints generated |
| 7 | Images captured with metadata |
| 8 | VLM returns defects JSON |
| 9 | PDF generated |

### Feedback Levels

- **Level 1 (Real-time):** Log stream during sim run
- **Level 2 (Per-sim):** Analyze outputs after sim
- **Level 3 (Per-story):** All acceptance criteria checked
- **Level 4 (Hardware):** Human verifies on real robot

---

## Story Deliverables (What You Get When Done)

| Story | Deliverable | Verification |
|-------|-------------|--------------|
| **1: Project Setup + Docker** | Cross-platform dev environment | `docker compose up` → shell in → `cmake && make` → `./g1_inspector --help` prints usage. CI green. |
| **2: Navigation + NavSim** | 2D navigation simulation | `./nav_sim --map office.png --goal 8,5` → outputs `trajectory.png` + `metrics.json` with `goal_reached: true` |
| **3: SLAM Core** | Map building from scans | `./slam_sim --map office.png` → outputs `built_map.png` + `accuracy.json` showing >90% match |
| **4: Hardware Integration** | Robot responds to commands | `./g1_inspector --robot 192.168.123.164 --hello-world` → Robot stands, walks 0.5m, stops. LiDAR/IMU data shown. |
| **5: Safety System** | E-stop and monitoring | `./g1_inspector --test-safety` → E-stop <500ms. Battery %. Collision detection works. |
| **6: State Machine + CLI + Plan** | Full operator control | `upload --plan floor.png` → `calibrate` → `start` → inspection runs. `pause`/`resume`/`stop` work. |
| **7: Visual Capture** | Geotagged images saved | 30s inspection → `data/inspections/` has `img_*.jpg` + `img_*.json` with pose metadata. |
| **8: VLM Detection** | AI defect analysis | `./detection_sim --images samples/` → `defects.json` with type, location, confidence. Annotated images. |
| **9: Report Generation** | PDF inspection report | `./g1_inspector --generate-report` → PDF with summary, plan overlay, photos, punch list. CSV export. |

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Stories** | 9 |
| **Simulation-testable** | 1-3, 8 |
| **Requires hardware** | 4-7, 9 |
| **Core navigation** | 2-3 |
| **SDK integration** | 4 |

### Critical Path

1. Setup + Docker → 2. Nav + Sim → 3. SLAM → 4. Hardware → 5. Safety → 6. State Machine → 7. Capture → 8. VLM → 9. Report

After Story 4, we have hardware connectivity proven. Remaining stories build on that foundation.

---

---

## Epic 2: E2E Testing Infrastructure

**Goal:** Capture real sensor data via teleoperation, replay it offline, and validate the full inspection pipeline without requiring hardware for every test run.

**Total Stories:** 3

**Rationale:** Current tests rely heavily on mocking and synthetic data. Real recorded data provides:
- Actual sensor noise and drift characteristics
- Real defect images for VLM validation
- Ground truth positions for localization accuracy testing
- CI-compatible testing without robot hardware

---

## Story 2-1: Teleop and Sensor Recording

As a **developer**,
I want **to teleoperate the robot while recording all sensor data**,
So that **I can capture real-world data for E2E testing without synthetic noise**.

**Scope:**
- Implement `TeleopController` class (parse gamepad from `wireless_remote[40]`)
- Implement `KeyboardTeleop` class (OpenCV window with camera feed, WASD control)
- Implement `SensorRecorder` class (stream LiDAR, IMU, pose, images to compressed binary)
- Add msgpack-c and zstd dependencies for high-performance binary serialization
- CLI commands: `--teleop gamepad`, `--teleop keyboard`, `--record <session_id>`
- File format: MessagePack + zstd compression (~8 MB/min for full sensor suite)
- Visual feedback during recording (frame counter, disk usage, duration)

**Acceptance Criteria:**
- Gamepad teleop works (left stick = movement, right stick = rotation, buttons = stand/sit)
- Keyboard teleop with camera view (WASD controls, live camera feed)
- Sensor data recorded to compressed binary format
- Recording maintains real-time performance (<5% CPU overhead)
- Session metadata saved (start time, duration, robot ID, plan loaded)
- Graceful stop on Ctrl+C or 'q' key (flushes all buffers)

**Verification:**
```bash
# Gamepad teleop with recording
./build/g1_inspector --teleop gamepad --record my_session_001 --plan floor.png

# Check recording output
ls data/recordings/my_session_001/
# sensors.bin.zst  metadata.json  images/

cat data/recordings/my_session_001/metadata.json
# {"session_id": "my_session_001", "duration_s": 300, "lidar_count": 3000, ...}
```

**Prerequisites:** Story 4 (Hardware Integration)

---

## Story 2-2: Sensor Replay System

As a **developer**,
I want **to replay recorded sensor data through the system**,
So that **I can test the full pipeline with real-world data offline**.

**Scope:**
- Implement `SensorReplayer` class (decompress and decode recorded sensor stream)
- Implement `ReplaySensorSource` class (implements `ISensorSource` for replay mode)
- Implement `ReplayController` class (playback controls: speed, pause, seek, loop)
- Variable playback speed: 0.25x, 0.5x, 1.0x, 2.0x, 4.0x
- Seek to any point in recording (by time or percentage)
- Ground truth pose accessible separately from estimated pose
- CLI command: `--replay <session_id>` runs pipeline with recorded data

**Acceptance Criteria:**
- Can replay recorded sensor data from Story 10 format
- Real-time playback (sensors delivered at original timestamps)
- `ReplaySensorSource` implements `ISensorSource` interface (drop-in replacement)
- Ground truth pose accessible for validation
- Variable speed and seek controls work
- Loop mode for continuous testing

**Verification:**
```bash
# Basic replay
./build/g1_inspector --replay my_session_001

# Replay at 2x speed with visualization
./build/g1_inspector --replay my_session_001 --replay-speed 2.0 --replay-visualize

# Replay in loop mode
./build/g1_inspector --replay my_session_001 --replay-loop
```

**Prerequisites:** Story 2-1 (Teleop + Sensor Recording)

---

## Story 2-3: E2E Replay Test

As a **developer**,
I want **an end-to-end test using real recorded data**,
So that **I can validate the full inspection pipeline before deploying to the robot**.

**Scope:**
- Implement real localization algorithm (scan matching against built map)
- Implement `E2EReplayTest` class (orchestrates full pipeline test)
- Implement `GroundTruthValidator` class (compares estimated vs actual results)
- Implement `TestScenario` format (JSON files defining expected defects, positions)
- Test report generation (accuracy metrics, detected vs expected defects, coverage)
- CI-compatible (headless mode, no GUI required)
- Deterministic results with seeded random

**Acceptance Criteria:**
- E2E test runs full pipeline: replay → localization → capture → detection → report
- Localization accuracy validated (estimated position within 0.3m of ground truth)
- Defect detection validated (>80% recall on known defects)
- Deterministic results with seeded random (same seed = same results)
- Test scenarios defined in configuration files (not hardcoded)
- Test report generated with clear pass/fail and metrics
- Works in CI without GUI or real hardware

**Verification:**
```bash
# Run E2E test with test scenario
./build/test_e2e_replay --scenario data/test_scenarios/room_001.json

# Output:
# E2E Test Report: room_001
# Localization: PASS (mean error 0.15m, max 0.28m)
# Detection: PASS (85% recall, 90% precision)
# Coverage: PASS (87% actual vs 85% expected)
# Overall: PASS

# Run in CI mode
./build/test_e2e_replay --mock-vlm --headless
```

**Prerequisites:** Story 2-2 (Replay System)

---

## Updated Story Dependency Graph

```
Epic 1: Construction Site Inspector MVP (Complete)
Story 1-1 (Setup) → 1-2 (Nav) → 1-3 (SLAM) → 1-4 (Hardware) → 1-5 (Safety) → 1-6 (CLI) → 1-7 (Capture) → 1-8 (VLM) → 1-9 (Report)

Epic 2: E2E Testing Infrastructure
                                    Story 1-4 (Hardware)
                                         │
                                         ▼
                                    Story 2-1 (Teleop + Recording)
                                         │
                                         ▼
                                    Story 2-2 (Replay System)
                                         │
                                         ▼
                                    Story 2-3 (E2E Replay Test)
```

---

## Updated Summary

| Metric | Value |
|--------|-------|
| **Total Epics** | 2 |
| **Total Stories** | 12 |
| **Epic 1 Stories** | 9 (Complete) |
| **Epic 2 Stories** | 3 (New) |
| **Simulation-testable** | 1-3, 8, 11-12 |
| **Requires hardware** | 4-7, 9, 10 |

---

## Deferred to Post-MVP

- **Advanced CI/CD** - Full pipeline with integration tests
- **Performance Benchmarks** - Automated perf validation
- **Multi-robot coordination** - Fleet management

---

---

## Epic 3: Autonomous Navigation with Live Mapping

**Goal:** Robot builds a map while being teleoperated (with real-time visualization), saves the map, then autonomously navigates using that map with proper localization.

**User Value:** Operator can drive robot to create a map, see it build in real-time, save it, and then have the robot autonomously navigate waypoints on that map.

**Total Stories:** 6

**PRD Coverage:** FR12-22 (Autonomous Navigation + Localization), FR26 (Interior Representation)

**Technical Context:**
- GridMapper exists and works (log-odds occupancy grid)
- Localizer is currently a stub (odometry pass-through only)
- Planner (A*) and PathFollower exist and work
- PlanManager auto-generates waypoints from maps
- Teleop and recording already implemented (Epic 2)

---

## Story 3-1: Real-Time SLAM Visualizer

As an **operator**,
I want **to see the occupancy grid map build in real-time while I drive the robot**,
So that **I can verify coverage and map quality during teleop mapping runs**.

**Scope:**
- Implement `SlamVisualizer` class:
  - OpenCV window showing occupancy grid (grayscale: black=occupied, white=free, gray=unknown)
  - Robot pose marker (triangle or arrow showing position + heading)
  - LiDAR rays overlay (optional, toggleable)
  - Real-time update at 10Hz
  - Window controls: zoom (scroll), pan (drag), reset view (r key)
- Integrate with existing `KeyboardTeleop`:
  - Second window or split view showing map alongside camera
  - Map updates as robot moves and scans environment
- CLI flag: `--visualize-slam` enables the visualizer
- Display stats overlay: cells mapped, robot pose, scan count

**Acceptance Criteria:**
- OpenCV window shows occupancy grid updating in real-time
- Robot position and heading visible on map
- LiDAR rays can be toggled on/off
- Zoom and pan controls work
- Stats overlay shows mapping progress
- Visualizer runs at 10Hz without impacting teleop performance
- Works with both keyboard and gamepad teleop

**Verification:**
```bash
# Teleop with SLAM visualization
./build/g1_inspector --teleop keyboard --visualize-slam

# Window shows:
# - Grayscale occupancy grid growing as robot explores
# - Blue triangle = robot position/heading
# - Green lines = current LiDAR rays (if enabled)
# - Stats: "Cells: 4523 | Pose: (2.3, 1.5, 0.4) | Scans: 342"

# Controls:
# - Scroll wheel: zoom in/out
# - Click+drag: pan view
# - 'r': reset view to robot
# - 'l': toggle LiDAR rays
# - 'q': quit
```

**Prerequisites:** Story 2-1 (Teleop), Story 3 (SLAM Core / GridMapper)

**Technical Notes:**
- Use existing `GridMapper::getMap()` for occupancy data
- Convert log-odds to grayscale: `pixel = 255 * (1 - sigmoid(log_odds))`
- Robot pose from odometry (IMU integration)
- Render at lower resolution if needed for performance (e.g., 2x downscale)

---

## Story 3-2: Map Save and Load

As an **operator**,
I want **to save the built map after a teleop run and load it later for navigation**,
So that **I don't need to rebuild the map every time I want the robot to navigate**.

**Scope:**
- Implement `MapManager` class:
  - `saveMap(path)` - Save occupancy grid as PNG + metadata JSON
  - `loadMap(path)` - Load saved map for navigation
  - Metadata: resolution, origin, dimensions, creation timestamp, robot start pose
- Map format:
  - PNG: Grayscale occupancy (0=occupied, 255=free, 128=unknown)
  - JSON: `{resolution: 0.05, origin: {x, y}, width: 200, height: 150, created: "2025-12-07T10:30:00Z"}`
- CLI commands:
  - `--save-map <path>` - Save current map when exiting teleop
  - `--load-map <path>` - Load map for autonomous navigation
- Auto-save option: save map on clean exit from teleop mode
- Integrate with `PlanManager` - loaded maps can be used for waypoint generation

**Acceptance Criteria:**
- Map saves as PNG + JSON metadata
- Saved map loads correctly
- Loaded map dimensions and resolution preserved
- Waypoints can be generated from loaded map
- Auto-save works on clean teleop exit

**Verification:**
```bash
# Teleop and save map
./build/g1_inspector --teleop keyboard --visualize-slam --save-map data/maps/office_001

# Check saved files
ls data/maps/office_001/
# map.png  metadata.json

cat data/maps/office_001/metadata.json
# {"resolution": 0.05, "origin": {"x": 0, "y": 0}, "width": 400, "height": 300, ...}

# Load map and generate waypoints
./build/g1_inspector --load-map data/maps/office_001
> waypoints
# Waypoint 1: (2.0, 1.5)
# Waypoint 2: (4.0, 1.5)
# ...
# Total: 23 waypoints generated from map
```

**Prerequisites:** Story 3-1 (SLAM Visualizer)

**Technical Notes:**
- PNG uses OpenCV `imwrite` with grayscale
- Resolution typically 0.05m/cell (5cm)
- Origin is bottom-left corner of map in world coordinates
- Metadata JSON uses nlohmann/json

---

## Story 3-3: Scan-Matching Localizer

As the **robot**,
I want **to know my position on a saved map using LiDAR scan matching**,
So that **I can navigate autonomously without accumulating odometry drift**.

**Scope:**
- Implement `ScanMatchingLocalizer` class:
  - Replace stub `Localizer` with real implementation
  - ICP (Iterative Closest Point) or correlation-based scan matching
  - Input: current LiDAR scan + occupancy grid map
  - Output: corrected pose estimate
  - Fuse with odometry for smooth updates
- Localization confidence score:
  - Based on scan match quality (residual error)
  - Threshold for "lost" detection
- Initial pose estimation:
  - Manual calibration (operator sets start position)
  - Future: automatic pose recovery
- Update rate: 10Hz (matches LiDAR rate)

**Acceptance Criteria:**
- Localizer estimates pose from scan matching against map
- Pose estimate within 0.1m of ground truth in known environment
- Confidence score reflects match quality
- "Lost" state detected when match quality drops below threshold
- Works with maps saved from Story 3-2
- 10Hz update rate maintained

**Verification:**
```bash
# Test localization with replay data
./build/g1_inspector --replay my_session_001 --load-map data/maps/office_001 --test-localizer

# Output:
# [LOCALIZER] Initialized with map: office_001 (400x300, 0.05m/cell)
# [LOCALIZER] Initial pose set: (0.0, 0.0, 0.0)
# [LOCALIZER] Frame 1: pose=(0.12, 0.05, 0.02), confidence=0.95
# [LOCALIZER] Frame 2: pose=(0.25, 0.08, 0.03), confidence=0.94
# ...
# [LOCALIZER] Test complete: mean_error=0.08m, max_error=0.15m

# Test with ground truth comparison
./build/test_localizer --replay my_session_001 --map data/maps/office_001
# Localization Test Results:
# - Mean position error: 0.08m
# - Max position error: 0.15m
# - Mean heading error: 2.3°
# - Frames with confidence < 0.5: 3/500 (0.6%)
# PASS
```

**Prerequisites:** Story 3-2 (Map Save/Load)

**Technical Notes:**
- Start with simple correlation-based matching (fast, good enough for indoor)
- Search window: ±0.5m translation, ±10° rotation from odometry prediction
- Score = sum of occupied cells hit by scan rays
- Use coarse-to-fine for speed (4x downscale first, then refine)
- Fuse with odometry using simple weighted average (80% scan match, 20% odometry)

---

## Story 3-4: Autonomous Navigation Loop

As the **robot**,
I want **to autonomously navigate through a sequence of waypoints**,
So that **I can perform inspections without manual control**.

**Scope:**
- Implement `AutonomousController` class:
  - Main loop: sense → localize → plan → move
  - Waypoint sequencing from `PlanManager::getInspectionWaypoints()`
  - Goal reaching detection (within tolerance)
  - Progress tracking (current waypoint, completion %)
- Integrate existing components:
  - `ScanMatchingLocalizer` for pose
  - `Planner` (A*) for path to next waypoint
  - `PathFollower` for velocity commands
  - `LocoController` to send commands to robot
- State machine integration:
  - INSPECTING state runs autonomous loop
  - Transition to BLOCKED if path fails
  - Transition to COMPLETE when all waypoints visited
- CLI command: `start` begins autonomous navigation (existing)

**Acceptance Criteria:**
- Robot autonomously navigates to each waypoint in sequence
- Goal tolerance: within 0.3m of waypoint center
- Path replanning on each loop iteration (handles minor drift)
- Progress displayed: "Waypoint 3/15 (20%)"
- Smooth velocity commands (no jerky motion)
- Stops cleanly at final waypoint

**Verification:**
```bash
# Load map and start autonomous navigation
./build/g1_inspector --robot 192.168.123.164 --load-map data/maps/office_001

> status
State: IDLE
Map: office_001 loaded
Waypoints: 15

> start
[AUTO] Starting autonomous navigation
[AUTO] Waypoint 1/15: (2.0, 1.5)
[AUTO] Planning path... 12 cells
[AUTO] Following path...
[AUTO] Reached waypoint 1
[AUTO] Waypoint 2/15: (4.0, 1.5)
...
[AUTO] Waypoint 15/15: (8.0, 6.0)
[AUTO] Reached waypoint 15
[AUTO] Navigation complete
State: COMPLETE

> status
State: COMPLETE
Waypoints visited: 15/15 (100%)
```

**Prerequisites:** Story 3-3 (Scan-Matching Localizer)

**Technical Notes:**
- Loop rate: 10Hz
- Replan every iteration (simple, handles dynamic changes)
- Goal tolerance: 0.3m position, 15° heading (configurable)
- Velocity limits from SafetyMonitor
- Use existing PathFollower pure pursuit implementation

---

## Story 3-5: Blocked Path Handling

As an **operator**,
I want **the robot to detect when it's stuck and notify me**,
So that **I can intervene when autonomous navigation fails**.

**Scope:**
- Implement blocked detection in `AutonomousController`:
  - Path planning failure (no valid path to goal)
  - Stall detection (commanding velocity but not moving)
  - Localization failure (confidence too low)
- Recovery attempts:
  - Replan up to 3 times with increasing search radius
  - Back up slightly and retry (if stalled)
  - Stop and notify if recovery fails
- Operator notification:
  - Console message with reason and location
  - State transition to BLOCKED
  - Option to skip waypoint or abort
- CLI commands in BLOCKED state:
  - `skip` - Skip current waypoint, continue to next
  - `retry` - Attempt recovery again
  - `abort` - Stop navigation, return to IDLE

**Acceptance Criteria:**
- Robot detects blocked path (no valid route)
- Robot detects stall (movement timeout)
- Robot detects localization loss
- Recovery attempted before giving up
- Clear notification to operator with reason
- Skip/retry/abort commands work in BLOCKED state

**Verification:**
```bash
# Simulate blocked scenario (place obstacle in path)
./build/g1_inspector --robot 192.168.123.164 --load-map data/maps/office_001

> start
[AUTO] Waypoint 3/15: (6.0, 3.0)
[AUTO] Planning path... FAILED (no valid path)
[AUTO] Retry 1/3...
[AUTO] Planning path... FAILED
[AUTO] Retry 2/3...
[AUTO] Planning path... FAILED
[AUTO] Retry 3/3...
[AUTO] Planning path... FAILED
[BLOCKED] Navigation blocked at (4.2, 2.8)
[BLOCKED] Reason: No valid path to waypoint 3
[BLOCKED] Options: 'skip' | 'retry' | 'abort'
State: BLOCKED

> skip
[AUTO] Skipping waypoint 3
[AUTO] Waypoint 4/15: (6.0, 4.5)
[AUTO] Planning path... 18 cells
[AUTO] Following path...
```

**Prerequisites:** Story 3-4 (Autonomous Navigation Loop)

**Technical Notes:**
- Stall detection: if velocity commanded > 0.1 m/s but position change < 0.02m over 2 seconds
- Localization failure: confidence < 0.3 for 5 consecutive frames
- Path failure: A* returns empty path
- Back up: -0.1 m/s for 1 second before retry

---

## Story 3-6: End-to-End Autonomous Demo

As a **developer**,
I want **to validate the complete autonomous navigation pipeline**,
So that **I can confirm the system works before real-world deployment**.

**Scope:**
- Create demo scenario:
  - Pre-recorded teleop session with known ground truth
  - Saved map from that session
  - Defined waypoint sequence
  - Expected trajectory
- Implement `AutonomousDemo` test:
  - Can run in simulation (NavSim) or replay mode
  - Validates: localization accuracy, waypoint reaching, path quality
  - Generates test report
- Success criteria validation:
  - All waypoints reached (or appropriately skipped if blocked)
  - Localization error < 0.2m mean
  - No collisions
  - Completion within reasonable iterations
- Documentation: step-by-step demo guide

**Acceptance Criteria:**
- Demo runs successfully in simulation
- Demo runs successfully with replay data
- All waypoints reached or handled appropriately
- Localization accuracy meets threshold
- No collisions detected
- Test report generated with metrics
- Demo guide documents the full process

**Verification:**
```bash
# Run demo in simulation
./build/test_autonomous_demo --mode sim --map test_data/office.png

# Output:
# Autonomous Demo Test
# Mode: Simulation
# Map: office.png (400x300)
# Waypoints: 10
#
# Running...
# [1/10] Reached (2.0, 1.5) ✓
# [2/10] Reached (4.0, 1.5) ✓
# ...
# [10/10] Reached (8.0, 6.0) ✓
#
# Results:
# - Waypoints reached: 10/10 (100%)
# - Mean localization error: 0.12m
# - Max localization error: 0.19m
# - Collisions: 0
# - Total path length: 24.5m
# PASS

# Run with replay data
./build/test_autonomous_demo --mode replay --session my_session_001 --map data/maps/office_001

# Generate full report
./build/test_autonomous_demo --mode sim --report outputs/demo_report.json
```

**Prerequisites:** Story 3-5 (Blocked Path Handling)

**Technical Notes:**
- Simulation mode uses NavSim with simulated LiDAR
- Replay mode uses recorded sensor data
- Ground truth comparison for localization validation
- Test report JSON includes all metrics for CI integration

---

## Epic 3 Story Dependency Graph

```
Epic 2 (Teleop/Recording)
         │
         ▼
    Story 3-1 (SLAM Visualizer)
         │
         ▼
    Story 3-2 (Map Save/Load)
         │
         ▼
    Story 3-3 (Scan-Matching Localizer)
         │
         ▼
    Story 3-4 (Autonomous Navigation Loop)
         │
         ▼
    Story 3-5 (Blocked Path Handling)
         │
         ▼
    Story 3-6 (E2E Demo)
```

**Parallelization Opportunity:**
- Story 3-1 (Visualizer) can be done first, enabling data collection
- While operator collects data with 3-1, stories 3-3 through 3-6 can be developed
- Story 3-2 (Save/Load) is quick and unblocks 3-3

---

## Epic 3 Deliverables Summary

| Story | Deliverable | Verification |
|-------|-------------|--------------|
| **3-1: SLAM Visualizer** | Real-time map display during teleop | `--visualize-slam` shows growing occupancy grid with robot pose |
| **3-2: Map Save/Load** | Persistent maps for navigation | `--save-map` creates PNG+JSON, `--load-map` restores for navigation |
| **3-3: Localizer** | Scan-matching pose estimation | Localization test shows <0.2m mean error vs ground truth |
| **3-4: Auto Nav Loop** | Autonomous waypoint navigation | Robot visits all waypoints autonomously, progress shown |
| **3-5: Blocked Handling** | Stuck detection and recovery | Robot detects blocks, notifies operator, skip/retry/abort work |
| **3-6: E2E Demo** | Full pipeline validation | Demo test passes in sim and replay modes |

---

## Updated Summary

| Metric | Value |
|--------|-------|
| **Total Epics** | 3 |
| **Total Stories** | 18 |
| **Epic 1 Stories** | 9 (Complete) |
| **Epic 2 Stories** | 3 (2 Complete, 1 Ready) |
| **Epic 3 Stories** | 6 (New) |

---

## FR Coverage Matrix (Epic 3)

| FR | Description | Story |
|----|-------------|-------|
| FR12 | Robot navigates autonomously through indoor environments | 3-4 |
| FR13 | Robot traverses stairs during inspection | 3-4 (uses existing locomotion) |
| FR14 | Robot navigates around static obstacles | 3-4, 3-5 |
| FR15 | Robot detects when planned path is blocked | 3-5 |
| FR16 | Robot attempts alternate routes when blocked | 3-5 |
| FR17 | Robot stops and notifies operator when no viable path | 3-5 |
| FR18 | System tracks route completion percentage | 3-4 |
| FR19 | Robot self-localizes using plans and sensors | 3-3 |
| FR20 | Robot maintains position awareness throughout route | 3-3, 3-4 |
| FR21 | Robot detects when localization confidence is low | 3-3, 3-5 |
| FR22 | Robot stops and notifies when localization fails | 3-5 |
| FR26 | System builds interior representation from sensor data | 3-1, 3-2 |

---

_Generated for Lightweight C++ Architecture v2.0_
_Updated: 2025-12-07 (Epic 3 added for Autonomous Navigation with Live Mapping)_
