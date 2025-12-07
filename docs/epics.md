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

_Generated for Lightweight C++ Architecture v2.0_
_Updated: 2025-12-06 (Epic 2 stories 2-1 and 2-2 completed, 2-3 ready for development)_
