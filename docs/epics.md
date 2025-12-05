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

---

## Epic 1: Construction Site Inspector MVP

**Goal:** Autonomous robot that navigates a construction site, captures images, detects defects via VLM API, and generates PDF reports.

**Total Stories:** 14

---

## Story 1: Project Setup

As a developer,
I want the C++ project structure with build system and dependencies configured,
So that I can build and develop the system.

**Scope:**
- Create CMakeLists.txt with all targets
- Set up directory structure (src/, sim/, test/, config/)
- Configure unitree_sdk2 as external dependency
- Add OpenCV, curl, nlohmann/json dependencies
- Create basic types (Point2D, Pose2D, Velocity)
- Create placeholder main.cpp that compiles

**Acceptance Criteria:**
- `cmake ..` configures without errors
- `make` builds successfully
- `./g1_inspector --help` shows usage
- All dependencies found and linked

**Verification:**
```bash
mkdir build && cd build
cmake ..
make -j
./g1_inspector --help
# Should show: "G1 Inspector v1.0 - Usage: ..."
```

**Prerequisites:** None

---

## Story 2: Navigation Core

As the robot,
I want path planning and obstacle avoidance algorithms,
So that I can navigate autonomously.

**Scope:**
- Implement `Planner` class with A* algorithm
- Implement `Costmap` class (2D occupancy grid)
- Implement `PathFollower` class (velocity from path)
- Define `ILocomotion` interface for abstraction
- Unit tests for A* correctness

**Acceptance Criteria:**
- A* finds shortest path around obstacles
- Costmap updates from simulated scan data
- PathFollower produces smooth velocity commands
- Unit tests pass

**Verification:**
```bash
./build/test_navigation
# All tests pass

# Manual verification with NavSim (Story 3)
```

**Prerequisites:** Story 1

---

## Story 3: Navigation Simulation (NavSim)

As a developer,
I want a 2D navigation simulation,
So that I can test navigation without hardware.

**Scope:**
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

**Acceptance Criteria:**
- NavSim loads test map
- Robot navigates from start to goal
- No collisions occur
- PNG shows trajectory, JSON shows metrics

**Verification:**
```bash
./build/nav_sim --map test_data/office.png --start 1,1,0 --goal 8,5 --output outputs/

# Check outputs
cat outputs/metrics.json
# {"goal_reached": true, "path_length": 12.3, "collisions": 0, "time_s": 4.2}

ls outputs/
# trajectory.png  metrics.json  nav.log
```

**Prerequisites:** Story 2

---

## Story 4: SLAM Core

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

**Prerequisites:** Story 3

---

## Story 5: Sensor Interface

As the system,
I want to receive sensor data from the robot via SDK,
So that navigation and SLAM have real data.

**Scope:**
- Implement `SensorManager` class:
  - Initialize SDK channel subscribers
  - Subscribe to LiDAR topic (`rt/utlidar/cloud`)
  - Subscribe to IMU topic (`rt/lowstate`)
  - Thread-safe data access with mutex or lock-free buffer
  - Callbacks for new data
- Implement `RealSensorSource` (implements ISensorSource)
- Parse SDK message types to internal types

**Acceptance Criteria:**
- SensorManager connects to SDK topics
- LiDAR data parsed correctly
- IMU data parsed correctly
- Data accessible from main thread

**Verification:**
```bash
# Requires connection to robot or SDK mock
./build/g1_inspector --robot 192.168.123.164 --test-sensors

# Output:
# [SENSORS] LiDAR: 3421 points received
# [SENSORS] IMU: orientation (0.01, 0.02, 0.00)
# [SENSORS] Battery: 85%
```

**Prerequisites:** Story 4

---

## Story 6: Locomotion Interface

As the system,
I want to send velocity commands to the robot via SDK,
So that navigation can control movement.

**Scope:**
- Implement `LocoController` class:
  - Initialize SDK LocoClient
  - `setVelocity(vx, vy, omega)` sends command
  - `stop()`, `standUp()`, `sitDown()`
  - `emergencyStop()` immediate halt
  - Connection status monitoring
- Implement `RealLocomotion` (implements ILocomotion)
- Safety limits on velocity commands

**Acceptance Criteria:**
- LocoController connects to robot
- Velocity commands sent successfully
- Emergency stop works
- Velocity limits enforced

**Verification:**
```bash
# Requires real robot
./build/g1_inspector --robot 192.168.123.164 --test-loco

# Output:
# [LOCO] Connected to robot
# [LOCO] Standing up... OK
# [LOCO] Moving forward 0.2 m/s for 2s... OK
# [LOCO] Stopping... OK
```

**Prerequisites:** Story 5

---

## Story 7: Hardware Hello World

As a developer,
I want to verify full hardware connectivity,
So that I confirm the robot responds to commands.

**Scope:**
- Create integration test that:
  - Connects to robot
  - Reads battery level
  - Commands stand up
  - Commands simple motion (forward 0.5m)
  - Commands stop
  - Reads LiDAR scan
- All using real SDK, real robot

**Acceptance Criteria:**
- Robot physically responds to commands
- Sensor data streams back
- No crashes or errors
- Battery level displayed

**Verification:**
```bash
# Robot must be powered on, connected via ethernet
./scripts/check_g1_network.sh  # Verify connectivity first

./build/g1_inspector --robot 192.168.123.164 --hello-world

# Robot should:
# 1. Stand up
# 2. Walk forward ~0.5m
# 3. Stop
# 4. Console shows sensor data
```

**Prerequisites:** Story 6

---

## Story 8: Safety System

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

**Prerequisites:** Story 7

---

## Story 9: State Machine + CLI

As an operator,
I want to control inspections via CLI,
So that I can manage the robot easily.

**Scope:**
- Implement `StateMachine` class:
  - States: IDLE, CALIBRATING, INSPECTING, PAUSED, BLOCKED, COMPLETE, EMERGENCY_STOP, RETURNING_HOME
  - Transitions per architecture diagram
  - Status publishing (state, battery, location, completion %)
- Implement CLI commands:
  - `status`, `start`, `pause`, `resume`, `stop`, `estop`
  - `upload --plan <file>`, `calibrate --position x,y,theta`
  - `report`, `help`
- Interactive mode and single-command mode

**Acceptance Criteria:**
- All CLI commands work
- State transitions correct
- Status shows real-time info
- E-stop always available

**Verification:**
```bash
./build/g1_inspector --robot 192.168.123.164

> status
State: IDLE
Battery: 85%
Location: (0.0, 0.0, 0.0)

> upload --plan test_data/floor.png
Plan uploaded: plan_001

> calibrate --position 0,0,0
Calibration complete

> start
Inspection started
State: INSPECTING

> status
State: INSPECTING
Battery: 84%
Location: (1.2, 0.5, 0.1)
Completion: 15%

> pause
Inspection paused

> resume
Inspection resumed

> stop
Inspection stopped
State: IDLE
```

**Prerequisites:** Story 8

---

## Story 10: Visual Capture

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

**Prerequisites:** Story 9

---

## Story 11: VLM Defect Detection

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

**Prerequisites:** Story 10

---

## Story 12: Report Generation

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

**Prerequisites:** Story 11

---

## Story 13: Integration Testing

As a developer,
I want end-to-end testing in simulation,
So that the full pipeline works before hardware testing.

**Scope:**
- Create integration test scenarios:
  - Happy path: full inspection cycle in NavSim
  - Obstacle blocking: path replan
  - E-stop during inspection
  - Low battery return
- Structured logging throughout
- Performance validation:
  - <500ms obstacle response
  - 10Hz navigation loop
  - 95% route completion

**Acceptance Criteria:**
- All scenarios pass in simulation
- Performance metrics meet requirements
- Logs structured and filterable

**Verification:**
```bash
./scripts/run_integration_tests.sh

# Output:
# [TEST] Happy path: PASS
# [TEST] Obstacle replan: PASS
# [TEST] E-stop: PASS
# [TEST] Low battery: PASS
# [PERF] Obstacle response: 340ms (< 500ms) PASS
# [PERF] Nav loop: 12Hz (> 10Hz) PASS
# All integration tests passed
```

**Prerequisites:** Story 12

---

## Story 14: Docker Deployment

As a developer,
I want Docker images for deployment,
So that I can deploy reproducibly.

**Scope:**
- Create Dockerfile:
  - Ubuntu 22.04 base
  - Build dependencies
  - Compile project
  - Minimal runtime image
- Create docker-compose.yml for development
- Create .env.example for configuration
- Volume mounts for data and config

**Acceptance Criteria:**
- `docker build` succeeds
- `docker-compose up` starts system
- Container connects to robot
- Data persisted via volumes

**Verification:**
```bash
cd docker
docker build -t g1-inspector .
docker-compose up -d

# From host
docker exec -it g1-inspector g1_inspector --status
# State: IDLE

# With real robot
docker-compose -f docker-compose.robot.yml up
```

**Prerequisites:** Story 13

---

## Story Dependency Graph

```
Story 1 (Setup)
    │
    ▼
Story 2 (Nav Core)
    │
    ▼
Story 3 (NavSim) ◄─────────────────────────┐
    │                                       │
    ▼                                       │ (uses for testing)
Story 4 (SLAM)                              │
    │                                       │
    ▼                                       │
Story 5 (Sensors)                           │
    │                                       │
    ▼                                       │
Story 6 (Locomotion)                        │
    │                                       │
    ▼                                       │
Story 7 (Hardware Hello)                    │
    │                                       │
    ▼                                       │
Story 8 (Safety)                            │
    │                                       │
    ▼                                       │
Story 9 (State Machine + CLI)               │
    │                                       │
    ▼                                       │
Story 10 (Visual Capture)                   │
    │                                       │
    ▼                                       │
Story 11 (VLM Detection)                    │
    │                                       │
    ▼                                       │
Story 12 (Report Gen)                       │
    │                                       │
    ▼                                       │
Story 13 (Integration) ─────────────────────┘
    │
    ▼
Story 14 (Docker)
```

---

## Agentic Development Verification

Each story has verification commands that can be run automatically by Claude Code:

| Story | Verification Method |
|-------|---------------------|
| 1 | Build success, binary runs |
| 2 | Unit tests pass |
| 3 | NavSim outputs: trajectory.png, metrics.json |
| 4 | Unit tests pass, slam_sim accuracy |
| 5 | Sensor data received (requires robot or mock) |
| 6 | Loco commands sent (requires robot) |
| 7 | Robot physically moves (requires robot + human) |
| 8 | Safety tests pass |
| 9 | CLI commands work |
| 10 | Images captured with metadata |
| 11 | VLM returns defects JSON |
| 12 | PDF generated |
| 13 | All integration tests pass |
| 14 | Docker builds and runs |

### Feedback Levels

- **Level 1 (Real-time):** Log stream during sim run
- **Level 2 (Per-sim):** Analyze outputs after sim
- **Level 3 (Per-story):** All acceptance criteria checked
- **Level 4 (Hardware):** Human verifies on real robot

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Stories** | 14 |
| **Simulation-testable** | 1-4, 11-13 |
| **Requires hardware** | 5-10, 14 |
| **Core navigation** | 2-4 |
| **SDK integration** | 5-7 |

### Critical Path

1. Setup → 2. Nav Core → 3. NavSim → 4. SLAM → 5. Sensors → 6. Loco → 7. Hello World

After Story 7, we have hardware connectivity proven. Remaining stories build on that foundation.

---

_Generated for Lightweight C++ Architecture v2.0_
_Date: 2025-12-04_
