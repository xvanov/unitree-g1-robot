# unitree-g1-robot - Epic Breakdown

**Author:** BMAD
**Date:** 2025-12-04
**Project Type:** IoT/Embedded Robotics Integration
**Approach:** Integration of proven libraries and APIs (Nav2, slam_toolbox, Unitree SDK, VLM APIs)

---

## Overview

This document provides the epic and story breakdown for unitree-g1-robot. This is primarily **integration work** - connecting well-established robotics libraries and AI APIs, not building novel systems from scratch.

**Core Stack (Already Built):**
- Unitree SDK2 - Robot locomotion (proven, just call it)
- Nav2 + slam_toolbox - Navigation and SLAM (industry standard)
- RealSense + Livox drivers - Sensor data (plug and play)
- VLM APIs (GPT-4V/Claude) - Defect detection (API calls)
- ReportLab - PDF generation (standard library)

**Our Job:** Wire these together into an inspection workflow.

**Key Principle:** Each story delivers a runnable, testable increment. No story is "done" until you can demonstrate the functionality working.

---

## Epic 1: Construction Site Inspector MVP

**Goal:** Autonomous robot that navigates a construction site, captures images, detects defects via VLM, and generates PDF reports.

**Total Stories:** 11

| # | Story | Runnable Deliverable |
|---|-------|---------------------|
| 1 | Project Setup & ROS2 Workspace | `colcon build` succeeds, interfaces importable |
| 2 | Simulation Environment | Launch sim, see robot in RViz, teleop works |
| 3 | Navigation Stack Integration | Send nav goal, robot navigates autonomously |
| 4 | Localization & Safety Systems | Trigger E-stop, see safety behaviors |
| 5 | Plan Management & Calibration | Upload plan via CLI, calibrate robot |
| 6 | Inspection State Machine & CLI | Run full CLI workflow, see state transitions |
| 7 | Visual Capture Pipeline | Run inspection, see captured images with poses |
| 8 | VLM Defect Detection | Run detection on test images, see defect JSON |
| 9 | Report Generation | Generate PDF report from detection results |
| 10 | Integration Testing | Full end-to-end inspection in simulation |
| 11 | Docker Deployment | `docker-compose up` runs on real hardware |

---

## Story 1: Project Setup & ROS2 Workspace

As a developer,
I want the complete project structure with all packages and dependencies configured,
So that I can build and run the system.

**Scope:**
- Create ROS2 workspace with all 6 packages (g1_bringup, g1_navigation, g1_perception, g1_inspection, g1_safety, g1_interfaces)
- Define custom messages: InspectionStatus, DefectReport, Notification
- Define services: StartInspection, PauseInspection, GetStatus
- Define action: ExecuteInspection
- Create `scripts/setup.sh` that clones external deps (unitree_sdk2_python, unitree_ros2, cyclonedds, unitree_mujoco)
- Configure CycloneDDS as default middleware
- Create config file templates (nav2_params.yaml, slam_params.yaml, robot_params.yaml)

**Acceptance Criteria:**
- `scripts/setup.sh` runs successfully on fresh Ubuntu 22.04 + ROS2 Humble
- `colcon build` completes without errors
- All interfaces importable: `from g1_interfaces.msg import InspectionStatus`
- External deps cloned to `external/` (gitignored)

**Technical Notes:**
- Pin dependency commits for reproducibility
- Use ament_python for packages, ament_cmake for interfaces
- All nodes use `/g1/` namespace

**Prerequisites:** None

**🧪 Runnable Verification:**
```bash
# 1. Run setup (one time)
./scripts/setup.sh

# 2. Build workspace
cd ~/unitree-g1-robot
colcon build

# 3. Source and verify interfaces
source install/setup.bash
python3 -c "from g1_interfaces.msg import InspectionStatus; print('✅ Interfaces working')"
python3 -c "from g1_interfaces.srv import StartInspection; print('✅ Services working')"

# 4. List packages
ros2 pkg list | grep g1_
# Should show: g1_bringup, g1_navigation, g1_perception, g1_inspection, g1_safety, g1_interfaces
```

**Definition of Done:** All verification commands pass. Developer can import interfaces in Python.

---

## Story 2: Simulation Environment

As a developer,
I want a MuJoCo simulation with fake locomotion,
So that I can test navigation and perception without hardware.

**Scope:**
- Set up MuJoCo with G1 robot model (from unitree_mujoco)
- Implement `SimLocomotionController` - teleport-based fake walking
- Create simulated sensor publishers matching real topics:
  - `/g1/camera/rgb`, `/g1/camera/depth` (simulated images)
  - `/g1/lidar/points` (simulated point cloud)
  - `/g1/imu/data` (simulated IMU)
- Create `sim_launch.py` that starts simulation + RViz
- Simulated environment with walls/obstacles for testing
- Basic teleop node for manual control testing

**Acceptance Criteria:**
- `ros2 launch g1_bringup sim_launch.py` opens MuJoCo + RViz
- Robot responds to `/g1/cmd_vel` commands (teleports)
- All sensor topics publish at expected rates
- Can visualize robot and sensors in RViz
- Teleop keyboard control moves robot in simulation

**Technical Notes:**
- Fake locomotion integrates velocity over time, sets base pose directly
- Same topic names as real robot for code compatibility
- Simple indoor environment sufficient for MVP testing

**Prerequisites:** Story 1

**🧪 Runnable Verification:**
```bash
# Terminal 1: Launch simulation
ros2 launch g1_bringup sim_launch.py

# Terminal 2: Check topics are publishing
ros2 topic list | grep g1
# Should see: /g1/cmd_vel, /g1/camera/rgb, /g1/lidar/points, etc.

ros2 topic hz /g1/camera/rgb
# Should show ~1 Hz

ros2 topic hz /g1/lidar/points
# Should show ~10 Hz

# Terminal 3: Teleop the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/g1/cmd_vel
# Use keyboard to move robot, verify it moves in RViz
```

**Definition of Done:** Simulation launches, robot visible in RViz, teleop moves robot, sensor topics publishing.

---

## Story 3: Navigation Stack Integration

As the robot,
I want Nav2 and slam_toolbox integrated with the Unitree SDK,
So that I can autonomously navigate and avoid obstacles.

**Scope:**
- Implement `Nav2LocoBridge` node - subscribes `/g1/cmd_vel`, calls `LocoClient.SetVelocity()`
- Configure slam_toolbox for 2D SLAM from LiDAR
- Configure Nav2 with:
  - Global planner (NavFn or Smac)
  - Local planner (DWB)
  - Costmap with obstacle + inflation layers
  - Recovery behaviors
- Fuse D435i depth into costmap for low obstacle detection
- Implement path replanning when obstacles detected
- Track coverage percentage based on visited cells

**Acceptance Criteria:**
- Robot builds map while navigating (slam_toolbox)
- `ros2 action send_goal /navigate_to_pose` moves robot to goal
- Robot avoids obstacles within 500ms of detection (NFR1)
- Path replans when blocked (FR15, FR16)
- Coverage percentage published to `/g1/inspection/coverage`

**Technical Notes:**
- Nav2 params in `config/nav2_params.yaml`
- SLAM params in `config/slam_params.yaml`
- Bridge handles velocity limits (0.5 m/s linear, 1.0 rad/s angular)
- cmd_vel timeout for safety stop

**Prerequisites:** Story 2

**🧪 Runnable Verification:**
```bash
# Terminal 1: Launch simulation with navigation
ros2 launch g1_bringup sim_nav_launch.py

# Terminal 2: Verify SLAM is building map
ros2 topic echo /map --once
# Should see occupancy grid data

# Terminal 3: Send navigation goal via CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}}"
# Robot should navigate to goal in RViz

# Terminal 4: Check coverage
ros2 topic echo /g1/inspection/coverage --once
# Should see coverage percentage

# Manual test: Place obstacle in path, verify replanning
```

**Definition of Done:** Robot autonomously navigates to goal, builds map, avoids obstacles, reports coverage.

---

## Story 4: Localization & Safety Systems

As an operator,
I want the robot to monitor its safety state and stop when needed,
So that operation is safe and predictable.

**Scope:**
- Implement `SafetyNode`:
  - E-stop service `/g1/emergency_stop` - immediate halt <500ms
  - Collision proximity monitoring from costmap
  - Fall detection from IMU
  - Publishes `/g1/safety/status`
- Implement `BatteryMonitor`:
  - Read battery from SDK (simulated in sim)
  - Warning at 20%, RETURNING_HOME at 10%
  - Battery level in status messages
- Implement localization confidence monitoring:
  - Monitor slam_toolbox covariance/match quality
  - Warn on low confidence, stop on failure
  - Publish `/g1/localization/confidence`
- Graceful degradation - nodes publish errors, don't crash

**Acceptance Criteria:**
- E-stop stops robot within 500ms (NFR11)
- Low battery triggers return behavior (NFR14)
- Localization failure stops robot and notifies (FR21, FR22)
- Safety status visible in RViz and CLI

**Technical Notes:**
- Safety node runs at high priority
- Can override navigation commands
- All thresholds configurable in `config/safety_params.yaml`

**Prerequisites:** Story 3

**🧪 Runnable Verification:**
```bash
# Terminal 1: Launch full system
ros2 launch g1_bringup sim_nav_launch.py

# Terminal 2: Start robot navigating
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"

# Terminal 3: Trigger E-stop while moving
ros2 service call /g1/emergency_stop std_srvs/srv/Trigger
# Robot should stop immediately

# Terminal 4: Check safety status
ros2 topic echo /g1/safety/status --once
# Should show EMERGENCY_STOP state

# Terminal 5: Simulate low battery (set param or publish test message)
ros2 param set /g1/battery_monitor simulated_battery_level 8.0
# Robot should start returning home

# Check localization confidence
ros2 topic echo /g1/localization/confidence --once
```

**Definition of Done:** E-stop works instantly, battery triggers return, localization monitoring active, all observable via topics.

---

## Story 5: Plan Management & Calibration

As an operator,
I want to upload construction plans and calibrate the robot's starting position,
So that the robot knows the expected layout and where it is.

**Scope:**
- Implement `PlanManager`:
  - Upload PDF/PNG plans via CLI
  - Store in `/data/plans/` with UUID
  - Basic parsing (room boundaries, scale if available)
  - Prepare plan images for VLM context
- Implement calibration flow:
  - Operator specifies starting position on plan
  - Set initial pose for slam_toolbox via `/initialpose`
  - Store plan-to-robot coordinate transform
  - Verify localization succeeds from position
- Plan metadata includes trade type (finishes for MVP)

**Acceptance Criteria:**
- `g1-inspect upload --plan floor.pdf` stores plan, returns ID
- `g1-inspect calibrate --plan <id> --position "x,y,theta"` initializes localization
- Plan persists across restarts (NFR21)
- Calibration failure provides clear error message

**Technical Notes:**
- PDF conversion via pdf2image
- Plan parsing is basic - VLM does heavy lifting during detection
- Coordinate transform stored for capture-to-plan correlation

**Prerequisites:** Story 4

**🧪 Runnable Verification:**
```bash
# Terminal 1: Launch system
ros2 launch g1_bringup sim_nav_launch.py

# Terminal 2: Upload a test plan
./scripts/g1_inspect.py upload --plan test_data/sample_floor_plan.png
# Should output: "Plan uploaded: <uuid>"

# List plans
./scripts/g1_inspect.py plans list
# Should show uploaded plan

# Calibrate robot position
./scripts/g1_inspect.py calibrate --plan <uuid> --position "0,0,0"
# Should output: "Calibration successful. Robot localized."

# Verify initial pose was set (check in RViz - robot should be at origin)
ros2 topic echo /initialpose --once
```

**Definition of Done:** Can upload plan via CLI, calibrate robot, see robot positioned correctly in RViz.

---

## Story 6: Inspection State Machine & CLI

As an operator,
I want to control inspections via CLI and see real-time status,
So that I can manage the robot easily.

**Scope:**
- Implement `InspectionStateMachine`:
  - States: IDLE, CALIBRATING, INSPECTING, PAUSED, BLOCKED, WAITING_OPERATOR, COMPLETE, EMERGENCY_STOP, RETURNING_HOME
  - Transitions per Architecture state diagram
  - Publishes `/g1/inspection/status` (state, battery, location, completion %)
- Implement CLI (`scripts/g1_inspect.py`):
  - `upload`, `calibrate`, `start`, `pause`, `resume`, `stop`, `estop`, `status`, `viz`
  - Clear feedback for all commands
  - `--help` documentation
- Implement notification system:
  - Publish to `/g1/notifications` on key events
  - Route blocked, localization failed, complete, low battery
- RViz config showing status, map, path, camera feed

**Acceptance Criteria:**
- All CLI commands work and provide feedback
- State transitions match Architecture diagram
- Status shows: state, battery %, location, completion %, elapsed time
- Notifications appear in CLI output and RViz
- `g1-inspect viz` opens configured RViz

**Technical Notes:**
- CLI wraps ROS2 service calls (Click or argparse)
- State machine can be simple Python, doesn't need library
- Log all state transitions with structured format

**Prerequisites:** Story 5

**🧪 Runnable Verification:**
```bash
# Terminal 1: Launch system
ros2 launch g1_bringup sim_nav_launch.py

# Terminal 2: Full CLI workflow
./scripts/g1_inspect.py status
# Should show: IDLE

./scripts/g1_inspect.py upload --plan test_data/sample_floor_plan.png
# Note the plan ID

./scripts/g1_inspect.py calibrate --plan <id> --position "0,0,0"
# Should show: Calibration successful

./scripts/g1_inspect.py start --plan <id>
# Should show: Inspection started, state: INSPECTING

./scripts/g1_inspect.py status
# Should show: INSPECTING, battery %, location, completion %

./scripts/g1_inspect.py pause
# Should show: Inspection paused

./scripts/g1_inspect.py status
# Should show: PAUSED

./scripts/g1_inspect.py resume
# Should show: Inspection resumed

./scripts/g1_inspect.py stop
# Should show: Inspection stopped, state: IDLE

# Test E-stop
./scripts/g1_inspect.py start --plan <id>
./scripts/g1_inspect.py estop
# Should immediately stop

# Open RViz
./scripts/g1_inspect.py viz
```

**Definition of Done:** Complete CLI workflow works - upload, calibrate, start, pause, resume, stop. Status updates in real-time.

---

## Story 7: Visual Capture Pipeline

As the robot,
I want to capture and store geotagged images during inspection,
So that visual data is available for defect detection.

**Scope:**
- Implement `ImageCapture` node:
  - Capture RGB at 1fps during INSPECTING state
  - Capture aligned depth alongside RGB
  - Tag each image with: timestamp, robot pose, camera orientation, unique ID
  - Save to ROS2 bag + image files
- LiDAR point cloud continuous capture to bag
- Implement `PlanCorrelator`:
  - Transform image pose to plan coordinates
  - Calculate camera FOV coverage area
  - Store correlation metadata with each image
- Track which plan areas have been photographed

**Acceptance Criteria:**
- Images captured at 1fps during inspection (NFR3)
- Each image has pose metadata
- Depth aligned and saved with RGB
- Can query "images covering plan area X"
- ROS2 bag contains all sensor data for replay

**Technical Notes:**
- Use realsense2_camera for D435i (already handles alignment)
- Use livox_ros2_driver for MID-360
- Resolution: 640x480 sufficient for MVP
- Correlation uses calibration transform from Story 5

**Prerequisites:** Story 6

**🧪 Runnable Verification:**
```bash
# Terminal 1: Launch system
ros2 launch g1_bringup sim_nav_launch.py

# Terminal 2: Start a short inspection
./scripts/g1_inspect.py upload --plan test_data/sample_floor_plan.png
./scripts/g1_inspect.py calibrate --plan <id> --position "0,0,0"
./scripts/g1_inspect.py start --plan <id>

# Let it run for 30 seconds, then stop
sleep 30
./scripts/g1_inspect.py stop

# Terminal 3: Check captured images
ls /data/inspections/<inspection_id>/images/
# Should see: img_001.jpg, img_002.jpg, etc.

# Check image metadata
cat /data/inspections/<inspection_id>/images/img_001.json
# Should see: timestamp, pose (x, y, theta), plan_coordinates

# Check ROS2 bag was recorded
ros2 bag info /data/inspections/<inspection_id>/rosbag/
# Should show topics: /g1/camera/rgb, /g1/camera/depth, /g1/lidar/points

# Replay bag and visualize
ros2 bag play /data/inspections/<inspection_id>/rosbag/
```

**Definition of Done:** Inspection captures images with pose metadata, viewable in filesystem and via bag replay.

---

## Story 8: VLM Defect Detection

As the system,
I want to analyze captured images via VLM API to detect defects,
So that issues are identified automatically.

**Scope:**
- Implement `DefectDetector` (runs on offload server):
  - Send images + plan context to VLM API (GPT-4V or Claude)
  - Structured prompt requesting: defect type, description, location in image, confidence
  - Parse structured JSON response
  - Support multiple VLM providers via config
- Detect two types:
  - Location errors: item in wrong position vs plan
  - Quality issues: scratches, damage, defects on finishes
- Implement defect localization:
  - Use image pose + depth to estimate world coordinates
  - Map to plan coordinates
  - Target: within 6 inches
- Generate annotated images with bounding boxes
- Assign confidence scores, filter low confidence

**Acceptance Criteria:**
- Images sent to VLM with plan context
- Defects returned with type, description, location, confidence (FR28-33)
- Defects localized to plan coordinates (FR31)
- Annotated images generated for evidence (FR32)
- API errors handled gracefully (retry, log)

**Technical Notes:**
- VLM prompt template per Architecture spec
- API key via environment variable
- Run on server, not Jetson (compute offload)
- Batch images for efficiency

**Prerequisites:** Story 7

**🧪 Runnable Verification:**
```bash
# Test with sample images (no full inspection needed)
# Terminal 1: Run defect detection on test images
export OPENAI_API_KEY="your-key-here"  # or ANTHROPIC_API_KEY

./scripts/detect_defects.py \
  --images test_data/sample_inspection_images/ \
  --plan test_data/sample_floor_plan.png \
  --output /tmp/defect_results/

# Check results
cat /tmp/defect_results/defects.json
# Should see:
# [
#   {
#     "id": "def_001",
#     "type": "quality_issue",
#     "description": "Scratch on tile surface",
#     "image": "img_003.jpg",
#     "location_in_image": {"x": 320, "y": 240},
#     "plan_coordinates": {"x": 2.5, "y": 1.2},
#     "confidence": 0.85
#   },
#   ...
# ]

# Check annotated images
ls /tmp/defect_results/annotated/
# Should see: img_003_annotated.jpg with bounding box

# Test with actual inspection data
./scripts/g1_inspect.py detect --inspection <inspection_id>
# Should process all captured images and output defects
```

**Definition of Done:** VLM analyzes images, returns structured defect JSON with locations and confidence, annotated images generated.

---

## Story 9: Report Generation

As an operator,
I want PDF inspection reports with photos and punch lists,
So that I can share findings with crews and stakeholders.

**Scope:**
- Implement `ReportGenerator` (runs on offload server):
  - Generate PDF with: summary, issue list, plan overlay, photos, punch list
  - Plan overlay: markers at defect locations, color-coded by type, numbered
  - Photo integration: thumbnails inline, annotated images
  - Punch list: sortable table with location, type, description, photo ref, status
  - Also export punch list as CSV
- Report saved to `/data/reports/<inspection_id>/`
- Report metadata JSON alongside PDF
- CLI commands: `g1-inspect reports list`, `g1-inspect reports get <id>`

**Acceptance Criteria:**
- Report generated within 30 minutes of inspection complete (NFR5)
- PDF includes all required sections (FR34-40)
- Plan overlay shows defect locations accurately (FR36)
- Punch list categorized by type (FR38)
- Reports accessible via CLI and filesystem
- Historical reports retained (NFR22)

**Technical Notes:**
- Use ReportLab for PDF generation
- Compress images to keep PDF <50MB
- Template-based for consistency
- Naming: `inspection_<date>_<plan_name>.pdf`

**Prerequisites:** Story 8

**🧪 Runnable Verification:**
```bash
# Generate report from detection results
./scripts/g1_inspect.py report --inspection <inspection_id>
# Should output: "Report generated: /data/reports/<inspection_id>/inspection_2025-12-04_floor_plan.pdf"

# Or generate from test defect data
./scripts/generate_report.py \
  --defects test_data/sample_defects.json \
  --plan test_data/sample_floor_plan.png \
  --output /tmp/test_report.pdf

# View the PDF
xdg-open /tmp/test_report.pdf
# Or: open /tmp/test_report.pdf (macOS)

# Verify PDF contents:
# - Cover page with inspection summary
# - Plan overlay with numbered markers
# - Defect list with photos
# - Punch list table

# Check CSV export
cat /data/reports/<inspection_id>/punch_list.csv
# Should be importable to spreadsheet

# List reports via CLI
./scripts/g1_inspect.py reports list
# Should show available reports

./scripts/g1_inspect.py reports get <id>
# Should copy report to current directory
```

**Definition of Done:** PDF report generated with all sections, viewable and shareable. Punch list exportable as CSV.

---

## Story 10: Integration Testing & Hardening

As a developer,
I want end-to-end testing and edge case handling,
So that the system works reliably in real conditions.

**Scope:**
- Create test scenarios:
  - Happy path: full inspection cycle in simulation
  - Obstacle blocking: path replanning and notification
  - Localization degradation: warning and stop behavior
  - Low battery: return home behavior
  - E-stop: immediate halt
  - WiFi disconnect: pause and resume on reconnect
- Add error handling throughout:
  - Sensor failures: log and notify
  - API failures: retry with backoff
  - Unexpected states: safe fallback
- Performance validation:
  - 500ms obstacle response (NFR1)
  - 10Hz localization (NFR2)
  - 95% route completion (NFR6)
- Add structured logging throughout (per Architecture conventions)

**Acceptance Criteria:**
- All test scenarios pass in simulation
- Error conditions handled gracefully (no crashes)
- Performance metrics meet NFRs
- Logs are structured and filterable by context tag

**Technical Notes:**
- pytest for unit tests, launch_testing for integration
- Test with simulated sensor noise and failures
- Document known limitations

**Prerequisites:** Story 9

**🧪 Runnable Verification:**
```bash
# Run unit tests
pytest src/g1_*/test/ -v
# All tests should pass

# Run integration test suite
./scripts/run_integration_tests.sh
# Runs all scenarios in simulation

# Manual end-to-end test
ros2 launch g1_bringup sim_nav_launch.py &

# Full inspection cycle
./scripts/g1_inspect.py upload --plan test_data/sample_floor_plan.png
./scripts/g1_inspect.py calibrate --plan <id> --position "0,0,0"
./scripts/g1_inspect.py start --plan <id>

# Watch status until complete
watch -n 2 './scripts/g1_inspect.py status'
# Wait for COMPLETE state

# Generate report
./scripts/g1_inspect.py report --inspection <id>

# View report
xdg-open /data/reports/<id>/*.pdf

# Test edge cases manually:
# 1. Start inspection, then trigger e-stop
# 2. Start inspection, simulate low battery
# 3. Start inspection, block path (in sim), verify notification
# 4. Start inspection, kill VLM API (timeout handling)

# Check logs are structured
grep "\[NAVIGATION\]" /var/log/g1_inspector/latest.log
grep "\[SAFETY\]" /var/log/g1_inspector/latest.log
```

**Definition of Done:** Full inspection cycle works end-to-end in simulation. All edge cases handled gracefully. Test suite passes.

---

## Story 11: Docker Deployment

As a developer,
I want Docker images for Jetson and server deployment,
So that I can deploy reproducibly to robot hardware.

**Scope:**
- Create `Dockerfile.jetson`:
  - Base: NVIDIA L4T + ROS2 Humble
  - All project packages built
  - External dependencies installed
  - Entry point for robot_launch.py
- Create `Dockerfile.server`:
  - Offload server for VLM + report generation
  - Python environment with API clients
- Create `docker-compose.yaml`:
  - Orchestrate robot + server containers
  - Network configuration
  - Volume mounts for data and config
- Create `.env.example` documenting all required variables
- Create `robot_launch.py` for real hardware (vs sim_launch.py)

**Acceptance Criteria:**
- `docker build` succeeds for both images
- `docker-compose up` starts full system
- Robot connects to real Unitree G1 hardware
- Offload server accessible from robot
- Config/data persisted via volumes

**Technical Notes:**
- Base image: dusty-nv/jetson-containers
- GPU access configured for Jetson
- Network: robot at 192.168.123.164, configurable via .env

**Prerequisites:** Story 10

**🧪 Runnable Verification:**
```bash
# Build images (on development machine)
docker build -f docker/Dockerfile.jetson -t g1-inspector:jetson .
docker build -f docker/Dockerfile.server -t g1-inspector:server .

# Test locally with docker-compose (simulation mode)
cp .env.example .env
# Edit .env with API keys

docker-compose -f docker/docker-compose.dev.yaml up
# Should start simulation environment in containers

# From host, verify system is running
curl http://localhost:8080/health  # Server health check
ros2 topic list  # Should see /g1/* topics

# Run CLI against containerized system
./scripts/g1_inspect.py status
# Should show IDLE

# On real Jetson hardware:
# 1. Copy images to Jetson
# 2. Connect to robot network (192.168.123.x)
# 3. Run docker-compose

docker-compose -f docker/docker-compose.robot.yaml up
# Should connect to real G1 robot

# Verify real robot connection
./scripts/g1_inspect.py status
# Should show real battery level from robot

# Run real inspection
./scripts/g1_inspect.py upload --plan real_site_plan.pdf
./scripts/g1_inspect.py calibrate --plan <id> --position "0,0,0"
./scripts/g1_inspect.py start --plan <id>
# Robot should physically move and inspect
```

**Definition of Done:** Docker images build, compose starts system, real robot connection verified.

---

## FR Coverage Matrix

| FR | Requirement | Story |
|----|-------------|-------|
| FR1-2 | Upload PDF/PNG plans | 5 |
| FR3-4 | Parse plans, specify trade | 5 |
| FR5-6 | Starting position, calibration | 5 |
| FR7-10 | Start/pause/resume/abort inspection | 6 |
| FR11 | Real-time status | 6 |
| FR12 | Autonomous indoor navigation | 3 |
| FR13 | Traverse stairs | ⬜ Deferred |
| FR14-17 | Obstacle handling | 3 |
| FR18 | Route completion tracking | 3 |
| FR19-20 | Self-localization | 3, 4 |
| FR21-22 | Localization confidence/failure | 4 |
| FR23-25 | Capture RGB/depth/LiDAR | 7 |
| FR26-27 | Interior representation, plan correlation | 7 |
| FR28-29 | Plan comparison, location errors | 8 |
| FR30 | Quality issue detection | 8 |
| FR31-32 | Defect localization, photo evidence | 8 |
| FR33 | Confidence scores | 8 |
| FR34-40 | Report generation | 9 |
| FR41-44 | Operator notifications | 4, 6 |
| FR45-47 | Blue tape placement | ⬜ Deferred Phase 2 |

**Coverage:** 43/47 FRs (4 intentionally deferred: FR13 stairs, FR45-47 blue tape)

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Epics** | 1 |
| **Total Stories** | 11 |
| **FRs Covered** | 43/43 MVP (100%) |
| **Deferred** | FR13 (stairs), FR45-47 (blue tape) |

### Story Sequence

| Story | Deliverable | Can Test By Running |
|-------|-------------|---------------------|
| 1 | Workspace | `colcon build` |
| 2 | Simulation | `ros2 launch sim_launch.py` + teleop |
| 3 | Navigation | Send nav goals, watch robot navigate |
| 4 | Safety | Trigger E-stop, low battery |
| 5 | Plans | CLI upload and calibrate |
| 6 | State Machine | Full CLI workflow |
| 7 | Capture | See saved images with metadata |
| 8 | Detection | Run VLM, see defect JSON |
| 9 | Reports | Generate and view PDF |
| 10 | Integration | Full end-to-end cycle |
| 11 | Deployment | `docker-compose up` on hardware |

### Key Insight

This is **integration work**. We're not building:
- A locomotion controller (Unitree SDK has it)
- A SLAM system (slam_toolbox is proven)
- A path planner (Nav2 is industry standard)
- A vision model (VLM APIs are ready)
- A PDF library (ReportLab works)

We're building the **glue** that connects these pieces into an inspection workflow.

Each story ends with something you can **run and verify** - no story is just "code written" without proof it works.

---

_Generated by BMAD Epic and Story Creation Workflow_
_Date: 2025-12-04_
