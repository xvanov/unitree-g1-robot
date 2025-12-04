---
stepsCompleted: [1, 2, 3, 4, 5, 6, 7]
inputDocuments:
  - docs/prd.md
  - docs/analysis/product-brief-unitree-g1-robot-2025-12-03.md
  - docs/research/implementation-starting-points-2025-12-03.md
workflowType: 'architecture'
lastStep: 7
project_name: 'unitree-g1-robot'
user_name: 'BMAD'
date: '2025-12-04'
---

# Architecture Decision Document

_This document builds collaboratively through step-by-step discovery. Sections are appended as we work through each architectural decision together._

## Project Context Analysis

### Requirements Overview

**Functional Requirements:**
44 functional requirements spanning 9 categories: Plan Management, Robot Deployment & Control, Autonomous Navigation, Localization, Visual Capture & Mapping, Defect Detection, Inspection Reporting, Operator Notifications, and Physical Marking (deferred).

Core inspection loop: Plan ingestion → Autonomous navigation → Visual capture → Defect detection → Report generation

**Non-Functional Requirements:**
- Performance: 500ms obstacle response, 10Hz localization, 1fps image capture
- Reliability: ≥95% route completion, ≥95% navigation success, graceful degradation
- Safety: E-stop <500ms, collision avoidance ≥0.5m, fall prevention, battery management
- Hardware: 2hr battery window, Jetson Orin compute budget, WiFi connectivity required

**Scale & Complexity:**
- Primary domain: IoT/Embedded Robotics with AI/ML
- Complexity level: HIGH
- Estimated architectural components: 12-15 major subsystems

### Technical Constraints & Dependencies

| Constraint | Architectural Impact |
|------------|---------------------|
| Unitree G1 EDU 23-DOF | Fixed hardware platform, no dexterous manipulation |
| Jetson Orin NX 16GB | Onboard compute ceiling, must offload heavy inference |
| unitree_sdk2 + ROS2 | Locked middleware stack |
| WiFi always-online | Hybrid compute architecture, connectivity dependency |
| 2hr battery | Route planning must budget power, staged inspections |
| SDK commands real-only | Locomotion abstraction required for simulation |

### Cross-Cutting Concerns Identified

1. **Locomotion Abstraction Layer** - Different implementations for simulation (fake/RL) vs real robot (SDK)
2. **Safety Subsystem** - E-stop, collision avoidance, fall prevention, battery monitoring
3. **State Machine** - Robot operational states and transitions
4. **Telemetry Pipeline** - Real-time status for operator supervision
5. **Coordinate Frame Management** - LiDAR ↔ robot ↔ plan ↔ world transformations
6. **Data Pipeline** - Sensor capture → storage → processing → reporting flow

### Key Architectural Decision: Locomotion Abstraction

Per user input and research findings, locomotion will be abstracted:
- **Simulation**: "Fake locomotion" (teleport base) OR RL policy wrapper
- **Real Robot**: SDK LocoClient high-level commands
- **Benefit**: Focus simulation effort on navigation, SLAM, perception, defect detection
- **Trade-off**: No locomotion testing in sim, but SDK walking is proven on flat floors

## Foundation Stack Evaluation

### Primary Technology Domain

IoT/Embedded Robotics with AI/ML - Fixed hardware platform (Unitree G1 EDU 23-DOF) with constrained technology choices.

### Foundation Options Evaluated

| Option | Repository | Purpose | Selection |
|--------|-----------|---------|-----------|
| unitree_sdk2_python | [GitHub](https://github.com/unitreerobotics/unitree_sdk2_python) | Direct robot control, high-level commands | ✅ Required |
| unitree_ros2 | [GitHub](https://github.com/unitreerobotics/unitree_ros2) | ROS2 integration, sensor topics | ✅ Required |
| Nav2 + slam_toolbox | [Nav2 Docs](https://docs.nav2.org/) | Navigation & SLAM | ✅ Required |
| unitree_rl_gym | [GitHub](https://github.com/unitreerobotics/unitree_rl_gym) | RL locomotion policies | ⬜ Optional (sim alternative) |

### Selected Foundation Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                    DUAL-PATH ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  HIGH-LEVEL LOCOMOTION (unitree_sdk2_python):                  │
│  ├── LocoClient.Move(vx, vy, vyaw)                             │
│  ├── LocoClient.SetVelocity(vx, vy, omega, duration)           │
│  ├── LocoClient.StandUp() / Sit()                              │
│  └── Direct to robot, proven walking controller                │
│                                                                 │
│  ROS2 LAYER (unitree_ros2 + Nav2):                             │
│  ├── Sensor topics (LiDAR, camera, IMU)                        │
│  ├── Nav2 path planning → /cmd_vel velocity commands           │
│  ├── slam_toolbox localization                                 │
│  └── Custom application nodes                                  │
│                                                                 │
│  BRIDGE (Nav2 → SDK):                                          │
│  └── /cmd_vel subscriber → LocoClient.SetVelocity()            │
│      Nav2 decides WHERE to go, SDK handles HOW to walk         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Key Insight:** ROS2/Nav2 and unitree_sdk2 are complementary, not mutually exclusive:
- **Nav2** = "where to go" (path planning, obstacle avoidance)
- **LocoClient** = "how to walk there" (Unitree's proven locomotion controller)
- **Bridge node** connects them: subscribes to `/cmd_vel`, calls `SetVelocity()`

### DDS Topic Access: ROS2 vs SDK

**Important:** The robot runs Unitree firmware, NOT ROS2. It publishes DDS topics directly:
- `rt/lowstate` - IMU, joint states, battery (SDK access via `ChannelSubscriber`)
- `rt/utlidar/cloud` - LiDAR point cloud (DDS, ROS2-compatible message type)

**Why ROS2 can see robot topics:** Both ROS2 Humble and Unitree SDK use CycloneDDS. When on the same network/DDS domain, ROS2 nodes can discover and subscribe to the robot's DDS topics directly.

**Sensor Access Strategy (Decision 2025-12-04):**
| Sensor | Access Method | Rationale |
|--------|---------------|-----------|
| IMU/Joints | SDK → `hardware_bridge.py` → ROS2 | Custom message conversion needed |
| LiDAR | Direct DDS (Option A) | Standard PointCloud2, try zero-code first |
| LiDAR | SDK bridge (Option B, fallback) | If Option A has QoS/discovery issues |

See Story 1.3 Dev Notes for implementation details.

### Architectural Decisions from Foundation

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Language | Python 3.10 | unitree_sdk2 hard requirement |
| DDS Middleware | CycloneDDS 0.10.x | Unitree + Nav2 compatibility |
| Path Planning | Nav2 Behavior Trees | ROS2 industry standard |
| 2D SLAM | slam_toolbox | Proven lifelong mapping |
| Sensor Fusion | robot_localization | Multi-sensor EKF fusion |
| Locomotion | SDK LocoClient | Use Unitree's built-in, not custom RL |

### Network Configuration

**Design Decision:** Network addresses are **configurable, not hardcoded**.

| Component | Default Address | Configuration |
|-----------|-----------------|---------------|
| G1 Robot (Ethernet) | 192.168.123.164 | Fixed by Unitree hardware |
| LiDAR | 192.168.123.120 | Fixed by Unitree hardware |
| Development Computer | 192.168.123.x | Configurable (x ≠ 164, 120) |
| WiFi Deployment | Dynamic | Discovered via config/environment |

**Implementation:** All network addresses loaded from configuration file or environment variables. No hardcoded IPs in application code.

### Simulation Strategy

| Environment | Locomotion | Navigation/Perception |
|-------------|------------|----------------------|
| **MuJoCo Sim** | Fake (teleport base) | Full Nav2 + slam_toolbox |
| **Real Robot** | SDK LocoClient | Full Nav2 + slam_toolbox |

**Rationale:** Same navigation/perception code runs in both environments. Only the locomotion implementation differs, abstracted behind a common interface.

### Key Integration: Nav2 to LocoClient Bridge

```python
# Conceptual bridge implementation
class Nav2LocoClientBridge(Node):
    def __init__(self):
        self.loco_client = LocoClient()
        self.loco_client.Init()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        self.loco_client.SetVelocity(
            vx=msg.linear.x,
            vy=msg.linear.y,
            omega=msg.angular.z,
            duration=0.1  # Continuous updates from Nav2
        )
```

This bridge is the critical integration point - Nav2's sophisticated planning drives Unitree's robust walking.

## Core Architectural Decisions

### Decision Summary

| Category | Decision | Choice | Rationale |
|----------|----------|--------|-----------|
| **Simulation** | Framework | MuJoCo | Fast, G1 models exist, Python bindings |
| | Locomotion | Teleport (fake) | Simple for MVP, focus on nav/perception |
| **Dependencies** | External libs | Clone to `external/`, gitignore | Reproducible, simple setup |
| | Setup | Single `scripts/setup.sh` | One command for new machine |
| **Perception** | Pipeline | ROS2-native (all topics) | Standard tooling, RViz visualization |
| | Image Capture | Continuous 1fps | Complete coverage for inspection |
| | Point Cloud | 2D SLAM + images | slam_toolbox for nav, RGB for defects |
| **State Mgmt** | State Machine | IDLE→CALIBRATING→INSPECTING→COMPLETE | Clear operational states |
| | Implementation | Hybrid (Nav2 BT + custom Python) | Nav2 for nav behaviors, Python for high-level |
| **Data Storage** | Format | ROS2 bags for all data | Native tooling, replay capability |
| | Compute Split | Onboard: nav/capture, Offloaded: ML/reports | Jetson for real-time, server for heavy ML |
| **Defect Detection** | Model | VLM API (GPT-4V/Claude) | No training needed, fast MVP iteration |
| | Location Errors | Object detection + localization | Automated comparison against plan |
| **Deployment** | To Robot | Docker containers | Reproducible, NVIDIA Jetson support |
| | Config | YAML + .env | ROS2 native params + secrets separation |

### Simulation Architecture

**Framework:** MuJoCo
- Fast simulation, good Python bindings
- Unitree G1 models available via `unitree_mujoco`
- Sufficient fidelity for navigation/perception testing

**Fake Locomotion Implementation:**
```python
class SimLocomotionController:
    """Teleport-based locomotion for simulation"""

    def set_velocity(self, vx, vy, omega, dt):
        # Integrate velocity to get new pose
        self.x += vx * dt
        self.y += vy * dt
        self.theta += omega * dt
        # Directly set robot base position in MuJoCo
        self.sim.set_base_pose(self.x, self.y, self.theta)
```

### Perception Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                   PERCEPTION PIPELINE                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Sensors (ROS2 Topics):                                         │
│  ┌─────────┐    ┌─────────────┐                                │
│  │ D435i   │───▶│ /camera/rgb │───┐                            │
│  │         │───▶│ /camera/depth│   │                            │
│  └─────────┘    └─────────────┘   │                            │
│  ┌─────────┐    ┌─────────────┐   │    ┌─────────────┐         │
│  │MID-360  │───▶│ /lidar/points│───┼───▶│  ROS2 Bag   │         │
│  └─────────┘    └─────────────┘   │    │  Recording  │         │
│  ┌─────────┐    ┌─────────────┐   │    └─────────────┘         │
│  │  IMU    │───▶│ /imu/data   │───┘                            │
│  └─────────┘    └─────────────┘                                │
│                                                                 │
│  Processing:                                                    │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │ /lidar/points│───▶│ slam_toolbox│───▶│ /map, /tf   │         │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │ /lidar/points│───▶│ Nav2 costmap│───▶│ Obstacles   │         │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Image Capture:** Continuous at 1fps throughout inspection route
- All images tagged with robot pose from SLAM
- Stored in ROS2 bags for replay and offline processing

### Robot State Machine

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROBOT STATE MACHINE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                      ┌──────────┐                               │
│                      │   IDLE   │◀─────────────────┐            │
│                      └────┬─────┘                  │            │
│                           │ start_inspection       │            │
│                      ┌────▼─────┐                  │            │
│                      │CALIBRATE │──────────────────┤            │
│                      └────┬─────┘     fail         │            │
│                           │ success                │            │
│                      ┌────▼─────┐                  │            │
│              ┌──────▶│INSPECTING│◀──────┐         │            │
│              │       └────┬─────┘       │         │            │
│              │            │             │         │            │
│         resume       ┌────▼─────┐   route_clear   │            │
│              │       │ BLOCKED  │───────┘         │            │
│              │       └────┬─────┘                 │            │
│              │            │ no_path               │            │
│              │       ┌────▼─────┐                 │ abort      │
│              └───────│ WAITING  │─────────────────┤            │
│                      │ OPERATOR │                 │            │
│                      └──────────┘                 │            │
│                                                   │            │
│                      ┌──────────┐                 │            │
│                      │ COMPLETE │─────────────────┘            │
│                      └──────────┘                              │
│                                                                 │
│  ANY STATE ──e_stop──▶ EMERGENCY_STOP                          │
│  ANY STATE ──low_battery──▶ RETURNING_HOME                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Implementation:** Hybrid approach
- **Nav2 Behavior Trees:** Navigation behaviors (go to waypoint, avoid obstacle)
- **Custom Python:** High-level inspection state management

### Compute Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    COMPUTE SPLIT                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ONBOARD (Jetson Orin NX 16GB):                                │
│  ├── Sensor capture and publishing                             │
│  ├── SLAM (slam_toolbox)                                       │
│  ├── Navigation (Nav2)                                         │
│  ├── Obstacle avoidance (real-time safety)                     │
│  ├── State machine execution                                   │
│  ├── ROS2 bag recording                                        │
│  └── Nav2 → LocoClient bridge                                  │
│                                                                 │
│  OFFLOADED (Server via WiFi):                                  │
│  ├── Plan parsing (PDF/PNG → spatial data)                     │
│  ├── Defect detection (VLM API calls)                          │
│  ├── Object detection + plan correlation                       │
│  ├── Report generation (PDF output)                            │
│  └── Data archival and analysis                                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Defect Detection Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                 DEFECT DETECTION PIPELINE                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Onboard (Jetson):                                              │
│  ┌─────────┐    ┌─────────┐    ┌─────────────┐                 │
│  │ Camera  │───▶│ Capture │───▶│ ROS2 Bag    │                 │
│  │ D435i   │    │  1fps   │    │ + pose tag  │                 │
│  └─────────┘    └─────────┘    └──────┬──────┘                 │
│                                       │                         │
│  ─────────────────────────────────────┼───────────────────────  │
│                                       │ WiFi                    │
│  Offloaded (Server):                  ▼                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │ Construction│    │   VLM API   │    │   Defect    │         │
│  │    Plan     │───▶│ (GPT-4V /   │───▶│   Report    │         │
│  │   Context   │    │  Claude)    │    │             │         │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│                                                                 │
│  Detection Types:                                               │
│  1. Quality Issues: VLM prompt for scratches, damage, defects  │
│  2. Location Errors: Object detection + pose → plan comparison │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**VLM Prompt Strategy (MVP):**
```
Given this construction plan section showing expected layout:
[plan_image]

And this photo taken at position (x, y) facing direction θ:
[captured_image]

Identify:
1. Any items installed in wrong locations vs the plan
2. Any visible defects (scratches, damage, quality issues)
3. Confidence level for each finding

Format response as structured JSON.
```

### Deployment Architecture

**Docker on Jetson Orin NX:**
- Base: NVIDIA L4T + ROS2 Humble ([jetson-containers](https://github.com/dusty-nv/jetson-containers))
- Tested: Jetson Orin NX 16GB with JetPack 6.x

**Container Structure:**
```
g1_inspector/
├── Dockerfile.jetson      # Orin deployment image
├── Dockerfile.server      # Offload server image
├── docker-compose.yaml    # Multi-container orchestration
└── .env.example           # Environment template
```

**Configuration Management:**
- `config/*.yaml` - ROS2 parameter files (navigation, SLAM, sensors)
- `.env` - Network addresses, API keys, secrets
- `launch/*.py` - ROS2 launch files referencing YAML configs

### External Dependencies

**Setup Script:** `scripts/setup.sh` clones all dependencies into `external/`:

```bash
external/
├── cyclonedds/           # DDS middleware (pinned commit)
├── unitree_sdk2/         # C++ SDK
├── unitree_sdk2_python/  # Python SDK
├── unitree_ros2/         # ROS2 integration
├── unitree_mujoco/       # MuJoCo simulation
└── .gitkeep
```

All paths in `external/` are gitignored. Setup script pins specific commits for reproducibility.

## Implementation Patterns & Consistency Rules

### Purpose

These patterns ensure AI agents and human developers write consistent, compatible code. All contributors MUST follow these conventions.

### ROS2 Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| **Node names** | snake_case | `nav2_loco_bridge`, `defect_detector` |
| **Topic names** | /namespace/topic (snake_case) | `/g1/cmd_vel`, `/g1/camera/rgb` |
| **Service names** | /namespace/verb_noun | `/g1/start_inspection` |
| **Action names** | /namespace/VerbNoun | `/g1/NavigateToWaypoint` |
| **Parameter names** | snake_case | `max_velocity`, `detection_threshold` |
| **Frame IDs** | snake_case | `base_link`, `camera_link`, `map`, `odom` |

**Namespace Convention:** All project nodes use `/g1/` namespace prefix.

### Python Code Style

| Element | Convention | Example |
|---------|------------|---------|
| **Files/modules** | snake_case | `loco_bridge.py`, `defect_detector.py` |
| **Classes** | PascalCase | `LocoBridge`, `InspectionStateMachine` |
| **Functions** | snake_case | `get_robot_pose()`, `detect_defects()` |
| **Variables** | snake_case | `current_waypoint`, `detection_result` |
| **Constants** | UPPER_SNAKE | `MAX_VELOCITY`, `DEFAULT_TIMEOUT` |
| **Type hints** | Required for public functions | `def get_pose() -> Pose:` |
| **Docstrings** | Google style | See example below |

**Docstring Example:**
```python
def detect_defects(image: np.ndarray, plan_context: dict) -> list[Defect]:
    """Detect defects in captured image using VLM API.

    Args:
        image: RGB image as numpy array (H, W, 3)
        plan_context: Dict containing plan section and expected items

    Returns:
        List of detected Defect objects with locations and confidence

    Raises:
        APIError: If VLM API call fails
    """
```

### Project Structure (Multi-Package)

```
unitree-g1-robot/
├── src/
│   ├── g1_bringup/              # Launch files, system config
│   │   ├── launch/
│   │   │   ├── robot_launch.py
│   │   │   ├── sim_launch.py
│   │   │   └── inspection_launch.py
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   ├── slam_params.yaml
│   │   │   └── robot_params.yaml
│   │   └── package.xml
│   │
│   ├── g1_navigation/           # Nav2 integration, loco bridge
│   │   ├── g1_navigation/
│   │   │   ├── __init__.py
│   │   │   ├── loco_bridge.py
│   │   │   └── waypoint_manager.py
│   │   ├── test/
│   │   └── package.xml
│   │
│   ├── g1_perception/           # Sensors, capture, SLAM interface
│   │   ├── g1_perception/
│   │   │   ├── __init__.py
│   │   │   ├── camera_node.py
│   │   │   └── image_capture.py
│   │   ├── test/
│   │   └── package.xml
│   │
│   ├── g1_inspection/           # State machine, defect detection, reporting
│   │   ├── g1_inspection/
│   │   │   ├── __init__.py
│   │   │   ├── state_machine.py
│   │   │   ├── defect_detector.py
│   │   │   └── report_generator.py
│   │   ├── test/
│   │   └── package.xml
│   │
│   ├── g1_safety/               # E-stop, collision, battery monitoring
│   │   ├── g1_safety/
│   │   │   ├── __init__.py
│   │   │   ├── safety_node.py
│   │   │   └── battery_monitor.py
│   │   ├── test/
│   │   └── package.xml
│   │
│   └── g1_interfaces/           # Custom messages, services, actions
│       ├── msg/
│       │   ├── InspectionStatus.msg
│       │   └── DefectReport.msg
│       ├── srv/
│       │   ├── StartInspection.srv
│       │   └── GetInspectionStatus.srv
│       ├── action/
│       │   └── ExecuteInspection.action
│       └── package.xml
│
├── external/                    # Gitignored, populated by setup.sh
├── scripts/
│   ├── setup.sh                 # One-command environment setup
│   └── deploy.sh                # Docker deployment script
├── docker/
│   ├── Dockerfile.jetson
│   ├── Dockerfile.server
│   └── docker-compose.yaml
├── config/                      # Non-ROS config (API keys, etc.)
│   └── .env.example
├── docs/
└── README.md
```

### Config File Conventions

**ROS2 Parameter Files:** `config/<component>_params.yaml`
```yaml
# config/nav2_params.yaml
nav2_loco_bridge:
  ros__parameters:
    max_linear_velocity: 0.5      # m/s
    max_angular_velocity: 1.0     # rad/s
    cmd_vel_timeout: 0.5          # seconds
```

**Launch Files:** Python-based (`*_launch.py`), not XML
```python
# launch/inspection_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g1_navigation',
            executable='loco_bridge',
            name='nav2_loco_bridge',
            namespace='g1',
            parameters=['config/nav2_params.yaml'],
        ),
    ])
```

### Message & Service Definitions

| Type | Convention | Example |
|------|------------|---------|
| **Message files** | PascalCase.msg | `InspectionStatus.msg` |
| **Service files** | VerbNoun.srv | `StartInspection.srv` |
| **Action files** | VerbNoun.action | `ExecuteInspection.action` |
| **Field names** | snake_case | `current_waypoint`, `defect_count` |

**Example Message:**
```
# msg/InspectionStatus.msg
std_msgs/Header header
uint8 state                    # Current state enum
uint32 current_waypoint
uint32 total_waypoints
float32 completion_percentage
string status_message
```

### Logging Conventions

**Format:** Structured with context tags
```python
# Good - structured and filterable
self.get_logger().info(f"[INSPECTION] Waypoint reached: id={waypoint_id}, pose=({x:.2f}, {y:.2f})")
self.get_logger().warn(f"[NAVIGATION] Path blocked, replanning: obstacle_at=({ox:.2f}, {oy:.2f})")
self.get_logger().error(f"[LOCALIZATION] Confidence low: confidence={conf:.2f}, threshold={thresh:.2f}")

# Bad - unstructured
self.get_logger().info("reached waypoint")
```

**Log Levels:**
| Level | Usage |
|-------|-------|
| `DEBUG` | Sensor values, state transitions, detailed traces |
| `INFO` | Normal operation milestones (started, completed, waypoint reached) |
| `WARN` | Recoverable issues (replanning, retry, low battery warning) |
| `ERROR` | Failures requiring attention (sensor failure, localization lost) |

**Context Tags:** `[INSPECTION]`, `[NAVIGATION]`, `[PERCEPTION]`, `[LOCALIZATION]`, `[SAFETY]`

### Error Handling Patterns

**ROS2 Node Errors:**
```python
try:
    result = self.call_vlm_api(image)
except APIError as e:
    self.get_logger().error(f"[PERCEPTION] VLM API failed: {e}")
    # Publish error status, don't crash node
    self.publish_error_status(str(e))
    return None
```

**Graceful Degradation:** Nodes should publish error status, not crash. Let the state machine handle recovery.

### Testing Conventions

| Test Type | Location | Naming |
|-----------|----------|--------|
| Unit tests | `<package>/test/test_*.py` | `test_loco_bridge.py` |
| Integration tests | `<package>/test/integration/` | `test_nav_integration.py` |
| Launch tests | `g1_bringup/test/` | `test_launch.py` |

**Test Framework:** pytest with pytest-ros (for ROS2 tests)

### Enforcement Guidelines

**All Contributors MUST:**
1. Follow ROS2 naming conventions (snake_case topics, PascalCase messages)
2. Use `/g1/` namespace for all project nodes/topics
3. Include type hints on all public functions
4. Use structured logging with context tags
5. Place tests in `test/` directory (not co-located)
6. Use Python launch files, not XML

**Code Review Checklist:**
- [ ] Naming follows conventions
- [ ] Type hints present
- [ ] Docstrings on public functions
- [ ] Structured logging used
- [ ] Error handling doesn't crash nodes
- [ ] Tests included for new functionality

## Project Structure & Boundaries

### Requirements → Package Mapping

| PRD Category | Package | Key Components |
|--------------|---------|----------------|
| **Plan Management (FR1-5)** | `g1_inspection` | `plan_parser.py`, plan storage |
| **Robot Control (FR6-11)** | `g1_bringup` | Launch files, operator interface |
| **Navigation (FR12-18)** | `g1_navigation` | `loco_bridge.py`, `waypoint_manager.py` |
| **Localization (FR19-22)** | `g1_navigation` | slam_toolbox integration |
| **Visual Capture (FR23-27)** | `g1_perception` | `camera_node.py`, `image_capture.py` |
| **Defect Detection (FR28-33)** | `g1_inspection` | `defect_detector.py` (offloaded) |
| **Reporting (FR34-40)** | `g1_inspection` | `report_generator.py` |
| **Notifications (FR41-44)** | `g1_inspection` | `state_machine.py` (status publishing) |

### ROS2 Integration Boundaries

#### Topics (Data Flow)

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 TOPIC BOUNDARIES                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Sensor Topics (g1_perception → all):                          │
│  ├── /g1/camera/rgb           → Image capture, defect detect   │
│  ├── /g1/camera/depth         → Obstacle detection             │
│  ├── /g1/lidar/points         → SLAM, Nav2 costmap             │
│  └── /g1/imu/data             → Localization                   │
│                                                                 │
│  Navigation Topics:                                             │
│  ├── /g1/cmd_vel              → Nav2 output, loco_bridge input │
│  ├── /g1/odom                 → Odometry for SLAM              │
│  ├── /map                     → slam_toolbox output            │
│  └── /tf, /tf_static          → Coordinate transforms          │
│                                                                 │
│  Inspection Topics (g1_inspection → operator):                 │
│  ├── /g1/inspection/status    → Current state, progress        │
│  ├── /g1/inspection/defects   → Detected defects stream        │
│  └── /g1/inspection/waypoint  → Current waypoint info          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### Services (Control)

| Service | Package | Purpose |
|---------|---------|---------|
| `/g1/start_inspection` | g1_inspection | Begin inspection route |
| `/g1/pause_inspection` | g1_inspection | Pause current inspection |
| `/g1/resume_inspection` | g1_inspection | Resume paused inspection |
| `/g1/abort_inspection` | g1_inspection | Cancel and return to idle |
| `/g1/get_status` | g1_inspection | Query current state |

#### Actions (Long-running)

| Action | Package | Purpose |
|--------|---------|---------|
| `/g1/ExecuteInspection` | g1_inspection | Full inspection with feedback |
| `/g1/navigate_to_pose` | Nav2 | Standard Nav2 navigation |

### Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      DATA FLOW                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────┐                                                   │
│  │  Sensors │                                                   │
│  └────┬─────┘                                                   │
│       │                                                         │
│       ▼                                                         │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │g1_perception │───▶│   ROS2 Bag   │───▶│  Offload     │      │
│  │ (capture)    │    │  (storage)   │    │  Server      │      │
│  └──────┬───────┘    └──────────────┘    └──────┬───────┘      │
│         │                                        │              │
│         ▼                                        ▼              │
│  ┌──────────────┐                         ┌──────────────┐      │
│  │slam_toolbox  │                         │ VLM API      │      │
│  │  (2D SLAM)   │                         │ (detection)  │      │
│  └──────┬───────┘                         └──────┬───────┘      │
│         │                                        │              │
│         ▼                                        ▼              │
│  ┌──────────────┐                         ┌──────────────┐      │
│  │    Nav2      │                         │   Report     │      │
│  │ (planning)   │                         │  Generator   │      │
│  └──────┬───────┘                         └──────┬───────┘      │
│         │                                        │              │
│         ▼                                        ▼              │
│  ┌──────────────┐                         ┌──────────────┐      │
│  │g1_navigation │                         │  PDF Report  │      │
│  │(loco_bridge) │                         │              │      │
│  └──────┬───────┘                         └──────────────┘      │
│         │                                                       │
│         ▼                                                       │
│  ┌──────────────┐                                               │
│  │  LocoClient  │                                               │
│  │ (SDK cmds)   │                                               │
│  └──────────────┘                                               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Coordinate Frame Hierarchy

```
map (slam_toolbox)
 └── odom (odometry)
      └── base_link (robot base)
           ├── imu_link
           ├── lidar_link (MID-360)
           └── camera_link (D435i)
                ├── camera_rgb_frame
                └── camera_depth_frame
```

**TF Publishers:**
- `slam_toolbox` → `map` → `odom`
- `robot_state_publisher` → `odom` → `base_link` → sensor frames
- Static transforms for sensor mounting positions

### Package Dependencies

```
g1_interfaces (messages/services)
       ▲
       │
┌──────┴──────┬──────────────┬──────────────┐
│             │              │              │
g1_bringup  g1_navigation  g1_perception  g1_inspection
    │             │              │              │
    └─────────────┴──────────────┴──────────────┘
                        │
              (all depend on g1_interfaces)
```

**External Dependencies:**
- `g1_navigation` → Nav2, slam_toolbox, unitree_sdk2_python
- `g1_perception` → realsense2_camera, livox_ros2_driver
- `g1_inspection` → (offload server for VLM API)

### Development vs Production Structure

| Aspect | Development | Production (Jetson) |
|--------|-------------|---------------------|
| **Launch** | `sim_launch.py` | `robot_launch.py` |
| **Locomotion** | Fake (teleport) | SDK LocoClient |
| **Sensors** | Simulated in MuJoCo | Real hardware |
| **Config** | `config/sim_*.yaml` | `config/robot_*.yaml` |
| **Network** | localhost | 192.168.123.x / WiFi |

## Architecture Validation Results

### Coherence Validation ✅

**Decision Compatibility:**
All technology choices validated as compatible:
- Python 3.10 + unitree_sdk2_python ✅
- ROS2 Humble + CycloneDDS 0.10.x ✅
- Nav2 + slam_toolbox ✅
- MuJoCo simulation + fake locomotion ✅
- Docker + Jetson Orin NX ✅

**Pattern Consistency:**
- ROS2 naming conventions (snake_case topics, PascalCase messages) applied consistently
- `/g1/` namespace used throughout
- Python PEP 8 style enforced

**Structure Alignment:**
- Multi-package structure supports modularity
- Package boundaries match functional areas
- TF frame hierarchy properly defined

### Requirements Coverage Validation ✅

**Functional Requirements Coverage:**

| FR Category | Status | Architecture Support |
|-------------|--------|---------------------|
| Plan Management (FR1-5) | ✅ | `g1_inspection` - plan_parser.py |
| Robot Control (FR6-11) | ✅ | `g1_bringup` + CLI + RViz |
| Navigation (FR12-18) | ✅ | Nav2 + loco_bridge (flat floors MVP) |
| Localization (FR19-22) | ✅ | slam_toolbox |
| Visual Capture (FR23-27) | ✅ | Coverage-based image capture |
| Defect Detection (FR28-33) | ✅ | VLM API (offloaded) |
| Reporting (FR34-40) | ✅ | report_generator.py |
| Notifications (FR41-44) | ✅ | Status topics + state machine |
| Physical Marking (FR45-47) | ⬜ | Deferred to Phase 2 |
| Stair Navigation (FR15-16) | ⬜ | Deferred post-MVP |

**Non-Functional Requirements:**
- ✅ 500ms obstacle response (onboard Nav2)
- ✅ 10Hz localization (slam_toolbox)
- ✅ 1fps image capture (continuous)
- ✅ E-stop <500ms (g1_safety node)
- ✅ 2hr battery monitoring (state machine)

### Implementation Readiness Validation ✅

**Decision Completeness:** All critical decisions documented with versions
**Structure Completeness:** Full directory tree with package boundaries
**Pattern Completeness:** Naming, logging, error handling patterns defined

### Gaps Addressed

| Gap | Resolution |
|-----|------------|
| Safety/E-stop | Added `g1_safety` package |
| Operator Interface | CLI + RViz (no custom UI for MVP) |
| Plan Input Format | PNG or PDF accepted |
| Inspection Strategy | Coverage-based (not waypoint-based) |
| Stairs | Deferred post-MVP (flat floors only) |
| WiFi Disconnect | State machine pauses, resumes on reconnect |

### Additional Architecture Decisions

#### Safety Package (`g1_safety`)

```
src/g1_safety/
├── g1_safety/
│   ├── __init__.py
│   ├── safety_node.py       # E-stop, collision, fall prevention
│   └── battery_monitor.py   # Low battery detection
├── test/
└── package.xml
```

**Safety Node Responsibilities:**
- Subscribe to E-stop commands (hardware + software)
- Monitor collision proximity from Nav2 costmap
- Track battery level, trigger RETURNING_HOME state
- Publish `/g1/safety/status` topic
- Service `/g1/emergency_stop` (immediate halt)

**E-stop Behavior:**
```python
def emergency_stop_callback(self, request, response):
    # Immediately command zero velocity
    self.loco_client.SetVelocity(0, 0, 0, 0)
    # Transition to EMERGENCY_STOP state
    self.publish_state_transition(State.EMERGENCY_STOP)
    # Log event
    self.get_logger().error("[SAFETY] E-STOP triggered")
    return response
```

#### Operator Interface (CLI + RViz)

**CLI Commands:**
```bash
# Start inspection with construction plan
g1-inspect start --plan /path/to/floor_plan.pdf

# Check status
g1-inspect status

# Pause/Resume
g1-inspect pause
g1-inspect resume

# Emergency stop
g1-inspect stop

# View live (opens RViz)
g1-inspect viz
```

**Implementation:** Simple Python CLI wrapping ROS2 service calls
```
scripts/
└── g1_inspect.py    # CLI entry point
```

**RViz for Development:**
- Visualize map, robot pose, path
- Camera feed overlay
- Inspection status panel
- No custom UI development needed

#### Plan Input Format

**Supported Formats:**
- PNG (raster floor plan image)
- PDF (vector or raster floor plan)

**Plan Parser Pipeline:**
```
┌─────────┐    ┌─────────────┐    ┌─────────────┐
│ PNG/PDF │───▶│ plan_parser │───▶│ Spatial     │
│  Input  │    │   .py       │    │ Context     │
└─────────┘    └─────────────┘    └─────────────┘
                     │
                     ▼
              ┌─────────────┐
              │ Room bounds │
              │ Scale info  │
              │ VLM context │
              └─────────────┘
```

**Usage:**
- Extract room boundaries for coverage planning
- Provide visual context to VLM for "expected vs actual" comparison
- No CAD parsing required for MVP

#### Coverage-Based Inspection Strategy

**Simplified Approach (No Predefined Waypoints):**

```
┌─────────────────────────────────────────────────────────────────┐
│              COVERAGE-BASED INSPECTION                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. Robot navigates space using Nav2 exploration               │
│  2. Continuous image capture at 1fps                           │
│  3. Track surface coverage based on:                           │
│     - Camera FOV (D435i ~87° horizontal)                       │
│     - Robot pose from SLAM                                     │
│     - Distance to surfaces                                     │
│  4. Continue until all reachable surfaces photographed         │
│  5. All images sent to VLM for defect analysis                │
│                                                                 │
│  Coverage Metrics:                                              │
│  - Percentage of floor area covered                            │
│  - Wall surfaces within camera range                           │
│  - Overlap for stereo/depth verification                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Benefits:**
- No manual waypoint definition
- Adapts to any floor layout
- Complete coverage guaranteed by algorithm
- Simpler than predefined inspection routes

#### WiFi Disconnect Handling

**Behavior:**
1. Detect WiFi loss (offload server unreachable)
2. Transition to `WAITING_OPERATOR` state
3. Continue local ROS2 bag recording
4. Pause defect detection (requires offload)
5. On reconnect: resume inspection, sync captured data

**Not a safety issue:** Navigation and obstacle avoidance run onboard, don't require WiFi.

### MVP Scope Boundaries

| Feature | MVP | Post-MVP |
|---------|-----|----------|
| Flat floor navigation | ✅ | |
| Stair climbing | | ✅ |
| Coverage-based inspection | ✅ | |
| VLM defect detection | ✅ | |
| CLI + RViz interface | ✅ | |
| Web dashboard | | ✅ |
| Multi-robot coordination | | ✅ |
| Physical marking (tape) | | ✅ |
| Automatic charging/docking | | ✅ |

### Architecture Completeness Checklist

**✅ Requirements Analysis**
- [x] Project context thoroughly analyzed
- [x] Scale and complexity assessed
- [x] Technical constraints identified
- [x] Cross-cutting concerns mapped

**✅ Architectural Decisions**
- [x] Critical decisions documented with versions
- [x] Technology stack fully specified
- [x] Integration patterns defined
- [x] Safety architecture defined

**✅ Implementation Patterns**
- [x] Naming conventions established
- [x] Structure patterns defined
- [x] Communication patterns specified
- [x] Process patterns documented

**✅ Project Structure**
- [x] Complete directory structure defined
- [x] Component boundaries established
- [x] Integration points mapped
- [x] Requirements to structure mapping complete

### Architecture Readiness Assessment

**Overall Status:** ✅ READY FOR IMPLEMENTATION

**Confidence Level:** HIGH

**Key Strengths:**
- Clean separation between Nav2 planning and SDK locomotion
- Simplified coverage-based inspection (no waypoint complexity)
- VLM API approach allows fast iteration without model training
- Docker deployment ensures reproducibility
- Safety node provides explicit E-stop handling

**First Implementation Priority:**
1. Run `scripts/setup.sh` to clone dependencies
2. Create `g1_interfaces` package (messages/services)
3. Implement `g1_navigation/loco_bridge.py` (critical integration)
4. Set up simulation environment with MuJoCo

