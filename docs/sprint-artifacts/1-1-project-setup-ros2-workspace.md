# Story 1.1: Project Setup & ROS2 Workspace

**Status:** Ready for Review

## Story

As a developer,
I want the complete project structure with all packages and dependencies configured,
So that I can build and run the system.

## Acceptance Criteria

1. `scripts/setup.sh` runs successfully on fresh Ubuntu 22.04 + ROS2 Humble
2. `colcon build` completes without errors
3. All custom interfaces importable: `from g1_interfaces.msg import InspectionStatus`
4. External deps cloned to `external/` (gitignored)
5. All 6 packages visible via `ros2 pkg list | grep g1_`

## Tasks / Subtasks

- [x] Task 1: Create ROS2 workspace structure (AC: 2, 5)
  - [x] 1.1 Create `src/` directory for ROS2 packages
  - [x] 1.2 Create `g1_interfaces` package (ament_cmake) with msg/, srv/, action/ directories
  - [x] 1.3 Create `g1_bringup` package (ament_python) with launch/, config/ directories
  - [x] 1.4 Create `g1_navigation` package (ament_python) with module directory
  - [x] 1.5 Create `g1_perception` package (ament_python) with module directory
  - [x] 1.6 Create `g1_inspection` package (ament_python) with module directory
  - [x] 1.7 Create `g1_safety` package (ament_python) with module directory

- [x] Task 2: Define custom messages (AC: 3)
  - [x] 2.1 Create `msg/InspectionStatus.msg` with state, waypoint, completion, battery fields
  - [x] 2.2 Create `msg/DefectReport.msg` with defect type, location, confidence, image_path fields
  - [x] 2.3 Create `msg/Notification.msg` with notification type, message, severity fields

- [x] Task 3: Define custom services (AC: 3)
  - [x] 3.1 Create `srv/StartInspection.srv` with plan_id request, success/message response
  - [x] 3.2 Create `srv/PauseInspection.srv` with empty request, success/message response
  - [x] 3.3 Create `srv/GetStatus.srv` with empty request, InspectionStatus response

- [x] Task 4: Define custom action (AC: 3)
  - [x] 4.1 Create `action/ExecuteInspection.action` with goal, feedback, result

- [x] Task 5: Configure package build files (AC: 2, 3)
  - [x] 5.1 Configure `g1_interfaces/CMakeLists.txt` with rosidl_generate_interfaces
  - [x] 5.2 Configure `g1_interfaces/package.xml` with rosidl dependencies
  - [x] 5.3 Configure each Python package with setup.py and package.xml

- [x] Task 6: Create config file templates (AC: 2)
  - [x] 6.1 Create `config/nav2_params.yaml` with Nav2 configuration template
  - [x] 6.2 Create `config/slam_params.yaml` with slam_toolbox parameters
  - [x] 6.3 Create `config/robot_params.yaml` with robot-specific parameters

- [x] Task 7: Verify and test build (AC: 1, 2, 3, 4, 5)
  - [x] 7.1 Run `colcon build` and fix any errors
  - [x] 7.2 Verify all interfaces importable in Python
  - [x] 7.3 Run `ros2 pkg list | grep g1_` to verify all packages visible

## Dev Notes

### Technical Requirements

**Language:** Python 3.10+ (required by unitree_sdk2_python)
**ROS2 Distribution:** Humble Hawksbill (Ubuntu 22.04)
**DDS Middleware:** CycloneDDS 0.10.x (Unitree + Nav2 compatibility)

### Package Types

| Package | Build Type | Rationale |
|---------|------------|-----------|
| `g1_interfaces` | ament_cmake | **REQUIRED** - Custom messages/services MUST use ament_cmake |
| `g1_bringup` | ament_python | Launch files and configs, no C++ needed |
| `g1_navigation` | ament_python | Nav2 integration, Python SDK bridge |
| `g1_perception` | ament_python | Sensor capture, image processing |
| `g1_inspection` | ament_python | State machine, defect detection |
| `g1_safety` | ament_python | E-stop, battery monitoring |

### Critical: Custom Interface Package Rules (ROS2 Humble)

Per [ROS2 Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html):

1. **Custom interface packages MUST be `ament_cmake`** - Cannot use `ament_python` for messages
2. **Separate package required** - Best practice is to keep interfaces in dedicated package
3. **Directory structure is mandatory:**
   ```
   g1_interfaces/
   ├── msg/
   │   ├── InspectionStatus.msg
   │   ├── DefectReport.msg
   │   └── Notification.msg
   ├── srv/
   │   ├── StartInspection.srv
   │   ├── PauseInspection.srv
   │   └── GetStatus.srv
   ├── action/
   │   └── ExecuteInspection.action
   ├── CMakeLists.txt
   └── package.xml
   ```

### CMakeLists.txt for g1_interfaces (Critical Pattern)

```cmake
cmake_minimum_required(VERSION 3.8)
project(g1_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InspectionStatus.msg"
  "msg/DefectReport.msg"
  "msg/Notification.msg"
  "srv/StartInspection.srv"
  "srv/PauseInspection.srv"
  "srv/GetStatus.srv"
  "action/ExecuteInspection.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_package()
```

### package.xml for g1_interfaces (Critical Pattern)

```xml
<?xml version="1.0"?>
<package format="3">
  <name>g1_interfaces</name>
  <version>0.1.0</version>
  <description>Custom ROS2 interfaces for G1 Inspector</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>action_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Message Definitions

**msg/InspectionStatus.msg:**
```
std_msgs/Header header
uint8 state                    # Current state enum (IDLE=0, CALIBRATING=1, INSPECTING=2, etc.)
uint32 current_waypoint
uint32 total_waypoints
float32 completion_percentage
float32 battery_percentage
geometry_msgs/Pose current_pose
string status_message
```

**msg/DefectReport.msg:**
```
std_msgs/Header header
string defect_id
uint8 defect_type              # LOCATION_ERROR=0, QUALITY_ISSUE=1
string description
geometry_msgs/Point location   # World coordinates
geometry_msgs/Point plan_location  # Plan coordinates
float32 confidence
string image_path
string annotated_image_path
```

**msg/Notification.msg:**
```
std_msgs/Header header
uint8 notification_type        # ROUTE_BLOCKED=0, LOCALIZATION_FAILED=1, COMPLETE=2, LOW_BATTERY=3
uint8 severity                 # INFO=0, WARNING=1, ERROR=2, CRITICAL=3
string message
```

### Service Definitions

**srv/StartInspection.srv:**
```
string plan_id
---
bool success
string message
string inspection_id
```

**srv/PauseInspection.srv:**
```
---
bool success
string message
```

**srv/GetStatus.srv:**
```
---
g1_interfaces/InspectionStatus status
```

### Action Definition

**action/ExecuteInspection.action:**
```
# Goal
string plan_id
---
# Result
bool success
string report_path
uint32 defects_found
float32 coverage_percentage
---
# Feedback
g1_interfaces/InspectionStatus status
```

### Python Package Setup Pattern (for g1_bringup, g1_navigation, etc.)

**setup.py:**
```python
from setuptools import setup

package_name = 'g1_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py', 'launch/robot_launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml', 'config/slam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='G1 Inspector launch files and configuration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

### Project Structure Notes

**Required Directory Structure:**
```
unitree-g1-robot/
├── src/
│   ├── g1_interfaces/        # ament_cmake - MUST be cmake for messages
│   │   ├── msg/
│   │   ├── srv/
│   │   ├── action/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── g1_bringup/           # ament_python
│   │   ├── g1_bringup/
│   │   │   └── __init__.py
│   │   ├── launch/
│   │   ├── config/
│   │   ├── resource/
│   │   │   └── g1_bringup
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   ├── g1_navigation/        # ament_python
│   ├── g1_perception/        # ament_python
│   ├── g1_inspection/        # ament_python
│   └── g1_safety/            # ament_python
├── external/                  # gitignored, populated by setup.sh
├── scripts/
│   └── setup.sh              # Already exists
├── docs/
└── README.md
```

### Naming Conventions (Architecture Compliance)

| Element | Convention | Example |
|---------|------------|---------|
| Topic names | /namespace/topic (snake_case) | `/g1/cmd_vel`, `/g1/inspection/status` |
| Service names | /namespace/verb_noun | `/g1/start_inspection` |
| Node names | snake_case | `nav2_loco_bridge`, `safety_node` |
| Package names | snake_case with g1_ prefix | `g1_navigation` |
| Message files | PascalCase.msg | `InspectionStatus.msg` |
| Field names | snake_case | `current_waypoint`, `battery_percentage` |

**Namespace:** All project nodes MUST use `/g1/` namespace prefix.

### Dependencies (Already Handled)

The `scripts/setup.sh` already handles external dependencies:
- CycloneDDS 0.10.5
- unitree_sdk2 (C++)
- unitree_sdk2_python
- unitree_ros2
- unitree_mujoco

**DO NOT** modify setup.sh - it's already complete.

### Build Commands

```bash
# Source ROS2 and set middleware (already in setup.sh instructions)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_HOME=/usr/local

# Build workspace
cd ~/unitree-g1-robot
colcon build

# Source workspace
source install/setup.bash

# Verify interfaces
python3 -c "from g1_interfaces.msg import InspectionStatus; print('OK')"
python3 -c "from g1_interfaces.srv import StartInspection; print('OK')"
python3 -c "from g1_interfaces.action import ExecuteInspection; print('OK')"

# Verify packages
ros2 pkg list | grep g1_
```

### Anti-Patterns to Avoid

1. **DO NOT** put messages in Python packages - will fail to build
2. **DO NOT** create messages without `rosidl_default_generators` dependency
3. **DO NOT** forget `<member_of_group>rosidl_interface_packages</member_of_group>` in package.xml
4. **DO NOT** use camelCase for topics/services - must be snake_case
5. **DO NOT** create packages outside `src/` directory - colcon won't find them
6. **DO NOT** forget resource file in Python packages - package won't be discovered

### Config File Templates

**config/nav2_params.yaml** (placeholder for Story 3):
```yaml
# Nav2 Configuration for G1 Inspector
# Full configuration will be added in Story 3: Navigation Stack Integration

nav2_loco_bridge:
  ros__parameters:
    max_linear_velocity: 0.5      # m/s
    max_angular_velocity: 1.0     # rad/s
    cmd_vel_timeout: 0.5          # seconds
```

**config/slam_params.yaml** (placeholder for Story 3):
```yaml
# SLAM Toolbox Configuration
# Full configuration will be added in Story 3: Navigation Stack Integration

slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
```

**config/robot_params.yaml:**
```yaml
# G1 Robot Parameters
robot:
  ros__parameters:
    robot_namespace: "g1"
    robot_ip: "192.168.123.164"   # Default, can be overridden
    lidar_ip: "192.168.123.120"   # Default, can be overridden
```

### References

- [Source: docs/architecture.md#Project-Structure]
- [Source: docs/architecture.md#Implementation-Patterns]
- [Source: docs/architecture.md#ROS2-Naming-Conventions]
- [Source: docs/epics.md#Story-1]
- [ROS2 Custom Interfaces Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

## Runnable Verification

```bash
# 1. Source ROS2 (if not already)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2. Build workspace
cd ~/unitree-g1-robot
colcon build

# 3. Source and verify interfaces
source install/setup.bash
python3 -c "from g1_interfaces.msg import InspectionStatus; print('InspectionStatus OK')"
python3 -c "from g1_interfaces.srv import StartInspection; print('StartInspection OK')"
python3 -c "from g1_interfaces.action import ExecuteInspection; print('ExecuteInspection OK')"

# 4. List packages
ros2 pkg list | grep g1_
# Expected output:
# g1_bringup
# g1_inspection
# g1_interfaces
# g1_navigation
# g1_perception
# g1_safety
```

## Definition of Done

All verification commands pass:
- `colcon build` succeeds with no errors
- All interfaces importable in Python
- All 6 packages visible via `ros2 pkg list | grep g1_`

## Dev Agent Record

### Context Reference

Story context from create-story workflow - ROS2 Humble on Ubuntu 22.04 with CycloneDDS middleware.

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Completion Notes List

- **Task 1:** Created complete ROS2 workspace structure with `src/` directory containing all 6 packages (g1_interfaces, g1_bringup, g1_navigation, g1_perception, g1_inspection, g1_safety). Each Python package has proper structure with `__init__.py`, `resource/`, `setup.py`, `setup.cfg`, and `package.xml`.

- **Task 2:** Created 3 custom message definitions (InspectionStatus.msg, DefectReport.msg, Notification.msg) with proper field types including std_msgs/Header and geometry_msgs types.

- **Task 3:** Created 3 custom service definitions (StartInspection.srv, PauseInspection.srv, GetStatus.srv) following ROS2 service patterns.

- **Task 4:** Created ExecuteInspection.action with goal (plan_id), result (success, report_path, defects_found, coverage), and feedback (InspectionStatus).

- **Task 5:** Configured g1_interfaces with CMakeLists.txt using rosidl_generate_interfaces and proper package.xml with rosidl dependencies. Configured all Python packages with setup.py, setup.cfg, and package.xml.

- **Task 6:** Created config file templates in g1_bringup/config/ for nav2_params.yaml, slam_params.yaml, and robot_params.yaml.

- **Task 7:** Successfully built workspace with `colcon build --packages-select`. Required installing `empy`, `lark`, and `catkin_pkg` in conda environment. All interfaces verified importable. All 6 packages visible via `ros2 pkg list | grep g1_`.

### File List

**New Files Created:**
- src/g1_interfaces/CMakeLists.txt
- src/g1_interfaces/package.xml
- src/g1_interfaces/msg/InspectionStatus.msg
- src/g1_interfaces/msg/DefectReport.msg
- src/g1_interfaces/msg/Notification.msg
- src/g1_interfaces/srv/StartInspection.srv
- src/g1_interfaces/srv/PauseInspection.srv
- src/g1_interfaces/srv/GetStatus.srv
- src/g1_interfaces/action/ExecuteInspection.action
- src/g1_bringup/g1_bringup/__init__.py
- src/g1_bringup/setup.py
- src/g1_bringup/setup.cfg
- src/g1_bringup/package.xml
- src/g1_bringup/resource/g1_bringup
- src/g1_bringup/config/nav2_params.yaml
- src/g1_bringup/config/slam_params.yaml
- src/g1_bringup/config/robot_params.yaml
- src/g1_navigation/g1_navigation/__init__.py
- src/g1_navigation/setup.py
- src/g1_navigation/setup.cfg
- src/g1_navigation/package.xml
- src/g1_navigation/resource/g1_navigation
- src/g1_perception/g1_perception/__init__.py
- src/g1_perception/setup.py
- src/g1_perception/setup.cfg
- src/g1_perception/package.xml
- src/g1_perception/resource/g1_perception
- src/g1_inspection/g1_inspection/__init__.py
- src/g1_inspection/setup.py
- src/g1_inspection/setup.cfg
- src/g1_inspection/package.xml
- src/g1_inspection/resource/g1_inspection
- src/g1_safety/g1_safety/__init__.py
- src/g1_safety/setup.py
- src/g1_safety/setup.cfg
- src/g1_safety/package.xml
- src/g1_safety/resource/g1_safety

## Change Log

- 2025-12-04: Story 1.1 implementation complete - Created ROS2 workspace with 6 packages, custom interfaces (3 msgs, 3 srvs, 1 action), and config templates. Build verified successful.
