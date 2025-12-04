# Story 1.3: Navigation Stack Integration

**Status:** ready-for-dev

## Story

As the robot,
I want Nav2 and slam_toolbox integrated with the Unitree SDK,
So that I can autonomously navigate and avoid obstacles.

## Acceptance Criteria

1. Robot builds map while navigating (slam_toolbox publishes to `/map` topic)
2. `ros2 action send_goal /navigate_to_pose` moves robot to goal autonomously
3. Robot avoids obstacles within 500ms of detection (NFR1)
4. Path replans when blocked (FR15, FR16)
5. Coverage percentage published to `/g1/inspection/coverage`

## Tasks / Subtasks

- [ ] Task 1: Configure slam_toolbox for 2D SLAM (AC: 1)
  - [ ] 1.1 Update `config/slam_params.yaml` with complete slam_toolbox configuration
  - [ ] 1.2 Set `odom_frame: odom`, `map_frame: map`, `base_frame: base_link`
  - [ ] 1.3 Configure scan topic to use `/g1/scan` (2D projection of MID-360 LiDAR)
  - [ ] 1.4 Set `mode: mapping` for online SLAM
  - [ ] 1.5 Configure `resolution: 0.05` (5cm grid cells for indoor navigation)
  - [ ] 1.6 Set `max_laser_range: 12.0` (match MID-360 range)
  - [ ] 1.7 Configure solver: `solver_plugin: solver_plugins::CeresSolver`

- [ ] Task 2: Create 2D LaserScan projection node (AC: 1, 3)
  - [ ] 2.1 Create `g1_perception/g1_perception/lidar_to_scan.py` node
  - [ ] 2.2 Subscribe to `/g1/lidar/points` (PointCloud2 from MID-360/simulation)
  - [ ] 2.3 Project 3D point cloud to 2D LaserScan at specified height range
  - [ ] 2.4 Publish to `/g1/scan` (sensor_msgs/LaserScan)
  - [ ] 2.5 Configure height range parameter (default: 0.1m to 1.0m above ground)
  - [ ] 2.6 Set scan rate to 10Hz (match LiDAR input rate)
  - [ ] 2.7 Add entry point in setup.py

- [ ] Task 3: Configure Nav2 navigation stack (AC: 2, 3, 4)
  - [ ] 3.1 Update `config/nav2_params.yaml` with complete Nav2 configuration
  - [ ] 3.2 Configure `bt_navigator` with default navigation behavior tree
  - [ ] 3.3 Configure `controller_server` with DWB controller (matches differential drive)
  - [ ] 3.4 Set velocity limits: `max_vel_x: 0.5`, `max_vel_theta: 1.0` (architecture spec)
  - [ ] 3.5 Configure `planner_server` with NavFn global planner
  - [ ] 3.6 Configure `global_costmap` with obstacle and inflation layers
  - [ ] 3.7 Configure `local_costmap` for reactive obstacle avoidance
  - [ ] 3.8 Set `obstacle_range: 2.5`, `raytrace_range: 3.0` for 500ms response time
  - [ ] 3.9 Add recovery behaviors: `spin`, `backup`, `wait`

- [ ] Task 4: Implement Nav2LocoBridge node (AC: 2)
  - [ ] 4.1 Create `g1_navigation/g1_navigation/loco_bridge.py` ROS2 node
  - [ ] 4.2 Subscribe to `/cmd_vel` from Nav2 controller
  - [ ] 4.3 For simulation: forward to `/g1/cmd_vel` (SimLocomotionController handles it)
  - [ ] 4.4 For real robot: call `LocoClient.SetVelocity(vx, vy, omega, duration)`
  - [ ] 4.5 Add `use_simulation` parameter to select mode
  - [ ] 4.6 Implement cmd_vel timeout (0.5s) with stop command
  - [ ] 4.7 Add velocity clamping to architecture limits
  - [ ] 4.8 Add entry point in setup.py

- [ ] Task 5: Fuse D435i depth into costmap (AC: 3)
  - [ ] 5.1 Create `g1_perception/g1_perception/depth_to_pointcloud.py` node (if not using standard plugin)
  - [ ] 5.2 Configure costmap to include depth camera obstacle layer via `obstacle_layer` plugin
  - [ ] 5.3 Set depth observation source in local_costmap: `/g1/camera/depth/points`
  - [ ] 5.4 Configure `max_obstacle_height: 1.5m`, `min_obstacle_height: 0.1m`
  - [ ] 5.5 Test low obstacle detection (obstacles below LiDAR plane)

- [ ] Task 6: Implement coverage tracking (AC: 5)
  - [ ] 6.1 Create `g1_navigation/g1_navigation/coverage_tracker.py` ROS2 node
  - [ ] 6.2 Subscribe to `/map` and `/g1/odom` topics
  - [ ] 6.3 Track visited cells based on robot pose and sensor FOV
  - [ ] 6.4 Publish coverage percentage to `/g1/inspection/coverage` (std_msgs/Float32)
  - [ ] 6.5 Configure coverage cell size parameter (default: 0.5m)
  - [ ] 6.6 Add entry point in setup.py

- [ ] Task 7: Create sim_nav_launch.py (AC: 1, 2)
  - [ ] 7.1 Create `g1_bringup/launch/sim_nav_launch.py`
  - [ ] 7.2 Include existing `sim_launch.py` components
  - [ ] 7.3 Launch slam_toolbox with `online_async_launch.py` and `use_sim_time:=True`
  - [ ] 7.4 Launch Nav2 bringup with `bringup_launch.py`
  - [ ] 7.5 Launch lidar_to_scan node
  - [ ] 7.6 Launch loco_bridge node with `use_simulation:=True`
  - [ ] 7.7 Launch coverage_tracker node
  - [ ] 7.8 Create updated RViz config `config/rviz/nav.rviz` with map, path, costmap displays

- [ ] Task 8: Verification and testing (AC: 1-5)
  - [ ] 8.1 Run `ros2 launch g1_bringup sim_nav_launch.py` and verify no errors
  - [ ] 8.2 Verify `/map` topic publishes occupancy grid
  - [ ] 8.3 Test navigation goal via CLI: `ros2 action send_goal /navigate_to_pose ...`
  - [ ] 8.4 Verify robot navigates to goal in RViz
  - [ ] 8.5 Test obstacle avoidance by placing obstacle in path
  - [ ] 8.6 Verify path replanning when blocked
  - [ ] 8.7 Verify `/g1/inspection/coverage` publishes coverage percentage
  - [ ] 8.8 Create unit tests for lidar_to_scan, loco_bridge, coverage_tracker

## Dev Notes

### Technical Requirements

**Language:** Python 3.10+ (required by unitree_sdk2_python)
**ROS2 Distribution:** Humble Hawksbill (Ubuntu 22.04)
**DDS Middleware:** CycloneDDS 0.10.x
**Navigation Stack:** Nav2 1.1.x (Humble)
**SLAM Library:** slam_toolbox 2.6.x (Humble)

### Installation Commands

```bash
# Install Nav2 and slam_toolbox (if not already installed)
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox

# Verify installation
ros2 pkg list | grep -E "nav2|slam_toolbox"
```

### Architecture Context

From architecture.md, the key integration pattern is:

```
┌─────────────────────────────────────────────────────────────────┐
│  ROS2 LAYER (Nav2 + slam_toolbox):                             │
│  ├── Nav2 path planning → /cmd_vel velocity commands           │
│  ├── slam_toolbox localization                                 │
│                                                                 │
│  BRIDGE (Nav2 → SDK/Sim):                                      │
│  └── /cmd_vel subscriber → LocoClient.SetVelocity() OR         │
│      forward to /g1/cmd_vel for simulation                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Key Insight:** Nav2 decides WHERE to go (path planning, obstacle avoidance), SDK/Sim handles HOW to walk there.

### Nav2 Configuration Structure

The complete Nav2 configuration must include these server configurations:

```yaml
# Essential Nav2 servers to configure:
bt_navigator:       # Behavior tree orchestrator
controller_server:  # Local path following (DWB)
planner_server:     # Global path planning (NavFn/Smac)
behavior_server:    # Recovery behaviors
waypoint_follower:  # Optional for inspection routes
velocity_smoother:  # Optional, smooth velocity commands
```

### DWB Controller Configuration (Recommended for Humanoid)

DWB (Dynamic Window Approach B) is recommended for the G1 because:
- Works with differential-drive-like velocity commands
- Good for tight spaces and reactive obstacle avoidance
- Well-tested with humanoid-style robots

Key DWB parameters:
```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5        # Architecture spec
      min_vel_x: 0.0
      max_vel_y: 0.0        # No lateral movement for humanoid
      max_vel_theta: 1.0    # Architecture spec
      min_speed_theta: 0.4  # Prevents getting stuck
      acc_lim_x: 1.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.0
      decel_lim_theta: -2.0
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.1
      transform_tolerance: 0.5
      # Critics for path following
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

### **CRITICAL: Complete Costmap Configuration**

The costmaps are essential for obstacle avoidance. Without proper configuration, the 500ms obstacle response (NFR1) cannot be achieved.

#### Local Costmap (Reactive Obstacle Avoidance)

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      update_frequency: 5.0        # 200ms update cycle
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3                     # 3m x 3m local window
      height: 3
      resolution: 0.05             # 5cm resolution
      robot_radius: 0.3            # G1 approximate radius
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan depth
        scan:
          topic: /g1/scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
        depth:
          topic: /g1/camera/depth/points
          max_obstacle_height: 1.5
          min_obstacle_height: 0.1
          clearing: True
          marking: True
          data_type: "PointCloud2"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55     # Must be > robot_radius

      always_send_full_costmap: True
```

#### Global Costmap (Path Planning)

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /g1/scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          obstacle_max_range: 2.5

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

### slam_toolbox Complete Configuration

```yaml
slam_toolbox:
  ros__parameters:
    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG

    # Frames
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /g1/scan
    mode: mapping

    # General
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 3.0
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_timeout: 0.5
    tf_buffer_duration: 30.0

    # Stack size for loop closure
    stack_size_to_use: 40000000

    # Matching
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Loop closure
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan matcher
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

### 2D LaserScan Projection Pattern

The MID-360 LiDAR produces 3D PointCloud2 data. slam_toolbox requires 2D LaserScan.
Create a projection node following this pattern:

```python
# Key implementation points:
# 1. Filter points within height range [min_height, max_height]
# 2. Project to 2D plane (x, y only)
# 3. Convert to polar coordinates (range, angle)
# 4. Fill LaserScan message with range values
# 5. Handle empty bins (set to inf or max_range)
```

### TF Frame Requirements for Nav2

Nav2 requires these TF transforms to be published:

```
map (slam_toolbox publishes)
 └── odom (odometry source publishes)
      └── base_link (robot base)
           ├── lidar_link (MID-360)
           ├── camera_link (D435i)
           └── imu_link
```

**From Previous Stories:**
- Story 1.2: SimLocomotionController publishes `odom` → `base_link`
- Story 1.2.5: HardwareBridge publishes `odom` → `base_link` for real robot
- slam_toolbox will publish `map` → `odom`

### ROS2 Topic Naming (Architecture Compliance)

| Topic | Message Type | Source | Purpose |
|-------|-------------|--------|---------|
| `/g1/scan` | sensor_msgs/LaserScan | lidar_to_scan | 2D scan for SLAM |
| `/g1/lidar/points` | sensor_msgs/PointCloud2 | Sim/Real LiDAR | 3D point cloud |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox | Generated map |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 | Velocity commands |
| `/g1/cmd_vel` | geometry_msgs/Twist | loco_bridge | Robot velocity |
| `/g1/inspection/coverage` | std_msgs/Float32 | coverage_tracker | Coverage % |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | Nav2 Action | Navigation goal |

### File Locations (Architecture Compliance)

```
src/
├── g1_bringup/
│   ├── launch/
│   │   ├── sim_launch.py           # EXISTS (from Story 1.2)
│   │   └── sim_nav_launch.py       # NEW - simulation with navigation
│   ├── config/
│   │   ├── nav2_params.yaml        # UPDATE - full Nav2 config
│   │   ├── slam_params.yaml        # UPDATE - full slam_toolbox config
│   │   └── rviz/
│   │       ├── sim.rviz            # EXISTS (from Story 1.2)
│   │       └── nav.rviz            # NEW - navigation visualization
├── g1_navigation/
│   └── g1_navigation/
│       ├── __init__.py             # EXISTS
│       ├── sim_locomotion.py       # EXISTS (from Story 1.2)
│       ├── hardware_bridge.py      # EXISTS (from Story 1.2.5)
│       ├── loco_bridge.py          # NEW - Nav2 to SDK/Sim bridge
│       └── coverage_tracker.py     # NEW - coverage percentage
└── g1_perception/
    └── g1_perception/
        ├── __init__.py             # EXISTS
        └── lidar_to_scan.py        # NEW - 3D to 2D projection
```

### Previous Story Intelligence

**Story 1.2 (Simulation Environment) - Key Patterns:**
- SimLocomotionController subscribes to `/g1/cmd_vel`, publishes `/g1/odom` and TF
- Velocity limits: `MAX_LINEAR_VELOCITY = 0.5`, `MAX_ANGULAR_VELOCITY = 1.0`
- cmd_vel timeout: 0.5 seconds
- Simulated LiDAR publishes to `/g1/lidar/points` at 10Hz

**Story 1.2.5 (Hardware Connectivity) - Key Patterns:**
- HardwareBridge uses `LocoClient.Move(vx, vy, omega)` for locomotion
- Conservative velocity limits: 0.3 m/s linear for safety
- Auto-stand on startup, damp on shutdown

**Story 1.2.6 (Sensor Hello World) - Key Learnings:**
- LiDAR topic is `utlidar/cloud` on real robot (from unitree_ros2)
- Frame is `utlidar_lidar`
- Need to map to `/g1/lidar/points` or use directly

**Build Commands:**
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_HOME=/usr/local
cd ~/unitree-g1-robot
colcon build
source install/setup.bash
```

### Code Style Requirements (Architecture Compliance)

**Python:**
- Classes: PascalCase (`Nav2LocoBridge`, `CoverageTracker`)
- Functions: snake_case (`compute_coverage()`)
- Constants: UPPER_SNAKE (`MAX_VELOCITY`)
- Type hints required on all public functions
- Google-style docstrings

**Logging:**
```python
self.get_logger().info(f"[NAVIGATION] Navigation goal received: x={x:.2f}, y={y:.2f}")
self.get_logger().warn(f"[NAVIGATION] Path blocked, replanning...")
self.get_logger().error(f"[NAVIGATION] Failed to compute path: {error}")
```

**Context Tags:** `[NAVIGATION]`, `[PERCEPTION]`, `[SLAM]`

### Nav2 Behavior Tree Configuration

Nav2 uses behavior trees for navigation logic. Configure in nav2_params.yaml:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /g1/odom
    bt_loop_duration: 10
    default_server_timeout: 20
    # Use built-in BT with replanning and recovery
    default_bt_xml_filename: ""  # Empty = use default navigate_to_pose_w_replanning_and_recovery.xml
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
```

Key behaviors included:
- ComputePathToPose (global planning)
- FollowPath (local controller)
- Recovery behaviors: Spin, Backup, Wait

### **CRITICAL: Launch Ordering and TF Timing**

Nav2 requires a complete TF tree (map → odom → base_link) before starting. If Nav2 initializes before slam_toolbox publishes the map→odom transform, navigation will fail with "Could not transform" errors.

**Required Launch Order:**
1. `robot_state_publisher` (static sensor TFs)
2. `sim_locomotion` / `mujoco_sim` (odom → base_link)
3. `slam_toolbox` (map → odom) - takes 1-2 seconds to initialize
4. `Nav2` servers (require all transforms)

**Implementation in sim_nav_launch.py:**

```python
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('g1_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 1. Start simulation (includes robot_state_publisher + odom→base_link)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'sim_launch.py')
        )
    )

    # 2. Start slam_toolbox (publishes map→odom)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                        'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam_params_file': os.path.join(bringup_dir, 'config', 'slam_params.yaml')
        }.items()
    )

    # 3. Start Nav2 AFTER delay to ensure TF tree is ready
    nav2_delayed = TimerAction(
        period=3.0,  # Wait 3 seconds for SLAM to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'params_file': os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
                    'map': '',  # No static map, using SLAM
                    'autostart': 'True',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        sim_launch,
        slam_launch,
        nav2_delayed,  # Delayed start ensures TF is ready
        # ... other nodes
    ])
```

### **CRITICAL: setup.py Entry Points Format**

When adding new nodes, update setup.py with exact entry_points format:

**g1_navigation/setup.py:**
```python
entry_points={
    'console_scripts': [
        'sim_locomotion = g1_navigation.sim_locomotion:main',
        'hardware_bridge = g1_navigation.hardware_bridge:main',
        'loco_bridge = g1_navigation.loco_bridge:main',           # NEW
        'coverage_tracker = g1_navigation.coverage_tracker:main', # NEW
    ],
},
```

**g1_perception/setup.py:**
```python
entry_points={
    'console_scripts': [
        'sim_camera = g1_perception.sim_camera:main',
        'sim_lidar = g1_perception.sim_lidar:main',
        'sim_imu = g1_perception.sim_imu:main',
        'lidar_to_scan = g1_perception.lidar_to_scan:main',       # NEW
    ],
},
```

### Anti-Patterns to Avoid

1. **DO NOT** hardcode frame names - use parameters
2. **DO NOT** skip TF timeout handling - Nav2 requires complete TF tree
3. **DO NOT** forget `use_sim_time` parameter for simulation
4. **DO NOT** use blocking calls in ROS2 callbacks
5. **DO NOT** set costmap resolution too fine (memory issues)
6. **DO NOT** forget to map `/cmd_vel` to `/g1/cmd_vel` for our namespace
7. **DO NOT** skip obstacle layer in local costmap (safety requirement)
8. **DO NOT** start Nav2 before slam_toolbox is publishing TF (use TimerAction)

### Performance Requirements (NFRs)

| NFR | Requirement | Implementation |
|-----|-------------|----------------|
| NFR1 | 500ms obstacle response | Local costmap update rate, DWB replanning frequency |
| NFR2 | 10Hz localization | slam_toolbox default is sufficient |
| NFR6 | ≥95% route completion | Recovery behaviors, path replanning |

### Dependencies to Install

```bash
# Core navigation packages
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox

# Additional utilities
sudo apt install ros-humble-tf2-tools
sudo apt install ros-humble-rviz2

# Verify
ros2 pkg list | grep -E "nav2|slam"
```

### Nav2 Topic Namespace Configuration

Nav2 uses default topic names (`/cmd_vel`, `/odom`, `/scan`). Our architecture uses `/g1/*` namespace. Configure remapping:

**In nav2_params.yaml (preferred):**
```yaml
# Each Nav2 server can specify topic names
controller_server:
  ros__parameters:
    odom_topic: /g1/odom

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom  # Not /g1/odom - TF frames don't use namespace

# The loco_bridge node handles /cmd_vel → /g1/cmd_vel translation
```

**Key Topic Mapping:**
| Nav2 Default | Our Topic | Handled By |
|--------------|-----------|------------|
| `/cmd_vel` | `/g1/cmd_vel` | loco_bridge node |
| `/odom` | `/g1/odom` | Configure in nav2_params.yaml |
| `/scan` | `/g1/scan` | Configure in costmap observation_sources |

**Note:** TF frame names (odom, base_link, map) do NOT use namespace prefix.

### Path Planning Failure Handling

When Nav2 cannot find a path, implement this handling in the state machine:

```python
# In your inspection state machine or loco_bridge node:

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class NavigationHandler:
    def navigation_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[NAVIGATION] Goal reached successfully")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("[NAVIGATION] Path planning failed - no valid path")
            self._publish_notification(NotificationType.ROUTE_BLOCKED)
            self._transition_to_state(InspectionState.BLOCKED)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("[NAVIGATION] Navigation canceled")
```

**State Transitions on Failure:**
1. `INSPECTING` → `BLOCKED` when path planning fails
2. `BLOCKED` → `WAITING_OPERATOR` after 3 failed replanning attempts
3. Publish `/g1/notifications` with ROUTE_BLOCKED type

### Real Robot LiDAR Topic Remapping

From Story 1.2.6, the real robot publishes LiDAR to different topics:

| Sensor | Simulation Topic | Real Robot Topic | Frame |
|--------|-----------------|------------------|-------|
| LiDAR | `/g1/lidar/points` | `utlidar/cloud` | `utlidar_lidar` |

---

### ⚠️ ARCHITECTURAL NOTE: ROS2 vs SDK for LiDAR Access

**Context:** The robot doesn't run ROS2 - it runs Unitree firmware that publishes DDS topics. ROS2 on the dev computer can see these topics because both use CycloneDDS as the middleware.

**Key Insight:** The robot's `utlidar/cloud` topic is published via DDS, NOT ROS2. However, because ROS2 Humble uses CycloneDDS as its RMW (ROS Middleware), ROS2 nodes can subscribe to this topic directly via DDS domain discovery.

**Two Options for LiDAR Access:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| **A (Try First)** | Direct DDS discovery - ROS2 subscribes to `utlidar/cloud` directly | Zero code needed, CycloneDDS handles bridging | May have topic naming/QoS mismatches |
| **B (Fallback)** | Add LiDAR to `hardware_bridge.py` using SDK `ChannelSubscriber` | Full control, consistent with IMU/joint bridging | Extra code, slight latency |

**Decision (2025-12-04):** Try Option A first. If slam_toolbox/Nav2 can't see the LiDAR topic via direct DDS discovery, implement Option B by adding SDK-based LiDAR subscription to `hardware_bridge.py`.

**Test Option A:**
```bash
# With robot connected and ROS2 sourced:
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Check if ROS2 can see the robot's DDS topic
ros2 topic list | grep utlidar
ros2 topic echo utlidar/cloud --no-arr --once

# If data flows, configure slam_toolbox/Nav2 to use utlidar/cloud directly
```

**If Option B Needed - Add to hardware_bridge.py:**
```python
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

# In HardwareBridge.__init__:
self.lidar_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
self.lidar_subscriber.Init(self._lidar_callback, 10)
self.lidar_pub = self.create_publisher(PointCloud2, '/g1/lidar/points', 10)

# Add callback to republish SDK data as ROS2 topic
```

---

**Option 1: Remap in launch file (recommended):**
```python
# In robot_nav_launch.py for real robot
Node(
    package='g1_perception',
    executable='lidar_to_scan',
    name='lidar_to_scan',
    parameters=[{
        'input_topic': 'utlidar/cloud',    # Real robot topic
        'output_topic': '/g1/scan',
        'input_frame': 'utlidar_lidar',
        'output_frame': 'lidar_link',      # Needs static TF
    }]
)
```

**Option 2: Use parameter in lidar_to_scan node:**
```python
# In lidar_to_scan.py
self.declare_parameter('input_topic', '/g1/lidar/points')  # Default for sim
input_topic = self.get_parameter('input_topic').value
```

**Required Static TF for Real Robot:**
```python
# Add to robot_launch.py - transform from real LiDAR frame to our frame
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'utlidar_lidar', 'lidar_link']
)
```

### Testing Simulation vs Real Robot

| Component | Simulation | Real Robot |
|-----------|------------|------------|
| Locomotion | SimLocomotionController | HardwareBridge |
| LiDAR Topic | `/g1/lidar/points` | `utlidar/cloud` |
| LiDAR Frame | `lidar_link` | `utlidar_lidar` |
| cmd_vel dest | `/g1/cmd_vel` | `/g1/cmd_vel` (via HardwareBridge) |
| lidar_to_scan input | `/g1/lidar/points` | `utlidar/cloud` (via param) |

### Troubleshooting Common Issues

#### TF Errors: "Could not transform..."

**Symptom:** Nav2 logs errors like "Could not transform from odom to map"

**Causes & Solutions:**
1. **slam_toolbox not running:** Verify with `ros2 topic echo /map --once`
2. **Nav2 started before SLAM:** Use TimerAction delay in launch file (see CRITICAL: Launch Ordering)
3. **Frame name mismatch:** Check `base_frame`, `odom_frame`, `map_frame` match across all configs

```bash
# Debug: Check TF tree
ros2 run tf2_tools view_frames
# Should show: map → odom → base_link → sensor frames
```

#### No Path Found

**Symptom:** Robot doesn't move, action returns STATUS_ABORTED

**Causes & Solutions:**
1. **Goal in obstacle:** Check costmap in RViz, ensure goal is in free space
2. **Costmap not updating:** Verify `/g1/scan` publishes, check observation_sources config
3. **Inflation too large:** Reduce `inflation_radius` if robot can't fit through openings

```bash
# Debug: Check costmaps are publishing
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

#### Robot Oscillates or Spins

**Symptom:** Robot can't settle at goal, keeps adjusting

**Causes & Solutions:**
1. **Goal tolerance too tight:** Increase `xy_goal_tolerance` and `yaw_goal_tolerance`
2. **DWB critics misconfigured:** Adjust `RotateToGoal.scale` and `PathAlign.scale`
3. **Velocity limits too low:** Ensure `min_speed_theta` > 0

#### SLAM Map Drifts or Has Holes

**Symptom:** Map quality degrades over time

**Causes & Solutions:**
1. **LaserScan quality:** Check `/g1/scan` has full 360° coverage
2. **Robot moving too fast:** Reduce velocity for better scan matching
3. **Loop closure failing:** Check `do_loop_closing: true` and tune thresholds

```bash
# Debug: Monitor SLAM quality
ros2 topic echo /slam_toolbox/scan_matching_response
```

#### cmd_vel Not Reaching Robot

**Symptom:** Nav2 sends commands but robot doesn't move

**Causes & Solutions:**
1. **loco_bridge not running:** Check `ros2 node list | grep loco`
2. **Topic mismatch:** Verify loco_bridge subscribes to `/cmd_vel` and publishes to `/g1/cmd_vel`
3. **Timeout:** Check cmd_vel_timeout isn't too short

```bash
# Debug: Echo both topics
ros2 topic echo /cmd_vel &
ros2 topic echo /g1/cmd_vel &
```

### References

- [Source: docs/architecture.md#Nav2-to-LocoClient-Bridge]
- [Source: docs/architecture.md#Perception-Pipeline]
- [Source: docs/architecture.md#Robot-State-Machine]
- [Source: docs/epics.md#Story-3]
- [Source: docs/sprint-artifacts/1-2-simulation-environment.md]
- [Source: docs/sprint-artifacts/1-2.5-hardware-connectivity-hello-world.md]
- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tuning Guide](https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/)
- [slam_toolbox Documentation](https://docs.ros.org/en/humble/p/slam_toolbox/)
- [slam_toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 with SLAM Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)

## Runnable Verification

```bash
# Terminal 1: Launch simulation with navigation
ros2 launch g1_bringup sim_nav_launch.py

# Terminal 2: Verify SLAM is building map
ros2 topic echo /map --once
# Should see occupancy grid data with info.resolution = 0.05

ros2 topic echo /g1/scan --once
# Should see LaserScan message with ranges array

# Terminal 3: Verify TF tree is complete
ros2 run tf2_tools view_frames
# Should show: map -> odom -> base_link -> sensor frames

# Terminal 4: Send navigation goal via CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
# Robot should navigate to goal in RViz

# Terminal 5: Check coverage
ros2 topic echo /g1/inspection/coverage --once
# Should see Float32 message with coverage percentage

# Manual test: Place obstacle in path (in sim), verify replanning
# 1. Start navigation to distant goal
# 2. Add obstacle in MuJoCo/RViz
# 3. Observe robot replan around obstacle
```

## Definition of Done

1. `ros2 launch g1_bringup sim_nav_launch.py` starts without errors
2. slam_toolbox builds occupancy grid map visible in RViz
3. `/g1/scan` topic publishes 2D LaserScan at 10Hz
4. Navigation goal via `ros2 action send_goal` moves robot to target
5. Robot avoids obstacles placed in its path
6. Path replans when blocked (visible in RViz)
7. `/g1/inspection/coverage` publishes coverage percentage
8. TF tree complete: `map` → `odom` → `base_link` → sensors
9. Unit tests pass for new nodes

## Dev Agent Record

### Context Reference

Story context from create-story workflow - Nav2 and slam_toolbox integration for autonomous navigation.

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List

**Files to Create:**
- src/g1_bringup/launch/sim_nav_launch.py
- src/g1_bringup/config/rviz/nav.rviz
- src/g1_navigation/g1_navigation/loco_bridge.py
- src/g1_navigation/g1_navigation/coverage_tracker.py
- src/g1_perception/g1_perception/lidar_to_scan.py

**Files to Update:**
- src/g1_bringup/config/nav2_params.yaml (full configuration)
- src/g1_bringup/config/slam_params.yaml (full configuration)
- src/g1_navigation/setup.py (add entry points)
- src/g1_perception/setup.py (add entry points)
- src/g1_navigation/package.xml (add nav2 dependencies)
- src/g1_perception/package.xml (add sensor_msgs dependencies)

## Change Log

- 2025-12-04: Story created by create-story workflow - comprehensive developer guide with Nav2/slam_toolbox integration details, previous story intelligence, and latest technical specifics
- 2025-12-04: Story validated and improved by validate-create-story workflow:
  - **CRITICAL FIX:** Added complete local_costmap and global_costmap configuration (was missing)
  - **CRITICAL FIX:** Added launch ordering and TF timing guidance with TimerAction example
  - **CRITICAL FIX:** Added explicit setup.py entry_points format for new nodes
  - **Enhancement:** Added Nav2 topic namespace remapping guidance
  - **Enhancement:** Added complete bt_navigator configuration with plugin list
  - **Enhancement:** Added path planning failure handling with state transitions
  - **Enhancement:** Added real robot LiDAR topic remapping with static TF
  - **Enhancement:** Added global costmap configuration for path planning
  - **Optimization:** Added troubleshooting section for common Nav2/SLAM issues
  - Validation report saved to: docs/sprint-artifacts/validation-report-1-3-2025-12-04.md
