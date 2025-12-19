# Unitree G1 Robot - Construction Site Inspector

ROS2-based autonomous inspection system for the Unitree G1 EDU humanoid robot.

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble Hawksbill
- CycloneDDS middleware

### Build

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_HOME=/usr/local
cd ~/unitree-g1-robot
colcon build
source install/setup.bash
```

### Run Simulation

Launch the simulation environment:

```bash
ros2 launch g1_bringup sim_launch.py
```

Launch options:
- `use_rviz:=true/false` - Enable/disable RViz visualization (default: true)

### Teleop Control

Control the simulated robot with keyboard:

```bash
# In a separate terminal (after sourcing workspace)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/g1/cmd_vel
```

Use the following keys to control the robot:
- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `u` / `o` - Move forward while turning
- `m` / `.` - Move backward while turning

### Verify Simulation

Check that all topics are publishing:

```bash
# List all G1 topics
ros2 topic list | grep g1

# Check sensor rates
ros2 topic hz /g1/camera/rgb      # Should be ~1 Hz
ros2 topic hz /g1/lidar/points    # Should be ~10 Hz
ros2 topic hz /g1/imu/data        # Should be ~100 Hz
ros2 topic hz /g1/odom            # Should be ~50 Hz

# View TF tree
ros2 run tf2_tools view_frames
```

## Package Structure

- `g1_bringup` - Launch files and configuration
- `g1_navigation` - Navigation stack and locomotion control
- `g1_perception` - Sensor processing and computer vision
- `g1_inspection` - Inspection state machine and defect detection
- `g1_safety` - Safety monitoring and emergency stop
- `g1_interfaces` - Custom ROS2 messages, services, and actions

## Topics

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/g1/cmd_vel` | geometry_msgs/Twist | Input | Velocity commands |
| `/g1/odom` | nav_msgs/Odometry | 50 Hz | Robot odometry |
| `/g1/camera/rgb` | sensor_msgs/Image | 1 Hz | RGB camera |
| `/g1/camera/depth` | sensor_msgs/Image | 1 Hz | Depth camera |
| `/g1/lidar/points` | sensor_msgs/PointCloud2 | 10 Hz | LiDAR point cloud |
| `/g1/imu/data` | sensor_msgs/Imu | 100 Hz | IMU data |

## Dual-Environment Development (Story 1-6)

This project supports building natively on both Docker (dev machine) and Jetson (robot).

### Docker Build (Dev Machine)

```bash
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm dev
# Inside container:
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Native Build on Robot (Jetson)

```bash
# 1. Deploy to robot
./scripts/deploy-to-robot.sh --build

# 2. Or SSH in and build manually
ssh unitree@192.168.123.164
cd g1_inspector
./scripts/setup-robot.sh  # First time only - installs dependencies
mkdir -p build && cd build
cmake .. -DROBOT_BUILD=ON
make -j4
```

### Quick Deploy Workflow

```bash
# Deploy, build, and run greeter demo
./scripts/deploy-to-robot.sh --build --run

# Deploy with different robot IP
ROBOT_IP=192.168.123.233 ./scripts/deploy-to-robot.sh --build

# Quick SSH access
./scripts/robot-shell.sh
```

### Environment Setup (Robot)

```bash
# Add to ~/.bashrc on robot
source ~/g1_inspector/config/robot.env
```

See `docs/jetson-setup.md` for detailed Jetson configuration.

## License

Apache-2.0
