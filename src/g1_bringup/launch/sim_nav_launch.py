# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Simulation with Navigation launch file for G1 Inspector.

Story 1.3: Navigation Stack Integration
Launches MuJoCo simulation with Nav2 and slam_toolbox for autonomous navigation.

Usage:
    ros2 launch g1_bringup sim_nav_launch.py
    ros2 launch g1_bringup sim_nav_launch.py use_rviz:=false
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for simulation with navigation."""

    # Get package directories
    bringup_share = FindPackageShare('g1_bringup')
    bringup_dir = get_package_share_directory('g1_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Get absolute paths for config files (needed for IncludeLaunchDescription)
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_params.yaml')

    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Config file paths (rviz_config uses substitution for Node parameter)
    rviz_config = PathJoinSubstitution([
        bringup_share, 'config', 'rviz', 'nav.rviz'
    ])

    # Simple URDF for robot_state_publisher (TF tree)
    # Same as sim_launch.py - provides static transforms
    robot_description = """<?xml version="1.0"?>
<robot name="g1_sim">
  <!-- Pelvis / Base -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.25 0.20 0.15"/></geometry>
      <material name="dark_blue"><color rgba="0.1 0.1 0.4 1"/></material>
    </visual>
  </link>

  <!-- Torso -->
  <link name="torso_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry><box size="0.22 0.18 0.30"/></geometry>
      <material name="blue"><color rgba="0.2 0.2 0.7 1"/></material>
    </visual>
  </link>
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head_link">
    <visual>
      <origin xyz="0 0 0.08" rpy="0 0 0"/>
      <geometry><sphere radius="0.10"/></geometry>
      <material name="silver"><color rgba="0.7 0.7 0.8 1"/></material>
    </visual>
  </link>
  <joint name="head_joint" type="fixed">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_arm_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry><cylinder radius="0.04" length="0.30"/></geometry>
      <material name="arm_blue"><color rgba="0.3 0.3 0.6 1"/></material>
    </visual>
  </link>
  <joint name="left_arm_joint" type="fixed">
    <parent link="torso_link"/>
    <child link="left_arm_link"/>
    <origin xyz="0 0.14 0.25" rpy="0 0.2 0"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_arm_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry><cylinder radius="0.04" length="0.30"/></geometry>
      <material name="arm_blue"><color rgba="0.3 0.3 0.6 1"/></material>
    </visual>
  </link>
  <joint name="right_arm_joint" type="fixed">
    <parent link="torso_link"/>
    <child link="right_arm_link"/>
    <origin xyz="0 -0.14 0.25" rpy="0 0.2 0"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_leg_link">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry><cylinder radius="0.05" length="0.50"/></geometry>
      <material name="leg_blue"><color rgba="0.15 0.15 0.5 1"/></material>
    </visual>
  </link>
  <joint name="left_leg_joint" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg_link"/>
    <origin xyz="0 0.08 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_leg_link">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry><cylinder radius="0.05" length="0.50"/></geometry>
      <material name="leg_blue"><color rgba="0.15 0.15 0.5 1"/></material>
    </visual>
  </link>
  <joint name="right_leg_joint" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg_link"/>
    <origin xyz="0 -0.08 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Sensors -->
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry><cylinder radius="0.05" length="0.04"/></geometry>
      <material name="lidar_gray"><color rgba="0.3 0.3 0.3 1"/></material>
    </visual>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="head_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry><box size="0.03 0.10 0.03"/></geometry>
      <material name="camera_black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="head_link"/>
    <child link="camera_link"/>
    <origin xyz="0.08 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="camera_rgb_frame"/>
  <joint name="camera_rgb_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_rgb_frame"/>
    <origin xyz="0.02 0 0" rpy="0 0 0"/>
  </joint>

  <link name="camera_depth_frame"/>
  <joint name="camera_depth_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_depth_frame"/>
    <origin xyz="0.02 0 0" rpy="0 0 0"/>
  </joint>
</robot>"""

    # ========================================
    # PHASE 1: Core Simulation (starts immediately)
    # ========================================

    # MuJoCo Simulation Manager - MUST start first to publish /clock
    # This node publishes /clock immediately before MuJoCo init
    mujoco_sim_node = Node(
        package='g1_bringup',
        executable='mujoco_sim',
        name='mujoco_sim_manager',
        output='screen',
        parameters=[{
            'physics_dt': 0.002,
            'use_g1_model': True,
        }],
    )

    # Robot state publisher for TF tree
    # Explicitly uses sim_time and delayed slightly to ensure /clock is available
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
            'use_sim_time': True,  # Explicit - ensures TF uses /clock timestamps
        }],
    )

    # Delay robot_state_publisher to ensure /clock is publishing first
    # MuJoCo init takes ~5-8 seconds for EGL renderer
    robot_state_publisher_delayed = TimerAction(
        period=8.0,  # Wait for MuJoCo to fully initialize
        actions=[robot_state_publisher_node],
    )

    # LiDAR to LaserScan projection (for slam_toolbox)
    # Delayed to ensure MuJoCo is fully initialized and /clock is stable
    lidar_to_scan_node = Node(
        package='g1_perception',
        executable='lidar_to_scan',
        name='lidar_to_scan',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Explicit for simulation
            'input_topic': '/g1/lidar/points',
            'output_topic': '/g1/scan',
            'min_height': 0.1,
            'max_height': 1.0,
            'range_min': 0.15,
            'range_max': 12.0,
            'scan_rate': 10.0,
            'output_frame': 'lidar_link',
        }],
    )

    lidar_to_scan_delayed = TimerAction(
        period=8.0,  # Start after MuJoCo is fully initialized
        actions=[lidar_to_scan_node],
    )

    # Depth to PointCloud conversion (for Nav2 costmap)
    # Delayed start to ensure MuJoCo is initialized and camera_info is available
    depth_to_pointcloud_node = Node(
        package='g1_perception',
        executable='depth_to_pointcloud',
        name='depth_to_pointcloud',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Explicit for simulation
            'depth_topic': '/g1/camera/depth',
            'camera_info_topic': '/g1/camera/depth/camera_info',
            'output_topic': '/g1/camera/depth/points',
            'min_depth': 0.1,
            'max_depth': 5.0,
            'output_frame': 'camera_depth_frame',
        }],
    )

    depth_to_pointcloud_delayed = TimerAction(
        period=8.0,  # Wait for MuJoCo to be fully initialized
        actions=[depth_to_pointcloud_node],
    )

    # ========================================
    # PHASE 2: SLAM (starts after 1 second)
    # ========================================

    # slam_toolbox for mapping
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # SLAM needs /clock and TF to be stable first
    # MuJoCo init takes ~5-8 seconds, then robot_state_publisher starts at 8s
    slam_delayed = TimerAction(
        period=10.0,  # Start after robot_state_publisher is up
        actions=[slam_launch],
    )

    # ========================================
    # PHASE 3: Navigation (starts after 3 seconds)
    # Delay ensures TF tree is complete: map -> odom -> base_link
    # ========================================

    # Nav2 navigation (without map_server since we're using SLAM)
    # Use navigation_launch.py instead of bringup_launch.py to skip map_server/amcl
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'False',  # Disable composition for better debugging
        }.items(),
    )

    # Nav2 needs SLAM to be running first (for map -> odom TF)
    nav2_delayed = TimerAction(
        period=12.0,  # Start after SLAM is up
        actions=[nav2_launch],
    )

    # Loco Bridge (Nav2 cmd_vel to simulation)
    loco_bridge_node = Node(
        package='g1_navigation',
        executable='loco_bridge',
        name='loco_bridge',
        output='screen',
        parameters=[{
            'use_simulation': True,
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'cmd_vel_timeout': 0.5,
        }],
    )

    loco_bridge_delayed = TimerAction(
        period=12.0,  # Start with Nav2
        actions=[loco_bridge_node],
    )

    # Coverage tracker
    coverage_tracker_node = Node(
        package='g1_navigation',
        executable='coverage_tracker',
        name='coverage_tracker',
        output='screen',
        parameters=[{
            'coverage_cell_size': 0.5,
            'sensor_fov': 87.0,
            'sensor_range': 5.0,
            'publish_rate': 1.0,
        }],
    )

    coverage_tracker_delayed = TimerAction(
        period=12.0,  # Start with Nav2
        actions=[coverage_tracker_node],
    )

    # ========================================
    # Visualization (RViz)
    # ========================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        use_sim_time_arg,

        # Global use_sim_time setting
        SetParameter(name='use_sim_time', value=use_sim_time),

        # Phase 1: Core simulation
        # MuJoCo must start FIRST to publish /clock before any other node
        # All other nodes are delayed to ensure /clock is stable before they start
        mujoco_sim_node,
        robot_state_publisher_delayed,  # 8s delay - after MuJoCo init
        lidar_to_scan_delayed,          # 8s delay - after MuJoCo init
        depth_to_pointcloud_delayed,    # 8s delay - after MuJoCo init

        # Phase 2: SLAM (1.5 second delay - adjusted for robot_state_publisher delay)
        slam_delayed,

        # Phase 3: Navigation (3 second delay for TF to be ready)
        nav2_delayed,
        loco_bridge_delayed,
        coverage_tracker_delayed,

        # Visualization
        rviz_node,
    ])
