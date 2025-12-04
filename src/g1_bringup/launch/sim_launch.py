# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Simulation launch file for G1 Inspector.

Launches MuJoCo simulation (headless) with all sensor data
visualized through RViz. Physics runs in MuJoCo, visualization in RViz.

Usage:
    ros2 launch g1_bringup sim_launch.py
    ros2 launch g1_bringup sim_launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for simulation environment."""

    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')

    # Get package share directories
    bringup_share = FindPackageShare('g1_bringup')

    # RViz config path
    rviz_config = PathJoinSubstitution([
        bringup_share, 'config', 'rviz', 'sim.rviz'
    ])

    # Simple URDF for robot_state_publisher (TF tree)
    # This provides the static transforms and visual for RViz
    # Simplified humanoid shape representing G1
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

    # Robot state publisher for TF tree visualization in RViz
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }],
    )

    # MuJoCo Simulation Manager - runs physics and publishes all sensor data
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

    # RViz visualization
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

        # Core nodes
        robot_state_publisher_node,
        mujoco_sim_node,

        # Visualization
        rviz_node,
    ])
