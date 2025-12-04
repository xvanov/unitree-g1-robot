# Copyright 2024 G1 Inspector Project
# Licensed under Apache-2.0

"""
Robot launch file for G1 Inspector on real hardware.

Launches all nodes for real Unitree G1 robot operation including
SDK bridge, navigation, perception, and safety systems.

Usage:
    ros2 launch g1_bringup robot_launch.py
    ros2 launch g1_bringup robot_launch.py robot_ip:=192.168.123.164
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for real robot operation."""

    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.164',
        description='IP address of Unitree G1 robot'
    )

    lidar_ip_arg = DeclareLaunchArgument(
        'lidar_ip',
        default_value='192.168.123.120',
        description='IP address of Livox MID-360 LiDAR'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='g1',
        description='Robot namespace'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz visualization (usually false on robot)'
    )

    # Get package share directory
    bringup_share = FindPackageShare('g1_bringup')

    # Config file paths
    robot_params = PathJoinSubstitution([
        bringup_share, 'config', 'robot_params.yaml'
    ])
    nav2_params = PathJoinSubstitution([
        bringup_share, 'config', 'nav2_params.yaml'
    ])
    slam_params = PathJoinSubstitution([
        bringup_share, 'config', 'slam_params.yaml'
    ])

    # Placeholder nodes - will be implemented in later stories
    # Story 2.5: Hardware connectivity
    # Story 3: Navigation stack
    # Story 4: Safety systems

    return LaunchDescription([
        robot_ip_arg,
        lidar_ip_arg,
        namespace_arg,
        use_rviz_arg,

        # Placeholder: Nodes will be added in subsequent stories
        # - Nav2 LocoClient bridge (Story 3)
        # - SLAM toolbox (Story 3)
        # - Safety node (Story 4)
        # - Perception nodes (Story 7)
    ])
