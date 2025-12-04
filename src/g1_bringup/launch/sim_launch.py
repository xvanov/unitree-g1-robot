# Copyright 2024 G1 Inspector Project
# Licensed under Apache-2.0

"""
Simulation launch file for G1 Inspector.

Launches MuJoCo simulation with fake locomotion for testing
navigation and perception without real hardware.

Usage:
    ros2 launch g1_bringup sim_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='g1',
        description='Robot namespace'
    )

    # Get package share directory
    bringup_share = FindPackageShare('g1_bringup')

    # Config file paths
    robot_params = PathJoinSubstitution([
        bringup_share, 'config', 'robot_params.yaml'
    ])

    # Placeholder nodes - will be implemented in later stories
    # Story 2: Simulation environment nodes
    # Story 3: Navigation stack nodes

    return LaunchDescription([
        use_rviz_arg,
        namespace_arg,

        # Placeholder: Nodes will be added in Story 2 (Simulation Environment)
        # - MuJoCo simulation node
        # - Fake locomotion controller
        # - Simulated sensor publishers
        # - RViz (if use_rviz=true)
    ])
