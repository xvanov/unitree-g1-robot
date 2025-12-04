# Copyright 2024 G1 Inspector Project
# Licensed under Apache-2.0

"""
Inspection launch file for G1 Inspector.

Launches all nodes required for a full inspection run including
state machine, image capture, and defect detection pipeline.

Usage:
    ros2 launch g1_bringup inspection_launch.py
    ros2 launch g1_bringup inspection_launch.py sim:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for inspection operation."""

    # Declare launch arguments
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Run in simulation mode'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='g1',
        description='Robot namespace'
    )

    plan_id_arg = DeclareLaunchArgument(
        'plan_id',
        default_value='',
        description='Construction plan ID to use for inspection'
    )

    # Get package share directories
    bringup_share = FindPackageShare('g1_bringup')

    # Include robot or sim launch based on sim argument
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'robot_launch.py'])
        ]),
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'sim_launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    # Placeholder nodes - will be implemented in later stories
    # Story 6: Inspection state machine
    # Story 7: Image capture
    # Story 8: Defect detection

    return LaunchDescription([
        sim_arg,
        namespace_arg,
        plan_id_arg,

        # Include base launch (robot or sim)
        robot_launch,
        sim_launch,

        # Placeholder: Inspection-specific nodes added in later stories
        # - Inspection state machine (Story 6)
        # - Image capture node (Story 7)
        # - Plan correlator (Story 7)
    ])
