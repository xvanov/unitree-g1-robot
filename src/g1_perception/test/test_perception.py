# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Test suite for g1_perception package.

Tests for sensor capture, image processing, and simulated sensors.
"""

import math
import pytest
import numpy as np


class TestPerception:
    """Test cases for perception functionality."""

    def test_package_imports(self):
        """Test that g1_perception package can be imported."""
        import g1_perception
        assert g1_perception is not None


class TestSimCamera:
    """Test cases for simulated camera node."""

    def test_sim_camera_imports(self):
        """Test that sim_camera module can be imported."""
        from g1_perception import sim_camera
        assert sim_camera is not None

    def test_sim_camera_node_class_exists(self):
        """Test that SimCameraNode class exists."""
        from g1_perception.sim_camera import SimCameraNode
        assert SimCameraNode is not None

    def test_camera_defaults(self):
        """Test camera default parameters match requirements."""
        from g1_perception.sim_camera import SimCameraNode

        assert SimCameraNode.DEFAULT_WIDTH == 640
        assert SimCameraNode.DEFAULT_HEIGHT == 480
        assert SimCameraNode.DEFAULT_FPS == 1.0  # 1 Hz per AC


class TestSimLidar:
    """Test cases for simulated LiDAR node."""

    def test_sim_lidar_imports(self):
        """Test that sim_lidar module can be imported."""
        from g1_perception import sim_lidar
        assert sim_lidar is not None

    def test_sim_lidar_node_class_exists(self):
        """Test that SimLidarNode class exists."""
        from g1_perception.sim_lidar import SimLidarNode
        assert SimLidarNode is not None

    def test_lidar_defaults(self):
        """Test LiDAR default parameters match requirements."""
        from g1_perception.sim_lidar import SimLidarNode

        assert SimLidarNode.DEFAULT_FPS == 10.0  # 10 Hz per AC
        assert SimLidarNode.DEFAULT_MIN_RANGE == 0.1
        assert SimLidarNode.DEFAULT_MAX_RANGE == 40.0


class TestSimImu:
    """Test cases for simulated IMU node."""

    def test_sim_imu_imports(self):
        """Test that sim_imu module can be imported."""
        from g1_perception import sim_imu
        assert sim_imu is not None

    def test_sim_imu_node_class_exists(self):
        """Test that SimImuNode class exists."""
        from g1_perception.sim_imu import SimImuNode
        assert SimImuNode is not None

    def test_imu_defaults(self):
        """Test IMU default parameters match requirements."""
        from g1_perception.sim_imu import SimImuNode

        assert SimImuNode.DEFAULT_FPS == 100.0  # 100 Hz per AC
        assert SimImuNode.GRAVITY == 9.81


class TestImageCapture:
    """Test cases for image capture functionality."""

    def test_placeholder(self):
        """Placeholder test - implementation in Story 7."""
        # Camera node tests will be added in Story 7
        pass
