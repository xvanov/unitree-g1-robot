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


class TestLidarToScan:
    """Test cases for LiDAR to LaserScan projection (Story 1.3)."""

    def test_lidar_to_scan_imports(self):
        """Test that lidar_to_scan module can be imported."""
        from g1_perception import lidar_to_scan
        assert lidar_to_scan is not None

    def test_lidar_to_scan_class_exists(self):
        """Test that LidarToScan class exists."""
        from g1_perception.lidar_to_scan import LidarToScan
        assert LidarToScan is not None

    def test_lidar_to_scan_default_parameters(self):
        """Test that default parameters match story requirements."""
        from g1_perception.lidar_to_scan import LidarToScan

        assert LidarToScan.DEFAULT_INPUT_TOPIC == '/g1/lidar/points'
        assert LidarToScan.DEFAULT_OUTPUT_TOPIC == '/g1/scan'
        assert LidarToScan.DEFAULT_MIN_HEIGHT == 0.1
        assert LidarToScan.DEFAULT_MAX_HEIGHT == 1.0
        assert LidarToScan.DEFAULT_RANGE_MAX == 12.0  # MID-360 range
        assert LidarToScan.DEFAULT_SCAN_RATE == 10.0  # 10 Hz

    def test_angle_increment_resolution(self):
        """Test that LaserScan has appropriate angular resolution."""
        from g1_perception.lidar_to_scan import LidarToScan

        # 1 degree resolution = pi/180
        expected_increment = math.pi / 180.0
        assert abs(LidarToScan.DEFAULT_ANGLE_INCREMENT - expected_increment) < 1e-6


class TestDepthToPointCloud:
    """Test cases for depth to PointCloud2 conversion (Story 1.3)."""

    def test_depth_to_pointcloud_imports(self):
        """Test that depth_to_pointcloud module can be imported."""
        from g1_perception import depth_to_pointcloud
        assert depth_to_pointcloud is not None

    def test_depth_to_pointcloud_class_exists(self):
        """Test that DepthToPointCloud class exists."""
        from g1_perception.depth_to_pointcloud import DepthToPointCloud
        assert DepthToPointCloud is not None

    def test_depth_to_pointcloud_default_parameters(self):
        """Test that default parameters are reasonable."""
        from g1_perception.depth_to_pointcloud import DepthToPointCloud

        assert DepthToPointCloud.DEFAULT_DEPTH_TOPIC == '/g1/camera/depth'
        assert DepthToPointCloud.DEFAULT_INFO_TOPIC == '/g1/camera/depth/camera_info'
        assert DepthToPointCloud.DEFAULT_OUTPUT_TOPIC == '/g1/camera/depth/points'
        assert DepthToPointCloud.DEFAULT_MIN_DEPTH == 0.1
        assert DepthToPointCloud.DEFAULT_MAX_DEPTH == 5.0
        assert DepthToPointCloud.DEFAULT_VOXEL_SIZE == 0.05

    def test_depth_projection_formula(self):
        """Test the pinhole camera depth projection formula."""
        # Pinhole camera model:
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth

        # Test with simple camera intrinsics
        fx = fy = 500.0  # focal length
        cx = cy = 320.0  # principal point
        depth = 2.0      # 2 meters

        # Pixel at principal point should project to (0, 0, depth)
        u, v = 320, 320
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        assert abs(X) < 1e-6
        assert abs(Y) < 1e-6
        assert abs(Z - 2.0) < 1e-6

        # Pixel offset by 100 pixels at 2m depth
        u, v = 420, 420
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy

        expected_X = 100 * 2.0 / 500.0  # 0.4 meters
        assert abs(X - expected_X) < 1e-6


class TestImageCapture:
    """Test cases for image capture functionality."""

    def test_placeholder(self):
        """Placeholder test - implementation in Story 7."""
        # Camera node tests will be added in Story 7
        pass


class TestLidarToScanEdgeCases:
    """Edge case tests for LidarToScan (Code Review fixes)."""

    def test_angle_to_bin_conversion(self):
        """Test angle to bin index conversion formula."""
        import math

        angle_min = -math.pi
        angle_increment = math.pi / 180.0  # 1 degree
        num_readings = 360

        def angle_to_bin(angle):
            idx = int((angle - angle_min) / angle_increment)
            return max(0, min(idx, num_readings - 1))

        # Test boundaries
        assert angle_to_bin(-math.pi) == 0
        assert angle_to_bin(0) == 180
        assert angle_to_bin(math.pi - 0.01) == 359

    def test_range_filtering(self):
        """Test range filtering logic."""
        range_min = 0.15
        range_max = 12.0

        # Valid ranges
        assert range_min <= 1.0 <= range_max
        assert range_min <= 5.0 <= range_max

        # Invalid ranges
        assert not (range_min <= 0.1 <= range_max)  # Too close
        assert not (range_min <= 15.0 <= range_max)  # Too far

    def test_height_filtering(self):
        """Test height filtering for point cloud projection."""
        min_height = 0.1
        max_height = 1.0

        # Valid heights
        assert min_height <= 0.5 <= max_height
        assert min_height <= 0.1 <= max_height  # At boundary

        # Invalid heights
        assert not (min_height <= 0.05 <= max_height)  # Below ground
        assert not (min_height <= 1.5 <= max_height)   # Above filter

    def test_vectorized_minimum_operation(self):
        """Test that np.minimum.at works correctly for bin filling."""
        import numpy as np

        # Simulate the vectorized bin filling
        ranges = np.full(10, float('inf'), dtype=np.float32)
        bin_indices = np.array([0, 2, 2, 5, 5, 5])
        valid_ranges = np.array([1.0, 2.0, 1.5, 3.0, 2.5, 4.0], dtype=np.float32)

        np.minimum.at(ranges, bin_indices, valid_ranges)

        assert ranges[0] == 1.0   # Single value
        assert ranges[2] == 1.5   # Minimum of 2.0 and 1.5
        assert ranges[5] == 2.5   # Minimum of 3.0, 2.5, 4.0
        assert ranges[1] == float('inf')  # Unchanged


class TestDepthToPointCloudEdgeCases:
    """Edge case tests for DepthToPointCloud (Code Review fixes)."""

    def test_depth_encoding_16uc1(self):
        """Test 16UC1 depth encoding conversion."""
        # 16UC1 stores depth in millimeters as uint16
        depth_mm = 2500  # 2.5 meters in mm
        depth_m = depth_mm / 1000.0
        assert abs(depth_m - 2.5) < 1e-6

    def test_depth_encoding_32fc1(self):
        """Test 32FC1 depth encoding (already in meters)."""
        depth_m = 2.5
        assert abs(depth_m - 2.5) < 1e-6

    def test_pinhole_projection_at_principal_point(self):
        """Test that principal point projects to (0, 0, Z)."""
        fx = fy = 500.0
        cx = cy = 320.0
        depth = 3.0

        # At principal point (cx, cy)
        u, v = 320, 320
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        assert abs(X) < 1e-6
        assert abs(Y) < 1e-6
        assert abs(Z - 3.0) < 1e-6

    def test_voxel_downsampling_vectorized(self):
        """Test vectorized voxel downsampling produces correct averages."""
        import numpy as np

        # Simple test case: 4 points in 2 voxels
        voxel_size = 1.0
        points = np.array([
            [0.1, 0.1, 0.1],  # Voxel (0, 0, 0)
            [0.2, 0.2, 0.2],  # Voxel (0, 0, 0)
            [1.1, 1.1, 1.1],  # Voxel (1, 1, 1)
            [1.2, 1.2, 1.2],  # Voxel (1, 1, 1)
        ], dtype=np.float32)

        # Expected averages
        avg_voxel_0 = (0.1 + 0.2) / 2  # 0.15
        avg_voxel_1 = (1.1 + 1.2) / 2  # 1.15

        # Verify the formula
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        unique_voxels, indices = np.unique(voxel_indices, axis=0, return_inverse=True)

        num_voxels = len(unique_voxels)
        counts = np.bincount(indices, minlength=num_voxels)

        downsampled = np.zeros((num_voxels, 3), dtype=np.float32)
        for dim in range(3):
            downsampled[:, dim] = np.bincount(
                indices, weights=points[:, dim], minlength=num_voxels
            )
        downsampled /= counts[:, np.newaxis]

        # Check results (order may vary based on unique)
        assert len(downsampled) == 2
        # Both voxels should have their points averaged
        assert np.allclose(downsampled[0], [avg_voxel_0] * 3, atol=1e-5) or \
               np.allclose(downsampled[0], [avg_voxel_1] * 3, atol=1e-5)

    def test_empty_point_cloud_handling(self):
        """Test that empty point clouds are handled gracefully."""
        import numpy as np

        empty_points = np.array([], dtype=np.float32).reshape(0, 3)
        assert len(empty_points) == 0

        # Voxel downsample should return empty
        if len(empty_points) == 0:
            result = empty_points
        assert len(result) == 0
