#!/usr/bin/env python3
"""
2D LaserScan Projection Node for G1 Inspector.

Story 1.3: Navigation Stack Integration
Converts 3D PointCloud2 from MID-360 LiDAR to 2D LaserScan for slam_toolbox.

Subscribes: /g1/lidar/points (sensor_msgs/PointCloud2)
Publishes: /g1/scan (sensor_msgs/LaserScan)
"""

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2


class LidarToScan(Node):
    """Projects 3D PointCloud2 to 2D LaserScan for SLAM."""

    # Default parameters
    DEFAULT_INPUT_TOPIC = '/g1/lidar/points'
    DEFAULT_OUTPUT_TOPIC = '/g1/scan'
    DEFAULT_MIN_HEIGHT = 0.1   # meters above ground
    DEFAULT_MAX_HEIGHT = 1.0   # meters above ground
    DEFAULT_RANGE_MIN = 0.15   # minimum valid range
    DEFAULT_RANGE_MAX = 12.0   # MID-360 max range
    DEFAULT_SCAN_RATE = 10.0   # Hz
    DEFAULT_ANGLE_MIN = -math.pi
    DEFAULT_ANGLE_MAX = math.pi
    DEFAULT_ANGLE_INCREMENT = math.pi / 180.0  # 1 degree resolution

    def __init__(self) -> None:
        """Initialize the LidarToScan node."""
        super().__init__('lidar_to_scan')

        # Declare parameters
        self.declare_parameter('input_topic', self.DEFAULT_INPUT_TOPIC)
        self.declare_parameter('output_topic', self.DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('min_height', self.DEFAULT_MIN_HEIGHT)
        self.declare_parameter('max_height', self.DEFAULT_MAX_HEIGHT)
        self.declare_parameter('range_min', self.DEFAULT_RANGE_MIN)
        self.declare_parameter('range_max', self.DEFAULT_RANGE_MAX)
        self.declare_parameter('scan_rate', self.DEFAULT_SCAN_RATE)
        self.declare_parameter('output_frame', 'lidar_link')
        # Note: use_sim_time is automatically declared by ROS2 when set globally

        # Get parameters
        self._input_topic = self.get_parameter('input_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._min_height = self.get_parameter('min_height').value
        self._max_height = self.get_parameter('max_height').value
        self._range_min = self.get_parameter('range_min').value
        self._range_max = self.get_parameter('range_max').value
        self._scan_rate = self.get_parameter('scan_rate').value
        self._output_frame = self.get_parameter('output_frame').value

        # LaserScan configuration
        self._angle_min = self.DEFAULT_ANGLE_MIN
        self._angle_max = self.DEFAULT_ANGLE_MAX
        self._angle_increment = self.DEFAULT_ANGLE_INCREMENT
        self._num_readings = int((self._angle_max - self._angle_min) / self._angle_increment)
        self._scan_time = 1.0 / self._scan_rate

        # QoS for sensor data (input - best effort for real sensor data)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for LaserScan output (RELIABLE for slam_toolbox compatibility)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber to PointCloud2
        self._pointcloud_sub = self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._pointcloud_callback,
            sensor_qos
        )

        # Publisher for LaserScan (RELIABLE for slam_toolbox)
        self._scan_pub = self.create_publisher(
            LaserScan,
            self._output_topic,
            scan_qos
        )

        # Rate limiting
        self._last_publish_time: Optional[float] = None
        self._min_publish_interval = 1.0 / self._scan_rate

        # Validate output frame parameter
        if not self._output_frame or self._output_frame.strip() == '':
            self.get_logger().warn(
                "[PERCEPTION] Empty output_frame parameter, defaulting to 'lidar_link'"
            )
            self._output_frame = 'lidar_link'

        self.get_logger().info(
            f"[PERCEPTION] LidarToScan initialized: "
            f"{self._input_topic} -> {self._output_topic}"
        )
        self.get_logger().info(
            f"[PERCEPTION] Height filter: {self._min_height}m - {self._max_height}m, "
            f"Range: {self._range_min}m - {self._range_max}m, "
            f"Frame: {self._output_frame}"
        )

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Convert PointCloud2 to LaserScan.

        Filters points within height range and projects to 2D plane.

        Args:
            msg: Input PointCloud2 message
        """
        # Rate limiting
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self._last_publish_time is not None:
            if (current_time - self._last_publish_time) < self._min_publish_interval:
                return

        try:
            # Extract points from PointCloud2
            points = self._extract_points(msg)
            if points is None or len(points) == 0:
                self.get_logger().debug("[PERCEPTION] No valid points in cloud")
                return

            # Filter by height
            height_mask = (points[:, 2] >= self._min_height) & (points[:, 2] <= self._max_height)
            filtered_points = points[height_mask]

            if len(filtered_points) == 0:
                self.get_logger().debug("[PERCEPTION] No points in height range")
                return

            # Project to 2D and create LaserScan
            scan_msg = self._create_laserscan(msg.header, filtered_points)
            self._scan_pub.publish(scan_msg)
            self._last_publish_time = current_time

        except Exception as e:
            self.get_logger().error(f"[PERCEPTION] Point cloud conversion error: {e}")

    def _extract_points(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """
        Extract XYZ points from PointCloud2 message.

        Args:
            msg: Input PointCloud2 message

        Returns:
            Numpy array of shape (N, 3) with XYZ coordinates, or None if empty
        """
        try:
            # Use sensor_msgs_py for efficient extraction
            points_gen = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )

            # Convert generator to structured numpy array
            points_structured = np.array(list(points_gen))

            if points_structured.size == 0:
                return None

            # Handle structured arrays (with named fields x, y, z)
            if points_structured.dtype.names:
                # Convert structured array to regular (N, 3) array
                points = np.column_stack([
                    points_structured['x'],
                    points_structured['y'],
                    points_structured['z']
                ]).astype(np.float32)
            else:
                # Already a regular array
                points = points_structured.astype(np.float32)

            return points

        except Exception as e:
            self.get_logger().warn(f"[PERCEPTION] Failed to extract points: {e}")
            return None

    def _create_laserscan(self, header, points: np.ndarray) -> LaserScan:
        """
        Create LaserScan message from 2D projected points.

        Args:
            header: Header from original PointCloud2
            points: Filtered 3D points to project

        Returns:
            LaserScan message
        """
        scan = LaserScan()

        # Header with our output frame
        scan.header.stamp = header.stamp
        scan.header.frame_id = self._output_frame

        # Scan parameters
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._angle_increment
        scan.time_increment = self._scan_time / self._num_readings
        scan.scan_time = self._scan_time
        scan.range_min = self._range_min
        scan.range_max = self._range_max

        # Initialize ranges to infinity (no detection)
        ranges = np.full(self._num_readings, float('inf'), dtype=np.float32)

        # Project points to 2D polar coordinates
        x = points[:, 0]
        y = points[:, 1]

        # Calculate range and angle for each point
        point_ranges = np.sqrt(x**2 + y**2)
        point_angles = np.arctan2(y, x)

        # Filter valid ranges
        valid_mask = (point_ranges >= self._range_min) & (point_ranges <= self._range_max)
        valid_ranges = point_ranges[valid_mask]
        valid_angles = point_angles[valid_mask]

        # Convert angles to bin indices
        # Normalize angles to [angle_min, angle_max)
        # Use floor to handle edge case where angle == angle_max maps to last valid bin
        bin_indices = np.floor(
            (valid_angles - self._angle_min) / self._angle_increment
        ).astype(int)

        # Clamp to valid range (handles floating point edge cases)
        bin_indices = np.clip(bin_indices, 0, self._num_readings - 1)

        # Fill in ranges (keep minimum range for each bin) - vectorized
        # np.minimum.at performs unbuffered in-place minimum operation
        np.minimum.at(ranges, bin_indices, valid_ranges)

        scan.ranges = ranges.tolist()

        # Empty intensities (not needed for SLAM)
        scan.intensities = []

        return scan


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)

    node = LidarToScan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
