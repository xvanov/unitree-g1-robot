# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Simulated LiDAR node for G1 Inspector.

Publishes simulated point cloud data at specified rate.
Simulates Livox MID-360 LiDAR characteristics.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


class SimLidarNode(Node):
    """
    Simulated LiDAR publisher node.

    Publishes PointCloud2 to /g1/lidar/points.
    Simulates MID-360 360-degree LiDAR with configurable parameters.
    """

    # LiDAR specifications (MID-360 simulation)
    DEFAULT_FPS = 10.0          # 10 Hz as per story requirements
    DEFAULT_NUM_POINTS = 5000   # Points per scan
    DEFAULT_MIN_RANGE = 0.1     # meters
    DEFAULT_MAX_RANGE = 40.0    # meters
    DEFAULT_FOV_H = 360.0       # degrees (horizontal)
    DEFAULT_FOV_V = 59.0        # degrees (vertical, MID-360: -7 to +52)

    def __init__(self) -> None:
        super().__init__('sim_lidar')

        # Declare parameters
        self.declare_parameter('fps', self.DEFAULT_FPS)
        self.declare_parameter('num_points', self.DEFAULT_NUM_POINTS)
        self.declare_parameter('min_range', self.DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range', self.DEFAULT_MAX_RANGE)
        self.declare_parameter('frame_id', 'lidar_link')

        # Get parameters
        self.fps = self.get_parameter('fps').value
        self.num_points = self.get_parameter('num_points').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/g1/lidar/points',
            10
        )

        # Pre-compute point field definitions
        self.point_fields = self._create_point_fields()

        # Frame counter
        self.frame_count = 0

        # Timer for publishing
        publish_period = 1.0 / self.fps
        self.timer = self.create_timer(publish_period, self._publish_pointcloud)

        self.get_logger().info(
            f"[SIMULATION] SimLidarNode initialized"
            f" - Rate: {self.fps} Hz"
            f" - Points: {self.num_points}"
            f" - Range: {self.min_range}-{self.max_range}m"
        )

    def _create_point_fields(self) -> list:
        """Create PointCloud2 point field definitions."""
        fields = []

        # x, y, z coordinates (float32)
        fields.append(PointField(
            name='x',
            offset=0,
            datatype=PointField.FLOAT32,
            count=1
        ))
        fields.append(PointField(
            name='y',
            offset=4,
            datatype=PointField.FLOAT32,
            count=1
        ))
        fields.append(PointField(
            name='z',
            offset=8,
            datatype=PointField.FLOAT32,
            count=1
        ))
        # Intensity (float32)
        fields.append(PointField(
            name='intensity',
            offset=12,
            datatype=PointField.FLOAT32,
            count=1
        ))

        return fields

    def _publish_pointcloud(self) -> None:
        """Publish simulated point cloud."""
        current_time = self.get_clock().now().to_msg()

        # Generate point cloud
        msg = self._create_pointcloud(current_time)
        self.pointcloud_pub.publish(msg)

        self.frame_count += 1

    def _create_pointcloud(self, timestamp) -> PointCloud2:
        """Create simulated PointCloud2 message."""
        msg = PointCloud2()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id

        # Point structure: x, y, z, intensity (4 floats = 16 bytes)
        point_step = 16
        msg.height = 1
        msg.width = self.num_points
        msg.fields = self.point_fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * self.num_points
        msg.is_dense = True

        # Generate points simulating indoor environment
        points = self._generate_indoor_points()

        # Pack points into bytes
        data = bytearray(self.num_points * point_step)
        for i, (x, y, z, intensity) in enumerate(points):
            offset = i * point_step
            struct.pack_into('ffff', data, offset, x, y, z, intensity)

        msg.data = bytes(data)
        return msg

    def _generate_indoor_points(self) -> list:
        """
        Generate points simulating indoor environment.

        Returns:
            List of (x, y, z, intensity) tuples
        """
        points = []

        # Room dimensions (10m x 10m x 3m)
        room_size = 10.0
        room_height = 3.0

        for i in range(self.num_points):
            # Random angle distribution (360 degree horizontal, -7 to +52 vertical)
            h_angle = np.random.uniform(0, 2 * math.pi)
            v_angle = np.random.uniform(math.radians(-7), math.radians(52))

            # Direction vector
            cos_v = math.cos(v_angle)
            dx = math.cos(h_angle) * cos_v
            dy = math.sin(h_angle) * cos_v
            dz = math.sin(v_angle)

            # Calculate distance to nearest surface (simplified box room)
            distance = self._raycast_box(dx, dy, dz, room_size, room_height)

            # Add noise to distance
            distance += np.random.normal(0, 0.02)  # 2cm noise
            distance = max(self.min_range, min(self.max_range, distance))

            # Calculate point position
            x = dx * distance
            y = dy * distance
            z = dz * distance

            # Intensity based on distance (closer = brighter)
            intensity = max(0.0, 1.0 - distance / self.max_range) * 100.0
            intensity += np.random.normal(0, 5)  # Add noise

            points.append((x, y, z, intensity))

        # Add some obstacle points
        num_obstacle_points = self.num_points // 10
        obstacle_positions = [
            (3.0, 2.0, 0.5),    # Obstacle 1
            (-2.0, 3.0, 0.6),   # Obstacle 2
            (0.0, -3.0, 0.4),   # Obstacle 3
        ]

        for obs_x, obs_y, obs_z in obstacle_positions:
            for _ in range(num_obstacle_points // len(obstacle_positions)):
                # Random points on obstacle surface
                angle = np.random.uniform(0, 2 * math.pi)
                radius = np.random.uniform(0.3, 0.5)
                height = np.random.uniform(0, obs_z * 2)

                x = obs_x + radius * math.cos(angle)
                y = obs_y + radius * math.sin(angle)
                z = height

                # Check if point is within sensor range
                distance = math.sqrt(x**2 + y**2 + z**2)
                if self.min_range < distance < self.max_range:
                    intensity = max(0.0, 1.0 - distance / self.max_range) * 100.0
                    points.append((x, y, z, intensity))

        return points[:self.num_points]  # Ensure exact count

    def _raycast_box(self, dx: float, dy: float, dz: float,
                     room_size: float, room_height: float) -> float:
        """
        Simple raycast against room boundaries.

        Args:
            dx, dy, dz: Ray direction (normalized)
            room_size: Room size in x and y
            room_height: Room height in z

        Returns:
            Distance to nearest wall/floor/ceiling
        """
        # Distance to each wall plane
        distances = []

        # Floor (z = 0, assuming sensor at z ~= 0.5m)
        sensor_height = 0.5
        if dz < -0.001:
            t = -sensor_height / dz
            if t > 0:
                distances.append(t)

        # Ceiling (z = room_height)
        if dz > 0.001:
            t = (room_height - sensor_height) / dz
            if t > 0:
                distances.append(t)

        # Walls at +/- room_size/2
        half_size = room_size / 2

        if abs(dx) > 0.001:
            t1 = (half_size) / dx if dx > 0 else (-half_size) / dx
            t2 = (-half_size) / dx if dx > 0 else (half_size) / dx
            if t1 > 0:
                distances.append(t1)
            if t2 > 0:
                distances.append(t2)

        if abs(dy) > 0.001:
            t1 = (half_size) / dy if dy > 0 else (-half_size) / dy
            t2 = (-half_size) / dy if dy > 0 else (half_size) / dy
            if t1 > 0:
                distances.append(t1)
            if t2 > 0:
                distances.append(t2)

        return min(distances) if distances else self.max_range


def main(args=None):
    """Main entry point for sim_lidar node."""
    rclpy.init(args=args)

    try:
        node = SimLidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[SIMULATION] LiDAR error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
