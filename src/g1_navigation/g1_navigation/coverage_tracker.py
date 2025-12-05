#!/usr/bin/env python3
"""
Coverage Tracker Node for G1 Inspector.

Story 1.3: Navigation Stack Integration
Tracks inspection coverage by monitoring robot pose and map data.

Subscribes: /map (nav_msgs/OccupancyGrid)
            /g1/odom (nav_msgs/Odometry)
Publishes: /g1/inspection/coverage (std_msgs/Float32)
"""

from typing import Optional, Set, Tuple
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32


class CoverageTracker(Node):
    """Tracks and publishes inspection coverage percentage."""

    # Default parameters
    DEFAULT_COVERAGE_CELL_SIZE = 0.5   # meters per coverage cell
    DEFAULT_SENSOR_FOV = 87.0          # degrees (D435i horizontal FOV)
    DEFAULT_SENSOR_RANGE = 5.0         # meters (effective coverage range)
    DEFAULT_PUBLISH_RATE = 1.0         # Hz
    DEFAULT_CAMERA_YAW_OFFSET = 0.0    # degrees (camera yaw offset from robot base)

    def __init__(self) -> None:
        """Initialize the CoverageTracker node."""
        super().__init__('coverage_tracker')

        # Declare parameters
        self.declare_parameter('coverage_cell_size', self.DEFAULT_COVERAGE_CELL_SIZE)
        self.declare_parameter('sensor_fov', self.DEFAULT_SENSOR_FOV)
        self.declare_parameter('sensor_range', self.DEFAULT_SENSOR_RANGE)
        self.declare_parameter('publish_rate', self.DEFAULT_PUBLISH_RATE)
        self.declare_parameter('camera_yaw_offset', self.DEFAULT_CAMERA_YAW_OFFSET)
        # Note: use_sim_time is automatically declared by ROS2 when set globally

        # Get parameters
        self._cell_size = self.get_parameter('coverage_cell_size').value
        self._sensor_fov = math.radians(self.get_parameter('sensor_fov').value)
        self._sensor_range = self.get_parameter('sensor_range').value
        self._publish_rate = self.get_parameter('publish_rate').value
        self._camera_yaw_offset = math.radians(
            self.get_parameter('camera_yaw_offset').value
        )

        # Map data
        self._map: Optional[OccupancyGrid] = None
        self._map_resolution: float = 0.05
        self._map_origin_x: float = 0.0
        self._map_origin_y: float = 0.0
        self._map_width: int = 0
        self._map_height: int = 0
        self._free_cells: Set[Tuple[int, int]] = set()  # Cells that can be covered
        self._total_free_cells: int = 0

        # Coverage tracking
        self._visited_cells: Set[Tuple[int, int]] = set()  # Cells that have been covered
        self._current_coverage: float = 0.0

        # Thread safety for map/coverage data and robot pose
        self._map_lock = threading.Lock()
        self._pose_lock = threading.Lock()

        # Robot pose (protected by _pose_lock)
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Map uses transient local for latched behavior
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribers
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos
        )

        self._odom_sub = self.create_subscription(
            Odometry,
            '/g1/odom',
            self._odom_callback,
            sensor_qos
        )

        # Publisher for coverage percentage
        self._coverage_pub = self.create_publisher(
            Float32,
            '/g1/inspection/coverage',
            10
        )

        # Timer for publishing coverage
        publish_period = 1.0 / self._publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_coverage)

        self.get_logger().info(
            f"[NAVIGATION] CoverageTracker initialized: "
            f"cell_size={self._cell_size}m, FOV={math.degrees(self._sensor_fov):.0f}°, "
            f"range={self._sensor_range}m, camera_yaw_offset={math.degrees(self._camera_yaw_offset):.1f}°"
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """
        Handle map updates from slam_toolbox.

        Args:
            msg: OccupancyGrid message
        """
        with self._map_lock:
            self._map = msg
            self._map_resolution = msg.info.resolution
            self._map_origin_x = msg.info.origin.position.x
            self._map_origin_y = msg.info.origin.position.y
            self._map_width = msg.info.width
            self._map_height = msg.info.height

            # Find all free cells in the map
            self._update_free_cells(msg)

            self.get_logger().debug(
                f"[NAVIGATION] Map updated: {self._map_width}x{self._map_height}, "
                f"{self._total_free_cells} free cells"
            )

    def _update_free_cells(self, msg: OccupancyGrid) -> None:
        """
        Update the set of free cells that can be covered.

        Free cells have occupancy value 0 (known free).
        Unknown cells (-1) and occupied cells (>0) are excluded.

        Note: When map is significantly updated (new SLAM session), visited cells
        are reset to avoid stale coverage data.

        Args:
            msg: OccupancyGrid message
        """
        old_free_count = len(self._free_cells)
        self._free_cells.clear()

        # Convert to numpy for efficient processing
        data = np.array(msg.data, dtype=np.int8).reshape(
            (self._map_height, self._map_width)
        )

        # Find free cells (occupancy = 0)
        free_indices = np.where(data == 0)

        # Convert map cells to coverage cells
        for row, col in zip(free_indices[0], free_indices[1]):
            # Convert map cell to world coordinates
            world_x = col * self._map_resolution + self._map_origin_x
            world_y = row * self._map_resolution + self._map_origin_y

            # Convert to coverage cell
            coverage_cell = self._world_to_coverage_cell(world_x, world_y)
            self._free_cells.add(coverage_cell)

        self._total_free_cells = len(self._free_cells)

        # Detect significant map change (new SLAM session or map reset)
        # If free cell count changed by more than 50%, reset visited cells
        if old_free_count > 0:
            change_ratio = abs(self._total_free_cells - old_free_count) / old_free_count
            if change_ratio > 0.5:
                self.get_logger().info(
                    f"[NAVIGATION] Significant map change detected "
                    f"({old_free_count} -> {self._total_free_cells} free cells). "
                    "Resetting coverage tracking."
                )
                self._visited_cells.clear()
                self._current_coverage = 0.0

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Handle odometry updates and track coverage.

        Args:
            msg: Odometry message
        """
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Update pose with lock protection
        with self._pose_lock:
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y
            self._robot_yaw = yaw

        # Update coverage based on current pose
        self._update_coverage()

    def _update_coverage(self) -> None:
        """Update coverage based on current robot pose and sensor FOV."""
        # Snapshot pose with lock to avoid race condition
        with self._pose_lock:
            robot_x = self._robot_x
            robot_y = self._robot_y
            robot_yaw = self._robot_yaw

        with self._map_lock:
            if self._map is None:
                return

            # Calculate cells visible from current pose
            # Apply camera yaw offset relative to robot base
            sensor_yaw = robot_yaw + self._camera_yaw_offset
            visible_cells = self._calculate_visible_cells(robot_x, robot_y, sensor_yaw)

            # Add visible cells to visited set
            for cell in visible_cells:
                if cell in self._free_cells:
                    self._visited_cells.add(cell)

            # Update coverage percentage (safe division)
            total_cells = self._total_free_cells
            if total_cells > 0:
                self._current_coverage = (
                    len(self._visited_cells) / total_cells * 100.0
                )

    def _calculate_visible_cells(
        self, robot_x: float, robot_y: float, robot_yaw: float
    ) -> Set[Tuple[int, int]]:
        """
        Calculate which coverage cells are visible from current pose.

        Uses sensor FOV and range to determine visibility.
        Implements ray tracing against the occupancy grid to stop at obstacles.

        Args:
            robot_x: Robot X position in world frame
            robot_y: Robot Y position in world frame
            robot_yaw: Robot yaw angle in world frame

        Returns:
            Set of visible coverage cell coordinates
        """
        visible = set()

        if self._map is None:
            return visible

        # Generate rays within sensor FOV
        # 36 rays = 10° spacing, provides good coverage without excessive computation
        num_rays = 36
        half_fov = self._sensor_fov / 2.0
        ray_start_distance = 0.1  # Start rays slightly ahead of robot

        for i in range(num_rays):
            # Calculate ray angle
            angle_offset = -half_fov + (i / (num_rays - 1)) * self._sensor_fov
            ray_angle = robot_yaw + angle_offset

            # Trace ray and add visible cells, stopping at obstacles
            for distance in np.arange(ray_start_distance, self._sensor_range, self._cell_size / 2):
                # Calculate point along ray
                x = robot_x + distance * math.cos(ray_angle)
                y = robot_y + distance * math.sin(ray_angle)

                # Check if this point hits an obstacle in the map
                if self._is_obstacle_at(x, y):
                    break  # Stop ray at obstacle

                # Convert to coverage cell
                cell = self._world_to_coverage_cell(x, y)
                visible.add(cell)

        return visible

    def _is_obstacle_at(self, x: float, y: float) -> bool:
        """
        Check if there is an obstacle at the given world coordinates.

        Args:
            x: World X coordinate
            y: World Y coordinate

        Returns:
            True if obstacle present, False otherwise
        """
        if self._map is None:
            return False

        # Convert world coordinates to map cell
        map_x = int((x - self._map_origin_x) / self._map_resolution)
        map_y = int((y - self._map_origin_y) / self._map_resolution)

        # Check bounds
        if map_x < 0 or map_x >= self._map_width:
            return True  # Out of bounds treated as obstacle
        if map_y < 0 or map_y >= self._map_height:
            return True  # Out of bounds treated as obstacle

        # Get occupancy value from map data
        # Map data is row-major: index = row * width + col
        idx = map_y * self._map_width + map_x
        occupancy = self._map.data[idx]

        # Occupied cells have value > 50 (0-100 scale, 50 is threshold)
        # Unknown cells (-1) are treated as passable for coverage purposes
        return occupancy > 50

    def _world_to_coverage_cell(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to coverage cell coordinates.

        Args:
            x: World X coordinate
            y: World Y coordinate

        Returns:
            Tuple of (cell_x, cell_y)
        """
        cell_x = int(math.floor(x / self._cell_size))
        cell_y = int(math.floor(y / self._cell_size))
        return (cell_x, cell_y)

    def _publish_coverage(self) -> None:
        """Publish current coverage percentage."""
        msg = Float32()
        msg.data = self._current_coverage

        self._coverage_pub.publish(msg)

        self.get_logger().debug(
            f"[NAVIGATION] Coverage: {self._current_coverage:.1f}% "
            f"({len(self._visited_cells)}/{self._total_free_cells} cells)"
        )


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)

    node = CoverageTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
