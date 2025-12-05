#!/usr/bin/env python3
"""
Depth Image to PointCloud2 Converter for G1 Inspector.

Story 1.3: Navigation Stack Integration
Converts depth images from D435i (or simulation) to PointCloud2 for Nav2 costmap.

Subscribes: /g1/camera/depth (sensor_msgs/Image)
            /g1/camera/depth/camera_info (sensor_msgs/CameraInfo)
Publishes: /g1/camera/depth/points (sensor_msgs/PointCloud2)
"""

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header


class DepthToPointCloud(Node):
    """Converts depth images to PointCloud2 for Nav2 obstacle detection."""

    DEFAULT_DEPTH_TOPIC = '/g1/camera/depth'
    DEFAULT_INFO_TOPIC = '/g1/camera/depth/camera_info'
    DEFAULT_OUTPUT_TOPIC = '/g1/camera/depth/points'
    DEFAULT_MIN_DEPTH = 0.1   # meters
    DEFAULT_MAX_DEPTH = 5.0   # meters
    DEFAULT_VOXEL_SIZE = 0.05  # meters (for downsampling)

    def __init__(self) -> None:
        """Initialize the DepthToPointCloud node."""
        super().__init__('depth_to_pointcloud')

        # Declare parameters
        self.declare_parameter('depth_topic', self.DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('camera_info_topic', self.DEFAULT_INFO_TOPIC)
        self.declare_parameter('output_topic', self.DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('min_depth', self.DEFAULT_MIN_DEPTH)
        self.declare_parameter('max_depth', self.DEFAULT_MAX_DEPTH)
        self.declare_parameter('voxel_size', self.DEFAULT_VOXEL_SIZE)
        self.declare_parameter('output_frame', 'camera_depth_frame')
        # Note: use_sim_time is automatically declared by ROS2 when set globally

        # Get parameters
        self._depth_topic = self.get_parameter('depth_topic').value
        self._info_topic = self.get_parameter('camera_info_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._min_depth = self.get_parameter('min_depth').value
        self._max_depth = self.get_parameter('max_depth').value
        self._voxel_size = self.get_parameter('voxel_size').value
        self._output_frame = self.get_parameter('output_frame').value

        # Camera intrinsics (will be populated from CameraInfo)
        self._fx: Optional[float] = None
        self._fy: Optional[float] = None
        self._cx: Optional[float] = None
        self._cy: Optional[float] = None
        self._camera_info_received = False

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self._info_sub = self.create_subscription(
            CameraInfo,
            self._info_topic,
            self._camera_info_callback,
            sensor_qos
        )

        self._depth_sub = self.create_subscription(
            Image,
            self._depth_topic,
            self._depth_callback,
            sensor_qos
        )

        # Publisher
        self._cloud_pub = self.create_publisher(
            PointCloud2,
            self._output_topic,
            sensor_qos
        )

        self.get_logger().info(
            f"[PERCEPTION] DepthToPointCloud initialized: "
            f"{self._depth_topic} -> {self._output_topic}"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """
        Handle camera info message to extract intrinsics.

        Args:
            msg: CameraInfo message
        """
        if self._camera_info_received:
            return

        # Extract intrinsics from K matrix
        # K = [fx  0  cx]
        #     [0  fy  cy]
        #     [0   0   1]
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]

        # Validate intrinsics - must be non-zero and finite
        if fx <= 0 or fy <= 0 or not np.isfinite(fx) or not np.isfinite(fy):
            self.get_logger().warn(
                f"[PERCEPTION] Invalid camera intrinsics received: "
                f"fx={fx}, fy={fy}. Waiting for valid camera_info."
            )
            return

        if not np.isfinite(cx) or not np.isfinite(cy):
            self.get_logger().warn(
                f"[PERCEPTION] Invalid principal point: cx={cx}, cy={cy}. "
                "Waiting for valid camera_info."
            )
            return

        self._fx = fx
        self._fy = fy
        self._cx = cx
        self._cy = cy

        self._camera_info_received = True
        self.get_logger().info(
            f"[PERCEPTION] Camera intrinsics received: "
            f"fx={self._fx:.1f}, fy={self._fy:.1f}, cx={self._cx:.1f}, cy={self._cy:.1f}"
        )

    def _depth_callback(self, msg: Image) -> None:
        """
        Convert depth image to PointCloud2.

        Args:
            msg: Depth image message (16UC1 in mm or 32FC1 in meters)
        """
        if not self._camera_info_received:
            self.get_logger().debug("[PERCEPTION] Waiting for camera info...")
            return

        try:
            # Convert depth image to numpy array
            depth_array = self._image_to_array(msg)
            if depth_array is None:
                return

            # Generate point cloud
            points = self._depth_to_points(depth_array, msg.width, msg.height)
            if points is None or len(points) == 0:
                return

            # Create and publish PointCloud2 message
            cloud_msg = self._create_pointcloud2(msg.header, points)
            self._cloud_pub.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f"[PERCEPTION] Depth conversion error: {e}")

    def _image_to_array(self, msg: Image) -> Optional[np.ndarray]:
        """
        Convert Image message to numpy depth array in meters.

        Args:
            msg: Depth Image message

        Returns:
            Numpy array with depth values in meters, or None on error
        """
        if msg.encoding == '16UC1':
            # 16-bit unsigned in millimeters
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                (msg.height, msg.width)
            )
            depth = depth.astype(np.float32) / 1000.0  # Convert mm to meters

        elif msg.encoding == '32FC1':
            # 32-bit float in meters
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                (msg.height, msg.width)
            )

        else:
            self.get_logger().warn(
                f"[PERCEPTION] Unsupported depth encoding: {msg.encoding}"
            )
            return None

        return depth

    def _depth_to_points(
        self,
        depth: np.ndarray,
        width: int,
        height: int
    ) -> Optional[np.ndarray]:
        """
        Convert depth image to 3D point cloud.

        Uses pinhole camera model:
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = depth

        Args:
            depth: Depth array in meters
            width: Image width
            height: Image height

        Returns:
            Numpy array of shape (N, 3) with XYZ points
        """
        # Create pixel coordinate grids
        u = np.arange(width)
        v = np.arange(height)
        u, v = np.meshgrid(u, v)

        # Filter valid depth values
        valid_mask = (depth > self._min_depth) & (depth < self._max_depth) & ~np.isnan(depth)

        if not np.any(valid_mask):
            return None

        # Get valid pixels
        u_valid = u[valid_mask]
        v_valid = v[valid_mask]
        z_valid = depth[valid_mask]

        # Project to 3D
        x = (u_valid - self._cx) * z_valid / self._fx
        y = (v_valid - self._cy) * z_valid / self._fy
        z = z_valid

        # Stack into point cloud
        points = np.stack([x, y, z], axis=1).astype(np.float32)

        # Optional: voxel downsampling for efficiency
        if self._voxel_size > 0:
            points = self._voxel_downsample(points)

        return points

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        Simple voxel grid downsampling.

        Args:
            points: Input point cloud (N, 3)

        Returns:
            Downsampled point cloud
        """
        if len(points) == 0:
            return points

        # Quantize points to voxel grid
        voxel_indices = np.floor(points / self._voxel_size).astype(np.int32)

        # Get unique voxels and their indices
        unique_voxels, indices = np.unique(
            voxel_indices, axis=0, return_inverse=True
        )

        # Average points within each voxel - vectorized using np.bincount
        num_voxels = len(unique_voxels)
        counts = np.bincount(indices, minlength=num_voxels)

        # Guard against divide by zero (shouldn't happen, but defensive)
        counts = np.maximum(counts, 1)

        # Sum points per voxel for each coordinate
        downsampled = np.zeros((num_voxels, 3), dtype=np.float32)
        for dim in range(3):
            downsampled[:, dim] = np.bincount(
                indices, weights=points[:, dim], minlength=num_voxels
            )

        # Compute average (safe, counts >= 1)
        downsampled /= counts[:, np.newaxis]

        return downsampled

    def _create_pointcloud2(
        self,
        header: Header,
        points: np.ndarray
    ) -> PointCloud2:
        """
        Create PointCloud2 message from numpy array.

        Args:
            header: Header from original depth image
            points: Point cloud array (N, 3)

        Returns:
            PointCloud2 message
        """
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = self._output_frame

        msg.height = 1
        msg.width = len(points)

        # Define fields for XYZ
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # Pack point data
        msg.data = points.tobytes()

        return msg


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)

    node = DepthToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
