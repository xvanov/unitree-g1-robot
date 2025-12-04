# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Simulated camera node for G1 Inspector.

Publishes simulated RGB and depth images at specified rates.
Uses same message types as real D435i camera for code compatibility.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class SimCameraNode(Node):
    """
    Simulated camera publisher node.

    Publishes RGB and depth images to /g1/camera/rgb and /g1/camera/depth.
    Matches D435i camera specifications for simulation.
    """

    # Camera specifications (D435i simulation)
    DEFAULT_WIDTH = 640
    DEFAULT_HEIGHT = 480
    DEFAULT_FPS = 1.0  # 1 Hz as per story requirements
    DEFAULT_FOV_H = 87.0  # degrees (D435i horizontal FOV)

    def __init__(self) -> None:
        super().__init__('sim_camera')

        # Declare parameters
        self.declare_parameter('width', self.DEFAULT_WIDTH)
        self.declare_parameter('height', self.DEFAULT_HEIGHT)
        self.declare_parameter('fps', self.DEFAULT_FPS)
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('rgb_frame_id', 'camera_rgb_frame')
        self.declare_parameter('depth_frame_id', 'camera_depth_frame')

        # Get parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.rgb_frame_id = self.get_parameter('rgb_frame_id').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value

        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/g1/camera/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/g1/camera/depth', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/g1/camera/rgb/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/g1/camera/depth/camera_info', 10)

        # Create camera info
        self.camera_info = self._create_camera_info()

        # Frame counter for generating varied images
        self.frame_count = 0

        # Timer for publishing
        publish_period = 1.0 / self.fps
        self.timer = self.create_timer(publish_period, self._publish_images)

        self.get_logger().info(
            f"[SIMULATION] SimCameraNode initialized"
            f" - Resolution: {self.width}x{self.height}"
            f" - Rate: {self.fps} Hz"
        )

    def _create_camera_info(self) -> CameraInfo:
        """Create camera info message with D435i-like parameters."""
        info = CameraInfo()
        info.width = self.width
        info.height = self.height

        # Focal length calculation from FOV
        fov_rad = np.radians(self.DEFAULT_FOV_H)
        fx = self.width / (2 * np.tan(fov_rad / 2))
        fy = fx  # Square pixels

        # Principal point at image center
        cx = self.width / 2.0
        cy = self.height / 2.0

        # Camera matrix K
        info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix (identity for non-stereo)
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P
        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # No distortion (ideal camera)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        return info

    def _publish_images(self) -> None:
        """Publish simulated RGB and depth images."""
        current_time = self.get_clock().now().to_msg()

        # Generate simulated RGB image
        rgb_msg = self._create_rgb_image(current_time)
        self.rgb_pub.publish(rgb_msg)

        # Generate simulated depth image
        depth_msg = self._create_depth_image(current_time)
        self.depth_pub.publish(depth_msg)

        # Publish camera info for both
        self.camera_info.header.stamp = current_time
        self.camera_info.header.frame_id = self.rgb_frame_id
        self.rgb_info_pub.publish(self.camera_info)

        self.camera_info.header.frame_id = self.depth_frame_id
        self.depth_info_pub.publish(self.camera_info)

        self.frame_count += 1

    def _create_rgb_image(self, timestamp: Time) -> Image:
        """Create simulated RGB image with some variation."""
        msg = Image()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.rgb_frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = self.width * 3

        # Generate a simple gradient pattern with some noise
        # This simulates a construction site environment
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Sky (top portion)
        sky_height = self.height // 3
        img[:sky_height, :] = [135, 206, 235]  # Light blue sky

        # Ground/floor (bottom portion)
        ground_start = self.height * 2 // 3
        img[ground_start:, :] = [139, 119, 101]  # Brown/gray concrete

        # Middle section (walls/obstacles)
        img[sky_height:ground_start, :] = [169, 169, 169]  # Gray walls

        # Add some variation based on frame count
        noise = np.random.randint(-10, 10, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Add a simple "obstacle" indicator that moves
        obstacle_x = (self.frame_count * 10) % self.width
        obstacle_y = self.height // 2
        obstacle_size = 30
        x1 = max(0, obstacle_x - obstacle_size)
        x2 = min(self.width, obstacle_x + obstacle_size)
        y1 = max(0, obstacle_y - obstacle_size)
        y2 = min(self.height, obstacle_y + obstacle_size)
        img[y1:y2, x1:x2] = [255, 165, 0]  # Orange obstacle

        msg.data = img.tobytes()
        return msg

    def _create_depth_image(self, timestamp: Time) -> Image:
        """Create simulated depth image."""
        msg = Image()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.depth_frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = '16UC1'  # 16-bit unsigned depth in mm
        msg.is_bigendian = False
        msg.step = self.width * 2

        # Generate depth image with gradient (simulating room)
        # Far wall at ~5m, obstacles at various distances
        depth = np.ones((self.height, self.width), dtype=np.uint16) * 5000  # 5m default

        # Ground plane gets closer toward bottom of image
        for row in range(self.height * 2 // 3, self.height):
            distance_factor = (row - self.height * 2 // 3) / (self.height // 3)
            depth[row, :] = int(5000 - distance_factor * 4000)  # 5m to 1m

        # Add simulated obstacle (closer)
        obstacle_x = (self.frame_count * 10) % self.width
        obstacle_y = self.height // 2
        obstacle_size = 30
        x1 = max(0, obstacle_x - obstacle_size)
        x2 = min(self.width, obstacle_x + obstacle_size)
        y1 = max(0, obstacle_y - obstacle_size)
        y2 = min(self.height, obstacle_y + obstacle_size)
        depth[y1:y2, x1:x2] = 2000  # 2m obstacle

        # Add noise
        noise = np.random.randint(-50, 50, depth.shape, dtype=np.int32)
        depth = np.clip(depth.astype(np.int32) + noise, 100, 10000).astype(np.uint16)

        msg.data = depth.tobytes()
        return msg


def main(args=None):
    """Main entry point for sim_camera node."""
    rclpy.init(args=args)

    try:
        node = SimCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[SIMULATION] Camera error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
