# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
MuJoCo simulation manager for G1 Inspector.

Central simulation node that runs MuJoCo physics and provides
sensor data to all other simulation nodes via shared state.
Runs headless by default with all visualization through RViz.
"""

import os
import math
import time
import threading
from pathlib import Path
from typing import Optional, Tuple
from dataclasses import dataclass, field

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu, CameraInfo
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import struct


@dataclass
class SimState:
    """Shared simulation state accessible by all components."""
    # Robot pose in world frame
    x: float = 0.0
    y: float = 0.0
    z: float = 0.793  # G1 pelvis height
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0

    # Robot velocities (body frame)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    omega_x: float = 0.0
    omega_y: float = 0.0
    omega_z: float = 0.0

    # IMU data
    accel: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 9.81]))
    gyro: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))

    # Timestamps
    sim_time: float = 0.0


class MuJoCoSimManager(Node):
    """
    Central MuJoCo simulation manager node.

    Runs physics simulation, handles locomotion commands, and provides
    sensor data (camera, LiDAR, IMU) to ROS2 topics.
    """

    # Simulation parameters
    PHYSICS_DT = 0.002      # 500 Hz physics
    CONTROL_DT = 0.01       # 100 Hz control loop

    # Velocity limits (architecture compliance)
    MAX_LINEAR_VELOCITY = 0.5   # m/s
    MAX_ANGULAR_VELOCITY = 1.0  # rad/s
    CMD_VEL_TIMEOUT = 0.5       # seconds

    # Sensor rates
    ODOM_RATE = 50.0    # Hz
    CAMERA_RATE = 1.0   # Hz
    LIDAR_RATE = 10.0   # Hz
    IMU_RATE = 100.0    # Hz

    # Camera parameters (D435i simulation)
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FOV = 87.0  # degrees horizontal

    # LiDAR parameters (MID-360 simulation)
    LIDAR_NUM_POINTS = 5000
    LIDAR_MIN_RANGE = 0.1
    LIDAR_MAX_RANGE = 40.0

    def __init__(self) -> None:
        super().__init__('mujoco_sim_manager')

        # Declare parameters
        self.declare_parameter('scene_file', '')
        self.declare_parameter('physics_dt', self.PHYSICS_DT)
        self.declare_parameter('use_g1_model', True)

        # Get parameters
        scene_file_param = self.get_parameter('scene_file').value
        self.physics_dt = self.get_parameter('physics_dt').value
        self.use_g1_model = self.get_parameter('use_g1_model').value

        # Resolve scene file path
        self.scene_file = self._resolve_scene_path(scene_file_param)

        # Shared simulation state
        self.state = SimState()
        self.state_lock = threading.Lock()

        # MuJoCo objects
        self.mj_model: Optional[mujoco.MjModel] = None
        self.mj_data: Optional[mujoco.MjData] = None
        self.renderer: Optional[mujoco.Renderer] = None

        # Control state
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_omega = 0.0
        self.last_cmd_time: Optional[float] = None

        # Threading
        self._running = False
        self._physics_thread: Optional[threading.Thread] = None

        # Initialize MuJoCo
        self._init_mujoco()

        # QoS for cmd_vel
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/g1/cmd_vel', self._cmd_vel_callback, cmd_vel_qos
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/g1/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/g1/imu/data', 10)
        self.rgb_pub = self.create_publisher(Image, '/g1/camera/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/g1/camera/depth', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/g1/camera/rgb/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/g1/camera/depth/camera_info', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/g1/lidar/points', 10)

        # TF broadcaster for odom -> base_link (dynamic transform)
        # Note: Static sensor frame transforms are handled by robot_state_publisher via URDF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create camera info
        self.camera_info = self._create_camera_info()

        # Create timers for publishing
        self.create_timer(1.0 / self.ODOM_RATE, self._publish_odom)
        self.create_timer(1.0 / self.IMU_RATE, self._publish_imu)
        self.create_timer(1.0 / self.CAMERA_RATE, self._publish_camera)
        self.create_timer(1.0 / self.LIDAR_RATE, self._publish_lidar)
        self.create_timer(self.CONTROL_DT, self._control_loop)

        # Start physics thread
        self._running = True
        self._physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._physics_thread.start()

        self.get_logger().info(
            f"[MUJOCO] Simulation manager initialized"
            f" - Scene: {self.scene_file}"
            f" - Physics dt: {self.physics_dt}s"
        )

    def _resolve_scene_path(self, scene_file_param: str) -> str:
        """Resolve the MuJoCo scene file path."""
        if scene_file_param:
            if os.path.isabs(scene_file_param):
                return scene_file_param
            return str(Path.cwd() / scene_file_param)

        # Default: look for unitree_mujoco G1 scene
        # Prefer 23DOF version to match real hardware
        # Try multiple possible locations
        possible_roots = [
            Path.home() / "unitree-g1-robot",
            Path("/home/ubuntu/unitree-g1-robot"),
            Path(__file__).parent.parent.parent.parent.parent,
            Path.cwd(),
        ]

        for root in possible_roots:
            # First try 23DOF (matches real G1 hardware)
            scene_23dof = root / "external" / "unitree_mujoco" / "unitree_robots" / "g1" / "scene_23dof.xml"
            if scene_23dof.exists():
                self.get_logger().info(f"[MUJOCO] Found G1 23DOF scene at {scene_23dof}")
                return str(scene_23dof)
            # Fallback to default scene.xml
            default_scene = root / "external" / "unitree_mujoco" / "unitree_robots" / "g1" / "scene.xml"
            if default_scene.exists():
                self.get_logger().info(f"[MUJOCO] Found G1 scene at {default_scene}")
                return str(default_scene)

        self.get_logger().warn(f"[MUJOCO] G1 scene not found, using simple scene")
        return self._create_simple_scene()

    def _create_simple_scene(self) -> str:
        """Create a simple indoor scene for simulation."""
        scene_xml = """
<mujoco model="g1_simple_scene">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
  </visual>

  <asset>
    <texture type="2d" name="groundplane" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5"/>
    <material name="wall" rgba="0.8 0.8 0.8 1"/>
    <material name="obstacle" rgba="0.6 0.4 0.2 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>

    <!-- Ground -->
    <geom name="floor" size="10 10 0.05" type="plane" material="groundplane"/>

    <!-- Walls (10m x 10m room) -->
    <geom name="wall_north" type="box" size="10 0.1 1.5" pos="0 10 0.75" material="wall"/>
    <geom name="wall_south" type="box" size="10 0.1 1.5" pos="0 -10 0.75" material="wall"/>
    <geom name="wall_east" type="box" size="0.1 10 1.5" pos="10 0 0.75" material="wall"/>
    <geom name="wall_west" type="box" size="0.1 10 1.5" pos="-10 0 0.75" material="wall"/>

    <!-- Obstacles -->
    <geom name="obstacle_1" type="box" size="0.5 0.5 0.5" pos="3 2 0.5" material="obstacle"/>
    <geom name="obstacle_2" type="cylinder" size="0.3 0.6" pos="-2 3 0.6" material="obstacle"/>
    <geom name="obstacle_3" type="box" size="1.0 0.3 0.4" pos="0 -3 0.4" material="obstacle"/>

    <!-- Robot base with sensors -->
    <body name="pelvis" pos="0 0 0.793">
      <freejoint name="floating_base_joint"/>
      <geom name="base_geom" type="box" size="0.2 0.15 0.3" rgba="0.2 0.2 0.8 1"/>
      <site name="imu" size="0.01" pos="0 0 0"/>

      <!-- Camera mount (front of robot, angled down slightly) -->
      <body name="camera_link" pos="0.25 0 0.1">
        <geom type="box" size="0.03 0.08 0.02" rgba="0.1 0.1 0.1 1"/>
        <site name="camera_site" pos="0 0 0" euler="0 0.1 0"/>
        <camera name="rgb_camera" pos="0 0 0" euler="0 0.1 0" fovy="58"/>
      </body>

      <!-- LiDAR mount (top of robot) -->
      <body name="lidar_link" pos="0 0 0.4">
        <geom type="cylinder" size="0.05 0.03" rgba="0.3 0.3 0.3 1"/>
        <site name="lidar_site" pos="0 0 0.03"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
        scene_path = "/tmp/g1_simple_scene.xml"
        with open(scene_path, 'w') as f:
            f.write(scene_xml)
        return scene_path

    def _init_mujoco(self) -> None:
        """Initialize MuJoCo simulation."""
        try:
            self.mj_model = mujoco.MjModel.from_xml_path(self.scene_file)
            self.mj_data = mujoco.MjData(self.mj_model)

            # Set physics timestep
            self.mj_model.opt.timestep = self.physics_dt

            # Initialize renderer for camera
            self.renderer = mujoco.Renderer(self.mj_model, self.CAMERA_HEIGHT, self.CAMERA_WIDTH)

            # Get initial pose from MuJoCo
            self._sync_state_from_mujoco()

            self.get_logger().info("[MUJOCO] MuJoCo initialized successfully")

        except Exception as e:
            self.get_logger().error(f"[MUJOCO] Failed to initialize: {e}")
            raise

    def _physics_loop(self) -> None:
        """Main physics simulation loop (runs in separate thread)."""
        while self._running:
            step_start = time.perf_counter()

            with self.state_lock:
                mujoco.mj_step(self.mj_model, self.mj_data)
                self._sync_state_from_mujoco()

            # Maintain real-time
            elapsed = time.perf_counter() - step_start
            sleep_time = self.physics_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _control_loop(self) -> None:
        """Control loop - apply velocity commands via teleportation."""
        # Check for cmd_vel timeout
        if self.last_cmd_time is not None:
            time_since_cmd = time.time() - self.last_cmd_time
            if time_since_cmd > self.CMD_VEL_TIMEOUT:
                if self.cmd_vx != 0.0 or self.cmd_vy != 0.0 or self.cmd_omega != 0.0:
                    self.get_logger().warn("[MUJOCO] cmd_vel timeout, stopping robot")
                self.cmd_vx = 0.0
                self.cmd_vy = 0.0
                self.cmd_omega = 0.0

        # Apply velocity by teleporting robot
        with self.state_lock:
            # Get current yaw from quaternion
            yaw = self._quaternion_to_yaw(self.state.qw, self.state.qx, self.state.qy, self.state.qz)

            # Transform velocities from body to world frame
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            vx_world = self.cmd_vx * cos_yaw - self.cmd_vy * sin_yaw
            vy_world = self.cmd_vx * sin_yaw + self.cmd_vy * cos_yaw

            # Update pose
            dt = self.CONTROL_DT
            new_x = self.state.x + vx_world * dt
            new_y = self.state.y + vy_world * dt
            new_yaw = yaw + self.cmd_omega * dt

            # Set new pose in MuJoCo
            self._set_robot_pose(new_x, new_y, self.state.z, new_yaw)

            # Update velocity state
            self.state.vx = self.cmd_vx
            self.state.vy = self.cmd_vy
            self.state.omega_z = self.cmd_omega

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Handle incoming velocity commands."""
        self.cmd_vx = self._clamp(msg.linear.x, -self.MAX_LINEAR_VELOCITY, self.MAX_LINEAR_VELOCITY)
        self.cmd_vy = self._clamp(msg.linear.y, -self.MAX_LINEAR_VELOCITY, self.MAX_LINEAR_VELOCITY)
        self.cmd_omega = self._clamp(msg.angular.z, -self.MAX_ANGULAR_VELOCITY, self.MAX_ANGULAR_VELOCITY)
        self.last_cmd_time = time.time()

    def _sync_state_from_mujoco(self) -> None:
        """Sync simulation state from MuJoCo data."""
        try:
            # Get pelvis body ID
            pelvis_id = self.mj_model.body('pelvis').id

            # Position (qpos for freejoint: x, y, z, qw, qx, qy, qz)
            self.state.x = self.mj_data.qpos[0]
            self.state.y = self.mj_data.qpos[1]
            self.state.z = self.mj_data.qpos[2]
            self.state.qw = self.mj_data.qpos[3]
            self.state.qx = self.mj_data.qpos[4]
            self.state.qy = self.mj_data.qpos[5]
            self.state.qz = self.mj_data.qpos[6]

            # Velocities (qvel for freejoint: vx, vy, vz, wx, wy, wz)
            self.state.vx = self.mj_data.qvel[0]
            self.state.vy = self.mj_data.qvel[1]
            self.state.vz = self.mj_data.qvel[2]
            self.state.omega_x = self.mj_data.qvel[3]
            self.state.omega_y = self.mj_data.qvel[4]
            self.state.omega_z = self.mj_data.qvel[5]

            # IMU data from sensor or body acceleration
            # MuJoCo provides acceleration in body frame
            body_acc = self.mj_data.body(pelvis_id).cacc[:3]  # Linear acceleration
            body_gyro = self.mj_data.body(pelvis_id).cacc[3:]  # Angular acceleration

            # For IMU, we need angular velocity (not acceleration) and linear acceleration + gravity
            self.state.gyro = np.array([self.state.omega_x, self.state.omega_y, self.state.omega_z])

            # Accelerometer measures specific force (acceleration - gravity in body frame)
            # For a stationary robot, this should read [0, 0, +g]
            # Transform gravity to body frame and add
            gravity_world = np.array([0, 0, -9.81])
            R = self._quaternion_to_rotation_matrix(self.state.qw, self.state.qx, self.state.qy, self.state.qz)
            gravity_body = R.T @ gravity_world
            self.state.accel = np.array([body_acc[0], body_acc[1], body_acc[2]]) - gravity_body

            self.state.sim_time = self.mj_data.time

        except Exception as e:
            self.get_logger().warn(f"[MUJOCO] Failed to sync state: {e}")

    def _set_robot_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        """Set robot pose in MuJoCo (teleport)."""
        try:
            # Convert yaw to quaternion
            qw = math.cos(yaw / 2)
            qz = math.sin(yaw / 2)

            # Set qpos
            self.mj_data.qpos[0] = x
            self.mj_data.qpos[1] = y
            self.mj_data.qpos[2] = z
            self.mj_data.qpos[3] = qw
            self.mj_data.qpos[4] = 0.0
            self.mj_data.qpos[5] = 0.0
            self.mj_data.qpos[6] = qz

            # Zero out velocities to prevent drift
            self.mj_data.qvel[:6] = 0.0

            # Forward kinematics to update body positions
            mujoco.mj_forward(self.mj_model, self.mj_data)

        except Exception as e:
            self.get_logger().warn(f"[MUJOCO] Failed to set pose: {e}")

    def _publish_odom(self) -> None:
        """Publish odometry and TF."""
        with self.state_lock:
            now = self.get_clock().now()

            # Odometry message
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.state.x
            odom.pose.pose.position.y = self.state.y
            odom.pose.pose.position.z = self.state.z
            odom.pose.pose.orientation.w = self.state.qw
            odom.pose.pose.orientation.x = self.state.qx
            odom.pose.pose.orientation.y = self.state.qy
            odom.pose.pose.orientation.z = self.state.qz

            odom.twist.twist.linear.x = self.state.vx
            odom.twist.twist.linear.y = self.state.vy
            odom.twist.twist.linear.z = self.state.vz
            odom.twist.twist.angular.x = self.state.omega_x
            odom.twist.twist.angular.y = self.state.omega_y
            odom.twist.twist.angular.z = self.state.omega_z

            # Covariance
            odom.pose.covariance[0] = 0.01
            odom.pose.covariance[7] = 0.01
            odom.pose.covariance[14] = 0.01
            odom.pose.covariance[35] = 0.01

            self.odom_pub.publish(odom)

            # TF: odom -> base_link
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = self.state.x
            tf.transform.translation.y = self.state.y
            tf.transform.translation.z = self.state.z
            tf.transform.rotation.w = self.state.qw
            tf.transform.rotation.x = self.state.qx
            tf.transform.rotation.y = self.state.qy
            tf.transform.rotation.z = self.state.qz

            self.tf_broadcaster.sendTransform(tf)

    def _publish_imu(self) -> None:
        """Publish IMU data from MuJoCo physics."""
        with self.state_lock:
            now = self.get_clock().now().to_msg()

            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'

            # Orientation
            imu.orientation.w = self.state.qw
            imu.orientation.x = self.state.qx
            imu.orientation.y = self.state.qy
            imu.orientation.z = self.state.qz
            imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            # Angular velocity (with small noise)
            noise = np.random.normal(0, 0.001, 3)
            imu.angular_velocity.x = float(self.state.gyro[0] + noise[0])
            imu.angular_velocity.y = float(self.state.gyro[1] + noise[1])
            imu.angular_velocity.z = float(self.state.gyro[2] + noise[2])
            imu.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]

            # Linear acceleration (with small noise)
            noise = np.random.normal(0, 0.01, 3)
            imu.linear_acceleration.x = float(self.state.accel[0] + noise[0])
            imu.linear_acceleration.y = float(self.state.accel[1] + noise[1])
            imu.linear_acceleration.z = float(self.state.accel[2] + noise[2])
            imu.linear_acceleration_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

            self.imu_pub.publish(imu)

    def _publish_camera(self) -> None:
        """Publish camera RGB and depth images rendered from MuJoCo."""
        with self.state_lock:
            now = self.get_clock().now().to_msg()

            try:
                # Get camera position and orientation from robot state
                # Camera is mounted at base_link + (0.25, 0, 0.1), looking forward
                # We need to compute the camera pose in world frame

                # Get yaw angle from quaternion
                yaw = self._quaternion_to_yaw(
                    self.state.qw, self.state.qx, self.state.qy, self.state.qz
                )

                # Camera offset in robot frame (forward, left, up)
                cam_offset_x = 0.25
                cam_offset_z = 0.1

                # Camera position in world frame
                cam_x = self.state.x + cam_offset_x * math.cos(yaw)
                cam_y = self.state.y + cam_offset_x * math.sin(yaw)
                cam_z = self.state.z + cam_offset_z

                # Create camera structure for MuJoCo
                # MjvCamera uses lookat point + distance + azimuth/elevation
                camera = mujoco.MjvCamera()

                # Look at a point 2m ahead of the robot
                look_distance = 2.0
                camera.lookat[0] = cam_x + look_distance * math.cos(yaw)
                camera.lookat[1] = cam_y + look_distance * math.sin(yaw)
                camera.lookat[2] = cam_z  # Same height as camera

                # Distance from lookat point (camera is at lookat - distance * view_dir)
                camera.distance = look_distance

                # Azimuth: rotation around vertical axis
                # MuJoCo: 0 = looking along -Y, 90 = looking along +X
                # Our yaw: 0 = looking along +X
                # So: azimuth = -yaw_degrees + 90
                camera.azimuth = -math.degrees(yaw) + 90.0

                # Elevation: angle above/below horizontal
                # MuJoCo: positive = looking UP, negative = looking DOWN
                # We want slight downward tilt
                camera.elevation = -10.0

                # Update scene with this camera
                self.renderer.update_scene(self.mj_data, camera=camera)

                # Render RGB
                rgb = self.renderer.render()

                # Publish RGB image
                rgb_msg = Image()
                rgb_msg.header.stamp = now
                rgb_msg.header.frame_id = 'camera_rgb_frame'
                rgb_msg.height = self.CAMERA_HEIGHT
                rgb_msg.width = self.CAMERA_WIDTH
                rgb_msg.encoding = 'rgb8'
                rgb_msg.is_bigendian = False
                rgb_msg.step = self.CAMERA_WIDTH * 3
                # Don't flip - MuJoCo camera orientation is correct
                rgb_msg.data = rgb.tobytes()
                self.rgb_pub.publish(rgb_msg)

                # Render depth
                self.renderer.enable_depth_rendering()
                depth = self.renderer.render()
                self.renderer.disable_depth_rendering()

                # Convert depth to 16-bit mm
                # MuJoCo depth is normalized, convert to approximate meters then mm
                depth_m = depth.astype(np.float32)
                # Clip and convert to mm (16-bit)
                depth_mm = np.clip(depth_m * 10000, 0, 65535).astype(np.uint16)

                depth_msg = Image()
                depth_msg.header.stamp = now
                depth_msg.header.frame_id = 'camera_depth_frame'
                depth_msg.height = self.CAMERA_HEIGHT
                depth_msg.width = self.CAMERA_WIDTH
                depth_msg.encoding = '16UC1'
                depth_msg.is_bigendian = False
                depth_msg.step = self.CAMERA_WIDTH * 2
                # Don't flip depth either
                depth_msg.data = depth_mm.tobytes()
                self.depth_pub.publish(depth_msg)

                # Publish camera info
                self.camera_info.header.stamp = now
                self.camera_info.header.frame_id = 'camera_rgb_frame'
                self.rgb_info_pub.publish(self.camera_info)
                self.camera_info.header.frame_id = 'camera_depth_frame'
                self.depth_info_pub.publish(self.camera_info)

            except Exception as e:
                self.get_logger().warn(f"[MUJOCO] Camera render failed: {e}", throttle_duration_sec=5.0)

    def _publish_lidar(self) -> None:
        """Publish LiDAR point cloud using MuJoCo raycasting."""
        with self.state_lock:
            now = self.get_clock().now().to_msg()

            try:
                # Get LiDAR position in world frame
                # LiDAR is at base_link + (0, 0, 0.4)
                lidar_pos = np.array([
                    self.state.x,
                    self.state.y,
                    self.state.z + 0.4  # LiDAR offset
                ])

                # Generate ray directions for 360° scan
                points = []
                for i in range(self.LIDAR_NUM_POINTS):
                    # Horizontal angle (full 360°)
                    h_angle = (i / self.LIDAR_NUM_POINTS) * 2 * math.pi

                    # Vertical angle (MID-360: -7° to +52°)
                    v_angle = np.random.uniform(math.radians(-7), math.radians(52))

                    # Ray direction
                    cos_v = math.cos(v_angle)
                    dx = math.cos(h_angle) * cos_v
                    dy = math.sin(h_angle) * cos_v
                    dz = math.sin(v_angle)

                    ray_dir = np.array([dx, dy, dz])

                    # Perform raycast
                    geom_id = np.zeros(1, dtype=np.int32)
                    distance = mujoco.mj_ray(
                        self.mj_model, self.mj_data,
                        lidar_pos, ray_dir,
                        None,  # geomgroup (all geoms)
                        1,     # flg_static
                        -1,    # bodyexclude (-1 = none)
                        geom_id
                    )

                    if distance > 0 and self.LIDAR_MIN_RANGE < distance < self.LIDAR_MAX_RANGE:
                        # Point in LiDAR frame
                        x = dx * distance
                        y = dy * distance
                        z = dz * distance

                        # Intensity based on distance
                        intensity = max(0, 100 * (1 - distance / self.LIDAR_MAX_RANGE))

                        points.append((x, y, z, intensity))

                # Create PointCloud2 message
                pc_msg = self._create_pointcloud2(points, now)
                self.lidar_pub.publish(pc_msg)

            except Exception as e:
                self.get_logger().warn(f"[MUJOCO] LiDAR raycast failed: {e}", throttle_duration_sec=5.0)

    def _create_pointcloud2(self, points: list, timestamp) -> PointCloud2:
        """Create PointCloud2 message from list of (x, y, z, intensity) points."""
        msg = PointCloud2()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'lidar_link'

        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(points)
        msg.is_dense = True

        # Pack data
        data = bytearray(len(points) * 16)
        for i, (x, y, z, intensity) in enumerate(points):
            struct.pack_into('ffff', data, i * 16, x, y, z, intensity)

        msg.data = bytes(data)
        return msg

    def _create_camera_info(self) -> CameraInfo:
        """Create camera info message."""
        info = CameraInfo()
        info.width = self.CAMERA_WIDTH
        info.height = self.CAMERA_HEIGHT

        # Focal length from FOV
        fov_rad = math.radians(self.CAMERA_FOV)
        fx = self.CAMERA_WIDTH / (2 * math.tan(fov_rad / 2))
        fy = fx
        cx = self.CAMERA_WIDTH / 2.0
        cy = self.CAMERA_HEIGHT / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        return info

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range."""
        return max(min_val, min(max_val, value))

    @staticmethod
    def _quaternion_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_to_rotation_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
        """Convert quaternion to rotation matrix."""
        return np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
        ])

    def shutdown(self) -> None:
        """Shutdown simulation."""
        self._running = False
        if self._physics_thread:
            self._physics_thread.join(timeout=1.0)
        self.get_logger().info("[MUJOCO] Simulation shutdown complete")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = MuJoCoSimManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[MUJOCO] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
