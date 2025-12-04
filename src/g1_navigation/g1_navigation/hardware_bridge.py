# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Hardware bridge node for real G1 robot.

Bridges between Unitree SDK2 and ROS2:
- Subscribes to SDK "rt/lowstate" and publishes to ROS2 sensor topics
- Subscribes to ROS2 /g1/cmd_vel and calls SDK LocoClient.Move()

This node enables the same ROS2 interfaces used in simulation to work
with the real robot hardware.

Usage:
    ros2 run g1_navigation hardware_bridge --ros-args -p network_interface:=eth0
"""

from __future__ import annotations

import math
import signal
import time
import threading
from typing import Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header

# SDK imports - these may not be available in all environments
SDK_AVAILABLE = False
SDK_IMPORT_ERROR = ""

try:
    from unitree_sdk2py.core.channel import (
        ChannelSubscriber,
        ChannelFactoryInitialize
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    SDK_AVAILABLE = True
except ImportError as e:
    SDK_IMPORT_ERROR = str(e)
    # Define placeholder types for type hints when SDK is not available
    LowState_ = Any
    LocoClient = Any
    ChannelSubscriber = Any
    ChannelFactoryInitialize = None


# G1 Joint names for JointState message
JOINT_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"
]

G1_NUM_MOTOR = 29


class HardwareBridge(Node):
    """
    ROS2 node that bridges Unitree SDK to ROS2 topics.

    Publishes:
        /g1/imu/data (sensor_msgs/Imu): IMU data from robot
        /g1/joint_states (sensor_msgs/JointState): Joint positions/velocities

    Subscribes:
        /g1/cmd_vel (geometry_msgs/Twist): Velocity commands for robot
    """

    # Velocity limits (conservative for safety)
    MAX_LINEAR_VELOCITY = 0.3   # m/s (architecture allows 0.5, using 0.3 for safety)
    MAX_ANGULAR_VELOCITY = 0.5  # rad/s
    CMD_VEL_TIMEOUT = 0.5       # seconds

    # Publishing rates
    IMU_PUBLISH_RATE = 100.0    # Hz
    JOINT_PUBLISH_RATE = 50.0   # Hz

    def __init__(self) -> None:
        super().__init__('hardware_bridge')

        # Check SDK availability
        if not SDK_AVAILABLE:
            self.get_logger().error(
                f"[NAVIGATION] Unitree SDK2 not available: {SDK_IMPORT_ERROR}"
            )
            self.get_logger().error(
                "[NAVIGATION] Install with: cd external/unitree_sdk2_python && pip install -e ."
            )
            raise RuntimeError("Unitree SDK2 not available")

        # Declare parameters
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('max_linear_velocity', self.MAX_LINEAR_VELOCITY)
        self.declare_parameter('max_angular_velocity', self.MAX_ANGULAR_VELOCITY)
        self.declare_parameter('cmd_vel_timeout', self.CMD_VEL_TIMEOUT)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('auto_stand', True)

        # Get parameters
        self.network_interface = self.get_parameter('network_interface').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.auto_stand = self.get_parameter('auto_stand').value

        # State
        self.last_low_state: Optional[LowState_] = None
        self.last_cmd_time: Optional[float] = None
        self.connected = False
        self.loco_client: Optional[LocoClient] = None
        self.state_lock = threading.Lock()

        # Initialize SDK
        self._initialize_sdk()

        # QoS for cmd_vel (best effort for real-time)
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.imu_pub = self.create_publisher(
            Imu,
            '/g1/imu/data',
            10
        )
        self.joint_pub = self.create_publisher(
            JointState,
            '/g1/joint_states',
            10
        )

        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/g1/cmd_vel',
            self._cmd_vel_callback,
            cmd_vel_qos
        )

        # Publishing timers
        imu_period = 1.0 / self.IMU_PUBLISH_RATE
        self.imu_timer = self.create_timer(imu_period, self._publish_imu)

        joint_period = 1.0 / self.JOINT_PUBLISH_RATE
        self.joint_timer = self.create_timer(joint_period, self._publish_joint_states)

        # Timeout checker timer
        self.timeout_timer = self.create_timer(0.1, self._check_cmd_timeout)

        self.get_logger().info(
            f"[NAVIGATION] HardwareBridge initialized"
            f" - Interface: {self.network_interface}"
            f" - Max linear: {self.max_linear_vel} m/s"
            f" - Max angular: {self.max_angular_vel} rad/s"
        )

    def _initialize_sdk(self) -> None:
        """Initialize Unitree SDK2 and connect to robot."""
        self.get_logger().info(
            f"[NAVIGATION] Initializing CycloneDDS on {self.network_interface}..."
        )

        try:
            ChannelFactoryInitialize(0, self.network_interface)
            self.get_logger().info("[NAVIGATION] CycloneDDS initialized")
        except Exception as e:
            self.get_logger().error(f"[NAVIGATION] CycloneDDS init failed: {e}")
            raise

        # Subscribe to low-level state
        self.get_logger().info("[NAVIGATION] Creating state subscriber...")
        try:
            self.state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self.state_subscriber.Init(self._low_state_callback, 10)
            self.get_logger().info("[NAVIGATION] State subscriber created")
        except Exception as e:
            self.get_logger().error(f"[NAVIGATION] State subscriber failed: {e}")
            raise

        # Create locomotion client
        self.get_logger().info("[NAVIGATION] Creating LocoClient...")
        try:
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(10.0)
            self.loco_client.Init()
            self.connected = True
            self.get_logger().info("[NAVIGATION] LocoClient connected!")
        except Exception as e:
            self.get_logger().error(f"[NAVIGATION] LocoClient init failed: {e}")
            raise

        # Auto-stand if configured
        if self.auto_stand:
            self.get_logger().info("[NAVIGATION] Auto-stand enabled, standing up...")
            try:
                self.loco_client.Squat2StandUp()
                time.sleep(2.0)
                self.get_logger().info("[NAVIGATION] Robot standing")
            except Exception as e:
                self.get_logger().warn(f"[NAVIGATION] Auto-stand failed: {e}")

    def _low_state_callback(self, msg: LowState_) -> None:
        """Handle incoming low-level state from SDK."""
        with self.state_lock:
            self.last_low_state = msg

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Handle incoming velocity commands.

        Args:
            msg: Twist message with linear and angular velocity
        """
        if not self.connected or self.loco_client is None:
            self.get_logger().warn("[NAVIGATION] Not connected, ignoring cmd_vel")
            return

        # Clamp velocities
        vx = self._clamp(msg.linear.x, -self.max_linear_vel, self.max_linear_vel)
        vy = self._clamp(msg.linear.y, -self.max_linear_vel, self.max_linear_vel)
        omega = self._clamp(msg.angular.z, -self.max_angular_vel, self.max_angular_vel)

        # Send to robot
        try:
            self.loco_client.Move(vx, vy, omega)
            self.last_cmd_time = time.time()
        except Exception as e:
            self.get_logger().error(f"[NAVIGATION] Move command failed: {e}")

    def _check_cmd_timeout(self) -> None:
        """Check for cmd_vel timeout and stop robot if needed."""
        if self.last_cmd_time is None:
            return

        time_since_cmd = time.time() - self.last_cmd_time

        if time_since_cmd > self.cmd_vel_timeout:
            # Stop the robot
            if self.connected and self.loco_client is not None:
                try:
                    self.loco_client.StopMove()
                except Exception as e:
                    self.get_logger().warn(f"[NAVIGATION] StopMove failed: {e}")
            self.last_cmd_time = None

    def _publish_imu(self) -> None:
        """Publish IMU data from robot state."""
        with self.state_lock:
            state = self.last_low_state

        if state is None:
            return

        imu = state.imu_state
        current_time = self.get_clock().now()

        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        # Orientation from RPY
        # Note: SDK provides rpy as [roll, pitch, yaw]
        roll = imu.rpy[0]
        pitch = imu.rpy[1]
        yaw = imu.rpy[2]

        # Convert Euler to quaternion (ZYX convention)
        qw, qx, qy, qz = self._euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz

        # Angular velocity (gyroscope)
        imu_msg.angular_velocity.x = imu.gyroscope[0]
        imu_msg.angular_velocity.y = imu.gyroscope[1]
        imu_msg.angular_velocity.z = imu.gyroscope[2]

        # Linear acceleration
        imu_msg.linear_acceleration.x = imu.accelerometer[0]
        imu_msg.linear_acceleration.y = imu.accelerometer[1]
        imu_msg.linear_acceleration.z = imu.accelerometer[2]

        # Covariance (unknown, use -1 for first element)
        # Or set diagonal values for known uncertainty
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.01

        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01

        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(imu_msg)

    def _publish_joint_states(self) -> None:
        """Publish joint states from robot state."""
        with self.state_lock:
            state = self.last_low_state

        if state is None:
            return

        current_time = self.get_clock().now()

        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.header.frame_id = ''

        joint_msg.name = JOINT_NAMES
        joint_msg.position = []
        joint_msg.velocity = []
        joint_msg.effort = []

        for i in range(G1_NUM_MOTOR):
            motor = state.motor_state[i]
            joint_msg.position.append(motor.q)
            joint_msg.velocity.append(motor.dq)
            joint_msg.effort.append(motor.tau_est if hasattr(motor, 'tau_est') else 0.0)

        self.joint_pub.publish(joint_msg)

    def shutdown(self) -> None:
        """Gracefully shutdown and put robot in safe mode."""
        self.get_logger().info("[NAVIGATION] Shutting down - entering damp mode...")

        if self.connected and self.loco_client is not None:
            try:
                self.loco_client.Damp()
                time.sleep(1.0)
                self.get_logger().info("[NAVIGATION] Robot in damp mode. Safe to disconnect.")
            except Exception as e:
                self.get_logger().error(f"[NAVIGATION] Damp failed: {e}")
                self.get_logger().error("[NAVIGATION] Manually verify robot is safe!")

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range [min_val, max_val]."""
        return max(min_val, min(max_val, value))

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
        """
        Convert Euler angles (ZYX convention) to quaternion.

        Args:
            roll: Roll angle (rotation around X) in radians
            pitch: Pitch angle (rotation around Y) in radians
            yaw: Yaw angle (rotation around Z) in radians

        Returns:
            Tuple of (w, x, y, z)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (w, x, y, z)


def main(args=None):
    """Main entry point for hardware_bridge node."""
    rclpy.init(args=args)

    node = None
    shutdown_requested = threading.Event()

    def signal_handler(signum, frame):
        """Handle SIGTERM and SIGINT for graceful shutdown."""
        signal_name = signal.Signals(signum).name
        print(f"[NAVIGATION] Received {signal_name}, initiating graceful shutdown...")
        shutdown_requested.set()
        # Request rclpy to stop spinning
        if rclpy.ok():
            rclpy.shutdown()

    # Register signal handlers for both SIGINT and SIGTERM
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node = HardwareBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if not shutdown_requested.is_set():
            print(f"[NAVIGATION] Error: {e}")
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
