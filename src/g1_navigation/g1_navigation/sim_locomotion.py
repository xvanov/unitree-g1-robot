# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Simulated locomotion controller for G1 Inspector.

Implements teleport-based locomotion for MuJoCo simulation.
Subscribes to /g1/cmd_vel and directly sets robot pose in simulation.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class SimLocomotionController(Node):
    """
    Teleport-based locomotion controller for simulation.

    Subscribes to /g1/cmd_vel and integrates velocity commands over time
    to compute pose delta, then directly sets robot base pose.
    Publishes odometry and TF transforms.
    """

    # Velocity limits from architecture
    MAX_LINEAR_VELOCITY = 0.5   # m/s
    MAX_ANGULAR_VELOCITY = 1.0  # rad/s
    CMD_VEL_TIMEOUT = 0.5       # seconds

    # Publishing rates
    ODOM_PUBLISH_RATE = 50.0    # Hz
    CONTROL_RATE = 100.0        # Hz

    def __init__(self) -> None:
        super().__init__('sim_locomotion_controller')

        # Declare parameters
        self.declare_parameter('max_linear_velocity', self.MAX_LINEAR_VELOCITY)
        self.declare_parameter('max_angular_velocity', self.MAX_ANGULAR_VELOCITY)
        self.declare_parameter('cmd_vel_timeout', self.CMD_VEL_TIMEOUT)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        # Robot state (pose in odom frame)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity state
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # Timing
        self.last_cmd_time: Optional[float] = None
        self.last_update_time = self.get_clock().now()

        # QoS for cmd_vel (best effort for real-time)
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/g1/cmd_vel',
            self._cmd_vel_callback,
            cmd_vel_qos
        )

        # Publisher for odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/g1/odom',
            10
        )

        # TF broadcaster for odom -> base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Control loop timer
        control_period = 1.0 / self.CONTROL_RATE
        self.control_timer = self.create_timer(control_period, self._control_loop)

        # Odometry publishing timer
        odom_period = 1.0 / self.ODOM_PUBLISH_RATE
        self.odom_timer = self.create_timer(odom_period, self._publish_odom)

        self.get_logger().info(
            f"[SIMULATION] SimLocomotionController initialized"
            f" - Max linear: {self.max_linear_vel} m/s"
            f" - Max angular: {self.max_angular_vel} rad/s"
            f" - Timeout: {self.cmd_vel_timeout}s"
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Handle incoming velocity commands."""
        # Clamp velocities to limits
        self.vx = self._clamp(msg.linear.x, -self.max_linear_vel, self.max_linear_vel)
        self.vy = self._clamp(msg.linear.y, -self.max_linear_vel, self.max_linear_vel)
        self.omega = self._clamp(msg.angular.z, -self.max_angular_vel, self.max_angular_vel)

        self.last_cmd_time = time.time()

    def _control_loop(self) -> None:
        """Main control loop - integrate velocity and update pose."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time

        # Check for cmd_vel timeout
        if self.last_cmd_time is not None:
            time_since_cmd = time.time() - self.last_cmd_time
            if time_since_cmd > self.cmd_vel_timeout:
                if self.vx != 0.0 or self.vy != 0.0 or self.omega != 0.0:
                    self.get_logger().warn(
                        f"[SIMULATION] cmd_vel timeout ({time_since_cmd:.2f}s), stopping robot"
                    )
                self.vx = 0.0
                self.vy = 0.0
                self.omega = 0.0

        # Integrate velocity to update pose
        # Transform velocities from body frame to world frame
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        # World frame velocities
        vx_world = self.vx * cos_theta - self.vy * sin_theta
        vy_world = self.vx * sin_theta + self.vy * cos_theta

        # Update pose
        self.x += vx_world * dt
        self.y += vy_world * dt
        self.theta += self.omega * dt

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def _publish_odom(self) -> None:
        """Publish odometry message and TF transform."""
        current_time = self.get_clock().now()

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        qw, qx, qy, qz = self._yaw_to_quaternion(self.theta)
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        # Velocity (in body frame)
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.omega

        # Covariance (simple diagonal)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        odom_msg.twist.covariance[0] = 0.01  # vx
        odom_msg.twist.covariance[7] = 0.01  # vy
        odom_msg.twist.covariance[35] = 0.01 # omega

        self.odom_pub.publish(odom_msg)

        # Publish TF transform: odom -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = self.odom_frame_id
        tf_msg.child_frame_id = self.base_frame_id

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.w = qw
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz

        self.tf_broadcaster.sendTransform(tf_msg)

    def set_pose(self, x: float, y: float, theta: float) -> None:
        """
        Set robot pose directly (teleport).

        Args:
            x: X position in meters
            y: Y position in meters
            theta: Yaw angle in radians
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.get_logger().info(
            f"[SIMULATION] Robot teleported to: x={x:.2f}, y={y:.2f}, theta={theta:.2f}"
        )

    def get_pose(self) -> tuple:
        """
        Get current robot pose.

        Returns:
            Tuple of (x, y, theta)
        """
        return (self.x, self.y, self.theta)

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range [min_val, max_val]."""
        return max(min_val, min(max_val, value))

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> tuple:
        """
        Convert yaw angle to quaternion (rotation around z-axis).

        Args:
            yaw: Yaw angle in radians

        Returns:
            Tuple of (w, x, y, z)
        """
        return (
            math.cos(yaw / 2),  # w
            0.0,                # x
            0.0,                # y
            math.sin(yaw / 2)   # z
        )


def main(args=None):
    """Main entry point for sim_locomotion node."""
    rclpy.init(args=args)

    try:
        node = SimLocomotionController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[SIMULATION] Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
