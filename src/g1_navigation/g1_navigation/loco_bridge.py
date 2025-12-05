#!/usr/bin/env python3
"""
Nav2 to Locomotion Bridge for G1 Inspector.

Story 1.3: Navigation Stack Integration
Bridges Nav2 velocity commands to the Unitree SDK or simulation.

Subscribes: /cmd_vel (geometry_msgs/Twist) from Nav2
Publishes: /g1/cmd_vel (geometry_msgs/Twist) for simulation
           OR calls LocoClient.SetVelocity() for real robot

Parameters:
    use_simulation: True for sim, False for real robot
    max_linear_velocity: Maximum linear velocity (m/s)
    max_angular_velocity: Maximum angular velocity (rad/s)
    cmd_vel_timeout: Timeout before sending stop command (s)
"""

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist


class LocoBridge(Node):
    """Bridges Nav2 cmd_vel to simulation or real robot locomotion."""

    # Default parameters matching architecture spec
    DEFAULT_MAX_LINEAR_VEL = 0.5    # m/s
    DEFAULT_MAX_ANGULAR_VEL = 1.0   # rad/s
    DEFAULT_CMD_VEL_TIMEOUT = 0.5   # seconds
    DEFAULT_USE_SIMULATION = True

    def __init__(self) -> None:
        """Initialize the LocoBridge node."""
        super().__init__('loco_bridge')

        # Declare parameters
        self.declare_parameter('use_simulation', self.DEFAULT_USE_SIMULATION)
        self.declare_parameter('max_linear_velocity', self.DEFAULT_MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_velocity', self.DEFAULT_MAX_ANGULAR_VEL)
        self.declare_parameter('cmd_vel_timeout', self.DEFAULT_CMD_VEL_TIMEOUT)
        # Note: use_sim_time is automatically declared by ROS2 when set globally

        # Get parameters
        self._use_simulation = self.get_parameter('use_simulation').value
        self._max_linear_vel = self.get_parameter('max_linear_velocity').value
        self._max_angular_vel = self.get_parameter('max_angular_velocity').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # SDK client for real robot (lazy initialized)
        self._loco_client = None
        self._sdk_initialized = False
        self._sdk_init_attempts = 0
        self._max_sdk_init_attempts = 3
        self._sdk_retry_interval = 5.0  # seconds
        self._sdk_retry_timer = None
        self._sdk_max_retries_reached = False

        # QoS profile for velocity commands
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to Nav2 cmd_vel
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            cmd_vel_qos
        )

        # Publisher for simulation mode
        if self._use_simulation:
            self._sim_cmd_vel_pub = self.create_publisher(
                Twist,
                '/g1/cmd_vel',
                cmd_vel_qos
            )
            self.get_logger().info(
                "[NAVIGATION] LocoBridge initialized in SIMULATION mode: "
                "/cmd_vel -> /g1/cmd_vel"
            )
        else:
            self._sim_cmd_vel_pub = None
            self._init_sdk()
            if self._sdk_initialized:
                self.get_logger().info(
                    "[NAVIGATION] LocoBridge initialized in REAL ROBOT mode: "
                    "/cmd_vel -> SDK LocoClient"
                )
            else:
                self.get_logger().warn(
                    "[NAVIGATION] LocoBridge started but SDK not initialized. "
                    "Will retry on first cmd_vel received."
                )

        # Timeout management
        self._last_cmd_time: Optional[float] = None
        self._timeout_timer = self.create_timer(0.1, self._check_timeout)
        self._timeout_lock = threading.Lock()

        self.get_logger().info(
            f"[NAVIGATION] Velocity limits: linear={self._max_linear_vel} m/s, "
            f"angular={self._max_angular_vel} rad/s"
        )

    def _init_sdk(self) -> None:
        """Initialize Unitree SDK for real robot control."""
        if self._sdk_initialized:
            return

        try:
            # Import SDK only when needed (not in simulation)
            from unitree_sdk2py.core.channel import ChannelFactorySingleton
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

            # Initialize channel factory
            ChannelFactorySingleton.Instance().Init(0, "eth0")

            # Create loco client
            self._loco_client = LocoClient()
            self._loco_client.Init()
            self._loco_client.SetTimeout(1.0)

            self._sdk_initialized = True
            self._sdk_init_attempts = 0  # Reset attempts on success
            self._sdk_max_retries_reached = False
            # Cancel retry timer if running
            if self._sdk_retry_timer is not None:
                self._sdk_retry_timer.cancel()
                self._sdk_retry_timer = None
            self.get_logger().info("[NAVIGATION] SDK LocoClient initialized successfully")

        except ImportError as e:
            self._sdk_init_attempts += 1
            self.get_logger().error(
                f"[NAVIGATION] Failed to import SDK: {e}. "
                "Ensure unitree_sdk2py is installed."
            )
        except Exception as e:
            self._sdk_init_attempts += 1
            self.get_logger().error(f"[NAVIGATION] Failed to initialize SDK: {e}")

    def _clamp(self, value: float, limit: float) -> float:
        """Clamp value to [-limit, limit]."""
        return max(-limit, min(limit, value))

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Handle incoming velocity commands from Nav2.

        Clamps velocities to architecture limits and forwards to sim or SDK.

        Args:
            msg: Velocity command from Nav2
        """
        with self._timeout_lock:
            self._last_cmd_time = self.get_clock().now().nanoseconds / 1e9

        # Clamp velocities to architecture limits
        vx = self._clamp(msg.linear.x, self._max_linear_vel)
        vy = self._clamp(msg.linear.y, self._max_linear_vel)  # Should be ~0 for humanoid
        omega = self._clamp(msg.angular.z, self._max_angular_vel)

        if self._use_simulation:
            self._forward_to_sim(vx, vy, omega)
        else:
            self._send_to_sdk(vx, vy, omega)

    def _forward_to_sim(self, vx: float, vy: float, omega: float) -> None:
        """
        Forward velocity command to simulation.

        Args:
            vx: Linear X velocity (m/s)
            vy: Linear Y velocity (m/s)
            omega: Angular Z velocity (rad/s)
        """
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = omega

        self._sim_cmd_vel_pub.publish(cmd)

    def _send_to_sdk(self, vx: float, vy: float, omega: float) -> None:
        """
        Send velocity command to real robot via SDK.

        Args:
            vx: Linear X velocity (m/s)
            vy: Linear Y velocity (m/s)
            omega: Angular Z velocity (rad/s)
        """
        if not self._sdk_initialized or self._loco_client is None:
            # Attempt SDK recovery if we haven't exceeded max attempts
            if self._sdk_init_attempts < self._max_sdk_init_attempts:
                self.get_logger().warn(
                    f"[NAVIGATION] SDK not initialized, attempting recovery "
                    f"(attempt {self._sdk_init_attempts + 1}/{self._max_sdk_init_attempts})"
                )
                self._init_sdk()
                if self._sdk_initialized:
                    self.get_logger().info("[NAVIGATION] SDK recovery successful!")
                else:
                    return  # Still failed, skip this command
            else:
                # Log error less frequently after max attempts exceeded
                if not self._sdk_max_retries_reached:
                    self._sdk_max_retries_reached = True
                    self.get_logger().error(
                        "[NAVIGATION] SDK initialization failed after max attempts. "
                        "Robot is not controllable. Will retry every 30s. Check network."
                    )
                    # Schedule periodic retry every 30 seconds
                    self._schedule_sdk_retry()
                return

        try:
            # Use Move(vx, vy, omega) - the correct SDK API
            # Continuous commands at 20Hz will maintain velocity
            self._loco_client.Move(vx, vy, omega)

        except Exception as e:
            self.get_logger().error(f"[NAVIGATION] SDK Move failed: {e}")
            # Mark SDK as needing reinitialization
            self._sdk_initialized = False
            self._loco_client = None

    def _schedule_sdk_retry(self) -> None:
        """Schedule a periodic SDK reconnection attempt."""
        if self._sdk_retry_timer is not None:
            self._sdk_retry_timer.cancel()

        def retry_callback():
            if self._sdk_initialized:
                # Already connected, cancel timer
                if self._sdk_retry_timer is not None:
                    self._sdk_retry_timer.cancel()
                    self._sdk_retry_timer = None
                return

            self.get_logger().info("[NAVIGATION] Attempting SDK reconnection...")
            self._sdk_init_attempts = 0  # Reset attempts for new retry cycle
            self._sdk_max_retries_reached = False
            self._init_sdk()

            if self._sdk_initialized:
                self.get_logger().info("[NAVIGATION] SDK reconnection successful!")
                if self._sdk_retry_timer is not None:
                    self._sdk_retry_timer.cancel()
                    self._sdk_retry_timer = None

        # Retry every 30 seconds
        self._sdk_retry_timer = self.create_timer(30.0, retry_callback)

    def _check_timeout(self) -> None:
        """Check for cmd_vel timeout and send stop if needed."""
        with self._timeout_lock:
            if self._last_cmd_time is None:
                return

            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - self._last_cmd_time

            if elapsed > self._cmd_vel_timeout:
                self._send_stop()
                self._last_cmd_time = None  # Don't keep stopping

    def _send_stop(self) -> None:
        """Send zero velocity (stop) command."""
        self.get_logger().debug("[NAVIGATION] cmd_vel timeout, sending stop")

        if self._use_simulation:
            self._forward_to_sim(0.0, 0.0, 0.0)
        else:
            self._send_to_sdk(0.0, 0.0, 0.0)

    def destroy_node(self) -> None:
        """Clean up on shutdown."""
        # Send stop before shutting down
        self._send_stop()

        # Clean up SDK if initialized
        if self._loco_client is not None:
            try:
                self._loco_client.Move(0.0, 0.0, 0.0)
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)

    node = LocoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
