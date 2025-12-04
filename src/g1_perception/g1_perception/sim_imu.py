# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Simulated IMU node for G1 Inspector.

Publishes simulated IMU data at high rate.
Simulates typical MEMS IMU characteristics with noise.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion


class SimImuNode(Node):
    """
    Simulated IMU publisher node.

    Publishes IMU data to /g1/imu/data at 100 Hz.
    Simulates realistic IMU behavior with configurable noise.
    """

    # IMU specifications
    DEFAULT_FPS = 100.0  # 100 Hz as per story requirements

    # Noise parameters (typical MEMS IMU)
    ACCEL_NOISE_DENSITY = 0.002  # m/s^2/sqrt(Hz)
    GYRO_NOISE_DENSITY = 0.001  # rad/s/sqrt(Hz)
    ACCEL_BIAS_INSTABILITY = 0.0001  # m/s^2
    GYRO_BIAS_INSTABILITY = 0.00001  # rad/s

    # Gravity
    GRAVITY = 9.81  # m/s^2

    def __init__(self) -> None:
        super().__init__('sim_imu')

        # Declare parameters
        self.declare_parameter('fps', self.DEFAULT_FPS)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('add_noise', True)

        # Get parameters
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.add_noise = self.get_parameter('add_noise').value

        # Publisher
        self.imu_pub = self.create_publisher(
            Imu,
            '/g1/imu/data',
            10
        )

        # IMU state
        self.orientation_q = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z (identity)

        # Bias states (random walk)
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])

        # Time tracking for bias random walk
        self.last_time = self.get_clock().now()

        # Frame counter
        self.frame_count = 0

        # Timer for publishing
        publish_period = 1.0 / self.fps
        self.timer = self.create_timer(publish_period, self._publish_imu)

        self.get_logger().info(
            f"[SIMULATION] SimImuNode initialized"
            f" - Rate: {self.fps} Hz"
            f" - Noise: {self.add_noise}"
        )

    def _publish_imu(self) -> None:
        """Publish simulated IMU data."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update bias random walk
        if self.add_noise:
            self._update_biases(dt)

        # Create IMU message
        msg = self._create_imu_message(current_time.to_msg())
        self.imu_pub.publish(msg)

        self.frame_count += 1

    def _update_biases(self, dt: float) -> None:
        """Update bias values with random walk."""
        # Accelerometer bias random walk
        accel_bias_walk = np.random.normal(0, self.ACCEL_BIAS_INSTABILITY * math.sqrt(dt), 3)
        self.accel_bias += accel_bias_walk

        # Gyroscope bias random walk
        gyro_bias_walk = np.random.normal(0, self.GYRO_BIAS_INSTABILITY * math.sqrt(dt), 3)
        self.gyro_bias += gyro_bias_walk

    def _create_imu_message(self, timestamp) -> Imu:
        """Create IMU message with simulated data."""
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id

        # Orientation (identity quaternion - stationary robot)
        # In real implementation, this would come from sensor fusion
        msg.orientation.w = self.orientation_q[0]
        msg.orientation.x = self.orientation_q[1]
        msg.orientation.y = self.orientation_q[2]
        msg.orientation.z = self.orientation_q[3]

        # Orientation covariance (-1 indicates unknown)
        # We provide a valid covariance for stationary robot
        orientation_cov = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        msg.orientation_covariance = orientation_cov

        # Angular velocity (should be near zero for stationary robot)
        angular_vel = np.array([0.0, 0.0, 0.0])
        if self.add_noise:
            # Add noise
            noise = np.random.normal(0, self.GYRO_NOISE_DENSITY * math.sqrt(self.fps), 3)
            angular_vel += noise + self.gyro_bias

        msg.angular_velocity.x = float(angular_vel[0])
        msg.angular_velocity.y = float(angular_vel[1])
        msg.angular_velocity.z = float(angular_vel[2])

        # Angular velocity covariance
        gyro_var = (self.GYRO_NOISE_DENSITY ** 2) * self.fps
        msg.angular_velocity_covariance = [
            gyro_var, 0, 0,
            0, gyro_var, 0,
            0, 0, gyro_var
        ]

        # Linear acceleration (gravity in z for stationary robot)
        # Accelerometer measures specific force = acceleration - gravity
        # For stationary robot: a = 0, so we measure -g = [0, 0, +g] in body frame
        linear_accel = np.array([0.0, 0.0, self.GRAVITY])
        if self.add_noise:
            # Add noise
            noise = np.random.normal(0, self.ACCEL_NOISE_DENSITY * math.sqrt(self.fps), 3)
            linear_accel += noise + self.accel_bias

        msg.linear_acceleration.x = float(linear_accel[0])
        msg.linear_acceleration.y = float(linear_accel[1])
        msg.linear_acceleration.z = float(linear_accel[2])

        # Linear acceleration covariance
        accel_var = (self.ACCEL_NOISE_DENSITY ** 2) * self.fps
        msg.linear_acceleration_covariance = [
            accel_var, 0, 0,
            0, accel_var, 0,
            0, 0, accel_var
        ]

        return msg

    def set_orientation(self, roll: float, pitch: float, yaw: float) -> None:
        """
        Set IMU orientation from Euler angles.

        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
        """
        # Convert Euler to quaternion (ZYX convention)
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        self.orientation_q = [
            cr * cp * cy + sr * sp * sy,  # w
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy   # z
        ]


def main(args=None):
    """Main entry point for sim_imu node."""
    rclpy.init(args=args)

    try:
        node = SimImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[SIMULATION] IMU error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
