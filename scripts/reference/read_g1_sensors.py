#!/usr/bin/env python3
# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
G1 Robot sensor data reader script.

Connects to the real G1 robot via CycloneDDS and streams sensor data
including IMU readings, joint states, and battery level.

Usage:
    python3 scripts/read_g1_sensors.py <network_interface>

Example:
    python3 scripts/read_g1_sensors.py eth0

Press Ctrl+C to stop reading.
"""

import sys
import time
import signal
from datetime import datetime
from typing import Optional, Any

# SDK imports
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
except ImportError as e:
    print(f"[PERCEPTION] ERROR: Failed to import Unitree SDK2: {e}")
    print("[PERCEPTION] Make sure unitree_sdk2_python is installed:")
    print("    cd external/unitree_sdk2_python && pip install -e .")
    sys.exit(1)


# G1 Joint Index Reference (23-DOF + optional wrist)
class G1JointIndex:
    """G1 robot joint indices for 29 motor state array."""

    # Legs (12 joints)
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11

    # Waist (3 joints - WaistRoll/Pitch invalid for 23-DOF)
    WaistYaw = 12
    WaistRoll = 13   # INVALID for 23-DOF
    WaistPitch = 14  # INVALID for 23-DOF

    # Arms (14 joints - WristPitch/Yaw invalid for 23-DOF)
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # INVALID for 23-DOF
    LeftWristYaw = 21     # INVALID for 23-DOF
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # INVALID for 23-DOF
    RightWristYaw = 28    # INVALID for 23-DOF


# Joint names for display
JOINT_NAMES = [
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee",
    "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee",
    "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw", "WaistRoll*", "WaistPitch*",
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw",
    "LeftElbow", "LeftWristRoll", "LeftWristPitch*", "LeftWristYaw*",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw",
    "RightElbow", "RightWristRoll", "RightWristPitch*", "RightWristYaw*"
]

# Number of motors in G1
G1_NUM_MOTOR = 29

# Display configuration
DISPLAY_RATE_HZ = 2.0  # How often to display data (Hz)
DISPLAY_INTERVAL = 1.0 / DISPLAY_RATE_HZ  # seconds between updates
SUBSCRIBER_QUEUE_LEN = 10  # DDS subscriber queue length


class SensorReader:
    """Reads and displays sensor data from G1 robot."""

    def __init__(self, network_interface: str):
        """
        Initialize sensor reader.

        Args:
            network_interface: Network interface name (e.g., 'eth0')
        """
        self.network_interface = network_interface
        self.last_state: Optional[LowState_] = None
        self.message_count = 0
        self.last_display_time = 0.0
        self.running = True

        # Register signal handlers for clean shutdown (SIGINT and SIGTERM)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig: int, frame: Any) -> None:
        """Handle Ctrl+C gracefully."""
        print("\n[PERCEPTION] Stopping sensor reader...")
        self.running = False

    def _low_state_handler(self, msg: LowState_) -> None:
        """
        Callback for receiving low-level state messages.

        Args:
            msg: LowState_ message from robot
        """
        self.last_state = msg
        self.message_count += 1

    def _format_timestamp(self) -> str:
        """Get formatted timestamp string."""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _display_imu_data(self, state: LowState_) -> None:
        """
        Display IMU data from robot state.

        Args:
            state: LowState_ message containing IMU data
        """
        imu = state.imu_state

        # Roll, Pitch, Yaw in radians
        rpy = imu.rpy
        print(f"[PERCEPTION] IMU RPY: roll={rpy[0]:+.4f}, pitch={rpy[1]:+.4f}, yaw={rpy[2]:+.4f} rad")

        # Gyroscope (angular velocity)
        gyro = imu.gyroscope
        print(f"[PERCEPTION] Gyroscope: x={gyro[0]:+.4f}, y={gyro[1]:+.4f}, z={gyro[2]:+.4f} rad/s")

        # Accelerometer
        accel = imu.accelerometer
        print(f"[PERCEPTION] Accelerometer: x={accel[0]:+.4f}, y={accel[1]:+.4f}, z={accel[2]:+.4f} m/s²")

    def _display_joint_states(self, state: LowState_, verbose: bool = False) -> None:
        """
        Display joint state data.

        Args:
            state: LowState_ message containing motor states
            verbose: If True, show all joints. If False, show summary only.
        """
        motor_states = state.motor_state

        if verbose:
            print(f"[PERCEPTION] Joint States ({G1_NUM_MOTOR} joints):")
            for i in range(G1_NUM_MOTOR):
                q = motor_states[i].q
                dq = motor_states[i].dq
                print(f"         [{i:2d}] {JOINT_NAMES[i]:20s}: q={q:+.4f} rad, dq={dq:+.4f} rad/s")
        else:
            # Show just a few key joints
            key_joints = [
                (G1JointIndex.LeftHipPitch, "LeftHipPitch"),
                (G1JointIndex.RightHipPitch, "RightHipPitch"),
                (G1JointIndex.LeftKnee, "LeftKnee"),
                (G1JointIndex.RightKnee, "RightKnee"),
            ]
            print("[PERCEPTION] Key Joint States:")
            for idx, name in key_joints:
                q = motor_states[idx].q
                dq = motor_states[idx].dq
                print(f"         {name:15s}: q={q:+.4f} rad, dq={dq:+.4f} rad/s")

    def _display_battery_mode(self, state: LowState_) -> None:
        """
        Display battery level and robot mode.

        Args:
            state: LowState_ message
        """
        mode = state.mode_machine

        # Try to get battery info - field name varies by firmware version
        battery_percent = None
        try:
            # G1 uses battery_state.soc (State of Charge) as percentage
            if hasattr(state, 'battery_state') and state.battery_state is not None:
                if hasattr(state.battery_state, 'soc'):
                    battery_percent = state.battery_state.soc
                elif hasattr(state.battery_state, 'percentage'):
                    battery_percent = state.battery_state.percentage
        except (AttributeError, TypeError):
            pass

        if battery_percent is not None:
            print(f"[PERCEPTION] Battery: {battery_percent:.0f}%")
        else:
            print(f"[PERCEPTION] Battery: N/A (not available in this firmware)")

        print(f"[PERCEPTION] Robot Mode: {mode}")

    def _display_all(self, state: LowState_) -> None:
        """
        Display all sensor data with formatting.

        Args:
            state: LowState_ message
        """
        timestamp = self._format_timestamp()
        print()
        print(f"{'='*60}")
        print(f"[PERCEPTION] Data Update @ {timestamp} (msg #{self.message_count})")
        print(f"{'='*60}")

        self._display_imu_data(state)
        print()
        self._display_joint_states(state, verbose=False)
        print()
        self._display_battery_mode(state)

    def run(self) -> int:
        """
        Main loop - connect to robot and display sensor data.

        Returns:
            Exit code (0 for success, 1 for failure)
        """
        print(f"[PERCEPTION] Initializing CycloneDDS on interface: {self.network_interface}")

        try:
            ChannelFactoryInitialize(0, self.network_interface)
            print("[PERCEPTION] CycloneDDS initialized successfully")
        except Exception as e:
            print(f"[PERCEPTION] ERROR: Failed to initialize CycloneDDS: {e}")
            return 1

        print("[PERCEPTION] Creating subscriber for 'rt/lowstate' topic...")
        try:
            subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            subscriber.Init(self._low_state_handler, SUBSCRIBER_QUEUE_LEN)
            print("[PERCEPTION] Subscriber initialized - waiting for data...")
        except Exception as e:
            print(f"[PERCEPTION] ERROR: Failed to create subscriber: {e}")
            return 1

        print()
        print("[PERCEPTION] Streaming sensor data. Press Ctrl+C to stop.")
        print()

        # Wait for first message
        wait_start = time.time()
        while self.last_state is None and self.running:
            if time.time() - wait_start > 10.0:
                print("[PERCEPTION] WARNING: No data received in 10 seconds.")
                print("[PERCEPTION] Check that robot is powered on and connected.")
                wait_start = time.time()
            time.sleep(0.1)

        if not self.running:
            return 0

        print("[PERCEPTION] First message received! Streaming data...")

        # Main display loop
        while self.running:
            current_time = time.time()

            if current_time - self.last_display_time >= DISPLAY_INTERVAL:
                if self.last_state is not None:
                    self._display_all(self.last_state)
                self.last_display_time = current_time

            time.sleep(0.01)  # Small sleep to avoid busy-waiting

        print("[PERCEPTION] Sensor reader stopped.")
        print(f"[PERCEPTION] Total messages received: {self.message_count}")
        return 0


def main() -> int:
    """
    Main entry point for sensor reader script.

    Returns:
        Exit code (0 for success, 1 for failure)
    """
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <network_interface>")
        print()
        print("Examples:")
        print(f"    python3 {sys.argv[0]} eth0")
        print(f"    python3 {sys.argv[0]} enp0s31f6")
        print()
        print("Find your interface with: ip addr | grep 192.168.123")
        return 1

    network_interface = sys.argv[1]

    print("=" * 60)
    print("     G1 ROBOT SENSOR READER")
    print("=" * 60)
    print()
    print(f"Network interface: {network_interface}")
    print(f"Display rate: {DISPLAY_RATE_HZ} Hz")
    print()

    reader = SensorReader(network_interface)
    return reader.run()


if __name__ == "__main__":
    sys.exit(main())
