#!/usr/bin/env python3
# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Interactive joint control for G1 robot.

Allows moving individual joints by small increments.
WARNING: This is low-level control - robot may become unstable!

Usage:
    python3 scripts/joint_control.py <network_interface>

Example:
    python3 scripts/joint_control.py eth0
    python3 scripts/joint_control.py en8

Controls:
    UP/DOWN arrows or w/s: Select joint
    LEFT/RIGHT arrows or a/d: Move joint by increment
    +/-: Change increment size
    r: Reset joint to initial position
    q: Quit (puts robot in damp mode)
"""

import sys
import time
import signal
from typing import Optional, List
import numpy as np

# SDK imports
try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
except ImportError as e:
    print(f"[JOINT] ERROR: Failed to import Unitree SDK2: {e}")
    print("[JOINT] Make sure unitree_sdk2_python is installed:")
    print("    cd external/unitree_sdk2_python && pip install -e .")
    sys.exit(1)

# Joint configuration
G1_NUM_MOTOR = 29

# PD gains (from Unitree example)
Kp = [
    60, 60, 60, 100, 40, 40,      # legs (0-5)
    60, 60, 60, 100, 40, 40,      # legs (6-11)
    60, 40, 40,                   # waist (12-14)
    40, 40, 40, 40, 40, 40, 40,   # left arm (15-21)
    40, 40, 40, 40, 40, 40, 40    # right arm (22-28)
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # left arm
    1, 1, 1, 1, 1, 1, 1   # right arm
]

# Joint names and groups for display
JOINT_INFO = [
    # Legs
    ("LeftHipPitch", "L Leg"),
    ("LeftHipRoll", "L Leg"),
    ("LeftHipYaw", "L Leg"),
    ("LeftKnee", "L Leg"),
    ("LeftAnklePitch", "L Leg"),
    ("LeftAnkleRoll", "L Leg"),
    ("RightHipPitch", "R Leg"),
    ("RightHipRoll", "R Leg"),
    ("RightHipYaw", "R Leg"),
    ("RightKnee", "R Leg"),
    ("RightAnklePitch", "R Leg"),
    ("RightAnkleRoll", "R Leg"),
    # Waist
    ("WaistYaw", "Waist"),
    ("WaistRoll*", "Waist"),   # May be locked
    ("WaistPitch*", "Waist"),  # May be locked
    # Left Arm
    ("LeftShoulderPitch", "L Arm"),
    ("LeftShoulderRoll", "L Arm"),
    ("LeftShoulderYaw", "L Arm"),
    ("LeftElbow", "L Arm"),
    ("LeftWristRoll", "L Arm"),
    ("LeftWristPitch*", "L Arm"),   # 23-DOF invalid
    ("LeftWristYaw*", "L Arm"),     # 23-DOF invalid
    # Right Arm
    ("RightShoulderPitch", "R Arm"),
    ("RightShoulderRoll", "R Arm"),
    ("RightShoulderYaw", "R Arm"),
    ("RightElbow", "R Arm"),
    ("RightWristRoll", "R Arm"),
    ("RightWristPitch*", "R Arm"),  # 23-DOF invalid
    ("RightWristYaw*", "R Arm"),    # 23-DOF invalid
]

# Safe joints to control (arms only - less risk of falling)
SAFE_JOINTS = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]  # Shoulder and elbow joints


class JointController:
    """Interactive joint controller for G1 robot."""

    def __init__(self, network_interface: str, safe_mode: bool = True):
        self.network_interface = network_interface
        self.safe_mode = safe_mode
        self.running = True

        # State
        self.low_state: Optional[LowState_] = None
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.mode_machine = 0
        self.ready = False

        # Control
        self.selected_joint = 15  # Start with left shoulder
        self.increment = 0.05  # radians (~3 degrees)
        self.target_positions: List[float] = [0.0] * G1_NUM_MOTOR
        self.initial_positions: List[float] = [0.0] * G1_NUM_MOTOR

        # Control loop timing
        self.control_dt = 0.002  # 2ms (500Hz)
        self.last_control_time = 0.0

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        print("\n[JOINT] Shutting down...")
        self.running = False

    def _low_state_handler(self, msg: LowState_):
        """Handle incoming low state messages."""
        self.low_state = msg
        if not self.ready:
            self.mode_machine = msg.mode_machine
            # Capture initial positions
            for i in range(G1_NUM_MOTOR):
                self.initial_positions[i] = msg.motor_state[i].q
                self.target_positions[i] = msg.motor_state[i].q
            self.ready = True

    def _send_command(self):
        """Send low-level command to robot."""
        if not self.ready or self.low_state is None:
            return

        for i in range(G1_NUM_MOTOR):
            self.low_cmd.mode_pr = 0  # PR mode
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.motor_cmd[i].mode = 1  # Enable
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].q = self.target_positions[i]
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = Kp[i]
            self.low_cmd.motor_cmd[i].kd = Kd[i]

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.cmd_publisher.Write(self.low_cmd)

    def _display_status(self):
        """Display current joint status."""
        print("\033[2J\033[H")  # Clear screen
        print("=" * 60)
        print("     G1 JOINT CONTROL - Use arrow keys or w/a/s/d")
        print("=" * 60)
        print()

        if self.safe_mode:
            print("[SAFE MODE] Only arm joints enabled")
        else:
            print("[FULL MODE] All joints enabled - BE CAREFUL!")
        print()

        print(f"Increment: {np.degrees(self.increment):.1f} deg  (+/- to change)")
        print()
        print("Joint                 Group    Current    Target     Diff")
        print("-" * 60)

        # Show a window of joints around selected
        start = max(0, self.selected_joint - 5)
        end = min(G1_NUM_MOTOR, self.selected_joint + 6)

        for i in range(start, end):
            name, group = JOINT_INFO[i]
            current = self.low_state.motor_state[i].q if self.low_state else 0
            target = self.target_positions[i]
            diff = target - self.initial_positions[i]

            # Check if joint is safe to control
            is_safe = i in SAFE_JOINTS
            controllable = is_safe or not self.safe_mode

            # Selection marker
            marker = ">>>" if i == self.selected_joint else "   "

            # Dim unavailable joints
            if not controllable:
                name = f"({name})"

            print(f"{marker} [{i:2d}] {name:20s} {group:8s} "
                  f"{np.degrees(current):+7.1f}° {np.degrees(target):+7.1f}° "
                  f"{np.degrees(diff):+6.1f}°")

        print()
        print("Controls: w/s=select  a/d=move  r=reset  +/-=increment  q=quit")
        if self.safe_mode:
            print("          Press 'f' to enable FULL MODE (all joints)")

    def _handle_input(self, key: str):
        """Handle keyboard input."""
        if key == 'q':
            self.running = False
        elif key in ['w', '\x1b[A']:  # Up
            self.selected_joint = max(0, self.selected_joint - 1)
        elif key in ['s', '\x1b[B']:  # Down
            self.selected_joint = min(G1_NUM_MOTOR - 1, self.selected_joint + 1)
        elif key in ['a', '\x1b[D']:  # Left - decrease
            self._move_joint(-self.increment)
        elif key in ['d', '\x1b[C']:  # Right - increase
            self._move_joint(self.increment)
        elif key == '+' or key == '=':
            self.increment = min(0.2, self.increment * 1.5)
        elif key == '-':
            self.increment = max(0.01, self.increment / 1.5)
        elif key == 'r':
            self.target_positions[self.selected_joint] = self.initial_positions[self.selected_joint]
        elif key == 'f' and self.safe_mode:
            print("\n[JOINT] WARNING: Enabling FULL MODE - all joints controllable!")
            print("[JOINT] The robot may fall if you move leg joints!")
            confirm = input("Type 'yes' to confirm: ")
            if confirm.lower() == 'yes':
                self.safe_mode = False

    def _move_joint(self, delta: float):
        """Move selected joint by delta."""
        if self.safe_mode and self.selected_joint not in SAFE_JOINTS:
            return

        self.target_positions[self.selected_joint] += delta
        # Clamp to reasonable range
        self.target_positions[self.selected_joint] = np.clip(
            self.target_positions[self.selected_joint], -np.pi, np.pi
        )

    def run(self) -> int:
        """Main control loop."""
        print(f"[JOINT] Initializing on interface: {self.network_interface}")

        try:
            ChannelFactoryInitialize(0, self.network_interface)
        except Exception as e:
            print(f"[JOINT] ERROR: Failed to initialize: {e}")
            return 1

        # Release motion control from high-level
        print("[JOINT] Releasing high-level control...")
        try:
            msc = MotionSwitcherClient()
            msc.SetTimeout(5.0)
            msc.Init()

            status, result = msc.CheckMode()
            while result.get('name'):
                msc.ReleaseMode()
                status, result = msc.CheckMode()
                time.sleep(0.5)
            print("[JOINT] High-level control released")
        except Exception as e:
            print(f"[JOINT] WARNING: Could not release mode: {e}")

        # Create publisher and subscriber
        self.cmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_publisher.Init()

        state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        state_subscriber.Init(self._low_state_handler, 10)

        # Wait for first state
        print("[JOINT] Waiting for robot state...")
        timeout = time.time() + 10.0
        while not self.ready and time.time() < timeout:
            time.sleep(0.1)

        if not self.ready:
            print("[JOINT] ERROR: No state received from robot")
            return 1

        print("[JOINT] Connected! Starting control loop...")
        print("[JOINT] Press any key to continue...")

        # Set up terminal for raw input
        import tty
        import termios
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())

            last_display = 0
            while self.running:
                current_time = time.time()

                # Send commands at control rate
                if current_time - self.last_control_time >= self.control_dt:
                    self._send_command()
                    self.last_control_time = current_time

                # Update display at lower rate
                if current_time - last_display >= 0.1:
                    self._display_status()
                    last_display = current_time

                # Check for input (non-blocking)
                import select
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    key = sys.stdin.read(1)
                    # Handle arrow keys (escape sequences)
                    if key == '\x1b':
                        key += sys.stdin.read(2)
                    self._handle_input(key)

        finally:
            # Restore terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

            print("\n[JOINT] Returning to initial positions...")
            # Smoothly return to initial
            for _ in range(100):
                for i in range(G1_NUM_MOTOR):
                    self.target_positions[i] = 0.9 * self.target_positions[i] + 0.1 * self.initial_positions[i]
                self._send_command()
                time.sleep(0.02)

            print("[JOINT] Done.")

        return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <network_interface>")
        print()
        print("Examples:")
        print(f"    python3 {sys.argv[0]} eth0")
        print(f"    python3 {sys.argv[0]} en8")
        return 1

    print("=" * 60)
    print("     G1 ROBOT JOINT CONTROL")
    print("=" * 60)
    print()
    print("WARNING: This script provides LOW-LEVEL joint control!")
    print("The robot may become unstable if you move leg joints.")
    print()
    print("SAFETY:")
    print("  - Start in SAFE MODE (arms only)")
    print("  - Keep E-stop ready")
    print("  - Clear area around robot")
    print()

    confirm = input("Press Enter to continue (Ctrl+C to abort)...")

    controller = JointController(sys.argv[1], safe_mode=True)
    return controller.run()


if __name__ == "__main__":
    sys.exit(main())
