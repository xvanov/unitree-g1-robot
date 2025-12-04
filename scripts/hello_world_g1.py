#!/usr/bin/env python3
# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Hello World script for G1 robot hardware connectivity test.

Connects to the real G1 robot via CycloneDDS and commands a simple wave gesture
to validate hardware integration. This is the simplest possible hardware test.

Usage:
    python3 scripts/hello_world_g1.py <network_interface>

Example:
    python3 scripts/hello_world_g1.py eth0

Safety Requirements:
    1. Clear area: Ensure no obstacles within 2 meters of robot
    2. E-stop ready: Know location of hardware E-stop button
    3. Supervisor present: Never run locomotion tests alone
"""

import sys
import time
from typing import Optional

# SDK imports
try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelSubscriber
    )
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
except ImportError as e:
    print(f"[HARDWARE] ERROR: Failed to import Unitree SDK2: {e}")
    print("[HARDWARE] Make sure unitree_sdk2_python is installed:")
    print("    cd external/unitree_sdk2_python && pip install -e .")
    sys.exit(1)


# Constants
DEFAULT_TIMEOUT = 10.0  # seconds
WAVE_SETTLE_TIME = 3.0  # seconds to wait after wave gesture
DAMP_SETTLE_TIME = 1.0  # seconds to wait after damp command
STAND_SETTLE_TIME = 2.0  # seconds to wait after stand up command
BATTERY_READ_TIMEOUT = 2.0  # seconds to wait for battery data


# Global to store latest state from subscriber
_latest_low_state: Optional[LowState_] = None


def _low_state_handler(msg: LowState_) -> None:
    """Callback for receiving low-level state messages."""
    global _latest_low_state
    _latest_low_state = msg


def print_banner() -> None:
    """Print safety warning banner."""
    print("=" * 60)
    print("     G1 ROBOT HELLO WORLD - HARDWARE CONNECTIVITY TEST")
    print("=" * 60)
    print()
    print("SAFETY REQUIREMENTS:")
    print("  1. Clear area: No obstacles within 2 meters of robot")
    print("  2. E-stop ready: Know location of hardware E-stop button")
    print("  3. Supervisor present: Never run locomotion tests alone")
    print()
    print("WHAT THIS SCRIPT DOES:")
    print("  - Connects to G1 robot at 192.168.123.164")
    print("  - Commands the robot to wave its hand")
    print("  - Puts robot in safe damp mode on exit")
    print()
    print("=" * 60)


def initialize_dds(network_interface: str) -> bool:
    """
    Initialize CycloneDDS with the specified network interface.

    Args:
        network_interface: Network interface name (e.g., 'eth0', 'enp0s31f6')

    Returns:
        True if initialization succeeded, False otherwise
    """
    print(f"[HARDWARE] Initializing CycloneDDS on interface: {network_interface}")
    try:
        ChannelFactoryInitialize(0, network_interface)
        print(f"[HARDWARE] CycloneDDS initialized successfully")
        return True
    except Exception as e:
        print(f"[HARDWARE] ERROR: Failed to initialize CycloneDDS: {e}")
        print(f"[HARDWARE] Check that interface '{network_interface}' exists and is on 192.168.123.x subnet")
        return False


def create_loco_client(timeout: float = DEFAULT_TIMEOUT) -> Optional[LocoClient]:
    """
    Create and initialize the LocoClient for robot control.

    Args:
        timeout: Connection timeout in seconds

    Returns:
        Initialized LocoClient, or None if initialization failed
    """
    print(f"[HARDWARE] Creating LocoClient with {timeout}s timeout...")
    try:
        client = LocoClient()
        client.SetTimeout(timeout)
        client.Init()
        print("[HARDWARE] LocoClient initialized successfully")
        return client
    except Exception as e:
        print(f"[HARDWARE] ERROR: Failed to initialize LocoClient: {e}")
        print("[HARDWARE] Check that robot is powered on and network is connected")
        return None


def print_robot_status() -> None:
    """
    Print robot connection status and battery level.

    Uses the global _latest_low_state populated by the subscriber.
    """
    global _latest_low_state

    # Create subscriber for low-level state
    print("[HARDWARE] Reading robot status...")
    try:
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(_low_state_handler, 10)
    except Exception as e:
        print(f"[HARDWARE] WARNING: Could not create state subscriber: {e}")
        return

    # Wait for data
    start_time = time.time()
    while _latest_low_state is None:
        if time.time() - start_time > BATTERY_READ_TIMEOUT:
            print("[HARDWARE] WARNING: No state data received (timeout)")
            return
        time.sleep(0.1)

    state = _latest_low_state

    # Print robot mode
    mode = state.mode_machine
    print(f"[HARDWARE] Robot Mode: {mode}")

    # Try to get battery info - field name varies by firmware version
    battery_percent = None
    try:
        if hasattr(state, 'battery_state') and state.battery_state is not None:
            if hasattr(state.battery_state, 'soc'):
                battery_percent = state.battery_state.soc
            elif hasattr(state.battery_state, 'percentage'):
                battery_percent = state.battery_state.percentage
    except (AttributeError, TypeError):
        pass

    if battery_percent is not None:
        print(f"[HARDWARE] Battery Level: {battery_percent:.0f}%")
    else:
        print("[HARDWARE] Battery Level: N/A (not available in this firmware)")


def stand_up_if_needed(client: LocoClient) -> bool:
    """
    Command robot to stand up from squat position if needed.

    Args:
        client: The LocoClient instance

    Returns:
        True if stand up completed successfully, False otherwise
    """
    print("[HARDWARE] Commanding robot to stand up...")
    try:
        client.Squat2StandUp()
        print(f"[HARDWARE] Stand up command sent. Waiting {STAND_SETTLE_TIME}s...")
        time.sleep(STAND_SETTLE_TIME)
        print("[HARDWARE] Robot should now be standing.")
        return True
    except Exception as e:
        print(f"[HARDWARE] WARNING: Stand up command failed: {e}")
        print("[HARDWARE] Robot may already be standing or in different state.")
        return True  # Continue anyway - robot may already be standing


def safe_shutdown(client: LocoClient) -> None:
    """
    Put robot in safe damp mode (motors disabled).

    Args:
        client: The LocoClient instance
    """
    print("[HARDWARE] Initiating safe shutdown - entering damp mode...")
    try:
        client.Damp()
        time.sleep(DAMP_SETTLE_TIME)
        print("[HARDWARE] Robot in damp mode. Safe to disconnect.")
    except Exception as e:
        print(f"[HARDWARE] WARNING: Error during damp command: {e}")
        print("[HARDWARE] Manually verify robot is in safe state!")


def wave_hello(client: LocoClient) -> bool:
    """
    Command the robot to perform a wave hand gesture.

    Args:
        client: The LocoClient instance

    Returns:
        True if wave completed successfully, False otherwise
    """
    print("[HARDWARE] Commanding robot to wave hand...")
    try:
        # WaveHand(turn=False) = wave without turning around
        client.WaveHand(False)
        print(f"[HARDWARE] Wave command sent. Waiting {WAVE_SETTLE_TIME}s for gesture to complete...")
        time.sleep(WAVE_SETTLE_TIME)
        print("[HARDWARE] Wave gesture complete!")
        return True
    except Exception as e:
        print(f"[HARDWARE] ERROR: Wave command failed: {e}")
        return False


def main() -> int:
    """
    Main entry point for hello world script.

    Returns:
        Exit code (0 for success, 1 for failure)
    """
    # Check command line arguments
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

    # Print safety banner
    print_banner()

    # Confirm user is ready
    print("Press Enter to continue (or Ctrl+C to abort)...")
    try:
        input()
    except KeyboardInterrupt:
        print("\n[HARDWARE] Aborted by user.")
        return 0

    # Initialize DDS
    if not initialize_dds(network_interface):
        return 1

    # Create LocoClient
    client = create_loco_client()
    if client is None:
        return 1

    # Execute wave gesture
    exit_code = 0
    try:
        print()
        print("[HARDWARE] Connected to G1 robot!")
        print()

        # Print connection status and battery level
        print_robot_status()
        print()

        # Stand up first if needed
        stand_up_if_needed(client)

        print("[HARDWARE] Robot will now wave its hand...")
        print()

        if not wave_hello(client):
            exit_code = 1

    except KeyboardInterrupt:
        print("\n[HARDWARE] Interrupted by user - entering safe mode...")
    except Exception as e:
        print(f"[HARDWARE] ERROR: Unexpected error: {e}")
        exit_code = 1
    finally:
        # Always attempt safe shutdown
        safe_shutdown(client)

    if exit_code == 0:
        print()
        print("[HARDWARE] SUCCESS: Hello World test completed!")
        print("[HARDWARE] Robot connectivity verified.")
    else:
        print()
        print("[HARDWARE] FAILED: Hello World test encountered errors.")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
