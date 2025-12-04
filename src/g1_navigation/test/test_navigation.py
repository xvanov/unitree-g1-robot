# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Test suite for g1_navigation package.

Tests for Nav2 integration, loco bridge, and simulation locomotion.
"""

import math
import pytest


class TestNavigation:
    """Test cases for navigation functionality."""

    def test_package_imports(self):
        """Test that g1_navigation package can be imported."""
        import g1_navigation
        assert g1_navigation is not None

    def test_sim_locomotion_imports(self):
        """Test that sim_locomotion module can be imported."""
        from g1_navigation import sim_locomotion
        assert sim_locomotion is not None

    def test_sim_locomotion_controller_class_exists(self):
        """Test that SimLocomotionController class exists."""
        from g1_navigation.sim_locomotion import SimLocomotionController
        assert SimLocomotionController is not None


class TestSimLocomotionController:
    """Test cases for SimLocomotionController."""

    def test_velocity_clamping(self):
        """Test that velocity clamping function works correctly."""
        from g1_navigation.sim_locomotion import SimLocomotionController

        # Test clamp function
        assert SimLocomotionController._clamp(1.0, -0.5, 0.5) == 0.5
        assert SimLocomotionController._clamp(-1.0, -0.5, 0.5) == -0.5
        assert SimLocomotionController._clamp(0.3, -0.5, 0.5) == 0.3

    def test_yaw_to_quaternion(self):
        """Test yaw to quaternion conversion."""
        from g1_navigation.sim_locomotion import SimLocomotionController

        # Test identity (yaw = 0)
        w, x, y, z = SimLocomotionController._yaw_to_quaternion(0.0)
        assert abs(w - 1.0) < 1e-6
        assert abs(x) < 1e-6
        assert abs(y) < 1e-6
        assert abs(z) < 1e-6

        # Test 90 degree rotation
        w, x, y, z = SimLocomotionController._yaw_to_quaternion(math.pi / 2)
        assert abs(w - math.cos(math.pi / 4)) < 1e-6
        assert abs(z - math.sin(math.pi / 4)) < 1e-6

        # Test 180 degree rotation
        w, x, y, z = SimLocomotionController._yaw_to_quaternion(math.pi)
        assert abs(w) < 1e-6
        assert abs(z - 1.0) < 1e-6

    def test_velocity_constants(self):
        """Test that velocity constants match architecture requirements."""
        from g1_navigation.sim_locomotion import SimLocomotionController

        assert SimLocomotionController.MAX_LINEAR_VELOCITY == 0.5
        assert SimLocomotionController.MAX_ANGULAR_VELOCITY == 1.0
        assert SimLocomotionController.CMD_VEL_TIMEOUT == 0.5


class TestLocoBridge:
    """Test cases for Nav2 to LocoClient bridge."""

    def test_placeholder(self):
        """Placeholder test - implementation in Story 3."""
        # Bridge node tests will be added in Story 3
        pass


class TestHardwareBridge:
    """Test cases for hardware_bridge module (Story 1.2.5).

    Note: These tests require the unitree_sdk2_python package to be installed.
    Tests will be skipped if SDK is not available.
    """

    @pytest.fixture(autouse=True)
    def check_sdk(self):
        """Skip tests if SDK is not available."""
        try:
            from g1_navigation import hardware_bridge
            if not hardware_bridge.SDK_AVAILABLE:
                pytest.skip("Unitree SDK2 not installed - skipping hardware bridge tests")
        except ImportError:
            pytest.skip("Unitree SDK2 not installed - skipping hardware bridge tests")

    def test_hardware_bridge_imports(self):
        """Test that hardware_bridge module can be imported."""
        from g1_navigation import hardware_bridge
        assert hardware_bridge is not None

    def test_hardware_bridge_class_exists(self):
        """Test that HardwareBridge class exists."""
        from g1_navigation.hardware_bridge import HardwareBridge
        assert HardwareBridge is not None

    def test_sdk_available_flag_defined(self):
        """Test that SDK availability flag is defined."""
        from g1_navigation.hardware_bridge import SDK_AVAILABLE
        assert isinstance(SDK_AVAILABLE, bool)

    def test_joint_names_defined(self):
        """Test that joint names are properly defined."""
        from g1_navigation.hardware_bridge import JOINT_NAMES, G1_NUM_MOTOR
        assert len(JOINT_NAMES) == G1_NUM_MOTOR
        assert G1_NUM_MOTOR == 29

    def test_velocity_clamping(self):
        """Test that velocity clamping function works correctly."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Test clamp function
        assert HardwareBridge._clamp(1.0, -0.5, 0.5) == 0.5
        assert HardwareBridge._clamp(-1.0, -0.5, 0.5) == -0.5
        assert HardwareBridge._clamp(0.3, -0.5, 0.5) == 0.3
        assert HardwareBridge._clamp(0.0, -0.5, 0.5) == 0.0

    def test_euler_to_quaternion_identity(self):
        """Test Euler to quaternion conversion for identity rotation."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Test identity (all angles = 0)
        w, x, y, z = HardwareBridge._euler_to_quaternion(0.0, 0.0, 0.0)
        assert abs(w - 1.0) < 1e-6
        assert abs(x) < 1e-6
        assert abs(y) < 1e-6
        assert abs(z) < 1e-6

    def test_euler_to_quaternion_yaw_90(self):
        """Test Euler to quaternion for 90 degree yaw rotation."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Test 90 degree yaw rotation
        w, x, y, z = HardwareBridge._euler_to_quaternion(0.0, 0.0, math.pi / 2)
        assert abs(w - math.cos(math.pi / 4)) < 1e-6
        assert abs(z - math.sin(math.pi / 4)) < 1e-6

    def test_euler_to_quaternion_roll_90(self):
        """Test Euler to quaternion for 90 degree roll rotation."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Test 90 degree roll rotation
        w, x, y, z = HardwareBridge._euler_to_quaternion(math.pi / 2, 0.0, 0.0)
        assert abs(w - math.cos(math.pi / 4)) < 1e-6
        assert abs(x - math.sin(math.pi / 4)) < 1e-6

    def test_velocity_constants_conservative(self):
        """Test that velocity constants are conservative for safety."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Hardware bridge uses conservative limits (0.3 m/s instead of 0.5)
        assert HardwareBridge.MAX_LINEAR_VELOCITY == 0.3
        assert HardwareBridge.MAX_ANGULAR_VELOCITY == 0.5
        assert HardwareBridge.CMD_VEL_TIMEOUT == 0.5

    def test_publishing_rates(self):
        """Test that publishing rates are defined."""
        from g1_navigation.hardware_bridge import HardwareBridge

        assert HardwareBridge.IMU_PUBLISH_RATE == 100.0
        assert HardwareBridge.JOINT_PUBLISH_RATE == 50.0


class TestHardwareBridgeBehavior:
    """Test cases for hardware_bridge behavior that don't require SDK.

    These tests verify the logic of callbacks and utility functions
    without needing actual hardware or SDK connection.
    """

    def test_clamp_boundary_values(self):
        """Test clamp function handles boundary values correctly."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Test exact boundaries
        assert HardwareBridge._clamp(0.3, -0.3, 0.3) == 0.3
        assert HardwareBridge._clamp(-0.3, -0.3, 0.3) == -0.3

        # Test values just outside boundaries
        assert HardwareBridge._clamp(0.31, -0.3, 0.3) == 0.3
        assert HardwareBridge._clamp(-0.31, -0.3, 0.3) == -0.3

        # Test negative range
        assert HardwareBridge._clamp(-0.5, -1.0, -0.2) == -0.5
        assert HardwareBridge._clamp(0.0, -1.0, -0.2) == -0.2

    def test_euler_to_quaternion_pitch_90(self):
        """Test Euler to quaternion for 90 degree pitch rotation."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Test 90 degree pitch rotation
        w, x, y, z = HardwareBridge._euler_to_quaternion(0.0, math.pi / 2, 0.0)
        assert abs(w - math.cos(math.pi / 4)) < 1e-6
        assert abs(y - math.sin(math.pi / 4)) < 1e-6

    def test_euler_to_quaternion_combined_rotation(self):
        """Test Euler to quaternion with combined rotations."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Test combined 45 degree rotations
        w, x, y, z = HardwareBridge._euler_to_quaternion(
            math.pi / 4,  # 45 deg roll
            math.pi / 4,  # 45 deg pitch
            0.0           # 0 yaw
        )
        # Verify quaternion is normalized (magnitude = 1)
        magnitude = math.sqrt(w**2 + x**2 + y**2 + z**2)
        assert abs(magnitude - 1.0) < 1e-6

    def test_joint_names_lowercase_snake_case(self):
        """Test that joint names follow ROS2 snake_case convention."""
        from g1_navigation.hardware_bridge import JOINT_NAMES

        for name in JOINT_NAMES:
            # Should be lowercase
            assert name == name.lower(), f"Joint name '{name}' should be lowercase"
            # Should not contain spaces
            assert ' ' not in name, f"Joint name '{name}' should not contain spaces"
            # Should use underscores for word separation
            assert name.replace('_', '').isalnum(), f"Joint name '{name}' has invalid characters"

    def test_velocity_limits_are_safe(self):
        """Test that velocity limits are within safe operating range."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # Architecture allows up to 0.5 m/s, hardware_bridge should be <= 0.3 for safety
        assert HardwareBridge.MAX_LINEAR_VELOCITY <= 0.5, "Linear velocity exceeds architecture limit"
        assert HardwareBridge.MAX_LINEAR_VELOCITY <= 0.3, "Linear velocity should be conservative"

        # Angular velocity should be reasonable
        assert HardwareBridge.MAX_ANGULAR_VELOCITY <= 1.0, "Angular velocity too high"

        # Timeout should be reasonable (not too short, not too long)
        assert 0.1 <= HardwareBridge.CMD_VEL_TIMEOUT <= 2.0, "Timeout out of reasonable range"

    def test_publishing_rates_are_reasonable(self):
        """Test that publishing rates are appropriate for real-time control."""
        from g1_navigation.hardware_bridge import HardwareBridge

        # IMU should be high frequency for stability
        assert HardwareBridge.IMU_PUBLISH_RATE >= 50.0, "IMU rate too low"
        assert HardwareBridge.IMU_PUBLISH_RATE <= 500.0, "IMU rate too high"

        # Joint states can be lower frequency
        assert HardwareBridge.JOINT_PUBLISH_RATE >= 10.0, "Joint rate too low"
        assert HardwareBridge.JOINT_PUBLISH_RATE <= 200.0, "Joint rate too high"


class TestHardwareBridgeNoSdk:
    """Test cases that don't require SDK to be installed."""

    def test_script_syntax_hello_world(self):
        """Test that hello_world_g1.py has valid syntax."""
        import py_compile
        import os
        script_path = os.path.join(
            os.path.dirname(__file__),
            '../../../scripts/hello_world_g1.py'
        )
        # Normalize the path
        script_path = os.path.normpath(script_path)
        # This will raise SyntaxError if invalid
        py_compile.compile(script_path, doraise=True)

    def test_script_syntax_read_sensors(self):
        """Test that read_g1_sensors.py has valid syntax."""
        import py_compile
        import os
        script_path = os.path.join(
            os.path.dirname(__file__),
            '../../../scripts/read_g1_sensors.py'
        )
        # Normalize the path
        script_path = os.path.normpath(script_path)
        # This will raise SyntaxError if invalid
        py_compile.compile(script_path, doraise=True)

    def test_script_syntax_hardware_bridge(self):
        """Test that hardware_bridge.py has valid syntax."""
        import py_compile
        import os
        script_path = os.path.join(
            os.path.dirname(__file__),
            '../g1_navigation/hardware_bridge.py'
        )
        # Normalize the path
        script_path = os.path.normpath(script_path)
        # This will raise SyntaxError if invalid
        py_compile.compile(script_path, doraise=True)
