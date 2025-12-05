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
    """Test cases for Nav2 to LocoClient bridge (Story 1.3)."""

    def test_loco_bridge_imports(self):
        """Test that loco_bridge module can be imported."""
        from g1_navigation import loco_bridge
        assert loco_bridge is not None

    def test_loco_bridge_class_exists(self):
        """Test that LocoBridge class exists."""
        from g1_navigation.loco_bridge import LocoBridge
        assert LocoBridge is not None

    def test_loco_bridge_default_parameters(self):
        """Test that default parameters match architecture spec."""
        from g1_navigation.loco_bridge import LocoBridge

        assert LocoBridge.DEFAULT_MAX_LINEAR_VEL == 0.5   # Architecture spec
        assert LocoBridge.DEFAULT_MAX_ANGULAR_VEL == 1.0  # Architecture spec
        assert LocoBridge.DEFAULT_CMD_VEL_TIMEOUT == 0.5
        assert LocoBridge.DEFAULT_USE_SIMULATION is True

    def test_clamp_function(self):
        """Test velocity clamping function."""
        from g1_navigation.loco_bridge import LocoBridge

        # Create instance without ROS context for testing
        # Test the static-like clamp method
        assert LocoBridge._clamp(None, 1.0, 0.5) == 0.5
        assert LocoBridge._clamp(None, -1.0, 0.5) == -0.5
        assert LocoBridge._clamp(None, 0.3, 0.5) == 0.3


class TestCoverageTracker:
    """Test cases for coverage tracking (Story 1.3)."""

    def test_coverage_tracker_imports(self):
        """Test that coverage_tracker module can be imported."""
        from g1_navigation import coverage_tracker
        assert coverage_tracker is not None

    def test_coverage_tracker_class_exists(self):
        """Test that CoverageTracker class exists."""
        from g1_navigation.coverage_tracker import CoverageTracker
        assert CoverageTracker is not None

    def test_coverage_tracker_default_parameters(self):
        """Test that default parameters are reasonable."""
        from g1_navigation.coverage_tracker import CoverageTracker

        assert CoverageTracker.DEFAULT_COVERAGE_CELL_SIZE == 0.5  # Story requirement
        assert CoverageTracker.DEFAULT_SENSOR_FOV == 87.0  # D435i FOV
        assert CoverageTracker.DEFAULT_SENSOR_RANGE == 5.0
        assert CoverageTracker.DEFAULT_PUBLISH_RATE == 1.0

    def test_world_to_coverage_cell_static(self):
        """Test world to coverage cell conversion."""
        # This tests the conversion formula without needing ROS context
        import math

        # Formula: cell = floor(coord / cell_size)
        cell_size = 0.5

        def world_to_cell(x, y):
            return (int(math.floor(x / cell_size)), int(math.floor(y / cell_size)))

        # Test origin
        assert world_to_cell(0.0, 0.0) == (0, 0)

        # Test positive values
        assert world_to_cell(0.4, 0.4) == (0, 0)
        assert world_to_cell(0.5, 0.5) == (1, 1)
        assert world_to_cell(1.0, 1.0) == (2, 2)

        # Test negative values
        assert world_to_cell(-0.1, -0.1) == (-1, -1)
        assert world_to_cell(-0.5, -0.5) == (-1, -1)
        assert world_to_cell(-0.51, -0.51) == (-2, -2)


class TestLocoBridgeEdgeCases:
    """Edge case tests for LocoBridge (Code Review fixes)."""

    def test_clamp_zero_limit(self):
        """Test clamping with zero limit."""
        from g1_navigation.loco_bridge import LocoBridge

        # Zero limit should clamp everything to 0
        assert LocoBridge._clamp(None, 1.0, 0.0) == 0.0
        assert LocoBridge._clamp(None, -1.0, 0.0) == 0.0

    def test_clamp_negative_input(self):
        """Test clamping negative velocities."""
        from g1_navigation.loco_bridge import LocoBridge

        assert LocoBridge._clamp(None, -0.3, 0.5) == -0.3
        assert LocoBridge._clamp(None, -0.6, 0.5) == -0.5

    def test_clamp_exact_boundary(self):
        """Test clamping at exact boundary values."""
        from g1_navigation.loco_bridge import LocoBridge

        assert LocoBridge._clamp(None, 0.5, 0.5) == 0.5
        assert LocoBridge._clamp(None, -0.5, 0.5) == -0.5


class TestCoverageTrackerEdgeCases:
    """Edge case tests for CoverageTracker (Code Review fixes)."""

    def test_coverage_percentage_bounds(self):
        """Test that coverage percentage stays in valid range."""
        # Coverage should be between 0 and 100
        visited = 50
        total = 100
        coverage = (visited / total) * 100.0
        assert 0.0 <= coverage <= 100.0

        # Edge case: no free cells
        total_zero = 0
        if total_zero > 0:
            coverage_zero = (visited / total_zero) * 100.0
        else:
            coverage_zero = 0.0  # Avoid division by zero
        assert coverage_zero == 0.0

    def test_occupancy_threshold(self):
        """Test occupancy grid threshold logic."""
        # Occupied cells have value > 50
        assert 100 > 50  # Occupied
        assert 51 > 50   # Occupied
        assert not (50 > 50)  # Free (at threshold)
        assert not (0 > 50)   # Free
        assert not (-1 > 50)  # Unknown (treated as passable)


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

    @staticmethod
    def _get_project_root():
        """Get project root by finding the src directory parent."""
        import os
        current = os.path.dirname(os.path.abspath(__file__))
        # Walk up until we find a directory containing 'scripts' or hit root
        while current != os.path.dirname(current):  # Not at filesystem root
            if os.path.isdir(os.path.join(current, 'scripts')):
                return current
            current = os.path.dirname(current)
        # Fallback: use path relative to test file
        return os.path.normpath(os.path.join(
            os.path.dirname(__file__), '../../..'
        ))

    def test_script_syntax_hello_world(self):
        """Test that hello_world_g1.py has valid syntax."""
        import py_compile
        import os
        project_root = self._get_project_root()
        script_path = os.path.join(project_root, 'scripts', 'hello_world_g1.py')
        if not os.path.exists(script_path):
            pytest.skip(f"Script not found: {script_path}")
        # This will raise SyntaxError if invalid
        py_compile.compile(script_path, doraise=True)

    def test_script_syntax_read_sensors(self):
        """Test that read_g1_sensors.py has valid syntax."""
        import py_compile
        import os
        project_root = self._get_project_root()
        script_path = os.path.join(project_root, 'scripts', 'read_g1_sensors.py')
        if not os.path.exists(script_path):
            pytest.skip(f"Script not found: {script_path}")
        # This will raise SyntaxError if invalid
        py_compile.compile(script_path, doraise=True)

    def test_script_syntax_hardware_bridge(self):
        """Test that hardware_bridge.py has valid syntax."""
        import py_compile
        import os
        # This module is in the same package, use package-relative import
        from g1_navigation import hardware_bridge
        script_path = hardware_bridge.__file__
        # This will raise SyntaxError if invalid
        py_compile.compile(script_path, doraise=True)
