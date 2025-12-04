# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Test suite for MuJoCo simulation manager.

Tests for physics simulation, sensor publishing, and locomotion control.
"""

import math
import pytest
import numpy as np


class TestMuJoCoSimManager:
    """Test cases for MuJoCoSimManager."""

    def test_mujoco_sim_imports(self):
        """Test that mujoco_sim module can be imported."""
        from g1_bringup import mujoco_sim
        assert mujoco_sim is not None

    def test_mujoco_sim_manager_class_exists(self):
        """Test that MuJoCoSimManager class exists."""
        from g1_bringup.mujoco_sim import MuJoCoSimManager
        assert MuJoCoSimManager is not None

    def test_sim_state_class_exists(self):
        """Test that SimState class exists."""
        from g1_bringup.mujoco_sim import SimState
        assert SimState is not None

    def test_sim_state_defaults(self):
        """Test SimState default values."""
        from g1_bringup.mujoco_sim import SimState

        state = SimState()
        assert state.x == 0.0
        assert state.y == 0.0
        assert state.z == 0.793  # G1 pelvis height
        assert state.qw == 1.0
        assert state.qx == 0.0
        assert state.qy == 0.0
        assert state.qz == 0.0

    def test_velocity_limits(self):
        """Test that velocity limits match architecture requirements."""
        from g1_bringup.mujoco_sim import MuJoCoSimManager

        assert MuJoCoSimManager.MAX_LINEAR_VELOCITY == 0.5
        assert MuJoCoSimManager.MAX_ANGULAR_VELOCITY == 1.0
        assert MuJoCoSimManager.CMD_VEL_TIMEOUT == 0.5

    def test_sensor_rates(self):
        """Test that sensor rates match story requirements."""
        from g1_bringup.mujoco_sim import MuJoCoSimManager

        assert MuJoCoSimManager.ODOM_RATE == 50.0
        assert MuJoCoSimManager.CAMERA_RATE == 1.0
        assert MuJoCoSimManager.LIDAR_RATE == 10.0
        assert MuJoCoSimManager.IMU_RATE == 100.0

    def test_camera_parameters(self):
        """Test camera parameters match D435i specs."""
        from g1_bringup.mujoco_sim import MuJoCoSimManager

        assert MuJoCoSimManager.CAMERA_WIDTH == 640
        assert MuJoCoSimManager.CAMERA_HEIGHT == 480
        assert MuJoCoSimManager.CAMERA_FOV == 87.0

    def test_lidar_parameters(self):
        """Test LiDAR parameters match MID-360 specs."""
        from g1_bringup.mujoco_sim import MuJoCoSimManager

        assert MuJoCoSimManager.LIDAR_NUM_POINTS == 5000
        assert MuJoCoSimManager.LIDAR_MIN_RANGE == 0.1
        assert MuJoCoSimManager.LIDAR_MAX_RANGE == 40.0


class TestMathUtilities:
    """Test cases for math utility functions."""

    def test_clamp_function(self):
        """Test value clamping."""
        from g1_bringup.mujoco_sim import MuJoCoSimManager

        assert MuJoCoSimManager._clamp(1.0, -0.5, 0.5) == 0.5
        assert MuJoCoSimManager._clamp(-1.0, -0.5, 0.5) == -0.5
        assert MuJoCoSimManager._clamp(0.3, -0.5, 0.5) == 0.3

    def test_quaternion_to_yaw(self):
        """Test quaternion to yaw conversion."""
        from g1_bringup.mujoco_sim import MuJoCoSimManager

        # Identity quaternion -> 0 yaw
        yaw = MuJoCoSimManager._quaternion_to_yaw(1.0, 0.0, 0.0, 0.0)
        assert abs(yaw) < 1e-6

        # 90 degree rotation around z
        yaw = MuJoCoSimManager._quaternion_to_yaw(
            math.cos(math.pi/4), 0.0, 0.0, math.sin(math.pi/4)
        )
        assert abs(yaw - math.pi/2) < 1e-6

    def test_quaternion_to_rotation_matrix(self):
        """Test quaternion to rotation matrix conversion."""
        from g1_bringup.mujoco_sim import MuJoCoSimManager

        # Identity quaternion -> identity matrix
        R = MuJoCoSimManager._quaternion_to_rotation_matrix(1.0, 0.0, 0.0, 0.0)
        assert np.allclose(R, np.eye(3), atol=1e-6)


class TestMuJoCoIntegration:
    """Integration tests requiring MuJoCo."""

    def test_mujoco_available(self):
        """Test that MuJoCo library is available."""
        import mujoco
        assert mujoco is not None

    def test_can_create_simple_model(self):
        """Test that we can create a simple MuJoCo model."""
        import mujoco

        xml = """
        <mujoco>
          <worldbody>
            <body name="test" pos="0 0 1">
              <freejoint/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        assert model is not None
        assert data is not None

    def test_mj_ray_available(self):
        """Test that mj_ray (raycast) function is available."""
        import mujoco
        assert hasattr(mujoco, 'mj_ray')
