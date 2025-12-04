# Copyright 2024 G1 Inspector Project
# Licensed under Apache-2.0

"""
MuJoCo simulation bringup node for G1 Inspector.

Initializes MuJoCo simulation with G1 robot model from unitree_mujoco.
Provides simulated indoor environment for testing navigation and perception.
"""

import os
import time
import threading
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


class SimBringupNode(Node):
    """
    MuJoCo simulation bringup node.

    Initializes and manages the MuJoCo simulation environment with the G1 robot.
    Provides access to simulation state for other nodes (locomotion, sensors).
    """

    # Default paths relative to external/unitree_mujoco
    DEFAULT_ROBOT_MODEL = "g1"
    DEFAULT_SCENE_FILE = "unitree_robots/g1/scene.xml"

    # Simulation parameters
    SIMULATE_DT = 0.005  # 200 Hz physics
    VIEWER_DT = 0.02     # 50 Hz viewer sync

    def __init__(self) -> None:
        super().__init__('sim_bringup')

        # Declare parameters
        self.declare_parameter('headless', False)
        self.declare_parameter('scene_file', '')
        self.declare_parameter('simulate_dt', self.SIMULATE_DT)
        self.declare_parameter('viewer_dt', self.VIEWER_DT)

        # Get parameters
        self.headless = self.get_parameter('headless').value
        scene_file_param = self.get_parameter('scene_file').value
        self.simulate_dt = self.get_parameter('simulate_dt').value
        self.viewer_dt = self.get_parameter('viewer_dt').value

        # Resolve scene file path
        self.scene_file = self._resolve_scene_path(scene_file_param)

        # MuJoCo state
        self.mj_model: Optional[mujoco.MjModel] = None
        self.mj_data: Optional[mujoco.MjData] = None
        self.viewer: Optional[mujoco.viewer.Handle] = None
        self.locker = threading.Lock()

        # Control flags
        self._running = False
        self._sim_thread: Optional[threading.Thread] = None
        self._viewer_thread: Optional[threading.Thread] = None

        # Publisher for simulation status
        self.sim_ready_pub = self.create_publisher(Bool, '/g1/sim/ready', 10)

        # Initialize simulation
        self._init_simulation()

        self.get_logger().info(
            f"[SIMULATION] SimBringupNode initialized"
            f" - Scene: {self.scene_file}"
            f" - Headless: {self.headless}"
            f" - Physics dt: {self.simulate_dt}s"
        )

    def _resolve_scene_path(self, scene_file_param: str) -> str:
        """Resolve the MuJoCo scene file path."""
        if scene_file_param:
            # Use provided path
            if os.path.isabs(scene_file_param):
                return scene_file_param
            # Relative to workspace
            return str(Path.cwd() / scene_file_param)

        # Default: look for unitree_mujoco in external/
        workspace_root = Path(__file__).parent.parent.parent.parent.parent
        default_scene = workspace_root / "external" / "unitree_mujoco" / self.DEFAULT_SCENE_FILE

        if default_scene.exists():
            return str(default_scene)

        # Fallback: create simple scene
        self.get_logger().warn(
            f"[SIMULATION] Default scene not found at {default_scene}, using simple environment"
        )
        return self._create_simple_scene()

    def _create_simple_scene(self) -> str:
        """Create a simple indoor scene for simulation."""
        scene_xml = """
<mujoco model="g1_simple_scene">
  <compiler angle="radian"/>

  <option timestep="0.005" gravity="0 0 -9.81"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <material name="wall" rgba="0.8 0.8 0.8 1"/>
    <material name="obstacle" rgba="0.6 0.4 0.2 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>

    <!-- Ground plane -->
    <geom name="floor" size="10 10 0.05" type="plane" material="groundplane"/>

    <!-- Indoor walls (10m x 10m room) -->
    <geom name="wall_north" type="box" size="10 0.1 1.5" pos="0 10 0.75" material="wall"/>
    <geom name="wall_south" type="box" size="10 0.1 1.5" pos="0 -10 0.75" material="wall"/>
    <geom name="wall_east" type="box" size="0.1 10 1.5" pos="10 0 0.75" material="wall"/>
    <geom name="wall_west" type="box" size="0.1 10 1.5" pos="-10 0 0.75" material="wall"/>

    <!-- Obstacles for navigation testing -->
    <geom name="obstacle_1" type="box" size="0.5 0.5 0.5" pos="3 2 0.25" material="obstacle"/>
    <geom name="obstacle_2" type="cylinder" size="0.3 0.6" pos="-2 3 0.3" material="obstacle"/>
    <geom name="obstacle_3" type="box" size="1.0 0.3 0.4" pos="0 -3 0.2" material="obstacle"/>

    <!-- Simple robot base (placeholder for G1) -->
    <body name="base_link" pos="0 0 0.5">
      <freejoint name="base_joint"/>
      <geom name="base_geom" type="box" size="0.3 0.2 0.4" rgba="0.2 0.2 0.8 1"/>

      <!-- Sensor mount points -->
      <body name="imu_link" pos="0 0 0.3">
        <geom type="box" size="0.02 0.02 0.02" rgba="1 0 0 1"/>
      </body>

      <body name="lidar_link" pos="0 0 0.5">
        <geom type="cylinder" size="0.05 0.03" rgba="0 1 0 1"/>
      </body>

      <body name="camera_link" pos="0.2 0 0.2">
        <geom type="box" size="0.03 0.08 0.02" rgba="0 0 1 1"/>
        <body name="camera_rgb_frame" pos="0.01 0 0"/>
        <body name="camera_depth_frame" pos="0.01 0 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
        # Write to temp file
        scene_path = "/tmp/g1_simple_scene.xml"
        with open(scene_path, 'w') as f:
            f.write(scene_xml)
        return scene_path

    def _init_simulation(self) -> None:
        """Initialize MuJoCo simulation."""
        try:
            # Load model
            self.mj_model = mujoco.MjModel.from_xml_path(self.scene_file)
            self.mj_data = mujoco.MjData(self.mj_model)

            # Set simulation timestep
            self.mj_model.opt.timestep = self.simulate_dt

            # Initialize viewer if not headless
            if not self.headless:
                self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
                time.sleep(0.2)  # Wait for viewer to initialize

            self._running = True

            # Start simulation threads
            self._sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
            self._sim_thread.start()

            if self.viewer:
                self._viewer_thread = threading.Thread(target=self._viewer_loop, daemon=True)
                self._viewer_thread.start()

            # Publish ready status
            ready_msg = Bool()
            ready_msg.data = True
            self.sim_ready_pub.publish(ready_msg)

            self.get_logger().info("[SIMULATION] MuJoCo simulation initialized successfully")

        except Exception as e:
            self.get_logger().error(f"[SIMULATION] Failed to initialize MuJoCo: {e}")
            raise

    def _simulation_loop(self) -> None:
        """Main simulation physics loop."""
        while self._running:
            if self.viewer and not self.viewer.is_running():
                self._running = False
                break

            step_start = time.perf_counter()

            with self.locker:
                mujoco.mj_step(self.mj_model, self.mj_data)

            # Maintain real-time physics
            elapsed = time.perf_counter() - step_start
            sleep_time = self.simulate_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _viewer_loop(self) -> None:
        """Viewer synchronization loop."""
        while self._running and self.viewer and self.viewer.is_running():
            with self.locker:
                self.viewer.sync()
            time.sleep(self.viewer_dt)

    def get_simulation_state(self) -> tuple:
        """
        Get current simulation state (thread-safe).

        Returns:
            Tuple of (mj_model, mj_data, locker) for external access
        """
        return self.mj_model, self.mj_data, self.locker

    def set_base_pose(self, x: float, y: float, theta: float) -> None:
        """
        Set robot base pose directly (teleport).

        Args:
            x: X position in meters
            y: Y position in meters
            theta: Yaw angle in radians
        """
        if self.mj_data is None:
            return

        with self.locker:
            # Find base_link body and set position
            try:
                base_id = self.mj_model.body('base_link').id
                # Set position (qpos for freejoint: x, y, z, qw, qx, qy, qz)
                # Keep z height, update x, y and quaternion for yaw
                self.mj_data.qpos[0] = x
                self.mj_data.qpos[1] = y
                # Convert yaw to quaternion (rotation around z-axis)
                self.mj_data.qpos[3] = np.cos(theta / 2)  # qw
                self.mj_data.qpos[4] = 0.0  # qx
                self.mj_data.qpos[5] = 0.0  # qy
                self.mj_data.qpos[6] = np.sin(theta / 2)  # qz
            except Exception as e:
                self.get_logger().warn(f"[SIMULATION] Failed to set base pose: {e}")

    def get_base_pose(self) -> tuple:
        """
        Get current robot base pose.

        Returns:
            Tuple of (x, y, z, qw, qx, qy, qz)
        """
        if self.mj_data is None:
            return (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

        with self.locker:
            return (
                self.mj_data.qpos[0],
                self.mj_data.qpos[1],
                self.mj_data.qpos[2],
                self.mj_data.qpos[3],
                self.mj_data.qpos[4],
                self.mj_data.qpos[5],
                self.mj_data.qpos[6],
            )

    def shutdown(self) -> None:
        """Shutdown simulation gracefully."""
        self._running = False

        if self._sim_thread:
            self._sim_thread.join(timeout=1.0)
        if self._viewer_thread:
            self._viewer_thread.join(timeout=1.0)

        if self.viewer:
            self.viewer.close()

        self.get_logger().info("[SIMULATION] Simulation shutdown complete")


# Global simulation instance for inter-node communication
_sim_instance: Optional[SimBringupNode] = None


def get_sim_instance() -> Optional[SimBringupNode]:
    """Get the global simulation instance."""
    return _sim_instance


def main(args=None):
    """Main entry point for sim_bringup node."""
    global _sim_instance

    rclpy.init(args=args)

    try:
        node = SimBringupNode()
        _sim_instance = node

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[SIMULATION] Error: {e}")
    finally:
        if _sim_instance:
            _sim_instance.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
