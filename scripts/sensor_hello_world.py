#!/usr/bin/env python3
# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Sensor Hello World - Multi-sensor validation script for G1 robot.

Captures data from G1 perception sensors (camera, LiDAR, microphone) via CLI
to validate sensor hardware integration before building perception pipelines.

Usage:
    python3 scripts/sensor_hello_world.py <network_interface> --camera
    python3 scripts/sensor_hello_world.py <network_interface> --lidar
    python3 scripts/sensor_hello_world.py <network_interface> --audio
    python3 scripts/sensor_hello_world.py <network_interface> --all
    python3 scripts/sensor_hello_world.py <network_interface> --lidar --rviz

Examples:
    python3 scripts/sensor_hello_world.py eth0 --camera
    python3 scripts/sensor_hello_world.py eth0 --all --output-dir ./my_captures
    python3 scripts/sensor_hello_world.py eth0 --audio --duration 10
    python3 scripts/sensor_hello_world.py eth0 --lidar --lidar-duration 5 --rviz

Note: IMU validation was completed in Story 1.2.5 via read_g1_sensors.py.
This script focuses on perception sensors (camera, LiDAR, audio).
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

# Default configuration
DEFAULT_OUTPUT_DIR = "./sensor_captures"
DEFAULT_AUDIO_DURATION = 5.0  # seconds
DEFAULT_LIDAR_DURATION = 2.0  # seconds
DEFAULT_AUDIO_SAMPLE_RATE = 48000  # Hz
LIDAR_TIMEOUT = 5.0  # seconds to wait for LiDAR data
CAMERA_TIMEOUT = 5.0  # seconds to wait for camera


def print_banner() -> None:
    """Print safety warning banner."""
    print("=" * 60)
    print("     G1 ROBOT SENSOR HELLO WORLD - PERCEPTION VALIDATION")
    print("=" * 60)
    print()
    print("SAFETY NOTE:")
    print("  This script is READ-ONLY (no locomotion commands).")
    print("  Follow Story 1.2.5 safety procedures before hardware testing.")
    print()
    print("WHAT THIS SCRIPT DOES:")
    print("  - Captures data from perception sensors")
    print("  - Saves captures to timestamped output directory")
    print("  - Optionally displays visualization in RViz")
    print()
    print("=" * 60)


class SensorTester(ABC):
    """Abstract base class for sensor testers."""

    def __init__(self, output_dir: Path):
        """
        Initialize sensor tester.

        Args:
            output_dir: Directory to save captured data
        """
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

    @abstractmethod
    def test(self) -> bool:
        """
        Run the sensor test.

        Returns:
            True if test succeeded, False otherwise
        """
        pass

    @abstractmethod
    def get_name(self) -> str:
        """Return sensor name for display."""
        pass


class CameraTester(SensorTester):
    """Tests Intel RealSense D435 camera via pyrealsense2."""

    def __init__(self, output_dir: Path):
        super().__init__(output_dir)
        self._pipeline = None
        self._config = None

    def get_name(self) -> str:
        return "Camera (RealSense D435)"

    def test(self) -> bool:
        """Capture RGB and depth images from RealSense camera."""
        try:
            import pyrealsense2 as rs
            import numpy as np
        except ImportError as e:
            print(f"[PERCEPTION] ERROR: Failed to import pyrealsense2: {e}")
            print("[PERCEPTION] Install with: pip install pyrealsense2>=2.54.0")
            return False

        print(f"[PERCEPTION] Initializing RealSense D435 camera...")

        try:
            # Create pipeline and config
            pipeline = rs.pipeline()
            config = rs.config()

            # Enable streams
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            # Start pipeline
            try:
                profile = pipeline.start(config)
            except RuntimeError as e:
                print(f"[PERCEPTION] ERROR: Could not start camera pipeline: {e}")
                print("[PERCEPTION] Check that RealSense D435 is connected via USB")
                return False

            # Get device info
            device = profile.get_device()
            device_name = device.get_info(rs.camera_info.name)
            serial = device.get_info(rs.camera_info.serial_number)
            print(f"[PERCEPTION] Camera connected: {device_name} (S/N: {serial})")

            # Wait for auto-exposure to settle
            print("[PERCEPTION] Waiting for auto-exposure to settle...")
            for _ in range(30):  # Skip first 30 frames
                pipeline.wait_for_frames()

            # Capture frames
            print("[PERCEPTION] Capturing RGB image...")
            frames = pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("[PERCEPTION] ERROR: Could not capture frames")
                pipeline.stop()
                return False

            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Get depth scale for range info
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()

            # Calculate depth range (excluding zeros)
            valid_depth = depth_image[depth_image > 0]
            if len(valid_depth) > 0:
                min_depth = valid_depth.min() * depth_scale
                max_depth = valid_depth.max() * depth_scale
            else:
                min_depth = max_depth = 0.0

            # Save RGB image
            try:
                import cv2
                rgb_path = self.output_dir / "camera_rgb.png"
                cv2.imwrite(str(rgb_path), color_image)
                print(f"[PERCEPTION] Saved: {rgb_path} ({color_image.shape[1]}x{color_image.shape[0]})")
            except ImportError:
                # Fallback without OpenCV - save as raw numpy
                rgb_path = self.output_dir / "camera_rgb.npy"
                np.save(str(rgb_path), color_image)
                print(f"[PERCEPTION] Saved: {rgb_path} ({color_image.shape[1]}x{color_image.shape[0]}) [numpy format - install opencv-python for PNG]")

            # Save depth image
            print("[PERCEPTION] Capturing depth image...")
            try:
                import cv2
                depth_path = self.output_dir / "camera_depth.png"
                cv2.imwrite(str(depth_path), depth_image)
                print(f"[PERCEPTION] Saved: {depth_path} ({depth_image.shape[1]}x{depth_image.shape[0]}, range: {min_depth:.2f}-{max_depth:.2f}m)")
            except ImportError:
                depth_path = self.output_dir / "camera_depth_png.npy"
                np.save(str(depth_path), depth_image)
                print(f"[PERCEPTION] Saved: {depth_path} ({depth_image.shape[1]}x{depth_image.shape[0]}, range: {min_depth:.2f}-{max_depth:.2f}m)")

            # Also save depth as numpy for precise values
            depth_npy_path = self.output_dir / "camera_depth.npy"
            np.save(str(depth_npy_path), depth_image)
            print(f"[PERCEPTION] Saved: {depth_npy_path} (16-bit depth data)")

            pipeline.stop()
            print("[PERCEPTION] Camera test complete!")
            return True

        except Exception as e:
            print(f"[PERCEPTION] ERROR: Camera test failed: {e}")
            return False


class LiDARTester(SensorTester):
    """Tests Livox MID-360 LiDAR via ROS2 topic subscription."""

    def __init__(self, output_dir: Path, duration: float = DEFAULT_LIDAR_DURATION):
        super().__init__(output_dir)
        self.duration = duration
        self._points: List = []
        self._received_msg = False

    def get_name(self) -> str:
        return "LiDAR (Livox MID-360)"

    def test(self) -> bool:
        """Subscribe to utlidar/cloud topic and accumulate point cloud."""
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import PointCloud2
            import numpy as np
        except ImportError as e:
            print(f"[PERCEPTION] ERROR: Failed to import ROS2 libraries: {e}")
            print("[PERCEPTION] Make sure ROS2 Humble is sourced:")
            print("    source /opt/ros/humble/setup.bash")
            print("    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
            return False

        print(f"[PERCEPTION] Subscribing to utlidar/cloud topic...")

        # Track rclpy initialization state for proper cleanup
        rclpy_initialized = False
        node = None

        class LiDARSubscriber(Node):
            def __init__(self, parent_tester: 'LiDARTester'):
                super().__init__('sensor_hello_world_lidar')
                self.parent = parent_tester
                self.subscription = self.create_subscription(
                    PointCloud2,
                    'utlidar/cloud',
                    self.listener_callback,
                    10
                )
                self.point_clouds = []
                self.start_time = None
                self.done = False

            def listener_callback(self, msg: PointCloud2):
                if self.start_time is None:
                    self.start_time = time.time()
                    print("[PERCEPTION] LiDAR connected, accumulating point cloud...")

                self.parent._received_msg = True

                # Store raw point cloud data
                self.point_clouds.append(msg)

                elapsed = time.time() - self.start_time
                if elapsed >= self.parent.duration:
                    self.done = True

        try:
            rclpy.init()
            rclpy_initialized = True
            node = LiDARSubscriber(self)

            # Wait for data with timeout
            start_wait = time.time()
            while not self._received_msg:
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.time() - start_wait > LIDAR_TIMEOUT:
                    print(f"[PERCEPTION] ERROR: No LiDAR data received in {LIDAR_TIMEOUT}s")
                    print("[PERCEPTION] Check that:")
                    print("  1. Robot is powered on and connected")
                    print("  2. ROS2 environment is sourced")
                    print("  3. Topic 'utlidar/cloud' is available (ros2 topic list)")
                    return False

            # Accumulate for specified duration
            while not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)

            # Process accumulated point clouds
            if not node.point_clouds:
                print("[PERCEPTION] ERROR: No point cloud data accumulated")
                return False

            # Convert point clouds to numpy and combine
            all_points = []
            for pc_msg in node.point_clouds:
                points = self._pointcloud2_to_xyz(pc_msg)
                if points is not None:
                    all_points.append(points)

            if not all_points:
                print("[PERCEPTION] ERROR: Could not parse point cloud data")
                return False

            combined_points = np.vstack(all_points)
            point_count = len(combined_points)

            # Calculate bounding box
            min_bounds = combined_points.min(axis=0)
            max_bounds = combined_points.max(axis=0)

            print(f"[PERCEPTION] Captured {point_count:,} points over {self.duration:.1f} seconds")

            # Save to PCD file
            pcd_path = self.output_dir / "lidar_pointcloud.pcd"
            self._save_pcd(combined_points, pcd_path)
            print(f"[PERCEPTION] Saved: {pcd_path}")
            print(f"[PERCEPTION] Bounding box: X[{min_bounds[0]:.1f}, {max_bounds[0]:.1f}] "
                  f"Y[{min_bounds[1]:.1f}, {max_bounds[1]:.1f}] "
                  f"Z[{min_bounds[2]:.1f}, {max_bounds[2]:.1f}] meters")

            print("[PERCEPTION] LiDAR test complete!")
            return True

        except Exception as e:
            print(f"[PERCEPTION] ERROR: LiDAR test failed: {e}")
            return False

        finally:
            # Guaranteed cleanup of ROS2 resources
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if rclpy_initialized:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

    def _pointcloud2_to_xyz(self, msg) -> Optional[Any]:
        """Convert PointCloud2 message to numpy array of XYZ points."""
        import numpy as np
        import struct

        # Get point cloud dimensions
        point_step = msg.point_step
        data = msg.data

        # Find XYZ field offsets
        x_offset = y_offset = z_offset = None
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if x_offset is None or y_offset is None or z_offset is None:
            return None

        # Calculate the maximum offset we need to read (all fields + 4 bytes for float)
        max_offset_needed = max(x_offset, y_offset, z_offset) + 4

        # Extract points
        points = []
        for i in range(0, len(data), point_step):
            # Ensure we can read all three fields (x, y, z) from this point
            if i + max_offset_needed <= len(data):
                x = struct.unpack('f', data[i + x_offset:i + x_offset + 4])[0]
                y = struct.unpack('f', data[i + y_offset:i + y_offset + 4])[0]
                z = struct.unpack('f', data[i + z_offset:i + z_offset + 4])[0]
                # Filter out invalid points
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                    if not (np.isinf(x) or np.isinf(y) or np.isinf(z)):
                        points.append([x, y, z])

        return np.array(points) if points else None

    def _save_pcd(self, points: Any, path: Path) -> None:
        """Save points to PCD file format."""
        try:
            import open3d as o3d
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(str(path), pcd)
        except ImportError:
            # Fallback: write simple ASCII PCD
            with open(path, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                for p in points:
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


class AudioTester(SensorTester):
    """Tests microphone array via sounddevice."""

    def __init__(self, output_dir: Path, duration: float = DEFAULT_AUDIO_DURATION,
                 device_index: Optional[int] = None):
        super().__init__(output_dir)
        self.duration = duration
        self.device_index = device_index

    def get_name(self) -> str:
        return "Audio (Microphone Array)"

    def test(self) -> bool:
        """Record audio from microphone array."""
        try:
            import sounddevice as sd
            import soundfile as sf
            import numpy as np
        except ImportError as e:
            print(f"[PERCEPTION] ERROR: Failed to import audio libraries: {e}")
            print("[PERCEPTION] Install with: pip install sounddevice>=0.4.6 soundfile>=0.12.0")
            return False

        print("[PERCEPTION] Discovering audio devices...")

        try:
            # List available devices
            devices = sd.query_devices()

            # Show available devices for user reference
            print("[PERCEPTION] Available audio input devices:")
            for idx, dev in enumerate(devices):
                if dev['max_input_channels'] >= 1:
                    marker = " *" if dev['max_input_channels'] >= 4 else ""
                    print(f"  [{idx}] {dev['name']} ({dev['max_input_channels']} ch){marker}")

            # Use specified device index or auto-detect
            input_device = None
            input_device_idx = None

            if self.device_index is not None:
                # User specified a device
                if self.device_index < len(devices):
                    input_device = devices[self.device_index]
                    input_device_idx = self.device_index
                    if input_device['max_input_channels'] < 1:
                        print(f"[PERCEPTION] ERROR: Device {self.device_index} has no input channels")
                        return False
                    print(f"[PERCEPTION] Using specified device [{self.device_index}]")
                else:
                    print(f"[PERCEPTION] ERROR: Device index {self.device_index} out of range")
                    return False
            else:
                # Auto-detect: prefer 4+ channel device (mic array)
                for idx, dev in enumerate(devices):
                    if dev['max_input_channels'] >= 1:
                        if input_device is None or dev['max_input_channels'] > input_device['max_input_channels']:
                            input_device = dev
                            input_device_idx = idx
                        # Prefer device with 4+ channels (mic array)
                        if dev['max_input_channels'] >= 4:
                            input_device = dev
                            input_device_idx = idx
                            break

            if input_device is None:
                print("[PERCEPTION] ERROR: No audio input device found")
                print("[PERCEPTION] Use --audio-device <idx> to specify a device")
                return False

            device_name = input_device['name']
            channels = min(input_device['max_input_channels'], 4)  # Cap at 4 channels
            sample_rate = int(input_device['default_samplerate'])

            # Use default sample rate if available, otherwise fallback
            if sample_rate == 0:
                sample_rate = DEFAULT_AUDIO_SAMPLE_RATE

            print(f"[PERCEPTION] Found mic array: {device_name}")
            print(f"[PERCEPTION] Recording audio for {self.duration} seconds...")

            # Record audio
            recording = sd.rec(
                int(self.duration * sample_rate),
                samplerate=sample_rate,
                channels=channels,
                device=input_device_idx,
                dtype='float32'
            )
            sd.wait()

            print("[PERCEPTION] Recording complete")

            # Save to WAV file
            wav_path = self.output_dir / "audio_recording.wav"
            sf.write(str(wav_path), recording, sample_rate)

            print(f"[PERCEPTION] Saved: {wav_path}")
            print(f"[PERCEPTION] Audio: {self.duration:.1f}s, {sample_rate} Hz, {channels} channels")
            print("[PERCEPTION] Audio test complete!")
            return True

        except Exception as e:
            print(f"[PERCEPTION] ERROR: Audio test failed: {e}")
            return False


class RVizLauncher:
    """Handles RViz visualization for sensor data."""

    def __init__(self, rviz_config_path: Optional[Path] = None):
        self.rviz_config_path = rviz_config_path
        self._process: Optional[subprocess.Popen] = None

    def launch(self) -> bool:
        """Launch RViz with sensor visualization config."""
        print("[PERCEPTION] Launching RViz for visualization...")

        try:
            cmd = ["ros2", "run", "rviz2", "rviz2"]

            if self.rviz_config_path and self.rviz_config_path.exists():
                cmd.extend(["-d", str(self.rviz_config_path)])
                print(f"[PERCEPTION] Using config: {self.rviz_config_path}")
            else:
                print("[PERCEPTION] No config file, launching with defaults")
                print("[PERCEPTION] Set Fixed Frame to 'utlidar_lidar' and add PointCloud2 display for 'utlidar/cloud'")

            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            # Give RViz time to start
            time.sleep(2.0)

            if self._process.poll() is not None:
                print("[PERCEPTION] ERROR: RViz failed to start")
                return False

            print("[PERCEPTION] RViz launched successfully")
            print("[PERCEPTION] Press Ctrl+C to stop visualization")
            return True

        except FileNotFoundError:
            print("[PERCEPTION] ERROR: RViz not found. Make sure ROS2 is sourced:")
            print("    source /opt/ros/humble/setup.bash")
            return False
        except Exception as e:
            print(f"[PERCEPTION] ERROR: Failed to launch RViz: {e}")
            return False

    def wait(self) -> None:
        """Wait for RViz to be closed or interrupted."""
        if self._process:
            try:
                self._process.wait()
            except KeyboardInterrupt:
                pass

    def stop(self) -> None:
        """Stop RViz process."""
        if self._process:
            print("[PERCEPTION] Stopping RViz...")
            self._process.terminate()
            try:
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None


def create_metadata(output_dir: Path, sensors_tested: List[str], args: argparse.Namespace) -> None:
    """Create metadata JSON file for the capture session."""
    metadata = {
        "timestamp": datetime.now().isoformat(),
        "sensors_tested": sensors_tested,
        "parameters": {
            "audio_duration": args.duration,
            "lidar_duration": args.lidar_duration,
            "rviz_enabled": args.rviz,
        },
        "output_directory": str(output_dir),
    }

    metadata_path = output_dir / "capture_metadata.json"
    with open(metadata_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    print(f"[PERCEPTION] Saved: {metadata_path}")


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="G1 Robot Sensor Hello World - Multi-sensor validation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    %(prog)s eth0 --camera           # Test camera only
    %(prog)s eth0 --lidar            # Test LiDAR only
    %(prog)s eth0 --audio            # Test audio only
    %(prog)s eth0 --all              # Test all sensors
    %(prog)s eth0 --lidar --rviz     # Test LiDAR with RViz visualization
    %(prog)s eth0 --all --rviz       # Test all with visualization
    %(prog)s eth0 --audio --duration 10  # Record 10 seconds of audio
        """
    )

    parser.add_argument(
        "network_interface",
        help="Network interface for robot connection (e.g., eth0, enp0s31f6)"
    )

    # Sensor selection
    parser.add_argument(
        "--camera",
        action="store_true",
        help="Test camera (RealSense D435)"
    )
    parser.add_argument(
        "--lidar",
        action="store_true",
        help="Test LiDAR (Livox MID-360 via utlidar/cloud topic)"
    )
    parser.add_argument(
        "--audio",
        action="store_true",
        help="Test audio (microphone array)"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Test all sensors"
    )

    # Visualization
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="Launch RViz for point cloud visualization"
    )

    # Parameters
    parser.add_argument(
        "--output-dir",
        type=str,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory for captured data (default: {DEFAULT_OUTPUT_DIR})"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_AUDIO_DURATION,
        help=f"Audio recording duration in seconds (default: {DEFAULT_AUDIO_DURATION})"
    )
    parser.add_argument(
        "--lidar-duration",
        type=float,
        default=DEFAULT_LIDAR_DURATION,
        help=f"LiDAR point cloud accumulation duration in seconds (default: {DEFAULT_LIDAR_DURATION})"
    )
    parser.add_argument(
        "--audio-device",
        type=int,
        default=None,
        help="Audio device index to use (run with --audio to see available devices)"
    )

    args = parser.parse_args()

    # If --all is specified, enable all sensors
    if args.all:
        args.camera = True
        args.lidar = True
        args.audio = True

    # Check that at least one sensor is selected
    if not (args.camera or args.lidar or args.audio):
        parser.error("At least one sensor must be selected (--camera, --lidar, --audio, or --all)")

    return args


def main() -> int:
    """Main entry point."""
    args = parse_args()

    # Print banner
    print_banner()

    # Log network interface (used for documentation/debugging purposes)
    # Note: Camera uses USB, Audio uses ALSA, LiDAR uses ROS2 DDS discovery
    # The network_interface is kept for consistency with other G1 scripts
    # and may be used in future for SDK-based sensor access
    print(f"[PERCEPTION] Network interface: {args.network_interface}")
    print(f"[PERCEPTION] Note: Sensors use USB/ALSA/ROS2 discovery, not direct SDK connection")

    # Confirmation prompt
    print("Press Enter to continue (or Ctrl+C to abort)...")
    try:
        input()
    except KeyboardInterrupt:
        print("\n[PERCEPTION] Aborted by user.")
        return 0

    # Create timestamped output directory
    timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    output_base = Path(args.output_dir)
    output_dir = output_base / timestamp
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"[PERCEPTION] Output directory: {output_dir}")
    print()

    # Track results
    sensors_tested: List[str] = []
    results: Dict[str, bool] = {}
    rviz_launcher: Optional[RVizLauncher] = None

    # Setup signal handler for clean shutdown
    def signal_handler(sig, frame):
        print("\n[PERCEPTION] Interrupted by user")
        if rviz_launcher:
            rviz_launcher.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Launch RViz if requested (before sensor tests so user can see live data)
    if args.rviz:
        rviz_config = Path("src/g1_bringup/config/rviz/sensor_test.rviz")
        rviz_launcher = RVizLauncher(rviz_config if rviz_config.exists() else None)
        if not rviz_launcher.launch():
            print("[PERCEPTION] WARNING: RViz launch failed, continuing without visualization")
        print()

    # Run sensor tests
    try:
        # Camera test
        if args.camera:
            print("-" * 40)
            print("[PERCEPTION] CAMERA TEST")
            print("-" * 40)
            tester = CameraTester(output_dir)
            results["camera"] = tester.test()
            sensors_tested.append("camera")
            print()

        # LiDAR test
        if args.lidar:
            print("-" * 40)
            print("[PERCEPTION] LIDAR TEST")
            print("-" * 40)
            tester = LiDARTester(output_dir, args.lidar_duration)
            results["lidar"] = tester.test()
            sensors_tested.append("lidar")
            print()

        # Audio test
        if args.audio:
            print("-" * 40)
            print("[PERCEPTION] AUDIO TEST")
            print("-" * 40)
            tester = AudioTester(output_dir, args.duration, args.audio_device)
            results["audio"] = tester.test()
            sensors_tested.append("audio")
            print()

        # Create metadata file
        create_metadata(output_dir, sensors_tested, args)

    except Exception as e:
        print(f"[PERCEPTION] ERROR: Unexpected error: {e}")
        if rviz_launcher:
            rviz_launcher.stop()
        return 1

    # Summary
    print("=" * 60)
    print("[PERCEPTION] TEST SUMMARY")
    print("=" * 60)

    all_passed = True
    for sensor, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {sensor.upper():10s}: {status}")
        if not passed:
            all_passed = False

    print()
    print(f"[PERCEPTION] Output saved to: {output_dir}")

    # If RViz is running, wait for user to close it
    if rviz_launcher and rviz_launcher._process:
        print()
        print("[PERCEPTION] RViz is running. Press Ctrl+C to stop...")
        try:
            rviz_launcher.wait()
        except KeyboardInterrupt:
            pass
        finally:
            rviz_launcher.stop()

    if all_passed:
        print()
        print("[PERCEPTION] SUCCESS: All sensor tests passed!")
        return 0
    else:
        print()
        print("[PERCEPTION] WARNING: Some sensor tests failed. Check hardware connections.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
