#!/usr/bin/env python3
# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0

"""
Unit tests for sensor_hello_world.py CLI framework.

Tests CLI argument parsing and basic functionality without requiring
actual sensor hardware.

Usage:
    pytest scripts/test_sensor_hello_world.py -v
"""

import argparse
import sys
import tempfile
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Add scripts directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from sensor_hello_world import (
    DEFAULT_AUDIO_DURATION,
    DEFAULT_LIDAR_DURATION,
    DEFAULT_OUTPUT_DIR,
    AudioTester,
    CameraTester,
    LiDARTester,
    RVizLauncher,
    SensorTester,
    create_metadata,
    parse_args,
    print_banner,
)


class TestCLIParsing:
    """Tests for command line argument parsing."""

    def test_camera_flag(self):
        """Test --camera flag sets camera=True."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--camera']):
            args = parse_args()
            assert args.camera is True
            assert args.lidar is False
            assert args.audio is False
            assert args.network_interface == 'eth0'

    def test_lidar_flag(self):
        """Test --lidar flag sets lidar=True."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--lidar']):
            args = parse_args()
            assert args.lidar is True
            assert args.camera is False
            assert args.audio is False

    def test_audio_flag(self):
        """Test --audio flag sets audio=True."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--audio']):
            args = parse_args()
            assert args.audio is True
            assert args.camera is False
            assert args.lidar is False

    def test_all_flag(self):
        """Test --all flag enables all sensors."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--all']):
            args = parse_args()
            assert args.camera is True
            assert args.lidar is True
            assert args.audio is True
            assert args.all is True

    def test_rviz_flag(self):
        """Test --rviz flag."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--lidar', '--rviz']):
            args = parse_args()
            assert args.rviz is True
            assert args.lidar is True

    def test_output_dir_argument(self):
        """Test --output-dir argument."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--camera', '--output-dir', './my_captures']):
            args = parse_args()
            assert args.output_dir == './my_captures'

    def test_default_output_dir(self):
        """Test default output directory."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--camera']):
            args = parse_args()
            assert args.output_dir == DEFAULT_OUTPUT_DIR

    def test_duration_argument(self):
        """Test --duration argument for audio."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--audio', '--duration', '10']):
            args = parse_args()
            assert args.duration == 10.0

    def test_default_duration(self):
        """Test default audio duration."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--audio']):
            args = parse_args()
            assert args.duration == DEFAULT_AUDIO_DURATION

    def test_lidar_duration_argument(self):
        """Test --lidar-duration argument."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--lidar', '--lidar-duration', '5']):
            args = parse_args()
            assert args.lidar_duration == 5.0

    def test_default_lidar_duration(self):
        """Test default lidar duration."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--lidar']):
            args = parse_args()
            assert args.lidar_duration == DEFAULT_LIDAR_DURATION

    def test_network_interface_required(self):
        """Test network interface is required."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', '--camera']):
            with pytest.raises(SystemExit):
                parse_args()

    def test_no_sensor_selected_error(self):
        """Test error when no sensor is selected."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0']):
            with pytest.raises(SystemExit):
                parse_args()

    def test_multiple_sensors(self):
        """Test selecting multiple sensors."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--camera', '--lidar']):
            args = parse_args()
            assert args.camera is True
            assert args.lidar is True
            assert args.audio is False

    def test_all_with_rviz(self):
        """Test --all --rviz combination."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--all', '--rviz']):
            args = parse_args()
            assert args.camera is True
            assert args.lidar is True
            assert args.audio is True
            assert args.rviz is True

    def test_audio_device_argument(self):
        """Test --audio-device argument."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--audio', '--audio-device', '3']):
            args = parse_args()
            assert args.audio_device == 3

    def test_default_audio_device(self):
        """Test default audio device is None (auto-detect)."""
        with patch.object(sys, 'argv', ['sensor_hello_world.py', 'eth0', '--audio']):
            args = parse_args()
            assert args.audio_device is None


class TestSensorTesters:
    """Tests for sensor tester classes."""

    def test_camera_tester_name(self):
        """Test CameraTester returns correct name."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = CameraTester(Path(tmpdir))
            assert "Camera" in tester.get_name()
            assert "RealSense" in tester.get_name()

    def test_lidar_tester_name(self):
        """Test LiDARTester returns correct name."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = LiDARTester(Path(tmpdir))
            assert "LiDAR" in tester.get_name()
            assert "Livox" in tester.get_name()

    def test_audio_tester_name(self):
        """Test AudioTester returns correct name."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = AudioTester(Path(tmpdir))
            assert "Audio" in tester.get_name()
            assert "Microphone" in tester.get_name()

    def test_sensor_tester_creates_output_dir(self):
        """Test sensor tester creates output directory."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = Path(tmpdir) / "test_output"
            assert not output_path.exists()
            tester = CameraTester(output_path)
            assert output_path.exists()

    def test_lidar_tester_custom_duration(self):
        """Test LiDARTester accepts custom duration."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = LiDARTester(Path(tmpdir), duration=10.0)
            assert tester.duration == 10.0

    def test_audio_tester_custom_duration(self):
        """Test AudioTester accepts custom duration."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = AudioTester(Path(tmpdir), duration=15.0)
            assert tester.duration == 15.0

    def test_audio_tester_custom_device(self):
        """Test AudioTester accepts custom device index."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = AudioTester(Path(tmpdir), device_index=2)
            assert tester.device_index == 2


class TestMetadata:
    """Tests for metadata creation."""

    def test_create_metadata(self):
        """Test metadata file is created correctly."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            args = argparse.Namespace(
                duration=5.0,
                lidar_duration=2.0,
                rviz=False,
                network_interface='eth0',
                audio_device=None
            )
            sensors_tested = ['camera', 'lidar']

            create_metadata(output_dir, sensors_tested, args)

            metadata_path = output_dir / "capture_metadata.json"
            assert metadata_path.exists()

            import json
            with open(metadata_path) as f:
                metadata = json.load(f)

            assert 'timestamp' in metadata
            assert metadata['network_interface'] == 'eth0'
            assert metadata['sensors_tested'] == ['camera', 'lidar']
            assert metadata['parameters']['audio_duration'] == 5.0
            assert metadata['parameters']['lidar_duration'] == 2.0
            assert metadata['parameters']['rviz_enabled'] is False
            assert metadata['parameters']['audio_device'] is None


class TestRVizLauncher:
    """Tests for RViz launcher."""

    def test_rviz_launcher_init(self):
        """Test RVizLauncher initialization."""
        launcher = RVizLauncher()
        assert launcher.rviz_config_path is None
        assert launcher._process is None

    def test_rviz_launcher_with_config(self):
        """Test RVizLauncher with config path."""
        config_path = Path("/tmp/test.rviz")
        launcher = RVizLauncher(config_path)
        assert launcher.rviz_config_path == config_path

    def test_rviz_launcher_stop_when_not_running(self):
        """Test stop() doesn't error when not running."""
        launcher = RVizLauncher()
        launcher.stop()  # Should not raise

    def test_rviz_launcher_is_running_when_not_started(self):
        """Test is_running() returns False when not started."""
        launcher = RVizLauncher()
        assert launcher.is_running() is False


class TestPrintBanner:
    """Tests for banner printing."""

    def test_print_banner_runs(self, capsys):
        """Test print_banner executes without error."""
        print_banner()
        captured = capsys.readouterr()
        assert "SENSOR HELLO WORLD" in captured.out
        assert "PERCEPTION VALIDATION" in captured.out
        assert "SAFETY NOTE" in captured.out


class TestSensorTesterAbstract:
    """Tests for SensorTester base class."""

    def test_sensor_tester_is_abstract(self):
        """Test SensorTester cannot be instantiated directly."""
        with tempfile.TemporaryDirectory() as tmpdir:
            with pytest.raises(TypeError):
                SensorTester(Path(tmpdir))


class TestCameraImportError:
    """Tests for camera import error handling."""

    def test_camera_handles_import_error(self, capsys):
        """Test camera tester handles missing pyrealsense2."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = CameraTester(Path(tmpdir))

            # Mock import to fail
            with patch.dict(sys.modules, {'pyrealsense2': None}):
                # Force reimport by clearing the import
                import importlib
                with patch('builtins.__import__', side_effect=ImportError("No module named 'pyrealsense2'")):
                    result = tester.test()
                    captured = capsys.readouterr()
                    assert result is False
                    assert "pyrealsense2" in captured.out


class TestLiDARImportError:
    """Tests for LiDAR import error handling."""

    def test_lidar_handles_import_error(self, capsys):
        """Test LiDAR tester handles missing ROS2."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = LiDARTester(Path(tmpdir))

            with patch('builtins.__import__', side_effect=ImportError("No module named 'rclpy'")):
                result = tester.test()
                captured = capsys.readouterr()
                assert result is False
                assert "ROS2" in captured.out


class TestAudioImportError:
    """Tests for audio import error handling."""

    def test_audio_handles_import_error(self, capsys):
        """Test audio tester handles missing sounddevice."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tester = AudioTester(Path(tmpdir))

            with patch('builtins.__import__', side_effect=ImportError("No module named 'sounddevice'")):
                result = tester.test()
                captured = capsys.readouterr()
                assert result is False
                assert "sounddevice" in captured.out


class TestPointCloud2Parsing:
    """Tests for PointCloud2 to XYZ conversion."""

    def test_pointcloud2_to_xyz_valid_data(self):
        """Test parsing valid PointCloud2 message."""
        import struct
        from unittest.mock import MagicMock

        with tempfile.TemporaryDirectory() as tmpdir:
            tester = LiDARTester(Path(tmpdir))

            # Create mock PointCloud2 message
            msg = MagicMock()
            msg.point_step = 12  # 3 floats x 4 bytes
            msg.row_step = 12

            # Create mock fields
            x_field = MagicMock()
            x_field.name = 'x'
            x_field.offset = 0

            y_field = MagicMock()
            y_field.name = 'y'
            y_field.offset = 4

            z_field = MagicMock()
            z_field.name = 'z'
            z_field.offset = 8

            msg.fields = [x_field, y_field, z_field]

            # Create test data: 2 points at (1,2,3) and (4,5,6)
            data = struct.pack('fff', 1.0, 2.0, 3.0) + struct.pack('fff', 4.0, 5.0, 6.0)
            msg.data = data

            result = tester._pointcloud2_to_xyz(msg)

            assert result is not None
            assert len(result) == 2
            assert abs(result[0][0] - 1.0) < 0.001
            assert abs(result[0][1] - 2.0) < 0.001
            assert abs(result[0][2] - 3.0) < 0.001
            assert abs(result[1][0] - 4.0) < 0.001

    def test_pointcloud2_to_xyz_missing_fields(self):
        """Test parsing message with missing XYZ fields."""
        from unittest.mock import MagicMock

        with tempfile.TemporaryDirectory() as tmpdir:
            tester = LiDARTester(Path(tmpdir))

            msg = MagicMock()
            msg.point_step = 4
            msg.fields = []  # No fields
            msg.data = b'\x00' * 12

            result = tester._pointcloud2_to_xyz(msg)
            assert result is None

    def test_pointcloud2_to_xyz_filters_nan(self):
        """Test that NaN points are filtered out."""
        import struct
        import math
        from unittest.mock import MagicMock

        with tempfile.TemporaryDirectory() as tmpdir:
            tester = LiDARTester(Path(tmpdir))

            msg = MagicMock()
            msg.point_step = 12
            msg.row_step = 12

            x_field = MagicMock()
            x_field.name = 'x'
            x_field.offset = 0
            y_field = MagicMock()
            y_field.name = 'y'
            y_field.offset = 4
            z_field = MagicMock()
            z_field.name = 'z'
            z_field.offset = 8
            msg.fields = [x_field, y_field, z_field]

            # One valid point, one NaN point
            nan = float('nan')
            data = struct.pack('fff', 1.0, 2.0, 3.0) + struct.pack('fff', nan, 5.0, 6.0)
            msg.data = data

            result = tester._pointcloud2_to_xyz(msg)

            assert result is not None
            assert len(result) == 1  # NaN point filtered


class TestPCDSaving:
    """Tests for PCD file saving."""

    def test_save_pcd_creates_valid_file(self):
        """Test that _save_pcd creates a valid ASCII PCD file."""
        import numpy as np

        with tempfile.TemporaryDirectory() as tmpdir:
            tester = LiDARTester(Path(tmpdir))
            points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
            pcd_path = Path(tmpdir) / "test.pcd"

            tester._save_pcd(points, pcd_path)

            assert pcd_path.exists()

            # Read and verify PCD format
            with open(pcd_path, 'r') as f:
                content = f.read()
                assert 'VERSION' in content
                assert 'FIELDS x y z' in content
                assert 'POINTS 2' in content
                assert '1.000000 2.000000 3.000000' in content


class TestRVizConfigValidity:
    """Tests for RViz config file."""

    def test_rviz_config_is_valid_yaml(self):
        """Test that RViz config file is valid YAML."""
        import yaml

        rviz_path = Path(__file__).parent.parent / "src/g1_bringup/config/rviz/sensor_test.rviz"
        if not rviz_path.exists():
            pytest.skip(f"RViz config not found at {rviz_path}")

        with open(rviz_path, 'r') as f:
            config = yaml.safe_load(f)

        assert 'Visualization Manager' in config
        assert config['Visualization Manager']['Global Options']['Fixed Frame'] == 'utlidar_lidar'

    def test_rviz_config_has_pointcloud_display(self):
        """Test that RViz config includes PointCloud2 display for utlidar/cloud."""
        import yaml

        rviz_path = Path(__file__).parent.parent / "src/g1_bringup/config/rviz/sensor_test.rviz"
        if not rviz_path.exists():
            pytest.skip(f"RViz config not found at {rviz_path}")

        with open(rviz_path, 'r') as f:
            config = yaml.safe_load(f)

        displays = config['Visualization Manager']['Displays']
        pointcloud_displays = [d for d in displays if d.get('Class') == 'rviz_default_plugins/PointCloud2']
        assert len(pointcloud_displays) >= 1
        assert pointcloud_displays[0]['Topic']['Value'] == 'utlidar/cloud'


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
