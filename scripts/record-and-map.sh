#!/bin/bash
# Record RGB-D data while teleoperating the robot, then generate a point cloud map
#
# Usage: ./scripts/record-and-map.sh <robot_ip> [session_name]
#
# This script:
# 1. Starts teleop with depth recording
# 2. When you quit (Q), generates a point cloud from the recorded data
# 3. Opens the point cloud viewer (if available)

set -e

ROBOT_IP="${1:-192.168.123.233}"
SESSION="${2:-rgbd_$(date +%Y%m%d_%H%M%S)}"
SESSION_DIR="data/recordings/${SESSION}"

echo "=== RGB-D Recording and Mapping ==="
echo "Robot IP:    $ROBOT_IP"
echo "Session:     $SESSION"
echo "Output dir:  $SESSION_DIR"
echo ""
echo "Controls:"
echo "  WASD    - Move robot"
echo "  Z/E     - Rotate"
echo "  SPACE   - Stop"
echo "  R       - Toggle recording"
echo "  Q       - Quit and generate map"
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Check if build exists
if [ ! -f build/g1_inspector ]; then
    echo "Error: build/g1_inspector not found. Run cmake && make first."
    exit 1
fi

# Run teleop with recording
# Note: Depth recording happens automatically when RealSense is detected
build/g1_inspector --teleop keyboard --robot-ip "$ROBOT_IP" --record "$SESSION"

# Check if depth data was recorded
if [ ! -d "${SESSION_DIR}/depth" ]; then
    echo ""
    echo "Warning: No depth data found in $SESSION_DIR/depth"
    echo "Make sure RealSense SDK is installed and camera is detected."
    exit 1
fi

DEPTH_COUNT=$(ls -1 "${SESSION_DIR}/depth/"*.png 2>/dev/null | wc -l)
echo ""
echo "Recording complete!"
echo "Depth frames: $DEPTH_COUNT"
echo ""

# Generate point cloud
echo "Generating point cloud..."
if [ -f build/generate_pointcloud ]; then
    build/generate_pointcloud "$SESSION_DIR" "${SESSION_DIR}/map.ply" --voxel 0.02
else
    echo "Warning: generate_pointcloud not found. Building..."
    cd build && make generate_pointcloud && cd ..
    build/generate_pointcloud "$SESSION_DIR" "${SESSION_DIR}/map.ply" --voxel 0.02
fi

# Try to open viewer
PLY_FILE="${SESSION_DIR}/map.ply"
if [ -f "$PLY_FILE" ]; then
    echo ""
    echo "Point cloud saved to: $PLY_FILE"
    echo ""

    # Try different viewers
    if command -v pcl_viewer &> /dev/null; then
        echo "Opening with pcl_viewer..."
        pcl_viewer "$PLY_FILE" &
    elif command -v meshlab &> /dev/null; then
        echo "Opening with meshlab..."
        meshlab "$PLY_FILE" &
    elif command -v cloudcompare &> /dev/null; then
        echo "Opening with cloudcompare..."
        cloudcompare "$PLY_FILE" &
    else
        echo "No point cloud viewer found. Install one of:"
        echo "  sudo apt install pcl-tools"
        echo "  sudo apt install meshlab"
        echo "  sudo snap install cloudcompare"
    fi
fi

echo ""
echo "Done!"
