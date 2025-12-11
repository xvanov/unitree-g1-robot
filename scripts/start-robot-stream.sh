#!/bin/bash
# Start video stream on Unitree G1 robot via SSH
# Usage: start-robot-stream.sh <robot_ip> <local_ip> [video_device] [port]

ROBOT_IP="${1:-192.168.123.233}"
LOCAL_IP="${2:-192.168.123.235}"
VIDEO_DEV="${3:-/dev/video2}"
PORT="${4:-5000}"

echo "[STREAM] Starting video stream from robot $ROBOT_IP to $LOCAL_IP:$PORT"
echo "[STREAM] Using video device: $VIDEO_DEV"

# SSH command to start GStreamer on the robot
# Using -o options for non-interactive connection
SSH_CMD="gst-launch-1.0 v4l2src device=$VIDEO_DEV ! \
  \"video/x-raw,format=UYVY,width=640,height=480,framerate=30/1\" ! \
  videoconvert ! x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast ! \
  rtph264pay ! udpsink host=$LOCAL_IP port=$PORT"

echo "[STREAM] Connecting to robot via SSH..."
echo "[STREAM] Command: $SSH_CMD"

# Run SSH with options to prevent hanging on first connect
# -o StrictHostKeyChecking=no - don't prompt for host key
# -o UserKnownHostsFile=/dev/null - don't save host key
# -o ConnectTimeout=5 - timeout after 5 seconds
# -o BatchMode=yes - don't prompt for password (requires key auth)
ssh -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    -o ConnectTimeout=10 \
    -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=3 \
    unitree@$ROBOT_IP "$SSH_CMD"

EXIT_CODE=$?
if [ $EXIT_CODE -ne 0 ]; then
    echo "[STREAM] SSH connection failed (exit code: $EXIT_CODE)"
    echo "[STREAM] Make sure:"
    echo "  1. Robot is powered on and connected to network"
    echo "  2. SSH key is set up: ssh-copy-id unitree@$ROBOT_IP"
    echo "  3. Robot IP is correct: $ROBOT_IP"
fi

exit $EXIT_CODE
