#!/bin/bash
# =============================================================================
# G1 Robot Teleop Control Script
# =============================================================================
# Launches keyboard teleop for the real G1 robot.
#
# SAFETY REQUIREMENTS:
#   1. Clear area: No obstacles within 2 meters of robot
#   2. E-stop ready: Know location of hardware E-stop button
#   3. Supervisor present: Never run locomotion tests alone
#
# Usage:
#   ./scripts/teleop_g1.sh <network_interface>
#
# Example:
#   ./scripts/teleop_g1.sh eth0
#
# Controls:
#   i - Move forward
#   , - Move backward
#   j - Turn left
#   l - Turn right
#   k - Stop
#   q/z - Increase/decrease max linear speed
#   w/x - Increase/decrease max angular speed
#
# IMPORTANT: Velocity limits are ENFORCED by hardware_bridge node:
#   Max linear:  0.3 m/s (hardware_bridge clamps all commands)
#   Max angular: 0.5 rad/s
#   Even if you increase speed in teleop_twist_keyboard, the hardware_bridge
#   will clamp velocities to safe limits before sending to the robot.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Track bridge PID for cleanup
BRIDGE_PID=""

# Cleanup function for graceful shutdown
cleanup() {
    echo
    echo -e "${BLUE}[NAVIGATION]${NC} Initiating graceful shutdown..."

    if [[ -n "$BRIDGE_PID" ]] && kill -0 $BRIDGE_PID 2>/dev/null; then
        echo -e "${BLUE}[NAVIGATION]${NC} Sending SIGTERM to hardware_bridge (PID: ${BRIDGE_PID})..."
        # Send SIGTERM first for graceful shutdown (allows Damp() call)
        kill -TERM $BRIDGE_PID 2>/dev/null || true

        # Wait up to 5 seconds for graceful shutdown
        local count=0
        while kill -0 $BRIDGE_PID 2>/dev/null && [[ $count -lt 50 ]]; do
            sleep 0.1
            ((count++))
        done

        # Force kill if still running
        if kill -0 $BRIDGE_PID 2>/dev/null; then
            echo -e "${YELLOW}[NAVIGATION]${NC} Force killing hardware_bridge..."
            kill -KILL $BRIDGE_PID 2>/dev/null || true
        fi

        wait $BRIDGE_PID 2>/dev/null || true
    fi

    echo -e "${GREEN}[NAVIGATION]${NC} Teleop session ended. Robot should be in damp mode."
}

# Register cleanup on EXIT, INT, TERM
trap cleanup EXIT INT TERM

# Check arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <network_interface>"
    echo ""
    echo "Example:"
    echo "  $0 eth0"
    echo ""
    echo "Find your interface with: ip addr | grep 192.168.123"
    exit 1
fi

NETWORK_INTERFACE=$1

echo "============================================================"
echo "     G1 ROBOT TELEOP CONTROL"
echo "============================================================"
echo
echo -e "${RED}SAFETY REQUIREMENTS:${NC}"
echo "  1. Clear area: No obstacles within 2 meters of robot"
echo "  2. E-stop ready: Know location of hardware E-stop button"
echo "  3. Supervisor present: Never run locomotion tests alone"
echo
echo "VELOCITY LIMITS (ENFORCED BY HARDWARE_BRIDGE):"
echo "  Max linear:  0.3 m/s (clamped regardless of teleop settings)"
echo "  Max angular: 0.5 rad/s"
echo
echo "CONTROLS:"
echo "  i - Move forward       , - Move backward"
echo "  j - Turn left          l - Turn right"
echo "  k - Stop (IMPORTANT!)"
echo "  q/z - Increase/decrease linear speed (clamped by bridge)"
echo "  w/x - Increase/decrease angular speed (clamped by bridge)"
echo
echo "============================================================"
echo

# Source environment
echo -e "${BLUE}[NAVIGATION]${NC} Sourcing ROS2 environment..."
source "$PROJECT_ROOT/scripts/env.sh"
source "$PROJECT_ROOT/install/setup.bash"

echo -e "${YELLOW}[NAVIGATION]${NC} Starting hardware_bridge with interface: ${NETWORK_INTERFACE}"
echo -e "${YELLOW}[NAVIGATION]${NC} Robot will stand up automatically..."
echo

# Start hardware bridge in background
ros2 run g1_navigation hardware_bridge --ros-args -p network_interface:=${NETWORK_INTERFACE} &
BRIDGE_PID=$!

# Wait for bridge to initialize
sleep 3

# Check if bridge is running
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo -e "${RED}[NAVIGATION]${NC} Hardware bridge failed to start!"
    BRIDGE_PID=""  # Clear so cleanup doesn't try to kill it
    exit 1
fi

echo -e "${GREEN}[NAVIGATION]${NC} Hardware bridge running (PID: ${BRIDGE_PID})"
echo -e "${GREEN}[NAVIGATION]${NC} Starting teleop keyboard..."
echo
echo -e "${YELLOW}Press Ctrl+C to stop. Robot will enter safe damp mode.${NC}"
echo

# Start teleop keyboard with conservative initial speeds
# Note: hardware_bridge will clamp to max 0.3 m/s linear, 0.5 rad/s angular
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/g1/cmd_vel \
    -p speed:=0.1 \
    -p turn:=0.3

# Cleanup is handled by trap
