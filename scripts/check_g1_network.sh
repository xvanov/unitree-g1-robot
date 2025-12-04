#!/bin/bash
# =============================================================================
# G1 Robot Network Configuration Check Script
# =============================================================================
# Verifies network connectivity to the G1 robot.
#
# Usage:
#   ./scripts/check_g1_network.sh
#
# This script auto-detects interfaces on the robot subnet.
# =============================================================================

# Network configuration (fixed by Unitree hardware)
ROBOT_IP="192.168.123.164"
LIDAR_IP="192.168.123.120"
SUBNET="192.168.123"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "============================================================"
echo "     G1 ROBOT NETWORK CONFIGURATION CHECK"
echo "============================================================"
echo
echo "Expected Network Configuration:"
echo "  Robot (Ethernet):   ${ROBOT_IP}"
echo "  LiDAR (MID-360):    ${LIDAR_IP}"
echo "  Subnet:             ${SUBNET}.x"
echo
echo "============================================================"

# Check for interfaces on the robot subnet
echo -e "${BLUE}[CHECK]${NC} Looking for interfaces on ${SUBNET}.x subnet..."
MATCHING_IFACES=$(ip addr | grep "${SUBNET}" | awk '{print $NF}')

if [ -z "$MATCHING_IFACES" ]; then
    echo -e "${RED}[FAIL]${NC} No interface found on ${SUBNET}.x subnet!"
    echo
    echo "To fix this:"
    echo "  1. Connect development computer to robot via Ethernet"
    echo "  2. Configure static IP on the Ethernet interface:"
    echo "     sudo ip addr add 192.168.123.10/24 dev <interface>"
    echo
    echo "Available network interfaces:"
    ip link show | grep -E "^[0-9]+" | awk '{print "  " $2}' | tr -d ':'
    exit 1
else
    echo -e "${GREEN}[OK]${NC} Found interface(s) on robot subnet:"
    echo "${MATCHING_IFACES}" | while read iface; do
        IP=$(ip addr show $iface 2>/dev/null | grep "${SUBNET}" | awk '{print $2}')
        echo "  Interface: $iface - IP: $IP"
    done
fi

echo

# Ping robot
echo -e "${BLUE}[CHECK]${NC} Testing connectivity to robot (${ROBOT_IP})..."
if ping -c 3 -W 2 ${ROBOT_IP} > /dev/null 2>&1; then
    echo -e "${GREEN}[OK]${NC} Robot is reachable at ${ROBOT_IP}"
else
    echo -e "${YELLOW}[WARN]${NC} Robot not responding at ${ROBOT_IP}"
    echo "  Make sure robot is powered on and connected."
fi

echo

# Ping LiDAR
echo -e "${BLUE}[CHECK]${NC} Testing connectivity to LiDAR (${LIDAR_IP})..."
if ping -c 3 -W 2 ${LIDAR_IP} > /dev/null 2>&1; then
    echo -e "${GREEN}[OK]${NC} LiDAR is reachable at ${LIDAR_IP}"
else
    echo -e "${YELLOW}[WARN]${NC} LiDAR not responding at ${LIDAR_IP}"
    echo "  LiDAR may not be enabled or connected."
fi

echo
echo "============================================================"
echo "NETWORK INTERFACE FOR SCRIPTS"
echo "============================================================"
echo
echo "When running hello_world_g1.py or hardware_bridge node,"
echo "use one of these interfaces connected to ${SUBNET}.x:"
echo

# Show matching interfaces
ip addr | grep -B2 "${SUBNET}" | grep -E "^[0-9]+:" | awk '{print $2}' | tr -d ':' | while read iface; do
    echo "  python3 scripts/hello_world_g1.py ${iface}"
done

echo
echo "============================================================"
