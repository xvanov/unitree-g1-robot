#!/bin/bash
# check_g1_network.sh - Verify network connectivity to G1 robot

set -e

ROBOT_IP="192.168.123.164"
ROBOT_SUBNET="192.168.123.0/24"
LIDAR_IP="192.168.123.120"
LOCO_IP="192.168.123.161"

echo "=== G1 Network Connectivity Check ==="
echo ""

# Check if we have an interface on the robot subnet
echo "Checking local network interfaces..."
if command -v ip &> /dev/null; then
    # Linux
    LOCAL_IF=$(ip addr | grep "192.168.123" | head -1)
elif command -v ifconfig &> /dev/null; then
    # macOS
    LOCAL_IF=$(ifconfig | grep "192.168.123" | head -1)
else
    LOCAL_IF=""
fi

if [ -z "$LOCAL_IF" ]; then
    echo "[WARNING] No interface found on robot subnet 192.168.123.x"
    echo ""
    echo "To configure your network:"
    echo ""
    echo "  LINUX:"
    echo "    sudo ip addr add 192.168.123.100/24 dev eth0"
    echo ""
    echo "  MAC:"
    echo "    1. Open System Preferences > Network"
    echo "    2. Select your Ethernet adapter"
    echo "    3. Configure IPv4: Manually"
    echo "    4. IP Address: 192.168.123.100"
    echo "    5. Subnet Mask: 255.255.255.0"
    echo "    6. Click Apply"
    echo ""
else
    echo "[OK] Found local interface on robot subnet:"
    echo "     $LOCAL_IF"
fi

echo ""

# Ping robot
echo "Pinging robot at $ROBOT_IP..."
if ping -c 2 -W 2 $ROBOT_IP &> /dev/null; then
    echo "[OK] Robot is reachable at $ROBOT_IP"
else
    echo "[FAIL] Robot not reachable at $ROBOT_IP"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Is the robot powered on?"
    echo "  2. Is your Ethernet cable connected?"
    echo "  3. Is your IP on the correct subnet? (192.168.123.x)"
    exit 1
fi

# Check SSH availability
echo ""
echo "Checking SSH access to robot..."
if timeout 3 bash -c "echo > /dev/tcp/$ROBOT_IP/22" 2>/dev/null; then
    echo "[OK] SSH port 22 is open"
    echo "     Connect with: ssh unitree@$ROBOT_IP (password: 123)"
else
    echo "[INFO] SSH port 22 not responding (may be normal if not Orin)"
fi

# Ping LiDAR (optional)
echo ""
echo "Pinging LiDAR at $LIDAR_IP..."
if ping -c 2 -W 2 $LIDAR_IP &> /dev/null; then
    echo "[OK] LiDAR is reachable at $LIDAR_IP"
else
    echo "[INFO] LiDAR not reachable at $LIDAR_IP (may be normal)"
fi

# Ping locomotion computer (optional)
echo ""
echo "Pinging locomotion computer at $LOCO_IP..."
if ping -c 2 -W 2 $LOCO_IP &> /dev/null; then
    echo "[OK] Locomotion computer is reachable at $LOCO_IP"
else
    echo "[INFO] Locomotion computer not reachable at $LOCO_IP (may be normal)"
fi

echo ""
echo "=== Network Check Complete ==="
echo ""
echo "If all checks pass, you can run:"
echo "  ./g1_inspector --test-sensors"
echo "  ./g1_inspector --test-loco"
echo "  ./g1_inspector --hello-world"
