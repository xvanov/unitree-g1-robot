#!/bin/bash
# Setup persistent WiFi on Unitree G1 robot
# Run this script ONCE on the robot to configure WiFi auto-connect
#
# This script:
# 1. Disables systemd-rfkill (stops it from restoring blocked state)
# 2. Creates a boot service to unblock WiFi and connect via wpa_supplicant
# 3. Bypasses NetworkManager which doesn't work well with the RTW driver

WIFI_SSID="${1:-BarryFi}"
WIFI_PASS="${2:-barryg11}"

echo "=== Unitree G1 Robot WiFi Setup ==="
echo "Network: $WIFI_SSID"
echo ""

# Must run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run with sudo: sudo $0 [SSID] [PASSWORD]"
    exit 1
fi

# 1. Disable systemd-rfkill permanently (it keeps restoring blocked state)
echo "[1/6] Disabling systemd-rfkill..."
systemctl stop systemd-rfkill.socket 2>/dev/null
systemctl disable systemd-rfkill.socket 2>/dev/null
systemctl mask systemd-rfkill.socket

# Clear any saved rfkill state
rm -f /var/lib/systemd/rfkill/*

# 2. Unblock WiFi now
echo "[2/6] Unblocking WiFi..."
rfkill unblock wifi
rfkill unblock all

# 3. Create wpa_supplicant config
echo "[3/6] Creating wpa_supplicant configuration..."
mkdir -p /etc/wpa_supplicant

cat > /etc/wpa_supplicant/wpa_supplicant-wlan0.conf << EOF
ctrl_interface=/var/run/wpa_supplicant
update_config=1
country=US

network={
    ssid="$WIFI_SSID"
    psk="$WIFI_PASS"
    key_mgmt=WPA-PSK
    priority=100
}
EOF

chmod 600 /etc/wpa_supplicant/wpa_supplicant-wlan0.conf

# 4. Create the boot service that handles everything
echo "[4/6] Creating WiFi boot service..."

cat > /etc/systemd/system/unitree-wifi.service << 'EOF'
[Unit]
Description=Unitree G1 WiFi Connect
After=network-pre.target systemd-udevd.service
Before=network.target
Wants=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/bin/sleep 2
ExecStart=/usr/local/bin/unitree-wifi-connect.sh
ExecStop=/usr/bin/pkill -f "wpa_supplicant.*wlan0"

[Install]
WantedBy=multi-user.target
EOF

# 5. Create the connection script
echo "[5/6] Creating connection script..."

cat > /usr/local/bin/unitree-wifi-connect.sh << 'SCRIPT'
#!/bin/bash
# Unitree G1 WiFi Connection Script
# Called by unitree-wifi.service at boot

LOG_TAG="unitree-wifi"

log() {
    echo "$1"
    logger -t "$LOG_TAG" "$1"
}

log "Starting WiFi connection..."

# Kill gsd-rfkill if running (GNOME keeps re-blocking)
pkill -f gsd-rfkill 2>/dev/null

# Unblock WiFi
/usr/sbin/rfkill unblock wifi
/usr/sbin/rfkill unblock all
sleep 1

# Verify unblocked
if /usr/sbin/rfkill list | grep -A1 "Wireless LAN" | grep -q "Soft blocked: yes"; then
    log "ERROR: WiFi still blocked"
    exit 1
fi

# Bring up interface
/sbin/ip link set wlan0 up
sleep 1

# Kill any existing wpa_supplicant for wlan0
pkill -f "wpa_supplicant.*wlan0" 2>/dev/null
sleep 1

# Start wpa_supplicant
/sbin/wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
sleep 3

# Wait for connection (up to 30 seconds)
for i in {1..10}; do
    if /sbin/iw dev wlan0 link | grep -q "Connected"; then
        log "Connected to WiFi"
        break
    fi
    log "Waiting for connection... ($i/10)"
    sleep 3
done

# Get IP via DHCP
/sbin/dhclient -v wlan0 2>&1 | head -5

# Log final status
IP_ADDR=$(/sbin/ip addr show wlan0 | grep "inet " | awk '{print $2}')
if [ -n "$IP_ADDR" ]; then
    log "WiFi connected with IP: $IP_ADDR"
else
    log "WARNING: No IP address obtained"
fi
SCRIPT

chmod +x /usr/local/bin/unitree-wifi-connect.sh

# 6. Enable and start the service
echo "[6/6] Enabling service..."
systemctl daemon-reload
systemctl enable unitree-wifi.service

# Start it now to test
echo ""
echo "=== Testing WiFi Connection ==="
systemctl start unitree-wifi.service
sleep 5

# Show status
echo ""
echo "=== Connection Status ==="
iw dev wlan0 link
echo ""
ip addr show wlan0 | grep -E "inet|state"
echo ""

# Test connectivity
if ping -c 2 -W 3 8.8.8.8 > /dev/null 2>&1; then
    echo "Internet connectivity: OK"
else
    echo "Internet connectivity: No (may be local network only)"
fi

echo ""
echo "=== Setup Complete ==="
echo "WiFi will auto-connect on every boot."
echo ""
echo "To change networks, edit: /etc/wpa_supplicant/wpa_supplicant-wlan0.conf"
echo "Then run: sudo systemctl restart unitree-wifi.service"
