#!/bin/bash
# NICE DCV Installation Script for Ubuntu 22.04 (x86_64)
# This script installs NICE DCV server with Ubuntu GNOME desktop
# Idempotent: safe to run multiple times

set -e

# Get the directory where this script lives (works from any working directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Installing NICE DCV Server with Ubuntu GNOME Desktop ==="

# Check if DCV is already installed and at the correct version
if dpkg -s nice-dcv-server 2>/dev/null | grep -q "Version: 2025.0.20103"; then
    echo "NICE DCV Server 2025.0.20103 is already installed, skipping package installation..."
    DCV_ALREADY_INSTALLED=true
else
    DCV_ALREADY_INSTALLED=false
fi

# Install dependencies (only if not already present)
echo "Installing Ubuntu GNOME desktop..."
sudo apt update
sudo apt install -y ubuntu-desktop gdm3

# Remove XFCE if present (to avoid ugly fallback UI)
echo "Removing XFCE if present..."
sudo apt remove -y xfce4 xfce4-session 2>/dev/null || true
sudo apt autoremove -y 2>/dev/null || true

# Only download and install DCV packages if not already installed
if [ "$DCV_ALREADY_INSTALLED" = false ]; then
    # Download NICE GPG key
    echo "Importing NICE GPG key..."
    wget -q -O /tmp/NICE-GPG-KEY https://d1uj6qtbmh3dt5.cloudfront.net/NICE-GPG-KEY
    gpg --import /tmp/NICE-GPG-KEY

    # Download NICE DCV packages
    echo "Downloading NICE DCV 2025.0..."
    wget -q --show-progress -O /tmp/nice-dcv-2025.0-20103-ubuntu2204-x86_64.tgz \
        https://d1uj6qtbmh3dt5.cloudfront.net/2025.0/Servers/nice-dcv-2025.0-20103-ubuntu2204-x86_64.tgz

    # Extract packages
    echo "Extracting packages..."
    tar -xzf /tmp/nice-dcv-2025.0-20103-ubuntu2204-x86_64.tgz -C /tmp

    # Install DCV server and dependencies
    echo "Installing DCV server..."
    sudo apt install -y /tmp/nice-dcv-2025.0-20103-ubuntu2204-x86_64/nice-dcv-server_2025.0.20103-1_amd64.ubuntu2204.deb
    sudo apt install -y /tmp/nice-dcv-2025.0-20103-ubuntu2204-x86_64/nice-xdcv_2025.0.688-1_amd64.ubuntu2204.deb
    sudo apt install -y /tmp/nice-dcv-2025.0-20103-ubuntu2204-x86_64/nice-dcv-simple-external-authenticator_2025.0.282-1_amd64.ubuntu2204.deb

    # Cleanup downloaded files
    echo "Cleaning up downloaded files..."
    rm -f /tmp/NICE-GPG-KEY
    rm -f /tmp/nice-dcv-2025.0-20103-ubuntu2204-x86_64.tgz
    rm -rf /tmp/nice-dcv-2025.0-20103-ubuntu2204-x86_64
fi

# Add dcv user to video group
sudo usermod -aG video dcv

# Configure GNOME as the default desktop for DCV virtual sessions
echo "Configuring Ubuntu GNOME desktop for DCV..."

# Create user xsessionrc for GNOME
cat > ~/.xsessionrc << 'EOF'
export XDG_SESSION_TYPE=x11
export GDK_BACKEND=x11
export GNOME_SHELL_SESSION_MODE=ubuntu
export XDG_CURRENT_DESKTOP=ubuntu:GNOME
EOF

# Create DCV virtual session init script for GNOME
# Note: Software rendering is needed because Xdcv doesn't provide hardware GL
sudo tee /etc/dcv/dcv-virtual-session.sh > /dev/null << 'EOF'
#!/bin/bash
export XDG_SESSION_TYPE=x11
export GDK_BACKEND=x11
export LIBGL_ALWAYS_SOFTWARE=1
export GNOME_SHELL_SESSION_MODE=ubuntu
export XDG_CURRENT_DESKTOP=ubuntu:GNOME
# Force Mutter to use software rendering fallback
export MUTTER_DEBUG_FORCE_FALLBACK=1
gnome-session --session=ubuntu
EOF
sudo chmod +x /etc/dcv/dcv-virtual-session.sh

# Enable and start DCV server
echo "Starting DCV server..."
sudo systemctl enable dcvserver
sudo systemctl start dcvserver

# Install systemd service to auto-create DCV session on boot
echo "Installing DCV session auto-start service..."
sudo cp "$SCRIPT_DIR/dcv-virtual-session.service" /etc/systemd/system/
# Replace 'ubuntu' with actual user in the service file
sudo sed -i "s/--owner ubuntu/--owner $USER/" /etc/systemd/system/dcv-virtual-session.service
sudo systemctl daemon-reload
sudo systemctl enable dcv-virtual-session.service
sudo systemctl restart dcv-virtual-session.service

# Show status
echo ""
echo "=== Installation Complete ==="
echo "DCV Server Status:"
sudo systemctl status dcvserver --no-pager
echo ""
echo "Active Sessions:"
dcv list-sessions
echo ""
echo "Connect via: https://<your-server-ip>:8443"
echo "Make sure port 8443 is open in your security group/firewall"
echo ""
echo "Desktop: Ubuntu GNOME"
