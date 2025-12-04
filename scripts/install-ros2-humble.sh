#!/bin/bash
# =============================================================================
# ROS2 Humble Installation Script
# =============================================================================
# Installs ROS2 Humble on Ubuntu 22.04
#
# Usage:
#   ./scripts/install-ros2-humble.sh
#
# Prerequisites:
#   - Ubuntu 22.04
#   - sudo access
#
# Reference:
#   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
}

# -----------------------------------------------------------------------------
# Check if already installed
# -----------------------------------------------------------------------------
if [ -f "/opt/ros/humble/setup.bash" ]; then
    log_success "ROS2 Humble is already installed at /opt/ros/humble"
    echo ""
    echo "To use ROS2, add to your ~/.bashrc:"
    echo "  source /opt/ros/humble/setup.bash"
    echo ""
    exit 0
fi

# -----------------------------------------------------------------------------
# Check Ubuntu version
# -----------------------------------------------------------------------------
log_step "Checking System"

if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "$VERSION_ID" != "22.04" ]]; then
        log_error "ROS2 Humble requires Ubuntu 22.04, found $VERSION_ID"
        exit 1
    fi
    log_success "Ubuntu 22.04 detected"
else
    log_error "Cannot determine OS version"
    exit 1
fi

# -----------------------------------------------------------------------------
# Set locale
# -----------------------------------------------------------------------------
log_step "Setting up Locale"

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

log_success "Locale configured"

# -----------------------------------------------------------------------------
# Enable required repositories
# -----------------------------------------------------------------------------
log_step "Enabling Required Repositories"

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

log_success "Universe repository enabled"

# -----------------------------------------------------------------------------
# Add ROS2 GPG key
# -----------------------------------------------------------------------------
log_step "Adding ROS2 Repository"

sudo apt update && sudo apt install -y curl

# Add the ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

log_success "ROS2 GPG key added"

# Add the repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

log_success "ROS2 repository added"

# -----------------------------------------------------------------------------
# Install ROS2 Humble
# -----------------------------------------------------------------------------
log_step "Installing ROS2 Humble Desktop"

log_info "Updating package index..."
sudo apt update

log_info "Upgrading existing packages..."
sudo apt upgrade -y

log_info "Installing ROS2 Humble Desktop (this may take a while)..."
sudo apt install -y ros-humble-desktop

log_success "ROS2 Humble Desktop installed"

# -----------------------------------------------------------------------------
# Install development tools
# -----------------------------------------------------------------------------
log_step "Installing Development Tools"

sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-dev-tools

log_success "Development tools installed"

# -----------------------------------------------------------------------------
# Initialize rosdep
# -----------------------------------------------------------------------------
log_step "Initializing rosdep"

if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi

rosdep update

log_success "rosdep initialized"

# -----------------------------------------------------------------------------
# Install additional ROS2 packages for the project
# -----------------------------------------------------------------------------
log_step "Installing Additional ROS2 Packages"

sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro

log_success "Additional ROS2 packages installed"

# -----------------------------------------------------------------------------
# Setup environment
# -----------------------------------------------------------------------------
log_step "Setting up Environment"

# Add to bashrc if not already present
BASHRC_LINE="source /opt/ros/humble/setup.bash"
if ! grep -q "$BASHRC_LINE" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble" >> ~/.bashrc
    echo "$BASHRC_LINE" >> ~/.bashrc
    log_success "Added ROS2 source to ~/.bashrc"
else
    log_success "ROS2 source already in ~/.bashrc"
fi

# Add CycloneDDS as default middleware
CYCLONE_LINE="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
if ! grep -q "$CYCLONE_LINE" ~/.bashrc; then
    echo "$CYCLONE_LINE" >> ~/.bashrc
    log_success "Set CycloneDDS as default middleware in ~/.bashrc"
else
    log_success "CycloneDDS already set in ~/.bashrc"
fi

# -----------------------------------------------------------------------------
# Verify installation
# -----------------------------------------------------------------------------
log_step "Verifying Installation"

source /opt/ros/humble/setup.bash

# Check ROS2 command
if command -v ros2 &> /dev/null; then
    ROS2_VERSION=$(ros2 --version 2>/dev/null || echo "unknown")
    log_success "ros2 command available: $ROS2_VERSION"
else
    log_error "ros2 command not found"
    exit 1
fi

# Check key packages
for pkg in rclpy nav2_bringup slam_toolbox; do
    if ros2 pkg list 2>/dev/null | grep -q "^$pkg"; then
        log_success "Package $pkg installed"
    else
        log_warn "Package $pkg not found"
    fi
done

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
echo ""
echo -e "${GREEN}==========================================${NC}"
echo -e "${GREEN}  ROS2 Humble Installation Complete!${NC}"
echo -e "${GREEN}==========================================${NC}"
echo ""
echo "ROS2 Humble has been installed successfully."
echo ""
echo "To use ROS2 in a new terminal:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "Or start a new terminal (it's already in your ~/.bashrc)"
echo ""
echo "To verify:"
echo "  ros2 --version"
echo "  ros2 pkg list | grep nav2"
echo ""
echo "Next step - run the project setup:"
echo "  ./scripts/setup.sh"
echo ""
