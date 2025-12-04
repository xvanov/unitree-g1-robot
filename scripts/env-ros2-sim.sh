#!/bin/bash
# Source this file to set up the ROS2 simulation environment
# Usage: source ~/unitree-g1-robot/scripts/env-ros2-sim.sh

# Deactivate conda if active
if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "Deactivating conda environment: $CONDA_DEFAULT_ENV"
    conda deactivate 2>/dev/null || true
fi

# Ensure system Python is used
export PATH="/usr/bin:$PATH"

# Source ROS2
source /opt/ros/humble/setup.bash

# Set CycloneDDS middleware
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_HOME=/usr/local

# Avoid network timeouts on cloud instances
export ROS_LOCALHOST_ONLY=1
export ROS2_DOMAIN_ID=42

# Source workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi

echo "ROS2 Simulation Environment Ready"
echo "  Python: $(which python3) ($(python3 --version))"
echo "  ROS2: $(ros2 --version 2>/dev/null || echo 'not found')"
echo ""
echo "Run: ros2 launch g1_bringup sim_launch.py"
