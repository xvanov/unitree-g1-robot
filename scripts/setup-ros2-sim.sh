#!/bin/bash
# =============================================================================
# Unitree G1 Inspector - ROS2 Simulation Setup
# =============================================================================
# This script sets up and builds the ROS2 simulation environment.
# It MUST use system Python 3.10 (required by ROS2 Humble).
#
# Usage:
#   ./scripts/setup-ros2-sim.sh           # Full setup + build
#   ./scripts/setup-ros2-sim.sh --build   # Rebuild workspace only
#   ./scripts/setup-ros2-sim.sh --check   # Verify installation only
#   ./scripts/setup-ros2-sim.sh --deps    # Install dependencies only
#
# IMPORTANT:
#   - This script will FORCE system Python 3.10 regardless of conda state
#   - Run this script directly (./setup-ros2-sim.sh), don't source it
#
# Prerequisites:
#   - Ubuntu 22.04
#   - ROS2 Humble installed (run ./scripts/install-ros2-humble.sh first)
#   - External repos cloned (run ./scripts/setup.sh first)
# =============================================================================

set -e

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
EXTERNAL_DIR="$PROJECT_ROOT/external"

# Force system Python - this is critical!
export PATH="/usr/bin:/usr/local/bin:$PATH"

# Remove conda from PATH completely for this script
CLEAN_PATH=""
IFS=':' read -ra PATH_PARTS <<< "$PATH"
for p in "${PATH_PARTS[@]}"; do
    if [[ "$p" != *"conda"* ]] && [[ "$p" != *"miniconda"* ]]; then
        if [[ -z "$CLEAN_PATH" ]]; then
            CLEAN_PATH="$p"
        else
            CLEAN_PATH="$CLEAN_PATH:$p"
        fi
    fi
done
export PATH="/usr/bin:/usr/local/bin:$CLEAN_PATH"

# Unset conda variables
unset CONDA_DEFAULT_ENV
unset CONDA_PREFIX
unset CONDA_SHLVL
unset CONDA_EXE

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------
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
# Verify Python 3.10 Environment
# -----------------------------------------------------------------------------
verify_python() {
    log_step "Verifying Python Environment"

    local python_path
    python_path=$(which python3)
    local python_version
    python_version=$(python3 --version 2>&1)

    echo "  Python path:    $python_path"
    echo "  Python version: $python_version"

    # Check it's system Python
    if [[ "$python_path" == *"conda"* ]] || [[ "$python_path" == *"miniconda"* ]]; then
        log_error "Still using conda Python: $python_path"
        log_error "This script should have removed conda from PATH"
        log_error ""
        log_error "Manual fix: Open a new terminal and run:"
        log_error "  export PATH=\"/usr/bin:\$PATH\""
        log_error "  ./scripts/setup-ros2-sim.sh"
        exit 1
    fi

    # Check version is 3.10
    if ! python3 -c "import sys; exit(0 if sys.version_info[:2] == (3, 10) else 1)" 2>/dev/null; then
        log_error "Python 3.10 is required for ROS2 Humble"
        log_error "Current version: $python_version"
        log_error "System Python 3.10 should be at /usr/bin/python3"
        exit 1
    fi

    log_success "Using system Python 3.10"
}

# -----------------------------------------------------------------------------
# Check Prerequisites
# -----------------------------------------------------------------------------
check_prerequisites() {
    log_step "Checking Prerequisites"

    local failed=0

    # Check Ubuntu version
    if [[ -f /etc/os-release ]]; then
        . /etc/os-release
        if [[ "$VERSION_ID" == "22.04" ]]; then
            log_success "Ubuntu 22.04"
        else
            log_warn "Expected Ubuntu 22.04, found $VERSION_ID"
        fi
    fi

    # Check ROS2 Humble
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        log_success "ROS2 Humble installed"
    else
        log_error "ROS2 Humble not found"
        log_error "Install with: ./scripts/install-ros2-humble.sh"
        failed=1
    fi

    # Check pip
    if /usr/bin/python3 -m pip --version &>/dev/null; then
        log_success "pip available"
    else
        log_warn "pip not available for system Python"
        log_info "Will attempt to install..."
    fi

    # Check G1 MuJoCo models
    if [[ -d "$EXTERNAL_DIR/unitree_mujoco/unitree_robots/g1" ]]; then
        log_success "G1 MuJoCo models found"
    else
        log_warn "G1 MuJoCo models not found at:"
        log_warn "  $EXTERNAL_DIR/unitree_mujoco/unitree_robots/g1"
        log_warn "Run ./scripts/setup.sh to clone repositories"
        # Not fatal - simulation can run without standalone MuJoCo
    fi

    # Check src directory
    if [[ -d "$PROJECT_ROOT/src" ]]; then
        log_success "Source directory exists"
    else
        log_error "Source directory not found: $PROJECT_ROOT/src"
        failed=1
    fi

    if [[ $failed -eq 1 ]]; then
        log_error "Prerequisites check failed"
        exit 1
    fi
}

# -----------------------------------------------------------------------------
# Install Python Dependencies
# -----------------------------------------------------------------------------
install_python_deps() {
    log_step "Installing Python Dependencies (System Python 3.10)"

    # Ensure pip is available
    if ! /usr/bin/python3 -m pip --version &>/dev/null; then
        log_info "Installing pip..."
        sudo apt update
        sudo apt install -y python3-pip
    fi

    # Install MuJoCo
    if /usr/bin/python3 -c "import mujoco" 2>/dev/null; then
        local mujoco_ver
        mujoco_ver=$(/usr/bin/python3 -c "import mujoco; print(mujoco.__version__)")
        log_success "MuJoCo $mujoco_ver already installed"
    else
        log_info "Installing MuJoCo..."
        /usr/bin/pip3 install --user mujoco
        log_success "MuJoCo installed"
    fi

    # Install numpy (should already be there with ROS2)
    if /usr/bin/python3 -c "import numpy" 2>/dev/null; then
        log_success "NumPy available"
    else
        log_info "Installing NumPy..."
        /usr/bin/pip3 install --user numpy
    fi

    # Verify imports work
    log_info "Verifying Python imports..."
    /usr/bin/python3 -c "
import mujoco
import numpy
print(f'  MuJoCo {mujoco.__version__}')
print(f'  NumPy {numpy.__version__}')
"
    log_success "Python dependencies verified"
}

# -----------------------------------------------------------------------------
# Setup ROS2 Environment
# -----------------------------------------------------------------------------
setup_ros2_env() {
    log_step "Setting up ROS2 Environment"

    # Source ROS2
    source /opt/ros/humble/setup.bash

    # Set CycloneDDS as middleware
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_HOME=/usr/local

    # Network settings for cloud/headless instances
    export ROS_LOCALHOST_ONLY=1
    export ROS_DOMAIN_ID=42

    # Check CycloneDDS is available
    if timeout 30 ros2 pkg list 2>/dev/null | grep -q rmw_cyclonedds_cpp; then
        log_success "CycloneDDS middleware available"
    else
        log_info "Installing CycloneDDS middleware..."
        sudo apt install -y ros-humble-rmw-cyclonedds-cpp
        log_success "CycloneDDS installed"
    fi

    log_success "ROS2 environment configured"
}

# -----------------------------------------------------------------------------
# Build ROS2 Workspace
# -----------------------------------------------------------------------------
build_workspace() {
    log_step "Building ROS2 Workspace"

    cd "$PROJECT_ROOT"

    # Check current build Python version
    local needs_clean=false
    local mujoco_sim="$PROJECT_ROOT/install/g1_bringup/lib/g1_bringup/mujoco_sim"

    if [[ -f "$mujoco_sim" ]]; then
        local shebang
        shebang=$(head -1 "$mujoco_sim")
        if [[ "$shebang" != *"python3.10"* ]] && [[ "$shebang" != *"/usr/bin/python3"* ]]; then
            log_warn "Workspace built with wrong Python: $shebang"
            needs_clean=true
        fi
    fi

    # Clean if forced or wrong Python
    if [[ "${1:-}" == "--clean" ]] || [[ "$needs_clean" == true ]]; then
        log_info "Cleaning previous build..."
        rm -rf build install log
    fi

    # Verify we're using system Python before build
    log_info "Build environment:"
    echo "  PYTHON: $(which python3)"
    echo "  VERSION: $(python3 --version)"

    # Build with colcon
    log_info "Running colcon build..."

    # Use explicit Python path in colcon
    colcon build \
        --symlink-install \
        --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        2>&1 | tee /tmp/colcon_build.log | tail -30

    if [[ ${PIPESTATUS[0]} -eq 0 ]]; then
        log_success "Build completed"
    else
        log_error "Build failed. Check /tmp/colcon_build.log for details"
        exit 1
    fi

    # Source the workspace
    source install/setup.bash

    # Verify the build used correct Python
    if [[ -f "$mujoco_sim" ]]; then
        local new_shebang
        new_shebang=$(head -1 "$mujoco_sim")
        log_info "Built executable shebang: $new_shebang"

        if [[ "$new_shebang" == *"python3.10"* ]] || [[ "$new_shebang" == *"/usr/bin/python3"* ]]; then
            log_success "Workspace built with correct Python 3.10"
        else
            log_error "Build still has wrong Python shebang!"
            log_error "Shebang: $new_shebang"
            log_error ""
            log_error "Try: rm -rf build install log && ./scripts/setup-ros2-sim.sh"
            exit 1
        fi
    fi
}

# -----------------------------------------------------------------------------
# Verify Installation
# -----------------------------------------------------------------------------
verify_installation() {
    log_step "Verifying Installation"

    local failed=0

    # Source environments
    source /opt/ros/humble/setup.bash
    export ROS_LOCALHOST_ONLY=1
    export ROS_DOMAIN_ID=42

    if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
        source "$PROJECT_ROOT/install/setup.bash"
    fi

    # Check Python
    local python_path
    python_path=$(which python3)
    if [[ "$python_path" == "/usr/bin/python3" ]]; then
        log_success "System Python: $python_path"
    else
        log_warn "Python: $python_path (expected /usr/bin/python3)"
    fi

    # Check MuJoCo import
    if /usr/bin/python3 -c "import mujoco" 2>/dev/null; then
        log_success "MuJoCo importable"
    else
        log_error "MuJoCo not importable"
        failed=1
    fi

    # Check rclpy import
    if /usr/bin/python3 -c "import rclpy" 2>/dev/null; then
        log_success "rclpy importable"
    else
        log_error "rclpy not importable"
        failed=1
    fi

    # Check ROS2 packages
    log_info "Checking ROS2 packages..."
    for pkg in g1_bringup g1_interfaces g1_navigation g1_perception; do
        if timeout 30 ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
            log_success "Package: $pkg"
        else
            log_warn "Package not found: $pkg"
        fi
    done

    # Check executable
    local mujoco_sim="$PROJECT_ROOT/install/g1_bringup/lib/g1_bringup/mujoco_sim"
    if [[ -f "$mujoco_sim" ]]; then
        log_success "mujoco_sim executable exists"

        local shebang
        shebang=$(head -1 "$mujoco_sim" | grep -o 'python[0-9.]*' || echo "unknown")
        if [[ "$shebang" == *"3.10"* ]]; then
            log_success "Built with Python 3.10"
        else
            log_warn "Built with $shebang (may cause issues)"
        fi
    else
        log_error "mujoco_sim executable not found"
        failed=1
    fi

    # Check G1 model
    if [[ -f "$EXTERNAL_DIR/unitree_mujoco/unitree_robots/g1/scene.xml" ]]; then
        log_success "G1 MuJoCo models available"
    else
        log_warn "G1 MuJoCo models not found (standalone sim won't work)"
    fi

    echo ""
    if [[ $failed -eq 0 ]]; then
        log_success "All verifications passed!"
        return 0
    else
        log_error "Some verifications failed"
        return 1
    fi
}

# -----------------------------------------------------------------------------
# Print Usage Instructions
# -----------------------------------------------------------------------------
print_usage() {
    echo ""
    echo -e "${GREEN}==========================================${NC}"
    echo -e "${GREEN}  ROS2 Simulation Setup Complete!${NC}"
    echo -e "${GREEN}==========================================${NC}"
    echo ""
    echo "To run the simulation, use the unified environment script:"
    echo ""
    echo -e "  ${BLUE}source ~/unitree-g1-robot/scripts/env.sh ros2${NC}"
    echo -e "  ${BLUE}ros2 launch g1_bringup sim_launch.py${NC}"
    echo ""
    echo "Or manually:"
    echo ""
    echo "  # 1. Ensure no conda (new terminal recommended)"
    echo "  conda deactivate  # if needed"
    echo ""
    echo "  # 2. Source ROS2 and workspace"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    echo "  source ~/unitree-g1-robot/install/setup.bash"
    echo ""
    echo "  # 3. Launch"
    echo "  ros2 launch g1_bringup sim_launch.py"
    echo ""
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
main() {
    echo ""
    echo -e "${BLUE}==========================================${NC}"
    echo -e "${BLUE}  Unitree G1 - ROS2 Simulation Setup${NC}"
    echo -e "${BLUE}==========================================${NC}"
    echo ""

    # Always verify Python first
    verify_python

    # Parse arguments
    case "${1:-}" in
        --check)
            check_prerequisites
            setup_ros2_env
            verify_installation
            exit $?
            ;;
        --build)
            setup_ros2_env
            build_workspace --clean
            verify_installation
            print_usage
            exit $?
            ;;
        --deps)
            check_prerequisites
            install_python_deps
            exit $?
            ;;
        --help|-h)
            echo "Usage: $0 [--check|--build|--deps|--help]"
            echo ""
            echo "Options:"
            echo "  (none)    Full setup: check prereqs, install deps, build"
            echo "  --check   Verify installation only"
            echo "  --build   Clean rebuild workspace only"
            echo "  --deps    Install Python dependencies only"
            echo "  --help    Show this help"
            echo ""
            exit 0
            ;;
    esac

    # Full setup
    check_prerequisites
    install_python_deps
    setup_ros2_env
    build_workspace --clean
    verify_installation
    print_usage
}

# Run main
main "$@"
