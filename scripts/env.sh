#!/bin/bash
# =============================================================================
# Unitree G1 Robot - Unified Environment Script
# =============================================================================
# This is the SINGLE entry point for setting up the development environment.
# It handles Python version conflicts between ROS2 (requires 3.10) and conda.
#
# Usage:
#   source ~/unitree-g1-robot/scripts/env.sh [mode]
#
# Modes:
#   ros2      - ROS2 simulation (Python 3.10, NO conda) [default]
#   mujoco    - Standalone MuJoCo sim (conda unitree_mujoco, Python 3.11)
#   rl        - RL training with Isaac Lab (conda isaaclab, Python 3.11)
#   check     - Show current environment status
#
# Examples:
#   source ~/unitree-g1-robot/scripts/env.sh ros2
#   source ~/unitree-g1-robot/scripts/env.sh mujoco
#   source ~/unitree-g1-robot/scripts/env.sh check
#
# IMPORTANT: This script must be SOURCED, not executed!
# =============================================================================

# Prevent execution (must be sourced)
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "ERROR: This script must be sourced, not executed!"
    echo "Usage: source $0 [ros2|mujoco|rl|check]"
    exit 1
fi

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONDA_DIR="$HOME/miniconda3"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------
_env_log_info() {
    echo -e "${BLUE}[ENV]${NC} $1"
}

_env_log_success() {
    echo -e "${GREEN}[ENV]${NC} $1"
}

_env_log_warn() {
    echo -e "${YELLOW}[ENV]${NC} $1"
}

_env_log_error() {
    echo -e "${RED}[ENV]${NC} $1"
}

# -----------------------------------------------------------------------------
# Completely disable conda and use system Python
# -----------------------------------------------------------------------------
_deactivate_conda_completely() {
    # Deactivate any active conda environment
    if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
        _env_log_info "Deactivating conda environment: $CONDA_DEFAULT_ENV"

        # Source conda to get deactivate function
        if [[ -f "$CONDA_DIR/etc/profile.d/conda.sh" ]]; then
            source "$CONDA_DIR/etc/profile.d/conda.sh"
        fi

        # Deactivate all nested environments
        while [[ -n "$CONDA_DEFAULT_ENV" ]]; do
            conda deactivate 2>/dev/null || break
        done
    fi

    # Remove ALL conda paths from PATH
    local new_path=""
    local IFS=':'
    for p in $PATH; do
        if [[ "$p" != *"conda"* ]] && [[ "$p" != *"miniconda"* ]]; then
            if [[ -z "$new_path" ]]; then
                new_path="$p"
            else
                new_path="$new_path:$p"
            fi
        fi
    done
    export PATH="$new_path"

    # Ensure /usr/bin is first in PATH
    export PATH="/usr/bin:/usr/local/bin:$PATH"

    # Unset conda variables
    unset CONDA_DEFAULT_ENV
    unset CONDA_PREFIX
    unset CONDA_SHLVL
    unset CONDA_PROMPT_MODIFIER
    unset CONDA_EXE
    unset _CE_CONDA
    unset _CE_M
}

# -----------------------------------------------------------------------------
# Activate a specific conda environment
# -----------------------------------------------------------------------------
_activate_conda_env() {
    local env_name="$1"

    if [[ ! -f "$CONDA_DIR/etc/profile.d/conda.sh" ]]; then
        _env_log_error "Conda not found at $CONDA_DIR"
        return 1
    fi

    source "$CONDA_DIR/etc/profile.d/conda.sh"

    if ! conda env list | grep -q "^$env_name "; then
        _env_log_error "Conda environment '$env_name' not found"
        _env_log_info "Available environments:"
        conda env list
        return 1
    fi

    conda activate "$env_name"
    _env_log_success "Activated conda environment: $env_name"
}

# -----------------------------------------------------------------------------
# Check if ROS2 workspace needs rebuild
# -----------------------------------------------------------------------------
_check_workspace_python() {
    local executable="$PROJECT_ROOT/install/g1_bringup/lib/g1_bringup/mujoco_sim"

    if [[ ! -f "$executable" ]]; then
        return 1  # Needs build
    fi

    local shebang
    shebang=$(head -1 "$executable" 2>/dev/null)

    if [[ "$shebang" == *"python3.10"* ]] || [[ "$shebang" == *"/usr/bin/python3"* ]]; then
        return 0  # Correct Python
    else
        return 1  # Wrong Python version
    fi
}

# -----------------------------------------------------------------------------
# Setup ROS2 Simulation Environment (System Python 3.10)
# -----------------------------------------------------------------------------
_setup_ros2_env() {
    _env_log_info "Setting up ROS2 Simulation Environment..."

    # Step 1: Completely remove conda from the environment
    _deactivate_conda_completely

    # Step 2: Verify we have system Python 3.10
    local python_path
    python_path=$(which python3)
    local python_version
    python_version=$(python3 --version 2>&1)

    if [[ "$python_path" != "/usr/bin/python3" ]]; then
        _env_log_error "Expected /usr/bin/python3, got $python_path"
        _env_log_error "Please check your PATH configuration"
        return 1
    fi

    if ! python3 -c "import sys; exit(0 if sys.version_info[:2] == (3, 10) else 1)" 2>/dev/null; then
        _env_log_error "Python 3.10 required for ROS2 Humble, got $python_version"
        return 1
    fi

    _env_log_success "Using system Python: $python_version"

    # Step 3: Check ROS2 installation
    if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
        _env_log_error "ROS2 Humble not found at /opt/ros/humble"
        _env_log_error "Install with: ./scripts/install-ros2-humble.sh"
        return 1
    fi

    # Step 4: Source ROS2
    source /opt/ros/humble/setup.bash

    # Step 5: Set ROS2 middleware and network settings
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=42
    # Note: Using default CycloneDDS config (no CYCLONEDDS_URI)
    # Custom configs can break localhost discovery on some systems
    unset CYCLONEDDS_URI 2>/dev/null || true

    # Step 6: Check workspace and source if available
    if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
        # Check if built with correct Python
        if _check_workspace_python; then
            source "$PROJECT_ROOT/install/setup.bash"
            _env_log_success "Sourced workspace (Python 3.10)"
        else
            _env_log_warn "Workspace was built with wrong Python version!"
            _env_log_warn "Run: ./scripts/setup-ros2-sim.sh --build"
            _env_log_info "Sourcing workspace anyway (may have issues)..."
            source "$PROJECT_ROOT/install/setup.bash"
        fi
    else
        _env_log_warn "Workspace not built yet"
        _env_log_warn "Run: ./scripts/setup-ros2-sim.sh"
    fi

    # Step 7: Verify key dependencies
    if ! python3 -c "import rclpy" 2>/dev/null; then
        _env_log_error "rclpy not importable - ROS2 Python bindings broken"
        return 1
    fi

    if ! python3 -c "import mujoco" 2>/dev/null; then
        _env_log_warn "MuJoCo not installed for system Python"
        _env_log_warn "Install with: pip3 install --user mujoco"
    fi

    echo ""
    _env_log_success "ROS2 Simulation Environment Ready!"
    echo ""
    echo "  Python:     $(which python3) ($(python3 --version 2>&1 | cut -d' ' -f2))"
    echo "  ROS2:       Humble"
    echo "  Middleware: CycloneDDS"
    echo "  Workspace:  $PROJECT_ROOT"
    echo ""
    echo "  Launch simulation:"
    echo "    ros2 launch g1_bringup sim_launch.py"
    echo ""
}

# -----------------------------------------------------------------------------
# Setup Standalone MuJoCo Environment (Conda Python 3.11)
# -----------------------------------------------------------------------------
_setup_mujoco_env() {
    _env_log_info "Setting up Standalone MuJoCo Environment..."

    _activate_conda_env "unitree_mujoco" || return 1

    # Set CycloneDDS
    export CYCLONEDDS_HOME=/usr/local

    # Verify MuJoCo
    if ! python -c "import mujoco" 2>/dev/null; then
        _env_log_warn "MuJoCo not installed, installing..."
        pip install mujoco pygame --quiet
    fi

    echo ""
    _env_log_success "Standalone MuJoCo Environment Ready!"
    echo ""
    echo "  Python:  $(which python) ($(python --version 2>&1 | cut -d' ' -f2))"
    echo "  Conda:   unitree_mujoco"
    echo ""
    echo "  Run standalone MuJoCo sim:"
    echo "    cd $PROJECT_ROOT/external/unitree_mujoco/simulate_python"
    echo "    python unitree_mujoco.py"
    echo ""
}

# -----------------------------------------------------------------------------
# Setup RL Training Environment (Conda Python 3.11 with Isaac Lab)
# -----------------------------------------------------------------------------
_setup_rl_env() {
    _env_log_info "Setting up RL Training Environment..."

    _activate_conda_env "isaaclab" || return 1

    echo ""
    _env_log_success "RL Training Environment Ready!"
    echo ""
    echo "  Python:  $(which python) ($(python --version 2>&1 | cut -d' ' -f2))"
    echo "  Conda:   isaaclab"
    echo ""
    echo "  To train G1 (requires NVIDIA GPU):"
    echo "    cd ~/unitree_rl_lab"
    echo "    ./unitree_rl_lab.sh -t --task Unitree-G1-29dof-Velocity --headless"
    echo ""
}

# -----------------------------------------------------------------------------
# Show current environment status
# -----------------------------------------------------------------------------
_show_env_status() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  Environment Status${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""

    # Python
    echo "Python:"
    echo "  Current:     $(which python3) ($(python3 --version 2>&1 | cut -d' ' -f2))"
    echo "  System:      /usr/bin/python3 ($(/usr/bin/python3 --version 2>&1 | cut -d' ' -f2))"

    # Conda
    echo ""
    echo "Conda:"
    if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
        echo "  Active env:  $CONDA_DEFAULT_ENV"
    else
        echo "  Active env:  (none)"
    fi
    echo "  Available:"
    conda env list 2>/dev/null | grep -v "^#" | sed 's/^/    /' || echo "    conda not configured"

    # ROS2
    echo ""
    echo "ROS2:"
    if [[ -n "$ROS_DISTRO" ]]; then
        echo "  Distro:      $ROS_DISTRO"
        echo "  Middleware:  ${RMW_IMPLEMENTATION:-default}"
    else
        echo "  Status:      Not sourced"
    fi

    # Workspace
    echo ""
    echo "Workspace:"
    if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
        if _check_workspace_python; then
            echo "  Status:      Built (Python 3.10 - correct)"
        else
            echo "  Status:      Built (WRONG Python - needs rebuild)"
        fi
    else
        echo "  Status:      Not built"
    fi

    # Key packages (system Python)
    echo ""
    echo "System Python packages:"
    /usr/bin/python3 -c "import mujoco; print(f'  mujoco:      {mujoco.__version__}')" 2>/dev/null || echo "  mujoco:      NOT INSTALLED"
    /usr/bin/python3 -c "import rclpy; print('  rclpy:       OK')" 2>/dev/null || echo "  rclpy:       NOT AVAILABLE"
    /usr/bin/python3 -c "import numpy; print(f'  numpy:       {numpy.__version__}')" 2>/dev/null || echo "  numpy:       NOT INSTALLED"

    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo "To setup an environment:"
    echo "  source ~/unitree-g1-robot/scripts/env.sh ros2    # ROS2 simulation"
    echo "  source ~/unitree-g1-robot/scripts/env.sh mujoco  # Standalone MuJoCo"
    echo "  source ~/unitree-g1-robot/scripts/env.sh rl      # RL training"
    echo ""
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
_env_main() {
    local mode="${1:-ros2}"

    case "$mode" in
        ros2|sim|simulation)
            _setup_ros2_env
            ;;
        mujoco|standalone)
            _setup_mujoco_env
            ;;
        rl|train|training|isaaclab)
            _setup_rl_env
            ;;
        check|status)
            _show_env_status
            ;;
        help|-h|--help)
            echo "Usage: source $0 [mode]"
            echo ""
            echo "Modes:"
            echo "  ros2    - ROS2 simulation (Python 3.10, no conda) [default]"
            echo "  mujoco  - Standalone MuJoCo (conda unitree_mujoco)"
            echo "  rl      - RL training (conda isaaclab)"
            echo "  check   - Show environment status"
            echo ""
            ;;
        *)
            _env_log_error "Unknown mode: $mode"
            echo "Use: source $0 help"
            ;;
    esac
}

# Run main with all arguments
_env_main "$@"
