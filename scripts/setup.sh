#!/bin/bash
# =============================================================================
# Unitree G1 Inspector - Environment Setup Script
# =============================================================================
# This script sets up all external dependencies for the unitree-g1-robot
# inspection system. It clones repos to external/ with pinned commits.
#
# Usage:
#   ./scripts/setup.sh           # Full setup
#   ./scripts/setup.sh --check   # Verify installation only
#   ./scripts/setup.sh --clean   # Remove external/ and start fresh
#
# Prerequisites:
#   - Ubuntu 22.04
#   - ROS2 Humble installed
#   - Conda with unitree_mujoco environment
#
# The script uses the 'unitree_mujoco' conda environment (Python 3.11)
# which has compatible cyclonedds bindings.
# =============================================================================

set -e

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
EXTERNAL_DIR="$PROJECT_ROOT/external"
CONDA_ENV="unitree_mujoco"
CONDA_DIR="$HOME/miniconda3"

# Pinned commits for reproducibility
# Update these when upgrading dependencies
declare -A REPO_COMMITS=(
    ["cyclonedds"]="0.10.5"  # Tag, not commit - stable release
    ["unitree_sdk2"]="main"
    ["unitree_sdk2_python"]="main"
    ["unitree_ros2"]="main"
    ["unitree_mujoco"]="main"
)

declare -A REPO_URLS=(
    ["cyclonedds"]="https://github.com/eclipse-cyclonedds/cyclonedds.git"
    ["unitree_sdk2"]="https://github.com/unitreerobotics/unitree_sdk2.git"
    ["unitree_sdk2_python"]="https://github.com/unitreerobotics/unitree_sdk2_python.git"
    ["unitree_ros2"]="https://github.com/unitreerobotics/unitree_ros2.git"
    ["unitree_mujoco"]="https://github.com/unitreerobotics/unitree_mujoco.git"
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

check_command() {
    if ! command -v "$1" &> /dev/null; then
        log_error "$1 is not installed"
        return 1
    fi
    return 0
}

# -----------------------------------------------------------------------------
# Activate Conda Environment
# -----------------------------------------------------------------------------
activate_conda() {
    log_step "Activating Conda Environment"

    # Check if conda is available
    if [ ! -f "$CONDA_DIR/etc/profile.d/conda.sh" ]; then
        log_error "Conda not found at $CONDA_DIR"
        log_error "Please install miniconda or update CONDA_DIR in this script"
        exit 1
    fi

    # Source conda
    source "$CONDA_DIR/etc/profile.d/conda.sh"

    # Check if target environment exists
    if ! conda env list | grep -q "^$CONDA_ENV "; then
        log_error "Conda environment '$CONDA_ENV' not found"
        log_error "Create it with: conda create -n $CONDA_ENV python=3.11"
        exit 1
    fi

    # Activate the environment
    conda activate "$CONDA_ENV"

    PYTHON_VERSION=$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
    log_success "Activated conda env '$CONDA_ENV' (Python $PYTHON_VERSION)"
}

# -----------------------------------------------------------------------------
# Prerequisite Checks
# -----------------------------------------------------------------------------
check_prerequisites() {
    log_step "Checking Prerequisites"

    local failed=0

    # Check Ubuntu version
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [[ "$VERSION_ID" != "22.04" ]]; then
            log_warn "Expected Ubuntu 22.04, found $VERSION_ID"
        else
            log_success "Ubuntu 22.04 detected"
        fi
    fi

    # Check required commands
    for cmd in git cmake; do
        if check_command "$cmd"; then
            log_success "$cmd found"
        else
            failed=1
        fi
    done

    # Check conda environment is active
    if [[ "$CONDA_DEFAULT_ENV" == "$CONDA_ENV" ]]; then
        PYTHON_VERSION=$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
        log_success "Conda env '$CONDA_ENV' active (Python $PYTHON_VERSION)"
    else
        log_error "Conda env '$CONDA_ENV' not active (current: ${CONDA_DEFAULT_ENV:-none})"
        failed=1
    fi

    # Check Python version (should be 3.10 or 3.11 for cyclonedds compatibility)
    PYTHON_VERSION=$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
    if [[ "$PYTHON_VERSION" == "3.10" ]] || [[ "$PYTHON_VERSION" == "3.11" ]]; then
        log_success "Python $PYTHON_VERSION (3.10 or 3.11 required for cyclonedds)"
    else
        log_warn "Python $PYTHON_VERSION - cyclonedds may have compatibility issues"
        log_warn "Recommended: Python 3.10 or 3.11"
    fi

    # Check ROS2 Humble
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        log_success "ROS2 Humble found"
    else
        log_warn "ROS2 Humble not found at /opt/ros/humble"
        log_warn "Install with: ./scripts/install-ros2-humble.sh"
        failed=1
    fi

    if [ $failed -eq 1 ]; then
        log_error "Prerequisites check failed"
        exit 1
    fi
}

# -----------------------------------------------------------------------------
# System Dependencies
# -----------------------------------------------------------------------------
install_system_deps() {
    log_step "Installing System Dependencies"

    PACKAGES=(
        # Build tools
        build-essential
        cmake
        git
        # CycloneDDS dependencies
        libyaml-cpp-dev
        libboost-all-dev
        # Unitree SDK dependencies
        libeigen3-dev
        libspdlog-dev
        libfmt-dev
        # MuJoCo dependencies
        libglfw3-dev
        libgl1-mesa-dev
        # ROS2 Nav2 dependencies (if not installed with ros-humble-desktop)
        ros-humble-navigation2
        ros-humble-nav2-bringup
        ros-humble-slam-toolbox
        ros-humble-robot-localization
        # ROS2 development
        python3-colcon-common-extensions
        python3-rosdep
    )

    MISSING_PACKAGES=()
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg -s "$pkg" &>/dev/null 2>&1; then
            MISSING_PACKAGES+=("$pkg")
        fi
    done

    if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
        log_info "Installing missing packages: ${MISSING_PACKAGES[*]}"
        sudo apt update
        sudo apt install -y "${MISSING_PACKAGES[@]}"
    else
        log_success "All system dependencies already installed"
    fi
}

# -----------------------------------------------------------------------------
# Clone Repository
# -----------------------------------------------------------------------------
clone_repo() {
    local name=$1
    local url=${REPO_URLS[$name]}
    local ref=${REPO_COMMITS[$name]}
    local dest="$EXTERNAL_DIR/$name"

    if [ -d "$dest" ]; then
        log_info "$name already exists, checking version..."
        cd "$dest"

        # Fetch latest
        git fetch --tags --quiet 2>/dev/null || true

        # Check if we're at the right ref
        current_ref=$(git describe --tags --exact-match 2>/dev/null || git rev-parse --short HEAD)
        if [[ "$current_ref" == "$ref"* ]] || [[ "$ref" == "main" ]]; then
            log_success "$name at $current_ref"
        else
            log_warn "$name at $current_ref, expected $ref"
            log_info "Checking out $ref..."
            git checkout "$ref" --quiet
        fi
    else
        log_info "Cloning $name..."
        git clone "$url" "$dest" --quiet
        cd "$dest"

        # Checkout specific ref if not main
        if [[ "$ref" != "main" ]]; then
            log_info "Checking out $ref..."
            git checkout "$ref" --quiet
        fi

        log_success "$name cloned"
    fi
}

# -----------------------------------------------------------------------------
# Build CycloneDDS
# -----------------------------------------------------------------------------
build_cyclonedds() {
    log_step "Building CycloneDDS"

    local cyclone_dir="$EXTERNAL_DIR/cyclonedds"

    if [ -f "/usr/local/lib/libddsc.so" ]; then
        log_success "CycloneDDS already installed"
        return
    fi

    cd "$cyclone_dir"
    mkdir -p build && cd build

    log_info "Configuring CycloneDDS..."
    cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF \
        -DENABLE_SSL=OFF \
        > /dev/null

    log_info "Building CycloneDDS..."
    make -j$(nproc) > /dev/null

    log_info "Installing CycloneDDS (requires sudo)..."
    sudo make install > /dev/null
    sudo ldconfig

    log_success "CycloneDDS installed"
}

# -----------------------------------------------------------------------------
# Build Unitree SDK2 (C++)
# -----------------------------------------------------------------------------
build_unitree_sdk2() {
    log_step "Building Unitree SDK2 (C++)"

    local sdk_dir="$EXTERNAL_DIR/unitree_sdk2"

    if [ -f "/usr/local/lib/libunitree_sdk2.so" ] || [ -f "$sdk_dir/build/libunitree_sdk2.so" ]; then
        log_success "Unitree SDK2 C++ already built"
        return
    fi

    cd "$sdk_dir"
    mkdir -p build && cd build

    log_info "Configuring Unitree SDK2..."
    cmake .. -DBUILD_EXAMPLES=OFF > /dev/null

    log_info "Building Unitree SDK2..."
    make -j$(nproc) > /dev/null

    log_info "Installing Unitree SDK2 (requires sudo)..."
    sudo make install > /dev/null
    sudo ldconfig

    log_success "Unitree SDK2 C++ installed"
}

# -----------------------------------------------------------------------------
# Install Unitree SDK2 Python
# -----------------------------------------------------------------------------
install_unitree_sdk2_python() {
    log_step "Installing Unitree SDK2 Python"

    if python -c "import unitree_sdk2py" 2>/dev/null; then
        log_success "unitree_sdk2py already installed"
        return
    fi

    cd "$EXTERNAL_DIR/unitree_sdk2_python"

    log_info "Installing unitree_sdk2_python..."
    pip install -e . --quiet

    log_success "unitree_sdk2_python installed"
}

# -----------------------------------------------------------------------------
# Install MuJoCo Dependencies
# -----------------------------------------------------------------------------
install_mujoco_deps() {
    log_step "Installing MuJoCo Dependencies"

    if python -c "import mujoco" 2>/dev/null; then
        log_success "MuJoCo Python already installed"
    else
        log_info "Installing MuJoCo..."
        pip install mujoco pygame --quiet
        log_success "MuJoCo installed"
    fi

    # Configure unitree_mujoco for G1
    local mujoco_config="$EXTERNAL_DIR/unitree_mujoco/simulate_python/config.py"
    if [ -f "$mujoco_config" ]; then
        if grep -q 'ROBOT = "go2"' "$mujoco_config" 2>/dev/null; then
            log_info "Configuring unitree_mujoco for G1 robot..."
            sed -i 's/ROBOT = "go2"/ROBOT = "g1"/g' "$mujoco_config"
            log_success "unitree_mujoco configured for G1"
        else
            log_success "unitree_mujoco already configured for G1"
        fi
    fi
}

# -----------------------------------------------------------------------------
# Setup ROS2 Environment
# -----------------------------------------------------------------------------
setup_ros2_env() {
    log_step "Setting up ROS2 Environment"

    # Source ROS2
    source /opt/ros/humble/setup.bash

    # Set CycloneDDS as default middleware
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    # Check if rmw_cyclonedds_cpp is installed
    if ! ros2 pkg list 2>/dev/null | grep -q rmw_cyclonedds_cpp; then
        log_info "Installing ROS2 CycloneDDS middleware..."
        sudo apt install -y ros-humble-rmw-cyclonedds-cpp
    fi

    # Initialize rosdep if needed
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        log_info "Initializing rosdep..."
        sudo rosdep init 2>/dev/null || true
    fi

    rosdep update --quiet 2>/dev/null || true

    log_success "ROS2 environment configured"
}

# -----------------------------------------------------------------------------
# Create .gitignore for external/
# -----------------------------------------------------------------------------
setup_gitignore() {
    log_step "Setting up .gitignore"

    local gitignore="$PROJECT_ROOT/.gitignore"

    # Check if external/ is already in .gitignore
    if [ -f "$gitignore" ] && grep -q "^external/" "$gitignore"; then
        log_success "external/ already in .gitignore"
    else
        log_info "Adding external/ to .gitignore..."
        echo "" >> "$gitignore"
        echo "# External dependencies (cloned by setup.sh)" >> "$gitignore"
        echo "external/" >> "$gitignore"
        log_success "Updated .gitignore"
    fi

    # Create .gitkeep to track the external directory structure
    touch "$EXTERNAL_DIR/.gitkeep"
}

# -----------------------------------------------------------------------------
# Verification
# -----------------------------------------------------------------------------
verify_installation() {
    log_step "Verifying Installation"

    local failed=0

    # Check directories
    for repo in "${!REPO_URLS[@]}"; do
        if [ -d "$EXTERNAL_DIR/$repo" ]; then
            log_success "$repo directory exists"
        else
            log_error "$repo directory missing"
            failed=1
        fi
    done

    # Check CycloneDDS
    if [ -f "/usr/local/lib/libddsc.so" ]; then
        log_success "CycloneDDS library installed"
    else
        log_error "CycloneDDS library not found"
        failed=1
    fi

    # Check Unitree SDK2 C++ (static library .a or shared .so)
    if [ -f "/usr/local/lib/libunitree_sdk2.so" ] || [ -f "/usr/local/lib/libunitree_sdk2.a" ]; then
        log_success "Unitree SDK2 C++ library installed"
    else
        log_warn "Unitree SDK2 C++ library not in /usr/local/lib (may be in build/)"
    fi

    # Check Python packages (using conda env's python)
    if python -c "import unitree_sdk2py" 2>/dev/null; then
        log_success "unitree_sdk2py importable"
    else
        log_error "unitree_sdk2py not importable"
        failed=1
    fi

    if python -c "import mujoco" 2>/dev/null; then
        log_success "MuJoCo importable"
    else
        log_error "MuJoCo not importable"
        failed=1
    fi

    if python -c "import cyclonedds" 2>/dev/null; then
        log_success "cyclonedds Python importable"
    else
        log_error "cyclonedds Python not importable"
        failed=1
    fi

    # Check ROS2 packages
    source /opt/ros/humble/setup.bash 2>/dev/null || true

    for pkg in navigation2 slam_toolbox robot_localization; do
        if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
            log_success "ROS2 $pkg installed"
        else
            log_warn "ROS2 $pkg not found"
        fi
    done

    echo ""
    if [ $failed -eq 0 ]; then
        log_success "All verifications passed!"
        return 0
    else
        log_error "Some verifications failed"
        return 1
    fi
}

# -----------------------------------------------------------------------------
# Print Summary
# -----------------------------------------------------------------------------
print_summary() {
    echo ""
    echo -e "${GREEN}==========================================${NC}"
    echo -e "${GREEN}  Setup Complete!${NC}"
    echo -e "${GREEN}==========================================${NC}"
    echo ""
    echo "External dependencies installed to:"
    echo "  $EXTERNAL_DIR/"
    echo ""
    echo "Conda environment: $CONDA_ENV"
    echo ""
    echo "To use the environment (add to ~/.bashrc):"
    echo ""
    echo "  # Activate conda environment"
    echo "  source $CONDA_DIR/etc/profile.d/conda.sh"
    echo "  conda activate $CONDA_ENV"
    echo ""
    echo "  # Source ROS2"
    echo "  source /opt/ros/humble/setup.bash"
    echo ""
    echo "  # Set CycloneDDS as middleware"
    echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    echo "  export CYCLONEDDS_HOME=/usr/local"
    echo ""
    echo "  # Build the workspace"
    echo "  cd $PROJECT_ROOT"
    echo "  colcon build"
    echo ""
    echo "  # Source the workspace"
    echo "  source install/setup.bash"
    echo ""
    echo "To test MuJoCo simulation:"
    echo "  conda activate $CONDA_ENV"
    echo "  cd $EXTERNAL_DIR/unitree_mujoco/simulate_python"
    echo "  python unitree_mujoco.py"
    echo ""
}

# -----------------------------------------------------------------------------
# Clean
# -----------------------------------------------------------------------------
clean() {
    log_step "Cleaning external/ directory"

    if [ -d "$EXTERNAL_DIR" ]; then
        log_warn "This will remove all cloned repositories in external/"
        read -p "Are you sure? (y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$EXTERNAL_DIR"
            mkdir -p "$EXTERNAL_DIR"
            touch "$EXTERNAL_DIR/.gitkeep"
            log_success "Cleaned external/ directory"
        else
            log_info "Aborted"
        fi
    else
        log_info "external/ directory does not exist"
    fi
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
main() {
    echo ""
    echo -e "${BLUE}==========================================${NC}"
    echo -e "${BLUE}  Unitree G1 Inspector - Setup Script${NC}"
    echo -e "${BLUE}==========================================${NC}"
    echo ""

    # Parse arguments
    case "${1:-}" in
        --check)
            activate_conda
            check_prerequisites
            verify_installation
            exit $?
            ;;
        --clean)
            clean
            exit 0
            ;;
        --help|-h)
            echo "Usage: $0 [--check|--clean|--help]"
            echo ""
            echo "Options:"
            echo "  --check   Verify installation only"
            echo "  --clean   Remove external/ and start fresh"
            echo "  --help    Show this help message"
            echo ""
            exit 0
            ;;
    esac

    # Activate conda environment first
    activate_conda

    # Create external directory
    mkdir -p "$EXTERNAL_DIR"

    # Run setup steps
    check_prerequisites
    install_system_deps

    # Clone all repositories
    log_step "Cloning External Repositories"
    for repo in "${!REPO_URLS[@]}"; do
        clone_repo "$repo"
    done

    # Build and install
    build_cyclonedds
    build_unitree_sdk2
    install_unitree_sdk2_python
    install_mujoco_deps
    setup_ros2_env
    setup_gitignore

    # Verify
    verify_installation

    # Summary
    print_summary
}

# Run main
main "$@"
