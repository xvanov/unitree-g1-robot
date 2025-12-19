#!/bin/bash
# setup-robot.sh - Install dependencies and download models for Jetson/robot native build
# Story 1-6: Dual-Environment Deployment
#
# Usage:
#   ./scripts/setup-robot.sh              # Full setup (check deps, download models)
#   ./scripts/setup-robot.sh --check      # Check dependencies only, don't install
#   ./scripts/setup-robot.sh --models     # Download models only
#   ./scripts/setup-robot.sh --deps       # Install dependencies only
#
# This script handles apt packages and local builds for Jetson.
# SDK deployment is handled separately by deploy-to-robot.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Ensure all scripts are executable (in case cloned without preserving permissions)
chmod +x "${SCRIPT_DIR}"/*.sh 2>/dev/null || true

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ============================================================================
# Configuration
# ============================================================================

# Model URLs
MODEL_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel"
PROTO_URL="https://raw.githubusercontent.com/opencv/opencv/4.x/samples/dnn/face_detector/deploy.prototxt"
SFACE_URL="https://github.com/opencv/opencv_zoo/raw/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx"

# Search paths for models (matches GreeterConfig::findModelPath)
MODEL_SEARCH_PATHS=(
    "${PROJECT_ROOT}/models"
    "${HOME}/.g1_inspector/models"
    "/opt/g1_inspector/models"
)

# Required model files
REQUIRED_MODELS=(
    "res10_300x300_ssd_iter_140000.caffemodel"
    "deploy.prototxt"
    "face_recognition_sface_2021dec.onnx"
)

# Required apt packages for Jetson
REQUIRED_PACKAGES=(
    "build-essential"
    "cmake"
    "libopencv-dev"
    "nlohmann-json3-dev"
    "libyaml-cpp-dev"
    "libcurl4-openssl-dev"
    "libhpdf-dev"
    "libgtest-dev"
    "libzstd-dev"
    "libmsgpack-dev"
    "curl"
)

# ============================================================================
# Helper Functions
# ============================================================================

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_check() {
    local status=$1
    local name=$2
    if [ "$status" = "PASS" ]; then
        echo -e "  ${GREEN}✓${NC} $name"
    elif [ "$status" = "WARN" ]; then
        echo -e "  ${YELLOW}⚠${NC} $name"
    else
        echo -e "  ${RED}✗${NC} $name"
    fi
}

is_jetson() {
    # Detect Jetson platform via /etc/nv_tegra_release
    [ -f "/etc/nv_tegra_release" ]
}

# ============================================================================
# Dependency Checks
# ============================================================================

check_package() {
    dpkg -s "$1" &>/dev/null
}

check_library() {
    # Check if library file exists in common locations
    local lib=$1
    [ -f "/usr/lib/$lib" ] || \
    [ -f "/usr/local/lib/$lib" ] || \
    [ -f "/usr/lib/aarch64-linux-gnu/$lib" ] || \
    [ -f "/usr/lib/x86_64-linux-gnu/$lib" ]
}

check_nlohmann_json_version() {
    # Check nlohmann-json version for 3.7.3 compatibility
    if check_package nlohmann-json3-dev; then
        local version=$(dpkg -s nlohmann-json3-dev 2>/dev/null | grep "^Version:" | awk '{print $2}')
        echo "$version"
        return 0
    fi
    echo "not installed"
    return 1
}

check_opencv_version() {
    if command -v pkg-config &>/dev/null; then
        pkg-config --modversion opencv4 2>/dev/null || echo "not found"
    else
        echo "pkg-config not found"
    fi
}

check_cyclonedds() {
    # CycloneDDS installed by unitree_sdk2 build
    check_library "libddsc.so"
}

check_unitree_sdk2() {
    # Check if unitree_sdk2 is installed
    [ -d "/opt/unitree_robotics" ] && [ -f "/opt/unitree_robotics/unitree_sdk2Config.cmake" ]
}

check_livox_sdk2() {
    # Check Livox SDK2 in various locations
    [ -f "${PROJECT_ROOT}/external/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so" ] || \
    [ -f "/opt/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so" ] || \
    check_library "liblivox_lidar_sdk_shared.so" || \
    check_library "liblivox_lidar_sdk_static.a"
}

# ============================================================================
# Model Management
# ============================================================================

find_model() {
    local model=$1
    for path in "${MODEL_SEARCH_PATHS[@]}"; do
        if [ -f "${path}/${model}" ]; then
            echo "${path}/${model}"
            return 0
        fi
    done
    return 1
}

download_models() {
    log_info "Checking and downloading models..."

    # Create models directory in project root
    local model_dir="${PROJECT_ROOT}/models"
    mkdir -p "$model_dir"

    # Download face detection Caffe model
    if ! find_model "res10_300x300_ssd_iter_140000.caffemodel" &>/dev/null; then
        log_info "Downloading face detection model..."
        curl -L -o "${model_dir}/res10_300x300_ssd_iter_140000.caffemodel" "$MODEL_URL"
        log_check "PASS" "Downloaded res10_300x300_ssd_iter_140000.caffemodel"
    else
        local found=$(find_model "res10_300x300_ssd_iter_140000.caffemodel")
        log_check "PASS" "Found caffemodel at: $found"
    fi

    # Download prototxt
    if ! find_model "deploy.prototxt" &>/dev/null; then
        log_info "Downloading deploy.prototxt..."
        curl -L -o "${model_dir}/deploy.prototxt" "$PROTO_URL"
        log_check "PASS" "Downloaded deploy.prototxt"
    else
        local found=$(find_model "deploy.prototxt")
        log_check "PASS" "Found prototxt at: $found"
    fi

    # Download SFace model for face recognition
    if ! find_model "face_recognition_sface_2021dec.onnx" &>/dev/null; then
        log_info "Downloading SFace recognition model..."
        curl -L -o "${model_dir}/face_recognition_sface_2021dec.onnx" "$SFACE_URL"
        log_check "PASS" "Downloaded face_recognition_sface_2021dec.onnx"
    else
        local found=$(find_model "face_recognition_sface_2021dec.onnx")
        log_check "PASS" "Found SFace model at: $found"
    fi

    log_info "All models available in ${model_dir}/"
}

# ============================================================================
# Dependency Installation
# ============================================================================

install_apt_packages() {
    log_info "Installing required apt packages..."

    local missing=()
    for pkg in "${REQUIRED_PACKAGES[@]}"; do
        if ! check_package "$pkg"; then
            missing+=("$pkg")
        fi
    done

    if [ ${#missing[@]} -eq 0 ]; then
        log_info "All required packages already installed"
        return 0
    fi

    log_info "Installing missing packages: ${missing[*]}"
    sudo apt-get update
    sudo apt-get install -y "${missing[@]}"

    log_info "Package installation complete"
}

build_unitree_sdk2() {
    if check_unitree_sdk2; then
        log_check "PASS" "unitree_sdk2 already installed at /opt/unitree_robotics"
        return 0
    fi

    log_info "Building unitree_sdk2..."

    # Clone SDK
    if [ ! -d "/tmp/unitree_sdk2_build" ]; then
        git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2_build
    fi

    cd /tmp/unitree_sdk2_build
    mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install

    # Create symlink to /opt/unitree_robotics if installed elsewhere
    if [ ! -d "/opt/unitree_robotics" ] && [ -d "/usr/local/lib/cmake/unitree_sdk2" ]; then
        sudo mkdir -p /opt/unitree_robotics
        sudo ln -sf /usr/local/lib/cmake/unitree_sdk2/* /opt/unitree_robotics/
    fi

    log_check "PASS" "unitree_sdk2 built and installed"
    cd "$PROJECT_ROOT"
}

build_livox_sdk2() {
    if check_livox_sdk2; then
        log_check "PASS" "Livox SDK2 already available"
        return 0
    fi

    log_info "Building Livox SDK2..."

    local livox_dir="${PROJECT_ROOT}/external/Livox-SDK2"
    if [ ! -d "$livox_dir" ]; then
        mkdir -p "${PROJECT_ROOT}/external"
        git clone https://github.com/Livox-SDK/Livox-SDK2.git "$livox_dir"
    fi

    cd "$livox_dir"
    mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
    make -j$(nproc)

    log_check "PASS" "Livox SDK2 built at ${livox_dir}/build/"
    cd "$PROJECT_ROOT"
}

# ============================================================================
# nlohmann-json 3.7.3 Compatibility Check
# ============================================================================

check_json_compatibility() {
    log_info "Checking nlohmann-json 3.7.3 compatibility..."

    # Check for incompatible features in src/greeter/
    local incompatible_features=("to_bson" "from_bson" "patch_inplace")
    local found_issues=false

    for feature in "${incompatible_features[@]}"; do
        if grep -r "$feature" "${PROJECT_ROOT}/src/greeter/" 2>/dev/null | grep -v "^Binary" | grep -v ".o:"; then
            log_warn "Found potentially incompatible feature: $feature"
            found_issues=true
        fi
    done

    if [ "$found_issues" = true ]; then
        log_warn "Some 3.8+ features may be used. Verify compatibility."
        return 1
    fi

    log_check "PASS" "No incompatible nlohmann-json features found"
    return 0
}

# ============================================================================
# Full Status Report
# ============================================================================

run_status_check() {
    echo ""
    echo "============================================================"
    echo "  Robot Environment Status Check"
    echo "============================================================"
    echo ""

    # Platform detection
    if is_jetson; then
        log_info "Platform: Jetson (L4T detected)"
        if [ -f "/etc/nv_tegra_release" ]; then
            cat /etc/nv_tegra_release 2>/dev/null | head -1
        fi
    else
        log_info "Platform: Standard Linux (not Jetson)"
    fi
    echo ""

    # Check apt packages
    echo "=== APT Packages ==="
    for pkg in "${REQUIRED_PACKAGES[@]}"; do
        if check_package "$pkg"; then
            log_check "PASS" "$pkg"
        else
            log_check "FAIL" "$pkg (not installed)"
        fi
    done
    echo ""

    # Check versions
    echo "=== Versions ==="
    local json_ver=$(check_nlohmann_json_version)
    log_check "PASS" "nlohmann-json: $json_ver"

    local opencv_ver=$(check_opencv_version)
    log_check "PASS" "OpenCV: $opencv_ver"
    echo ""

    # Check SDKs
    echo "=== SDKs ==="
    if check_unitree_sdk2; then
        log_check "PASS" "unitree_sdk2 (/opt/unitree_robotics)"
    else
        log_check "FAIL" "unitree_sdk2 (not found - will be built)"
    fi

    if check_livox_sdk2; then
        log_check "PASS" "Livox SDK2"
    else
        log_check "WARN" "Livox SDK2 (not found - will be built)"
    fi

    if check_cyclonedds; then
        log_check "PASS" "CycloneDDS"
    else
        log_check "WARN" "CycloneDDS (installed with unitree_sdk2)"
    fi
    echo ""

    # Check models
    echo "=== Models ==="
    for model in "${REQUIRED_MODELS[@]}"; do
        if found=$(find_model "$model" 2>/dev/null); then
            log_check "PASS" "$model"
        else
            log_check "FAIL" "$model (not found - will download)"
        fi
    done
    echo ""

    # Check config files
    echo "=== Config Files ==="
    if [ -f "${PROJECT_ROOT}/config/greeter.yaml" ]; then
        log_check "PASS" "config/greeter.yaml"
    else
        log_check "WARN" "config/greeter.yaml (not found)"
    fi

    if [ -f "${PROJECT_ROOT}/config/cyclonedds-robot.xml" ]; then
        log_check "PASS" "config/cyclonedds-robot.xml"
    else
        log_check "WARN" "config/cyclonedds-robot.xml (will be created)"
    fi
    echo ""

    # Check environment
    echo "=== Environment ==="
    if [ -n "$CYCLONEDDS_URI" ]; then
        log_check "PASS" "CYCLONEDDS_URI is set"
    else
        log_check "WARN" "CYCLONEDDS_URI not set (source config/robot.env)"
    fi

    if [ -n "$ANTHROPIC_API_KEY" ]; then
        log_check "PASS" "ANTHROPIC_API_KEY is set"
    else
        log_check "WARN" "ANTHROPIC_API_KEY not set (required for LLM calls)"
    fi
    echo ""
}

# ============================================================================
# Main
# ============================================================================

main() {
    echo "============================================================"
    echo "  G1 Robot Setup Script (Story 1-6)"
    echo "============================================================"
    echo ""

    cd "$PROJECT_ROOT"

    case "${1:-}" in
        --check)
            run_status_check
            ;;
        --models)
            download_models
            ;;
        --deps)
            install_apt_packages
            build_unitree_sdk2
            build_livox_sdk2
            ;;
        --help|-h)
            echo "Usage: $0 [--check|--models|--deps|--help]"
            echo ""
            echo "  (no args)  Full setup: check deps, install missing, download models"
            echo "  --check    Check dependencies and show status report"
            echo "  --models   Download models only"
            echo "  --deps     Install dependencies only (apt packages, SDKs)"
            echo "  --help     Show this help"
            ;;
        "")
            # Full setup
            run_status_check
            echo ""
            log_info "Starting full setup..."
            echo ""

            install_apt_packages
            build_unitree_sdk2
            build_livox_sdk2
            download_models
            check_json_compatibility

            echo ""
            log_info "Setup complete! Next steps:"
            echo ""
            echo "  1. Build the project:"
            echo "     mkdir -p build && cd build"
            echo "     cmake .. -DROBOT_BUILD=ON"
            echo "     make -j\$(nproc)"
            echo ""
            echo "  2. Source environment (add to ~/.bashrc):"
            echo "     source ${PROJECT_ROOT}/config/robot.env"
            echo ""
            echo "  3. Run the demo:"
            echo "     ./g1_inspector --greeter --dry-run"
            echo ""
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
}

main "$@"
