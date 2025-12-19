#!/bin/bash
# verify-robot-setup.sh - Validate robot environment for Barry demo
# Story 1-6: Dual-Environment Deployment
#
# Run this on the robot to verify all dependencies and configuration.
#
# Usage:
#   ./scripts/verify-robot-setup.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass_count=0
warn_count=0
fail_count=0

check_pass() {
    echo -e "  ${GREEN}[PASS]${NC} $1"
    ((pass_count++))
}

check_warn() {
    echo -e "  ${YELLOW}[WARN]${NC} $1"
    ((warn_count++))
}

check_fail() {
    echo -e "  ${RED}[FAIL]${NC} $1"
    ((fail_count++))
}

echo ""
echo "=== Robot Environment Verification ==="
echo ""

# Check 1: nlohmann-json version
echo "[1/8] nlohmann-json version:"
if dpkg -s nlohmann-json3-dev &>/dev/null; then
    version=$(dpkg -s nlohmann-json3-dev 2>/dev/null | grep "^Version:" | awk '{print $2}')
    if [[ "$version" == 3.7* ]]; then
        check_pass "Version $version (3.7.x compatible)"
    else
        check_warn "Version $version - expected 3.7.x for Jetson compatibility"
    fi
else
    check_fail "Not installed - run: sudo apt install nlohmann-json3-dev"
fi

# Check 2: OpenCV version
echo "[2/8] OpenCV version:"
if command -v pkg-config &>/dev/null; then
    opencv_ver=$(pkg-config --modversion opencv4 2>/dev/null || echo "")
    if [ -n "$opencv_ver" ]; then
        check_pass "OpenCV $opencv_ver"
    else
        check_fail "OpenCV 4 not found - run: sudo apt install libopencv-dev"
    fi
else
    check_warn "pkg-config not found"
fi

# Check 3: CycloneDDS available
echo "[3/8] CycloneDDS:"
if [ -f /usr/local/lib/libddsc.so ] || [ -f /usr/lib/libddsc.so ]; then
    check_pass "libddsc.so found"
else
    check_warn "libddsc.so not found - will be installed with unitree_sdk2"
fi

# Check 4: Model files
echo "[4/8] Model files:"
cd "$PROJECT_ROOT"
models_ok=true
for model in "res10_300x300_ssd_iter_140000.caffemodel" "deploy.prototxt" "face_recognition_sface_2021dec.onnx"; do
    # Check multiple locations (matches GreeterConfig::findModelPath)
    found=false
    for dir in "./models" "${HOME}/.g1_inspector/models" "/opt/g1_inspector/models"; do
        if [ -f "${dir}/${model}" ]; then
            found=true
            break
        fi
    done
    if [ "$found" = true ]; then
        check_pass "$model"
    else
        check_fail "$model - run: ./scripts/setup-robot.sh --models"
        models_ok=false
    fi
done

# Check 5: Config files
echo "[5/8] Config files:"
if [ -f "${PROJECT_ROOT}/config/greeter.yaml" ]; then
    check_pass "config/greeter.yaml"
else
    check_warn "config/greeter.yaml not found"
fi

if [ -f "${PROJECT_ROOT}/config/cyclonedds-robot.xml" ]; then
    check_pass "config/cyclonedds-robot.xml"
else
    check_warn "config/cyclonedds-robot.xml not found"
fi

# Check 6: Environment variables
echo "[6/8] Environment variables:"
if [ -n "$CYCLONEDDS_URI" ]; then
    check_pass "CYCLONEDDS_URI is set"
else
    check_warn "CYCLONEDDS_URI not set - source config/robot.env"
fi

if [ -n "$ANTHROPIC_API_KEY" ]; then
    check_pass "ANTHROPIC_API_KEY is set"
else
    check_warn "ANTHROPIC_API_KEY not set (required for LLM calls)"
fi

# Check 7: Jetson platform
echo "[7/8] Platform detection:"
if [ -f /etc/nv_tegra_release ]; then
    tegra_info=$(cat /etc/nv_tegra_release | head -1)
    check_pass "Jetson detected: $tegra_info"
else
    check_warn "Not Jetson (no /etc/nv_tegra_release)"
fi

# Check 8: Build status
echo "[8/8] Build status:"
if [ -f "${PROJECT_ROOT}/build/g1_inspector" ]; then
    check_pass "g1_inspector binary exists"
else
    check_warn "g1_inspector not built - run: mkdir -p build && cd build && cmake .. -DROBOT_BUILD=ON && make"
fi

# Summary
echo ""
echo "=== Verification Summary ==="
echo -e "  ${GREEN}PASS:${NC} $pass_count"
echo -e "  ${YELLOW}WARN:${NC} $warn_count"
echo -e "  ${RED}FAIL:${NC} $fail_count"
echo ""

if [ $fail_count -gt 0 ]; then
    echo "Some checks failed. Fix issues above before running the demo."
    exit 1
elif [ $warn_count -gt 0 ]; then
    echo "Some warnings. Demo may work but check configuration."
    exit 0
else
    echo "All checks passed! Ready to run the demo."
    exit 0
fi
