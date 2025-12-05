#!/bin/bash
# =============================================================================
# External Dependencies Setup Script
# =============================================================================
# Clones and builds all external dependencies for the G1 Inspector project.
#
# Usage:
#   ./scripts/setup-external.sh
#
# Run this inside the Docker container or on a Linux development machine.
# =============================================================================

set -e

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
EXTERNAL_DIR="${PROJECT_ROOT}/external"

# Installation prefix for libraries
INSTALL_PREFIX="/opt/unitree_robotics"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "============================================================"
echo "     G1 INSPECTOR - EXTERNAL DEPENDENCIES SETUP"
echo "============================================================"
echo
echo "Project root:    ${PROJECT_ROOT}"
echo "External dir:    ${EXTERNAL_DIR}"
echo "Install prefix:  ${INSTALL_PREFIX}"
echo

# Ensure external directory exists
mkdir -p "$EXTERNAL_DIR"
cd "$EXTERNAL_DIR"

# =============================================================================
# UNITREE SDK2
# =============================================================================
echo -e "${BLUE}[1/1]${NC} Setting up unitree_sdk2..."

SDK_DIR="${EXTERNAL_DIR}/unitree_sdk2"

if [ ! -d "$SDK_DIR" ]; then
    echo -e "${YELLOW}[INFO]${NC} Cloning unitree_sdk2..."
    git clone https://github.com/unitreerobotics/unitree_sdk2.git
else
    echo -e "${GREEN}[OK]${NC} unitree_sdk2 already cloned"
fi

cd "$SDK_DIR"

# Check if already installed
if [ -f "${INSTALL_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" ]; then
    echo -e "${GREEN}[OK]${NC} unitree_sdk2 already installed at ${INSTALL_PREFIX}"
else
    echo -e "${YELLOW}[INFO]${NC} Building unitree_sdk2..."

    # Clean previous build if exists
    rm -rf build
    mkdir -p build
    cd build

    # Configure with install prefix
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"

    # Build (use available cores)
    NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
    make -j"$NPROC"

    # Install (requires sudo for /opt)
    echo -e "${YELLOW}[INFO]${NC} Installing to ${INSTALL_PREFIX} (may require sudo)..."
    if [ -w "${INSTALL_PREFIX}" ] 2>/dev/null || [ -w "$(dirname ${INSTALL_PREFIX})" ] 2>/dev/null; then
        make install
    else
        sudo make install
    fi

    echo -e "${GREEN}[OK]${NC} unitree_sdk2 installed successfully"
fi

# =============================================================================
# SUMMARY
# =============================================================================
echo
echo "============================================================"
echo -e "${GREEN}External dependencies setup complete!${NC}"
echo "============================================================"
echo
echo "Installed libraries:"
echo "  - unitree_sdk2 -> ${INSTALL_PREFIX}"
echo
echo "CMake will find these with:"
echo "  list(APPEND CMAKE_PREFIX_PATH \"${INSTALL_PREFIX}\")"
echo
echo "Next steps:"
echo "  1. Build the project: mkdir build && cd build && cmake .. && make"
echo "  2. (Optional) Set up network: ./scripts/setup-network.sh"
echo "============================================================"
