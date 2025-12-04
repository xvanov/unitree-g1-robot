#!/bin/bash
set -e

echo "=========================================="
echo "Unitree RL Lab + MuJoCo Setup Script"
echo "=========================================="
echo ""
echo "This script installs:"
echo "  - Isaac Lab (unitree_rl_lab) for RL training"
echo "  - MuJoCo (unitree_mujoco) for sim2sim validation"
echo ""

# Configuration
INSTALL_DIR="/home/ubuntu"
CONDA_ENV="isaaclab"
PYTHON_VERSION="3.11"

# -----------------------------------------------------------------------------
# Step 1: System Dependencies
# -----------------------------------------------------------------------------
echo "[Step 1] Installing system dependencies..."
PACKAGES="libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev libglfw3-dev cmake build-essential git"
MISSING_PACKAGES=""

for pkg in $PACKAGES; do
    if ! dpkg -s "$pkg" &>/dev/null; then
        MISSING_PACKAGES="$MISSING_PACKAGES $pkg"
    fi
done

if [ -n "$MISSING_PACKAGES" ]; then
    echo "Installing missing packages:$MISSING_PACKAGES"
    sudo apt update
    sudo apt install -y $MISSING_PACKAGES
else
    echo "All system dependencies already installed."
fi

# -----------------------------------------------------------------------------
# Step 2: Conda Installation
# -----------------------------------------------------------------------------
echo ""
echo "[Step 2] Checking Conda installation..."
if [ -d "$INSTALL_DIR/miniconda3" ]; then
    echo "Conda already installed, skipping..."
else
    echo "Installing Miniconda..."
    cd "$INSTALL_DIR"
    wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O Miniconda3-latest-Linux-x86_64.sh
    bash Miniconda3-latest-Linux-x86_64.sh -b -p "$INSTALL_DIR/miniconda3"
    rm -f Miniconda3-latest-Linux-x86_64.sh
fi

# Initialize conda for this script
source "$INSTALL_DIR/miniconda3/etc/profile.d/conda.sh"

# -----------------------------------------------------------------------------
# Step 3: Create Conda Environment
# -----------------------------------------------------------------------------
echo ""
echo "[Step 3] Checking conda environment '$CONDA_ENV'..."
if conda env list | grep -q "^$CONDA_ENV "; then
    echo "Environment '$CONDA_ENV' already exists, skipping creation..."
else
    echo "Creating '$CONDA_ENV' environment with Python $PYTHON_VERSION..."
    conda create -n "$CONDA_ENV" python="$PYTHON_VERSION" -y
fi
conda activate "$CONDA_ENV"

# -----------------------------------------------------------------------------
# Step 4: Install Isaac Sim
# -----------------------------------------------------------------------------
echo ""
echo "[Step 4] Checking Isaac Sim installation..."
if python -c "import isaacsim" 2>/dev/null; then
    echo "Isaac Sim already installed, skipping..."
else
    echo "Installing Isaac Sim (this may take a while)..."
    pip install isaacsim --quiet
fi

# -----------------------------------------------------------------------------
# Step 5: Install Isaac Lab
# -----------------------------------------------------------------------------
echo ""
echo "[Step 5] Checking Isaac Lab installation..."
cd "$INSTALL_DIR"

if [ -d "IsaacLab" ]; then
    echo "Isaac Lab already cloned..."
else
    echo "Cloning Isaac Lab..."
    git clone https://github.com/isaac-sim/IsaacLab.git
fi

cd "$INSTALL_DIR/IsaacLab"

# Check if Isaac Lab is already installed
if python -c "import isaaclab" 2>/dev/null; then
    echo "Isaac Lab already installed, skipping..."
else
    echo "Installing Isaac Lab (this may take a while)..."
    ./isaaclab.sh -i
fi

# -----------------------------------------------------------------------------
# Step 6: Install Unitree RL Lab
# -----------------------------------------------------------------------------
echo ""
echo "[Step 6] Checking Unitree RL Lab installation..."
cd "$INSTALL_DIR"

if [ -d "unitree_rl_lab" ]; then
    echo "unitree_rl_lab already cloned..."
else
    echo "Cloning unitree_rl_lab..."
    git clone https://github.com/unitreerobotics/unitree_rl_lab.git
fi

cd "$INSTALL_DIR/unitree_rl_lab"

# Check if unitree_rl_lab is installed
if python -c "import unitree_rl_lab" 2>/dev/null; then
    echo "Unitree RL Lab already installed, skipping..."
else
    echo "Installing Unitree RL Lab..."
    ./unitree_rl_lab.sh -i
fi

# -----------------------------------------------------------------------------
# Step 7: Download Robot Models (URDF)
# -----------------------------------------------------------------------------
echo ""
echo "[Step 7] Checking robot models..."
cd "$INSTALL_DIR"

if [ -d "unitree_ros" ]; then
    echo "unitree_ros (robot models) already cloned..."
else
    echo "Cloning unitree_ros for robot URDF models..."
    git clone https://github.com/unitreerobotics/unitree_ros.git
fi

# Update the config path if needed
UNITREE_CONFIG="$INSTALL_DIR/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py"
if [ -f "$UNITREE_CONFIG" ]; then
    if grep -q 'UNITREE_ROS_DIR = ""' "$UNITREE_CONFIG" 2>/dev/null; then
        echo "Updating UNITREE_ROS_DIR in config..."
        sed -i "s|UNITREE_ROS_DIR = \"\"|UNITREE_ROS_DIR = \"$INSTALL_DIR/unitree_ros\"|g" "$UNITREE_CONFIG"
    fi
fi

# -----------------------------------------------------------------------------
# Step 8: Install MuJoCo Dependencies
# -----------------------------------------------------------------------------
echo ""
echo "[Step 8] Installing MuJoCo dependencies..."

# Check if mujoco is installed
if python -c "import mujoco" 2>/dev/null; then
    echo "MuJoCo Python already installed, skipping..."
else
    echo "Installing MuJoCo and pygame..."
    pip install mujoco pygame --quiet
fi

# -----------------------------------------------------------------------------
# Step 9: Install Unitree SDK2 Python
# -----------------------------------------------------------------------------
echo ""
echo "[Step 9] Checking Unitree SDK2 Python..."
cd "$INSTALL_DIR"

if [ -d "unitree_sdk2_python" ]; then
    echo "unitree_sdk2_python already cloned..."
else
    echo "Cloning unitree_sdk2_python..."
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
fi

# Check if installed
if python -c "import unitree_sdk2py" 2>/dev/null; then
    echo "Unitree SDK2 Python already installed, skipping..."
else
    echo "Installing Unitree SDK2 Python..."
    cd "$INSTALL_DIR/unitree_sdk2_python"
    pip install -e . --quiet
fi

# -----------------------------------------------------------------------------
# Step 10: Install Unitree MuJoCo
# -----------------------------------------------------------------------------
echo ""
echo "[Step 10] Checking Unitree MuJoCo..."
cd "$INSTALL_DIR"

if [ -d "unitree_mujoco" ]; then
    echo "unitree_mujoco already cloned..."
else
    echo "Cloning unitree_mujoco..."
    git clone https://github.com/unitreerobotics/unitree_mujoco.git
fi

# Configure for G1 robot
MUJOCO_CONFIG="$INSTALL_DIR/unitree_mujoco/simulate_python/config.py"
if [ -f "$MUJOCO_CONFIG" ]; then
    if grep -q 'ROBOT = "go2"' "$MUJOCO_CONFIG" 2>/dev/null; then
        echo "Configuring MuJoCo for G1 robot..."
        sed -i 's/ROBOT = "go2"/ROBOT = "g1"/g' "$MUJOCO_CONFIG"
    fi
fi

# -----------------------------------------------------------------------------
# Step 11: Install Unitree SDK2 (C++ - for deployment)
# -----------------------------------------------------------------------------
echo ""
echo "[Step 11] Checking Unitree SDK2 (C++)..."
cd "$INSTALL_DIR"

if [ -d "unitree_sdk2" ]; then
    echo "unitree_sdk2 already cloned..."
else
    echo "Cloning unitree_sdk2..."
    git clone https://github.com/unitreerobotics/unitree_sdk2.git
fi

# Build if not already built
if [ -f "/usr/local/lib/libunitree_sdk2.so" ] || [ -f "$INSTALL_DIR/unitree_sdk2/build/libunitree_sdk2.so" ]; then
    echo "Unitree SDK2 C++ already built, skipping..."
else
    echo "Building Unitree SDK2 C++..."
    cd "$INSTALL_DIR/unitree_sdk2"
    mkdir -p build && cd build
    cmake .. -DBUILD_EXAMPLES=OFF
    make -j$(nproc)
    sudo make install
fi

# -----------------------------------------------------------------------------
# Verification
# -----------------------------------------------------------------------------
echo ""
echo "[Verification] Checking installations..."
echo ""

VERIFY_FAILED=0

# Check conda environment
echo -n "  Conda env '$CONDA_ENV': "
if conda env list | grep -q "^$CONDA_ENV "; then
    echo "OK"
else
    echo "FAILED"
    VERIFY_FAILED=1
fi

# Check Python packages
python -c "import isaacsim; print('  Isaac Sim: OK')" 2>/dev/null || { echo "  Isaac Sim: FAILED (may need NVIDIA GPU)"; VERIFY_FAILED=1; }
python -c "import isaaclab; print('  Isaac Lab: OK')" 2>/dev/null || { echo "  Isaac Lab: FAILED"; VERIFY_FAILED=1; }
python -c "import unitree_rl_lab; print('  Unitree RL Lab: OK')" 2>/dev/null || { echo "  Unitree RL Lab: FAILED"; VERIFY_FAILED=1; }
python -c "import mujoco; print('  MuJoCo: OK')" 2>/dev/null || { echo "  MuJoCo: FAILED"; VERIFY_FAILED=1; }
python -c "import unitree_sdk2py; print('  Unitree SDK2 Python: OK')" 2>/dev/null || { echo "  Unitree SDK2 Python: FAILED"; VERIFY_FAILED=1; }

# Check directories
echo -n "  unitree_mujoco: "
if [ -d "$INSTALL_DIR/unitree_mujoco" ]; then echo "OK"; else echo "FAILED"; VERIFY_FAILED=1; fi

echo -n "  unitree_ros (models): "
if [ -d "$INSTALL_DIR/unitree_ros" ]; then echo "OK"; else echo "FAILED"; VERIFY_FAILED=1; fi

if [ $VERIFY_FAILED -eq 1 ]; then
    echo ""
    echo "WARNING: Some components failed verification!"
    echo "Note: Isaac Sim/Lab require NVIDIA GPU with proper drivers."
fi

# -----------------------------------------------------------------------------
# Done
# -----------------------------------------------------------------------------
echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "To activate the environment:"
echo "  source $INSTALL_DIR/miniconda3/etc/profile.d/conda.sh"
echo "  conda activate $CONDA_ENV"
echo ""
echo "To train G1 with Isaac Lab:"
echo "  cd $INSTALL_DIR/unitree_rl_lab"
echo "  ./unitree_rl_lab.sh -t --task Unitree-G1-29dof-Velocity --headless"
echo ""
echo "To run MuJoCo simulation:"
echo "  cd $INSTALL_DIR/unitree_mujoco/simulate_python"
echo "  python3 unitree_mujoco.py"
echo ""
echo "To list available training tasks:"
echo "  cd $INSTALL_DIR/unitree_rl_lab"
echo "  ./unitree_rl_lab.sh -l"
echo ""
