#!/bin/bash
# Gather Jetson system specs for Docker image creation

echo "========================================"
echo "JETSON SYSTEM SPECS"
echo "Gathered: $(date)"
echo "========================================"

echo ""
echo "=== OS & KERNEL ==="
uname -a
cat /etc/os-release
echo ""

echo "=== TEGRA/JETPACK VERSION ==="
cat /etc/nv_tegra_release 2>/dev/null || echo "Not found"
echo ""

echo "=== HARDWARE MODEL ==="
cat /proc/device-tree/model 2>/dev/null && echo "" || echo "Not found"
echo ""

echo "=== CPU ==="
nproc
lscpu | grep -E "^(Architecture|Model name|CPU\(s\))"
echo ""

echo "=== MEMORY ==="
free -h
echo ""

echo "=== DISK ==="
df -h /
echo ""

echo "=== NVIDIA/CUDA ==="
nvcc --version 2>/dev/null || echo "nvcc not found"
cat /usr/local/cuda/version.txt 2>/dev/null || echo "CUDA version.txt not found"
echo ""

echo "=== CUDA PACKAGES ==="
dpkg -l | grep -i cuda | head -20
echo ""

echo "=== TENSORRT ==="
dpkg -l | grep -i tensorrt | head -10
echo ""

echo "=== CUDNN ==="
dpkg -l | grep -i cudnn | head -10
echo ""

echo "=== OPENCV ==="
pkg-config --modversion opencv4 2>/dev/null || echo "opencv4 not found via pkg-config"
dpkg -l | grep -i opencv | head -10
echo ""

echo "=== REALSENSE ==="
dpkg -l | grep -i realsense 2>/dev/null | head -10
rs-enumerate-devices --version 2>/dev/null || echo "rs-enumerate-devices not found"
echo ""

echo "=== CYCLONEDDS ==="
dpkg -l | grep -i cyclonedds 2>/dev/null || echo "Not installed via dpkg"
ls /usr/local/lib/libddsc* 2>/dev/null || echo "Not found in /usr/local/lib"
echo ""

echo "=== LIVOX SDK ==="
dpkg -l | grep -i livox 2>/dev/null || echo "Not installed via dpkg"
ls /usr/local/lib/liblivox* 2>/dev/null || echo "Not found in /usr/local/lib"
echo ""

echo "=== GSTREAMER ==="
gst-launch-1.0 --version 2>/dev/null || echo "gst-launch-1.0 not found"
echo ""

echo "=== GSTREAMER PLUGINS ==="
dpkg -l | grep -i gstreamer | head -20
echo ""

echo "=== ZSTD ==="
pkg-config --modversion libzstd 2>/dev/null || echo "libzstd not found via pkg-config"
dpkg -l | grep -i zstd | head -5
echo ""

echo "=== JSON LIBRARIES ==="
dpkg -l | grep -i nlohmann 2>/dev/null | head -5
ls /usr/include/nlohmann 2>/dev/null && echo "nlohmann found in /usr/include" || echo "nlohmann not in /usr/include"
echo ""

echo "=== CMAKE ==="
cmake --version 2>/dev/null | head -1
echo ""

echo "=== GCC/G++ ==="
gcc --version | head -1
g++ --version | head -1
echo ""

echo "=== PYTHON ==="
python3 --version 2>/dev/null
echo ""

echo "=== KEY SHARED LIBRARIES ==="
ldconfig -p | grep -E "(opencv|realsense|cuda|ddsc|livox|zstd)" | head -30
echo ""

echo "=== ENVIRONMENT VARIABLES ==="
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo "CUDA_HOME=$CUDA_HOME"
echo "PATH=$PATH"
echo ""

echo "========================================"
echo "END OF SPECS"
echo "========================================"
