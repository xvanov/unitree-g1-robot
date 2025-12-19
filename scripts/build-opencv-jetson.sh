#!/bin/bash
# build-opencv-jetson.sh - Build OpenCV 4.8 with FaceRecognizerSF on Jetson
# This enables face recognition (requires opencv_contrib face module)
#
# Usage: ./scripts/build-opencv-jetson.sh
# Time: ~1-2 hours on Jetson Orin

set -e

OPENCV_VERSION="4.8.0"
INSTALL_PREFIX="/usr/local"
BUILD_DIR="$HOME/opencv_build"

echo "============================================================"
echo "  OpenCV $OPENCV_VERSION Build for Jetson"
echo "  This will take 1-2 hours to compile"
echo "============================================================"

# Check if already installed
if pkg-config --modversion opencv4 2>/dev/null | grep -q "4\.[5-9]\|4\.[1-9][0-9]"; then
    echo "OpenCV $(pkg-config --modversion opencv4) already installed with FaceRecognizerSF support"
    exit 0
fi

# Install dependencies
echo "[1/6] Installing build dependencies..."
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git pkg-config \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libgtk-3-dev libatlas-base-dev gfortran \
    python3-dev python3-numpy \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# Create build directory
echo "[2/6] Setting up build directory..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Download OpenCV and contrib
echo "[3/6] Downloading OpenCV $OPENCV_VERSION..."
if [ ! -d "opencv" ]; then
    git clone --depth 1 --branch $OPENCV_VERSION https://github.com/opencv/opencv.git
fi
if [ ! -d "opencv_contrib" ]; then
    git clone --depth 1 --branch $OPENCV_VERSION https://github.com/opencv/opencv_contrib.git
fi

# Configure
echo "[4/6] Configuring build (CUDA + contrib modules)..."
cd opencv
mkdir -p build && cd build

cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
    -D OPENCV_EXTRA_MODULES_PATH=$BUILD_DIR/opencv_contrib/modules \
    -D WITH_CUDA=ON \
    -D CUDA_ARCH_BIN="7.2,8.7" \
    -D CUDA_ARCH_PTX="" \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_FAST_MATH=ON \
    -D CUDA_FAST_MATH=ON \
    -D WITH_CUBLAS=ON \
    -D WITH_GSTREAMER=ON \
    -D WITH_V4L=ON \
    -D WITH_OPENGL=ON \
    -D BUILD_opencv_python3=ON \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    ..

# Build (use fewer jobs to avoid OOM on Jetson)
echo "[5/6] Building OpenCV (this takes 1-2 hours)..."
make -j$(nproc --ignore=2)

# Install
echo "[6/6] Installing..."
sudo make install
sudo ldconfig

# Verify
echo ""
echo "============================================================"
echo "  Build Complete!"
echo "============================================================"
echo "OpenCV version: $(pkg-config --modversion opencv4)"
echo ""
echo "To verify FaceRecognizerSF:"
echo "  python3 -c \"import cv2; print(cv2.FaceRecognizerSF)\""
echo ""
echo "Rebuild g1_inspector to use new OpenCV:"
echo "  cd ~/g1_inspector/build && rm -rf * && cmake .. -DROBOT_BUILD=ON && make -j4"
