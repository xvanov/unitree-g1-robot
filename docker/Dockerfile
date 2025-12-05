# Multi-arch Dockerfile for G1 Inspector
# Supports: Mac (Docker Desktop), Ubuntu 22.04 amd64, Ubuntu 20.04 arm64
#
# This image includes ALL dependencies pre-built:
#   - unitree_sdk2 (installed to /opt/unitree_robotics)
#   - OpenCV, curl, nlohmann_json, libharu (via apt)
#   - Google Test (built from source, required for unit tests)
#   - Python3 with numpy and Pillow (for utility scripts)
#
# Build: docker build -t g1-inspector -f docker/Dockerfile .
# Run:   docker compose -f docker/compose.yaml up -d
#
FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies for unitree_sdk2 and project
RUN apt-get update && apt-get install -y \
    # Build essentials
    build-essential \
    cmake \
    git \
    # Network tools (for robot communication setup)
    iproute2 \
    iputils-ping \
    # unitree_sdk2 dependencies
    libyaml-cpp-dev \
    libeigen3-dev \
    libboost-all-dev \
    libspdlog-dev \
    libfmt-dev \
    # OpenCV 4.x development libraries
    libopencv-dev \
    # curl and libcurl for HTTP client
    libcurl4-openssl-dev \
    curl \
    # libharu for PDF generation (output)
    libhpdf-dev \
    # Poppler for PDF parsing (input - load PDF construction plans)
    libpoppler-cpp-dev \
    # nlohmann/json header-only library
    nlohmann-json3-dev \
    # Google Test for unit testing
    libgtest-dev \
    # Python for utility scripts (test map generation, etc.)
    python3 \
    python3-numpy \
    python3-pil \
    && rm -rf /var/lib/apt/lists/*

# Build and install GTest
# Note: Ubuntu's libgtest-dev only provides source files, not compiled libraries.
# We must build it ourselves for CMake's find_package(GTest) to work.
RUN cd /usr/src/gtest \
    && cmake . \
    && make \
    && cp lib/*.a /usr/lib/

# Clone and build unitree_sdk2
# Installed to /opt/unitree_robotics for CMake to find
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 \
    && cd /tmp/unitree_sdk2 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/unitree_sdk2

# Set environment for CycloneDDS
ENV CYCLONEDDS_URI=file:///workspace/config/cyclonedds.xml

# Set working directory
WORKDIR /workspace

# Default command - keep container running for interactive use
CMD ["/bin/bash"]
