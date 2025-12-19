# Jetson System Specs (G1 Robot)

Reference for Docker image creation targeting the Unitree G1 robot's onboard Jetson.

---

## Hardware

| Component | Value |
|-----------|-------|
| **Model** | NVIDIA Orin NX Developer Kit |
| **Architecture** | aarch64 (ARMv8) |
| **CPU** | 8 cores, ARMv8 Processor rev 1 (v8l) |
| **RAM** | 16 GB |
| **Swap** | 7.5 GB |
| **Storage** | 1.9 TB NVMe (26 GB used) |

---

## OS & Kernel

| Component | Version |
|-----------|---------|
| **OS** | Ubuntu 20.04.6 LTS (Focal Fossa) |
| **Kernel** | 5.10.104-tegra |
| **L4T (Tegra)** | R35.3.1 (March 2023) |
| **JetPack** | 5.1.1 (corresponds to R35.3.1) |

---

## NVIDIA / CUDA Stack

| Component | Version |
|-----------|---------|
| **CUDA** | 11.4.315 |
| **cuDNN** | 8.6.0.166 |
| **TensorRT** | 8.5.2 |

### Key CUDA Packages
- cuda-cudart-11-4
- cuda-nvcc-11-4
- cuda-libraries-11-4
- cuda-compiler-11-4

---

## Core Libraries

| Library | Version | Install Method |
|---------|---------|----------------|
| **OpenCV** | 4.2.0 | apt (libopencv-dev) |
| **GStreamer** | 1.16.3 | apt |
| **zstd** | 1.4.4 | apt (libzstd-dev) |
| **nlohmann-json** | 3.7.3 | apt (nlohmann-json3-dev) |
| **CycloneDDS** | 0.10.2 | Built from source (/usr/local) |
| **Livox SDK 2** | latest (2023) | Built from source (/usr/local/lib) |

### CycloneDDS (0.10.2)

Built from source. Version confirmed via `/usr/local/include/dds/version.h`:
```c
#define DDS_VERSION "0.10.2"
#define DDS_VERSION_MAJOR 0
#define DDS_VERSION_MINOR 10
#define DDS_VERSION_PATCH 2
```

Libraries:
```
/usr/local/lib/libddsc.so
/usr/local/lib/libddsc.so.0
/usr/local/lib/libddscxx.so
/usr/local/lib/libddscxx.so.0
```

### Livox SDK 2

Built from source (https://github.com/Livox-SDK/Livox-SDK2).

Libraries:
```
/usr/local/lib/liblivox_lidar_sdk_shared.so
/usr/local/lib/liblivox_lidar_sdk_static.a
```

---

## RealSense

| Package | Version |
|---------|---------|
| ros-noetic-librealsense2 | 2.50.0 |
| ros-noetic-realsense2-camera | 2.3.2 |
| ros-noetic-realsense2-description | 2.3.2 |

**Note:** `rs-enumerate-devices` not available (ROS wrapper only, no standalone SDK).

---

## ROS

| Distribution | Notes |
|--------------|-------|
| **ROS Noetic** | For RealSense packages |
| **ROS 2 Foxy** | For CycloneDDS, zstd-vendor |

---

## Build Tools

| Tool | Version |
|------|---------|
| **CMake** | 3.16.3 |
| **GCC** | 9.4.0 |
| **G++** | 9.4.0 |
| **Python** | 3.13.11 (via miniconda) |

---

## GStreamer Plugins

Installed plugin sets:
- gstreamer1.0-plugins-base
- gstreamer1.0-plugins-good
- gstreamer1.0-plugins-bad
- gstreamer1.0-plugins-ugly
- gstreamer1.0-libav
- gstreamer1.0-gl
- gstreamer1.0-x

---

## Environment Variables

```bash
LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:
PATH=/home/unitree/.local/bin:/home/unitree/miniconda3/bin:/home/unitree/miniconda3/condabin:/usr/local/cuda-11.4/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
```

---

## Docker Base Image Recommendation

```dockerfile
FROM nvcr.io/nvidia/l4t-base:r35.3.1

# Or for ML workloads:
FROM nvcr.io/nvidia/l4t-pytorch:r35.3.1-pth2.0-py3
```

Key points for Dockerfile:
- Base on L4T r35.3.1 for driver compatibility
- CUDA 11.4 is pre-installed in L4T images
- Install OpenCV 4.2.0 from apt
- Build CycloneDDS and Livox SDK from source
- GStreamer available via apt

---

*Gathered: 2025-12-18*
