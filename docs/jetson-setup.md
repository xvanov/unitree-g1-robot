# Jetson Setup Guide

This document covers setting up the G1 robot's Jetson (NVIDIA Orin NX) for native builds.

## Quick Start

```bash
# 1. SSH to robot
ssh unitree@192.168.123.164  # password: 123

# 2. Clone/sync code
cd ~
git clone <repo-url> g1_inspector  # Or use deploy-to-robot.sh

# 3. Run setup script (first time only)
cd g1_inspector
./scripts/setup-robot.sh

# 4. Build
mkdir -p build && cd build
cmake .. -DROBOT_BUILD=ON
make -j4

# 5. Run (IMPORTANT: run from project root, not from build/)
cd ~/g1_inspector  # Must be in project root for model paths to resolve
./build/g1_inspector --greeter --dry-run
```

## Jetson Environment

| Component | Version | Notes |
|-----------|---------|-------|
| OS | Ubuntu 20.04 (L4T) | JetPack 5.1.1 |
| Arch | aarch64 (ARMv8) | |
| CUDA | 11.4.315 | GPU acceleration available |
| OpenCV | 4.2.0 | From apt |
| CycloneDDS | 0.10.2 | Built from source |
| nlohmann-json | 3.7.3 | Avoid 3.8+ features |

## Environment Variables

Add to `~/.bashrc`:

```bash
source ~/g1_inspector/config/robot.env
```

Or set manually:

```bash
# CycloneDDS (required for DDS communication)
export CYCLONEDDS_URI=file:///home/unitree/g1_inspector/config/cyclonedds-robot.xml

# Library paths
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:/usr/local/lib:$LD_LIBRARY_PATH

# CUDA binaries
export PATH=/usr/local/cuda-11.4/bin:$PATH
```

## Model Files

The following models must be present (setup-robot.sh downloads them):

| Model | Size | Location |
|-------|------|----------|
| deploy.prototxt | 28 KB | models/deploy.prototxt |
| res10_300x300_ssd.caffemodel | 10.7 MB | models/res10_300x300_ssd_iter_140000.caffemodel |
| face_recognition_sface.onnx | 37 MB | models/face_recognition_sface_2021dec.onnx |

Search order for models:
1. `./models/` (project directory)
2. `~/.g1_inspector/models/` (user directory)
3. `/opt/g1_inspector/models/` (system-wide)

## ROBOT_BUILD CMake Option

When building on Jetson, use `-DROBOT_BUILD=ON`:

```bash
cmake .. -DROBOT_BUILD=ON
```

This enables:
- Jetson-specific optimizations
- CUDA backend for DNN (GPU acceleration)
- nlohmann-json 3.7.3 compatibility checks

The option is auto-detected on Jetson via `/etc/nv_tegra_release`.

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| `libddsc.so not found` | CycloneDDS not in path | `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH` |
| `CUDA libraries not found` | CUDA not in path | `export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH` |
| SSH connection refused | Wrong IP | Verify robot IP with `ping 192.168.123.164` |
| Face model not found | Models not downloaded | Run `./scripts/setup-robot.sh --models` |
| Face model not found (models exist) | Wrong working directory | Run from project root: `cd ~/g1_inspector && ./build/g1_inspector` |
| Build template errors | nlohmann-json version | Avoid 3.8+ features (to_bson, patch_inplace) |
| OpenCV not found | Not installed | `sudo apt install libopencv-dev` |

## Network Configuration

The robot uses a direct Ethernet connection:

| Interface | Dev Machine | Robot |
|-----------|-------------|-------|
| IP Subnet | 192.168.123.x | 192.168.123.164 |
| Gateway | N/A | N/A (direct link) |
| DDS Config | cyclonedds.xml | cyclonedds-robot.xml |

## Verification

Run the verification script to check all dependencies:

```bash
./scripts/verify-robot-setup.sh
```

Expected output:
```
[1/8] nlohmann-json version:
  [PASS] Version 3.7.3
[2/8] OpenCV version:
  [PASS] OpenCV 4.2.0
...
```

## See Also

- [docs/jetson-specs.md](jetson-specs.md) - Full hardware/software inventory
- [docs/architecture.md](architecture.md) - Soul-Brain-Body deployment philosophy
