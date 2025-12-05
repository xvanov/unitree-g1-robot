# Story 1.4: Hardware Integration

**Status:** ready-for-dev

---

## Story

As the **system**,
I want **to connect to the robot hardware via SDK**,
So that **I can receive sensor data and send movement commands**.

---

## Acceptance Criteria

1. **AC1:** SensorManager connects to SDK topics and receives data
2. **AC2:** LiDAR data parsed correctly from `rt/utlidar/*` topics
3. **AC3:** IMU data parsed correctly from `rt/lowstate` topic
4. **AC4:** LocoController connects to robot and sends velocity commands
5. **AC5:** Emergency stop works (robot halts within 500ms)
6. **AC6:** Velocity limits enforced (safety bounds on commands)
7. **AC7:** Robot physically responds to commands (hello world test)
8. **AC8:** RealLocomotion implements ILocomotion interface
9. **AC9:** RealSensorSource implements ISensorSource interface

---

## Tasks / Subtasks

- [ ] **Task 1: Implement SensorManager Class** (AC: 1, 2, 3)
  - [ ] 1.1 Create `src/sensors/SensorManager.h` - sensor hub class
    - Constructor: `SensorManager()`
    - `init(network_interface)` - Initialize SDK channel subscribers
    - `setLidarCallback(std::function<void(const LidarScan&)>)` - register callback
    - `setImuCallback(std::function<void(const ImuData&)>)` - register callback
    - `getLatestLidar()` - blocking getter for latest LiDAR scan
    - `getLatestImu()` - blocking getter for latest IMU data
    - `getBatteryPercent()` - return battery level from lowstate
    - `isConnected()` - return connection status
  - [ ] 1.2 Create `src/sensors/SensorManager.cpp` - implementation
    - Subscribe to `rt/utlidar/cloud` or `rt/utlidar/voxel_map` for point cloud
    - Subscribe to `rt/lowstate` for IMU and battery data
    - Thread-safe data access with `std::mutex`
    - Parse SDK message types to internal types (LidarScan, ImuData)
    - Handle connection loss gracefully
  - [ ] 1.3 Define ImuData struct in Types.h or new header
    - `float roll, pitch, yaw` - orientation (radians)
    - `float gyro_x, gyro_y, gyro_z` - angular velocity (rad/s)
    - `float accel_x, accel_y, accel_z` - linear acceleration (m/s^2)

- [ ] **Task 2: Implement RealSensorSource** (AC: 9)
  - [ ] 2.1 Create `src/sensors/RealSensorSource.h/cpp`
    - Implements `ISensorSource` interface
    - Wraps `SensorManager` for LiDAR and pose data
    - `getLidarScan()` - return latest scan from SensorManager
    - `getPose()` - return pose (from IMU integration or odometry)
    - NOTE: For MVP, pose may come from odometry estimate, not full SLAM

- [ ] **Task 3: Implement LocoController Class** (AC: 4, 5, 6)
  - [ ] 3.1 Create `src/locomotion/LocoController.h` - locomotion wrapper
    - Constructor: `LocoController()`
    - `init()` - Initialize SDK LocoClient connection
    - `setVelocity(vx, vy, omega)` - send velocity command with safety limits
    - `stop()` - stop all motion
    - `standUp()` - command robot to stand
    - `sitDown()` - command robot to sit
    - `emergencyStop()` - immediate halt (zero torque mode)
    - `isReady()` - return connection/ready status
    - `getState()` - return current robot state (FSM state)
  - [ ] 3.2 Create `src/locomotion/LocoController.cpp` - implementation
    - Use `unitree::robot::g1::LocoClient` for G1 control
    - Initialize with service name `"sport"`
    - Apply velocity limits before sending commands:
      - Max vx: 0.5 m/s (conservative for indoor)
      - Max vy: 0.3 m/s
      - Max omega: 0.5 rad/s
    - Emergency stop: call `Damp()` or `ZeroTorque()` method
    - Monitor connection status

- [ ] **Task 4: Implement RealLocomotion** (AC: 8)
  - [ ] 4.1 Create `src/locomotion/RealLocomotion.h/cpp`
    - Implements `ILocomotion` interface
    - Wraps `LocoController` for velocity commands
    - `setVelocity(vx, vy, omega)` - delegate to LocoController
    - `stop()` - delegate to LocoController
    - `isReady()` - return LocoController ready status

- [ ] **Task 5: Create Hardware Test Commands** (AC: 7)
  - [ ] 5.1 Update `src/main.cpp` with test commands
    - `--test-sensors` - Connect and display sensor data
    - `--test-loco` - Test stand up, move forward, stop
    - `--hello-world` - Full integration test sequence
  - [ ] 5.2 Create `scripts/check_g1_network.sh`
    - Verify network connectivity to robot (192.168.123.164)
    - Ping robot and check SDK service availability

- [ ] **Task 6: CMake Integration** (AC: 1-9)
  - [ ] 6.1 Update CMakeLists.txt
    - Add SensorManager to sensors library or g1_inspector
    - Add LocoController to locomotion library
    - Link unitree_sdk2 when available
    - Conditional compilation with `HAS_UNITREE_SDK2` define

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Use ROS2, Nav2, or any ROS dependencies - this is pure C++ with unitree_sdk2
- Create mock SDK classes - use `#ifdef HAS_UNITREE_SDK2` for conditional compilation
- Block the main thread on SDK calls - use callbacks and threading
- Send velocity commands without safety limits
- Assume network is always available - handle disconnection gracefully

**MUST USE:**
- C++17 standard (`-std=c++17`)
- unitree_sdk2 for all robot communication (when available)
- Existing interfaces: `ILocomotion`, `ISensorSource` from Stories 1-2
- Existing types: `Point2D`, `Pose2D`, `Velocity`, `LidarScan` from `src/util/Types.h`
- Thread-safe data structures (`std::mutex`, `std::lock_guard`)

### Unitree SDK2 API Reference

**LocoClient for G1 (from unitree::robot::g1::LocoClient):**
```cpp
#include "unitree/robot/g1/loco/g1_loco_client.hpp"

unitree::robot::g1::LocoClient loco_client;
loco_client.Init();  // Initialize with default "sport" service
loco_client.SetTimeout(1.0f);  // Set command timeout

// State control
loco_client.StandUp();
loco_client.Sit();
loco_client.Damp();  // Emergency: passive mode

// Motion control
loco_client.Move(vx, vy, vyaw);  // Velocity command
loco_client.StopMove();

// FSM status
int fsm_id;
loco_client.GetFsmId(fsm_id);
```

**Channel Subscriber for Sensors:**
```cpp
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree_go/msg/low_state.hpp"

// LowState subscription (IMU, battery, joint states)
unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_> state_sub;
state_sub.InitChannel("rt/lowstate");
state_sub.SetDataCallback([](const unitree_go::msg::dds_::LowState_& msg) {
    // IMU data from msg.imu_state()
    // Battery from msg.bms_state().soc()
});
```

### DDS Topic Reference

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `rt/lowstate` | `LowState_` | IMU, battery, joint states |
| `rt/lowcmd` | `LowCmd_` | Low-level motor commands |
| `rt/sportmodestate` | `SportModeState_` | FSM status |
| `rt/utlidar/voxel_map` | `PointCloud2_` | LiDAR point cloud |
| `rt/utlidar/cloud` | `PointCloud2_` | LiDAR raw cloud (alternative) |

### Network Configuration

| Device | IP Address | Purpose |
|--------|------------|---------|
| Development Computer (Orin) | 192.168.123.164 | SSH: unitree/123 |
| Locomotion Computer | 192.168.123.161 | Internal control |
| Livox LiDAR | 192.168.123.120 | Point cloud |
| Your Development Machine | 192.168.123.x | Must be on same subnet |

**CycloneDDS Domain:** Default domain ID is 0. Must match robot configuration.

**No DHCP:** Configure static IP on your development machine:
```bash
# Example for Linux
sudo ip addr add 192.168.123.100/24 dev eth0
```

### Safety Limits

**CRITICAL: All velocity commands must be bounded:**
```cpp
// src/locomotion/LocoController.cpp
const float MAX_VX = 0.5f;     // m/s forward (conservative indoor)
const float MAX_VY = 0.3f;     // m/s lateral
const float MAX_OMEGA = 0.5f;  // rad/s rotation

void LocoController::setVelocity(float vx, float vy, float omega) {
    // Clamp to safe limits
    vx = std::clamp(vx, -MAX_VX, MAX_VX);
    vy = std::clamp(vy, -MAX_VY, MAX_VY);
    omega = std::clamp(omega, -MAX_OMEGA, MAX_OMEGA);

    loco_client_.Move(vx, vy, omega);
}
```

### Safety Termination Conditions

The SDK monitors these conditions - implement similar checks:
- `bad_orientation` - tilt > 1.0 rad
- `joint_vel_out_of_limit` - joint velocity > 10.0 rad/s
- `ang_vel_out_of_limit` - angular velocity > 6.0 rad/s
- `motor_overheat` - winding > 120C or casing > 85C
- `low_battery` - SOC < 20%
- `lost_connection` - no response > 1000ms

### ImuData Struct Definition

Add to `src/util/Types.h`:
```cpp
struct ImuData {
    // Orientation (radians)
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // Angular velocity (rad/s)
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    // Linear acceleration (m/s^2)
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
};
```

### Thread Safety Pattern

```cpp
// src/sensors/SensorManager.h
class SensorManager {
public:
    LidarScan getLatestLidar() {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        return latest_lidar_;
    }

private:
    void onLidarData(const PointCloud2& msg) {
        LidarScan scan = parsePointCloud(msg);
        {
            std::lock_guard<std::mutex> lock(lidar_mutex_);
            latest_lidar_ = scan;
        }
        if (lidar_callback_) {
            lidar_callback_(scan);
        }
    }

    std::mutex lidar_mutex_;
    LidarScan latest_lidar_;
    std::function<void(const LidarScan&)> lidar_callback_;
};
```

### LiDAR to LidarScan Conversion

```cpp
// Convert PointCloud2 to our LidarScan format
LidarScan parsePointCloud(const sensor_msgs::msg::dds_::PointCloud2_& cloud) {
    LidarScan scan;
    scan.angle_min = 0.0f;
    scan.angle_max = 2.0f * M_PI;

    // Extract ranges from point cloud
    // PointCloud2 contains x,y,z for each point
    // Convert to ranges: range = sqrt(x^2 + y^2)
    // Bin into angular sectors for LidarScan format

    const int num_rays = 360;
    scan.ranges.resize(num_rays, 10.0f);  // Default max range

    // Process each point and bin by angle
    // angle = atan2(y, x)
    // index = (angle - angle_min) / (angle_max - angle_min) * num_rays

    return scan;
}
```

### Conditional Compilation Pattern

```cpp
// src/locomotion/LocoController.cpp
#include "locomotion/LocoController.h"

#ifdef HAS_UNITREE_SDK2
#include "unitree/robot/g1/loco/g1_loco_client.hpp"
#endif

bool LocoController::init() {
#ifdef HAS_UNITREE_SDK2
    try {
        loco_client_.Init();
        loco_client_.SetTimeout(1.0f);
        connected_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "LocoController init failed: " << e.what() << std::endl;
        return false;
    }
#else
    std::cerr << "WARNING: unitree_sdk2 not available, LocoController disabled" << std::endl;
    return false;
#endif
}
```

### Project Structure Notes

Files and directories to create in this story:

```
src/sensors/
├── ISensorSource.h        # EXISTS - interface
├── SensorManager.h        # NEW
└── SensorManager.cpp      # NEW
└── RealSensorSource.h     # NEW
└── RealSensorSource.cpp   # NEW

src/locomotion/
├── ILocomotion.h          # EXISTS - interface
├── LocoController.h       # NEW
├── LocoController.cpp     # NEW
├── RealLocomotion.h       # NEW
└── RealLocomotion.cpp     # NEW

src/util/
└── Types.h                # MODIFY - add ImuData struct

scripts/
└── check_g1_network.sh    # NEW

src/main.cpp               # MODIFY - add test commands
CMakeLists.txt             # MODIFY - add new sources, link SDK
```

### Dependencies on Previous Stories

**Story 1-1 (Project Setup):**
- `src/util/Types.h` - Point2D, Pose2D, Velocity (add ImuData)
- `src/locomotion/ILocomotion.h` - interface to implement
- `src/sensors/ISensorSource.h` - interface with LidarScan
- CMakeLists.txt with unitree_sdk2 conditional linking

**Story 1-2 (Navigation + NavSim):**
- `src/navigation/*` - navigation algorithms (uses ILocomotion, ISensorSource)
- SimLocomotion/SimSensorSource patterns to follow

**Story 1-3 (SLAM Core):**
- `src/slam/*` - SLAM algorithms (will use RealSensorSource)

### CMake Additions Required

```cmake
# ============================================
# Hardware Integration (Story 1-4)
# ============================================

# Sensors library
add_library(sensors
    src/sensors/SensorManager.cpp
    src/sensors/RealSensorSource.cpp
)

# Locomotion library
add_library(loco_hw
    src/locomotion/LocoController.cpp
    src/locomotion/RealLocomotion.cpp
)

# Conditionally link unitree_sdk2
if(unitree_sdk2_FOUND)
    target_link_libraries(sensors unitree_sdk2)
    target_link_libraries(loco_hw unitree_sdk2)
    target_compile_definitions(sensors PRIVATE HAS_UNITREE_SDK2)
    target_compile_definitions(loco_hw PRIVATE HAS_UNITREE_SDK2)
endif()

# Update g1_inspector to include hardware components
target_link_libraries(g1_inspector
    navigation
    slam
    sensors
    loco_hw
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    ${HPDF_LIBRARY}
    nlohmann_json::nlohmann_json
)
```

---

## Verification Commands

```bash
# Verify network connectivity first
./scripts/check_g1_network.sh
# Expected: Robot pingable at 192.168.123.164

# Build (inside Docker or with SDK installed)
mkdir -p build && cd build
cmake .. && make -j

# Test sensor connection (requires real robot)
./g1_inspector --robot 192.168.123.164 --test-sensors
# Expected:
# [SENSORS] LiDAR: 3421 points received
# [SENSORS] IMU: orientation (0.01, 0.02, 0.00)
# [SENSORS] Battery: 85%

# Test locomotion (requires real robot - BE CAREFUL!)
./g1_inspector --robot 192.168.123.164 --test-loco
# Expected:
# [LOCO] Connected to robot
# [LOCO] Standing up... OK
# [LOCO] Moving forward 0.2 m/s for 2s... OK
# [LOCO] Stopping... OK

# Full hello world test (requires real robot - OPERATOR MUST BE WATCHING!)
./g1_inspector --robot 192.168.123.164 --hello-world
# Expected: Robot stands up, walks forward ~0.5m, stops
# Console shows sensor data throughout
```

### Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| Network connectivity | `ping 192.168.123.164` | Responds |
| Build with SDK | `cmake .. && make` | No errors |
| Sensor connection | `--test-sensors` | LiDAR and IMU data shown |
| Locomotion connection | `--test-loco` | Robot responds to commands |
| Emergency stop | Press E-stop during test | Robot halts < 500ms |
| Velocity limits | Send vx=2.0 | Clamped to 0.5 |

### Safety Verification Protocol

**BEFORE running --test-loco or --hello-world:**
1. Clear area around robot (2m radius minimum)
2. Operator ready at physical E-stop button
3. Robot on flat, stable surface
4. Battery > 30%
5. All cables secured and clear of legs

---

## Previous Story Intelligence

### Key Patterns from Stories 1-2 and 1-3

**Interface patterns:**
- `ILocomotion` and `ISensorSource` provide clean abstraction
- `SimLocomotion` and `SimSensorSource` show implementation pattern
- Use same pattern for `RealLocomotion` and `RealSensorSource`

**Code conventions:**
- `#pragma once` for headers
- PascalCase classes, camelCase methods
- Const refs for large structs
- OpenCV for images

**Testing approach:**
- Unit tests for logic (not hardware)
- Integration tests require real robot
- Use conditional compilation for SDK-dependent code

### What Worked in Previous Stories

- Clean separation of interfaces and implementations
- Simulation-first development (NavSim, SlamSim)
- Comprehensive dev notes in story files
- CMake conditional linking for optional dependencies

---

## Technical Reference Information

### Unitree SDK2 Resources

| Resource | URL |
|----------|-----|
| C++ SDK | https://github.com/unitreerobotics/unitree_sdk2 |
| Python SDK | https://github.com/unitreerobotics/unitree_sdk2_python |
| G1 Developer Guide | https://support.unitree.com/home/en/G1_developer |
| DeepWiki Reference | https://deepwiki.com/unitreerobotics/unitree_sdk2/3-g1-humanoid-robot |
| Weston Robot Guide | https://docs.westonrobot.com/tutorial/unitree/g1_dev_guide/ |

### G1 LocoClient API Summary

| Method | Description |
|--------|-------------|
| `Init()` | Initialize client |
| `StandUp()` | Stand from sitting |
| `Sit()` | Sit down |
| `Damp()` | Emergency passive mode |
| `Move(vx, vy, vyaw)` | Set velocity |
| `StopMove()` | Stop motion |
| `GetFsmId(int&)` | Get FSM state |
| `SetBalanceMode(int)` | Balance control |
| `HighStand()` | Stand tall |
| `LowStand()` | Stand low |

### Error Handling

```cpp
// Graceful degradation on connection loss
void SensorManager::checkConnection() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - last_data_time_;

    if (elapsed > std::chrono::seconds(2)) {
        connected_ = false;
        std::cerr << "[SENSORS] Connection lost!" << std::endl;
    }
}
```

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **SensorManager class** | Connects to SDK, receives LiDAR/IMU |
| **RealSensorSource** | Implements ISensorSource with real data |
| **LocoController class** | Sends velocity commands to robot |
| **RealLocomotion** | Implements ILocomotion with real robot |
| **Safety limits** | Commands bounded to safe values |
| **Test commands** | `--test-sensors`, `--test-loco`, `--hello-world` |
| **Network check script** | Verifies robot connectivity |

### Demo Script (Run This When Done)

```bash
# 1. Setup network (on your dev machine)
sudo ip addr add 192.168.123.100/24 dev eth0  # Linux
# Or configure via System Preferences on Mac

# 2. Verify connectivity
./scripts/check_g1_network.sh
# Should show: "Robot reachable at 192.168.123.164"

# 3. SSH to robot (optional, for debugging)
ssh unitree@192.168.123.164  # password: 123

# 4. Build the project
cd /workspace
mkdir -p build && cd build
cmake ..
make -j

# 5. Test sensors
./g1_inspector --robot 192.168.123.164 --test-sensors
# Should show LiDAR points, IMU orientation, battery level

# 6. Test locomotion (CAREFUL - robot will move!)
# Clear area, operator at E-stop ready
./g1_inspector --robot 192.168.123.164 --test-loco
# Robot should: stand, move forward briefly, stop

# 7. Full hello world
./g1_inspector --robot 192.168.123.164 --hello-world
# Robot: stands, walks forward ~0.5m, stops
# Console: continuous sensor data stream
```

**SUCCESS CRITERIA:** Story 1.4 is DONE when:
1. `--test-sensors` shows live LiDAR and IMU data from robot
2. `--test-loco` makes robot stand and move
3. Emergency stop halts robot within 500ms
4. Velocity commands are bounded to safe limits
5. Code compiles with and without `HAS_UNITREE_SDK2`

---

## References

- [Source: docs/architecture.md#4.4-sensor-interface] - SensorManager design
- [Source: docs/architecture.md#4.5-locomotion] - LocoController design
- [Source: docs/architecture.md#3-communication] - SDK usage patterns
- [Source: docs/epics.md#story-4] - Original story requirements
- [Source: docs/PRD.md#iot-embedded] - Hardware platform specs
- [External: Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) - Official C++ SDK
- [External: G1 Developer Guide](https://support.unitree.com/home/en/G1_developer) - Official docs
- [External: DeepWiki G1 Reference](https://deepwiki.com/unitreerobotics/unitree_sdk2/3-g1-humanoid-robot) - API details

---

## Dev Agent Record

### Context Reference

This story file serves as the complete implementation context.

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List

**New files to create:**
- src/sensors/SensorManager.h
- src/sensors/SensorManager.cpp
- src/sensors/RealSensorSource.h
- src/sensors/RealSensorSource.cpp
- src/locomotion/LocoController.h
- src/locomotion/LocoController.cpp
- src/locomotion/RealLocomotion.h
- src/locomotion/RealLocomotion.cpp
- scripts/check_g1_network.sh

**Files to modify:**
- src/util/Types.h (add ImuData struct)
- src/main.cpp (add test commands)
- CMakeLists.txt (add new libraries and sources)

### Change Log

- 2025-12-05: Story 1-4 created by create-story workflow - comprehensive hardware integration guide with SDK2 API details, safety requirements, and verification protocol.
