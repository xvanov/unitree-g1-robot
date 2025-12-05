# Architecture Document - Lightweight Inspection Stack

**Project:** unitree-g1-robot
**Author:** BMAD
**Date:** 2025-12-04
**Version:** 2.0 (Complete rewrite)

---

## 1. Executive Summary

This document describes the system architecture for an autonomous construction site inspection robot built on the Unitree G1 platform.

### Key Architectural Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Language | C++17 | Performance, single binary, Unitree SDK native |
| Robot Communication | unitree_sdk2 (DDS) | SDK handles complexity, proven |
| Middleware | None | SDK is sufficient, no ROS2 needed |
| Simulation | Component-specific C++ sims | Fast, simple, no physics engine |
| VLM Integration | HTTP API calls | No ML infrastructure needed |
| Deployment | Docker container | Portable across machines |

### What We Don't Use

| Not Used | Reason |
|----------|--------|
| ROS2 | Middleware overhead, dependency hell, clock sync issues |
| MuJoCo | Physics useless when locomotion only works on real robot |
| Python | Dependency management pain, slower, GIL |
| Nav2 | Too heavy, brings all of ROS2 |

---

## 2. Soul-Brain-Body Architecture

### Philosophy

The system separates into three conceptual layers:

```
┌─────────────────────────────────────────────────────────────────┐
│                         SOUL                                     │
│                   (Application Logic)                            │
│                                                                  │
│   Portable, reusable logic that defines robot behavior:         │
│   • Navigation algorithms (A*, costmap)                         │
│   • SLAM (grid mapping)                                          │
│   • State machine                                                │
│   • Defect detection logic                                       │
│   • Report generation                                            │
│                                                                  │
│   Packaged as Docker container. Runs anywhere.                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ runs on
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         BRAIN                                    │
│                    (Compute Platform)                            │
│                                                                  │
│   The hardware that hosts the soul:                             │
│   • Your Mac (development)                                       │
│   • Linux server (deployment)                                    │
│   • Jetson (optional, on-robot)                                 │
│                                                                  │
│   Provides CPU, memory, network. Interchangeable.               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ network (WiFi/Ethernet)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         BODY                                     │
│                      (Unitree G1)                                │
│                                                                  │
│   Physical robot:                                                │
│   • Sensors (LiDAR, Camera, IMU)                                │
│   • Actuators (legs, locomotion)                                │
│   • Unitree firmware + SDK server                               │
│                                                                  │
│   Receives commands, sends sensor data. Dumb terminal.          │
└─────────────────────────────────────────────────────────────────┘
```

### Benefits

| Benefit | Explanation |
|---------|-------------|
| Develop without robot | Soul runs in simulation on Brain |
| Deploy anywhere | Same Docker container on any machine |
| Easy testing | Test soul logic without physical hardware |
| Hardware independence | Swap Brain or Body without changing Soul |

---

## 3. Communication Architecture

### Robot Communication: Unitree SDK

The Unitree G1 uses DDS (CycloneDDS) for internal communication. The `unitree_sdk2` C++ library handles all DDS complexity.

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROBOT COMMUNICATION                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Your C++ Code              unitree_sdk2              Robot     │
│   ──────────────              ───────────              ─────     │
│                                                                  │
│   Locomotion:                                                    │
│   LocoClient.SetVelocity() ──▶ [SDK: DDS] ──▶ Robot moves       │
│   LocoClient.StandUp()     ──▶ [SDK: DDS] ──▶ Robot stands      │
│                                                                  │
│   Sensors:                                                       │
│   ChannelSubscriber ◀──────── [SDK: DDS] ◀── LiDAR data         │
│   ChannelSubscriber ◀──────── [SDK: DDS] ◀── IMU data           │
│   ChannelSubscriber ◀──────── [SDK: DDS] ◀── Joint states       │
│                                                                  │
│   Camera:                                                        │
│   Direct via RealSense SDK (USB) or SDK channel                 │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

**Key point:** We never touch DDS directly. The SDK is the abstraction layer.

### SDK Usage Examples

```cpp
#include "unitree/robot/b2/loco/loco_client.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

// Locomotion control
unitree::robot::b2::LocoClient loco_client;
loco_client.Init();
loco_client.SetVelocity(0.5, 0.0, 0.0, 1.0);  // Forward 0.5 m/s for 1s

// Sensor subscription
unitree::robot::ChannelSubscriber<unitree_go::msg::LowState> state_sub;
state_sub.InitChannel("rt/lowstate", [](const auto& msg) {
    float battery = msg.power_v();
    // IMU, joint states, etc. available in msg
});

// LiDAR subscription
unitree::robot::ChannelSubscriber<sensor_msgs::msg::PointCloud2> lidar_sub;
lidar_sub.InitChannel("rt/utlidar/cloud", [](const auto& msg) {
    // Process point cloud
});
```

### Network Configuration

| Component | Address | Protocol |
|-----------|---------|----------|
| G1 Robot (Ethernet) | 192.168.123.164 | DDS via SDK |
| LiDAR | 192.168.123.120 | DDS via SDK |
| Development Machine | 192.168.123.x | Same network |

**CycloneDDS Domain:** Must match robot's domain ID (default: 0)

### VLM API Communication

Defect detection uses HTTP calls to Claude/GPT-4V APIs:

```cpp
// Simple HTTP POST to Anthropic API
std::string analyze_image(const cv::Mat& image) {
    std::string base64_img = base64_encode(image);

    json request = {
        {"model", "claude-sonnet-4-20250514"},
        {"max_tokens", 4096},
        {"messages", {{
            {"role", "user"},
            {"content", {
                {{"type", "image"}, {"source", {{"type", "base64"}, {"data", base64_img}}}},
                {{"type", "text"}, {"text", "Analyze for construction defects..."}}
            }}
        }}}
    };

    return http_post("https://api.anthropic.com/v1/messages", request.dump());
}
```

**No ML server needed.** The foundation model runs on Anthropic's infrastructure.

---

## 4. System Components

### 4.1 Single Binary Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    g1_inspector (single C++ binary)              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  main.cpp                                                        │
│  ├── Parse CLI arguments                                        │
│  ├── Initialize components                                       │
│  ├── Run main loop                                               │
│  └── Cleanup on exit                                             │
│                                                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │ Inspector   │  │ Navigation  │  │    SLAM     │              │
│  │    App      │  │             │  │             │              │
│  │             │  │ • A* planner│  │ • Grid map  │              │
│  │ • State mgmt│  │ • Costmap   │  │ • Localizer │              │
│  │ • CLI       │  │ • Path exec │  │             │              │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
│         │                │                │                      │
│         └────────────────┼────────────────┘                      │
│                          │                                       │
│                   Internal Message Bus                           │
│                   (function calls, no IPC)                       │
│                          │                                       │
│         ┌────────────────┼────────────────┐                      │
│         │                │                │                      │
│  ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐              │
│  │  Sensors    │  │ Locomotion  │  │   Safety    │              │
│  │             │  │             │  │             │              │
│  │ • SDK subs  │  │ • LocoClient│  │ • E-stop    │              │
│  │ • LiDAR     │  │ • Velocity  │  │ • Battery   │              │
│  │ • Camera    │  │             │  │ • Collision │              │
│  └─────────────┘  └─────────────┘  └─────────────┘              │
│                                                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │  Detection  │  │   Report    │  │   Capture   │              │
│  │             │  │  Generator  │  │             │              │
│  │ • VLM HTTP  │  │ • PDF       │  │ • 1fps img  │              │
│  │ • Analysis  │  │ • Overlay   │  │ • Pose tag  │              │
│  └─────────────┘  └─────────────┘  └─────────────┘              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 Navigation

**A* path planning + occupancy grid costmap. No Nav2.**

```cpp
// navigation/Planner.h
class Planner {
public:
    std::vector<Point2D> planPath(Point2D start, Point2D goal);
    bool isPathValid(const std::vector<Point2D>& path);
private:
    float heuristic(Point2D a, Point2D b);  // Euclidean
    const Costmap& costmap_;
};

// navigation/Costmap.h
class Costmap {
public:
    void updateFromScan(const LidarScan& scan, Pose2D robot_pose);
    uint8_t getCost(float x, float y) const;
    void inflate(float robot_radius);
    void savePng(const std::string& path) const;
private:
    std::vector<uint8_t> grid_;
    float resolution_;  // meters per cell
};

// navigation/PathFollower.h
class PathFollower {
public:
    void setPath(const std::vector<Point2D>& path);
    Velocity getVelocityCommand(Pose2D current_pose);
    bool isComplete() const;
};
```

### 4.3 SLAM

**Simple occupancy grid mapping. No slam_toolbox.**

```cpp
// slam/GridMapper.h
class GridMapper {
public:
    void update(Pose2D pose, const std::vector<float>& ranges);
    const OccupancyGrid& getMap() const;
    void saveMap(const std::string& path) const;
private:
    void traceRay(Point2D from, Point2D to);
    void updateCell(int x, int y, float log_odds_delta);
    OccupancyGrid map_;
};

// slam/Localizer.h (optional, can use odometry only for MVP)
class Localizer {
public:
    Pose2D getPose() const;
    void update(const LidarScan& scan, const OccupancyGrid& map);
private:
    // Simple scan matching or particle filter
};
```

### 4.4 Sensor Interface

**Direct SDK subscription, no ROS2 topics.**

```cpp
// sensors/SensorManager.h
class SensorManager {
public:
    void init(const std::string& network_interface);

    // Callbacks for new data
    void setLidarCallback(std::function<void(const LidarScan&)> cb);
    void setCameraCallback(std::function<void(const CameraFrame&)> cb);
    void setImuCallback(std::function<void(const ImuData&)> cb);

    // Blocking getters (latest data)
    LidarScan getLatestLidar();
    CameraFrame getLatestCamera();
    ImuData getLatestImu();

private:
    // SDK subscribers
    unitree::robot::ChannelSubscriber<PointCloud2> lidar_sub_;
    unitree::robot::ChannelSubscriber<LowState> state_sub_;
    // Thread-safe buffers
    std::mutex lidar_mutex_;
    LidarScan latest_lidar_;
};
```

### 4.5 Locomotion

**Thin wrapper around Unitree SDK.**

```cpp
// locomotion/LocoController.h
class LocoController {
public:
    bool init();

    // High-level commands
    void setVelocity(float vx, float vy, float omega);
    void stop();
    void standUp();
    void sitDown();
    void emergencyStop();

    // Status
    bool isReady() const;
    RobotState getState() const;

private:
    unitree::robot::b2::LocoClient loco_client_;
};
```

### 4.6 Safety

```cpp
// safety/SafetyMonitor.h
class SafetyMonitor {
public:
    void init(LocoController* loco, SensorManager* sensors);
    void update();  // Called every frame

    // Triggers
    void triggerEstop();
    void clearEstop();

    // Status
    SafetyState getState() const;
    float getBatteryPercent() const;
    bool isCollisionImminent() const;

private:
    void checkBattery();
    void checkCollision();
    void checkLocalization();

    LocoController* loco_;
    SensorManager* sensors_;
    SafetyState state_;
};
```

### 4.7 Defect Detection

**HTTP calls to VLM API.**

```cpp
// detection/VlmClient.h
class VlmClient {
public:
    VlmClient(const std::string& api_key);

    std::vector<Defect> analyzeImage(
        const cv::Mat& image,
        const std::string& plan_context,
        const Pose2D& pose
    );

private:
    std::string buildPrompt(const std::string& plan_context);
    std::vector<Defect> parseResponse(const std::string& json);
    std::string httpPost(const std::string& body);

    std::string api_key_;
    std::string api_url_ = "https://api.anthropic.com/v1/messages";
};

struct Defect {
    std::string id;
    std::string type;        // "location_error" | "quality_issue"
    std::string description;
    cv::Point2f image_loc;   // Pixel coords
    Point2D plan_loc;        // World coords
    float confidence;
};
```

### 4.8 State Machine

```cpp
// app/StateMachine.h
enum class InspectionState {
    IDLE,
    CALIBRATING,
    INSPECTING,
    PAUSED,
    BLOCKED,
    WAITING_OPERATOR,
    COMPLETE,
    EMERGENCY_STOP,
    RETURNING_HOME
};

class StateMachine {
public:
    InspectionState getState() const;

    // Commands
    bool startInspection(const std::string& plan_path);
    bool pause();
    bool resume();
    bool stop();
    bool emergencyStop();

    // Update
    void update(float dt);

    // Status
    float getCompletionPercent() const;
    std::string getStatusString() const;

private:
    void transitionTo(InspectionState new_state);
    InspectionState state_ = InspectionState::IDLE;
};
```

---

## 5. Component Simulations

### Philosophy

**No mocks. Real simulations that test real logic.**

Each component has a standalone simulation that:
- Runs without the robot
- Produces measurable outputs (PNG, JSON, logs)
- Enables fast iteration for agentic development

### 5.1 Navigation Simulation (NavSim)

Tests: Path planning, obstacle avoidance, costmap updates

```cpp
// sim/nav_sim/NavSim.h
class NavSim {
public:
    NavSim(const std::string& map_png);

    // Robot as 2D point
    void setRobotPose(Pose2D pose);
    void applyVelocity(float vx, float vy, float omega, float dt);

    // Simulate sensors
    std::vector<float> simulateLidar(int num_rays = 360);

    // Checks
    bool checkCollision() const;
    bool checkGoalReached(Point2D goal, float tolerance = 0.5f) const;

    // Outputs
    void saveSnapshot(const std::string& path);  // PNG visualization
    void saveMetrics(const std::string& path);   // JSON metrics

private:
    cv::Mat map_;        // Grayscale: 0=obstacle, 255=free
    Pose2D robot_pose_;
    std::vector<Point2D> path_;

    float raycast(Point2D origin, float angle);
};
```

**Usage:**
```bash
./nav_sim --map test_maps/office.png --start 1,1,0 --goal 8,5 --output results/

# Outputs:
# results/trajectory.png  - Robot path visualization
# results/metrics.json    - {"goal_reached": true, "path_length": 12.3}
# results/nav.log         - Detailed execution log
```

### 5.2 SLAM Simulation (SlamSim)

Tests: Map building, localization

```cpp
// sim/slam_sim/SlamSim.h
class SlamSim {
public:
    SlamSim(const std::string& ground_truth_map);

    // Run simulation
    void step(Pose2D robot_pose, float dt);

    // Outputs
    void saveBuiltMap(const std::string& path);
    void saveComparison(const std::string& path);  // Side-by-side with ground truth
    float computeAccuracy() const;
};
```

### 5.3 Detection Simulation (DetectionSim)

Tests: VLM prompts, defect localization accuracy

```cpp
// sim/detection_sim/DetectionSim.h
class DetectionSim {
public:
    DetectionSim(const std::string& test_images_dir);

    // Run detection on all test images
    void run(VlmClient& client);

    // Evaluate against ground truth
    DetectionMetrics evaluate(const std::string& ground_truth_json);

    // Outputs
    void saveResults(const std::string& path);
};
```

### No Locomotion Simulation

**Locomotion only works on the real robot.** The Unitree SDK's high-level commands (SetVelocity, StandUp) work only with real hardware. We don't simulate walking physics.

In NavSim, we abstract the robot as a 2D point that moves according to velocity commands. This tests navigation logic without needing physics.

---

## 6. Agentic Development System

### Overview

Claude Code autonomously implements stories using component simulations for verification.

```
┌─────────────────────────────────────────────────────────────────┐
│                    AGENTIC DEVELOPMENT LOOP                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────┐    ┌────────┐    ┌────────┐    ┌────────┐          │
│  │ Story  │───▶│ Claude │───▶│ Code   │───▶│ Build  │          │
│  │ Input  │    │ Code   │    │ Output │    │ Check  │          │
│  └────────┘    └────────┘    └────────┘    └───┬────┘          │
│                     ▲                          │                │
│                     │                          ▼                │
│               ┌─────┴─────┐              ┌─────────┐            │
│               │ Feedback  │◀─────────────│ Run Sim │            │
│               │ Analysis  │              └────┬────┘            │
│               └───────────┘                   │                 │
│                     │              ┌──────────┴──────────┐      │
│                     │              ▼                     ▼      │
│                     │        ┌──────────┐         ┌──────────┐  │
│                     │        │   PASS   │         │   FAIL   │  │
│                     │        │ Deliver  │         │  Retry   │  │
│                     │        └────┬─────┘         └────┬─────┘  │
│                     │             │                    │        │
│                     │             ▼                    └───────▶│
│                     │       ┌──────────┐                        │
│                     │       │  Human   │                        │
│                     │       │  Verify  │                        │
│                     │       │ on Robot │                        │
│                     │       └──────────┘                        │
│                     │                                           │
└─────────────────────┴───────────────────────────────────────────┘
```

### Feedback Levels

| Level | When | Speed | What's Checked |
|-------|------|-------|----------------|
| 1: Real-time | During sim | Seconds | Log stream for errors |
| 2: Per-sim | After sim run | Minutes | Outputs: PNG, JSON, logs |
| 3: Per-story | Story complete | Hours | All acceptance criteria |
| 4: Hardware | Human verify | Days | Real robot behavior |

### Verification Specification

Each story has a verification spec:

```yaml
# verification/story_navigation.yaml

story: "Navigation Core"

build:
  command: "cmake --build build"
  expect_exit: 0

unit_tests:
  command: "./build/test_navigation"
  expect_exit: 0

simulation:
  binary: "./build/nav_sim"
  args: "--map test_maps/office.png --goal 8,5 --output outputs/"
  timeout: 60s

  checks:
    - type: json
      file: outputs/metrics.json
      field: goal_reached
      expect: true

    - type: json
      file: outputs/metrics.json
      field: collisions
      expect: 0

    - type: log
      file: outputs/nav.log
      not_contains: ["ERROR", "FATAL"]

    - type: png_analysis
      file: outputs/trajectory.png
      prompt: "Verify path avoids obstacles and reaches goal"
```

---

## 7. Project Structure

```
unitree-g1-robot/
├── src/
│   ├── main.cpp                    # Entry point
│   ├── app/
│   │   ├── InspectorApp.h/cpp      # Main application
│   │   └── StateMachine.h/cpp      # State management
│   ├── navigation/
│   │   ├── Planner.h/cpp           # A* path planning
│   │   ├── Costmap.h/cpp           # Occupancy grid
│   │   └── PathFollower.h/cpp      # Velocity from path
│   ├── slam/
│   │   ├── GridMapper.h/cpp        # Occupancy grid SLAM
│   │   └── Localizer.h/cpp         # Pose estimation
│   ├── sensors/
│   │   └── SensorManager.h/cpp     # SDK subscriptions
│   ├── locomotion/
│   │   └── LocoController.h/cpp    # SDK locomotion wrapper
│   ├── safety/
│   │   └── SafetyMonitor.h/cpp     # E-stop, battery, collision
│   ├── detection/
│   │   └── VlmClient.h/cpp         # HTTP to Claude API
│   ├── report/
│   │   └── ReportGenerator.h/cpp   # PDF output
│   ├── capture/
│   │   └── ImageCapture.h/cpp      # 1fps image capture
│   └── util/
│       ├── Types.h                 # Common types
│       ├── Json.h                  # JSON helpers
│       └── Http.h                  # HTTP client
│
├── sim/                            # Component simulations
│   ├── nav_sim/
│   │   ├── NavSim.h/cpp
│   │   └── main.cpp
│   ├── slam_sim/
│   │   ├── SlamSim.h/cpp
│   │   └── main.cpp
│   └── detection_sim/
│       ├── DetectionSim.h/cpp
│       └── main.cpp
│
├── test/                           # Unit tests
│   ├── test_navigation.cpp
│   ├── test_slam.cpp
│   └── test_detection.cpp
│
├── scripts/
│   ├── g1_inspect                  # CLI wrapper
│   └── run_verification.sh         # Agentic verification
│
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
│
├── config/
│   ├── robot.yaml                  # Robot parameters
│   └── navigation.yaml             # Nav parameters
│
├── external/                       # External dependencies
│   └── unitree_sdk2/               # Cloned SDK
│
├── CMakeLists.txt
├── README.md
└── docs/
    ├── architecture.md             # This document
    ├── prd.md
    └── epics.md
```

---

## 8. Dependencies

### Required

| Dependency | Version | Purpose |
|------------|---------|---------|
| unitree_sdk2 | latest | Robot communication |
| OpenCV | 4.x | Image processing |
| curl | 7.x | HTTP client for VLM |
| nlohmann/json | 3.x | JSON parsing (header-only) |

### Optional

| Dependency | Purpose |
|------------|---------|
| libharu | PDF generation |
| ImGui | Debug visualization |

### Build Requirements

- CMake 3.16+
- C++17 compiler (GCC 9+, Clang 10+)
- CycloneDDS (comes with unitree_sdk2)

### Comparison

| Old Stack | New Stack |
|-----------|-----------|
| ROS2 Humble (50+ packages) | unitree_sdk2 |
| Nav2 (12+ packages) | Custom A* (~500 lines) |
| slam_toolbox | Custom GridMapper (~300 lines) |
| Python + conda + pip | C++ only |
| MuJoCo | NavSim (2D grid) |
| ~100 dependencies | ~5 dependencies |

---

## 9. Build & Deployment

### Build

```bash
# Clone and setup
git clone <repo>
cd unitree-g1-robot
./scripts/setup.sh  # Clones unitree_sdk2 to external/

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run tests
./test_navigation
./test_slam

# Run simulation
./nav_sim --map ../test_maps/office.png --goal 8,5
```

### Docker

```dockerfile
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    build-essential cmake \
    libopencv-dev libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

RUN mkdir build && cd build && cmake .. && make -j

ENTRYPOINT ["/app/build/g1_inspector"]
```

### CLI Usage

```bash
# Connect to robot
./g1_inspector --robot 192.168.123.164

# Interactive commands
> status                    # Show state
> start --plan floor.png    # Start inspection
> pause                     # Pause
> resume                    # Resume
> stop                      # Stop
> estop                     # Emergency stop
> report                    # Generate report
```

---

## 10. Story Mapping

| Story | Implementation |
|-------|---------------|
| 1: Project Setup | CMake project, dependencies |
| 2: Navigation Core | Planner, Costmap, PathFollower |
| 3: NavSim | 2D simulation |
| 4: SLAM Core | GridMapper |
| 5: Sensor Interface | SensorManager with SDK |
| 6: Locomotion | LocoController |
| 7: Hardware Hello | Connect to real robot |
| 8: Safety | SafetyMonitor |
| 9: State Machine + CLI | StateMachine, CLI |
| 10: Visual Capture | ImageCapture |
| 11: VLM Detection | VlmClient |
| 12: Report Generation | ReportGenerator |
| 13: Integration Testing | End-to-end with sims |
| 14: Docker Deployment | Containerization |

---

## 11. Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| A* not as good as Nav2 | Sufficient for flat floors; can optimize later |
| GridMapper not as good as slam_toolbox | Sufficient for MVP; indoor environments |
| No physics simulation | Locomotion tested on real robot only |
| VLM API latency | Async calls, batch processing |
| SDK changes | Pin SDK version, test on updates |

---

## 12. Success Criteria

### Architecture Validated When:

- [ ] Single binary builds and runs
- [ ] NavSim produces valid paths
- [ ] SlamSim builds reasonable maps
- [ ] VLM client returns defect JSON
- [ ] Real robot responds to commands
- [ ] Full inspection cycle in simulation
- [ ] Docker container runs on Mac

### NFR Compliance:

| Requirement | Target | How Verified |
|-------------|--------|--------------|
| Obstacle response | <500ms | NavSim timing |
| Localization update | 10Hz | SlamSim metrics |
| E-stop response | <500ms | Hardware test |
| Route completion | ≥95% | NavSim scenarios |
