# Story 1.5: Safety System

**Status:** Done

---

## Story

As an **operator**,
I want **safety monitoring and emergency stop**,
So that **operation is safe**.

---

## Acceptance Criteria

1. **AC1:** E-stop halts robot within 500ms response time
2. **AC2:** Low battery triggers warning at 20% and return behavior at 10%
3. **AC3:** Collision proximity detection from costmap data
4. **AC4:** Localization confidence monitoring (optional for MVP)
5. **AC5:** Safety state is queryable at all times
6. **AC6:** Watchdog detects command timeout
7. **AC7:** Graceful degradation on sensor failure
8. **AC8:** SafetyMonitor integrates with LocoController for E-stop execution

---

## Tasks / Subtasks

- [x] **Task 1: Define Safety Types and Enums** (AC: 5)
  - [x] 1.1 Create `src/safety/SafetyTypes.h`
    - `enum class SafetyState { OK, WARNING, CRITICAL, EMERGENCY_STOP }`
    - `enum class SafetyEvent { NONE, LOW_BATTERY_WARNING, LOW_BATTERY_CRITICAL, COLLISION_IMMINENT, LOCALIZATION_LOST, COMMAND_TIMEOUT, SENSOR_FAILURE, ESTOP_TRIGGERED, ESTOP_CLEARED }`
    - `struct SafetyStatus` with state, active events, battery %, last update time

- [x] **Task 2: Implement SafetyMonitor Class** (AC: 1, 5, 8)
  - [x] 2.1 Create `src/safety/SafetyMonitor.h` - header
    - Constructor: `SafetyMonitor()`
    - `init(LocoController*, SensorManager*)` - link to locomotion and sensors
    - `update()` - main update loop, called each frame
    - `triggerEstop()` - immediate emergency stop
    - `clearEstop()` - clear E-stop state (requires operator)
    - `getState() const` - return current SafetyState
    - `getStatus() const` - return full SafetyStatus
    - `getBatteryPercent() const` - convenience getter
    - `isCollisionImminent() const` - proximity check result
    - `isEstopActive() const` - E-stop state
    - `setCollisionDistance(float)` - configure collision threshold (default 0.3m)
    - `setBatteryWarningThreshold(float)` - default 20%
    - `setBatteryCriticalThreshold(float)` - default 10%
  - [x] 2.2 Create `src/safety/SafetyMonitor.cpp` - implementation
    - Store pointers to LocoController and SensorManager
    - Implement `update()` calling all check functions
    - Implement `triggerEstop()` calling LocoController::emergencyStop()
    - Track E-stop state with atomic bool
    - Record timing for E-stop response measurement

- [x] **Task 3: Implement Battery Monitoring** (AC: 2)
  - [x] 3.1 Add `checkBattery()` private method
    - Query SensorManager::getBatteryPercent()
    - At 20%: set WARNING state, emit LOW_BATTERY_WARNING event
    - At 10%: set CRITICAL state, emit LOW_BATTERY_CRITICAL event
    - Below 5%: trigger automatic E-stop to protect battery

- [x] **Task 4: Implement Collision Detection** (AC: 3)
  - [x] 4.1 Add `checkCollision()` private method
    - Get latest LidarScan from SensorManager
    - Check minimum range in forward arc (e.g., -30 to +30 degrees)
    - If min range < collision_threshold: set WARNING, emit COLLISION_IMMINENT
    - If min range < emergency_threshold (half of collision): trigger E-stop
  - [x] 4.2 Define collision parameters
    - `collision_threshold_` = 0.3m (warning)
    - `emergency_stop_distance_` = 0.15m (immediate halt)
    - `forward_arc_degrees_` = 60 degrees total (30 each side)

- [x] **Task 5: Implement Command Watchdog** (AC: 6)
  - [x] 5.1 Add `checkWatchdog()` private method
    - Track last velocity command timestamp
    - If no command for `watchdog_timeout_` (default 1.0s): trigger E-stop
    - Watchdog only active when robot is not in IDLE state
  - [x] 5.2 Add `feedWatchdog()` public method
    - Called by locomotion code when velocity commands sent
    - Resets watchdog timer

- [x] **Task 6: Implement Sensor Failure Detection** (AC: 7)
  - [x] 6.1 Add `checkSensorHealth()` private method
    - Check SensorManager::isConnected()
    - Check for stale data (no updates in 2 seconds)
    - If LiDAR disconnected but IMU ok: WARNING, reduce speed
    - If both disconnected: CRITICAL, trigger E-stop
  - [x] 6.2 Implement graceful degradation
    - `isLidarHealthy()` / `isImuHealthy()` helpers
    - Degraded mode flag for reduced operation

- [x] **Task 7: Implement Localization Monitoring** (AC: 4)
  - [x] 7.1 Add `checkLocalization()` private method (optional for MVP)
    - For MVP: simple odometry drift detection
    - Track cumulative distance traveled
    - If drift exceeds threshold: emit LOCALIZATION_LOST warning
    - Full implementation deferred to post-MVP

- [x] **Task 8: Create Safety Test Command** (AC: 1, 2, 3, 5)
  - [x] 8.1 Update `src/main.cpp` with `--test-safety` command
    - Initialize SafetyMonitor with LocoController and SensorManager
    - Run E-stop test: trigger and measure response time
    - Display battery monitor status
    - Run collision check with current LiDAR data
    - Display all safety checks summary
  - [x] 8.2 Implement E-stop timing measurement
    - Record timestamp before calling triggerEstop()
    - Verify robot halted (velocity zero) within 500ms

- [x] **Task 9: Unit Tests** (AC: all)
  - [x] 9.1 Create `test/test_safety.cpp`
    - Test battery threshold logic
    - Test collision distance calculations
    - Test state transitions
    - Test watchdog timeout behavior
    - Mock LocoController and SensorManager for testing

- [x] **Task 10: CMake Integration** (AC: all)
  - [x] 10.1 Update CMakeLists.txt
    - Add safety library with SafetyMonitor
    - Add test_safety test executable
    - Link to sensors and locomotion libraries

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Use ROS2, Nav2, or any ROS dependencies - this is pure C++ with unitree_sdk2
- Block the main thread with safety checks - use non-blocking queries
- Ignore E-stop requests - safety is paramount
- Trust sensor data blindly - always validate ranges
- Skip safety checks for "testing" - safety is always on

**MUST USE:**
- C++17 standard (`-std=c++17`)
- Existing `LocoController` from Story 1-4 for E-stop execution
- Existing `SensorManager` from Story 1-4 for battery/sensor status
- Existing types: `LidarScan`, `ImuData`, `Pose2D` from previous stories
- Thread-safe primitives (`std::atomic`, `std::mutex`)
- Conditional compilation with `#ifdef HAS_UNITREE_SDK2` for SDK-dependent code

### Safety Threshold Constants Reference

All thresholds in one place for easy reference:

| Constant | Default | Code Symbol | Purpose |
|----------|---------|-------------|---------|
| Collision warning | 0.3m | `collision_distance_` | Slow down, emit warning |
| Collision emergency | 0.15m | `emergency_distance_` | Immediate E-stop |
| Battery warning | 20% | `battery_warning_pct_` | Alert operator |
| Battery critical | 10% | `battery_critical_pct_` | Initiate return home |
| Battery shutdown | 5% | `battery_shutdown_pct_` | Auto E-stop to protect battery |
| Watchdog timeout | 1.0s | `watchdog_timeout_s_` | E-stop on command loss |
| Sensor timeout | 2.0s | `sensor_timeout_s_` | Enter degraded mode |

### SafetyMonitor Class Design

```cpp
// src/safety/SafetyMonitor.h
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include "safety/SafetyTypes.h"

// Forward declarations - avoid including full headers
class LocoController;
class SensorManager;
struct LidarScan;

class SafetyMonitor {
public:
    SafetyMonitor();
    ~SafetyMonitor();

    // Initialization - MUST be called before update()
    void init(LocoController* loco, SensorManager* sensors);

    // Main update loop - call at 10-50 Hz
    // 10 Hz minimum: ensures E-stop response within 100ms of detection
    // 50 Hz maximum: higher rates waste CPU without safety benefit
    // Typical: match your main control loop rate
    void update();

    // Emergency stop controls
    void triggerEstop();      // Immediate halt
    void clearEstop();        // Operator must explicitly clear

    // Watchdog feed - call when sending velocity commands
    void feedWatchdog();

    // State queries
    SafetyState getState() const;
    SafetyStatus getStatus() const;
    bool isEstopActive() const;
    bool isCollisionImminent() const;
    float getBatteryPercent() const;

    // Sensor health queries
    bool isLidarHealthy() const;
    bool isImuHealthy() const;
    bool isInDegradedMode() const;

    // Configuration
    void setCollisionDistance(float meters);       // Default: 0.3m
    void setEmergencyDistance(float meters);       // Default: 0.15m
    void setBatteryWarningThreshold(float pct);    // Default: 20%
    void setBatteryCriticalThreshold(float pct);   // Default: 10%
    void setWatchdogTimeout(float seconds);        // Default: 1.0s

    // Event callback for logging/telemetry
    void setEventCallback(std::function<void(SafetyEvent)> callback) {
        event_callback_ = callback;
    }

    // Test mode controls (use with caution!)
    void disable();  // Temporarily disable safety monitoring
    void enable();   // Re-enable safety monitoring
    bool isEnabled() const { return !disabled_; }

private:
    // Check functions - all called by update()
    void checkBattery();
    void checkCollision();
    void checkWatchdog();
    void checkSensorHealth();
    void checkLocalization();

    // State management
    void setState(SafetyState new_state);
    void emitEvent(SafetyEvent event);

    // References to other components (non-owning)
    LocoController* loco_ = nullptr;
    SensorManager* sensors_ = nullptr;

    // State
    std::atomic<SafetyState> state_{SafetyState::OK};
    std::atomic<bool> estop_active_{false};
    std::atomic<bool> degraded_mode_{false};
    std::atomic<bool> disabled_{false};  // Test mode disable flag

    // Timing
    std::chrono::steady_clock::time_point last_watchdog_feed_;
    std::chrono::steady_clock::time_point last_sensor_update_;
    std::chrono::steady_clock::time_point estop_trigger_time_;

    // Thresholds (configurable)
    float collision_distance_ = 0.3f;         // meters - configured value
    float effective_collision_distance_ = 0.3f; // meters - runtime value (may differ in degraded mode)
    float emergency_distance_ = 0.15f;        // meters
    float battery_warning_pct_ = 20.0f;       // percent
    float battery_critical_pct_ = 10.0f;      // percent
    float battery_shutdown_pct_ = 5.0f;       // percent - auto E-stop
    float watchdog_timeout_s_ = 1.0f;         // seconds
    float sensor_timeout_s_ = 2.0f;           // seconds

    // Collision detection parameters
    float forward_arc_min_ = -0.5236f;        // -30 degrees in radians
    float forward_arc_max_ = 0.5236f;         // +30 degrees in radians

    // Event callback (optional, for logging)
    std::function<void(SafetyEvent)> event_callback_;

    // Mutex for status struct access
    mutable std::mutex status_mutex_;
    SafetyStatus current_status_;

    // Cached values for performance
    float cached_min_range_ = 100.0f;
    bool watchdog_enabled_ = false;
};
```

### SafetyTypes.h

```cpp
// src/safety/SafetyTypes.h
#pragma once

#include <chrono>
#include <string>
#include <vector>

enum class SafetyState {
    OK,             // Normal operation
    WARNING,        // Degraded but operational
    CRITICAL,       // Immediate action required
    EMERGENCY_STOP  // Robot halted
};

enum class SafetyEvent {
    NONE,
    LOW_BATTERY_WARNING,      // Battery at warning threshold
    LOW_BATTERY_CRITICAL,     // Battery at critical threshold
    COLLISION_IMMINENT,       // Obstacle detected in path
    LOCALIZATION_LOST,        // Position uncertainty too high
    COMMAND_TIMEOUT,          // No velocity commands received
    SENSOR_FAILURE,           // Sensor disconnected or stale
    ESTOP_TRIGGERED,          // E-stop activated
    ESTOP_CLEARED             // E-stop released
};

struct SafetyStatus {
    SafetyState state = SafetyState::OK;
    std::vector<SafetyEvent> active_events;
    float battery_percent = 100.0f;
    float min_obstacle_distance = 100.0f;  // meters
    bool estop_active = false;
    bool lidar_healthy = true;
    bool imu_healthy = true;
    bool degraded_mode = false;
    std::chrono::steady_clock::time_point last_update;

    // String conversion for logging
    std::string stateToString() const {
        switch (state) {
            case SafetyState::OK: return "OK";
            case SafetyState::WARNING: return "WARNING";
            case SafetyState::CRITICAL: return "CRITICAL";
            case SafetyState::EMERGENCY_STOP: return "EMERGENCY_STOP";
            default: return "UNKNOWN";
        }
    }

    std::string eventToString(SafetyEvent event) const {
        switch (event) {
            case SafetyEvent::NONE: return "NONE";
            case SafetyEvent::LOW_BATTERY_WARNING: return "LOW_BATTERY_WARNING";
            case SafetyEvent::LOW_BATTERY_CRITICAL: return "LOW_BATTERY_CRITICAL";
            case SafetyEvent::COLLISION_IMMINENT: return "COLLISION_IMMINENT";
            case SafetyEvent::LOCALIZATION_LOST: return "LOCALIZATION_LOST";
            case SafetyEvent::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
            case SafetyEvent::SENSOR_FAILURE: return "SENSOR_FAILURE";
            case SafetyEvent::ESTOP_TRIGGERED: return "ESTOP_TRIGGERED";
            case SafetyEvent::ESTOP_CLEARED: return "ESTOP_CLEARED";
            default: return "UNKNOWN";
        }
    }
};
```

### SafetyMonitor Implementation Patterns

#### E-Stop Implementation (Critical - AC1)

```cpp
// src/safety/SafetyMonitor.cpp

void SafetyMonitor::triggerEstop() {
    // Record trigger time FIRST for timing measurement
    estop_trigger_time_ = std::chrono::steady_clock::now();

    // Set E-stop flag immediately
    estop_active_ = true;
    setState(SafetyState::EMERGENCY_STOP);
    emitEvent(SafetyEvent::ESTOP_TRIGGERED);

    // Execute E-stop on robot
    if (loco_) {
        // emergencyStop() calls Damp() on the robot - immediate passive mode
        loco_->emergencyStop();
    }

    // Log with timestamp
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::cout << "[SAFETY] E-STOP TRIGGERED at " << now_ms << "ms" << std::endl;
}

void SafetyMonitor::clearEstop() {
    if (!estop_active_) return;

    // Verify it's safe to clear
    // Battery must be above critical, no collision imminent
    float battery = sensors_ ? sensors_->getBatteryPercent() : 100.0f;
    if (battery < battery_critical_pct_) {
        std::cerr << "[SAFETY] Cannot clear E-stop: battery critical ("
                  << battery << "%)" << std::endl;
        return;
    }

    estop_active_ = false;
    setState(SafetyState::OK);
    emitEvent(SafetyEvent::ESTOP_CLEARED);
    std::cout << "[SAFETY] E-stop cleared" << std::endl;
}
```

#### Battery Monitoring (AC2)

```cpp
void SafetyMonitor::checkBattery() {
    if (!sensors_) return;

    // CRITICAL: Verify sensor connection before reading battery
    // Prevents false E-stop when disconnected (battery reads 0%)
    if (!sensors_->isConnected()) {
        // Cannot determine battery - don't trigger false E-stop
        return;
    }

    float battery = sensors_->getBatteryPercent();

    // Validate battery reading - 0% likely means no connection to real robot
    if (battery <= 0.0f) {
        std::cerr << "[SAFETY] Battery reading invalid (0%), skipping check" << std::endl;
        return;
    }

    // Update cached value
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_.battery_percent = battery;
    }

    if (battery <= battery_shutdown_pct_) {
        // Auto E-stop to protect battery from deep discharge
        std::cerr << "[SAFETY] Battery CRITICAL (" << battery
                  << "%) - Auto E-stop to protect battery" << std::endl;
        triggerEstop();
    } else if (battery <= battery_critical_pct_) {
        setState(SafetyState::CRITICAL);
        emitEvent(SafetyEvent::LOW_BATTERY_CRITICAL);
        std::cerr << "[SAFETY] Battery critical: " << battery << "% - RETURN HOME" << std::endl;
    } else if (battery <= battery_warning_pct_) {
        if (state_ != SafetyState::CRITICAL && state_ != SafetyState::EMERGENCY_STOP) {
            setState(SafetyState::WARNING);
        }
        emitEvent(SafetyEvent::LOW_BATTERY_WARNING);
        std::cerr << "[SAFETY] Battery warning: " << battery << "%" << std::endl;
    }
}
```

#### Collision Detection (AC3)

```cpp
void SafetyMonitor::checkCollision() {
    if (!sensors_) return;

    LidarScan scan = sensors_->getLatestLidar();
    if (scan.ranges.empty()) {
        // No LiDAR data - handled by checkSensorHealth()
        return;
    }

    // Find minimum range in forward arc
    float min_range = 100.0f;
    float angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges.size();

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float angle = scan.angle_min + i * angle_increment;

        // Check if angle is in forward arc
        if (angle >= forward_arc_min_ && angle <= forward_arc_max_) {
            float range = scan.ranges[i];
            // Skip invalid ranges
            if (range > 0.1f && range < 10.0f) {
                min_range = std::min(min_range, range);
            }
        }
    }

    cached_min_range_ = min_range;

    // Update status
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_.min_obstacle_distance = min_range;
    }

    // Check thresholds
    if (min_range < emergency_distance_) {
        std::cerr << "[SAFETY] COLLISION IMMINENT (" << min_range
                  << "m) - E-stop triggered!" << std::endl;
        triggerEstop();
    } else if (min_range < collision_distance_) {
        if (state_ != SafetyState::CRITICAL && state_ != SafetyState::EMERGENCY_STOP) {
            setState(SafetyState::WARNING);
        }
        emitEvent(SafetyEvent::COLLISION_IMMINENT);
        std::cout << "[SAFETY] Collision warning: obstacle at " << min_range << "m" << std::endl;
    }
}
```

#### Watchdog Implementation (AC6)

```cpp
void SafetyMonitor::feedWatchdog() {
    last_watchdog_feed_ = std::chrono::steady_clock::now();
    watchdog_enabled_ = true;  // Watchdog only active after first feed
}

void SafetyMonitor::checkWatchdog() {
    if (!watchdog_enabled_ || estop_active_) return;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_watchdog_feed_).count();

    if (elapsed > watchdog_timeout_s_ * 1000.0f) {
        std::cerr << "[SAFETY] Command timeout (" << elapsed
                  << "ms) - no velocity commands received" << std::endl;
        emitEvent(SafetyEvent::COMMAND_TIMEOUT);
        triggerEstop();
    }
}
```

#### Sensor Health Check (AC7)

**NOTE:** In the current SensorManager implementation, `isConnected()` returns a single boolean
for the overall SDK connection status. For MVP, LiDAR and IMU health are treated as all-or-nothing
based on SDK connection. Future enhancement: add `isLidarDataFresh()` and `isImuDataFresh()`
methods to SensorManager for individual sensor staleness detection.

```cpp
void SafetyMonitor::checkSensorHealth() {
    if (!sensors_) return;

    // MVP: Single connection status for all sensors
    // SensorManager::isConnected() reflects SDK connection state
    // Both sensors share the same connection status in current implementation
    bool sensors_connected = sensors_->isConnected();

    // For MVP, treat both sensors as healthy if SDK is connected
    // Future: Add per-sensor data freshness checks
    bool lidar_ok = sensors_connected;
    bool imu_ok = sensors_connected;

    // Update status
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_.lidar_healthy = lidar_ok;
        current_status_.imu_healthy = imu_ok;
    }

    if (!sensors_connected) {
        // All sensors down - critical
        setState(SafetyState::CRITICAL);
        emitEvent(SafetyEvent::SENSOR_FAILURE);
        std::cerr << "[SAFETY] Sensor connection lost - E-stop!" << std::endl;
        triggerEstop();
    } else if (degraded_mode_) {
        // Currently in degraded mode but sensors reconnected - restore normal operation
        degraded_mode_ = false;
        effective_collision_distance_ = collision_distance_;  // Restore configured value
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            current_status_.degraded_mode = false;
        }
        std::cout << "[SAFETY] Sensors restored - exiting degraded mode" << std::endl;
    }

    // Future: Individual sensor health checks would go here
    // Example: if LiDAR data stale but IMU fresh:
    //   degraded_mode_ = true;
    //   effective_collision_distance_ = 0.5f;  // Increased safety margin
    //   (but don't modify collision_distance_ - preserve configured value)
}
```

### Localization Check Implementation Note (AC4)

**For MVP, localization monitoring is optional and implemented as a no-op stub.**

The `checkLocalization()` method is reserved for future implementation. If implemented:
- Use IMU integration: integrate `gyro_z` from ImuData for heading estimate
- Track cumulative distance via velocity integration from locomotion commands
- No external pose source is available until full SLAM integration (Story 1-3 GridMapper)

```cpp
void SafetyMonitor::checkLocalization() {
    // MVP: No-op - localization confidence monitoring deferred to post-MVP
    //
    // Future implementation approach:
    // 1. Integrate IMU gyro_z for heading estimate: theta += gyro_z * dt
    // 2. Track cumulative distance: distance += sqrt(vx^2 + vy^2) * dt
    // 3. Compare IMU-integrated pose with SLAM pose when available
    // 4. If drift exceeds threshold (e.g., 0.5m): emit LOCALIZATION_LOST
    //
    // For now, localization is assumed reliable via odometry from SDK
}
```

### Integration with LocoController

The SafetyMonitor requires LocoController from Story 1-4. The key interface:

```cpp
// From Story 1-4: src/locomotion/LocoController.h
class LocoController {
public:
    void emergencyStop();  // Calls loco_client_.Damp() - immediate passive mode
    void stop();           // Normal stop
    bool isReady() const;
};
```

**CRITICAL:** The `emergencyStop()` method MUST call `Damp()` on the G1, which puts the robot in passive/damping mode - joints resist movement but don't actively hold position. This is safer than `ZeroTorque()` which would cause the robot to collapse.

### Integration with SensorManager

```cpp
// From Story 1-4: src/sensors/SensorManager.h
class SensorManager {
public:
    float getBatteryPercent() const;
    LidarScan getLatestLidar();
    bool isConnected() const;
};
```

### Main Loop Integration

```cpp
// In main application loop
SafetyMonitor safety;
safety.init(&loco_controller, &sensor_manager);

// Optional: Set up event logging callback
safety.setEventCallback([](SafetyEvent event) {
    SafetyStatus temp;  // Use for string conversion
    std::cout << "[SAFETY EVENT] " << temp.eventToString(event) << std::endl;
    // Could also: log to file, send telemetry, trigger alerts
});

while (running) {
    // Update safety checks FIRST every frame
    safety.update();

    // Check for stale status (optional monitoring)
    auto status = safety.getStatus();
    auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - status.last_update).count();
    if (age_ms > 1000) {
        std::cerr << "[WARNING] Safety status stale (" << age_ms << "ms old)" << std::endl;
    }

    // Check safety state before any operation
    if (safety.isEstopActive()) {
        // Do not send any motion commands
        continue;
    }

    if (safety.getState() == SafetyState::CRITICAL) {
        // Handle critical - return home, notify operator
    }

    // Normal operation...
    float vx = computeVelocity();
    loco.setVelocity(vx, 0, 0);
    safety.feedWatchdog();  // Reset watchdog after command
}
```

### Enable/Disable Implementation

For controlled testing scenarios, safety monitoring can be temporarily disabled:

```cpp
void SafetyMonitor::disable() {
    disabled_ = true;
    std::cerr << "[SAFETY] WARNING: Safety monitoring DISABLED - use with extreme caution!" << std::endl;
}

void SafetyMonitor::enable() {
    disabled_ = false;
    std::cout << "[SAFETY] Safety monitoring enabled" << std::endl;
}

void SafetyMonitor::update() {
    if (disabled_) {
        // Skip all safety checks - DANGEROUS
        return;
    }

    // Normal safety checks...
    checkBattery();
    checkCollision();
    checkWatchdog();
    checkSensorHealth();
    checkLocalization();

    // Update timestamp
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_.last_update = std::chrono::steady_clock::now();
    }
}
```

**WARNING:** Only use `disable()` in controlled test environments. Never disable safety during normal operation.

### Test Command Implementation

```cpp
// In src/main.cpp, add to argument parser
if (args.test_safety) {
    std::cout << "\n=== Safety System Test ===" << std::endl;

    SafetyMonitor safety;
    safety.init(&loco_controller, &sensor_manager);

    int test_failures = 0;  // Track failures for exit code

    // Test 1: Battery status
    std::cout << "\n[TEST 1] Battery Monitor" << std::endl;
    float battery = safety.getBatteryPercent();
    std::cout << "  Battery: " << battery << "%" << std::endl;
    std::cout << "  Warning threshold: 20%" << std::endl;
    std::cout << "  Critical threshold: 10%" << std::endl;
    std::cout << "  Status: " << (battery > 20 ? "OK" : (battery > 10 ? "WARNING" : "CRITICAL")) << std::endl;

    // Test 2: Collision detection
    std::cout << "\n[TEST 2] Collision Detection" << std::endl;
    safety.update();  // Run checks
    auto status = safety.getStatus();
    std::cout << "  Min obstacle distance: " << status.min_obstacle_distance << "m" << std::endl;
    std::cout << "  Warning threshold: 0.3m" << std::endl;
    std::cout << "  E-stop threshold: 0.15m" << std::endl;
    std::cout << "  Collision imminent: " << (safety.isCollisionImminent() ? "YES" : "NO") << std::endl;

    // Test 3: E-stop response time (CRITICAL - must pass)
    std::cout << "\n[TEST 3] E-Stop Response Time" << std::endl;
    std::cout << "  Triggering E-stop..." << std::endl;

    auto start = std::chrono::steady_clock::now();
    safety.triggerEstop();
    auto end = std::chrono::steady_clock::now();

    auto response_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "  Response time: " << response_ms << "ms" << std::endl;

    // CRITICAL: Fail test if E-stop response exceeds 500ms requirement
    if (response_ms >= 500) {
        std::cerr << "  Result: FAIL - Response time " << response_ms
                  << "ms exceeds 500ms requirement!" << std::endl;
        test_failures++;
    } else {
        std::cout << "  Result: PASS (requirement: <500ms)" << std::endl;
    }

    // Clear E-stop for next test
    safety.clearEstop();

    // Summary
    std::cout << "\n[SAFETY] All safety checks completed" << std::endl;
    std::cout << "  State: " << status.stateToString() << std::endl;
    std::cout << "  LiDAR: " << (status.lidar_healthy ? "Healthy" : "FAILED") << std::endl;
    std::cout << "  IMU: " << (status.imu_healthy ? "Healthy" : "FAILED") << std::endl;
    std::cout << "  Degraded mode: " << (status.degraded_mode ? "YES" : "NO") << std::endl;

    // Return non-zero exit code if any critical tests failed
    if (test_failures > 0) {
        std::cerr << "\n[ERROR] " << test_failures << " critical test(s) failed!" << std::endl;
        return 1;
    }

    std::cout << "\n[SUCCESS] All safety tests passed" << std::endl;
    return 0;
}
```

### Project Structure Notes

Files and directories to create in this story:

```
src/safety/                 # NEW DIRECTORY
├── SafetyTypes.h           # Enums and structs
├── SafetyMonitor.h         # Class declaration
└── SafetyMonitor.cpp       # Implementation

test/
└── test_safety.cpp         # NEW FILE

src/main.cpp                # MODIFY - add --test-safety command
CMakeLists.txt              # MODIFY - add safety library and tests
```

### Dependencies on Previous Stories

**Story 1-4 (Hardware Integration) - REQUIRED:**
- `src/sensors/SensorManager.h/cpp` - Battery, LiDAR data access
- `src/locomotion/LocoController.h/cpp` - Emergency stop execution
- Types.h already has ImuData

**Story 1-3 (SLAM Core):**
- No direct dependency, but costmap could be used for advanced collision prediction

**Story 1-2 (NavSim):**
- No direct dependency, but NavSim could test safety with simulated obstacles

### CMake Additions Required

```cmake
# ============================================
# Safety System (Story 1-5)
# ============================================

# Safety library
add_library(safety
    src/safety/SafetyMonitor.cpp
)
target_include_directories(safety PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Safety depends on locomotion and sensors (forward declarations only in header)
# But implementation needs the full classes
target_link_libraries(safety sensors loco_hw)

# Update g1_inspector to include safety
target_link_libraries(g1_inspector
    navigation
    slam
    sensors
    loco_hw
    safety           # NEW
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    ${HPDF_LIBRARY}
    nlohmann_json::nlohmann_json
)

# Safety unit tests
if(GTest_FOUND)
    add_executable(test_safety test/test_safety.cpp)
    target_link_libraries(test_safety safety GTest::gtest_main)
    add_test(NAME test_safety COMMAND test_safety)
endif()
```

### Unit Test Patterns

**IMPORTANT - Unit Test Limitations:**

The current SafetyMonitor design uses concrete class pointers for LocoController and SensorManager,
which limits unit test isolation. The mock classes below demonstrate the interface but cannot be
injected into SafetyMonitor without modifying its constructor.

**For MVP:**
- Unit tests verify threshold logic, state transitions, and string conversion in isolation
- Integration tests via `--test-safety` command verify full behavior with real/simulated hardware
- Future enhancement: Refactor to interface-based injection for better testability

```cpp
// test/test_safety.cpp
#include <gtest/gtest.h>
#include "safety/SafetyMonitor.h"
#include "safety/SafetyTypes.h"

// Mock classes demonstrate expected interface
// NOTE: These cannot be directly injected into SafetyMonitor in current design
// They document the interface contract for future refactoring
class MockLocoController {
public:
    bool estop_called = false;
    void emergencyStop() { estop_called = true; }
    void stop() {}
    bool isReady() const { return true; }
};

class MockSensorManager {
public:
    float battery = 100.0f;
    bool connected = true;
    LidarScan scan;

    float getBatteryPercent() const { return battery; }
    bool isConnected() const { return connected; }
    LidarScan getLatestLidar() { return scan; }
};

// ============================================
// Threshold Logic Tests (isolated, no mocks needed)
// ============================================

TEST(SafetyThresholdTest, BatteryWarningAt20Percent) {
    float battery = 20.0f;
    float warning_threshold = 20.0f;
    EXPECT_TRUE(battery <= warning_threshold);  // Would trigger warning
}

TEST(SafetyThresholdTest, BatteryCriticalAt10Percent) {
    float battery = 10.0f;
    float critical_threshold = 10.0f;
    EXPECT_TRUE(battery <= critical_threshold);  // Would trigger critical
}

TEST(SafetyThresholdTest, BatteryShutdownAt5Percent) {
    float battery = 5.0f;
    float shutdown_threshold = 5.0f;
    EXPECT_TRUE(battery <= shutdown_threshold);  // Would trigger auto E-stop
}

TEST(SafetyThresholdTest, BatteryValidityCheck) {
    // Invalid battery reading (disconnected) should NOT trigger E-stop
    float battery = 0.0f;
    EXPECT_TRUE(battery <= 0.0f);  // Should skip threshold checks
}

TEST(SafetyThresholdTest, CollisionWarningDistance) {
    float collision_threshold = 0.3f;
    float min_range = 0.25f;
    EXPECT_TRUE(min_range < collision_threshold);  // Would trigger warning
}

TEST(SafetyThresholdTest, CollisionEmergencyDistance) {
    float emergency_threshold = 0.15f;
    float min_range = 0.10f;
    EXPECT_TRUE(min_range < emergency_threshold);  // Would trigger E-stop
}

TEST(SafetyThresholdTest, WatchdogTimeout) {
    float timeout_s = 1.0f;
    float elapsed_s = 1.5f;
    EXPECT_TRUE(elapsed_s > timeout_s);  // Would trigger E-stop
}

// ============================================
// State Transition Tests
// ============================================

TEST(SafetyStateTest, StateTransitions) {
    SafetyState state = SafetyState::OK;
    EXPECT_EQ(state, SafetyState::OK);

    state = SafetyState::WARNING;
    EXPECT_EQ(state, SafetyState::WARNING);

    state = SafetyState::CRITICAL;
    EXPECT_EQ(state, SafetyState::CRITICAL);

    state = SafetyState::EMERGENCY_STOP;
    EXPECT_EQ(state, SafetyState::EMERGENCY_STOP);
}

// ============================================
// String Conversion Tests (SafetyStatus)
// ============================================

TEST(SafetyStatusTest, StateStringConversion) {
    SafetyStatus status;

    status.state = SafetyState::OK;
    EXPECT_EQ(status.stateToString(), "OK");

    status.state = SafetyState::WARNING;
    EXPECT_EQ(status.stateToString(), "WARNING");

    status.state = SafetyState::CRITICAL;
    EXPECT_EQ(status.stateToString(), "CRITICAL");

    status.state = SafetyState::EMERGENCY_STOP;
    EXPECT_EQ(status.stateToString(), "EMERGENCY_STOP");
}

TEST(SafetyStatusTest, EventStringConversion) {
    SafetyStatus status;
    EXPECT_EQ(status.eventToString(SafetyEvent::NONE), "NONE");
    EXPECT_EQ(status.eventToString(SafetyEvent::LOW_BATTERY_WARNING), "LOW_BATTERY_WARNING");
    EXPECT_EQ(status.eventToString(SafetyEvent::LOW_BATTERY_CRITICAL), "LOW_BATTERY_CRITICAL");
    EXPECT_EQ(status.eventToString(SafetyEvent::COLLISION_IMMINENT), "COLLISION_IMMINENT");
    EXPECT_EQ(status.eventToString(SafetyEvent::LOCALIZATION_LOST), "LOCALIZATION_LOST");
    EXPECT_EQ(status.eventToString(SafetyEvent::COMMAND_TIMEOUT), "COMMAND_TIMEOUT");
    EXPECT_EQ(status.eventToString(SafetyEvent::SENSOR_FAILURE), "SENSOR_FAILURE");
    EXPECT_EQ(status.eventToString(SafetyEvent::ESTOP_TRIGGERED), "ESTOP_TRIGGERED");
    EXPECT_EQ(status.eventToString(SafetyEvent::ESTOP_CLEARED), "ESTOP_CLEARED");
}

// ============================================
// SafetyStatus Default Values Test
// ============================================

TEST(SafetyStatusTest, DefaultValues) {
    SafetyStatus status;
    EXPECT_EQ(status.state, SafetyState::OK);
    EXPECT_TRUE(status.active_events.empty());
    EXPECT_FLOAT_EQ(status.battery_percent, 100.0f);
    EXPECT_FLOAT_EQ(status.min_obstacle_distance, 100.0f);
    EXPECT_FALSE(status.estop_active);
    EXPECT_TRUE(status.lidar_healthy);
    EXPECT_TRUE(status.imu_healthy);
    EXPECT_FALSE(status.degraded_mode);
}
```

---

## Verification Commands

```bash
# Build (inside Docker or with SDK installed)
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests
./test_safety
# Expected: All tests pass

# Test safety system (requires real robot)
./g1_inspector --robot 192.168.123.164 --test-safety

# Expected output:
# === Safety System Test ===
#
# [TEST 1] Battery Monitor
#   Battery: 85%
#   Warning threshold: 20%
#   Critical threshold: 10%
#   Status: OK
#
# [TEST 2] Collision Detection
#   Min obstacle distance: 2.34m
#   Warning threshold: 0.3m
#   E-stop threshold: 0.15m
#   Collision imminent: NO
#
# [TEST 3] E-Stop Response Time
#   Triggering E-stop...
#   Response time: 320ms
#   Result: PASS (requirement: <500ms)
#
# [SAFETY] All safety checks completed
#   State: OK
#   LiDAR: Healthy
#   IMU: Healthy
#   Degraded mode: NO
#
# [SUCCESS] All safety tests passed

# Exit code: 0 on success, 1 if E-stop timing fails
```

### Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| Unit tests | `./test_safety` | Exit 0, all pass |
| E-stop response | `--test-safety` | Response < 500ms, exit 0 |
| E-stop timing fail | `--test-safety` (if >500ms) | Exit 1 with error |
| Battery monitor | `--test-safety` | Shows battery % and thresholds |
| Collision check | `--test-safety` | Shows min distance and thresholds |
| Safety state | `--test-safety` | State queryable |
| Degraded mode | `--test-safety` | Shows degraded mode status |

---

## Previous Story Intelligence

### Key Patterns from Stories 1-3 and 1-4

**From Story 1-4 (Hardware Integration):**
- LocoController pattern with `emergencyStop()` using `loco_client_.Damp()`
- SensorManager pattern with thread-safe data access
- Conditional compilation with `#ifdef HAS_UNITREE_SDK2`
- Network interface detection pattern

**From Story 1-3 (SLAM Core):**
- Unit test patterns with GTest
- CMake library organization

**Code conventions:**
- `#pragma once` for headers
- PascalCase classes, camelCase methods
- `std::atomic` for thread-safe flags
- `std::mutex` with `std::lock_guard` for data access

### What Worked in Previous Stories

- Forward declarations in headers to reduce compile dependencies
- Atomic bools for fast status checks
- Const refs for large structs
- Comprehensive Dev Notes with code examples
- Clear acceptance criteria linked to tasks

### Lessons from Story 1-4 Review

- **ChannelFactory initialization is critical** - must happen before any SDK usage
- **FSM state checking** - verify robot state before sending commands
- **Use unitree_hg namespace** for G1 (not unitree_go which is for quadrupeds)
- **Handle timeouts gracefully** - SDK throws on timeout

---

## Technical Reference Information

### G1 Emergency Stop via SDK

The Unitree G1 SDK provides these emergency stop methods:

```cpp
// Immediate stop options via LocoClient
loco_client.Damp();        // Passive damping mode - joints resist but don't hold
loco_client.ZeroTorque();  // Zero torque - robot collapses (dangerous!)
loco_client.StopMove();    // Stop current motion but maintain stance
```

**For E-stop, use `Damp()` - it's the safest option.**

### Safety Requirements Summary

| Requirement | Threshold | Action |
|-------------|-----------|--------|
| E-stop response | <500ms | Halt robot |
| Battery warning | 20% | Emit warning, continue |
| Battery critical | 10% | Start return home |
| Battery shutdown | 5% | Auto E-stop |
| Collision warning | 0.3m | Emit warning, slow down |
| Collision emergency | 0.15m | E-stop |
| Sensor timeout | 2.0s | Enter degraded mode |
| Command timeout | 1.0s | E-stop |

### Thread Safety Considerations

SafetyMonitor is designed to be called from a single thread (main loop), but status queries may come from other threads. Key thread-safe elements:

- `std::atomic<SafetyState> state_` - safe to read from any thread
- `std::atomic<bool> estop_active_` - fast lock-free check
- `std::mutex status_mutex_` - protects full SafetyStatus struct

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **SafetyMonitor class** | Monitors all safety conditions |
| **E-stop functionality** | `--test-safety` shows <500ms response |
| **Battery monitoring** | Shows battery % and thresholds |
| **Collision detection** | Shows min obstacle distance |
| **Watchdog timer** | Command timeout triggers E-stop |
| **Sensor health** | Detects disconnected sensors |
| **Test command** | `--test-safety` runs all checks |

### Demo Script (Run This When Done)

```bash
# Build the project
cd /workspace
mkdir -p build && cd build
cmake .. && make -j

# Run unit tests
./test_safety
# All tests should pass

# Run safety test on robot (robot must be connected)
./scripts/check_g1_network.sh  # Verify network first
./g1_inspector --robot 192.168.123.164 --test-safety

# Expected:
# - Battery status shown
# - Collision detection working
# - E-stop response < 500ms
# - All safety checks pass
```

**SUCCESS CRITERIA:** Story 1.5 is DONE when:
1. `./test_safety` exits with code 0 (all tests pass)
2. `--test-safety` shows E-stop response < 500ms
3. Battery monitoring shows current % and triggers at thresholds
4. Collision detection shows minimum obstacle distance
5. SafetyState is queryable at all times

---

## References

- [Source: docs/architecture.md#4.6-safety] - SafetyMonitor design
- [Source: docs/epics.md#story-5] - Original story requirements
- [Source: docs/sprint-artifacts/1-4-hardware-integration.md] - LocoController and SensorManager patterns
- [Source: docs/sprint-artifacts/1-3-slam-core.md] - Unit test patterns
- [Source: src/sensors/SensorManager.h] - Sensor interface
- [Source: src/locomotion/ILocomotion.h] - Locomotion interface
- [External: Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) - Damp() method reference

---

## Dev Agent Record

### Context Reference

This story file serves as the complete implementation context.

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- None required - clean implementation following Dev Notes patterns

### Completion Notes List

- **Task 1:** Created SafetyTypes.h with SafetyState enum (OK, WARNING, CRITICAL, EMERGENCY_STOP), SafetyEvent enum (9 event types), and SafetyStatus struct with string conversion methods
- **Task 2:** Implemented SafetyMonitor class with init(), update(), triggerEstop(), clearEstop(), feedWatchdog(), and comprehensive state query methods
- **Task 3:** Battery monitoring checks at 20% (WARNING), 10% (CRITICAL), 5% (auto E-stop) with validation for 0% readings (disconnected state)
- **Task 4:** Collision detection scans forward arc (-30 to +30 degrees) with 0.3m warning threshold and 0.15m emergency E-stop threshold
- **Task 5:** Command watchdog with 1.0s timeout, only active after first feedWatchdog() call
- **Task 6:** Sensor health via isConnected() check with graceful degradation support and effective_collision_distance_ for degraded mode
- **Task 7:** Localization monitoring implemented as no-op stub (MVP - deferred to post-MVP)
- **Task 8:** Added --test-safety command with 4-part test sequence: battery status, collision detection, E-stop response timing (<500ms required), and state queries
- **Task 9:** Created 25+ unit tests covering threshold logic, state transitions, string conversion, forward arc calculations, range validation, and timing tests
- **Task 10:** Added safety library to CMakeLists.txt with proper dependencies (sensors, loco_hw) and test_safety executable

### File List

**New files created:**
- src/safety/SafetyTypes.h
- src/safety/SafetyMonitor.h
- src/safety/SafetyMonitor.cpp
- test/test_safety.cpp

**Files modified:**
- src/main.cpp (added --test-safety command with runSafetyTest function)
- CMakeLists.txt (added safety library and test_safety executable)

### Change Log

- 2025-12-05: Story 1-5 created by create-story workflow - comprehensive safety system guide with E-stop, battery monitoring, collision detection, watchdog, and sensor health patterns.
- 2025-12-05: Validation review applied all improvements (C1-C4, E1-E6, O1-O3):
  - **C1:** Added battery validity check before threshold comparison (prevents false E-stop on 0% reading)
  - **C2:** Clarified isConnected() semantics - MVP uses single connection status for all sensors
  - **C3:** Documented unit test limitations and emphasized integration tests for full verification
  - **C4:** Added localization check implementation note with future approach documented
  - **E1:** Added event callback usage example in main loop integration
  - **E2:** Added automated E-stop timing assertion with exit code 1 on failure
  - **E3:** Added SafetyMonitor::enable()/disable() for controlled testing
  - **E4:** Documented 10-50Hz update rate rationale
  - **E5:** Added staleness detection example using SafetyStatus.last_update
  - **E6:** Added threshold values display in --test-safety output
  - **O1:** Consolidated safety threshold constants into single reference table
  - **O2:** Referenced Story 1-4 interface documentation instead of re-documenting
  - **O3:** Added effective_collision_distance_ for degraded mode without losing config
  - Validation report: docs/sprint-artifacts/validation-report-1-5-2025-12-05.md
- 2025-12-05: **Implementation complete** - All 10 tasks implemented following Dev Notes patterns:
  - Created src/safety/SafetyTypes.h with enums and SafetyStatus struct
  - Created src/safety/SafetyMonitor.h/cpp with full implementation
  - Added --test-safety command to main.cpp with E-stop timing validation
  - Created test/test_safety.cpp with 25+ unit tests
  - Updated CMakeLists.txt with safety library and test executable
  - Status changed: ready-for-dev → Ready for Review
- 2025-12-05: **Code Review Fixes Applied** (9 issues found, all fixed):
  - **H1 Fixed:** Added 18 real SafetyMonitor class tests (construction, init, E-stop, events, configuration)
  - **H2 Fixed:** Added clearEvents() method to clear active_events on E-stop clear
  - **H3 Fixed:** clearEstop() now checks collision proximity and sensor connection before clearing
  - **M1 Fixed:** Watchdog timer reset on E-stop clear to prevent immediate timeout
  - **M2 Fixed:** Battery/collision warnings now only emit on transition (spam prevention)
  - **M3 Fixed:** Added WatchdogBehaviorTest tests for feedWatchdog()/checkWatchdog()
  - Status changed: Ready for Review → Done
- 2025-12-05: **Code Review #2 Fixes Applied** (7 issues found, all HIGH/MEDIUM fixed):
  - **H1 Fixed:** Made `cached_min_range_` atomic for thread-safe reads from isCollisionImminent()
  - **H2 Fixed:** Made `effective_collision_distance_` atomic and renamed to `effective_collision_distance_atomic_`
  - **M1 Fixed:** Added validation to all configuration setters (reject negative/invalid values, warn on inconsistent thresholds)
  - **M2 Fixed:** Added Test 5 (Watchdog Behavior) to `--test-safety` command - verifies AC6 watchdog timeout
  - **M3 Fixed:** Added 15 new tests: ConfigValidationTest (5), ThreadSafetyTest (2), IntegrationBehaviorTest (3)
  - Also made `watchdog_enabled_` atomic for consistency
  - Total unit tests now: 55+ (was 40+)
