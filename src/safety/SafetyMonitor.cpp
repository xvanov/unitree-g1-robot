#include "safety/SafetyMonitor.h"
#include "sensors/SensorManager.h"
#include "locomotion/LocoController.h"
#include "sensors/ISensorSource.h"

#include <iostream>
#include <algorithm>
#include <cmath>

SafetyMonitor::SafetyMonitor() {
    last_watchdog_feed_ = std::chrono::steady_clock::now();
    last_sensor_update_ = std::chrono::steady_clock::now();
    current_status_.last_update = std::chrono::steady_clock::now();
}

SafetyMonitor::~SafetyMonitor() = default;

void SafetyMonitor::init(LocoController* loco, SensorManager* sensors) {
    loco_ = loco;
    sensors_ = sensors;

    // Reset state
    state_ = SafetyState::OK;
    estop_active_ = false;
    degraded_mode_ = false;
    watchdog_enabled_.store(false);
    effective_collision_distance_atomic_.store(collision_distance_);

    std::cout << "[SAFETY] SafetyMonitor initialized" << std::endl;
}

void SafetyMonitor::update() {
    if (disabled_) {
        // Skip all safety checks - DANGEROUS
        return;
    }

    // Run all safety checks
    checkBattery();
    checkCollision();
    checkWatchdog();
    checkSensorHealth();
    checkLocalization();

    // Update timestamp
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_.last_update = std::chrono::steady_clock::now();
        current_status_.state = state_;
        current_status_.estop_active = estop_active_;
        current_status_.degraded_mode = degraded_mode_;
    }
}

// ============================================
// E-Stop Implementation (AC1)
// ============================================

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
    std::cerr << "[SAFETY] E-STOP TRIGGERED at " << now_ms << "ms" << std::endl;
}

void SafetyMonitor::clearEstop() {
    if (!estop_active_) return;

    // Verify it's safe to clear - check ALL safety conditions
    // 1. Battery must be above critical
    float battery = cached_battery_;
    if (battery > 0.0f && battery < battery_critical_pct_) {
        std::cerr << "[SAFETY] Cannot clear E-stop: battery critical ("
                  << battery << "%)" << std::endl;
        return;
    }

    // 2. No collision imminent
    float min_range = cached_min_range_.load();
    if (min_range < emergency_distance_) {
        std::cerr << "[SAFETY] Cannot clear E-stop: collision imminent ("
                  << min_range << "m)" << std::endl;
        return;
    }

    // 3. Sensors must be connected
    if (sensors_ && !sensors_->isConnected()) {
        std::cerr << "[SAFETY] Cannot clear E-stop: sensors disconnected" << std::endl;
        return;
    }

    estop_active_ = false;
    setState(SafetyState::OK);
    clearEvents();  // Clear stale events on E-stop clear
    emitEvent(SafetyEvent::ESTOP_CLEARED);

    // Reset watchdog timer to prevent immediate timeout after E-stop clear
    last_watchdog_feed_ = std::chrono::steady_clock::now();
    watchdog_enabled_.store(false);  // Require explicit feed to re-enable

    // Reset event spam prevention state
    last_battery_event_ = SafetyEvent::NONE;
    collision_warning_active_ = false;

    std::cerr << "[SAFETY] E-stop cleared" << std::endl;
}

// ============================================
// Battery Monitoring (AC2)
// ============================================

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
        // Skip threshold checks for invalid reading
        return;
    }

    // Update cached value
    cached_battery_ = battery;
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
        // Only emit event on transition to prevent spam
        if (last_battery_event_ != SafetyEvent::LOW_BATTERY_CRITICAL) {
            emitEvent(SafetyEvent::LOW_BATTERY_CRITICAL);
            last_battery_event_ = SafetyEvent::LOW_BATTERY_CRITICAL;
            std::cerr << "[SAFETY] Battery critical: " << battery << "% - RETURN HOME" << std::endl;
        }
    } else if (battery <= battery_warning_pct_) {
        if (state_ != SafetyState::CRITICAL && state_ != SafetyState::EMERGENCY_STOP) {
            setState(SafetyState::WARNING);
        }
        // Only emit event on transition to prevent spam
        if (last_battery_event_ != SafetyEvent::LOW_BATTERY_WARNING &&
            last_battery_event_ != SafetyEvent::LOW_BATTERY_CRITICAL) {
            emitEvent(SafetyEvent::LOW_BATTERY_WARNING);
            last_battery_event_ = SafetyEvent::LOW_BATTERY_WARNING;
            std::cerr << "[SAFETY] Battery warning: " << battery << "%" << std::endl;
        }
    } else {
        // Battery OK - reset event tracking
        last_battery_event_ = SafetyEvent::NONE;
    }
}

// ============================================
// Collision Detection (AC3)
// ============================================

void SafetyMonitor::checkCollision() {
    if (!sensors_) return;

    LidarScan scan = sensors_->getLatestLidar();
    if (scan.ranges.empty()) {
        // No LiDAR data - handled by checkSensorHealth()
        return;
    }

    // Find minimum range in forward arc
    float min_range = 100.0f;
    float angle_increment = (scan.angle_max - scan.angle_min) /
                            static_cast<float>(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float angle = scan.angle_min + static_cast<float>(i) * angle_increment;

        // Check if angle is in forward arc
        if (angle >= forward_arc_min_ && angle <= forward_arc_max_) {
            float range = scan.ranges[i];
            // Skip invalid ranges
            if (range > 0.1f && range < 10.0f) {
                min_range = std::min(min_range, range);
            }
        }
    }

    cached_min_range_.store(min_range);

    // Update status
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_.min_obstacle_distance = min_range;
    }

    // Check thresholds (use atomic load for thread-safe read)
    float effective_dist = effective_collision_distance_atomic_.load();
    if (min_range < emergency_distance_) {
        std::cerr << "[SAFETY] COLLISION IMMINENT (" << min_range
                  << "m) - E-stop triggered!" << std::endl;
        triggerEstop();
    } else if (min_range < effective_dist) {
        if (state_ != SafetyState::CRITICAL && state_ != SafetyState::EMERGENCY_STOP) {
            setState(SafetyState::WARNING);
        }
        // Only emit event on transition to prevent spam
        if (!collision_warning_active_) {
            emitEvent(SafetyEvent::COLLISION_IMMINENT);
            collision_warning_active_ = true;
            std::cerr << "[SAFETY] Collision warning: obstacle at " << min_range << "m" << std::endl;
        }
    } else {
        // Clear of collision zone - reset tracking
        collision_warning_active_ = false;
    }
}

// ============================================
// Command Watchdog (AC6)
// ============================================

void SafetyMonitor::feedWatchdog() {
    last_watchdog_feed_ = std::chrono::steady_clock::now();
    watchdog_enabled_.store(true);  // Watchdog only active after first feed
}

void SafetyMonitor::checkWatchdog() {
    if (!watchdog_enabled_.load() || estop_active_) return;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_watchdog_feed_).count();

    if (elapsed > static_cast<long long>(watchdog_timeout_s_ * 1000.0f)) {
        std::cerr << "[SAFETY] Command timeout (" << elapsed
                  << "ms) - no velocity commands received" << std::endl;
        emitEvent(SafetyEvent::COMMAND_TIMEOUT);
        triggerEstop();
    }
}

// ============================================
// Sensor Health Check (AC7)
// ============================================

void SafetyMonitor::checkSensorHealth() {
    if (!sensors_) return;

    // MVP: Single connection status for all sensors
    // SensorManager::isConnected() reflects SDK connection state
    bool sensors_connected = sensors_->isConnected();

    // For MVP, treat both sensors as healthy if SDK is connected
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
        effective_collision_distance_atomic_.store(collision_distance_);  // Restore configured value
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            current_status_.degraded_mode = false;
        }
        std::cout << "[SAFETY] Sensors restored - exiting degraded mode" << std::endl;
    }
}

// ============================================
// Localization Monitoring (AC4 - Optional MVP)
// ============================================

void SafetyMonitor::checkLocalization() {
    // MVP: No-op - localization confidence monitoring deferred to post-MVP
    //
    // Future implementation approach:
    // 1. Integrate IMU gyro_z for heading estimate: theta += gyro_z * dt
    // 2. Track cumulative distance: distance += sqrt(vx^2 + vy^2) * dt
    // 3. Compare IMU-integrated pose with SLAM pose when available
    // 4. If drift exceeds threshold (e.g., 0.5m): emit LOCALIZATION_LOST
}

// ============================================
// State Management
// ============================================

void SafetyMonitor::setState(SafetyState new_state) {
    state_ = new_state;
}

void SafetyMonitor::emitEvent(SafetyEvent event) {
    // Add to active events
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        // Check if event is already in active_events to avoid duplicates
        bool found = false;
        for (const auto& e : current_status_.active_events) {
            if (e == event) {
                found = true;
                break;
            }
        }
        if (!found) {
            current_status_.active_events.push_back(event);
        }
    }

    // Call callback if set
    if (event_callback_) {
        event_callback_(event);
    }
}

void SafetyMonitor::clearEvents() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_.active_events.clear();
}

// ============================================
// State Queries
// ============================================

SafetyState SafetyMonitor::getState() const {
    return state_;
}

SafetyStatus SafetyMonitor::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_;
}

bool SafetyMonitor::isEstopActive() const {
    return estop_active_;
}

bool SafetyMonitor::isCollisionImminent() const {
    return cached_min_range_.load() < effective_collision_distance_atomic_.load();
}

float SafetyMonitor::getBatteryPercent() const {
    return cached_battery_;
}

bool SafetyMonitor::isLidarHealthy() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_.lidar_healthy;
}

bool SafetyMonitor::isImuHealthy() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_.imu_healthy;
}

bool SafetyMonitor::isInDegradedMode() const {
    return degraded_mode_;
}

// ============================================
// Configuration (with validation)
// ============================================

void SafetyMonitor::setCollisionDistance(float meters) {
    if (meters <= 0.0f) {
        std::cerr << "[SAFETY] Invalid collision distance: " << meters << "m (must be > 0)" << std::endl;
        return;
    }
    if (meters <= emergency_distance_) {
        std::cerr << "[SAFETY] Warning: collision distance (" << meters
                  << "m) should be > emergency distance (" << emergency_distance_ << "m)" << std::endl;
    }
    collision_distance_ = meters;
    effective_collision_distance_atomic_.store(meters);
}

void SafetyMonitor::setEmergencyDistance(float meters) {
    if (meters <= 0.0f) {
        std::cerr << "[SAFETY] Invalid emergency distance: " << meters << "m (must be > 0)" << std::endl;
        return;
    }
    if (meters >= collision_distance_) {
        std::cerr << "[SAFETY] Warning: emergency distance (" << meters
                  << "m) should be < collision distance (" << collision_distance_ << "m)" << std::endl;
    }
    emergency_distance_ = meters;
}

void SafetyMonitor::setBatteryWarningThreshold(float pct) {
    if (pct <= 0.0f || pct > 100.0f) {
        std::cerr << "[SAFETY] Invalid battery warning threshold: " << pct << "% (must be 0-100)" << std::endl;
        return;
    }
    if (pct <= battery_critical_pct_) {
        std::cerr << "[SAFETY] Warning: battery warning (" << pct
                  << "%) should be > critical (" << battery_critical_pct_ << "%)" << std::endl;
    }
    battery_warning_pct_ = pct;
}

void SafetyMonitor::setBatteryCriticalThreshold(float pct) {
    if (pct <= 0.0f || pct > 100.0f) {
        std::cerr << "[SAFETY] Invalid battery critical threshold: " << pct << "% (must be 0-100)" << std::endl;
        return;
    }
    if (pct >= battery_warning_pct_) {
        std::cerr << "[SAFETY] Warning: battery critical (" << pct
                  << "%) should be < warning (" << battery_warning_pct_ << "%)" << std::endl;
    }
    battery_critical_pct_ = pct;
}

void SafetyMonitor::setWatchdogTimeout(float seconds) {
    if (seconds <= 0.0f) {
        std::cerr << "[SAFETY] Invalid watchdog timeout: " << seconds << "s (must be > 0)" << std::endl;
        return;
    }
    if (seconds > 10.0f) {
        std::cerr << "[SAFETY] Warning: watchdog timeout (" << seconds
                  << "s) is very long - robot may travel far before stopping" << std::endl;
    }
    watchdog_timeout_s_ = seconds;
}

// ============================================
// Enable/Disable for Testing
// ============================================

void SafetyMonitor::disable() {
    disabled_ = true;
    std::cerr << "[SAFETY] WARNING: Safety monitoring DISABLED - use with extreme caution!" << std::endl;
}

void SafetyMonitor::enable() {
    disabled_ = false;
    std::cout << "[SAFETY] Safety monitoring enabled" << std::endl;
}
