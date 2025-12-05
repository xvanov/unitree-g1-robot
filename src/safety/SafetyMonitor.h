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
    void clearEvents();  // Clear active_events list

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

    // Thresholds (configurable) - collision_distance_ is config, effective is runtime (atomic)
    float collision_distance_ = 0.3f;         // meters - configured value (written only via setter)
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

    // Cached values for performance (atomic for thread-safe reads)
    std::atomic<float> cached_min_range_{100.0f};
    std::atomic<float> cached_battery_{100.0f};
    std::atomic<float> effective_collision_distance_atomic_{0.3f};
    std::atomic<bool> watchdog_enabled_{false};

    // Event spam prevention - track last emitted events
    SafetyEvent last_battery_event_ = SafetyEvent::NONE;
    bool collision_warning_active_ = false;
};
