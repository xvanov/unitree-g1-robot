#pragma once

#include <cstdint>
#include <atomic>
#include "locomotion/LocoController.h"

// Use official Unitree SDK Gamepad class for parsing
// This ensures compatibility with Unitree's gamepad data format
#include "external/unitree_sdk2/example/g1/low_level/gamepad.hpp"

// Teleop command structure with velocities and button states
struct TeleopCommand {
    float vx = 0.0f;           // Forward/back velocity (m/s)
    float vy = 0.0f;           // Lateral velocity (m/s)
    float omega = 0.0f;        // Rotation velocity (rad/s)

    bool stand = false;        // Request to stand up
    bool sit = false;          // Request to sit down
    bool emergency_stop = false; // Emergency stop request

    // Button edge detection for single-shot actions
    bool stand_pressed = false;   // Stand button just pressed
    bool sit_pressed = false;     // Sit button just pressed
    bool estop_pressed = false;   // E-stop just pressed
};

class TeleopController {
public:
    TeleopController();
    ~TeleopController() = default;

    // Update from raw wireless_remote 40-byte buffer
    void update(const uint8_t wireless_remote[40]);

    // Get current teleop command (with safety limits applied)
    TeleopCommand getCommand() const;

    // Check if gamepad is connected (has valid header bytes)
    bool isConnected() const { return connected_; }

    // Configuration
    void setDeadzone(float deadzone);
    void setSmoothFactor(float smooth);
    void setMaxVelocityScale(float scale) { velocity_scale_ = scale; }

    // Get raw stick values (for debugging/display)
    float getRawLX() const { return gamepad_.lx; }
    float getRawLY() const { return gamepad_.ly; }
    float getRawRX() const { return gamepad_.rx; }
    float getRawRY() const { return gamepad_.ry; }

private:
    // Official Unitree SDK Gamepad class - handles parsing, deadzone, and smoothing
    unitree::common::Gamepad gamepad_;

    // Configuration
    float velocity_scale_ = 1.0f; // Scale factor for velocities (0-1)

    // Connection state
    bool connected_ = false;
    uint8_t last_header_[2] = {0, 0};
};
