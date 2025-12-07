#include "teleop/TeleopController.h"
#include <cstring>
#include <cmath>
#include <algorithm>

TeleopController::TeleopController() {
    // Configure SDK Gamepad with reasonable defaults for teleop
    gamepad_.dead_zone = 0.05f;  // 5% deadzone
    gamepad_.smooth = 0.2f;      // Smoothing factor
}

void TeleopController::setDeadzone(float deadzone) {
    gamepad_.dead_zone = deadzone;
}

void TeleopController::setSmoothFactor(float smooth) {
    gamepad_.smooth = smooth;
}

void TeleopController::update(const uint8_t wireless_remote[40]) {
    // Parse raw data using SDK's REMOTE_DATA_RX union
    unitree::common::REMOTE_DATA_RX remote;
    std::memcpy(remote.buff, wireless_remote, 40);

    // Check for valid gamepad connection (header bytes should be non-zero when connected)
    bool has_valid_header = (remote.buff[0] != 0 || remote.buff[1] != 0);

    // Additional check: header bytes should be stable (not changing randomly)
    if (has_valid_header) {
        if (last_header_[0] == 0 && last_header_[1] == 0) {
            // First time seeing valid header
            connected_ = true;
        } else if (last_header_[0] == remote.buff[0] && last_header_[1] == remote.buff[1]) {
            // Header matches previous - consistent connection
            connected_ = true;
        }
        last_header_[0] = remote.buff[0];
        last_header_[1] = remote.buff[1];
    } else {
        connected_ = false;
        return;
    }

    // Use official SDK Gamepad class for parsing - handles deadzone and smoothing
    gamepad_.update(remote.RF_RX);
}

TeleopCommand TeleopController::getCommand() const {
    TeleopCommand cmd;

    if (!connected_) {
        // Return zero command when disconnected
        return cmd;
    }

    // Map sticks to velocities using SDK Gamepad's processed values:
    // Left stick Y (ly) -> forward/back velocity (vx)
    // Left stick X (lx) -> lateral velocity (vy)
    // Right stick X (rx) -> rotation (omega)
    float raw_vx = gamepad_.ly * SafetyLimits::MAX_VX;
    float raw_vy = gamepad_.lx * SafetyLimits::MAX_VY;
    float raw_omega = gamepad_.rx * SafetyLimits::MAX_OMEGA;

    // Apply velocity scale
    raw_vx *= velocity_scale_;
    raw_vy *= velocity_scale_;
    raw_omega *= velocity_scale_;

    // Apply safety limits (clamp to max velocities)
    cmd.vx = std::clamp(raw_vx, -SafetyLimits::MAX_VX, SafetyLimits::MAX_VX);
    cmd.vy = std::clamp(raw_vy, -SafetyLimits::MAX_VY, SafetyLimits::MAX_VY);
    cmd.omega = std::clamp(raw_omega, -SafetyLimits::MAX_OMEGA, SafetyLimits::MAX_OMEGA);

    // Button mappings using SDK Button class:
    // A button = stand up
    // B button = sit down
    // Start button = emergency stop
    cmd.stand = gamepad_.A.pressed;
    cmd.sit = gamepad_.B.pressed;
    cmd.emergency_stop = gamepad_.start.pressed;

    // Edge detection using SDK Button's on_press
    cmd.stand_pressed = gamepad_.A.on_press;
    cmd.sit_pressed = gamepad_.B.on_press;
    cmd.estop_pressed = gamepad_.start.on_press;

    return cmd;
}
