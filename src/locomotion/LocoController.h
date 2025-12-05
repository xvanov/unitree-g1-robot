#pragma once

#include <string>
#include <atomic>
#include <mutex>
#include <memory>

// G1 FSM State IDs
namespace G1FSM {
    constexpr int INVALID = 0;
    constexpr int DAMP = 1;
    constexpr int STAND_UP = 2;
    constexpr int STAND_DOWN = 3;
    constexpr int STANDING = 100;
    constexpr int WALKING = 101;
}

// Safety limits for velocity commands (conservative indoor values)
namespace SafetyLimits {
    constexpr float MAX_VX = 0.5f;     // m/s forward
    constexpr float MAX_VY = 0.3f;     // m/s lateral
    constexpr float MAX_OMEGA = 0.5f;  // rad/s rotation
}

class LocoController {
public:
    LocoController();
    ~LocoController();

    // Initialize SDK LocoClient connection
    // network_interface: empty string to auto-detect, or specify like "eth0"
    // Returns true on success, false on failure
    bool init(const std::string& network_interface = "");

    // Motion control with safety limits
    // Returns true if command was sent, false if robot not ready
    bool setVelocity(float vx, float vy, float omega);

    // Stop all motion
    void stop();

    // Posture control
    bool standUp();
    bool sitDown();

    // Emergency stop - immediate halt (zero torque / damp mode)
    // Returns within 500ms
    void emergencyStop();

    // Gesture commands (G1 high-level arm control)
    // These use predefined motion sequences in the robot firmware
    // Robot should be standing for best results
    bool waveHand(bool leftHand = false);   // Wave hand gesture
    bool shakeHand(int stage = -1);          // Handshake gesture (-1 = full sequence)

    // State queries
    bool isReady() const;
    int getState() const;  // Returns FSM state ID
    bool isConnected() const;

    // Safe velocity command with exception handling
    bool setVelocitySafe(float vx, float vy, float omega);

private:
    std::atomic<bool> connected_{false};
    std::atomic<int> fsm_state_{G1FSM::INVALID};
    std::string network_interface_;
    mutable std::mutex command_mutex_;

    // Check if current FSM state allows motion commands
    bool canSendMotionCommand() const;

    // Apply safety limits to velocity
    void clampVelocity(float& vx, float& vy, float& omega) const;

#ifdef HAS_UNITREE_SDK2
    class Impl;
    std::unique_ptr<Impl> impl_;
#endif
};
