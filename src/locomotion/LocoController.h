#pragma once

#include <string>
#include <atomic>
#include <mutex>
#include <memory>

// G1 FSM State IDs (from Unitree G1 SDK - g1_loco_client.hpp and g1_arm_action_error.hpp)
// Note: G1 uses different FSM values than H1!
namespace G1FSM {
    constexpr int ZERO_TORQUE = 0;    // Zero torque mode
    constexpr int DAMP = 1;           // Damp/passive mode (SetFsmId(1))
    constexpr int SQUAT = 2;          // Squat position (SetFsmId(2))
    constexpr int SIT = 3;            // Sitting (SetFsmId(3))
    constexpr int STAND_UP = 4;       // Stand up transition (SetFsmId(4))
    constexpr int START = 500;        // Start/ready for motion (SetFsmId(500))
    constexpr int WALKING = 501;      // Walking mode
    constexpr int AI_MODE = 801;      // AI/advanced locomotion mode (supports motion commands)
}

// G1 Robot Model Variants
// The G1 comes in different configurations with varying DOF counts
namespace G1Model {
    constexpr int EDU_23_DOF = 23;   // Education version - legs only, no arms
    constexpr int STANDARD_29_DOF = 29;  // Standard version - legs + basic arms
    constexpr int FULL_36_DOF = 36;  // Full version - legs + dexterous arms
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
    virtual ~LocoController();

    // Initialize SDK LocoClient connection
    // network_interface: empty string to auto-detect, or specify like "eth0"
    // robot_dof: G1 DOF count (23=EDU, 29=standard, 36=full) - affects available features
    // Returns true on success, false on failure
    virtual bool init(const std::string& network_interface = "", int robot_dof = G1Model::EDU_23_DOF);

    // Motion control with safety limits
    // Returns true if command was sent, false if robot not ready
    virtual bool setVelocity(float vx, float vy, float omega);

    // Stop all motion
    virtual void stop();

    // Posture control
    virtual bool standUp();
    virtual bool sitDown();

    // Emergency stop - immediate halt (zero torque / damp mode)
    // Returns within 500ms
    virtual void emergencyStop();

    // Gesture commands (G1 high-level arm control)
    // These use predefined motion sequences in the robot firmware
    // Robot should be standing for best results
    // NOTE: All G1 models (23/29/36 DOF) have arms and can wave/shake
    //       Only 29+ DOF models have dexterous hand grippers for grip actions
    virtual bool waveHand(bool leftHand = false);   // Wave hand gesture (all models)
    virtual bool shakeHand(int stage = -1);          // Handshake gesture (arm motion on all, grip on 29+)

    // Check if dexterous hand grippers are available (requires 29+ DOF)
    // All models have arms for wave/shake, only 29+ have grip capability
    virtual bool hasArmControl() const { return robot_dof_ >= G1Model::STANDARD_29_DOF; }
    int getRobotDOF() const { return robot_dof_; }

    // State queries
    virtual bool isReady() const;
    int getState() const;  // Returns FSM state ID
    bool isConnected() const;

    // Safe velocity command with exception handling
    bool setVelocitySafe(float vx, float vy, float omega);

protected:
    // Protected constructor for mocking - skips SDK initialization
    struct MockTag {};
    explicit LocoController(MockTag);

private:
    std::atomic<bool> connected_{false};
    std::atomic<int> fsm_state_{G1FSM::ZERO_TORQUE};
    std::string network_interface_;
    int robot_dof_ = G1Model::EDU_23_DOF;  // Robot DOF count (default to EDU)
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
