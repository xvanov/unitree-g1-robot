#include "locomotion/LocoController.h"
#include "util/NetworkUtil.h"
#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>

#ifdef HAS_UNITREE_SDK2
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/g1/loco/g1_loco_client.hpp"

class LocoController::Impl {
public:
    unitree::robot::g1::LocoClient loco_client_;
};
#endif

LocoController::LocoController() {
#ifdef HAS_UNITREE_SDK2
    impl_ = std::make_unique<Impl>();
#endif
}

LocoController::~LocoController() = default;

bool LocoController::init(const std::string& network_interface) {
#ifdef HAS_UNITREE_SDK2
    try {
        // Store network interface for reference
        network_interface_ = network_interface.empty() ? NetworkUtil::findRobotInterface() : network_interface;

        // Initialize ChannelFactory using centralized singleton
        // Safe to call multiple times - only initializes once across all SDK users
        if (!NetworkUtil::initChannelFactory(network_interface_)) {
            std::cerr << "[LOCO] Failed to initialize SDK ChannelFactory" << std::endl;
            return false;
        }

        // Initialize LocoClient
        impl_->loco_client_.Init();
        impl_->loco_client_.SetTimeout(10.0f);

        // Wait for valid FSM state
        int fsm_id = 0;
        int retries = 10;
        while (fsm_id == 0 && retries-- > 0) {
            impl_->loco_client_.GetFsmId(fsm_id);
            if (fsm_id == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        if (fsm_id == 0) {
            std::cerr << "[LOCO] Failed to get valid FSM state from robot" << std::endl;
            std::cerr << "[LOCO] Another controller may have exclusive access." << std::endl;
            std::cerr << "[LOCO] Try: sudo systemctl stop sport_mode (if running on robot)" << std::endl;
            std::cerr << "[LOCO] Or restart the robot if issue persists." << std::endl;
            return false;
        }

        fsm_state_.store(fsm_id);
        connected_.store(true);
        std::cout << "[LOCO] Connected, FSM state: " << fsm_id << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[LOCO] Init failed: " << e.what() << std::endl;
        connected_.store(false);
        return false;
    }
#else
    (void)network_interface;
    std::cerr << "[LOCO] unitree_sdk2 not available, LocoController disabled" << std::endl;
    return false;
#endif
}

bool LocoController::setVelocity(float vx, float vy, float omega) {
#ifdef HAS_UNITREE_SDK2
    std::lock_guard<std::mutex> lock(command_mutex_);

    if (!connected_.load()) {
        std::cerr << "[LOCO] Not connected" << std::endl;
        return false;
    }

    // Update FSM state
    int fsm_id;
    impl_->loco_client_.GetFsmId(fsm_id);
    fsm_state_.store(fsm_id);

    if (!canSendMotionCommand()) {
        std::cerr << "[LOCO] Cannot set velocity - FSM state " << fsm_id
                  << " does not allow motion" << std::endl;
        return false;
    }

    // Apply safety limits
    clampVelocity(vx, vy, omega);

    // Send velocity command
    impl_->loco_client_.Move(vx, vy, omega);
    return true;
#else
    (void)vx; (void)vy; (void)omega;
    return false;
#endif
}

void LocoController::stop() {
#ifdef HAS_UNITREE_SDK2
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (connected_.load()) {
        impl_->loco_client_.StopMove();
    }
#endif
}

bool LocoController::standUp() {
#ifdef HAS_UNITREE_SDK2
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!connected_.load()) return false;

    try {
        impl_->loco_client_.StandUp();
        std::cout << "[LOCO] StandUp command sent" << std::endl;

        // Wait briefly and check FSM state
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int fsm_id;
        impl_->loco_client_.GetFsmId(fsm_id);
        fsm_state_.store(fsm_id);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[LOCO] StandUp failed: " << e.what() << std::endl;
        return false;
    }
#else
    return false;
#endif
}

bool LocoController::sitDown() {
#ifdef HAS_UNITREE_SDK2
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!connected_.load()) return false;

    try {
        impl_->loco_client_.Sit();
        std::cout << "[LOCO] Sit command sent" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int fsm_id;
        impl_->loco_client_.GetFsmId(fsm_id);
        fsm_state_.store(fsm_id);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[LOCO] Sit failed: " << e.what() << std::endl;
        return false;
    }
#else
    return false;
#endif
}

void LocoController::emergencyStop() {
#ifdef HAS_UNITREE_SDK2
    std::lock_guard<std::mutex> lock(command_mutex_);

    // Emergency stop must execute within 500ms
    auto start = std::chrono::steady_clock::now();

    try {
        // First stop any motion
        impl_->loco_client_.StopMove();

        // Enter damp mode (passive/safe mode)
        impl_->loco_client_.Damp();

        fsm_state_.store(G1FSM::DAMP);

        auto elapsed = std::chrono::steady_clock::now() - start;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

        // Validate AC5: Emergency stop must complete within 500ms
        constexpr long EMERGENCY_STOP_DEADLINE_MS = 500;
        if (ms <= EMERGENCY_STOP_DEADLINE_MS) {
            std::cout << "[LOCO] Emergency stop completed in " << ms << "ms (OK)" << std::endl;
        } else {
            std::cerr << "[LOCO] WARNING: Emergency stop took " << ms << "ms, exceeds "
                      << EMERGENCY_STOP_DEADLINE_MS << "ms safety requirement!" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[LOCO] Emergency stop exception: " << e.what() << std::endl;
        // Even on exception, try zero torque as last resort
        try {
            impl_->loco_client_.ZeroTorque();
        } catch (...) {}
    }
#endif
}

bool LocoController::waveHand(bool leftHand) {
#ifdef HAS_UNITREE_SDK2
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!connected_.load()) {
        std::cerr << "[LOCO] Not connected" << std::endl;
        return false;
    }

    try {
        // WaveHand uses a predefined gesture in the G1 firmware
        // turn_flag: false = right hand, true = left hand
        std::cout << "[LOCO] Waving " << (leftHand ? "left" : "right") << " hand..." << std::endl;
        impl_->loco_client_.WaveHand(leftHand);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[LOCO] WaveHand failed: " << e.what() << std::endl;
        return false;
    }
#else
    (void)leftHand;
    std::cerr << "[LOCO] unitree_sdk2 not available" << std::endl;
    return false;
#endif
}

bool LocoController::shakeHand(int stage) {
#ifdef HAS_UNITREE_SDK2
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!connected_.load()) {
        std::cerr << "[LOCO] Not connected" << std::endl;
        return false;
    }

    try {
        // ShakeHand uses a predefined handshake gesture
        // stage: -1 = full sequence, or specific stage number
        std::cout << "[LOCO] Performing handshake gesture (stage=" << stage << ")..." << std::endl;
        impl_->loco_client_.ShakeHand(stage);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[LOCO] ShakeHand failed: " << e.what() << std::endl;
        return false;
    }
#else
    (void)stage;
    std::cerr << "[LOCO] unitree_sdk2 not available" << std::endl;
    return false;
#endif
}

bool LocoController::isReady() const {
    return connected_.load() && canSendMotionCommand();
}

int LocoController::getState() const {
    return fsm_state_.load();
}

bool LocoController::isConnected() const {
    return connected_.load();
}

bool LocoController::setVelocitySafe(float vx, float vy, float omega) {
#ifdef HAS_UNITREE_SDK2
    try {
        return setVelocity(vx, vy, omega);
    } catch (const std::exception& e) {
        std::cerr << "[LOCO] Command timeout: " << e.what() << std::endl;
        connected_.store(false);
        return false;
    }
#else
    (void)vx; (void)vy; (void)omega;
    return false;
#endif
}

bool LocoController::canSendMotionCommand() const {
    int state = fsm_state_.load();
    return (state == G1FSM::STANDING || state == G1FSM::WALKING);
}

void LocoController::clampVelocity(float& vx, float& vy, float& omega) const {
    vx = std::clamp(vx, -SafetyLimits::MAX_VX, SafetyLimits::MAX_VX);
    vy = std::clamp(vy, -SafetyLimits::MAX_VY, SafetyLimits::MAX_VY);
    omega = std::clamp(omega, -SafetyLimits::MAX_OMEGA, SafetyLimits::MAX_OMEGA);
}
