#pragma once

#include "ILocomotion.h"
#include "LocoController.h"
#include <memory>

// Implements ILocomotion interface using real robot hardware via LocoController
class RealLocomotion : public ILocomotion {
public:
    // Initialize with a LocoController instance
    explicit RealLocomotion(std::shared_ptr<LocoController> loco_controller);
    ~RealLocomotion() override = default;

    // ILocomotion interface
    void setVelocity(float vx, float vy, float omega) override;
    void stop() override;
    bool isReady() const override;

    // Additional methods for real locomotion
    bool standUp();
    bool sitDown();
    void emergencyStop();
    int getState() const;

    // Gesture commands (G1 high-level arm control)
    bool waveHand(bool leftHand = false);
    bool shakeHand(int stage = -1);

private:
    std::shared_ptr<LocoController> loco_controller_;
};
