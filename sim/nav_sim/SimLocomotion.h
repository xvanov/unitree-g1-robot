#pragma once

#include "locomotion/ILocomotion.h"
#include "NavSim.h"

class SimLocomotion : public ILocomotion {
public:
    explicit SimLocomotion(NavSim& sim, float dt = 0.1f);

    void setVelocity(float vx, float vy, float omega) override;
    void stop() override;
    bool isReady() const override;

    // Apply stored velocity command to simulation
    void step();

private:
    NavSim& sim_;
    float dt_;
    Velocity current_cmd_;
    bool ready_;
};
