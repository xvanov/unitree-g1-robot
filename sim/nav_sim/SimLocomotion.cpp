#include "SimLocomotion.h"

SimLocomotion::SimLocomotion(NavSim& sim, float dt)
    : sim_(sim), dt_(dt), current_cmd_{0, 0, 0}, ready_(true) {
}

void SimLocomotion::setVelocity(float vx, float vy, float omega) {
    current_cmd_.vx = vx;
    current_cmd_.vy = vy;
    current_cmd_.omega = omega;
}

void SimLocomotion::stop() {
    current_cmd_ = {0, 0, 0};
}

bool SimLocomotion::isReady() const {
    return ready_;
}

void SimLocomotion::step() {
    sim_.applyVelocity(current_cmd_.vx, current_cmd_.vy, current_cmd_.omega, dt_);
}
