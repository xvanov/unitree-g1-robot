#include "locomotion/RealLocomotion.h"
#include <iostream>

RealLocomotion::RealLocomotion(std::shared_ptr<LocoController> loco_controller)
    : loco_controller_(std::move(loco_controller))
{
}

void RealLocomotion::setVelocity(float vx, float vy, float omega) {
    if (!loco_controller_->setVelocitySafe(vx, vy, omega)) {
        std::cerr << "[RealLocomotion] Failed to set velocity" << std::endl;
    }
}

void RealLocomotion::stop() {
    loco_controller_->stop();
}

bool RealLocomotion::isReady() const {
    return loco_controller_->isReady();
}

bool RealLocomotion::standUp() {
    return loco_controller_->standUp();
}

bool RealLocomotion::sitDown() {
    return loco_controller_->sitDown();
}

void RealLocomotion::emergencyStop() {
    loco_controller_->emergencyStop();
}

int RealLocomotion::getState() const {
    return loco_controller_->getState();
}

bool RealLocomotion::waveHand(bool leftHand) {
    return loco_controller_->waveHand(leftHand);
}

bool RealLocomotion::shakeHand(int stage) {
    return loco_controller_->shakeHand(stage);
}
