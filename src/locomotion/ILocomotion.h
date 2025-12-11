#pragma once
#include "util/Types.h"

class ILocomotion {
public:
    virtual ~ILocomotion() = default;
    virtual void setVelocity(float vx, float vy, float omega) = 0;
    virtual void stop() = 0;
    virtual bool isReady() const = 0;
};
