#pragma once

#include "util/Types.h"

// MVP Localizer: Simple odometry pass-through
// No scan matching for MVP (defer to post-MVP if needed)
class Localizer {
public:
    Localizer() = default;

    // Store pose from odometry
    void setOdometry(const Pose2D& pose) { pose_ = pose; }

    // Return stored pose
    Pose2D getPose() const { return pose_; }

private:
    Pose2D pose_;
};
