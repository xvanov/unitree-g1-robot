#pragma once

#include <vector>
#include <cstddef>
#include "util/Types.h"

class PathFollower {
public:
    PathFollower();

    // Load path to follow
    void setPath(const std::vector<Point2D>& path);

    // Get velocity command toward next waypoint
    Velocity getVelocityCommand(const Pose2D& current_pose);

    // Return true if goal reached
    bool isComplete() const;

    // Get current waypoint index for progress tracking
    size_t getCurrentWaypointIndex() const;

private:
    // Advances current_waypoint_index_ and returns the lookahead target point
    // NOTE: This method mutates current_waypoint_index_ as a side effect
    Point2D advanceToLookaheadPoint(const Pose2D& current_pose);

    // Normalize angle to [-PI, PI]
    float normalizeAngle(float angle) const;

    std::vector<Point2D> path_;
    size_t current_waypoint_index_;
    bool complete_;

    // Parameters
    static constexpr float LOOKAHEAD_DISTANCE = 0.5f;   // meters
    static constexpr float MAX_LINEAR_VELOCITY = 0.3f;  // m/s
    static constexpr float MAX_ANGULAR_VELOCITY = 0.5f; // rad/s
    static constexpr float GOAL_TOLERANCE = 0.2f;       // meters
};
