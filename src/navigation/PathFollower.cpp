#include "navigation/PathFollower.h"
#include <cmath>
#include <algorithm>

PathFollower::PathFollower()
    : current_waypoint_index_(0), complete_(true) {
}

void PathFollower::setPath(const std::vector<Point2D>& path) {
    path_ = path;
    current_waypoint_index_ = 0;
    complete_ = path.empty();
}

Velocity PathFollower::getVelocityCommand(const Pose2D& current_pose) {
    Velocity cmd{0, 0, 0};

    if (path_.empty() || complete_) {
        return cmd;
    }

    // Get goal (last point)
    const Point2D& goal = path_.back();
    float dx_goal = goal.x - current_pose.x;
    float dy_goal = goal.y - current_pose.y;
    float dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    // Check if goal reached
    if (dist_to_goal < GOAL_TOLERANCE) {
        complete_ = true;
        return cmd;
    }

    // Advance waypoint index and get lookahead target
    Point2D target = advanceToLookaheadPoint(current_pose);

    // Calculate direction to target
    float dx = target.x - current_pose.x;
    float dy = target.y - current_pose.y;
    float target_angle = std::atan2(dy, dx);

    // Calculate angular error
    float angle_error = normalizeAngle(target_angle - current_pose.theta);

    // Pure pursuit: angular velocity proportional to angle error
    cmd.omega = std::clamp(2.0f * angle_error, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    // Linear velocity: reduce when turning sharply
    float turn_factor = 1.0f - std::min(static_cast<float>(std::abs(angle_error)) / (static_cast<float>(M_PI) / 2.0f), 1.0f);
    cmd.vx = MAX_LINEAR_VELOCITY * turn_factor;

    // Slow down near goal
    if (dist_to_goal < 1.0f) {
        cmd.vx *= dist_to_goal;
    }

    return cmd;
}

bool PathFollower::isComplete() const {
    return complete_;
}

size_t PathFollower::getCurrentWaypointIndex() const {
    return current_waypoint_index_;
}

Point2D PathFollower::advanceToLookaheadPoint(const Pose2D& current_pose) {
    // Find the closest point on path ahead of current waypoint
    while (current_waypoint_index_ < path_.size() - 1) {
        const Point2D& wp = path_[current_waypoint_index_];
        float dx = wp.x - current_pose.x;
        float dy = wp.y - current_pose.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist > LOOKAHEAD_DISTANCE) {
            break;  // This waypoint is far enough
        }
        current_waypoint_index_++;
    }

    // Search for a point on the path at lookahead distance
    for (size_t i = current_waypoint_index_; i < path_.size(); ++i) {
        const Point2D& wp = path_[i];
        float dx = wp.x - current_pose.x;
        float dy = wp.y - current_pose.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= LOOKAHEAD_DISTANCE) {
            return wp;
        }
    }

    // Return final goal if no lookahead point found
    return path_.back();
}

float PathFollower::normalizeAngle(float angle) const {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}
