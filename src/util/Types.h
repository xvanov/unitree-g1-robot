#pragma once

struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
};

struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;  // radians
};

struct Velocity {
    float vx = 0.0f;     // m/s forward
    float vy = 0.0f;     // m/s lateral
    float omega = 0.0f;  // rad/s rotation
};
