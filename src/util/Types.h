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

struct ImuData {
    // Orientation (radians)
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // Angular velocity (rad/s)
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    // Linear acceleration (m/s^2)
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
};
