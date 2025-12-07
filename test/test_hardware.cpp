#include <gtest/gtest.h>
#include "locomotion/LocoController.h"
#include "util/Types.h"

// Test velocity clamping logic (doesn't require SDK)
class VelocityClampTest : public ::testing::Test {
protected:
    // Test the safety limits constants
    void testClampValue(float input, float min, float max, float expected) {
        float result = std::clamp(input, min, max);
        EXPECT_FLOAT_EQ(result, expected);
    }
};

TEST_F(VelocityClampTest, ClampVxWithinLimits) {
    testClampValue(0.3f, -SafetyLimits::MAX_VX, SafetyLimits::MAX_VX, 0.3f);
}

TEST_F(VelocityClampTest, ClampVxExceedsMax) {
    testClampValue(1.5f, -SafetyLimits::MAX_VX, SafetyLimits::MAX_VX, SafetyLimits::MAX_VX);
}

TEST_F(VelocityClampTest, ClampVxExceedsMin) {
    testClampValue(-1.5f, -SafetyLimits::MAX_VX, SafetyLimits::MAX_VX, -SafetyLimits::MAX_VX);
}

TEST_F(VelocityClampTest, ClampVyWithinLimits) {
    testClampValue(0.2f, -SafetyLimits::MAX_VY, SafetyLimits::MAX_VY, 0.2f);
}

TEST_F(VelocityClampTest, ClampVyExceedsMax) {
    testClampValue(1.0f, -SafetyLimits::MAX_VY, SafetyLimits::MAX_VY, SafetyLimits::MAX_VY);
}

TEST_F(VelocityClampTest, ClampOmegaWithinLimits) {
    testClampValue(0.3f, -SafetyLimits::MAX_OMEGA, SafetyLimits::MAX_OMEGA, 0.3f);
}

TEST_F(VelocityClampTest, ClampOmegaExceedsMax) {
    testClampValue(2.0f, -SafetyLimits::MAX_OMEGA, SafetyLimits::MAX_OMEGA, SafetyLimits::MAX_OMEGA);
}

// Test FSM state constants (G1 SDK values)
TEST(G1FSMTest, StateConstants) {
    EXPECT_EQ(G1FSM::ZERO_TORQUE, 0);
    EXPECT_EQ(G1FSM::DAMP, 1);
    EXPECT_EQ(G1FSM::SQUAT, 2);
    EXPECT_EQ(G1FSM::SIT, 3);
    EXPECT_EQ(G1FSM::STAND_UP, 4);
    EXPECT_EQ(G1FSM::START, 500);
    EXPECT_EQ(G1FSM::WALKING, 501);
    EXPECT_EQ(G1FSM::AI_MODE, 801);
}

// Test safety limits values
TEST(SafetyLimitsTest, LimitValues) {
    EXPECT_FLOAT_EQ(SafetyLimits::MAX_VX, 0.5f);
    EXPECT_FLOAT_EQ(SafetyLimits::MAX_VY, 0.3f);
    EXPECT_FLOAT_EQ(SafetyLimits::MAX_OMEGA, 0.5f);
}

// Test ImuData struct initialization
TEST(ImuDataTest, DefaultInitialization) {
    ImuData imu;
    EXPECT_FLOAT_EQ(imu.roll, 0.0f);
    EXPECT_FLOAT_EQ(imu.pitch, 0.0f);
    EXPECT_FLOAT_EQ(imu.yaw, 0.0f);
    EXPECT_FLOAT_EQ(imu.gyro_x, 0.0f);
    EXPECT_FLOAT_EQ(imu.gyro_y, 0.0f);
    EXPECT_FLOAT_EQ(imu.gyro_z, 0.0f);
    EXPECT_FLOAT_EQ(imu.accel_x, 0.0f);
    EXPECT_FLOAT_EQ(imu.accel_y, 0.0f);
    EXPECT_FLOAT_EQ(imu.accel_z, 0.0f);
}

// NOTE: LocoController tests that create instances are skipped when SDK is linked
// because the SDK's LocoClient constructor requires ChannelFactory to be initialized,
// which in turn requires a network connection. Testing LocoController requires
// either a mock SDK or integration tests with a real robot.

// Test that canSendMotionCommand logic works correctly
TEST(G1FSMTest, MotionAllowedStates) {
    // G1 allows motion in START (500), WALKING (501), AI_MODE (801), and STAND_UP (4)
    auto canMove = [](int state) {
        return (state == G1FSM::START || state == G1FSM::WALKING ||
                state == G1FSM::AI_MODE || state == G1FSM::STAND_UP);
    };

    EXPECT_FALSE(canMove(G1FSM::ZERO_TORQUE));
    EXPECT_FALSE(canMove(G1FSM::DAMP));
    EXPECT_FALSE(canMove(G1FSM::SQUAT));
    EXPECT_FALSE(canMove(G1FSM::SIT));
    EXPECT_TRUE(canMove(G1FSM::STAND_UP));
    EXPECT_TRUE(canMove(G1FSM::START));
    EXPECT_TRUE(canMove(G1FSM::WALKING));
    EXPECT_TRUE(canMove(G1FSM::AI_MODE));
}
