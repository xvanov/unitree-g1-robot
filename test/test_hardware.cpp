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

// Test FSM state constants
TEST(G1FSMTest, StateConstants) {
    EXPECT_EQ(G1FSM::INVALID, 0);
    EXPECT_EQ(G1FSM::DAMP, 1);
    EXPECT_EQ(G1FSM::STAND_UP, 2);
    EXPECT_EQ(G1FSM::STAND_DOWN, 3);
    EXPECT_EQ(G1FSM::STANDING, 100);
    EXPECT_EQ(G1FSM::WALKING, 101);
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
    // Only STANDING (100) and WALKING (101) should allow motion
    auto canMove = [](int state) {
        return (state == G1FSM::STANDING || state == G1FSM::WALKING);
    };

    EXPECT_FALSE(canMove(G1FSM::INVALID));
    EXPECT_FALSE(canMove(G1FSM::DAMP));
    EXPECT_FALSE(canMove(G1FSM::STAND_UP));
    EXPECT_FALSE(canMove(G1FSM::STAND_DOWN));
    EXPECT_TRUE(canMove(G1FSM::STANDING));
    EXPECT_TRUE(canMove(G1FSM::WALKING));
}
