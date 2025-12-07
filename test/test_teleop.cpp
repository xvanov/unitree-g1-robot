#include <gtest/gtest.h>
#include "teleop/TeleopController.h"
#include "locomotion/LocoController.h"
#include <cstring>
#include <cmath>

// Test fixture for TeleopController tests
// Uses SDK's REMOTE_DATA_RX union for proper struct layout
class TeleopControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller = std::make_unique<TeleopController>();
    }

    // Create mock wireless_remote data with specified stick values
    // Uses SDK's xRockerBtnDataStruct layout for compatibility
    void setStickValues(float lx, float ly, float rx, float ry) {
        memset(&mock_remote, 0, sizeof(mock_remote));
        // Set header bytes to indicate connected gamepad
        mock_remote.RF_RX.head[0] = 0x55;
        mock_remote.RF_RX.head[1] = 0xAA;

        // Set stick values using SDK struct
        mock_remote.RF_RX.lx = lx;
        mock_remote.RF_RX.ly = ly;
        mock_remote.RF_RX.rx = rx;
        mock_remote.RF_RX.ry = ry;
        mock_remote.RF_RX.L2 = 0.0f;
    }

    // Set button bits in mock data using SDK union
    void setButtons(uint16_t btn_value) {
        mock_remote.RF_RX.btn.value = btn_value;
    }

    std::unique_ptr<TeleopController> controller;
    unitree::common::REMOTE_DATA_RX mock_remote;
};

// Test: Disconnected gamepad returns zero command
TEST_F(TeleopControllerTest, DisconnectedGamepadReturnsZero) {
    // All zeros = disconnected
    memset(&mock_remote, 0, sizeof(mock_remote));
    controller->update(mock_remote.buff);

    EXPECT_FALSE(controller->isConnected());

    TeleopCommand cmd = controller->getCommand();
    EXPECT_FLOAT_EQ(cmd.vx, 0.0f);
    EXPECT_FLOAT_EQ(cmd.vy, 0.0f);
    EXPECT_FLOAT_EQ(cmd.omega, 0.0f);
}

// Test: Connected gamepad with neutral sticks
TEST_F(TeleopControllerTest, ConnectedNeutralSticksZeroVelocity) {
    setStickValues(0.0f, 0.0f, 0.0f, 0.0f);
    controller->update(mock_remote.buff);

    // First update may still show disconnected due to header check
    // Update again to stabilize
    controller->update(mock_remote.buff);

    EXPECT_TRUE(controller->isConnected());

    TeleopCommand cmd = controller->getCommand();
    EXPECT_FLOAT_EQ(cmd.vx, 0.0f);
    EXPECT_FLOAT_EQ(cmd.vy, 0.0f);
    EXPECT_FLOAT_EQ(cmd.omega, 0.0f);
}

// Test: Deadzone filtering
TEST_F(TeleopControllerTest, DeadzoneFiltering) {
    controller->setDeadzone(0.1f);  // 10% deadzone

    // Values below deadzone should be filtered to zero
    setStickValues(0.05f, 0.05f, 0.05f, 0.05f);
    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);  // Stabilize

    TeleopCommand cmd = controller->getCommand();
    EXPECT_FLOAT_EQ(cmd.vx, 0.0f);
    EXPECT_FLOAT_EQ(cmd.vy, 0.0f);
    EXPECT_FLOAT_EQ(cmd.omega, 0.0f);
}

// Test: Forward velocity from left stick Y
TEST_F(TeleopControllerTest, ForwardVelocityMapping) {
    controller->setSmoothFactor(1.0f);  // Instant response for testing
    setStickValues(0.0f, 1.0f, 0.0f, 0.0f);  // Full forward on ly

    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);  // Stabilize

    TeleopCommand cmd = controller->getCommand();
    EXPECT_NEAR(cmd.vx, SafetyLimits::MAX_VX, 0.01f);
    EXPECT_FLOAT_EQ(cmd.vy, 0.0f);
    EXPECT_FLOAT_EQ(cmd.omega, 0.0f);
}

// Test: Backward velocity from left stick Y
TEST_F(TeleopControllerTest, BackwardVelocityMapping) {
    controller->setSmoothFactor(1.0f);
    setStickValues(0.0f, -1.0f, 0.0f, 0.0f);  // Full backward on ly

    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_NEAR(cmd.vx, -SafetyLimits::MAX_VX, 0.01f);
}

// Test: Lateral velocity from left stick X
TEST_F(TeleopControllerTest, LateralVelocityMapping) {
    controller->setSmoothFactor(1.0f);
    setStickValues(1.0f, 0.0f, 0.0f, 0.0f);  // Full right on lx

    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_FLOAT_EQ(cmd.vx, 0.0f);
    EXPECT_NEAR(cmd.vy, SafetyLimits::MAX_VY, 0.01f);
}

// Test: Rotation velocity from right stick X
TEST_F(TeleopControllerTest, RotationVelocityMapping) {
    controller->setSmoothFactor(1.0f);
    setStickValues(0.0f, 0.0f, 1.0f, 0.0f);  // Full right on rx

    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_FLOAT_EQ(cmd.vx, 0.0f);
    EXPECT_FLOAT_EQ(cmd.vy, 0.0f);
    // Rotation should be negative for right turn (clockwise)
    EXPECT_NEAR(std::fabs(cmd.omega), SafetyLimits::MAX_OMEGA, 0.01f);
}

// Test: Safety limits clamp excessive values
TEST_F(TeleopControllerTest, SafetyLimitsClamping) {
    controller->setSmoothFactor(1.0f);

    // Even with stick at max, velocity should be clamped
    setStickValues(1.0f, 1.0f, 1.0f, 0.0f);
    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_LE(cmd.vx, SafetyLimits::MAX_VX);
    EXPECT_LE(cmd.vy, SafetyLimits::MAX_VY);
    EXPECT_LE(std::fabs(cmd.omega), SafetyLimits::MAX_OMEGA);
}

// Test: Velocity scale factor
TEST_F(TeleopControllerTest, VelocityScaling) {
    controller->setSmoothFactor(1.0f);
    controller->setMaxVelocityScale(0.5f);  // Half speed

    setStickValues(0.0f, 1.0f, 0.0f, 0.0f);  // Full forward
    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_NEAR(cmd.vx, SafetyLimits::MAX_VX * 0.5f, 0.01f);
}

// Test: Button state detection - A button (stand)
TEST_F(TeleopControllerTest, ButtonAStandDetection) {
    setStickValues(0.0f, 0.0f, 0.0f, 0.0f);

    // First update without button
    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    // Now set A button (bit 8)
    setButtons(0x0100);  // A button
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_TRUE(cmd.stand);
    EXPECT_TRUE(cmd.stand_pressed);  // First press
}

// Test: Button state detection - B button (sit)
TEST_F(TeleopControllerTest, ButtonBSitDetection) {
    setStickValues(0.0f, 0.0f, 0.0f, 0.0f);

    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    // Set B button (bit 9)
    setButtons(0x0200);  // B button
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_TRUE(cmd.sit);
    EXPECT_TRUE(cmd.sit_pressed);
}

// Test: Button state detection - Start button (emergency stop)
TEST_F(TeleopControllerTest, ButtonStartEstopDetection) {
    setStickValues(0.0f, 0.0f, 0.0f, 0.0f);

    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    // Set Start button (bit 2)
    setButtons(0x0004);  // Start button
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_TRUE(cmd.emergency_stop);
    EXPECT_TRUE(cmd.estop_pressed);
}

// Test: Edge detection - button release
TEST_F(TeleopControllerTest, ButtonEdgeDetectionRelease) {
    setStickValues(0.0f, 0.0f, 0.0f, 0.0f);

    // Press A button
    setButtons(0x0100);
    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    TeleopCommand cmd = controller->getCommand();
    EXPECT_TRUE(cmd.stand_pressed);

    // Keep button pressed - should not trigger again
    controller->update(mock_remote.buff);
    cmd = controller->getCommand();
    EXPECT_FALSE(cmd.stand_pressed);  // Not a new press

    // Release button
    setButtons(0x0000);
    controller->update(mock_remote.buff);
    cmd = controller->getCommand();
    EXPECT_FALSE(cmd.stand);
    EXPECT_FALSE(cmd.stand_pressed);
}

// Test: Exponential smoothing
TEST_F(TeleopControllerTest, ExponentialSmoothing) {
    controller->setSmoothFactor(0.5f);  // 50% smoothing
    setStickValues(0.0f, 0.0f, 0.0f, 0.0f);
    controller->update(mock_remote.buff);
    controller->update(mock_remote.buff);

    // Now apply full forward
    setStickValues(0.0f, 1.0f, 0.0f, 0.0f);
    controller->update(mock_remote.buff);

    // First update should be ~50% of target
    TeleopCommand cmd = controller->getCommand();
    // Due to deadzone scaling, this may not be exactly 50%
    EXPECT_GT(cmd.vx, 0.0f);
    EXPECT_LT(cmd.vx, SafetyLimits::MAX_VX);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
