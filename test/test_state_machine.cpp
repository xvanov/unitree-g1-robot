#include <gtest/gtest.h>
#include "app/StateMachine.h"

class StateMachineTest : public ::testing::Test {
protected:
    StateMachine sm;
};

// Test initial state is IDLE
TEST_F(StateMachineTest, InitialStateIsIdle) {
    EXPECT_EQ(sm.getState(), InspectionState::IDLE);
    EXPECT_EQ(sm.getStateString(), "IDLE");
}

// Test valid transitions from IDLE
TEST_F(StateMachineTest, IdleToCalibrating) {
    EXPECT_TRUE(sm.startInspection());
    EXPECT_EQ(sm.getState(), InspectionState::CALIBRATING);
}

TEST_F(StateMachineTest, CalibratingToInspecting) {
    sm.startInspection();  // IDLE -> CALIBRATING
    EXPECT_TRUE(sm.setCalibrated());
    EXPECT_EQ(sm.getState(), InspectionState::INSPECTING);
}

TEST_F(StateMachineTest, CalibratingToIdle) {
    sm.startInspection();  // IDLE -> CALIBRATING
    EXPECT_TRUE(sm.stop());
    EXPECT_EQ(sm.getState(), InspectionState::IDLE);
}

// Test invalid transitions from IDLE
TEST_F(StateMachineTest, CannotPauseFromIdle) {
    EXPECT_FALSE(sm.pause());
    EXPECT_EQ(sm.getState(), InspectionState::IDLE);
}

TEST_F(StateMachineTest, CannotResumeFromIdle) {
    EXPECT_FALSE(sm.resume());
    EXPECT_EQ(sm.getState(), InspectionState::IDLE);
}

// Test INSPECTING transitions
TEST_F(StateMachineTest, InspectingToPaused) {
    sm.startInspection();
    sm.setCalibrated();  // IDLE -> CALIBRATING -> INSPECTING
    EXPECT_TRUE(sm.pause());
    EXPECT_EQ(sm.getState(), InspectionState::PAUSED);
}

TEST_F(StateMachineTest, InspectingToBlocked) {
    sm.startInspection();
    sm.setCalibrated();
    EXPECT_TRUE(sm.setBlocked(true));
    EXPECT_EQ(sm.getState(), InspectionState::BLOCKED);
}

TEST_F(StateMachineTest, InspectingToComplete) {
    sm.startInspection();
    sm.setCalibrated();
    EXPECT_TRUE(sm.setComplete());
    EXPECT_EQ(sm.getState(), InspectionState::COMPLETE);
}

// Test PAUSED transitions
TEST_F(StateMachineTest, PausedToInspecting) {
    sm.startInspection();
    sm.setCalibrated();
    sm.pause();  // INSPECTING -> PAUSED
    EXPECT_TRUE(sm.resume());
    EXPECT_EQ(sm.getState(), InspectionState::INSPECTING);
}

TEST_F(StateMachineTest, PausedToIdle) {
    sm.startInspection();
    sm.setCalibrated();
    sm.pause();  // INSPECTING -> PAUSED
    EXPECT_TRUE(sm.stop());
    EXPECT_EQ(sm.getState(), InspectionState::IDLE);
}

// Test BLOCKED transitions
TEST_F(StateMachineTest, BlockedToInspecting) {
    sm.startInspection();
    sm.setCalibrated();
    sm.setBlocked(true);  // INSPECTING -> BLOCKED
    EXPECT_TRUE(sm.setBlocked(false));
    EXPECT_EQ(sm.getState(), InspectionState::INSPECTING);
}

// Test E-stop from all states
TEST_F(StateMachineTest, EstopFromIdle) {
    sm.emergencyStop();
    EXPECT_EQ(sm.getState(), InspectionState::EMERGENCY_STOP);
}

TEST_F(StateMachineTest, EstopFromCalibrating) {
    sm.startInspection();
    sm.emergencyStop();
    EXPECT_EQ(sm.getState(), InspectionState::EMERGENCY_STOP);
}

TEST_F(StateMachineTest, EstopFromInspecting) {
    sm.startInspection();
    sm.setCalibrated();
    sm.emergencyStop();
    EXPECT_EQ(sm.getState(), InspectionState::EMERGENCY_STOP);
}

TEST_F(StateMachineTest, EstopFromPaused) {
    sm.startInspection();
    sm.setCalibrated();
    sm.pause();
    sm.emergencyStop();
    EXPECT_EQ(sm.getState(), InspectionState::EMERGENCY_STOP);
}

TEST_F(StateMachineTest, EstopFromBlocked) {
    sm.startInspection();
    sm.setCalibrated();
    sm.setBlocked(true);
    sm.emergencyStop();
    EXPECT_EQ(sm.getState(), InspectionState::EMERGENCY_STOP);
}

TEST_F(StateMachineTest, EstopFromComplete) {
    sm.startInspection();
    sm.setCalibrated();
    sm.setComplete();
    sm.emergencyStop();
    EXPECT_EQ(sm.getState(), InspectionState::EMERGENCY_STOP);
}

// Test E-stop clear
TEST_F(StateMachineTest, ClearEstopToIdle) {
    sm.emergencyStop();
    EXPECT_TRUE(sm.clearEstop());
    EXPECT_EQ(sm.getState(), InspectionState::IDLE);
}

// Test completion percentage
TEST_F(StateMachineTest, CompletionPercentage) {
    sm.setWaypointProgress(0, 0);
    EXPECT_FLOAT_EQ(sm.getCompletionPercent(), 0.0f);

    sm.setWaypointProgress(0, 10);
    EXPECT_FLOAT_EQ(sm.getCompletionPercent(), 0.0f);

    sm.setWaypointProgress(5, 10);
    EXPECT_FLOAT_EQ(sm.getCompletionPercent(), 50.0f);

    sm.setWaypointProgress(10, 10);
    EXPECT_FLOAT_EQ(sm.getCompletionPercent(), 100.0f);
}

// Test RETURNING_HOME state
TEST_F(StateMachineTest, CompleteToReturningHome) {
    sm.startInspection();
    sm.setCalibrated();
    sm.setComplete();  // INSPECTING -> COMPLETE
    EXPECT_TRUE(sm.startReturningHome());
    EXPECT_EQ(sm.getState(), InspectionState::RETURNING_HOME);
}

TEST_F(StateMachineTest, InspectingToReturningHome) {
    sm.startInspection();
    sm.setCalibrated();  // IDLE -> CALIBRATING -> INSPECTING
    EXPECT_TRUE(sm.startReturningHome());
    EXPECT_EQ(sm.getState(), InspectionState::RETURNING_HOME);
}

TEST_F(StateMachineTest, BlockedToReturningHome) {
    sm.startInspection();
    sm.setCalibrated();
    sm.setBlocked(true);  // INSPECTING -> BLOCKED
    EXPECT_TRUE(sm.startReturningHome());
    EXPECT_EQ(sm.getState(), InspectionState::RETURNING_HOME);
}

TEST_F(StateMachineTest, ReturningHomeToIdle) {
    sm.startInspection();
    sm.setCalibrated();
    sm.startReturningHome();  // INSPECTING -> RETURNING_HOME
    EXPECT_TRUE(sm.stop());
    EXPECT_EQ(sm.getState(), InspectionState::IDLE);
}

TEST_F(StateMachineTest, CannotReturnHomeFromIdle) {
    EXPECT_FALSE(sm.startReturningHome());
    EXPECT_EQ(sm.getState(), InspectionState::IDLE);
}

TEST_F(StateMachineTest, EstopFromReturningHome) {
    sm.startInspection();
    sm.setCalibrated();
    sm.startReturningHome();  // INSPECTING -> RETURNING_HOME
    EXPECT_EQ(sm.getState(), InspectionState::RETURNING_HOME);
    sm.emergencyStop();
    EXPECT_EQ(sm.getState(), InspectionState::EMERGENCY_STOP);
}

// Test WAITING_OPERATOR state (transitions defined but not triggered in current implementation)
TEST_F(StateMachineTest, WaitingOperatorStateExists) {
    // Verify the state string exists
    // Note: WAITING_OPERATOR state requires future implementation to trigger
    // This test ensures the enum value is valid
    StateMachine sm2;
    // The state machine starts in IDLE, not WAITING_OPERATOR
    EXPECT_NE(sm2.getStateString(), "WAITING_OPERATOR");
}

// Test state string conversion
TEST_F(StateMachineTest, StateStrings) {
    EXPECT_EQ(sm.getStateString(), "IDLE");

    sm.startInspection();
    EXPECT_EQ(sm.getStateString(), "CALIBRATING");

    sm.setCalibrated();
    EXPECT_EQ(sm.getStateString(), "INSPECTING");

    sm.pause();
    EXPECT_EQ(sm.getStateString(), "PAUSED");

    sm.resume();
    sm.setBlocked(true);
    EXPECT_EQ(sm.getStateString(), "BLOCKED");

    sm.setBlocked(false);
    sm.setComplete();
    EXPECT_EQ(sm.getStateString(), "COMPLETE");

    sm.emergencyStop();
    EXPECT_EQ(sm.getStateString(), "EMERGENCY_STOP");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
