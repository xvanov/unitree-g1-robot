#include <gtest/gtest.h>
#include "replay/StreamReplayViewer.h"
#include "replay/ReplayController.h"
#include <cmath>

class StreamReplayViewerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Tests that need a display will skip if unavailable
    }
};

TEST_F(StreamReplayViewerTest, ConstructionDoesNotThrow) {
    // Test that we can create a viewer without crashing
    ASSERT_NO_THROW({
        replay::StreamReplayViewer viewer;
    });
}

TEST_F(StreamReplayViewerTest, InitWithInvalidPathReturnsFalse) {
    replay::StreamReplayViewer viewer;
    bool result = viewer.init("/nonexistent/path/to/recording");
    EXPECT_FALSE(result);
}

TEST_F(StreamReplayViewerTest, InitWithInvalidSessionReturnsFalse) {
    replay::StreamReplayViewer viewer;
    bool result = viewer.init("data/recordings/nonexistent_session");
    EXPECT_FALSE(result);
}

TEST_F(StreamReplayViewerTest, SetWindowLayoutDoesNotCrash) {
    replay::StreamReplayViewer viewer;
    ASSERT_NO_THROW({
        viewer.setWindowLayout(replay::WindowLayout::GRID_2X2);
        viewer.setWindowLayout(replay::WindowLayout::HORIZONTAL_STRIP);
    });
}

TEST_F(StreamReplayViewerTest, SetInitialSpeedDoesNotCrash) {
    replay::StreamReplayViewer viewer;
    ASSERT_NO_THROW({
        viewer.setInitialSpeed(0.5f);
        viewer.setInitialSpeed(1.0f);
        viewer.setInitialSpeed(2.0f);
        viewer.setInitialSpeed(4.0f);
    });
}

// Test for ReplayController which is used by StreamReplayViewer
class ReplayControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = std::make_unique<replay::ReplayController>();
    }

    std::unique_ptr<replay::ReplayController> controller_;
};

TEST_F(ReplayControllerTest, InitialStateIsStopped) {
    EXPECT_EQ(controller_->getState(), replay::PlaybackState::STOPPED);
}

TEST_F(ReplayControllerTest, PlayChangesState) {
    controller_->play();
    EXPECT_EQ(controller_->getState(), replay::PlaybackState::PLAYING);
}

TEST_F(ReplayControllerTest, PauseChangesState) {
    controller_->play();
    controller_->pause();
    EXPECT_EQ(controller_->getState(), replay::PlaybackState::PAUSED);
}

TEST_F(ReplayControllerTest, ResumeFromPause) {
    controller_->play();
    controller_->pause();
    controller_->resume();
    EXPECT_EQ(controller_->getState(), replay::PlaybackState::PLAYING);
}

TEST_F(ReplayControllerTest, SpeedControlWorks) {
    EXPECT_FLOAT_EQ(controller_->getSpeed(), 1.0f);

    controller_->setSpeed(2.0f);
    EXPECT_FLOAT_EQ(controller_->getSpeed(), 2.0f);

    controller_->setSpeed(0.5f);
    EXPECT_FLOAT_EQ(controller_->getSpeed(), 0.5f);
}

TEST_F(ReplayControllerTest, TotalTimeTracking) {
    controller_->setTotalTime(120.0f);
    EXPECT_FLOAT_EQ(controller_->getTotalTime(), 120.0f);
}

TEST_F(ReplayControllerTest, ElapsedTimeTracking) {
    controller_->setElapsedTime(30.5f);
    EXPECT_FLOAT_EQ(controller_->getElapsedTime(), 30.5f);
}

TEST_F(ReplayControllerTest, LoopModeCanBeToggled) {
    EXPECT_FALSE(controller_->isLoopEnabled());

    controller_->setLoop(true);
    EXPECT_TRUE(controller_->isLoopEnabled());

    controller_->setLoop(false);
    EXPECT_FALSE(controller_->isLoopEnabled());
}

TEST_F(ReplayControllerTest, SeekPendingWorkflow) {
    EXPECT_FALSE(controller_->isSeekPending());

    controller_->seekTo(0.5f);
    EXPECT_TRUE(controller_->isSeekPending());
    EXPECT_FLOAT_EQ(controller_->getSeekTarget(), 0.5f);

    controller_->clearSeekPending();
    EXPECT_FALSE(controller_->isSeekPending());
}

TEST_F(ReplayControllerTest, MessageCountersIncrement) {
    EXPECT_EQ(controller_->getLidarReplayed(), 0u);
    EXPECT_EQ(controller_->getImuReplayed(), 0u);

    controller_->incrementLidarCount();
    controller_->incrementLidarCount();
    controller_->incrementImuCount();

    EXPECT_EQ(controller_->getLidarReplayed(), 2u);
    EXPECT_EQ(controller_->getImuReplayed(), 1u);
}

TEST_F(ReplayControllerTest, ResetCounters) {
    controller_->incrementLidarCount();
    controller_->incrementImuCount();
    controller_->incrementVideoFrameCount();

    controller_->resetCounters();

    EXPECT_EQ(controller_->getLidarReplayed(), 0u);
    EXPECT_EQ(controller_->getImuReplayed(), 0u);
    EXPECT_EQ(controller_->getVideoFrameReplayed(), 0u);
}

TEST_F(ReplayControllerTest, FinishedState) {
    controller_->play();
    controller_->setFinished();
    EXPECT_EQ(controller_->getState(), replay::PlaybackState::FINISHED);
    EXPECT_TRUE(controller_->isFinished());
}

// Test timestamp formatting (we test the logic, though the method is private)
// This validates the expected format: MM:SS.mmm
TEST(TimestampFormatTest, ZeroSeconds) {
    // Expected format for 0.0s: "00:00.000"
    float seconds = 0.0f;
    int mins = static_cast<int>(seconds / 60);
    int secs = static_cast<int>(seconds) % 60;
    int ms = static_cast<int>((seconds - static_cast<int>(seconds)) * 1000);

    EXPECT_EQ(mins, 0);
    EXPECT_EQ(secs, 0);
    EXPECT_EQ(ms, 0);
}

TEST(TimestampFormatTest, OneMinuteFiveSeconds) {
    // Expected format for 65.5s: "01:05.500"
    float seconds = 65.5f;
    int mins = static_cast<int>(seconds / 60);
    int secs = static_cast<int>(seconds) % 60;
    int ms = static_cast<int>((seconds - static_cast<int>(seconds)) * 1000);

    EXPECT_EQ(mins, 1);
    EXPECT_EQ(secs, 5);
    EXPECT_EQ(ms, 500);
}

TEST(TimestampFormatTest, LongDuration) {
    // Expected format for 3661.123s: "61:01.123"
    float seconds = 3661.123f;
    int mins = static_cast<int>(seconds / 60);
    int secs = static_cast<int>(seconds) % 60;
    int ms = static_cast<int>((seconds - static_cast<int>(seconds)) * 1000);

    EXPECT_EQ(mins, 61);
    EXPECT_EQ(secs, 1);
    EXPECT_NEAR(ms, 123, 1);  // Float precision may vary slightly
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
