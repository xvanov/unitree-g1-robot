#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <atomic>

#include "replay/SensorReplayer.h"
#include "replay/ReplaySensorSource.h"
#include "replay/ReplayController.h"
#include "recording/SensorRecorder.h"

// Global running flag required by ReplaySensorSource
std::atomic<bool> g_running{true};

// Test fixture that creates a small test recording
class ReplayTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_session_id_ = "test_replay_session";
        test_recording_dir_ = "data/recordings/" + test_session_id_;

        // Clean up any existing test recording
        std::filesystem::remove_all(test_recording_dir_);

        // Create a test recording using SensorRecorder
        createTestRecording();
    }

    void TearDown() override {
        // Clean up test recording
        std::filesystem::remove_all(test_recording_dir_);
    }

    void createTestRecording() {
        SensorRecorder recorder;
        recorder.setOutputDir("data/recordings");

        ASSERT_TRUE(recorder.startRecording(test_session_id_));

        // Record some test data
        for (int i = 0; i < 50; ++i) {
            // LiDAR scan
            LidarScan scan;
            scan.ranges.resize(360, static_cast<float>(i) * 0.1f);
            scan.angle_min = 0.0f;
            scan.angle_max = 6.28318f;
            recorder.recordLidarScan(scan);

            // IMU data
            ImuData imu;
            imu.roll = static_cast<float>(i) * 0.01f;
            imu.pitch = static_cast<float>(i) * 0.02f;
            imu.yaw = static_cast<float>(i) * 0.03f;
            imu.gyro_x = 0.1f;
            imu.gyro_y = 0.2f;
            imu.gyro_z = 0.3f;
            imu.accel_x = 0.0f;
            imu.accel_y = 0.0f;
            imu.accel_z = 9.8f;
            recorder.recordImu(imu);

            // Pose
            Pose2D pose;
            pose.x = static_cast<float>(i) * 0.1f;
            pose.y = static_cast<float>(i) * 0.05f;
            pose.theta = static_cast<float>(i) * 0.01f;
            recorder.recordPose(pose);

            // Sleep to spread timestamps
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        recorder.stopRecording();
        recording_duration_ = recorder.getMetadata().duration_seconds;
    }

    std::string test_session_id_;
    std::string test_recording_dir_;
    float recording_duration_ = 0.0f;
};

// Test: Open valid recording
TEST_F(ReplayTest, OpenValidRecording) {
    replay::SensorReplayer replayer;

    EXPECT_TRUE(replayer.open(test_recording_dir_));
    EXPECT_TRUE(replayer.isOpen());
    EXPECT_GT(replayer.getDuration(), 0.0f);

    replayer.close();
    EXPECT_FALSE(replayer.isOpen());
}

// Test: Open non-existent recording (graceful failure)
TEST_F(ReplayTest, OpenNonExistentRecording) {
    replay::SensorReplayer replayer;

    EXPECT_FALSE(replayer.open("data/recordings/nonexistent_session"));
    EXPECT_FALSE(replayer.isOpen());
}

// Test: Read messages sequentially
TEST_F(ReplayTest, ReadMessages) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    replay::ReplayMessage msg;
    int lidar_count = 0;
    int imu_count = 0;
    int pose_count = 0;

    while (replayer.readNext(msg)) {
        switch (msg.type) {
            case recording::MessageType::LIDAR_SCAN:
                lidar_count++;
                break;
            case recording::MessageType::IMU_DATA:
                imu_count++;
                break;
            case recording::MessageType::POSE:
                pose_count++;
                break;
            default:
                break;
        }
    }

    // We recorded 50 of each type
    EXPECT_EQ(lidar_count, 50);
    EXPECT_EQ(imu_count, 50);
    EXPECT_EQ(pose_count, 50);
}

// Test: Decode LiDAR scan
TEST_F(ReplayTest, DecodeLidarScan) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    replay::ReplayMessage msg;
    bool found = false;

    while (replayer.readNext(msg)) {
        if (msg.type == recording::MessageType::LIDAR_SCAN) {
            replay::DecodedLidarScan decoded;
            ASSERT_TRUE(replay::SensorReplayer::decodeLidarScan(msg, decoded));

            EXPECT_EQ(decoded.scan.ranges.size(), 360u);
            EXPECT_FLOAT_EQ(decoded.scan.angle_min, 0.0f);
            EXPECT_FLOAT_EQ(decoded.scan.angle_max, 6.28318f);
            found = true;
            break;
        }
    }

    EXPECT_TRUE(found);
}

// Test: Decode IMU data
TEST_F(ReplayTest, DecodeImuData) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    replay::ReplayMessage msg;
    bool found = false;

    while (replayer.readNext(msg)) {
        if (msg.type == recording::MessageType::IMU_DATA) {
            replay::DecodedImuData decoded;
            ASSERT_TRUE(replay::SensorReplayer::decodeImuData(msg, decoded));

            // Check gyro values (we recorded 0.1, 0.2, 0.3)
            EXPECT_NEAR(decoded.imu.gyro_x, 0.1f, 0.01f);
            EXPECT_NEAR(decoded.imu.gyro_y, 0.2f, 0.01f);
            EXPECT_NEAR(decoded.imu.gyro_z, 0.3f, 0.01f);
            found = true;
            break;
        }
    }

    EXPECT_TRUE(found);
}

// Test: Decode pose
TEST_F(ReplayTest, DecodePose) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    replay::ReplayMessage msg;
    bool found = false;

    while (replayer.readNext(msg)) {
        if (msg.type == recording::MessageType::POSE) {
            replay::DecodedPose decoded;
            ASSERT_TRUE(replay::SensorReplayer::decodePose(msg, decoded));

            // First pose should be x=0, y=0, theta=0
            EXPECT_GE(decoded.pose.x, 0.0f);
            found = true;
            break;
        }
    }

    EXPECT_TRUE(found);
}

// Test: Seeking
TEST_F(ReplayTest, SeekToPosition) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    // Seek to 50%
    EXPECT_TRUE(replayer.seek(0.5f));

    // Progress should be approximately 50%
    float progress = replayer.getProgress();
    EXPECT_GE(progress, 0.4f);
    EXPECT_LE(progress, 0.6f);

    // Should still be able to read messages
    replay::ReplayMessage msg;
    EXPECT_TRUE(replayer.readNext(msg));
}

// Test: ReplayController basic operations
TEST_F(ReplayTest, ReplayControllerBasic) {
    replay::ReplayController controller;

    // Initial state
    EXPECT_TRUE(controller.isStopped());
    EXPECT_FLOAT_EQ(controller.getSpeed(), 1.0f);
    EXPECT_FALSE(controller.isLoopEnabled());

    // Play
    controller.play();
    EXPECT_TRUE(controller.isPlaying());

    // Pause
    controller.pause();
    EXPECT_TRUE(controller.isPaused());

    // Resume
    controller.resume();
    EXPECT_TRUE(controller.isPlaying());

    // Speed control
    controller.setSpeed(2.0f);
    EXPECT_FLOAT_EQ(controller.getSpeed(), 2.0f);

    // Clamp speed
    controller.setSpeed(10.0f);
    EXPECT_FLOAT_EQ(controller.getSpeed(), 4.0f);

    controller.setSpeed(0.1f);
    EXPECT_FLOAT_EQ(controller.getSpeed(), 0.25f);

    // Loop mode
    controller.setLoop(true);
    EXPECT_TRUE(controller.isLoopEnabled());

    // Stop
    controller.stop();
    EXPECT_TRUE(controller.isStopped());
}

// Test: ReplayController seeking
TEST_F(ReplayTest, ReplayControllerSeek) {
    replay::ReplayController controller;

    controller.play();

    // Seek to 50%
    controller.seekTo(0.5f);
    EXPECT_TRUE(controller.isSeekPending());
    EXPECT_FLOAT_EQ(controller.getSeekTarget(), 0.5f);

    // Clear seek
    controller.clearSeekPending();
    EXPECT_FALSE(controller.isSeekPending());
    EXPECT_TRUE(controller.isPlaying());
}

// Test: ReplaySensorSource implements ISensorSource
TEST_F(ReplayTest, ReplaySensorSourceInterface) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    replay::ReplayController controller;
    replay::ReplaySensorSource source;

    ASSERT_TRUE(source.init(&replayer, &controller));

    // Start playback
    ASSERT_TRUE(source.start());

    // Wait for some messages to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Get sensor data (should not crash, may return default values)
    LidarScan scan = source.getLidarScan();
    ImuData imu = source.getImu();
    Pose2D pose = source.getPose();
    float battery = source.getBatteryPercent();

    // Battery should return 100% (not recorded)
    EXPECT_FLOAT_EQ(battery, 100.0f);

    // After some time, we should have data
    EXPECT_FALSE(scan.ranges.empty());

    source.stop();
}

// Test: Ground truth pose access
TEST_F(ReplayTest, GroundTruthPose) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    replay::ReplayController controller;
    replay::ReplaySensorSource source;

    ASSERT_TRUE(source.init(&replayer, &controller));
    source.setPoseMode(replay::PoseMode::GROUND_TRUTH);

    ASSERT_TRUE(source.start());

    // Wait for data
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    Pose2D pose = source.getPose();
    Pose2D gt_pose = source.getGroundTruthPose();

    // In ground truth mode, both should be equal
    EXPECT_FLOAT_EQ(pose.x, gt_pose.x);
    EXPECT_FLOAT_EQ(pose.y, gt_pose.y);
    EXPECT_FLOAT_EQ(pose.theta, gt_pose.theta);

    source.stop();
}

// Test: Playback timing (real-time delivery)
TEST_F(ReplayTest, PlaybackTiming) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    replay::ReplayController controller;
    replay::ReplaySensorSource source;

    ASSERT_TRUE(source.init(&replayer, &controller));
    controller.setSpeed(4.0f);  // Fast playback for test

    ASSERT_TRUE(source.start());

    // Playback should complete in approximately duration/speed time
    auto start = std::chrono::steady_clock::now();

    while (source.isRunning() && !controller.isFinished()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Timeout after 5 seconds
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::seconds(5)) {
            break;
        }
    }

    auto end = std::chrono::steady_clock::now();
    float elapsed_s = std::chrono::duration<float>(end - start).count();

    // At 4x speed, should complete in roughly duration/4 seconds
    // Allow generous tolerance for test overhead
    float expected_duration = recording_duration_ / 4.0f;
    EXPECT_LT(elapsed_s, expected_duration + 2.0f);

    source.stop();
}

// Test: Corrupted recording handling
TEST_F(ReplayTest, CorruptedRecordingGracefulFailure) {
    // Create a corrupted recording (invalid binary data)
    std::string corrupt_dir = "data/recordings/corrupt_test";
    std::filesystem::create_directories(corrupt_dir);

    // Write invalid metadata
    std::ofstream meta(corrupt_dir + "/metadata.json");
    meta << "{\"session_id\": \"corrupt\", \"duration_seconds\": 1.0}";
    meta.close();

    // Write garbage binary data
    std::ofstream bin(corrupt_dir + "/sensors.bin.zst", std::ios::binary);
    bin << "this is not valid zstd compressed data";
    bin.close();

    replay::SensorReplayer replayer;
    bool opened = replayer.open(corrupt_dir);

    // Opening might succeed (metadata is valid), but reading should fail gracefully
    if (opened) {
        replay::ReplayMessage msg;
        // Should not crash, just fail to read
        replayer.readNext(msg);
    }

    replayer.close();

    // Clean up
    std::filesystem::remove_all(corrupt_dir);
}

// Test: Message counters
TEST_F(ReplayTest, MessageCounters) {
    replay::SensorReplayer replayer;
    ASSERT_TRUE(replayer.open(test_recording_dir_));

    replay::ReplayController controller;
    replay::ReplaySensorSource source;

    ASSERT_TRUE(source.init(&replayer, &controller));
    controller.setSpeed(4.0f);

    ASSERT_TRUE(source.start());

    // Wait for playback to complete or timeout
    auto start = std::chrono::steady_clock::now();
    while (source.isRunning() && !controller.isFinished()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) {
            break;
        }
    }

    source.stop();

    // Check counters
    EXPECT_EQ(controller.getLidarReplayed(), 50u);
    EXPECT_EQ(controller.getImuReplayed(), 50u);
    EXPECT_EQ(controller.getPoseReplayed(), 50u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
