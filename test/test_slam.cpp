#include <gtest/gtest.h>
#include <cmath>
#include "slam/OccupancyGrid.h"
#include "slam/GridMapper.h"
#include "slam/Localizer.h"

// Test log-odds conversion functions
TEST(OccupancyGrid, LogOddsToProb) {
    // l=0 -> p=0.5
    EXPECT_NEAR(logOddsToProb(0.0f), 0.5f, 0.001f);

    // l>0 -> p>0.5 (occupied)
    EXPECT_GT(logOddsToProb(1.0f), 0.5f);
    EXPECT_GT(logOddsToProb(5.0f), 0.99f);

    // l<0 -> p<0.5 (free)
    EXPECT_LT(logOddsToProb(-1.0f), 0.5f);
    EXPECT_LT(logOddsToProb(-5.0f), 0.01f);
}

TEST(OccupancyGrid, ProbToLogOdds) {
    // Round-trip test
    float p = 0.7f;
    float l = probToLogOdds(p);
    EXPECT_NEAR(logOddsToProb(l), p, 0.001f);

    // p=0.5 -> l=0
    EXPECT_NEAR(probToLogOdds(0.5f), 0.0f, 0.001f);
}

TEST(OccupancyGrid, LogOddsToPng) {
    // l=0 (unknown) -> gray (127)
    uint8_t gray = logOddsToPng(0.0f);
    EXPECT_NEAR(gray, 127, 2);

    // l>0 (occupied) -> dark (low value)
    uint8_t dark = logOddsToPng(5.0f);
    EXPECT_LT(dark, 50);

    // l<0 (free) -> light (high value)
    uint8_t light = logOddsToPng(-5.0f);
    EXPECT_GT(light, 200);
}

// Test GridMapper construction
TEST(GridMapper, Construction) {
    GridMapper mapper(0.05f, 100, 100);

    EXPECT_EQ(mapper.getWidth(), 100);
    EXPECT_EQ(mapper.getHeight(), 100);
    EXPECT_FLOAT_EQ(mapper.getResolution(), 0.05f);

    // Initial log-odds should all be 0.0f
    const auto& log_odds = mapper.getLogOddsMap();
    EXPECT_EQ(log_odds.size(), 10000u);
    for (float l : log_odds) {
        EXPECT_FLOAT_EQ(l, 0.0f);
    }
}

// Test Bresenham ray tracing - verify cells are updated correctly
TEST(GridMapper, RayTracing) {
    GridMapper mapper(0.1f, 50, 50);  // 5m x 5m map at 0.1m resolution

    // Create a simple scan - single ray from center pointing right
    Pose2D pose{2.5f, 2.5f, 0.0f};  // Center of map, facing right
    LidarScan scan;
    scan.angle_min = 0.0f;
    scan.angle_max = 0.0f;  // Single angle
    scan.ranges.push_back(1.0f);  // 1 meter range

    mapper.update(pose, scan);

    const auto& log_odds = mapper.getLogOddsMap();

    // Robot position (25, 25) should be marked free (ray passes through)
    int robot_idx = 25 * 50 + 25;
    EXPECT_LT(log_odds[robot_idx], 0.0f);  // Free

    // Endpoint at (35, 25) should be marked occupied
    // 1m at 0.1m resolution = 10 cells from robot
    int endpoint_idx = 25 * 50 + 35;
    EXPECT_GT(log_odds[endpoint_idx], 0.0f);  // Occupied
}

// Test update with multiple rays
TEST(GridMapper, MultipleRays) {
    GridMapper mapper(0.1f, 50, 50);

    // 4 rays in cardinal directions
    Pose2D pose{2.5f, 2.5f, 0.0f};
    LidarScan scan;
    scan.angle_min = 0.0f;
    scan.angle_max = 2.0f * M_PI;
    scan.ranges = {1.0f, 1.0f, 1.0f, 1.0f};  // 4 rays, each 1m

    mapper.update(pose, scan);

    const auto& log_odds = mapper.getLogOddsMap();

    // Count cells that changed from initial (0.0f)
    int changed = 0;
    for (float l : log_odds) {
        if (std::abs(l) > 0.001f) changed++;
    }

    // Should have updated multiple cells (rays + endpoints)
    EXPECT_GT(changed, 20);
}

// Test log-odds clamping
TEST(GridMapper, LogOddsClamping) {
    GridMapper mapper(0.1f, 20, 20);

    // Apply many updates to same location to test clamping
    Pose2D pose{1.0f, 1.0f, 0.0f};
    LidarScan scan;
    scan.angle_min = 0.0f;
    scan.angle_max = 0.0f;
    scan.ranges = {0.5f};  // Short range, endpoint close to robot

    // Update many times
    for (int i = 0; i < 100; ++i) {
        mapper.update(pose, scan);
    }

    const auto& log_odds = mapper.getLogOddsMap();

    // All values should be within [-5, 5]
    for (float l : log_odds) {
        EXPECT_GE(l, -5.0f);
        EXPECT_LE(l, 5.0f);
    }
}

// Test accuracy calculation
TEST(GridMapper, AccuracyCalculation) {
    GridMapper mapper(0.1f, 10, 10);  // Small map for testing

    // Create ground truth: top half free (white=255), bottom half obstacle (black=0)
    std::vector<uint8_t> ground_truth(100);
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10; ++x) {
            ground_truth[y * 10 + x] = (y < 5) ? 255 : 0;  // Top free, bottom obstacle
        }
    }

    // With all zeros (unknown), accuracy should be 0% (no explored cells)
    float acc = mapper.computeAccuracy(ground_truth);
    EXPECT_NEAR(acc, 0.0f, 0.01f);

    // Now update the mapper with some scans to create explored cells
    // Scan in the free area (top half, y < 5)
    Pose2D pose{0.5f, 0.25f, 0.0f};  // In top half
    LidarScan scan;
    scan.angle_min = 0.0f;
    scan.angle_max = 0.0f;
    scan.ranges = {0.3f};  // Short ray in free space

    mapper.update(pose, scan);

    // Now there should be some explored cells, and accuracy should be > 0
    float acc_after = mapper.computeAccuracy(ground_truth);
    EXPECT_GT(acc_after, 0.0f);
}

// Test Localizer (simple odometry pass-through)
TEST(Localizer, OdometryPassThrough) {
    Localizer localizer;

    // Initial pose should be (0,0,0)
    Pose2D initial = localizer.getPose();
    EXPECT_FLOAT_EQ(initial.x, 0.0f);
    EXPECT_FLOAT_EQ(initial.y, 0.0f);
    EXPECT_FLOAT_EQ(initial.theta, 0.0f);

    // Set odometry
    Pose2D new_pose{1.5f, 2.5f, 0.5f};
    localizer.setOdometry(new_pose);

    // Get pose should return exact same values
    Pose2D result = localizer.getPose();
    EXPECT_FLOAT_EQ(result.x, 1.5f);
    EXPECT_FLOAT_EQ(result.y, 2.5f);
    EXPECT_FLOAT_EQ(result.theta, 0.5f);
}

// Test GridMapper handles empty scan gracefully (no division by zero)
TEST(GridMapper, EmptyScan) {
    GridMapper mapper(0.1f, 50, 50);

    Pose2D pose{2.5f, 2.5f, 0.0f};
    LidarScan empty_scan;
    // ranges is empty by default

    // Should not crash or modify map
    mapper.update(pose, empty_scan);

    const auto& log_odds = mapper.getLogOddsMap();

    // All values should still be 0.0f (no updates)
    for (float l : log_odds) {
        EXPECT_FLOAT_EQ(l, 0.0f);
    }
}

// Test GridMapper with scan including invalid ranges
TEST(GridMapper, InvalidRanges) {
    GridMapper mapper(0.1f, 50, 50);

    Pose2D pose{2.5f, 2.5f, 0.0f};
    LidarScan scan;
    scan.angle_min = 0.0f;
    scan.angle_max = 2.0f * M_PI;
    scan.ranges = {
        0.05f,   // Too close (< 0.1), should be skipped
        10.0f,   // Max range (>= 9.9), should be skipped
        1.0f,    // Valid
        1.5f     // Valid
    };

    mapper.update(pose, scan);

    const auto& log_odds = mapper.getLogOddsMap();

    // Count changed cells - should only reflect 2 valid rays
    int changed = 0;
    for (float l : log_odds) {
        if (std::abs(l) > 0.001f) changed++;
    }

    // 2 valid rays should update cells
    EXPECT_GT(changed, 5);
    EXPECT_LT(changed, 100);  // Not too many (only 2 rays)
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
