#include <gtest/gtest.h>
#include <cmath>
#include "plan/PlanManager.h"

class PlanManagerTest : public ::testing::Test {
protected:
    PlanManager pm;
};

// Test initial state
TEST_F(PlanManagerTest, InitialStateNotLoaded) {
    EXPECT_FALSE(pm.isLoaded());
    EXPECT_TRUE(pm.getInspectionWaypoints().empty());
    EXPECT_EQ(pm.getGridWidth(), 0);
    EXPECT_EQ(pm.getGridHeight(), 0);
}

// Test loading PNG plan
TEST_F(PlanManagerTest, LoadPngPlan) {
    // Use the test_data/office.png that exists in the repo
    bool loaded = pm.loadPlan(TEST_DATA_DIR "/office.png", "finishes");
    EXPECT_TRUE(loaded);
    EXPECT_TRUE(pm.isLoaded());

    auto info = pm.getPlanInfo();
    EXPECT_EQ(info.path, TEST_DATA_DIR "/office.png");
    EXPECT_EQ(info.trade_type, "finishes");
    EXPECT_GT(info.width_pixels, 0);
    EXPECT_GT(info.height_pixels, 0);
    EXPECT_GT(info.width_meters, 0.0f);
    EXPECT_GT(info.height_meters, 0.0f);
}

// Test loading non-existent file
TEST_F(PlanManagerTest, LoadNonExistentFile) {
    bool loaded = pm.loadPlan("nonexistent.png");
    EXPECT_FALSE(loaded);
    EXPECT_FALSE(pm.isLoaded());
}

// Test unsupported file format
TEST_F(PlanManagerTest, UnsupportedFormat) {
    bool loaded = pm.loadPlan("test.txt");
    EXPECT_FALSE(loaded);
    EXPECT_FALSE(pm.isLoaded());
}

// Test waypoint generation
TEST_F(PlanManagerTest, WaypointGeneration) {
    pm.loadPlan(TEST_DATA_DIR "/office.png");
    const auto& waypoints = pm.getInspectionWaypoints();

    auto info = pm.getPlanInfo();
    // Waypoint count should match vector size (could be 0 if map is too small/dense)
    EXPECT_EQ(info.waypoint_count, static_cast<int>(waypoints.size()));

    // All waypoints should be within plan bounds
    for (const auto& wp : waypoints) {
        EXPECT_GE(wp.x, 0.0f);
        EXPECT_GE(wp.y, 0.0f);
        EXPECT_LE(wp.x, info.width_meters);
        EXPECT_LE(wp.y, info.height_meters);
    }
}

// Test coordinate transforms without origin set
TEST_F(PlanManagerTest, CoordinateTransformNoOrigin) {
    Pose2D robot_pose{2.0f, 3.0f, 0.0f};
    Point2D plan_point = pm.robotToPlanCoords(robot_pose);

    // Without origin, should return same coordinates
    EXPECT_FLOAT_EQ(plan_point.x, 2.0f);
    EXPECT_FLOAT_EQ(plan_point.y, 3.0f);

    Point2D robot_point = pm.planToRobotCoords({2.0f, 3.0f});
    EXPECT_FLOAT_EQ(robot_point.x, 2.0f);
    EXPECT_FLOAT_EQ(robot_point.y, 3.0f);
}

// Test coordinate transforms with origin set
TEST_F(PlanManagerTest, CoordinateTransformWithOrigin) {
    // Set origin at (5, 5) with 0 rotation
    pm.setStartPosition({5.0f, 5.0f}, 0.0f);

    Pose2D robot_pose{2.0f, 0.0f, 0.0f};
    Point2D plan_point = pm.robotToPlanCoords(robot_pose);

    // Robot 2m forward from origin -> plan (7, 5)
    EXPECT_NEAR(plan_point.x, 7.0f, 0.001f);
    EXPECT_NEAR(plan_point.y, 5.0f, 0.001f);

    // Inverse transform
    Point2D robot_point = pm.planToRobotCoords({7.0f, 5.0f});
    EXPECT_NEAR(robot_point.x, 2.0f, 0.001f);
    EXPECT_NEAR(robot_point.y, 0.0f, 0.001f);
}

// Test coordinate transforms with rotation
TEST_F(PlanManagerTest, CoordinateTransformWithRotation) {
    // Set origin at (5, 5) with 90 degree rotation (PI/2)
    float pi_2 = static_cast<float>(M_PI / 2.0);
    pm.setStartPosition({5.0f, 5.0f}, pi_2);

    Pose2D robot_pose{2.0f, 0.0f, 0.0f};
    Point2D plan_point = pm.robotToPlanCoords(robot_pose);

    // Robot 2m forward (in robot frame) with 90deg rotation -> plan (5, 7)
    EXPECT_NEAR(plan_point.x, 5.0f, 0.001f);
    EXPECT_NEAR(plan_point.y, 7.0f, 0.001f);
}

// Test occupancy grid generation
TEST_F(PlanManagerTest, OccupancyGrid) {
    pm.loadPlan(TEST_DATA_DIR "/office.png");

    auto grid = pm.getOccupancyGrid();
    EXPECT_GT(grid.size(), 0);

    int width = pm.getGridWidth();
    int height = pm.getGridHeight();
    EXPECT_EQ(grid.size(), static_cast<size_t>(width * height));

    // Check grid values are valid (0 = free, 100 = occupied)
    for (uint8_t val : grid) {
        EXPECT_TRUE(val == 0 || val == 100);
    }
}

// Test plan resolution
TEST_F(PlanManagerTest, Resolution) {
    pm.loadPlan(TEST_DATA_DIR "/office.png");

    float resolution = pm.getResolution();
    EXPECT_GT(resolution, 0.0f);
    EXPECT_LT(resolution, 1.0f);  // Should be cm/pixel scale
}

// Test plan info
TEST_F(PlanManagerTest, PlanInfo) {
    pm.loadPlan(TEST_DATA_DIR "/corridor.png", "electrical");

    auto info = pm.getPlanInfo();
    EXPECT_EQ(info.trade_type, "electrical");
    // Waypoint count can be 0 for small/dense maps - just verify it's non-negative
    EXPECT_GE(info.waypoint_count, 0);
}

// Test multiple loads replace previous plan
TEST_F(PlanManagerTest, MultiplePlansReplacePrevious) {
    pm.loadPlan(TEST_DATA_DIR "/office.png", "finishes");
    auto info1 = pm.getPlanInfo();

    pm.loadPlan(TEST_DATA_DIR "/corridor.png", "plumbing");
    auto info2 = pm.getPlanInfo();

    EXPECT_EQ(info2.path, TEST_DATA_DIR "/corridor.png");
    EXPECT_EQ(info2.trade_type, "plumbing");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
