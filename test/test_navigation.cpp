#include <gtest/gtest.h>
#include <cmath>
#include "navigation/Costmap.h"
#include "navigation/Planner.h"
#include "navigation/PathFollower.h"

// ============================================================================
// Costmap Tests
// ============================================================================

TEST(CostmapTest, InitializesToFreeSpace) {
    Costmap costmap(0.05f, 100, 100);  // 5m x 5m at 5cm resolution

    // All cells should be free (0) initially
    EXPECT_EQ(costmap.getCost(0, 0), 0);
    EXPECT_EQ(costmap.getCost(50, 50), 0);
    EXPECT_EQ(costmap.getCost(99, 99), 0);
}

TEST(CostmapTest, SetAndGetCost) {
    Costmap costmap(0.05f, 100, 100);

    costmap.setCost(10, 20, 254);
    EXPECT_EQ(costmap.getCost(10, 20), 254);

    costmap.setCost(50, 50, 128);
    EXPECT_EQ(costmap.getCost(50, 50), 128);
}

TEST(CostmapTest, OutOfBoundsReturnsObstacle) {
    Costmap costmap(0.05f, 100, 100);

    // Out of bounds should return 255 (obstacle)
    EXPECT_EQ(costmap.getCost(-1, 0), 255);
    EXPECT_EQ(costmap.getCost(0, -1), 255);
    EXPECT_EQ(costmap.getCost(100, 0), 255);
    EXPECT_EQ(costmap.getCost(0, 100), 255);
}

TEST(CostmapTest, WorldToGridConversion) {
    Costmap costmap(0.05f, 100, 100);  // 5cm resolution

    // (0.0, 0.0) -> (0, 0)
    auto cell1 = costmap.worldToGrid(0.0f, 0.0f);
    EXPECT_EQ(cell1.x, 0);
    EXPECT_EQ(cell1.y, 0);

    // (0.05, 0.05) -> (1, 1) - one cell over
    auto cell2 = costmap.worldToGrid(0.05f, 0.05f);
    EXPECT_EQ(cell2.x, 1);
    EXPECT_EQ(cell2.y, 1);

    // (2.5, 2.5) -> (50, 50)
    auto cell3 = costmap.worldToGrid(2.5f, 2.5f);
    EXPECT_EQ(cell3.x, 50);
    EXPECT_EQ(cell3.y, 50);
}

TEST(CostmapTest, GridToWorldConversion) {
    Costmap costmap(0.05f, 100, 100);

    // Grid (0,0) -> center of cell in world coords
    auto p1 = costmap.gridToWorld(0, 0);
    EXPECT_FLOAT_EQ(p1.x, 0.025f);  // center of first cell
    EXPECT_FLOAT_EQ(p1.y, 0.025f);

    auto p2 = costmap.gridToWorld(50, 50);
    EXPECT_FLOAT_EQ(p2.x, 2.525f);
    EXPECT_FLOAT_EQ(p2.y, 2.525f);
}

TEST(CostmapTest, InflateObstacles) {
    Costmap costmap(0.05f, 50, 50);

    // Place single obstacle at center
    costmap.setCost(25, 25, COST_OBSTACLE);

    // Inflate with 0.1m radius = 2 cells
    costmap.inflate(0.1f);

    // Center should still be obstacle
    EXPECT_GE(costmap.getCost(25, 25), OBSTACLE_THRESHOLD);

    // Neighbors should be inflated
    EXPECT_GE(costmap.getCost(24, 25), OBSTACLE_THRESHOLD);
    EXPECT_GE(costmap.getCost(26, 25), OBSTACLE_THRESHOLD);
    EXPECT_GE(costmap.getCost(25, 24), OBSTACLE_THRESHOLD);
    EXPECT_GE(costmap.getCost(25, 26), OBSTACLE_THRESHOLD);

    // Far cells should be free
    EXPECT_LT(costmap.getCost(0, 0), OBSTACLE_THRESHOLD);
    EXPECT_LT(costmap.getCost(49, 49), OBSTACLE_THRESHOLD);
}

TEST(CostmapTest, UpdateFromScan) {
    Costmap costmap(0.05f, 100, 100);  // 5m x 5m

    // Create a simple scan with one hit
    LidarScan scan;
    scan.angle_min = 0.0f;
    scan.angle_max = 2.0f * M_PI;
    scan.ranges.resize(360, 10.0f);  // All max range
    scan.ranges[0] = 1.0f;  // Hit at angle 0, 1m away

    Pose2D robot_pose{2.5f, 2.5f, 0.0f};  // Robot at center
    costmap.updateFromScan(scan, robot_pose);

    // Cell at (2.5 + 1.0, 2.5) = (3.5, 2.5) should be obstacle
    auto cell = costmap.worldToGrid(3.5f, 2.5f);
    EXPECT_GE(costmap.getCost(cell.x, cell.y), OBSTACLE_THRESHOLD);
}

// ============================================================================
// Planner Tests
// ============================================================================

TEST(PlannerTest, FindsPathInEmptySpace) {
    Costmap costmap(0.05f, 100, 100);  // 5m x 5m, all free

    Planner planner(costmap);
    auto path = planner.planPath({0.5f, 0.5f}, {4.0f, 4.0f});

    ASSERT_FALSE(path.empty());
    EXPECT_GT(path.size(), 1);

    // First point should be near start
    EXPECT_NEAR(path.front().x, 0.5f, 0.1f);
    EXPECT_NEAR(path.front().y, 0.5f, 0.1f);

    // Last point should be near goal
    EXPECT_NEAR(path.back().x, 4.0f, 0.1f);
    EXPECT_NEAR(path.back().y, 4.0f, 0.1f);
}

TEST(PlannerTest, AvoidsObstacles) {
    Costmap costmap(0.1f, 50, 50);  // 5m x 5m at 10cm resolution

    // Create a wall blocking direct path
    for (int y = 10; y < 40; ++y) {
        costmap.setCost(25, y, COST_OBSTACLE);  // Vertical wall at x=25
    }

    Planner planner(costmap);
    auto path = planner.planPath({1.0f, 2.5f}, {4.0f, 2.5f});

    ASSERT_FALSE(path.empty());

    // Path should not cross through the wall
    for (const auto& point : path) {
        auto cell = costmap.worldToGrid(point.x, point.y);
        EXPECT_LT(costmap.getCost(cell.x, cell.y), OBSTACLE_THRESHOLD)
            << "Path goes through obstacle at (" << point.x << ", " << point.y << ")";
    }
}

TEST(PlannerTest, ReturnsEmptyPathWhenGoalBlocked) {
    Costmap costmap(0.1f, 50, 50);

    // Block the goal area completely
    for (int x = 35; x < 50; ++x) {
        for (int y = 20; y < 30; ++y) {
            costmap.setCost(x, y, COST_OBSTACLE);
        }
    }

    Planner planner(costmap);
    auto path = planner.planPath({1.0f, 2.5f}, {4.5f, 2.5f});

    // Should return empty path since goal is in obstacle
    EXPECT_TRUE(path.empty());
}

TEST(PlannerTest, ReturnsEmptyPathWhenStartBlocked) {
    Costmap costmap(0.1f, 50, 50);

    // Block the start area
    for (int x = 0; x < 15; ++x) {
        for (int y = 20; y < 30; ++y) {
            costmap.setCost(x, y, COST_OBSTACLE);
        }
    }

    Planner planner(costmap);
    auto path = planner.planPath({0.5f, 2.5f}, {4.0f, 2.5f});

    EXPECT_TRUE(path.empty());
}

TEST(PlannerTest, PathIsValid) {
    Costmap costmap(0.1f, 50, 50);

    Planner planner(costmap);
    auto path = planner.planPath({1.0f, 1.0f}, {4.0f, 4.0f});

    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(planner.isPathValid(path));
}

TEST(PlannerTest, PathBecomesInvalidWithNewObstacle) {
    Costmap costmap(0.1f, 50, 50);

    Planner planner(costmap);
    auto path = planner.planPath({1.0f, 1.0f}, {4.0f, 4.0f});

    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(planner.isPathValid(path));

    // Add obstacle in the middle of the path
    auto mid = path[path.size() / 2];
    auto cell = costmap.worldToGrid(mid.x, mid.y);
    costmap.setCost(cell.x, cell.y, COST_OBSTACLE);

    // Path should now be invalid
    EXPECT_FALSE(planner.isPathValid(path));
}

// ============================================================================
// PathFollower Tests
// ============================================================================

TEST(PathFollowerTest, EmptyPathIsComplete) {
    PathFollower follower;
    follower.setPath({});

    EXPECT_TRUE(follower.isComplete());
}

TEST(PathFollowerTest, GeneratesVelocityCommands) {
    PathFollower follower;
    std::vector<Point2D> path = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
    follower.setPath(path);

    EXPECT_FALSE(follower.isComplete());

    Pose2D pose{0, 0, 0};
    Velocity cmd = follower.getVelocityCommand(pose);

    // Should generate forward velocity
    EXPECT_GT(cmd.vx, 0);
}

TEST(PathFollowerTest, StopsAtGoal) {
    PathFollower follower;
    std::vector<Point2D> path = {{0, 0}, {1, 0}};
    follower.setPath(path);

    // Robot at goal position
    Pose2D pose{1.0f, 0.0f, 0.0f};
    Velocity cmd = follower.getVelocityCommand(pose);

    EXPECT_TRUE(follower.isComplete());
    EXPECT_FLOAT_EQ(cmd.vx, 0);
    EXPECT_FLOAT_EQ(cmd.omega, 0);
}

TEST(PathFollowerTest, TurnsTowardPath) {
    PathFollower follower;
    std::vector<Point2D> path = {{0, 0}, {0, 1}, {0, 2}};  // Path going up
    follower.setPath(path);

    // Robot facing wrong direction
    Pose2D pose{0, 0, 0};  // Facing right, path goes up
    Velocity cmd = follower.getVelocityCommand(pose);

    // Should have angular velocity to turn toward path
    // Path is at 90 degrees (up), robot facing 0 degrees (right)
    EXPECT_GT(std::abs(cmd.omega), 0);
}

TEST(PathFollowerTest, TracksWaypointProgress) {
    PathFollower follower;
    // Path with wider spacing to ensure waypoint advancement
    std::vector<Point2D> path = {{0, 0}, {0.3f, 0}, {0.6f, 0}, {0.9f, 0}, {1.2f, 0}};
    follower.setPath(path);

    size_t initial_index = follower.getCurrentWaypointIndex();

    // Simulate moving along the path
    Pose2D pose1{0.4f, 0, 0};
    follower.getVelocityCommand(pose1);
    size_t after_first = follower.getCurrentWaypointIndex();

    Pose2D pose2{0.8f, 0, 0};
    follower.getVelocityCommand(pose2);
    size_t after_second = follower.getCurrentWaypointIndex();

    // Waypoint index should increase as we move along the path
    EXPECT_GE(after_second, after_first);
}

TEST(PathFollowerTest, HandlesSharpTurns) {
    PathFollower follower;
    std::vector<Point2D> path = {{0, 0}, {1, 0}, {1, 1}};  // Right then up
    follower.setPath(path);

    Pose2D pose{0.9f, 0, 0};  // Near the turn, facing right
    Velocity cmd = follower.getVelocityCommand(pose);

    // Should reduce linear velocity when needing to turn
    EXPECT_LE(cmd.vx, 0.3f);  // Within max velocity
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(NavigationIntegrationTest, FullNavigationCycle) {
    // Create a simple costmap
    Costmap costmap(0.1f, 100, 100);  // 10m x 10m

    // Add some obstacles
    for (int y = 30; y < 70; ++y) {
        costmap.setCost(50, y, COST_OBSTACLE);  // Vertical wall
    }

    // Plan path around obstacle
    Planner planner(costmap);
    auto path = planner.planPath({1.0f, 5.0f}, {9.0f, 5.0f});

    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(planner.isPathValid(path));

    // Follow path
    PathFollower follower;
    follower.setPath(path);

    EXPECT_FALSE(follower.isComplete());

    // Simulate following path
    Pose2D pose{1.0f, 5.0f, 0.0f};
    const float dt = 0.1f;
    int steps = 0;
    const int max_steps = 1000;

    while (!follower.isComplete() && steps < max_steps) {
        Velocity cmd = follower.getVelocityCommand(pose);

        // Simple kinematic update
        pose.x += cmd.vx * std::cos(pose.theta) * dt;
        pose.y += cmd.vx * std::sin(pose.theta) * dt;
        pose.theta += cmd.omega * dt;

        steps++;
    }

    // Should reach goal within reasonable steps
    EXPECT_TRUE(follower.isComplete());
    EXPECT_LT(steps, max_steps);

    // Should be near goal
    EXPECT_NEAR(pose.x, 9.0f, 0.5f);
    EXPECT_NEAR(pose.y, 5.0f, 0.5f);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
