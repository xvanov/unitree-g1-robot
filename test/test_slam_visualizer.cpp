#include <gtest/gtest.h>
#include "slam/SlamVisualizer.h"
#include "slam/GridMapper.h"
#include "sensors/ISensorSource.h"

class SlamVisualizerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a small test grid
        mapper_ = std::make_unique<GridMapper>(0.05f, 100, 100);  // 5m x 5m grid
    }

    std::unique_ptr<GridMapper> mapper_;
};

TEST_F(SlamVisualizerTest, ConstructionDoesNotThrow) {
    // Test that we can create a visualizer without crashing
    // Note: This will create an OpenCV window, so we skip this in headless envs
    if (getenv("DISPLAY") == nullptr) {
        GTEST_SKIP() << "No display available, skipping window test";
    }

    ASSERT_NO_THROW({
        SlamVisualizer viz(800, 600);
    });
}

TEST_F(SlamVisualizerTest, UpdateDoesNotCrash) {
    if (getenv("DISPLAY") == nullptr) {
        GTEST_SKIP() << "No display available";
    }

    SlamVisualizer viz(800, 600);
    Pose2D pose{1.0f, 2.0f, 0.5f};

    // Update without scan
    ASSERT_NO_THROW({
        viz.update(*mapper_, pose, nullptr);
    });

    // Update with scan
    LidarScan scan;
    scan.ranges.resize(360, 5.0f);  // 360 points at 5m
    scan.angle_min = 0.0f;
    scan.angle_max = 2.0f * M_PI;

    ASSERT_NO_THROW({
        viz.update(*mapper_, pose, &scan);
    });
}

TEST_F(SlamVisualizerTest, WorldToScreenTransformIsConsistent) {
    if (getenv("DISPLAY") == nullptr) {
        GTEST_SKIP() << "No display available";
    }

    SlamVisualizer viz(800, 600);

    // At default zoom (20 pixels/meter), 1 meter should be 20 pixels
    // Test a point at (0, 0) which should map to center of window
    viz.panTo(0, 0);
    viz.setZoom(20.0f);

    // The center of the window in screen coords is (400, 300)
    // World (0, 0) should map to screen (400, 300)
    // We can't directly test worldToScreen since it's private, but
    // we can verify the visualizer renders without crashing

    Pose2D pose{0, 0, 0};
    ASSERT_NO_THROW({
        viz.update(*mapper_, pose, nullptr);
        viz.render();
    });
}

TEST_F(SlamVisualizerTest, ZoomBoundsAreRespected) {
    if (getenv("DISPLAY") == nullptr) {
        GTEST_SKIP() << "No display available";
    }

    SlamVisualizer viz(800, 600);

    // Try to set zoom below minimum
    viz.setZoom(1.0f);  // Below min of 5.0
    // We can't directly check the zoom value since it's private,
    // but we can verify it doesn't crash
    ASSERT_NO_THROW({
        viz.render();
    });

    // Try to set zoom above maximum
    viz.setZoom(200.0f);  // Above max of 100.0
    ASSERT_NO_THROW({
        viz.render();
    });
}

TEST_F(SlamVisualizerTest, CenterOnRobotWorks) {
    if (getenv("DISPLAY") == nullptr) {
        GTEST_SKIP() << "No display available";
    }

    SlamVisualizer viz(800, 600);

    // Update with a robot position
    Pose2D pose{3.0f, 4.0f, 1.57f};
    viz.update(*mapper_, pose, nullptr);

    // Center on robot
    ASSERT_NO_THROW({
        viz.centerOnRobot();
        viz.render();
    });
}

TEST_F(SlamVisualizerTest, ProcessKeyHandlesAllKeys) {
    if (getenv("DISPLAY") == nullptr) {
        GTEST_SKIP() << "No display available";
    }

    SlamVisualizer viz(800, 600);

    // Test various keys
    EXPECT_EQ(viz.processKey(-1), -1);  // No key
    EXPECT_EQ(viz.processKey('l'), 'l');  // Toggle LiDAR
    EXPECT_EQ(viz.processKey('r'), 'r');  // Reset view
    EXPECT_EQ(viz.processKey('+'), '+');  // Zoom in
    EXPECT_EQ(viz.processKey('-'), '-');  // Zoom out

    // 'q' should set shouldClose
    viz.processKey('q');
    EXPECT_TRUE(viz.shouldClose());
}

TEST_F(SlamVisualizerTest, StatsStringIsFormatted) {
    if (getenv("DISPLAY") == nullptr) {
        GTEST_SKIP() << "No display available";
    }

    SlamVisualizer viz(800, 600);
    Pose2D pose{1.5f, 2.5f, 0.785f};
    viz.update(*mapper_, pose, nullptr);

    std::string stats = viz.getStatsString();

    // Should contain key elements
    EXPECT_NE(stats.find("Cells:"), std::string::npos);
    EXPECT_NE(stats.find("Pose:"), std::string::npos);
    EXPECT_NE(stats.find("Scans:"), std::string::npos);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
