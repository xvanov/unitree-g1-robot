#pragma once

#include "detection/VlmClient.h"

class MockVlmClient : public VlmClient {
public:
    MockVlmClient() : VlmClient("mock-api-key") {}

    std::vector<Defect> analyzeImage(
        const cv::Mat& image,
        const std::string& plan_context,
        const Pose2D& pose
    ) override {
        call_count_++;
        last_image_size_ = {image.cols, image.rows};
        last_plan_context_ = plan_context;
        last_pose_ = pose;
        return mock_defects_;
    }

    // Test setup
    void setMockDefects(const std::vector<Defect>& defects) { mock_defects_ = defects; }
    void clearMockDefects() { mock_defects_.clear(); }

    // Test verification
    int getCallCount() const { return call_count_; }
    cv::Size getLastImageSize() const { return last_image_size_; }
    std::string getLastPlanContext() const { return last_plan_context_; }
    Pose2D getLastPose() const { return last_pose_; }
    void resetCallCount() { call_count_ = 0; }

private:
    std::vector<Defect> mock_defects_;
    int call_count_ = 0;
    cv::Size last_image_size_;
    std::string last_plan_context_;
    Pose2D last_pose_;
};
