#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "util/Types.h"
#include "detection/DefectTypes.h"

class VlmClient {
public:
    explicit VlmClient(const std::string& api_key);
    virtual ~VlmClient();  // Virtual for MockVlmClient inheritance

    // CRITICAL: Call once at application startup before creating any VlmClient
    static void globalInit();
    // CRITICAL: Call once at application shutdown after all VlmClients destroyed
    static void globalCleanup();

    // Main analysis function (virtual for mocking)
    virtual std::vector<Defect> analyzeImage(
        const cv::Mat& image,
        const std::string& plan_context,
        const Pose2D& pose
    );

    // Configuration
    void setApiUrl(const std::string& url) { api_url_ = url; }
    void setModel(const std::string& model) { model_ = model; }
    void setMaxRetries(int n) { max_retries_ = n; }
    void setTimeout(long ms) { timeout_ms_ = ms; }
    void setConfidenceThreshold(float t) { confidence_threshold_ = t; }

    // Status
    int getLastStatusCode() const { return last_status_code_; }
    std::string getLastError() const { return last_error_; }
    int getTokensUsed() const { return tokens_used_; }

protected:
    std::string buildPrompt(const std::string& plan_context, const Pose2D& pose) const;
    std::string encodeImageBase64(const cv::Mat& image) const;
    std::string httpPost(const std::string& body);
    std::vector<Defect> parseResponse(const std::string& json_response);
    std::vector<Defect> filterByConfidence(std::vector<Defect>& defects) const;
    bool shouldRetry(int status_code) const;

    std::string api_key_;
    std::string api_url_ = "https://api.anthropic.com/v1/messages";
    std::string model_ = "claude-sonnet-4-5-20250514";
    int max_retries_ = 3;
    long timeout_ms_ = 30000;
    float confidence_threshold_ = 0.5f;

    // Last request status
    int last_status_code_ = 0;
    std::string last_error_;
    int tokens_used_ = 0;

private:
    static bool curl_initialized_;
};
