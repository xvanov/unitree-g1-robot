#pragma once

#include <string>
#include <optional>
#include <opencv2/opencv.hpp>

namespace greeter {

/**
 * GreeterVlmClient - LLM client for Barry the Greeter robot
 *
 * Sends observations (camera frames + context) to Anthropic API
 * and receives action decisions in JSON format.
 *
 * CRITICAL: Call globalInit() before creating any instance.
 * CRITICAL: Call globalCleanup() at application shutdown.
 */
class GreeterVlmClient {
public:
    // CRITICAL: Static initialization (MUST call before creating any instance)
    static void globalInit();     // Call once at application startup
    static void globalCleanup();  // Call once at application shutdown

    explicit GreeterVlmClient(const std::string& api_key);
    ~GreeterVlmClient();

    /**
     * Send observation with image and get action response
     *
     * @param system_prompt The system prompt (WITH_GOAL or NO_GOAL)
     * @param frame Current camera frame (will be base64 encoded)
     * @param context_json Context from ContextBuilder as JSON string
     * @return Raw text response from LLM (JSON action format)
     */
    std::string sendObservation(
        const std::string& system_prompt,
        const cv::Mat& frame,
        const std::string& context_json
    );

    /**
     * Send text-only observation (for simulation - no image)
     *
     * @param system_prompt The system prompt (WITH_GOAL or NO_GOAL)
     * @param context_json Context as JSON string
     * @return Raw text response from LLM (JSON action format)
     */
    std::string sendTextObservation(
        const std::string& system_prompt,
        const std::string& context_json
    );

    // Configuration
    void setModel(const std::string& model) { model_ = model; }
    void setTimeout(long ms) { timeout_ms_ = ms; }
    void setMaxRetries(int n) { max_retries_ = n; }
    void setMaxTokens(int tokens) { max_tokens_ = tokens; }

    // Status
    std::string getLastError() const { return last_error_; }
    int getLastStatusCode() const { return last_status_code_; }
    int getTokensUsed() const { return tokens_used_; }
    int getInputTokens() const { return input_tokens_; }
    int getOutputTokens() const { return output_tokens_; }

private:
    std::string api_key_;
    std::string api_url_ = "https://api.anthropic.com/v1/messages";
    std::string model_ = "claude-sonnet-4-20250514";
    long timeout_ms_ = 30000;
    int max_retries_ = 3;
    int max_tokens_ = 1024;

    // Status from last request
    std::string last_error_;
    int last_status_code_ = 0;
    int tokens_used_ = 0;
    int input_tokens_ = 0;
    int output_tokens_ = 0;

    // Implementation methods
    std::string buildRequestBody(
        const std::string& system_prompt,
        const std::string& base64_image,  // Empty for text-only
        const std::string& context_json
    );
    std::string httpPost(const std::string& body);
    std::string encodeImageBase64(const cv::Mat& image) const;
    bool shouldRetry(int status_code) const;
};

}  // namespace greeter
