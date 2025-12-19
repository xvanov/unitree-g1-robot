#include "greeter/GreeterVlmClient.h"
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

namespace greeter {

// Static initialization flag and mutex for thread safety
static bool curl_initialized_ = false;
static std::mutex curl_init_mutex_;

// Base64 encoding table
static const char* base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Callback for curl response
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// CRITICAL: Call once at application startup (thread-safe)
void GreeterVlmClient::globalInit() {
    std::lock_guard<std::mutex> lock(curl_init_mutex_);
    if (!curl_initialized_) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl_initialized_ = true;
    }
}

// CRITICAL: Call once at application shutdown (thread-safe)
void GreeterVlmClient::globalCleanup() {
    std::lock_guard<std::mutex> lock(curl_init_mutex_);
    if (curl_initialized_) {
        curl_global_cleanup();
        curl_initialized_ = false;
    }
}

GreeterVlmClient::GreeterVlmClient(const std::string& api_key)
    : api_key_(api_key) {}

GreeterVlmClient::~GreeterVlmClient() = default;

std::string GreeterVlmClient::sendObservation(
    const std::string& system_prompt,
    const cv::Mat& frame,
    const std::string& context_json
) {
    if (frame.empty()) {
        last_error_ = "Empty frame provided";
        return "";
    }

    std::string base64_image = encodeImageBase64(frame);
    std::string body = buildRequestBody(system_prompt, base64_image, context_json);
    return httpPost(body);
}

std::string GreeterVlmClient::sendTextObservation(
    const std::string& system_prompt,
    const std::string& context_json
) {
    std::string body = buildRequestBody(system_prompt, "", context_json);
    return httpPost(body);
}

std::string GreeterVlmClient::buildRequestBody(
    const std::string& system_prompt,
    const std::string& base64_image,
    const std::string& context_json
) {
    nlohmann::json body;
    body["model"] = model_;
    body["max_tokens"] = max_tokens_;
    body["system"] = system_prompt;

    nlohmann::json content;
    if (!base64_image.empty()) {
        // Vision request with image
        content = nlohmann::json::array({
            {
                {"type", "image"},
                {"source", {
                    {"type", "base64"},
                    {"media_type", "image/jpeg"},
                    {"data", base64_image}
                }}
            },
            {
                {"type", "text"},
                {"text", context_json}
            }
        });
    } else {
        // Text-only request
        content = context_json;
    }

    body["messages"] = nlohmann::json::array({
        {{"role", "user"}, {"content", content}}
    });

    return body.dump();
}

std::string GreeterVlmClient::httpPost(const std::string& body) {
    int retries = 0;
    int backoff_ms = 1000;

    while (retries <= max_retries_) {
        CURL* curl = curl_easy_init();
        if (!curl) {
            last_error_ = "Failed to initialize curl";
            return "";
        }

        std::string response;
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        headers = curl_slist_append(headers,
            ("x-api-key: " + api_key_).c_str());
        headers = curl_slist_append(headers, "anthropic-version: 2023-06-01");

        curl_easy_setopt(curl, CURLOPT_URL, api_url_.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms_);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &last_status_code_);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            last_error_ = std::string("curl error: ") + curl_easy_strerror(res);
            if (retries < max_retries_) {
                retries++;
                std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
                backoff_ms *= 2;
                continue;
            }
            return "";
        }

        // Check for retry-able errors
        if (shouldRetry(last_status_code_) && retries < max_retries_) {
            retries++;
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            backoff_ms *= 2;  // Exponential backoff
            continue;
        }

        // Check for non-success status codes
        if (last_status_code_ != 200) {
            last_error_ = "HTTP error: " + std::to_string(last_status_code_);
            // Log the error response for debugging
            std::cerr << "[GreeterVlmClient] API error " << last_status_code_
                      << ": " << response.substr(0, 500) << std::endl;
            return "";
        }

        // Parse response for token usage and extract content
        try {
            auto json_response = nlohmann::json::parse(response);

            // Track token usage
            if (json_response.contains("usage")) {
                input_tokens_ = json_response["usage"].value("input_tokens", 0);
                output_tokens_ = json_response["usage"].value("output_tokens", 0);
                tokens_used_ = input_tokens_ + output_tokens_;
            }

            // Extract content text
            if (json_response.contains("content") &&
                json_response["content"].is_array() &&
                !json_response["content"].empty()) {
                return json_response["content"][0]["text"].get<std::string>();
            }

            last_error_ = "No content in API response";
            return "";

        } catch (const std::exception& e) {
            last_error_ = std::string("JSON parse error: ") + e.what();
            return "";
        }
    }

    last_error_ = "Max retries exceeded";
    return "";
}

bool GreeterVlmClient::shouldRetry(int status_code) const {
    // Retry on rate limit (429) and server errors (5xx)
    return status_code == 429 || status_code >= 500;
}

// CRITICAL: Match VlmClient::encodeImageBase64() exactly - includes resize + quality optimization
std::string GreeterVlmClient::encodeImageBase64(const cv::Mat& image) const {
    // CRITICAL: Resize to max 2048px to reduce token cost
    cv::Mat resized = image;
    int max_dim = 2048;
    if (image.cols > max_dim || image.rows > max_dim) {
        float scale = static_cast<float>(max_dim) / std::max(image.cols, image.rows);
        cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    }

    // CRITICAL: Use quality 85 for good compression without visible artifacts
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    cv::imencode(".jpg", resized, buffer, params);

    // Base64 encode with reservation for efficiency
    std::string base64;
    base64.reserve(((buffer.size() + 2) / 3) * 4);  // Pre-allocate

    size_t i = 0;
    size_t len = buffer.size();

    // Process complete 3-byte groups
    while (i + 2 < len) {
        uint32_t triple = (static_cast<uint32_t>(buffer[i]) << 16) |
                         (static_cast<uint32_t>(buffer[i + 1]) << 8) |
                         static_cast<uint32_t>(buffer[i + 2]);

        base64 += base64_chars[(triple >> 18) & 0x3F];
        base64 += base64_chars[(triple >> 12) & 0x3F];
        base64 += base64_chars[(triple >> 6) & 0x3F];
        base64 += base64_chars[triple & 0x3F];
        i += 3;
    }

    // Handle remaining bytes with proper padding
    size_t remaining = len - i;
    if (remaining == 1) {
        uint32_t val = static_cast<uint32_t>(buffer[i]) << 16;
        base64 += base64_chars[(val >> 18) & 0x3F];
        base64 += base64_chars[(val >> 12) & 0x3F];
        base64 += '=';
        base64 += '=';
    } else if (remaining == 2) {
        uint32_t val = (static_cast<uint32_t>(buffer[i]) << 16) |
                      (static_cast<uint32_t>(buffer[i + 1]) << 8);
        base64 += base64_chars[(val >> 18) & 0x3F];
        base64 += base64_chars[(val >> 12) & 0x3F];
        base64 += base64_chars[(val >> 6) & 0x3F];
        base64 += '=';
    }

    return base64;
}

}  // namespace greeter
