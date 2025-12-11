#include "detection/VlmClient.h"
#include <curl/curl.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <mutex>
#include <nlohmann/json.hpp>

// Static initialization flag and mutex for thread safety
bool VlmClient::curl_initialized_ = false;
static std::mutex curl_init_mutex_;

// Base64 encoding table
static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Callback for curl response
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append(static_cast<char*>(contents), size * nmemb);
    return size * nmemb;
}

VlmClient::VlmClient(const std::string& api_key) : api_key_(api_key) {}

VlmClient::~VlmClient() = default;

// CRITICAL: Call once at application startup (thread-safe)
void VlmClient::globalInit() {
    std::lock_guard<std::mutex> lock(curl_init_mutex_);
    if (!curl_initialized_) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl_initialized_ = true;
    }
}

// CRITICAL: Call once at application shutdown (thread-safe)
void VlmClient::globalCleanup() {
    std::lock_guard<std::mutex> lock(curl_init_mutex_);
    if (curl_initialized_) {
        curl_global_cleanup();
        curl_initialized_ = false;
    }
}

std::string VlmClient::encodeImageBase64(const cv::Mat& image) const {
    // Resize if too large (reduce API token cost)
    cv::Mat resized = image;
    int max_dim = 2048;  // Max dimension to keep token usage reasonable
    if (image.cols > max_dim || image.rows > max_dim) {
        float scale = static_cast<float>(max_dim) / std::max(image.cols, image.rows);
        cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    }

    // Encode to JPEG
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    cv::imencode(".jpg", resized, buffer, params);

    // Base64 encode with correct padding
    std::string base64;
    base64.reserve(((buffer.size() + 2) / 3) * 4);

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
        // One byte left: outputs 2 chars + 2 padding
        uint32_t val = static_cast<uint32_t>(buffer[i]) << 16;
        base64 += base64_chars[(val >> 18) & 0x3F];
        base64 += base64_chars[(val >> 12) & 0x3F];
        base64 += '=';
        base64 += '=';
    } else if (remaining == 2) {
        // Two bytes left: outputs 3 chars + 1 padding
        uint32_t val = (static_cast<uint32_t>(buffer[i]) << 16) |
                      (static_cast<uint32_t>(buffer[i + 1]) << 8);
        base64 += base64_chars[(val >> 18) & 0x3F];
        base64 += base64_chars[(val >> 12) & 0x3F];
        base64 += base64_chars[(val >> 6) & 0x3F];
        base64 += '=';
    }

    return base64;
}

std::string VlmClient::buildPrompt(const std::string& plan_context, const Pose2D& pose) const {
    std::ostringstream prompt;
    prompt << R"(You are an expert construction site inspector analyzing an image for defects.

CONTEXT:
- This is a construction site inspection photo
- Trade type: )" << (plan_context.empty() ? "general finishes" : plan_context) << R"(
- Robot position: ()" << pose.x << ", " << pose.y << R"() meters, facing )" << pose.theta << R"( radians

TASK:
Analyze this image for construction defects. Look for:
1. LOCATION_ERROR - Elements installed in wrong positions
2. QUALITY_ISSUE - Scratches, cracks, stains, poor workmanship, misalignment
3. SAFETY_HAZARD - Exposed wiring, missing covers, trip hazards
4. MISSING_ELEMENT - Expected elements not present

OUTPUT FORMAT:
Respond with ONLY valid JSON in this exact format:
{
  "defects": [
    {
      "id": "def_001",
      "type": "QUALITY_ISSUE",
      "description": "Scratch visible on tile surface",
      "image_location": {"x": 320, "y": 240},
      "bounding_box": {"x": 280, "y": 200, "width": 80, "height": 80},
      "confidence": 0.87,
      "severity": "medium",
      "trade": "finishes"
    }
  ],
  "summary": "Brief overall assessment"
}

If no defects found, return: {"defects": [], "summary": "No defects detected"}

IMPORTANT:
- Only report defects you are confident about (confidence > 0.5)
- Provide accurate pixel coordinates for bounding boxes
- Be specific in descriptions
)";
    return prompt.str();
}

std::string VlmClient::httpPost(const std::string& body) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        last_error_ = "Failed to initialize curl";
        last_status_code_ = 0;
        return "";
    }

    std::string response;
    struct curl_slist* headers = nullptr;

    // Set headers
    headers = curl_slist_append(headers, ("x-api-key: " + api_key_).c_str());
    headers = curl_slist_append(headers, "anthropic-version: 2023-06-01");
    headers = curl_slist_append(headers, "content-type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, api_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms_);

    // Retry loop with exponential backoff
    int retries = 0;
    while (retries <= max_retries_) {
        response.clear();
        CURLcode res = curl_easy_perform(curl);

        if (res != CURLE_OK) {
            last_error_ = curl_easy_strerror(res);
            last_status_code_ = 0;
        } else {
            long status_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
            last_status_code_ = static_cast<int>(status_code);

            if (last_status_code_ == 200) {
                break;  // Success
            }

            if (!shouldRetry(last_status_code_)) {
                break;  // Non-retryable error
            }
        }

        // Exponential backoff: 1s, 2s, 4s...
        if (retries < max_retries_) {
            int delay_ms = 1000 * (1 << retries);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
        retries++;
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return response;
}

bool VlmClient::shouldRetry(int status_code) const {
    // Retry on rate limit (429) and server errors (5xx)
    return status_code == 429 || (status_code >= 500 && status_code < 600);
}

std::vector<Defect> VlmClient::filterByConfidence(std::vector<Defect>& defects) const {
    defects.erase(
        std::remove_if(defects.begin(), defects.end(),
            [this](const Defect& d) { return d.confidence < confidence_threshold_; }),
        defects.end()
    );
    return defects;
}

std::vector<Defect> VlmClient::parseResponse(const std::string& json_response) {
    std::vector<Defect> defects;
    try {
        auto j = nlohmann::json::parse(json_response);

        // Extract content text from response
        if (!j.contains("content") || j["content"].empty()) {
            last_error_ = "No content in response";
            std::cerr << "[VLM] Parse error: No content in API response" << std::endl;
            return {};
        }

        std::string text = j["content"][0]["text"].get<std::string>();

        // Parse the JSON from the text - VLM output may not be valid JSON
        nlohmann::json result;
        try {
            result = nlohmann::json::parse(text);
        } catch (const nlohmann::json::parse_error& e) {
            // Log the raw VLM output for debugging
            last_error_ = std::string("VLM returned non-JSON response: ") + e.what();
            std::cerr << "[VLM] Parse error: VLM did not return valid JSON" << std::endl;
            std::cerr << "[VLM] Raw VLM output (first 500 chars): "
                      << text.substr(0, std::min(text.size(), size_t(500))) << std::endl;
            return {};
        }

        if (result.contains("defects")) {
            for (const auto& d : result["defects"]) {
                Defect defect;
                defect.id = d.value("id", "unknown");
                defect.type = stringToDefectType(d.value("type", "QUALITY_ISSUE"));
                defect.description = d.value("description", "");
                defect.confidence = d.value("confidence", 0.0f);
                defect.severity = d.value("severity", "medium");
                defect.trade = d.value("trade", "finishes");

                if (d.contains("image_location")) {
                    defect.image_loc.x = d["image_location"].value("x", 0.0f);
                    defect.image_loc.y = d["image_location"].value("y", 0.0f);
                }

                if (d.contains("bounding_box")) {
                    defect.bbox_x = d["bounding_box"].value("x", 0);
                    defect.bbox_y = d["bounding_box"].value("y", 0);
                    defect.bbox_width = d["bounding_box"].value("width", 0);
                    defect.bbox_height = d["bounding_box"].value("height", 0);
                }

                defects.push_back(defect);
            }
        }

        // Track token usage
        if (j.contains("usage")) {
            tokens_used_ = j["usage"].value("input_tokens", 0) + j["usage"].value("output_tokens", 0);
        }

    } catch (const nlohmann::json::exception& e) {
        last_error_ = std::string("JSON parse error: ") + e.what();
        return {};
    }

    // Apply confidence filtering
    return filterByConfidence(defects);
}

std::vector<Defect> VlmClient::analyzeImage(
    const cv::Mat& image,
    const std::string& plan_context,
    const Pose2D& pose
) {
    if (image.empty()) {
        last_error_ = "Empty image provided";
        return {};
    }

    if (api_key_.empty()) {
        last_error_ = "API key not set";
        return {};
    }

    // Build request
    std::string base64_img = encodeImageBase64(image);
    std::string prompt = buildPrompt(plan_context, pose);

    nlohmann::json request = {
        {"model", model_},
        {"max_tokens", 4096},
        {"messages", {{
            {"role", "user"},
            {"content", {
                {{"type", "image"}, {"source", {{"type", "base64"}, {"media_type", "image/jpeg"}, {"data", base64_img}}}},
                {{"type", "text"}, {"text", prompt}}
            }}
        }}}
    };

    // Send request
    std::string response = httpPost(request.dump());

    if (response.empty()) {
        std::cerr << "[VLM] Request failed: " << last_error_ << std::endl;
        return {};
    }

    // Parse response
    return parseResponse(response);
}
