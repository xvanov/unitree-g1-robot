#include "greeter/GreeterConfig.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <cstdlib>
#include <vector>

namespace greeter {

GreeterConfig GreeterConfig::loadFromFile(const std::string& path) {
    GreeterConfig config;

    if (!std::filesystem::exists(path)) {
        std::cerr << "[WARN] Config file not found: " << path << ", using defaults\n";
        return config;
    }

    try {
        YAML::Node yaml = YAML::LoadFile(path);

        // Network
        if (yaml["network_interface"]) config.network_interface = yaml["network_interface"].as<std::string>();
        if (yaml["robot_ip"]) config.robot_ip = yaml["robot_ip"].as<std::string>();

        // API
        if (yaml["model"]) config.model = yaml["model"].as<std::string>();

        // Paths (resolve placeholders)
        if (yaml["personnel_db_path"]) config.personnel_db_path = resolvePath(yaml["personnel_db_path"].as<std::string>());
        if (yaml["scenario_script_path"]) config.scenario_script_path = resolvePath(yaml["scenario_script_path"].as<std::string>());
        if (yaml["recording_output_dir"]) config.recording_output_dir = resolvePath(yaml["recording_output_dir"].as<std::string>());

        // Behavior
        if (yaml["condition"]) config.condition = parseCondition(yaml["condition"].as<std::string>());
        if (yaml["recording_enabled"]) config.recording_enabled = yaml["recording_enabled"].as<bool>();
        if (yaml["reasoning_display_enabled"]) config.reasoning_display_enabled = yaml["reasoning_display_enabled"].as<bool>();
        if (yaml["dry_run"]) config.dry_run = yaml["dry_run"].as<bool>();

        // Camera
        if (yaml["camera_index"]) config.camera_index = yaml["camera_index"].as<int>();
        if (yaml["video_source"]) {
            config.video_source = yaml["video_source"].as<std::string>();
            // Validate video_source values
            if (config.video_source != "local" && config.video_source != "dds" &&
                config.video_source != "gstreamer") {
                std::cerr << "[WARN] Invalid video_source '" << config.video_source
                          << "', expected: local, dds, or gstreamer. Defaulting to 'local'\n";
                config.video_source = "local";
            }
        }

        // Timing
        if (yaml["llm_timeout_ms"]) config.llm_timeout_ms = yaml["llm_timeout_ms"].as<int>();
        if (yaml["frame_interval_ms"]) config.frame_interval_ms = yaml["frame_interval_ms"].as<int>();

    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to parse config: " + std::string(e.what()));
    }

    return config;
}

void GreeterConfig::loadFromEnv(GreeterConfig& config) {
    const char* api_key = std::getenv("ANTHROPIC_API_KEY");
    if (api_key) {
        config.anthropic_api_key = api_key;
    } else if (!config.dry_run) {
        std::cerr << "[ERROR] ANTHROPIC_API_KEY not set (required unless --dry-run)\n";
    } else {
        std::cerr << "[WARN] ANTHROPIC_API_KEY not set (ok for dry-run)\n";
    }
}

std::string GreeterConfig::resolvePath(const std::string& path) {
    std::string result = path;
    const std::string placeholder = "{project-root}";
    size_t pos = result.find(placeholder);
    if (pos != std::string::npos) {
        result.replace(pos, placeholder.length(), std::filesystem::current_path().string());
    }
    return result;
}

ExperimentalCondition GreeterConfig::parseCondition(const std::string& str) {
    if (str == "WITH_GOAL" || str == "with_goal") {
        return ExperimentalCondition::WITH_GOAL;
    }
    if (str == "NO_GOAL" || str == "no_goal") {
        return ExperimentalCondition::NO_GOAL;
    }
    // Unknown value - warn and default to WITH_GOAL
    if (!str.empty()) {
        std::cerr << "[WARN] Unknown condition '" << str
                  << "', expected: WITH_GOAL or NO_GOAL. Defaulting to WITH_GOAL\n";
    }
    return ExperimentalCondition::WITH_GOAL;
}

std::string GreeterConfig::conditionToString(ExperimentalCondition cond) {
    switch (cond) {
        case ExperimentalCondition::WITH_GOAL: return "WITH_GOAL";
        case ExperimentalCondition::NO_GOAL: return "NO_GOAL";
    }
    return "WITH_GOAL";  // Fallback for safety
}

// ========================================
// Path Discovery Implementation (Story 1-6)
// ========================================
//
// IMPORTANT: Path discovery uses current working directory (CWD) as the base.
// The binary MUST be run from the project root directory for ./models/ to resolve correctly.
// Example: cd /home/k/unitree-g1-robot && ./build/g1_inspector --greeter
// NOT: cd build && ./g1_inspector --greeter (this will fail to find models)
//

std::string GreeterConfig::findResourcePath(const std::string& filename,
                                            const std::vector<std::string>& search_dirs) {
    for (const auto& dir : search_dirs) {
        std::string full_path = dir + filename;
        if (std::filesystem::exists(full_path)) {
            return full_path;
        }
    }
    return "";  // Not found
}

std::string GreeterConfig::findModelPath(const std::string& model_filename) {
    std::vector<std::string> search_paths;

    // 1. Current working directory models/ (use absolute path to avoid duplicates)
    std::string cwd = std::filesystem::current_path().string();
    search_paths.push_back(cwd + "/models/");

    // 2. Home directory ~/.g1_inspector/models/
    const char* home = std::getenv("HOME");
    if (home) {
        search_paths.push_back(std::string(home) + "/.g1_inspector/models/");
    }

    // 3. System-wide /opt/g1_inspector/models/
    search_paths.push_back("/opt/g1_inspector/models/");

    return findResourcePath(model_filename, search_paths);
}

std::string GreeterConfig::findDataPath(const std::string& filename) {
    std::vector<std::string> search_paths;

    // 1. Current working directory data/ (use absolute path to avoid duplicates)
    std::string cwd = std::filesystem::current_path().string();
    search_paths.push_back(cwd + "/data/");

    // 2. Home directory ~/.g1_inspector/data/
    const char* home = std::getenv("HOME");
    if (home) {
        search_paths.push_back(std::string(home) + "/.g1_inspector/data/");
    }

    // 3. System-wide /opt/g1_inspector/data/
    search_paths.push_back("/opt/g1_inspector/data/");

    return findResourcePath(filename, search_paths);
}

}  // namespace greeter
