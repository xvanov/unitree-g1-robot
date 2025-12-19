#pragma once

#include <string>

namespace greeter {

enum class ExperimentalCondition {
    WITH_GOAL,    // Condition 1: Barry has identity, purpose, pride in role
    NO_GOAL       // Condition 2: Barry has capabilities only, no explicit goal
};

struct GreeterConfig {
    // Network
    std::string network_interface = "eth0";
    std::string robot_ip = "192.168.123.164";

    // API (anthropic_api_key loaded from env, never from file)
    std::string anthropic_api_key;
    std::string model = "claude-sonnet-4-5-20250514";

    // Paths (relative to project root, support {project-root} placeholder)
    std::string personnel_db_path = "data/personnel/gauntlet_personnel.json";
    std::string scenario_script_path = "data/scripts/barry_demo.json";
    std::string recording_output_dir = "data/recordings";

    // Behavior
    ExperimentalCondition condition = ExperimentalCondition::WITH_GOAL;
    bool recording_enabled = true;
    bool reasoning_display_enabled = true;
    bool dry_run = false;  // Skip robot connection for local testing

    // Camera
    int camera_index = 0;
    std::string video_source = "local";  // Valid: "local", "dds", "gstreamer"

    // Timing
    int llm_timeout_ms = 30000;
    int frame_interval_ms = 33;  // ~30 FPS

    // Load from YAML file (returns default config if file missing, throws on parse error)
    static GreeterConfig loadFromFile(const std::string& path);

    // Load/overlay from environment variables (call after loadFromFile)
    static void loadFromEnv(GreeterConfig& config);

    // Helper: resolve {project-root} placeholder in paths
    static std::string resolvePath(const std::string& path);

    // Helper: convert string to ExperimentalCondition
    static ExperimentalCondition parseCondition(const std::string& str);

    // Helper: convert ExperimentalCondition to string
    static std::string conditionToString(ExperimentalCondition cond);
};

}  // namespace greeter
