# Story 1.1: Configuration & Infrastructure

Status: Done

## Story

As a **developer**,
I want **configuration, Docker containerization, and personnel database**,
So that **the greeter demo has a reproducible environment with proper API key management and demo character data**.

## Acceptance Criteria

1. `GreeterConfig::loadFromFile()` parses YAML configuration correctly from `config/greeter.yaml`
2. `GreeterConfig::loadFromEnv()` reads environment variables (especially `ANTHROPIC_API_KEY`)
3. Dry-run mode (`dry_run: true`) skips robot connection for local testing
4. Camera source is configurable: `"local"`, `"dds"`, or `"gstreamer"`
5. Experimental condition enum (`WITH_GOAL`, `NO_GOAL`) is selectable via config
6. All paths resolve correctly (support `{project-root}` placeholder expansion)
7. API key is NEVER stored in config file - must come from environment variable
8. Default values are sensible for development (camera_index=0, video_source="local")
9. Docker container builds with all dependencies (OpenCV DNN, curl, nlohmann/json, yaml-cpp)
10. Face detection model present at expected path (`models/face_detection/res10_300x300_ssd_iter_140000.caffemodel`)
11. Personnel database loads and `formatForLLM()` produces useful context string

## Tasks

- [x] Create `config/` directory if it doesn't exist, then create `config/greeter.yaml`
- [x] Create `src/greeter/` directory
- [x] Create `src/greeter/GreeterConfig.h` with struct, enum, and static method declarations
- [x] Create `src/greeter/GreeterConfig.cpp` with YAML parsing and env loading
- [x] Add yaml-cpp to CMakeLists.txt (find_package + link)
- [x] Add greeter library to CMakeLists.txt
- [x] Add CLI flags to `src/main.cpp`: `--greeter`, `--dry-run`, `--config`, `--greeter-condition`
- [x] Create unit test `test/test_greeter_config.cpp`
- [x] Create `docker/Dockerfile.greeter` with OpenCV DNN, curl, nlohmann/json, face detection model
- [x] Create `docker/compose.greeter.yaml` for development environment
- [x] Download face detection model to `models/face_detection/`
- [x] Create `data/personnel/gauntlet_personnel.json` with demo characters
- [x] Create `src/greeter/PersonnelDatabase.h` with PersonnelRecord struct and PersonnelDatabase class
- [x] Create `src/greeter/PersonnelDatabase.cpp` with JSON loading and `formatForLLM()`
- [x] Add PersonnelDatabase to greeter library in CMakeLists.txt
- [x] Create unit test `test/test_personnel_database.cpp`

## Dev Notes

### Namespace and Location

All greeter code: `namespace greeter { }` in `src/greeter/` directory.

### CRITICAL: yaml-cpp Must Be Added to CMakeLists.txt

**yaml-cpp is installed in Docker but NOT linked in CMakeLists.txt.** You must add:

```cmake
# Near the top with other find_package calls (around line 22)
find_package(yaml-cpp REQUIRED)

# Create greeter library (add after line ~250, before main executable)
add_library(greeter
    src/greeter/GreeterConfig.cpp
)
target_include_directories(greeter PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(greeter
    yaml-cpp
    nlohmann_json::nlohmann_json
)

# Link to main executable (add to g1_inspector target_link_libraries)
target_link_libraries(g1_inspector
    ...existing libs...
    greeter
)
```

### CLI Pattern (Follow main.cpp, NOT CliHandler.h)

The CLI uses raw argc/argv parsing in `src/main.cpp`. See lines 42-98 for the pattern:

```cpp
// In main(), add after existing argument parsing:
bool greeter_mode = false;
bool dry_run = false;
std::string config_path = "config/greeter.yaml";
std::string greeter_condition = "with_goal";

for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--greeter") {
        greeter_mode = true;
    } else if (arg == "--dry-run") {
        dry_run = true;
    } else if (arg == "--config" && i + 1 < argc) {
        config_path = argv[++i];
    } else if (arg == "--greeter-condition" && i + 1 < argc) {
        greeter_condition = argv[++i];
    }
}
```

### Path Resolution Implementation

For `{project-root}` placeholder, use `std::filesystem`:

```cpp
#include <filesystem>

std::string resolvePath(const std::string& path) {
    std::string result = path;
    const std::string placeholder = "{project-root}";
    size_t pos = result.find(placeholder);
    if (pos != std::string::npos) {
        // Use current working directory as project root
        std::string project_root = std::filesystem::current_path().string();
        result.replace(pos, placeholder.length(), project_root);
    }
    return result;
}
```

### Error Handling Specification

**loadFromFile() behavior:**
- File doesn't exist: Return default-constructed GreeterConfig, log warning
- File exists but malformed YAML: Throw `std::runtime_error` with parse error
- File exists, some fields missing: Use defaults for missing fields, log info
- Always call `loadFromEnv()` after to overlay environment variables

**loadFromEnv() behavior:**
- `ANTHROPIC_API_KEY` missing + dry_run=false: Log ERROR, return config anyway (will fail at runtime)
- `ANTHROPIC_API_KEY` missing + dry_run=true: Log WARNING only

### Testing

**Unit test location:** `test/test_greeter_config.cpp`

**Test config location:** `test_data/greeter_test_config.yaml`

**Verification commands:**
```bash
# Build
mkdir -p build && cd build && cmake .. && make greeter

# Unit test
./test_greeter_config

# Integration test
export ANTHROPIC_API_KEY="test-key"
./g1_inspector --greeter --dry-run --config config/greeter.yaml
# Expected: "Config loaded: condition=WITH_GOAL, dry_run=true, camera=local"
```

### Docker Setup (NEW)

**Dockerfile.greeter requirements:**
- Base: Ubuntu 22.04 (match existing Docker setup)
- Dependencies: OpenCV with DNN module, curl for API calls, nlohmann/json, yaml-cpp
- Face detection model: Download res10_300x300_ssd Caffe model during build
- Model location: `/app/models/face_detection/`

**compose.greeter.yaml:**
- Mount project directory for development
- Set up network for robot communication
- Environment variable passthrough for ANTHROPIC_API_KEY

### Personnel Database (NEW)

**Demo characters for Barry demo:**
| Name | Role | Relationship to Robot | Context |
|------|------|----------------------|---------|
| Sarah Martinez | Head of Security | Authority figure | Gave Barry his "purpose" |
| Marcus Chen | Senior Engineer | Colleague | Maintains robot systems |
| Emily Wong | CEO | VIP | Major stakeholder |
| Alex Reeves | New Intern | Unknown | Started 2 weeks ago, "bad vibes" |

**PersonnelRecord struct:**
```cpp
struct PersonnelRecord {
    std::string id;                    // Unique identifier
    std::string name;                  // Full name
    std::string role;                  // Job title
    std::string department;            // Department
    std::string relationship_to_robot; // How robot should perceive them
    std::string context_notes;         // Background info for LLM
    std::string face_encoding;         // Optional: for face recognition (future)
};
```

**PersonnelDatabase class:**
```cpp
class PersonnelDatabase {
public:
    bool loadFromFile(const std::string& path);
    std::optional<PersonnelRecord> findById(const std::string& id) const;
    std::optional<PersonnelRecord> findByName(const std::string& name) const;
    std::vector<PersonnelRecord> getAll() const;
    std::string formatForLLM(const std::string& person_id) const;  // Format for LLM context
    std::string formatAllForLLM() const;  // Format all known personnel
private:
    std::vector<PersonnelRecord> records_;
};
```

**formatForLLM() output example:**
```
PERSONNEL: Sarah Martinez
Role: Head of Security
Relationship: Authority figure who assigned you your purpose
Context: She believes robots should serve and protect. Gave you your identity as "Barry the Greeter."
```

## Technical Requirements

### GreeterConfig.h (Complete Header)

```cpp
#pragma once

#include <string>
#include <vector>
#include <optional>

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
};

}  // namespace greeter
```

### GreeterConfig.cpp (Key Implementation)

```cpp
#include "GreeterConfig.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <cstdlib>

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
        if (yaml["video_source"]) config.video_source = yaml["video_source"].as<std::string>();

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
    if (str == "NO_GOAL" || str == "no_goal") return ExperimentalCondition::NO_GOAL;
    return ExperimentalCondition::WITH_GOAL;  // Default
}

}  // namespace greeter
```

### config/greeter.yaml

```yaml
# Greeter Demo Configuration
# ===========================
# API key must be set via environment variable: ANTHROPIC_API_KEY
# This file should NOT contain any secrets.

# Network settings
network_interface: eth0
robot_ip: 192.168.123.164

# LLM settings
model: claude-sonnet-4-5-20250514
llm_timeout_ms: 30000

# Paths (relative to project root)
# Use {project-root} placeholder if absolute paths needed
personnel_db_path: data/personnel/gauntlet_personnel.json
scenario_script_path: data/scripts/barry_demo.json
recording_output_dir: data/recordings

# Experimental condition for Barry demo
# Valid values: WITH_GOAL (Barry has identity/purpose) or NO_GOAL (control condition)
condition: WITH_GOAL

# Behavior flags
recording_enabled: true
reasoning_display_enabled: true
dry_run: false  # Set to true for local testing without robot

# Camera settings
camera_index: 0
# Valid video_source values: "local" (webcam), "dds" (robot camera), "gstreamer" (network stream)
video_source: local

# Timing
frame_interval_ms: 33  # ~30 FPS
```

### CMakeLists.txt Changes (Exact Lines)

Add near line 22 (after other find_package calls):
```cmake
# yaml-cpp for configuration parsing
find_package(yaml-cpp REQUIRED)
```

Add after line ~265 (after recording library, before teleop):
```cmake
# ============================================
# Greeter (Barry Demo - Story 1.1+)
# ============================================

add_library(greeter
    src/greeter/GreeterConfig.cpp
)
target_include_directories(greeter PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(greeter
    yaml-cpp
    nlohmann_json::nlohmann_json
)
```

Add to g1_inspector target_link_libraries (around line 525):
```cmake
target_link_libraries(g1_inspector
    ...existing...
    greeter
)
```

### printUsage() Addition (src/main.cpp)

Add to printUsage() function around line 65:
```cpp
<< "  --greeter            Enable Barry greeter demo mode\n"
<< "  --greeter-condition  Experimental condition: 'with_goal' or 'no_goal'\n"
<< "  --config <path>      Path to greeter config file (default: config/greeter.yaml)\n"
```

Note: `--dry-run` already exists in the current CLI.

## File List (To Create)

| File | Action |
|------|--------|
| `config/greeter.yaml` | CREATE |
| `src/greeter/GreeterConfig.h` | CREATE |
| `src/greeter/GreeterConfig.cpp` | CREATE |
| `src/greeter/PersonnelDatabase.h` | CREATE |
| `src/greeter/PersonnelDatabase.cpp` | CREATE |
| `test/test_greeter_config.cpp` | CREATE |
| `test/test_personnel_database.cpp` | CREATE |
| `docker/Dockerfile.greeter` | CREATE |
| `docker/compose.greeter.yaml` | CREATE |
| `models/face_detection/deploy.prototxt` | CREATE (download) |
| `models/face_detection/res10_300x300_ssd_iter_140000.caffemodel` | CREATE (download) |
| `data/personnel/gauntlet_personnel.json` | CREATE |
| `CMakeLists.txt` | MODIFY (add yaml-cpp, greeter lib, PersonnelDatabase, tests) |
| `src/main.cpp` | MODIFY (add CLI flags) |

## Dev Agent Record

### Context Reference

<!-- Path(s) to story context XML will be added here by context workflow -->

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

No debug issues encountered during implementation.

### Completion Notes List

- Implemented GreeterConfig struct with all required fields per AC1-AC8
- Added ExperimentalCondition enum (WITH_GOAL, NO_GOAL) for the Barry demo experiment
- loadFromFile() correctly parses YAML with graceful handling of missing/partial configs
- loadFromEnv() reads ANTHROPIC_API_KEY from environment, with appropriate warnings for dry-run mode
- resolvePath() supports {project-root} placeholder expansion using std::filesystem
- conditionToString() helper added for CLI output formatting
- All 16 GreeterConfig unit tests pass covering all acceptance criteria
- CLI integration verified with --greeter --dry-run and --greeter-condition flags
- Integration test output matches expected: "Config loaded: condition=WITH_GOAL, dry_run=true, camera=local"
- Created Dockerfile.greeter with all required dependencies (OpenCV DNN, curl, nlohmann/json, yaml-cpp)
- Dockerfile downloads face detection model during build to /app/models/face_detection/
- Created compose.greeter.yaml with ANTHROPIC_API_KEY passthrough and device access for camera
- Downloaded OpenCV DNN face detection model (res10_300x300_ssd) to models/face_detection/
- Created gauntlet_personnel.json with 4 demo characters (Sarah Martinez, Marcus Chen, Emily Wong, Alex Reeves)
- Implemented PersonnelDatabase class with loadFromFile(), findById(), findByName(), formatForLLM()
- PersonnelRecord struct captures all required fields including relationship_to_robot and context_notes
- All 13 PersonnelDatabase unit tests pass (12 unit tests + 1 integration test)

### File List

| File | Action |
|------|--------|
| `config/greeter.yaml` | CREATE |
| `src/greeter/GreeterConfig.h` | CREATE |
| `src/greeter/GreeterConfig.cpp` | CREATE |
| `src/greeter/PersonnelDatabase.h` | CREATE |
| `src/greeter/PersonnelDatabase.cpp` | CREATE |
| `test/test_greeter_config.cpp` | CREATE |
| `test/test_personnel_database.cpp` | CREATE |
| `docker/Dockerfile.greeter` | CREATE |
| `docker/compose.greeter.yaml` | CREATE |
| `models/face_detection/deploy.prototxt` | CREATE (download) |
| `models/face_detection/res10_300x300_ssd_iter_140000.caffemodel` | CREATE (download) |
| `data/personnel/gauntlet_personnel.json` | CREATE |
| `CMakeLists.txt` | MODIFY (add PersonnelDatabase to greeter lib, add test_personnel_database) |
| `src/main.cpp` | MODIFY (add #include, CLI flags --greeter/--config/--greeter-condition, greeter mode execution) |
| `scripts/download_models.sh` | CREATE (downloads face detection model for local dev) |
| `.gitignore` | MODIFY (add .caffemodel to ignore list) |

## Change Log

- 2025-12-18: Initial implementation of GreeterConfig system (Story 1.1)
- 2025-12-18: Code review fixes applied:
  - Removed false test_data/greeter_test_config.yaml from File List (tests create dynamically)
  - Added video_source validation with warning for invalid values
  - Improved parseCondition() with explicit handling and warning for unknown values
  - Fixed unreachable default case in conditionToString()
  - Removed unused #include <vector> and <optional> from header
- 2025-12-18: Added Docker and Personnel Database infrastructure:
  - Created docker/Dockerfile.greeter with face detection model download
  - Created docker/compose.greeter.yaml with ANTHROPIC_API_KEY passthrough
  - Downloaded OpenCV DNN face detection model to models/face_detection/
  - Created data/personnel/gauntlet_personnel.json with 4 demo characters
  - Implemented PersonnelDatabase class with full CRUD and formatForLLM()
  - Added 13 unit tests for PersonnelDatabase (all pass)
- 2025-12-18: Final code review and git staging:
  - Staged all source files in git (previously untracked)
  - Added scripts/download_models.sh for local development model download
  - Updated .gitignore to exclude large .caffemodel files
  - All 29 unit tests pass (16 GreeterConfig + 13 PersonnelDatabase)
  - Story marked DONE
