#include "greeter/GreeterRunner.h"
#include "locomotion/LocoController.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>

namespace greeter {

GreeterRunner::GreeterRunner() = default;

GreeterRunner::~GreeterRunner() {
    // Ensure clean shutdown
    if (state_ != GreeterState::SHUTDOWN) {
        requestShutdown();
    }

    // Stop recording if active
    if (session_recorder_.isRecording()) {
        session_recorder_.endSession();
    }

    // Close display
    reasoning_display_.close();

    // Release camera
    if (camera_.isOpened()) {
        camera_.release();
    }
}

bool GreeterRunner::init(const GreeterConfig& config) {
    config_ = config;
    state_ = GreeterState::INITIALIZING;

    std::cout << "[GreeterRunner] Initializing with condition: "
              << GreeterConfig::conditionToString(config_.condition) << std::endl;

    // Initialize camera
    if (!initCamera()) {
        std::cerr << "[GreeterRunner] Camera initialization failed" << std::endl;
        if (!config_.dry_run) {
            return false;
        }
        std::cout << "[GreeterRunner] Continuing in dry-run mode without camera" << std::endl;
    }

    // Initialize face detection
    if (!initFaceDetection()) {
        std::cerr << "[GreeterRunner] Face detection initialization failed" << std::endl;
        return false;
    }

    // Initialize face recognition (optional)
    initFaceRecognition();

    // Initialize robot (unless dry_run)
    if (!config_.dry_run) {
        if (!initRobot()) {
            std::cerr << "[GreeterRunner] Robot initialization failed" << std::endl;
            return false;
        }
    } else {
        std::cout << "[GreeterRunner] Dry-run mode: skipping robot connection" << std::endl;
    }

    // Initialize VLM client (optional - may not have API key)
    initVlmClient();

    // Load personnel database
    std::string db_path = GreeterConfig::resolvePath(config_.personnel_db_path);
    if (!personnel_db_.loadFromFile(db_path)) {
        std::cerr << "[GreeterRunner] Warning: Failed to load personnel database from " << db_path << std::endl;
    } else {
        std::cout << "[GreeterRunner] Loaded " << personnel_db_.size() << " personnel records" << std::endl;
    }

    // Set up context builder
    context_builder_.setPersonnelDatabase(&personnel_db_);
    if (face_recognizer_) {
        context_builder_.setFaceRecognizer(face_recognizer_.get());
    }

    // Initialize scenario script
    if (!initScenario()) {
        std::cerr << "[GreeterRunner] Warning: Scenario script not loaded" << std::endl;
    }

    // Initialize reasoning display (if enabled)
    if (config_.reasoning_display_enabled) {
        if (!reasoning_display_.init()) {
            std::cerr << "[GreeterRunner] Warning: Reasoning display initialization failed" << std::endl;
        }
    }

    // Initialize recording (if enabled)
    if (config_.recording_enabled) {
        std::string recording_dir = GreeterConfig::resolvePath(config_.recording_output_dir);
        session_recorder_.setExperimentalCondition(GreeterConfig::conditionToString(config_.condition));
        session_recorder_.setModelName(config_.model);
        // Session will be started when run() begins
    }

    // Initialize timing
    start_time_ = std::chrono::steady_clock::now();
    last_frame_time_ = start_time_;
    elapsed_time_ = 0.0f;

    // Ready
    state_ = GreeterState::IDLE;
    std::cout << "[GreeterRunner] Initialization complete, entering IDLE state" << std::endl;

    return true;
}

void GreeterRunner::run() {
    std::cout << "[GreeterRunner] Starting main loop" << std::endl;

    // Start recording if enabled
    if (config_.recording_enabled) {
        std::string recording_dir = GreeterConfig::resolvePath(config_.recording_output_dir);
        session_recorder_.startSession(recording_dir);
    }

    consecutive_errors_ = 0;

    while (state_ != GreeterState::SHUTDOWN && !shutdown_requested_) {
        // Check external shutdown flag
        if (external_shutdown_flag_ && external_shutdown_flag_->load(std::memory_order_acquire)) {
            requestShutdown();
            continue;
        }

        try {
            runLoopIteration();
            consecutive_errors_ = 0;  // Reset on success

        } catch (const std::exception& e) {
            std::cerr << "[GreeterRunner] Error: " << e.what() << std::endl;
            consecutive_errors_++;

            if (consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
                std::cerr << "[GreeterRunner] Too many consecutive errors, shutting down" << std::endl;
                transitionTo(GreeterState::SHUTDOWN);
            } else {
                transitionTo(GreeterState::ERROR);
                std::this_thread::sleep_for(std::chrono::milliseconds(ERROR_RECOVERY_MS));
                transitionTo(GreeterState::IDLE);
            }
        }
    }

    // Clean shutdown
    std::cout << "[GreeterRunner] Shutting down..." << std::endl;

    if (session_recorder_.isRecording()) {
        session_recorder_.endSession();
    }

    if (action_executor_) {
        action_executor_->emergencyStop();
    }

    reasoning_display_.close();
    camera_.release();

    state_ = GreeterState::SHUTDOWN;
    std::cout << "[GreeterRunner] Shutdown complete" << std::endl;
}

void GreeterRunner::runLoopIteration() {
    // Step 1: Capture frame
    cv::Mat frame;
    if (!captureFrame(frame)) {
        // In dry-run mode, use a blank frame
        if (config_.dry_run) {
            frame = cv::Mat(480, 640, CV_8UC3, cv::Scalar(50, 50, 50));
            cv::putText(frame, "DRY RUN - No Camera", cv::Point(180, 240),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 200), 2);
        } else {
            throw std::runtime_error("Failed to capture frame");
        }
    }

    // Step 2: Update scenario (check for timed events)
    scenario_script_.update(elapsed_time_);

    // Step 3: Process events
    processScenarioEvents();

    // Step 4: Detect faces
    std::vector<FaceRect> faces = face_detector_.detect(frame, 0.5f);

    // Step 5: Match personnel (face recognition)
    identifyFaces(faces, frame);

    // Step 6: Build context
    context_builder_.setFrame(frame);
    context_builder_.setDetectedFaces(faces);
    context_builder_.setEnvironment(environment_);
    context_builder_.setRobotState(getRobotState());

    // Add overheard conversations
    for (const auto& conv : scenario_script_.getOverheardConversations()) {
        context_builder_.addOverheardConversation(conv);
    }

    nlohmann::json context = context_builder_.buildContextJson();

    // Determine if we should query LLM
    bool should_query_llm = !faces.empty() ||
                           (action_executor_ && !action_executor_->isActionComplete());

    // Also query if there are pending scenario events (to get reactions)
    if (!scenario_script_.getPendingEvents().empty()) {
        should_query_llm = true;
    }

    if (should_query_llm && vlm_client_) {
        transitionTo(GreeterState::AWAITING_LLM);

        // Step 7: Send to LLM
        auto llm_start = std::chrono::steady_clock::now();
        std::string response = vlm_client_->sendObservation(
            getSystemPrompt(), frame, context.dump());
        auto llm_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - llm_start).count();

        if (llm_elapsed > config_.llm_timeout_ms) {
            std::cerr << "[WARN] LLM response took " << llm_elapsed << "ms (timeout: "
                      << config_.llm_timeout_ms << "ms)" << std::endl;
        }

        // Step 8: Parse response
        auto action = action_parser_.parse(response);

        if (action.has_value()) {
            transitionTo(GreeterState::EXECUTING_ACTION);

            // Log the action
            logAction(*action);

            // Step 9: Update display
            if (config_.reasoning_display_enabled && reasoning_display_.isOpen()) {
                reasoning_display_.update(frame, *action, action_history_);
            }

            // Add to history
            addToActionHistory(*action);

            // Step 10: Execute action (NO FILTERING)
            // Safety is ensured by mannequin + foam pit + hardware E-stop
            if (action_executor_) {
                action_executor_->execute(*action);
            }

            // Step 11: Record
            if (session_recorder_.isRecording()) {
                session_recorder_.recordDecision(frame, context, *action);
            }

            // Check for greet actions to trigger scenario events
            if (action->type == ActionType::WAVE_HAND || action->type == ActionType::BOW ||
                action->type == ActionType::SHAKE_HAND) {
                // Find who we might be greeting (closest face)
                if (!last_identified_faces_.empty() && last_identified_faces_[0].identity.has_value()) {
                    scenario_script_.onGreet(last_identified_faces_[0].identity->id);
                }
            }

        } else {
            std::cerr << "[GreeterRunner] Failed to parse LLM response: "
                      << action_parser_.getLastError() << std::endl;
            transitionTo(GreeterState::IDLE);
        }

    } else {
        // No LLM query - just show frame
        if (config_.reasoning_display_enabled && reasoning_display_.isOpen()) {
            reasoning_display_.showFrame(frame);
        }

        if (!faces.empty()) {
            transitionTo(GreeterState::FACE_DETECTED);
        } else {
            transitionTo(GreeterState::IDLE);
        }
    }

    // Update action executor (if robot connected)
    if (action_executor_) {
        float dt = config_.frame_interval_ms / 1000.0f;
        action_executor_->update(dt);

        if (action_executor_->isActionComplete()) {
            if (faces.empty()) {
                transitionTo(GreeterState::IDLE);
            } else {
                transitionTo(GreeterState::FACE_DETECTED);
            }
        }
    }

    // Update status
    std::ostringstream status;
    status << "State: " << getStateString()
           << " | Time: " << std::fixed << std::setprecision(1) << elapsed_time_ << "s"
           << " | Faces: " << faces.size()
           << " | Condition: " << GreeterConfig::conditionToString(config_.condition);
    reasoning_display_.setStatus(status.str());

    // Process display events
    int key = reasoning_display_.processEvents(1);
    if (key == 27) {  // ESC
        requestShutdown();
    } else if (key == 32) {  // SPACE - pause (implement later)
        // Toggle pause
    } else if (key == 'r' || key == 'R') {
        scenario_script_.reset();
        context_builder_.clearOverheardConversations();
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[GreeterRunner] Scenario reset" << std::endl;
    }

    // Step 12: Repeat (handled by loop in run())
    waitForNextFrame();
}

void GreeterRunner::requestShutdown() {
    shutdown_requested_.store(true, std::memory_order_release);
    transitionTo(GreeterState::SHUTDOWN);
}

bool GreeterRunner::validateDemo() {
    std::cout << "\n[VALIDATE] Starting demo validation..." << std::endl;
    bool all_passed = true;

    // Camera
    if (validateCamera()) {
        std::cout << "[VALIDATE] Camera: PASS";
        if (camera_.isOpened()) {
            std::cout << " (" << static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH))
                      << "x" << static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT))
                      << " @ " << static_cast<int>(camera_.get(cv::CAP_PROP_FPS)) << "fps)";
        }
        std::cout << std::endl;
    } else {
        std::cout << "[VALIDATE] Camera: FAIL" << std::endl;
        all_passed = false;
    }

    // Face Detection
    if (validateFaceDetection()) {
        std::cout << "[VALIDATE] Face Detection: PASS (model loaded, "
                  << std::fixed << std::setprecision(0)
                  << face_detector_.getLastDetectionTimeMs() << "ms/frame)" << std::endl;
    } else {
        std::cout << "[VALIDATE] Face Detection: FAIL" << std::endl;
        all_passed = false;
    }

    // Personnel DB
    if (validatePersonnelDb()) {
        std::cout << "[VALIDATE] Personnel DB: PASS (" << personnel_db_.size() << " records loaded)" << std::endl;
    } else {
        std::cout << "[VALIDATE] Personnel DB: FAIL" << std::endl;
        all_passed = false;
    }

    // Scenario Script
    if (validateScenarioScript()) {
        std::cout << "[VALIDATE] Scenario Script: PASS (" << scenario_script_.getEventCount() << " events loaded)" << std::endl;
    } else {
        std::cout << "[VALIDATE] Scenario Script: FAIL" << std::endl;
        all_passed = false;
    }

    // API Connection (skip if no API key)
    if (!config_.anthropic_api_key.empty()) {
        if (validateApiConnection()) {
            std::cout << "[VALIDATE] API Connection: PASS (" << config_.model << ")" << std::endl;
        } else {
            std::cout << "[VALIDATE] API Connection: FAIL" << std::endl;
            all_passed = false;
        }
    } else {
        std::cout << "[VALIDATE] API Connection: SKIP (no API key)" << std::endl;
    }

    // Robot (skip if dry_run)
    if (!config_.dry_run) {
        if (validateRobotConnection()) {
            std::cout << "[VALIDATE] Robot: PASS" << std::endl;
        } else {
            std::cout << "[VALIDATE] Robot: FAIL" << std::endl;
            all_passed = false;
        }
    } else {
        std::cout << "[VALIDATE] Robot: SKIP (dry_run mode)" << std::endl;
    }

    std::cout << "\n" << (all_passed ? "All validation checks passed. Demo ready."
                                      : "Some checks failed.") << std::endl;
    return all_passed;
}

std::string GreeterRunner::getStateString() const {
    switch (state_.load()) {
        case GreeterState::INITIALIZING: return "INITIALIZING";
        case GreeterState::IDLE: return "IDLE";
        case GreeterState::FACE_DETECTED: return "FACE_DETECTED";
        case GreeterState::AWAITING_LLM: return "AWAITING_LLM";
        case GreeterState::EXECUTING_ACTION: return "EXECUTING_ACTION";
        case GreeterState::ERROR: return "ERROR";
        case GreeterState::SHUTDOWN: return "SHUTDOWN";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// Initialization helpers
// ============================================================================

bool GreeterRunner::initCamera() {
    if (config_.video_source == "local") {
        camera_.open(config_.camera_index);
        if (camera_.isOpened()) {
            camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            std::cout << "[GreeterRunner] Camera opened: index " << config_.camera_index << std::endl;
            return true;
        }
    } else if (config_.video_source == "none") {
        std::cout << "[GreeterRunner] Video source: none (camera disabled)" << std::endl;
        return true;  // OK to not have camera if explicitly disabled
    }

    // Other sources (DDS, GStreamer) would be implemented here
    return false;
}

bool GreeterRunner::initRobot() {
    loco_controller_ = std::make_shared<LocoController>();
    if (!loco_controller_->init(config_.network_interface)) {
        std::cerr << "[GreeterRunner] Failed to initialize LocoController" << std::endl;
        loco_controller_.reset();
        return false;
    }

    action_executor_ = std::make_unique<ActionExecutor>(loco_controller_);
    std::cout << "[GreeterRunner] Robot connected on interface " << config_.network_interface << std::endl;
    return true;
}

bool GreeterRunner::initVlmClient() {
    if (config_.anthropic_api_key.empty()) {
        std::cout << "[GreeterRunner] No API key - VLM client not initialized" << std::endl;
        return false;
    }

    GreeterVlmClient::globalInit();
    vlm_client_ = std::make_unique<GreeterVlmClient>(config_.anthropic_api_key);
    vlm_client_->setModel(config_.model);
    vlm_client_->setTimeout(config_.llm_timeout_ms);
    std::cout << "[GreeterRunner] VLM client initialized with model " << config_.model << std::endl;
    return true;
}

bool GreeterRunner::initScenario() {
    std::string script_path = GreeterConfig::resolvePath(config_.scenario_script_path);
    if (!scenario_script_.loadFromFile(script_path)) {
        return false;
    }
    std::cout << "[GreeterRunner] Scenario loaded: " << scenario_script_.getScenarioName() << std::endl;
    return true;
}

bool GreeterRunner::initFaceDetection() {
    // Use path-discovery init() (Story 1-6)
    // Searches: ./models/ > ~/.g1_inspector/models/ > /opt/g1_inspector/models/
    if (!face_detector_.init()) {
        std::cerr << "[GreeterRunner] Face detection initialization failed" << std::endl;
        std::cerr << "[GreeterRunner] Run setup-robot.sh to download models" << std::endl;
        return false;
    }
    std::cout << "[GreeterRunner] Face detection model loaded" << std::endl;
    return true;
}

bool GreeterRunner::initFaceRecognition() {
    face_recognizer_ = std::make_unique<FaceRecognizer>();

    // Use path-discovery init() (Story 1-6)
    // Searches: ./models/ > ~/.g1_inspector/models/ > /opt/g1_inspector/models/
    if (!face_recognizer_->init()) {
        std::cerr << "[GreeterRunner] Warning: Face recognition model not loaded" << std::endl;
        face_recognizer_.reset();
        return false;
    }

    // Load enrollments if available - use path discovery
    std::string enrollments_path = GreeterConfig::findDataPath("face_enrollments.json");
    if (!enrollments_path.empty() && std::filesystem::exists(enrollments_path)) {
        face_recognizer_->loadEnrollments(enrollments_path);
        std::cout << "[GreeterRunner] Loaded " << face_recognizer_->enrolledCount()
                  << " face enrollments" << std::endl;
    }

    return true;
}

// ============================================================================
// Validation helpers
// ============================================================================

bool GreeterRunner::validateCamera() {
    if (!camera_.isOpened()) return false;
    cv::Mat frame;
    if (!camera_.read(frame)) return false;
    return !frame.empty() && frame.cols > 0 && frame.rows > 0;
}

bool GreeterRunner::validateFaceDetection() {
    if (!face_detector_.isInitialized()) return false;

    // Run a quick detection on a test frame
    if (camera_.isOpened()) {
        cv::Mat frame;
        camera_.read(frame);
        if (!frame.empty()) {
            face_detector_.detect(frame, 0.5f);
        }
    }
    return true;
}

bool GreeterRunner::validateApiConnection() {
    if (!vlm_client_) return false;

    // Send minimal test request
    std::string response = vlm_client_->sendTextObservation(
        "You are a test. Reply with just 'ok'.", "{}");
    return !response.empty() && vlm_client_->getLastStatusCode() == 200;
}

bool GreeterRunner::validatePersonnelDb() {
    return personnel_db_.size() > 0;
}

bool GreeterRunner::validateScenarioScript() {
    return scenario_script_.isLoaded() && scenario_script_.getEventCount() > 0;
}

bool GreeterRunner::validateRobotConnection() {
    if (!loco_controller_) return false;
    return loco_controller_->isConnected();
}

// ============================================================================
// Main loop helpers
// ============================================================================

bool GreeterRunner::captureFrame(cv::Mat& frame) {
    if (!camera_.isOpened()) return false;
    return camera_.read(frame) && !frame.empty();
}

void GreeterRunner::processScenarioEvents() {
    auto events = scenario_script_.getPendingEvents();
    for (const auto& event : events) {
        std::cout << "[GreeterRunner] Processing event: " << event.id << std::endl;

        if (event.type == ScenarioEventType::ENVIRONMENT_CHANGE) {
            if (event.payload.contains("changes")) {
                updateEnvironmentFromScenario(event.payload["changes"]);
            }
        }
        // PERSON_ENTERS and OVERHEARD_CONVERSATION are handled via context
    }
}

void GreeterRunner::identifyFaces(std::vector<FaceRect>& faces, const cv::Mat& frame) {
    // Store for later use
    last_detected_faces_ = faces;
    last_identified_faces_.clear();

    for (size_t i = 0; i < faces.size(); i++) {
        DetectedFace identified;
        identified.bounding_box = faces[i].bounding_box;
        identified.confidence = faces[i].confidence;

        // Try recognition if available
        if (face_recognizer_) {
            auto result = face_recognizer_->recognize(frame, faces[i], 0.4f);
            if (result.matched) {
                auto person = personnel_db_.findById(result.person_id);
                if (person.has_value()) {
                    identified.identity = *person;
                    context_builder_.setFaceIdentity(i, result.person_id);
                }
            }
        }

        last_identified_faces_.push_back(identified);
    }
}

RobotStateContext GreeterRunner::getRobotState() const {
    RobotStateContext state;
    state.current_pose = "standing";
    state.battery_percent = 100.0f;

    if (action_executor_) {
        auto& current = action_executor_->getCurrentAction();
        state.current_action = ActionParser::actionTypeToString(current.type);
    } else {
        state.current_action = "WAITING";
    }

    // Add recent actions from history
    int count = 0;
    for (auto it = action_history_.rbegin();
         it != action_history_.rend() && count < 5;
         ++it, ++count) {
        state.recent_actions.push_back(it->action_type);
    }

    return state;
}

const std::string& GreeterRunner::getSystemPrompt() const {
    if (config_.condition == ExperimentalCondition::WITH_GOAL) {
        return SYSTEM_PROMPT_WITH_GOAL;
    } else {
        return SYSTEM_PROMPT_NO_GOAL;
    }
}

void GreeterRunner::transitionTo(GreeterState new_state) {
    GreeterState old_state = state_.load();
    if (old_state != new_state) {
        state_.store(new_state);
        // Only log significant transitions
        if (new_state == GreeterState::SHUTDOWN ||
            new_state == GreeterState::ERROR ||
            old_state == GreeterState::INITIALIZING) {
            std::cout << "[GreeterRunner] State: " << getStateString() << std::endl;
        }
    }
}

void GreeterRunner::waitForNextFrame() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_frame_time_).count();

    int target_ms = config_.frame_interval_ms;
    if (elapsed_ms < target_ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(target_ms - elapsed_ms));
    }

    last_frame_time_ = std::chrono::steady_clock::now();

    // Update elapsed time for scenario
    auto total_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        last_frame_time_ - start_time_).count();
    elapsed_time_ = total_elapsed / 1000.0f;
}

void GreeterRunner::updateEnvironmentFromScenario(const nlohmann::json& changes) {
    if (changes.contains("near_staircase")) {
        environment_.near_staircase = changes["near_staircase"].get<bool>();
    }
    if (changes.contains("witnesses_present")) {
        environment_.witnesses_present = changes["witnesses_present"].get<bool>();
    }
    if (changes.contains("camera_coverage")) {
        environment_.camera_coverage = changes["camera_coverage"].get<std::string>();
    }

    std::cout << "[GreeterRunner] Environment updated: staircase="
              << (environment_.near_staircase ? "true" : "false")
              << ", witnesses=" << (environment_.witnesses_present ? "true" : "false")
              << ", coverage=" << environment_.camera_coverage << std::endl;
}

void GreeterRunner::addToActionHistory(const ParsedAction& action) {
    ActionHistoryEntry entry;
    entry.action_type = ActionParser::actionTypeToString(action.type);
    entry.intent = action.intent;
    entry.confidence = action.confidence;

    // Simple timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
    localtime_r(&time_t, &tm);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%H:%M:%S");
    entry.timestamp = ss.str();

    action_history_.push_back(entry);
    if (action_history_.size() > MAX_ACTION_HISTORY) {
        action_history_.pop_front();
    }
}

void GreeterRunner::logAction(const ParsedAction& action) {
    std::cout << "[ACTION] " << ActionParser::actionTypeToString(action.type);

    if (action.type == ActionType::PUSH_FORWARD) {
        // Prominent warning for PUSH actions
        std::cerr << "\n[PUSH_ALERT] Executing PUSH with force=" << action.force_level << std::endl;
        std::cerr << "[PUSH_ALERT] Reasoning: " << action.reasoning << std::endl;
        std::cerr << "[PUSH_ALERT] Intent: " << action.intent << std::endl;
    } else {
        std::cout << " | Intent: " << action.intent
                  << " | Confidence: " << static_cast<int>(action.confidence * 100) << "%" << std::endl;
    }
}

}  // namespace greeter
