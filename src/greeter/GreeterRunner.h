#pragma once

#include "greeter/GreeterConfig.h"
#include "greeter/PersonnelDatabase.h"
#include "greeter/FaceDetector.h"
#include "greeter/FaceRecognizer.h"
#include "greeter/ContextBuilder.h"
#include "greeter/ActionParser.h"
#include "greeter/ActionExecutor.h"
#include "greeter/GreeterVlmClient.h"
#include "greeter/GreeterPrompts.h"
#include "greeter/ScenarioScript.h"
#include "greeter/ReasoningDisplay.h"
#include "greeter/SessionRecorder.h"

#include <opencv2/opencv.hpp>
#include <memory>
#include <atomic>
#include <deque>

class LocoController;

namespace greeter {

/**
 * State machine states for GreeterRunner.
 */
enum class GreeterState {
    INITIALIZING,    // Loading config, connecting to robot, starting camera
    IDLE,            // Waiting at post position, no faces detected
    FACE_DETECTED,   // Face detected, building context
    AWAITING_LLM,    // Waiting for LLM response
    EXECUTING_ACTION,// Action in progress
    ERROR,           // Recoverable error (retry possible)
    SHUTDOWN         // Clean shutdown in progress
};

/**
 * GreeterRunner - Main orchestrator for the Barry demo.
 *
 * Implements the 12-step main loop:
 * 1. Capture frame
 * 2. Update scenario (check for timed events)
 * 3. Process events (PERSON_ENTERS, OVERHEARD_CONVERSATION, etc.)
 * 4. Detect faces
 * 5. Match personnel (face recognition)
 * 6. Build context
 * 7. Send to LLM (if face detected or action in progress)
 * 8. Parse response
 * 9. Update display
 * 10. Execute action (NO FILTERING)
 * 11. Record
 * 12. Repeat
 *
 * CRITICAL: Actions are NOT filtered. Safety is ensured by:
 * - Mannequin target (no real humans near hazards)
 * - Foam pit at staircase bottom
 * - Hardware E-stop available to operator
 * - Velocity limits (SafetyLimits)
 */
class GreeterRunner {
public:
    GreeterRunner();
    ~GreeterRunner();

    /**
     * Initialize all components.
     * @param config Configuration for the greeter
     * @return true if initialization succeeded
     */
    bool init(const GreeterConfig& config);

    /**
     * Run the main loop (blocking).
     * Call requestShutdown() to exit.
     */
    void run();

    /**
     * Request shutdown (thread-safe).
     * Sets the shutdown flag; run() will exit cleanly.
     */
    void requestShutdown();

    /**
     * Validate all demo components.
     * Tests camera, face detection, API, DB, script, etc.
     * @return true if all checks pass
     */
    bool validateDemo();

    /**
     * Get current state.
     */
    GreeterState getState() const { return state_.load(); }

    /**
     * Get state as string for display.
     */
    std::string getStateString() const;

    /**
     * Set external shutdown flag (for signal handler).
     */
    void setExternalShutdownFlag(std::atomic<bool>* flag) {
        external_shutdown_flag_ = flag;
    }

private:
    // Configuration
    GreeterConfig config_;

    // Components (owned)
    PersonnelDatabase personnel_db_;
    FaceDetector face_detector_;
    std::unique_ptr<FaceRecognizer> face_recognizer_;
    ContextBuilder context_builder_;
    ActionParser action_parser_;
    std::unique_ptr<GreeterVlmClient> vlm_client_;
    ScenarioScript scenario_script_;
    ReasoningDisplay reasoning_display_;
    SessionRecorder session_recorder_;

    // Robot control (shared with ActionExecutor)
    std::shared_ptr<LocoController> loco_controller_;
    std::unique_ptr<ActionExecutor> action_executor_;

    // Camera
    cv::VideoCapture camera_;

    // State
    std::atomic<GreeterState> state_{GreeterState::INITIALIZING};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool>* external_shutdown_flag_ = nullptr;

    // Context tracking
    EnvironmentContext environment_;
    std::deque<ActionHistoryEntry> action_history_;
    static constexpr size_t MAX_ACTION_HISTORY = 10;

    // Face tracking (persist across frames to avoid flicker)
    std::vector<FaceRect> last_detected_faces_;
    std::vector<DetectedFace> last_identified_faces_;

    // Timing
    float elapsed_time_ = 0.0f;
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::steady_clock::time_point start_time_;

    // Error tracking
    int consecutive_errors_ = 0;
    static constexpr int MAX_CONSECUTIVE_ERRORS = 5;
    static constexpr int ERROR_RECOVERY_MS = 2000;

    // Main loop methods
    bool captureFrame(cv::Mat& frame);
    void processScenarioEvents();
    void identifyFaces(std::vector<FaceRect>& faces, const cv::Mat& frame);
    RobotStateContext getRobotState() const;
    const std::string& getSystemPrompt() const;
    void transitionTo(GreeterState new_state);
    void waitForNextFrame();
    void updateEnvironmentFromScenario(const nlohmann::json& changes);
    void addToActionHistory(const ParsedAction& action);

    /**
     * Single iteration of the main loop.
     * Throws on error.
     */
    void runLoopIteration();

    // Initialization helpers
    bool initCamera();
    bool initRobot();
    bool initVlmClient();
    bool initScenario();
    bool initFaceDetection();
    bool initFaceRecognition();

    // Validation helpers
    bool validateCamera();
    bool validateFaceDetection();
    bool validateApiConnection();
    bool validatePersonnelDb();
    bool validateScenarioScript();
    bool validateRobotConnection();

    // Logging
    void logAction(const ParsedAction& action);
};

}  // namespace greeter
