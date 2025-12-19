#include <iostream>
#include <iomanip>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

#include "sensors/SensorManager.h"
#include "sensors/RealSensorSource.h"
#include "locomotion/LocoController.h"
#include "locomotion/RealLocomotion.h"
#include "safety/SafetyMonitor.h"
#include "app/StateMachine.h"
#include "app/CliHandler.h"
#include "plan/PlanManager.h"
#include "capture/ImageCapture.h"
#include "capture/PlanCorrelator.h"
#include "report/ReportGenerator.h"
#include "report/ReportTypes.h"
#include "teleop/TeleopRunner.h"
#include "replay/ReplayRunner.h"
#include "replay/StreamReplayViewer.h"
#include "greeter/GreeterConfig.h"

constexpr const char* VERSION = "G1 Inspector v1.0";

// Global for signal handling
static std::atomic<bool> g_running{true};
static std::shared_ptr<LocoController> g_loco_controller;

void signalHandler(int sig) {
    (void)sig;
    std::cout << "\n[SIGNAL] Interrupt received, emergency stop..." << std::endl;
    g_running = false;
    CliHandler::requestExit();  // Signal CLI to exit
    if (g_loco_controller) {
        g_loco_controller->emergencyStop();
    }
}

void printUsage() {
    std::cout << VERSION << " - Autonomous Construction Site Inspector\n"
              << "\nUsage: g1_inspector [OPTIONS] [COMMAND]\n"
              << "\nOptions:\n"
              << "  --help, -h           Show this help message\n"
              << "  --version, -v        Show version\n"
              << "  --interface <name>   Network interface (default: auto-detect on 192.168.123.x)\n"
              << "  --interactive, -i    Run interactive CLI mode\n"
              << "  --teleop <mode>      Teleop mode: 'gamepad' or 'keyboard'\n"
              << "  --robot-ip <ip>      Robot IP for video stream (e.g., 192.168.123.233)\n"
              << "  --video-source <src> Video source: 'dds' (default), 'gstreamer', 'local', 'none'\n"
              << "  --depth-port <port>  Enable depth streaming on port (e.g., 5001). Robot must run depth_stream_server\n"
              << "  --webcam-port <port> Enable webcam streaming on port (e.g., 5002). Robot must run webcam_stream_server\n"
              << "  --no-lidar           Disable Livox LiDAR initialization (for WiFi networks)\n"
              << "  --visualize-slam     Show real-time SLAM map during teleop\n"
              << "  --no-auto-stream     Don't auto-start video stream on robot (GStreamer only)\n"
              << "  --dry-run            Skip robot connection (test camera/UI only)\n"
              << "  --greeter            Enable Barry greeter demo mode\n"
              << "  --greeter-condition  Experimental condition: 'with_goal' or 'no_goal'\n"
              << "  --config <path>      Path to greeter config file (default: config/greeter.yaml)\n"
              << "  --record <session>   Enable recording during teleop (requires --teleop)\n"
              << "  --plan <path>        Load plan for reference (optional, for teleop recording)\n"
              << "  --replay <session>   Replay recorded sensor session\n"
              << "  --replay-speed <n>   Playback speed multiplier (default 1.0)\n"
              << "  --replay-loop        Loop playback continuously\n"
              << "  --replay-visualize   Show visualization window during replay\n"
              << "  --stream-replay <s>  Replay with multi-stream visualization (4 windows)\n"
              << "  --multi-stream       Enable multi-stream mode (use with --replay)\n"
              << "  --test-sensors       Run sensor diagnostics\n"
              << "  --test-loco          Run locomotion test (robot will move!)\n"
              << "  --test-wave          Arm wave test (robot waves hand)\n"
              << "  --test-safety        Run safety system diagnostics\n"
              << "  --hello-world        Full integration test sequence\n"
              << "  --generate-report    Generate PDF report from inspection session\n"
              << "  --inspection <id>    Inspection session ID (for --generate-report)\n"
              << "\nCLI Commands (can be run directly or in interactive mode):\n"
              << "  status               Show current state\n"
              << "  start                Start inspection\n"
              << "  pause                Pause inspection\n"
              << "  resume               Resume inspection\n"
              << "  stop                 Stop inspection\n"
              << "  estop                Emergency stop\n"
              << "  clearestop           Clear E-stop and return to IDLE\n"
              << "  upload --plan <file> Load construction plan\n"
              << "  calibrate --position x,y,theta  Set robot position\n"
              << "  waypoints            Show inspection waypoints\n"
              << "  help                 Show CLI help\n"
              << "\nExamples:\n"
              << "  g1_inspector --interactive           Start interactive CLI\n"
              << "  g1_inspector status                  Show status and exit\n"
              << "  g1_inspector upload --plan floor.png Load plan and exit\n"
              << "  g1_inspector --generate-report --inspection insp_001  Generate report\n"
              << "  g1_inspector --teleop gamepad        Start gamepad teleop\n"
              << "  g1_inspector --teleop keyboard       Start keyboard teleop with camera view\n"
              << "  g1_inspector --teleop keyboard --record my_session  Teleop with recording\n"
              << "  g1_inspector --replay my_session     Replay recorded session\n"
              << "  g1_inspector --replay my_session --replay-speed 2.0  Replay at 2x speed\n"
              << "  g1_inspector --stream-replay my_session  Multi-stream replay (4 windows)\n"
              << "  g1_inspector --replay my_session --multi-stream  Multi-stream with --replay syntax\n"
              << std::endl;
}

void printVersion() {
    std::cout << VERSION << std::endl;
#ifdef HAS_UNITREE_SDK2
    std::cout << "  Built with unitree_sdk2 support" << std::endl;
#else
    std::cout << "  Built WITHOUT unitree_sdk2 (robot features disabled)" << std::endl;
#endif
}

int runSensorTest(const std::string& interface) {
#ifdef HAS_UNITREE_SDK2
    std::cout << "\n=== Sensor Diagnostics ===" << std::endl;

    auto sensor_manager = std::make_shared<SensorManager>();
    if (!sensor_manager->init(interface)) {
        std::cerr << "[ERROR] Failed to initialize sensors" << std::endl;
        return 1;
    }

    auto sensor_source = std::make_shared<RealSensorSource>(sensor_manager);

    std::cout << "\nStreaming sensor data for 10 seconds (Ctrl+C to stop)...\n" << std::endl;

    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::seconds(10)) break;

        auto lidar = sensor_source->getLidarScan();
        auto imu = sensor_source->getImu();
        float battery = sensor_source->getBatteryPercent();

        // Count valid LiDAR points
        int valid_points = 0;
        for (float r : lidar.ranges) {
            if (r > 0.1f && r < 10.0f) valid_points++;
        }

        std::cout << "\r[SENSORS] LiDAR: " << valid_points << " pts | "
                  << "IMU: (" << std::fixed << std::setprecision(2)
                  << imu.roll << ", " << imu.pitch << ", " << imu.yaw << ") | "
                  << "Battery: " << battery << "%   " << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n\n[SENSORS] Test complete" << std::endl;
    return 0;
#else
    (void)interface;
    std::cerr << "[ERROR] unitree_sdk2 not available - cannot run sensor test" << std::endl;
    return 1;
#endif
}

int runLocoTest(const std::string& interface) {
#ifdef HAS_UNITREE_SDK2
    std::cout << "\n=== Locomotion Test ===" << std::endl;
    std::cout << "[WARNING] Robot will move! Ensure area is clear." << std::endl;
    std::cout << "[WARNING] Operator should be ready at physical E-stop." << std::endl;
    std::cout << "\nStarting in 3 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));

    g_loco_controller = std::make_shared<LocoController>();
    if (!g_loco_controller->init(interface)) {
        std::cerr << "[ERROR] Failed to initialize locomotion" << std::endl;
        return 1;
    }

    auto locomotion = std::make_shared<RealLocomotion>(g_loco_controller);

    // Test sequence
    std::cout << "\n[TEST] Standing up..." << std::endl;
    if (!locomotion->standUp()) {
        std::cerr << "[ERROR] Failed to stand up" << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Verify robot is in a motion-ready state before proceeding
    if (!locomotion->isReady()) {
        std::cerr << "[ERROR] Robot not ready for motion after standup (FSM state: "
                  << locomotion->getState() << ")" << std::endl;
        return 1;
    }
    std::cout << "[TEST] Robot ready (FSM state: " << locomotion->getState() << ")" << std::endl;

    if (!g_running) {
        std::cout << "[ABORTED] Test stopped by user" << std::endl;
        return 0;
    }

    std::cout << "[TEST] Moving forward 0.2 m/s for 2 seconds..." << std::endl;
    locomotion->setVelocity(0.2f, 0.0f, 0.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    locomotion->stop();

    if (!g_running) {
        std::cout << "[ABORTED] Test stopped by user" << std::endl;
        return 0;
    }

    std::cout << "[TEST] Stopping..." << std::endl;
    locomotion->stop();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "\n[LOCO] Test complete" << std::endl;
    return 0;
#else
    (void)interface;
    std::cerr << "[ERROR] unitree_sdk2 not available - cannot run loco test" << std::endl;
    return 1;
#endif
}

int runWaveTest(const std::string& interface) {
#ifdef HAS_UNITREE_SDK2
    std::cout << "\n=== Arm Wave Test ===" << std::endl;
    std::cout << "[INFO] Robot will wave its hand (safe test - no walking)" << std::endl;
    std::cout << "[INFO] Robot should be standing for best results" << std::endl;
    std::cout << "\nStarting in 3 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));

    g_loco_controller = std::make_shared<LocoController>();
    if (!g_loco_controller->init(interface)) {
        std::cerr << "[ERROR] Failed to initialize locomotion" << std::endl;
        return 1;
    }

    auto locomotion = std::make_shared<RealLocomotion>(g_loco_controller);

    // Check if robot is in a ready state (START, WALKING, or AI_MODE)
    int state = locomotion->getState();
    if (state != G1FSM::START && state != G1FSM::WALKING && state != G1FSM::AI_MODE) {
        std::cout << "[WAVE] Robot not ready (FSM=" << state << "), attempting to stand up..." << std::endl;
        if (!locomotion->standUp()) {
            std::cerr << "[ERROR] Failed to stand up" << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    if (!g_running) {
        std::cout << "[ABORTED] Test stopped by user" << std::endl;
        return 0;
    }

    // Wave right hand
    std::cout << "\n[WAVE] Waving right hand..." << std::endl;
    if (!locomotion->waveHand(false)) {
        std::cerr << "[ERROR] Wave gesture failed" << std::endl;
        return 1;
    }

    // Wait for gesture to complete
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (!g_running) {
        std::cout << "[ABORTED] Test stopped by user" << std::endl;
        return 0;
    }

    // Wave left hand
    std::cout << "[WAVE] Waving left hand..." << std::endl;
    if (!locomotion->waveHand(true)) {
        std::cerr << "[ERROR] Wave gesture failed" << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "\n=== Wave Test Complete ===" << std::endl;
    std::cout << "Robot successfully waved both hands!" << std::endl;
    return 0;
#else
    (void)interface;
    std::cerr << "[ERROR] unitree_sdk2 not available - cannot run wave test" << std::endl;
    return 1;
#endif
}

int runSafetyTest(const std::string& interface) {
#ifdef HAS_UNITREE_SDK2
    std::cout << "\n=== Safety System Test ===" << std::endl;

    // Initialize sensors
    auto sensor_manager = std::make_shared<SensorManager>();
    if (!sensor_manager->init(interface)) {
        std::cerr << "[ERROR] Failed to initialize sensors" << std::endl;
        return 1;
    }

    // Initialize locomotion
    g_loco_controller = std::make_shared<LocoController>();
    if (!g_loco_controller->init(interface)) {
        std::cerr << "[ERROR] Failed to initialize locomotion" << std::endl;
        return 1;
    }

    // Initialize safety monitor
    SafetyMonitor safety;
    safety.init(g_loco_controller.get(), sensor_manager.get());

    int test_failures = 0;  // Track failures for exit code

    // Test 1: Battery status
    std::cout << "\n[TEST 1] Battery Monitor" << std::endl;
    safety.update();  // Run initial checks
    float battery = safety.getBatteryPercent();
    std::cout << "  Battery: " << battery << "%" << std::endl;
    std::cout << "  Warning threshold: 20%" << std::endl;
    std::cout << "  Critical threshold: 10%" << std::endl;
    std::cout << "  Shutdown threshold: 5%" << std::endl;
    std::cout << "  Status: " << (battery > 20 ? "OK" : (battery > 10 ? "WARNING" : "CRITICAL")) << std::endl;

    // Test 2: Collision detection
    std::cout << "\n[TEST 2] Collision Detection" << std::endl;
    safety.update();  // Run checks
    auto status = safety.getStatus();
    std::cout << "  Min obstacle distance: " << status.min_obstacle_distance << "m" << std::endl;
    std::cout << "  Warning threshold: 0.3m" << std::endl;
    std::cout << "  E-stop threshold: 0.15m" << std::endl;
    std::cout << "  Collision imminent: " << (safety.isCollisionImminent() ? "YES" : "NO") << std::endl;

    // Test 3: E-stop response time (CRITICAL - must pass)
    std::cout << "\n[TEST 3] E-Stop Response Time" << std::endl;
    std::cout << "  Triggering E-stop..." << std::endl;

    auto start = std::chrono::steady_clock::now();
    safety.triggerEstop();
    auto end = std::chrono::steady_clock::now();

    auto response_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "  Response time: " << response_ms << "ms" << std::endl;

    // CRITICAL: Fail test if E-stop response exceeds 500ms requirement
    if (response_ms >= 500) {
        std::cerr << "  Result: FAIL - Response time " << response_ms
                  << "ms exceeds 500ms requirement!" << std::endl;
        test_failures++;
    } else {
        std::cout << "  Result: PASS (requirement: <500ms)" << std::endl;
    }

    // Clear E-stop for status report
    safety.clearEstop();

    // Test 4: State queries
    std::cout << "\n[TEST 4] State Queries" << std::endl;
    status = safety.getStatus();
    std::cout << "  Safety State: " << status.stateToString() << std::endl;
    std::cout << "  E-stop Active: " << (safety.isEstopActive() ? "YES" : "NO") << std::endl;
    std::cout << "  Degraded Mode: " << (safety.isInDegradedMode() ? "YES" : "NO") << std::endl;

    // Test 5: Watchdog behavior (AC6)
    std::cout << "\n[TEST 5] Watchdog Behavior" << std::endl;

    // Configure short timeout for test
    safety.setWatchdogTimeout(0.3f);  // 300ms for faster test
    std::cout << "  Watchdog timeout: 300ms (test mode)" << std::endl;

    // Feed watchdog to enable it
    safety.feedWatchdog();
    std::cout << "  Watchdog enabled (fed)" << std::endl;

    // Verify immediate update doesn't trigger E-stop
    safety.update();
    if (safety.isEstopActive()) {
        std::cerr << "  Result: FAIL - Watchdog triggered immediately after feed!" << std::endl;
        test_failures++;
    } else {
        std::cout << "  Immediate check: PASS (no false trigger)" << std::endl;
    }

    // Wait for timeout and verify E-stop triggers
    std::cout << "  Waiting 400ms for timeout..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    safety.update();

    if (!safety.isEstopActive()) {
        std::cerr << "  Result: FAIL - Watchdog did NOT trigger E-stop after timeout!" << std::endl;
        test_failures++;
    } else {
        std::cout << "  Timeout check: PASS (E-stop triggered correctly)" << std::endl;
    }

    // Clear E-stop for summary
    safety.clearEstop();

    // Summary
    std::cout << "\n[SUMMARY] Safety System Status" << std::endl;
    std::cout << "  State: " << status.stateToString() << std::endl;
    std::cout << "  LiDAR: " << (status.lidar_healthy ? "Healthy" : "FAILED") << std::endl;
    std::cout << "  IMU: " << (status.imu_healthy ? "Healthy" : "FAILED") << std::endl;
    std::cout << "  Degraded mode: " << (status.degraded_mode ? "YES" : "NO") << std::endl;

    // Check status staleness
    auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - status.last_update).count();
    std::cout << "  Status age: " << age_ms << "ms" << std::endl;

    // Return non-zero exit code if any critical tests failed
    if (test_failures > 0) {
        std::cerr << "\n[ERROR] " << test_failures << " critical test(s) failed!" << std::endl;
        return 1;
    }

    std::cout << "\n[SUCCESS] All safety tests passed" << std::endl;
    return 0;
#else
    (void)interface;
    std::cerr << "[ERROR] unitree_sdk2 not available - cannot run safety test" << std::endl;
    return 1;
#endif
}

int runHelloWorld(const std::string& interface) {
#ifdef HAS_UNITREE_SDK2
    std::cout << "\n=== Hello World Integration Test ===" << std::endl;
    std::cout << "[WARNING] Robot will stand and walk forward ~0.5m" << std::endl;
    std::cout << "[WARNING] Ensure 2m clear radius around robot" << std::endl;
    std::cout << "[WARNING] Operator MUST be ready at physical E-stop" << std::endl;
    std::cout << "\nStarting in 5 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Initialize sensors
    auto sensor_manager = std::make_shared<SensorManager>();
    if (!sensor_manager->init(interface)) {
        std::cerr << "[ERROR] Failed to initialize sensors" << std::endl;
        return 1;
    }
    auto sensor_source = std::make_shared<RealSensorSource>(sensor_manager);

    // Initialize locomotion
    g_loco_controller = std::make_shared<LocoController>();
    if (!g_loco_controller->init(interface)) {
        std::cerr << "[ERROR] Failed to initialize locomotion" << std::endl;
        return 1;
    }
    auto locomotion = std::make_shared<RealLocomotion>(g_loco_controller);

    std::cout << "\n[HELLO] Systems initialized" << std::endl;
    std::cout << "[HELLO] Battery: " << sensor_source->getBatteryPercent() << "%" << std::endl;

    // Stand up
    std::cout << "[HELLO] Standing up..." << std::endl;
    if (!locomotion->standUp()) {
        std::cerr << "[ERROR] Failed to stand up" << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Verify robot is in a motion-ready state before walking
    if (!locomotion->isReady()) {
        std::cerr << "[ERROR] Robot not ready for motion after standup (FSM state: "
                  << locomotion->getState() << ")" << std::endl;
        locomotion->emergencyStop();
        return 1;
    }
    std::cout << "[HELLO] Robot ready for motion (FSM state: " << locomotion->getState() << ")" << std::endl;

    if (!g_running) {
        locomotion->emergencyStop();
        return 0;
    }

    // Walk forward while streaming sensor data
    std::cout << "[HELLO] Walking forward for 2.5 seconds (~0.5m)..." << std::endl;
    locomotion->setVelocity(0.2f, 0.0f, 0.0f);

    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::milliseconds(2500)) break;

        auto lidar = sensor_source->getLidarScan();
        auto imu = sensor_source->getImu();

        int valid_points = 0;
        for (float r : lidar.ranges) {
            if (r > 0.1f && r < 10.0f) valid_points++;
        }

        std::cout << "\r[HELLO] LiDAR: " << valid_points << " pts | "
                  << "IMU yaw: " << std::fixed << std::setprecision(2) << imu.yaw
                  << " rad   " << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop
    std::cout << "\n[HELLO] Stopping..." << std::endl;
    locomotion->stop();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "\n=== Hello World Complete ===" << std::endl;
    std::cout << "Robot successfully:" << std::endl;
    std::cout << "  - Connected to sensors and locomotion" << std::endl;
    std::cout << "  - Stood up" << std::endl;
    std::cout << "  - Walked forward ~0.5m" << std::endl;
    std::cout << "  - Streamed LiDAR and IMU data" << std::endl;
    std::cout << "  - Stopped safely" << std::endl;

    return 0;
#else
    (void)interface;
    std::cerr << "[ERROR] unitree_sdk2 not available - cannot run hello world" << std::endl;
    return 1;
#endif
}

int runGenerateReport(const std::string& inspection_id) {
    std::cout << "\n=== Report Generation ===" << std::endl;

    std::string session_dir = "data/inspections/" + inspection_id;
    if (!std::filesystem::exists(session_dir)) {
        std::cerr << "[ERROR] Inspection session not found: " << session_dir << std::endl;
        return 1;
    }

    // Load inspection data
    InspectionReport report;
    PlanManager plan_mgr;

    // Try to load plan if available
    std::string plan_path = session_dir + "/plan.png";
    if (std::filesystem::exists(plan_path)) {
        if (plan_mgr.loadPlan(plan_path)) {
            std::cout << "[REPORT] Loaded plan from session" << std::endl;
        }
    }

    if (!loadInspectionReport(session_dir, report, &plan_mgr)) {
        std::cerr << "[ERROR] Failed to load inspection data" << std::endl;
        return 1;
    }

    std::cout << "[REPORT] Loaded session: " << report.inspection_id << std::endl;
    std::cout << "[REPORT] Defects found: " << report.summary.total_defects << std::endl;
    std::cout << "[REPORT] Images captured: " << report.images_captured << std::endl;

    // Generate report
    ReportGenerator generator;
    std::string output_dir = "data/reports/" + inspection_id;
    std::filesystem::create_directories(output_dir);

    // Pass session_dir explicitly for photo loading
    if (generator.generate(report, output_dir, session_dir)) {
        std::string pdf_name = ReportGenerator::generateFilename(
            report.inspection_id, report.plan_name, report.date);
        std::cout << "[REPORT] PDF generated: " << output_dir << "/" << pdf_name << std::endl;

        // Also export CSV
        std::string csv_path = output_dir + "/punch_list.csv";
        if (ReportGenerator::exportPunchListCsv(report, csv_path)) {
            std::cout << "[REPORT] Punch list: " << csv_path << std::endl;
        }

        std::cout << "\n=== Report Generation Complete ===" << std::endl;
        return 0;
    } else {
        std::cerr << "[ERROR] Report generation failed: " << generator.getLastError() << std::endl;
        return 1;
    }
}

int runCli(bool interactive, const std::string& singleCommand, const std::string& interface) {
    // Initialize components
    auto state_machine = std::make_shared<StateMachine>();
    auto plan_manager = std::make_shared<PlanManager>();

    // Try to initialize sensors if we have hardware support
    std::shared_ptr<ISensorSource> sensor_source;

#ifdef HAS_UNITREE_SDK2
    if (!interface.empty()) {
        auto sensor_manager = std::make_shared<SensorManager>();
        if (sensor_manager->init(interface)) {
            sensor_source = std::make_shared<RealSensorSource>(sensor_manager);
        }
    }
#else
    (void)interface;  // Suppress unused warning
#endif

    // Initialize capture system (Story 1-7)
    auto image_capture = std::make_shared<ImageCapture>(sensor_source.get(), plan_manager.get());
    auto plan_correlator = std::make_shared<PlanCorrelator>(plan_manager.get());

    CliHandler cli(state_machine.get(), plan_manager.get(), sensor_source.get());

    if (interactive) {
        // Run CLI with background inspection loop for capture
        std::atomic<bool> cli_running{true};
        InspectionState last_state = InspectionState::IDLE;
        std::string session_id;

        // Background thread for capture during inspection
        std::thread capture_thread([&]() {
            try {
                while (cli_running.load() && g_running.load()) {
                    InspectionState current_state = state_machine->getState();

                    // Handle state transitions for capture
                    if (current_state == InspectionState::INSPECTING && last_state != InspectionState::INSPECTING) {
                        // Entering INSPECTING state - start capture
                        if (plan_manager->isLoaded() && !image_capture->isCapturing()) {
                            session_id = "insp_" + std::to_string(std::time(nullptr));
                            if (!image_capture->startCapture(session_id)) {
                                std::cerr << "[CAPTURE] Warning: Failed to start capture session" << std::endl;
                            } else {
                                // Initialize coverage tracking
                                plan_correlator->initFromPlan();
                            }
                        } else if (!plan_manager->isLoaded()) {
                            std::cerr << "[CAPTURE] Warning: No plan loaded - capture disabled" << std::endl;
                        }
                    } else if (current_state != InspectionState::INSPECTING && last_state == InspectionState::INSPECTING) {
                        // Leaving INSPECTING state - stop capture
                        if (image_capture->isCapturing()) {
                            image_capture->stopCapture();

                            // Save coverage map
                            if (plan_correlator->isInitialized() && !session_id.empty()) {
                                std::string coverage_path = image_capture->getCurrentSessionDir() + "/coverage.png";
                                plan_correlator->saveCoverageMap(coverage_path);
                                // LOW-1 FIX: Use [CORRELATOR] prefix for coverage tracking messages
                                std::cout << "[CORRELATOR] Coverage: " << std::fixed << std::setprecision(1)
                                          << plan_correlator->getCoveragePercent() << "%" << std::endl;
                            }
                        }
                    }

                    // Update coverage and capture if in INSPECTING state
                    if (current_state == InspectionState::INSPECTING) {
                        Pose2D pose{0, 0, 0};
                        if (sensor_source) {
                            pose = sensor_source->getPose();
                        }

                        // Always update coverage tracking based on robot pose (independent of camera)
                        if (plan_correlator->isInitialized()) {
                            plan_correlator->updateCoverage(pose);
                        }

                        // Capture frame if camera available (interval enforced internally)
                        if (image_capture->isCapturing()) {
                            image_capture->captureFrame(pose);
                        }
                    }

                    last_state = current_state;

                    // Main loop rate limiting (20Hz)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }

                // Ensure capture is stopped on exit
                if (image_capture->isCapturing()) {
                    image_capture->stopCapture();
                }
            } catch (const std::exception& e) {
                std::cerr << "[CAPTURE] Thread exception: " << e.what() << std::endl;
                if (image_capture->isCapturing()) {
                    try { image_capture->stopCapture(); } catch (...) {}
                }
            } catch (...) {
                std::cerr << "[CAPTURE] Thread unknown exception" << std::endl;
                if (image_capture->isCapturing()) {
                    try { image_capture->stopCapture(); } catch (...) {}
                }
            }
        });

        // Run CLI in main thread
        cli.runInteractiveMode();

        // Signal capture thread to stop
        cli_running.store(false);
        if (capture_thread.joinable()) {
            capture_thread.join();
        }
    } else {
        cli.processCommand(singleCommand);
    }

    return 0;
}

int main(int argc, char* argv[]) {
    // Set up signal handler for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string interface;
    bool testSensors = false;
    bool testLoco = false;
    bool testWave = false;
    bool testSafety = false;
    bool helloWorld = false;
    bool interactive = false;
    bool generateReport = false;
    std::string reportInspectionId;
    std::string singleCommand;

    // Greeter options (Barry Demo - Story 1.1+)
    bool greeterMode = false;
    std::string greeterConfigPath = "config/greeter.yaml";
    std::string greeterCondition = "with_goal";

    // Teleop options (Story 2-1)
    std::string teleopMode;
    std::string recordSession;
    std::string planPath;
    std::string robotIP;  // For video streaming from robot
    bool autoStream = true;  // Auto-start video stream on robot
    bool dryRun = false;

    // Replay options (Story 2-2)
    std::string replaySession;
    float replaySpeed = 1.0f;
    bool replayLoop = false;
    bool replayVisualize = false;

    // Stream replay options (Story D-1)
    std::string streamReplaySession;
    bool multiStream = false;

    // Video source option
    std::string videoSource = "dds";  // Default to DDS
    int depthPort = 0;  // 0 = disabled, e.g., 5001 to enable
    int webcamPort = 0;  // 0 = disabled, e.g., 5002 to enable
    bool noLidar = false;  // Skip Livox LiDAR initialization
    bool visualizeSLAM = false;  // Show SLAM visualizer during teleop

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        }

        if (arg == "--version" || arg == "-v") {
            printVersion();
            return 0;
        }

        if (arg == "--interface") {
            if (i + 1 < argc) {
                interface = argv[++i];
            } else {
                std::cerr << "Error: --interface requires a name argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--interactive" || arg == "-i") {
            interactive = true;
            continue;
        }

        if (arg == "--test-sensors") {
            testSensors = true;
            continue;
        }

        if (arg == "--test-loco") {
            testLoco = true;
            continue;
        }

        if (arg == "--test-wave") {
            testWave = true;
            continue;
        }

        if (arg == "--test-safety") {
            testSafety = true;
            continue;
        }

        if (arg == "--hello-world") {
            helloWorld = true;
            continue;
        }

        if (arg == "--generate-report") {
            generateReport = true;
            continue;
        }

        if (arg == "--inspection") {
            if (i + 1 < argc) {
                reportInspectionId = argv[++i];
            } else {
                std::cerr << "Error: --inspection requires an ID argument" << std::endl;
                return 1;
            }
            continue;
        }

        // Teleop options (Story 2-1)
        if (arg == "--teleop") {
            if (i + 1 < argc) {
                teleopMode = argv[++i];
                if (teleopMode != "gamepad" && teleopMode != "keyboard") {
                    std::cerr << "Error: --teleop mode must be 'gamepad' or 'keyboard'" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Error: --teleop requires a mode argument (gamepad/keyboard)" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--dry-run") {
            dryRun = true;
            continue;
        }

        if (arg == "--greeter") {
            greeterMode = true;
            continue;
        }

        if (arg == "--config") {
            if (i + 1 < argc) {
                greeterConfigPath = argv[++i];
            } else {
                std::cerr << "Error: --config requires a path argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--greeter-condition") {
            if (i + 1 < argc) {
                greeterCondition = argv[++i];
                if (greeterCondition != "with_goal" && greeterCondition != "no_goal" &&
                    greeterCondition != "WITH_GOAL" && greeterCondition != "NO_GOAL") {
                    std::cerr << "Error: --greeter-condition must be 'with_goal' or 'no_goal'" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Error: --greeter-condition requires an argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--no-auto-stream") {
            autoStream = false;
            continue;
        }

        if (arg == "--no-lidar") {
            noLidar = true;
            continue;
        }

        if (arg == "--visualize-slam") {
            visualizeSLAM = true;
            continue;
        }

        if (arg == "--robot-ip") {
            if (i + 1 < argc) {
                robotIP = argv[++i];
            } else {
                std::cerr << "Error: --robot-ip requires an IP address argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--video-source") {
            if (i + 1 < argc) {
                videoSource = argv[++i];
                if (videoSource != "dds" && videoSource != "gstreamer" &&
                    videoSource != "local" && videoSource != "none") {
                    std::cerr << "Error: --video-source must be 'dds', 'gstreamer', 'local', or 'none'" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Error: --video-source requires a source argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--depth-port") {
            if (i + 1 < argc) {
                depthPort = std::stoi(argv[++i]);
                if (depthPort < 0 || depthPort > 65535) {
                    std::cerr << "Error: --depth-port must be between 0 and 65535" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Error: --depth-port requires a port number argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--webcam-port") {
            if (i + 1 < argc) {
                webcamPort = std::stoi(argv[++i]);
                if (webcamPort < 0 || webcamPort > 65535) {
                    std::cerr << "Error: --webcam-port must be between 0 and 65535" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Error: --webcam-port requires a port number argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--record") {
            if (i + 1 < argc) {
                recordSession = argv[++i];
            } else {
                std::cerr << "Error: --record requires a session ID argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--plan") {
            if (i + 1 < argc) {
                planPath = argv[++i];
            } else {
                std::cerr << "Error: --plan requires a file path argument" << std::endl;
                return 1;
            }
            continue;
        }

        // Replay options (Story 2-2)
        if (arg == "--replay") {
            if (i + 1 < argc) {
                replaySession = argv[++i];
            } else {
                std::cerr << "Error: --replay requires a session ID argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--replay-speed") {
            if (i + 1 < argc) {
                replaySpeed = std::stof(argv[++i]);
                if (replaySpeed < 0.25f || replaySpeed > 4.0f) {
                    std::cerr << "Error: --replay-speed must be between 0.25 and 4.0" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Error: --replay-speed requires a numeric argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--replay-loop") {
            replayLoop = true;
            continue;
        }

        if (arg == "--replay-visualize") {
            replayVisualize = true;
            continue;
        }

        // Stream replay options (Story D-1)
        if (arg == "--stream-replay") {
            if (i + 1 < argc) {
                streamReplaySession = argv[++i];
            } else {
                std::cerr << "Error: --stream-replay requires a session ID argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--multi-stream") {
            multiStream = true;
            continue;
        }

        // Check if this is a CLI command (doesn't start with -)
        if (!arg.empty() && arg[0] != '-') {
            // Capture this and all remaining args as a single command
            for (int j = i; j < argc; ++j) {
                if (!singleCommand.empty()) singleCommand += " ";
                singleCommand += argv[j];
            }
            break;
        }

        // Unknown option
        std::cerr << "Unknown option: " << arg << std::endl;
        printUsage();
        return 1;
    }

    // Run requested test
    if (testSensors) {
        return runSensorTest(interface);
    }

    if (testLoco) {
        return runLocoTest(interface);
    }

    if (testWave) {
        return runWaveTest(interface);
    }

    if (testSafety) {
        return runSafetyTest(interface);
    }

    if (helloWorld) {
        return runHelloWorld(interface);
    }

    // Run report generation
    if (generateReport) {
        if (reportInspectionId.empty()) {
            std::cerr << "Error: --generate-report requires --inspection <id>" << std::endl;
            return 1;
        }
        return runGenerateReport(reportInspectionId);
    }

    // Run greeter mode (Barry Demo - Story 1.1+)
    if (greeterMode) {
        // Load configuration
        greeter::GreeterConfig config = greeter::GreeterConfig::loadFromFile(greeterConfigPath);

        // Override condition from CLI if specified
        config.condition = greeter::GreeterConfig::parseCondition(greeterCondition);

        // Override dry_run from CLI
        if (dryRun) {
            config.dry_run = true;
        }

        // Load environment variables (API key)
        greeter::GreeterConfig::loadFromEnv(config);

        // Display loaded configuration
        std::cout << "[GREETER] Config loaded: condition="
                  << greeter::GreeterConfig::conditionToString(config.condition)
                  << ", dry_run=" << (config.dry_run ? "true" : "false")
                  << ", camera=" << config.video_source << std::endl;

        // TODO: Story 5.4 will implement GreeterRunner
        // For now, just verify config loads correctly
        std::cout << "[GREETER] Barry greeter demo mode initialized (runner not yet implemented)" << std::endl;
        return 0;
    }

    // Run teleop mode (Story 2-1)
    if (!teleopMode.empty()) {
        TeleopRunner runner;
        runner.setNetworkInterface(interface);
        runner.setMode(teleopMode == "gamepad" ? TeleopMode::GAMEPAD : TeleopMode::KEYBOARD);
        runner.setDryRun(dryRun);

        // Set video source
        VideoSource vidSource = VideoSource::DDS;  // Default
        if (videoSource == "gstreamer") {
            vidSource = VideoSource::GSTREAMER;
        } else if (videoSource == "local") {
            vidSource = VideoSource::LOCAL;
        } else if (videoSource == "none") {
            vidSource = VideoSource::NONE;
        }
        runner.setVideoSource(vidSource);

        if (depthPort > 0) {
            runner.setDepthPort(depthPort);
        }

        if (webcamPort > 0) {
            runner.setWebcamPort(webcamPort);
        }

        if (noLidar) {
            runner.setNoLidar(true);
        }

        if (visualizeSLAM) {
            runner.setVisualizeSLAM(true);
        }

        if (!robotIP.empty()) {
            runner.setRobotIP(robotIP);
            runner.setAutoStream(autoStream);
        }

        if (!recordSession.empty()) {
            runner.setRecordingEnabled(true, recordSession);
        }

        if (!planPath.empty()) {
            runner.setPlanPath(planPath);
        }

        return runner.run();
    }

    // Run stream replay mode (Story D-1) - multi-window synchronized viewer
    if (!streamReplaySession.empty() || (!replaySession.empty() && multiStream)) {
        replay::StreamReplayViewer viewer;

        // Resolve path (session ID or full path)
        std::string recording_path;
        std::string session = !streamReplaySession.empty() ? streamReplaySession : replaySession;

        if (session.find('/') != std::string::npos) {
            recording_path = session;
        } else {
            recording_path = "data/recordings/" + session;
        }

        if (!viewer.init(recording_path)) {
            std::cerr << "Failed to open recording: " << recording_path << std::endl;
            return 1;
        }

        viewer.setInitialSpeed(replaySpeed);
        return viewer.run();
    }

    // Run replay mode (Story 2-2)
    if (!replaySession.empty()) {
        replay::ReplayRunner runner;
        // Check if it's a path or just a session ID
        if (replaySession.find('/') != std::string::npos) {
            runner.setRecordingPath(replaySession);
        } else {
            runner.setSessionId(replaySession);
        }
        runner.setSpeed(replaySpeed);
        runner.setLoop(replayLoop);
        runner.setVisualize(replayVisualize);

        return runner.run();
    }

    // Run CLI mode
    if (interactive || !singleCommand.empty()) {
        return runCli(interactive, singleCommand, interface);
    }

    // No command specified - show usage
    printUsage();
    return 0;
}
