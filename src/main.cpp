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
#include "app/StateMachine.h"
#include "app/CliHandler.h"
#include "plan/PlanManager.h"

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
              << "  --test-sensors       Run sensor diagnostics\n"
              << "  --test-loco          Run locomotion test (robot will move!)\n"
              << "  --test-wave          Arm wave test (robot waves hand)\n"
              << "  --hello-world        Full integration test sequence\n"
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

    // Check if robot is standing
    int state = locomotion->getState();
    if (state != G1FSM::STANDING && state != G1FSM::WALKING) {
        std::cout << "[WAVE] Robot not standing (FSM=" << state << "), attempting to stand up..." << std::endl;
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

    CliHandler cli(state_machine.get(), plan_manager.get(), sensor_source.get());

    if (interactive) {
        cli.runInteractiveMode();
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
    bool helloWorld = false;
    bool interactive = false;
    std::string singleCommand;

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

        if (arg == "--hello-world") {
            helloWorld = true;
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

    if (helloWorld) {
        return runHelloWorld(interface);
    }

    // Run CLI mode
    if (interactive || !singleCommand.empty()) {
        return runCli(interactive, singleCommand, interface);
    }

    // No command specified - show usage
    printUsage();
    return 0;
}
