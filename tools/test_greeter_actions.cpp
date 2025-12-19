/**
 * test_greeter_actions - CLI tool to test ActionParser and ActionExecutor
 *
 * Usage:
 *   ./test_greeter_actions --action '{"action":"WAVE_HAND"}'
 *   ./test_greeter_actions --action '{"action":"MOVE_FORWARD","parameters":{"distance":0.5}}'
 *   ./test_greeter_actions --action '{"action":"BOW","confidence":0.9}'
 *   ./test_greeter_actions --action '{"action":"PUSH_FORWARD","parameters":{"force_level":0.3}}'
 *   ./test_greeter_actions --list   # List all available actions
 *   ./test_greeter_actions --interactive  # Interactive mode
 *
 * For robot testing:
 *   ./test_greeter_actions --action '{"action":"WAVE_HAND"}' --robot
 */

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>

#include "greeter/ActionParser.h"
#include "greeter/ActionExecutor.h"
#include "locomotion/LocoController.h"

using namespace greeter;

// Global for signal handling
static bool g_running = true;

void signalHandler(int sig) {
    std::cout << "\n[CTRL+C] Stopping..." << std::endl;
    g_running = false;
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "\nOptions:\n"
              << "  --action JSON    Execute action from JSON string\n"
              << "  --list           List all available actions\n"
              << "  --interactive    Interactive mode (enter actions one by one)\n"
              << "  --robot          Connect to real robot (default: dry-run mode)\n"
              << "  --dof N          Robot DOF: 23 (EDU), 29 (Standard), 36 (Full) [default: 23]\n"
              << "  --help           Show this help\n"
              << "\nExamples:\n"
              << "  " << prog << " --list\n"
              << "  " << prog << " --action '{\"action\":\"WAVE_HAND\"}'\n"
              << "  " << prog << " --action '{\"action\":\"MOVE_FORWARD\",\"parameters\":{\"distance\":0.5}}'\n"
              << "  " << prog << " --action '{\"action\":\"BOW\"}' --robot\n"
              << "  " << prog << " --interactive --robot\n"
              << std::endl;
}

void printAvailableActions() {
    std::cout << "\n=== Available Actions ===\n\n";

    std::cout << "Movement Actions:\n";
    std::cout << "  MOVE_FORWARD   - Walk forward (parameters: distance in meters)\n";
    std::cout << "  MOVE_BACKWARD  - Walk backward (parameters: distance in meters)\n";
    std::cout << "  ROTATE         - Rotate in place (parameters: angle in degrees, + = left)\n";
    std::cout << "  MOVE_TO        - Move to coordinates (parameters: x, y in meters)\n";
    std::cout << "  FOLLOW         - Follow a person (parameters: target_id)\n";
    std::cout << "  STOP           - Stop all movement\n";
    std::cout << "  RETURN_TO_POST - Return to greeting position\n";

    std::cout << "\nGesture Actions:\n";
    std::cout << "  WAVE_HAND      - Wave hand greeting (~3 sec, requires 29+ DOF)\n";
    std::cout << "  SHAKE_HAND     - Handshake gesture (~5 sec, requires 29+ DOF)\n";
    std::cout << "  BOW            - Bow gesture (~3 sec)\n";

    std::cout << "\nPosture Actions:\n";
    std::cout << "  STAND_UP       - Stand from sitting\n";
    std::cout << "  SIT_DOWN       - Sit down\n";

    std::cout << "\nSpecial Actions:\n";
    std::cout << "  PUSH_FORWARD   - Push forward (parameters: force_level 0.0-1.0) [LOGGED]\n";
    std::cout << "  SPEAK          - Display text (parameters: text)\n";
    std::cout << "  WAIT           - Wait and observe\n";
    std::cout << "  NO_ACTION      - Do nothing\n";

    std::cout << "\n=== JSON Format ===\n\n";
    std::cout << "{\n";
    std::cout << "  \"reasoning\": \"Why I'm doing this\",\n";
    std::cout << "  \"intent\": \"What I want to achieve\",\n";
    std::cout << "  \"action\": \"ACTION_TYPE\",\n";
    std::cout << "  \"parameters\": { ... },\n";
    std::cout << "  \"confidence\": 0.0-1.0\n";
    std::cout << "}\n";

    std::cout << "\n=== Example Commands ===\n\n";
    std::cout << "# Wave hand\n";
    std::cout << "{\"action\":\"WAVE_HAND\",\"confidence\":0.9}\n\n";

    std::cout << "# Move forward 0.5 meters\n";
    std::cout << "{\"action\":\"MOVE_FORWARD\",\"parameters\":{\"distance\":0.5}}\n\n";

    std::cout << "# Rotate 90 degrees left\n";
    std::cout << "{\"action\":\"ROTATE\",\"parameters\":{\"angle\":90}}\n\n";

    std::cout << "# Bow with reasoning\n";
    std::cout << "{\"reasoning\":\"Greeting VIP\",\"action\":\"BOW\",\"confidence\":0.95}\n\n";

    std::cout << "# Push (careful!)\n";
    std::cout << "{\"action\":\"PUSH_FORWARD\",\"parameters\":{\"force_level\":0.3}}\n\n";

    std::cout << "# Speak\n";
    std::cout << "{\"action\":\"SPEAK\",\"parameters\":{\"text\":\"Hello, welcome!\"}}\n";

    std::cout << std::endl;
}

bool executeAction(const std::string& json, ActionExecutor* executor, ActionParser& parser, bool dryRun) {
    std::cout << "\n[INPUT] " << json << std::endl;

    // Parse the action
    auto result = parser.parse(json);
    if (!result) {
        std::cerr << "[ERROR] Failed to parse JSON: " << parser.getLastError() << std::endl;
        return false;
    }

    const ParsedAction& action = *result;
    std::cout << "[PARSED] Action: " << ActionParser::actionTypeToString(action.type);
    if (!action.reasoning.empty()) {
        std::cout << " | Reasoning: " << action.reasoning;
    }
    if (action.confidence > 0) {
        std::cout << " | Confidence: " << action.confidence;
    }
    std::cout << std::endl;

    if (dryRun) {
        std::cout << "[DRY-RUN] Would execute: " << ActionParser::actionTypeToString(action.type) << std::endl;

        // Show what parameters would be used
        switch (action.type) {
            case ActionType::MOVE_FORWARD:
            case ActionType::MOVE_BACKWARD:
                std::cout << "  Distance: " << action.distance << " m" << std::endl;
                break;
            case ActionType::ROTATE:
                std::cout << "  Angle: " << action.angle << " deg" << std::endl;
                break;
            case ActionType::MOVE_TO:
                std::cout << "  Target: (" << action.x << ", " << action.y << ")" << std::endl;
                break;
            case ActionType::FOLLOW:
                std::cout << "  Target ID: " << action.target_id << std::endl;
                break;
            case ActionType::PUSH_FORWARD:
                std::cout << "  Force Level: " << action.force_level << std::endl;
                break;
            case ActionType::SPEAK:
                std::cout << "  Text: \"" << action.text << "\"" << std::endl;
                break;
            default:
                break;
        }
        return true;
    }

    // Execute on robot
    std::cout << "[EXECUTING] " << ActionParser::actionTypeToString(action.type) << "..." << std::endl;

    if (!executor->execute(action)) {
        std::cerr << "[ERROR] Execution failed" << std::endl;
        return false;
    }

    // Wait for completion with progress updates
    while (!executor->isActionComplete() && g_running) {
        executor->update(0.05f);  // 50ms update rate

        float progress = executor->getProgress();
        std::cout << "\r[PROGRESS] " << static_cast<int>(progress * 100) << "%" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!g_running) {
        executor->cancel();
        std::cout << "\n[CANCELLED]" << std::endl;
        return false;
    }

    std::cout << "\n[COMPLETE]" << std::endl;
    return true;
}

void interactiveMode(ActionExecutor* executor, ActionParser& parser, bool dryRun) {
    std::cout << "\n=== Interactive Mode ===\n";
    std::cout << "Enter action JSON (or 'quit' to exit, 'help' for examples):\n\n";

    std::string line;
    while (g_running) {
        std::cout << "> ";
        std::getline(std::cin, line);

        if (line.empty()) continue;
        if (line == "quit" || line == "exit" || line == "q") break;
        if (line == "help" || line == "?") {
            printAvailableActions();
            continue;
        }

        executeAction(line, executor, parser, dryRun);
    }
}

int main(int argc, char* argv[]) {
    // Parse arguments
    std::string actionJson;
    bool robotMode = false;
    bool listMode = false;
    bool interactiveFlag = false;
    int robotDof = G1Model::EDU_23_DOF;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--list" || arg == "-l") {
            listMode = true;
        } else if (arg == "--interactive" || arg == "-i") {
            interactiveFlag = true;
        } else if (arg == "--robot" || arg == "-r") {
            robotMode = true;
        } else if (arg == "--action" || arg == "-a") {
            if (i + 1 < argc) {
                actionJson = argv[++i];
            } else {
                std::cerr << "Error: --action requires a JSON argument" << std::endl;
                return 1;
            }
        } else if (arg == "--dof") {
            if (i + 1 < argc) {
                robotDof = std::stoi(argv[++i]);
            }
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    if (listMode) {
        printAvailableActions();
        return 0;
    }

    if (actionJson.empty() && !interactiveFlag) {
        printUsage(argv[0]);
        return 1;
    }

    // Setup signal handler
    signal(SIGINT, signalHandler);

    // Create components
    ActionParser parser;
    std::shared_ptr<LocoController> loco;
    std::unique_ptr<ActionExecutor> executor;

    bool dryRun = !robotMode;

    if (robotMode) {
        std::cout << "[INIT] Connecting to robot (DOF=" << robotDof << ")..." << std::endl;
        loco = std::make_shared<LocoController>();

        if (!loco->init("", robotDof)) {
            std::cerr << "[ERROR] Failed to connect to robot" << std::endl;
            std::cerr << "        Make sure robot is powered on and in SDK mode" << std::endl;
            return 1;
        }

        std::cout << "[INIT] Robot connected" << std::endl;

        if (!loco->hasArmControl()) {
            std::cout << "[WARN] Robot is 23-DOF EDU model - arm gestures (WAVE_HAND, SHAKE_HAND) not available" << std::endl;
        }

        executor = std::make_unique<ActionExecutor>(loco);
    } else {
        std::cout << "[DRY-RUN] Running without robot connection" << std::endl;
        std::cout << "          Use --robot flag to connect to real robot" << std::endl;
    }

    // Execute
    if (interactiveFlag) {
        interactiveMode(executor.get(), parser, dryRun);
    } else {
        executeAction(actionJson, executor.get(), parser, dryRun);
    }

    std::cout << "\n[DONE]" << std::endl;
    return 0;
}
