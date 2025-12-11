#include "app/CliHandler.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>

std::atomic<bool> CliHandler::exit_requested_{false};

CliHandler::CliHandler(StateMachine* sm, PlanManager* pm, ISensorSource* sensors)
    : state_machine_(sm)
    , plan_manager_(pm)
    , sensors_(sensors)
{
}

std::string CliHandler::trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\n\r");
    return str.substr(first, last - first + 1);
}

std::vector<std::string> CliHandler::parseArgs(const std::string& input) {
    std::vector<std::string> args;
    std::istringstream iss(input);
    std::string token;

    while (iss >> token) {
        args.push_back(token);
    }

    return args;
}

bool CliHandler::processCommand(const std::string& input) {
    std::string trimmed = trim(input);
    if (trimmed.empty()) {
        return true;  // Continue
    }

    std::vector<std::string> args = parseArgs(trimmed);
    if (args.empty()) {
        return true;
    }

    std::string cmd = args[0];
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

    if (cmd == "status") {
        cmdStatus();
    } else if (cmd == "help" || cmd == "?") {
        cmdHelp();
    } else if (cmd == "upload") {
        // Parse upload arguments
        std::string plan_path;
        std::string trade = "finishes";

        for (size_t i = 1; i < args.size(); i++) {
            if (args[i] == "--plan" && i + 1 < args.size()) {
                plan_path = args[++i];
            } else if (args[i] == "--trade" && i + 1 < args.size()) {
                trade = args[++i];
            } else if (!args[i].empty() && args[i][0] != '-') {
                // Assume it's the plan path if not prefixed
                plan_path = args[i];
            }
        }

        if (plan_path.empty()) {
            std::cout << "Usage: upload --plan <file> [--trade <type>]" << std::endl;
            std::cout << "       upload <file>" << std::endl;
        } else {
            cmdUpload(plan_path, trade);
        }
    } else if (cmd == "calibrate") {
        // Parse calibrate arguments
        float x = 0.0f, y = 0.0f, theta = 0.0f;
        bool has_position = false;

        for (size_t i = 1; i < args.size(); i++) {
            if (args[i] == "--position" && i + 1 < args.size()) {
                std::string pos = args[++i];
                // Parse x,y,theta format
                std::replace(pos.begin(), pos.end(), ',', ' ');
                std::istringstream pss(pos);
                if (pss >> x >> y >> theta) {
                    has_position = true;
                }
            }
        }

        if (!has_position) {
            std::cout << "Usage: calibrate --position x,y,theta" << std::endl;
            std::cout << "Example: calibrate --position 0,0,0" << std::endl;
        } else {
            cmdCalibrate(x, y, theta);
        }
    } else if (cmd == "waypoints") {
        cmdWaypoints();
    } else if (cmd == "start") {
        cmdStart();
    } else if (cmd == "pause") {
        cmdPause();
    } else if (cmd == "resume") {
        cmdResume();
    } else if (cmd == "stop") {
        cmdStop();
    } else if (cmd == "estop") {
        cmdEstop();
    } else if (cmd == "clearestop") {
        cmdClearEstop();
    } else if (cmd == "home" || cmd == "return" || cmd == "gohome") {
        cmdHome();
    } else if (cmd == "report") {
        cmdReport();
    } else if (cmd == "quit" || cmd == "exit") {
        std::cout << "Goodbye!" << std::endl;
        return false;  // Signal to exit
    } else {
        std::cout << "Unknown command: " << cmd << std::endl;
        std::cout << "Type 'help' for available commands" << std::endl;
    }

    return true;  // Continue
}

void CliHandler::runInteractiveMode() {
    std::cout << "G1 Inspector Interactive Mode (type 'help' for commands, 'quit' to exit)" << std::endl;
    std::cout << std::endl;

    std::string line;
    while (!exit_requested_.load()) {
        std::cout << "g1> " << std::flush;
        if (!std::getline(std::cin, line)) {
            break;  // EOF or error
        }
        if (!processCommand(line)) {
            break;  // quit command
        }
    }
}

void CliHandler::cmdStatus() {
    std::cout << "State: " << state_machine_->getStateString() << std::endl;

    // Show battery and location if sensors available
    if (sensors_) {
        auto pose = sensors_->getPose();
        std::cout << "Location: (" << std::fixed << std::setprecision(2)
                  << pose.x << ", " << pose.y << ", " << pose.theta << ")" << std::endl;
        std::cout << "Battery: " << std::fixed << std::setprecision(0)
                  << sensors_->getBatteryPercent() << "%" << std::endl;
    } else {
        std::cout << "Location: (0.0, 0.0, 0.0)" << std::endl;
        std::cout << "Battery: N/A (no sensors)" << std::endl;
    }

    // Show plan info
    if (plan_manager_->isLoaded()) {
        auto info = plan_manager_->getPlanInfo();
        std::cout << "Plan: " << info.path << std::endl;
        std::cout << "  Size: " << info.width_pixels << "x" << info.height_pixels << " px ("
                  << info.width_meters << " x " << info.height_meters << " m)" << std::endl;
        std::cout << "  Waypoints: " << info.waypoint_count << std::endl;
    } else {
        std::cout << "Plan: Not loaded" << std::endl;
    }

    // Show completion if inspecting
    if (state_machine_->getState() == InspectionState::INSPECTING ||
        state_machine_->getState() == InspectionState::PAUSED ||
        state_machine_->getState() == InspectionState::COMPLETE) {
        std::cout << "Completion: " << std::fixed << std::setprecision(1)
                  << state_machine_->getCompletionPercent() << "%" << std::endl;
    }
}

void CliHandler::cmdHelp() {
    std::cout << "Available commands:" << std::endl;
    std::cout << "  status                       Show current state, location, battery, completion" << std::endl;
    std::cout << "  start                        Start inspection (requires plan + calibration)" << std::endl;
    std::cout << "  pause                        Pause current inspection" << std::endl;
    std::cout << "  resume                       Resume paused inspection" << std::endl;
    std::cout << "  stop                         Stop inspection, return to IDLE" << std::endl;
    std::cout << "  home                         Return robot to start position" << std::endl;
    std::cout << "  estop                        Emergency stop (immediate halt)" << std::endl;
    std::cout << "  clearestop                   Clear E-stop and return to IDLE" << std::endl;
    std::cout << "  upload --plan <file> [--trade <type>]  Load construction plan" << std::endl;
    std::cout << "  calibrate --position x,y,theta         Set robot position on plan" << std::endl;
    std::cout << "  waypoints                    Show generated inspection waypoints" << std::endl;
    std::cout << "  report                       Generate inspection report (deferred to Story 1-9)" << std::endl;
    std::cout << "  help                         Show this help message" << std::endl;
    std::cout << "  quit                         Exit interactive mode" << std::endl;
}

void CliHandler::cmdUpload(const std::string& plan_path, const std::string& trade) {
    if (plan_manager_->loadPlan(plan_path, trade)) {
        auto info = plan_manager_->getPlanInfo();
        std::cout << "Plan loaded: " << plan_path << std::endl;
        std::cout << "  Size: " << info.width_pixels << "x" << info.height_pixels << " px ("
                  << info.width_meters << " x " << info.height_meters << " m)" << std::endl;
        std::cout << "  Trade: " << trade << std::endl;
        std::cout << "  Waypoints: " << info.waypoint_count << " generated" << std::endl;

        // Update state machine with waypoint count
        state_machine_->setWaypointProgress(0, info.waypoint_count);
    } else {
        std::cout << "Failed to load plan: " << plan_path << std::endl;
    }
}

void CliHandler::cmdCalibrate(float x, float y, float theta) {
    Point2D pos{x, y};
    plan_manager_->setStartPosition(pos, theta);
    std::cout << "Calibration complete" << std::endl;
    std::cout << "Robot origin set at plan position (" << x << ", " << y << ")" << std::endl;

    // If we're in CALIBRATING state, transition to INSPECTING
    if (state_machine_->getState() == InspectionState::CALIBRATING) {
        state_machine_->setCalibrated();
    }
}

void CliHandler::cmdWaypoints() {
    if (!plan_manager_->isLoaded()) {
        std::cout << "No plan loaded. Use 'upload' to load a plan first." << std::endl;
        return;
    }

    const auto& waypoints = plan_manager_->getInspectionWaypoints();
    if (waypoints.empty()) {
        std::cout << "No waypoints generated." << std::endl;
        return;
    }

    int num = 1;
    for (const auto& wp : waypoints) {
        std::cout << "Waypoint " << num++ << ": ("
                  << std::fixed << std::setprecision(2)
                  << wp.x << ", " << wp.y << ")" << std::endl;

        // Limit output to first 20 waypoints for readability
        if (num > 20 && waypoints.size() > 25) {
            std::cout << "... (" << waypoints.size() - 20 << " more)" << std::endl;
            break;
        }
    }
    std::cout << "Total: " << waypoints.size() << " waypoints" << std::endl;
}

void CliHandler::cmdStart() {
    if (!plan_manager_->isLoaded()) {
        std::cout << "Cannot start: No plan loaded. Use 'upload' first." << std::endl;
        return;
    }

    InspectionState current = state_machine_->getState();

    if (current == InspectionState::EMERGENCY_STOP) {
        std::cout << "Cannot start: E-stop is active. Clear E-stop first." << std::endl;
        return;
    }

    if (state_machine_->startInspection()) {
        std::cout << "Inspection started" << std::endl;
        std::cout << "State: " << state_machine_->getStateString() << std::endl;
        std::cout << "Use 'calibrate' to set robot position on plan" << std::endl;
    } else {
        std::cout << "Cannot start inspection from state: " << state_machine_->getStateString() << std::endl;
    }
}

void CliHandler::cmdPause() {
    if (state_machine_->pause()) {
        std::cout << "Inspection paused" << std::endl;
    } else {
        std::cout << "Cannot pause from state: " << state_machine_->getStateString() << std::endl;
    }
}

void CliHandler::cmdResume() {
    if (state_machine_->resume()) {
        std::cout << "Inspection resumed" << std::endl;
    } else {
        std::cout << "Cannot resume from state: " << state_machine_->getStateString() << std::endl;
    }
}

void CliHandler::cmdStop() {
    if (state_machine_->stop()) {
        std::cout << "Inspection stopped" << std::endl;
        std::cout << "State: " << state_machine_->getStateString() << std::endl;
    } else {
        std::cout << "Cannot stop from state: " << state_machine_->getStateString() << std::endl;
    }
}

void CliHandler::cmdEstop() {
    state_machine_->emergencyStop();
    std::cout << "EMERGENCY STOP activated" << std::endl;
    std::cout << "Robot halted. Use 'clearestop' to clear E-stop and return to IDLE." << std::endl;
}

void CliHandler::cmdClearEstop() {
    if (state_machine_->clearEstop()) {
        std::cout << "E-stop cleared" << std::endl;
        std::cout << "State: " << state_machine_->getStateString() << std::endl;
    } else {
        std::cout << "Cannot clear E-stop from state: " << state_machine_->getStateString() << std::endl;
    }
}

void CliHandler::cmdHome() {
    if (state_machine_->startReturningHome()) {
        std::cout << "Returning home..." << std::endl;
        std::cout << "State: " << state_machine_->getStateString() << std::endl;
    } else {
        std::cout << "Cannot return home from state: " << state_machine_->getStateString() << std::endl;
    }
}

void CliHandler::cmdReport() {
    std::cout << "Report generation not yet implemented." << std::endl;
    std::cout << "This feature will be available in Story 1-9." << std::endl;
}
