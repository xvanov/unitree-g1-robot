#pragma once

#include <string>
#include <vector>
#include <atomic>
#include "app/StateMachine.h"
#include "plan/PlanManager.h"
#include "sensors/ISensorSource.h"

class CliHandler {
public:
    CliHandler(StateMachine* sm, PlanManager* pm,
               ISensorSource* sensors = nullptr);

    // Process single command, return true if should continue
    bool processCommand(const std::string& input);

    // Run interactive REPL
    void runInteractiveMode();

    // Commands
    void cmdStatus();
    void cmdHelp();
    void cmdUpload(const std::string& plan_path, const std::string& trade = "finishes");
    void cmdCalibrate(float x, float y, float theta);
    void cmdWaypoints();
    void cmdStart();
    void cmdPause();
    void cmdResume();
    void cmdStop();
    void cmdEstop();
    void cmdClearEstop();
    void cmdHome();
    void cmdReport();

    // Set exit flag (for signal handler integration)
    static void requestExit() { exit_requested_.store(true); }

private:
    std::vector<std::string> parseArgs(const std::string& input);
    std::string trim(const std::string& str);

    StateMachine* state_machine_;
    PlanManager* plan_manager_;
    ISensorSource* sensors_;

    static std::atomic<bool> exit_requested_;
};
