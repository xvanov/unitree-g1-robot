#include <iostream>
#include <string>
#include <cstdlib>
#include "DetectionSim.h"
#include "detection/VlmClient.h"

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "\nOptions:\n"
              << "  --images <dir>     Input images directory\n"
              << "  --session <dir>    Load images from ImageCapture session directory\n"
              << "  --plan <context>   Plan context/trade type (default: finishes)\n"
              << "  --output <dir>     Output directory for results (default: ./outputs)\n"
              << "  --key <api_key>    Anthropic API key (or use ANTHROPIC_API_KEY env var)\n"
              << "  --model <model>    Claude model to use (default: claude-sonnet-4-5-20250514)\n"
              << "  --dry-run          Test without API calls (use mock responses)\n"
              << "  --help             Show this help message\n"
              << "\nExamples:\n"
              << "  " << prog << " --images ./test_images --output ./results\n"
              << "  " << prog << " --session ./data/inspections/insp_001 --dry-run\n"
              << "  " << prog << " --images ./test_images --key sk-ant-...\n"
              << std::endl;
}

int main(int argc, char* argv[]) {
    std::string images_dir;
    std::string session_dir;
    std::string output_dir = "./outputs";
    std::string plan_context = "finishes";
    std::string api_key;
    std::string model = "claude-sonnet-4-5-20250514";
    bool dry_run = false;

    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--images" && i + 1 < argc) {
            images_dir = argv[++i];
        } else if (arg == "--session" && i + 1 < argc) {
            session_dir = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--plan" && i + 1 < argc) {
            plan_context = argv[++i];
        } else if (arg == "--key" && i + 1 < argc) {
            api_key = argv[++i];
        } else if (arg == "--model" && i + 1 < argc) {
            model = argv[++i];
        } else if (arg == "--dry-run") {
            dry_run = true;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    // Require either --images or --session
    if (images_dir.empty() && session_dir.empty()) {
        std::cerr << "Error: Must specify --images or --session\n" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // Get API key from environment if not provided
    if (api_key.empty()) {
        const char* env_key = std::getenv("ANTHROPIC_API_KEY");
        if (env_key) {
            api_key = env_key;
        }
    }

    // Check API key requirement
    if (!dry_run && api_key.empty()) {
        std::cerr << "Error: API key required. Set ANTHROPIC_API_KEY or use --key\n"
                  << "Use --dry-run for testing without API calls.\n" << std::endl;
        return 1;
    }

    // Initialize curl
    VlmClient::globalInit();

    // Create client
    VlmClient client(api_key);
    client.setModel(model);

    // Create simulator
    std::string source_dir = session_dir.empty() ? images_dir : session_dir;
    DetectionSim sim(source_dir);
    sim.setOutputDir(output_dir);
    sim.setDryRun(dry_run);

    std::cout << "=== Detection Simulation ===" << std::endl;
    std::cout << "Source: " << source_dir << std::endl;
    std::cout << "Output: " << output_dir << std::endl;
    std::cout << "Plan context: " << plan_context << std::endl;
    std::cout << "Model: " << model << std::endl;
    std::cout << "Dry run: " << (dry_run ? "yes" : "no") << std::endl;
    std::cout << "============================\n" << std::endl;

    // Run detection
    SessionResult result;
    if (!session_dir.empty()) {
        result = sim.runSession(client, session_dir, plan_context);
    } else {
        result = sim.run(client, plan_context);
    }

    // Save results
    std::string results_path = output_dir + "/defects.json";
    if (sim.saveResults(result, results_path)) {
        std::cout << "\nResults saved to: " << results_path << std::endl;
    }

    // Cleanup
    VlmClient::globalCleanup();

    return 0;
}
