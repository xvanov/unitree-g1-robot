#include <iostream>
#include <string>

constexpr const char* VERSION = "G1 Inspector v1.0";

void printUsage() {
    std::cout << VERSION << " - Autonomous Construction Site Inspector\n"
              << "\nUsage: g1_inspector [OPTIONS]\n"
              << "\nOptions:\n"
              << "  --help, -h          Show this help message\n"
              << "  --version, -v       Show version\n"
              << "  --robot <IP>        Connect to robot at IP address\n"
              << "  --test-sensors      Run sensor diagnostics\n"
              << "  --test-loco         Run locomotion test\n"
              << std::endl;
}

void printVersion() {
    std::cout << VERSION << std::endl;
}

int main(int argc, char* argv[]) {
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

        if (arg == "--robot") {
            if (i + 1 < argc) {
                std::string robotIp = argv[++i];
                std::cout << "Connecting to robot at: " << robotIp << std::endl;
                // Robot connection will be implemented in Story 4
            } else {
                std::cerr << "Error: --robot requires an IP address argument" << std::endl;
                return 1;
            }
            continue;
        }

        if (arg == "--test-sensors") {
            std::cout << "Running sensor diagnostics..." << std::endl;
            // Sensor diagnostics will be implemented in Story 4
            return 0;
        }

        if (arg == "--test-loco") {
            std::cout << "Running locomotion test..." << std::endl;
            // Locomotion test will be implemented in Story 4
            return 0;
        }

        // Unknown argument
        std::cerr << "Unknown option: " << arg << std::endl;
        printUsage();
        return 1;
    }

    // No arguments provided - show usage
    if (argc == 1) {
        printUsage();
        return 0;
    }

    return 0;
}
