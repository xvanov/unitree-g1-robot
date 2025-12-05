#pragma once

#include <string>
#include <mutex>

#ifdef HAS_UNITREE_SDK2
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include "unitree/robot/channel/channel_factory.hpp"

namespace NetworkUtil {

// Find network interface connected to robot subnet (192.168.123.x)
// Returns interface name (e.g., "eth0", "en0") or "eth0" as fallback
inline std::string findRobotInterface() {
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        std::cerr << "[NETWORK] Cannot enumerate network interfaces, using 'eth0' fallback" << std::endl;
        return "eth0";
    }

    std::string result = "";
    bool found = false;
    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) continue;

        auto* addr = (struct sockaddr_in*)ifa->ifa_addr;
        std::string ip = inet_ntoa(addr->sin_addr);

        // Check if on robot subnet 192.168.123.x
        if (ip.find("192.168.123.") == 0) {
            result = ifa->ifa_name;
            found = true;
            std::cout << "[NETWORK] Found robot subnet on interface '" << result
                      << "' (" << ip << ")" << std::endl;
            break;
        }
    }
    freeifaddrs(ifaddr);

    if (!found) {
        std::cerr << "[NETWORK] No interface found on robot subnet 192.168.123.x" << std::endl;
        std::cerr << "[NETWORK] Verify your network configuration:" << std::endl;
        std::cerr << "  Linux:  sudo ip addr add 192.168.123.100/24 dev eth0" << std::endl;
        std::cerr << "  Mac:    System Preferences > Network > Configure IPv4: Manually" << std::endl;
        std::cerr << "          IP: 192.168.123.100, Subnet: 255.255.255.0" << std::endl;
        result = "eth0";
    }
    return result;
}

// Thread-safe singleton initialization for ChannelFactory
// Call this before using any SDK channels (SensorManager, LocoController)
// Safe to call multiple times - only initializes once
// Returns true if factory is ready, false on failure
inline bool initChannelFactory(const std::string& network_interface = "") {
    static std::mutex init_mutex;
    static bool initialized = false;
    static bool success = false;

    std::lock_guard<std::mutex> lock(init_mutex);

    if (initialized) {
        return success;
    }

    try {
        std::string iface = network_interface.empty() ? findRobotInterface() : network_interface;
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);
        std::cout << "[SDK] ChannelFactory initialized on interface '" << iface << "'" << std::endl;
        initialized = true;
        success = true;
    } catch (const std::exception& e) {
        std::cerr << "[SDK] ChannelFactory initialization failed: " << e.what() << std::endl;
        initialized = true;
        success = false;
    }

    return success;
}

}  // namespace NetworkUtil

#else
// No-op implementation when SDK not available
namespace NetworkUtil {

inline std::string findRobotInterface() {
    return "eth0";
}

inline bool initChannelFactory(const std::string& = "") {
    return false;
}

}  // namespace NetworkUtil
#endif
