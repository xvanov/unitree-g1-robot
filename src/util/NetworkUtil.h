#pragma once

#include <string>
#include <mutex>
#include <fstream>
#include <cstdlib>

#ifdef HAS_UNITREE_SDK2
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <iostream>
#include "unitree/robot/channel/channel_factory.hpp"

namespace NetworkUtil {

// Extract subnet prefix from IP (e.g., "192.168.123" from "192.168.123.164")
inline std::string getSubnetPrefix(const std::string& ip) {
    size_t last_dot = ip.rfind('.');
    if (last_dot != std::string::npos) {
        return ip.substr(0, last_dot + 1);  // Include the dot: "192.168.123."
    }
    return "";
}

// Find interface that's on the same subnet as the given IP
// Returns interface name or empty string if not found
inline std::string findInterfaceForIP(const std::string& target_ip) {
    std::string target_subnet = getSubnetPrefix(target_ip);
    if (target_subnet.empty()) {
        return "";
    }

    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        return "";
    }

    std::string result;
    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) continue;
        if (!(ifa->ifa_flags & IFF_UP) || !(ifa->ifa_flags & IFF_RUNNING)) continue;

        auto* addr = (struct sockaddr_in*)ifa->ifa_addr;
        std::string ip = inet_ntoa(addr->sin_addr);

        // Check if this interface is on the same subnet as target
        if (ip.find(target_subnet) == 0) {
            result = ifa->ifa_name;
            std::cout << "[NETWORK] Found interface '" << result << "' (" << ip
                      << ") on same subnet as robot " << target_ip << std::endl;
            break;
        }
    }
    freeifaddrs(ifaddr);
    return result;
}

// Generate a CycloneDDS config file for the given robot IP and interface
// Returns path to the generated config, or empty string on failure
inline std::string generateCycloneDDSConfig(const std::string& robot_ip, const std::string& interface) {
    std::string config_path = "/tmp/cyclonedds_" + std::to_string(getpid()) + ".xml";

    std::ofstream out(config_path);
    if (!out.is_open()) {
        std::cerr << "[NETWORK] Failed to create CycloneDDS config: " << config_path << std::endl;
        return "";
    }

    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<CycloneDDS xmlns=\"https://cdds.io/config\">\n"
        << "  <Domain id=\"0\">\n"
        << "    <General>\n"
        << "      <NetworkInterfaceAddress>" << interface << "</NetworkInterfaceAddress>\n"
        << "    </General>\n"
        << "    <Discovery>\n"
        << "      <Peers>\n"
        << "        <Peer address=\"" << robot_ip << "\"/>\n"
        << "      </Peers>\n"
        << "    </Discovery>\n"
        << "  </Domain>\n"
        << "</CycloneDDS>\n";

    out.close();

    // Set environment variable for CycloneDDS
    std::string uri = "file://" + config_path;
    setenv("CYCLONEDDS_URI", uri.c_str(), 1);

    std::cout << "[NETWORK] Generated CycloneDDS config for robot " << robot_ip
              << " on interface " << interface << std::endl;

    return config_path;
}

// Find network interface connected to robot subnet
// Supports both ethernet (192.168.123.x) and WiFi (192.168.124.x) subnets
// Returns interface name (e.g., "eth0", "wlp4s0") or empty string on failure
// Prefers wireless interfaces (wl*) over ethernet
inline std::string findRobotInterface() {
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        std::cerr << "[NETWORK] Cannot enumerate network interfaces" << std::endl;
        return "";
    }

    std::string wlan_result = "";
    std::string eth_result = "";
    std::string wlan_ip = "";
    std::string eth_ip = "";

    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) continue;

        // Skip interfaces that aren't UP and RUNNING
        if (!(ifa->ifa_flags & IFF_UP) || !(ifa->ifa_flags & IFF_RUNNING)) continue;

        auto* addr = (struct sockaddr_in*)ifa->ifa_addr;
        std::string ip = inet_ntoa(addr->sin_addr);
        std::string name = ifa->ifa_name;

        // Check if on robot subnet: 192.168.123.x (ethernet) or 192.168.124.x (WiFi)
        bool is_robot_subnet = (ip.find("192.168.123.") == 0) || (ip.find("192.168.124.") == 0);
        if (is_robot_subnet) {
            // Prefer wireless interfaces (wl*, wlan*) - more reliable in Docker/host setups
            if (name.find("wl") == 0 || name.find("wlan") == 0) {
                wlan_result = name;
                wlan_ip = ip;
            } else if (eth_result.empty()) {
                eth_result = name;
                eth_ip = ip;
            }
        }
    }
    freeifaddrs(ifaddr);

    // Prefer wireless over ethernet
    if (!wlan_result.empty()) {
        std::cout << "[NETWORK] Found robot network on interface '" << wlan_result
                  << "' (" << wlan_ip << ")" << std::endl;
        return wlan_result;
    }
    if (!eth_result.empty()) {
        std::cout << "[NETWORK] Found robot network on interface '" << eth_result
                  << "' (" << eth_ip << ")" << std::endl;
        return eth_result;
    }

    std::cerr << "[NETWORK] No interface found on robot subnet (192.168.123.x or 192.168.124.x)" << std::endl;
    std::cerr << "[NETWORK] Verify your network configuration:" << std::endl;
    std::cerr << "  Ethernet: sudo ip addr add 192.168.123.100/24 dev eth0" << std::endl;
    std::cerr << "  WiFi:     Connect to robot's WiFi network" << std::endl;
    return "";
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
