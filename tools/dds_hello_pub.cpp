// Simple DDS Hello World Publisher using unitree_sdk2
// Tests Zenoh-DDS bridge connectivity

#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <csignal>
#include <atomic>

#ifdef HAS_UNITREE_SDK2
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/ros2/String_.hpp>

std::atomic<bool> running{true};

void signalHandler(int sig) {
    (void)sig;
    running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    std::string interface = "lo";  // Default to localhost for testing
    if (argc > 1) {
        interface = argv[1];
    }

    std::cout << "[DDS PUB] Starting DDS publisher on interface: " << interface << std::endl;

    // Initialize channel factory
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);

    // Create publisher for a simple test topic
    unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_> publisher("rt/test/hello");
    publisher.InitChannel();

    std::cout << "[DDS PUB] Publisher created on topic: rt/test/hello" << std::endl;
    std::cout << "[DDS PUB] Press Ctrl+C to exit" << std::endl;

    int count = 0;
    while (running) {
        std_msgs::msg::dds_::String_ msg;
        std::string data = "Hello from DDS! Count: " + std::to_string(count);
        msg.data(data);

        if (publisher.Write(msg)) {
            std::cout << "[DDS PUB] Sent: " << data << std::endl;
        } else {
            std::cout << "[DDS PUB] Failed to send message" << std::endl;
        }

        count++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    publisher.CloseChannel();
    std::cout << "[DDS PUB] Exiting..." << std::endl;
    return 0;
}
#else
int main() {
    std::cerr << "unitree_sdk2 not available" << std::endl;
    return 1;
}
#endif
