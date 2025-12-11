// Simple DDS Hello World Subscriber using unitree_sdk2
// Tests Zenoh-DDS bridge connectivity

#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <atomic>
#include <csignal>

#ifdef HAS_UNITREE_SDK2
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>

std::atomic<bool> running{true};
std::atomic<int> message_count{0};

void signalHandler(int sig) {
    (void)sig;
    running = false;
}

void messageCallback(const void* msg) {
    const std_msgs::msg::dds_::String_* str_msg =
        static_cast<const std_msgs::msg::dds_::String_*>(msg);
    std::cout << "[DDS SUB] Received: " << str_msg->data() << std::endl;
    message_count++;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    std::string interface = "lo";  // Default to localhost for testing
    if (argc > 1) {
        interface = argv[1];
    }

    std::cout << "[DDS SUB] Starting DDS subscriber on interface: " << interface << std::endl;

    // Initialize channel factory
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);

    // Create subscriber for test topic with callback
    unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
        "rt/test/hello", messageCallback);
    subscriber.InitChannel();

    std::cout << "[DDS SUB] Subscriber created on topic: rt/test/hello" << std::endl;
    std::cout << "[DDS SUB] Waiting for messages... Press Ctrl+C to exit" << std::endl;

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    subscriber.CloseChannel();
    std::cout << "[DDS SUB] Received " << message_count.load() << " messages total" << std::endl;
    std::cout << "[DDS SUB] Exiting..." << std::endl;
    return 0;
}
#else
int main() {
    std::cerr << "unitree_sdk2 not available" << std::endl;
    return 1;
}
#endif
