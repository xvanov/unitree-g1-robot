/**
 * Test DDS Video Streaming from G1 Robot
 *
 * Verifies that we can receive video frames from the robot's videohub
 * service via DDS RPC (no SSH/GStreamer needed).
 *
 * Usage: ./build/test_dds_video [network_interface]
 *   e.g.: ./build/test_dds_video wlp4s0
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstring>

#ifdef HAS_UNITREE_SDK2
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>
#endif

#include <opencv2/opencv.hpp>

static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

int main(int argc, char* argv[]) {
    std::cout << "=== DDS Video Stream Test for G1 ===" << std::endl;

#ifndef HAS_UNITREE_SDK2
    std::cerr << "ERROR: unitree_sdk2 not available. Build with SDK support." << std::endl;
    return 1;
#else

    // Get network interface from args or use default
    std::string networkInterface = "eth0";
    if (argc > 1) {
        networkInterface = argv[1];
    }

    std::cout << "Network interface: " << networkInterface << std::endl;
    std::cout << "Initializing DDS..." << std::endl;

    // Set up signal handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        // Initialize DDS channel factory
        unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);
        std::cout << "DDS initialized on domain 0" << std::endl;

        // Create video client
        unitree::robot::go2::VideoClient videoClient;
        videoClient.SetTimeout(5.0f);  // 5 second timeout
        videoClient.Init();
        std::cout << "VideoClient initialized, connecting to 'videohub' service..." << std::endl;

        // Try to get frames
        std::vector<uint8_t> imageData;
        int frameCount = 0;
        int errorCount = 0;

        auto startTime = std::chrono::steady_clock::now();

        // Create window for display
        cv::namedWindow("DDS Video Test", cv::WINDOW_AUTOSIZE);

        std::cout << "\nStreaming video. Press 'q' in window or Ctrl+C to quit.\n" << std::endl;

        while (g_running) {
            auto frameStart = std::chrono::steady_clock::now();

            int ret = videoClient.GetImageSample(imageData);

            if (ret == 0 && !imageData.empty()) {
                frameCount++;

                // Decode JPEG
                cv::Mat frame = cv::imdecode(imageData, cv::IMREAD_COLOR);

                if (!frame.empty()) {
                    // Calculate FPS
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - startTime).count();
                    double fps = frameCount / elapsed;

                    // Add info overlay
                    char info[128];
                    snprintf(info, sizeof(info), "Frame %d | Size: %zu bytes | FPS: %.1f",
                             frameCount, imageData.size(), fps);
                    cv::putText(frame, info, cv::Point(10, 30),
                               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                    cv::putText(frame, "DDS Video Stream - Press 'q' to quit", cv::Point(10, 60),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                    // Display
                    cv::imshow("DDS Video Test", frame);

                    // Save first frame as proof
                    if (frameCount == 1) {
                        std::string filename = "dds_video_test_frame.jpg";
                        cv::imwrite(filename, frame);
                        std::cout << "First frame saved to: " << filename << std::endl;
                        std::cout << "  Resolution: " << frame.cols << "x" << frame.rows << std::endl;
                        std::cout << "  JPEG size: " << imageData.size() << " bytes" << std::endl;
                    }

                    // Print periodic status
                    if (frameCount % 30 == 0) {
                        std::cout << "Frames: " << frameCount << " | FPS: " << fps
                                  << " | Errors: " << errorCount << std::endl;
                    }
                } else {
                    std::cerr << "Warning: Failed to decode JPEG (" << imageData.size() << " bytes)" << std::endl;
                }
            } else {
                errorCount++;
                if (errorCount <= 3) {
                    std::cerr << "Error getting frame (ret=" << ret << "), attempt " << errorCount << std::endl;
                }
                if (errorCount > 10) {
                    std::cerr << "Too many errors, is videohub service running on robot?" << std::endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            // Handle OpenCV window events
            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q' || key == 27) {  // q, Q, or ESC
                std::cout << "Quit requested" << std::endl;
                break;
            }

            // Target ~30 FPS
            auto frameEnd = std::chrono::steady_clock::now();
            auto frameTime = std::chrono::duration_cast<std::chrono::milliseconds>(frameEnd - frameStart);
            if (frameTime.count() < 33) {
                std::this_thread::sleep_for(std::chrono::milliseconds(33 - frameTime.count()));
            }
        }

        cv::destroyAllWindows();

        // Final stats
        auto endTime = std::chrono::steady_clock::now();
        double totalTime = std::chrono::duration<double>(endTime - startTime).count();

        std::cout << "\n=== Test Results ===" << std::endl;
        std::cout << "Total frames: " << frameCount << std::endl;
        std::cout << "Total errors: " << errorCount << std::endl;
        std::cout << "Duration: " << totalTime << " seconds" << std::endl;
        if (totalTime > 0) {
            std::cout << "Average FPS: " << (frameCount / totalTime) << std::endl;
        }

        if (frameCount > 0) {
            std::cout << "\nSUCCESS: DDS video streaming works!" << std::endl;
            return 0;
        } else {
            std::cout << "\nFAILED: No frames received" << std::endl;
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

#endif
}
