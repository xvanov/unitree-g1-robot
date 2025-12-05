// Map generator for SLAM testing
// Compile: g++ -o generate_maps generate_maps.cpp `pkg-config --cflags --libs opencv4`

#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

cv::Mat createCorridorMap(int width = 200, int height = 200) {
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(255));  // White = free

    // Outer walls (black = obstacle)
    cv::rectangle(img, cv::Point(0, 0), cv::Point(width, 10), 0, -1);      // Top
    cv::rectangle(img, cv::Point(0, height-10), cv::Point(width, height), 0, -1);  // Bottom
    cv::rectangle(img, cv::Point(0, 0), cv::Point(10, height), 0, -1);     // Left
    cv::rectangle(img, cv::Point(width-10, 0), cv::Point(width, height), 0, -1);   // Right

    // Inner walls creating L-corridor
    cv::rectangle(img, cv::Point(60, 10), cv::Point(70, 120), 0, -1);
    cv::rectangle(img, cv::Point(70, 110), cv::Point(140, 120), 0, -1);
    cv::rectangle(img, cv::Point(130, 80), cv::Point(140, 190), 0, -1);

    return img;
}

cv::Mat createRoomsMap(int width = 200, int height = 200) {
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(255));

    // Outer walls
    cv::rectangle(img, cv::Point(0, 0), cv::Point(width, 8), 0, -1);
    cv::rectangle(img, cv::Point(0, height-8), cv::Point(width, height), 0, -1);
    cv::rectangle(img, cv::Point(0, 0), cv::Point(8, height), 0, -1);
    cv::rectangle(img, cv::Point(width-8, 0), cv::Point(width, height), 0, -1);

    // Center cross walls
    cv::rectangle(img, cv::Point(8, 96), cv::Point(192, 104), 0, -1);   // Horizontal
    cv::rectangle(img, cv::Point(96, 8), cv::Point(104, 192), 0, -1);   // Vertical

    // Doorways (openings)
    cv::rectangle(img, cv::Point(40, 96), cv::Point(60, 104), 255, -1);
    cv::rectangle(img, cv::Point(140, 96), cv::Point(160, 104), 255, -1);
    cv::rectangle(img, cv::Point(96, 40), cv::Point(104, 60), 255, -1);
    cv::rectangle(img, cv::Point(96, 140), cv::Point(104, 160), 255, -1);

    // Furniture in rooms
    cv::rectangle(img, cv::Point(30, 30), cv::Point(50, 50), 0, -1);
    cv::rectangle(img, cv::Point(30, 140), cv::Point(45, 160), 0, -1);
    cv::rectangle(img, cv::Point(150, 30), cv::Point(170, 45), 0, -1);
    cv::rectangle(img, cv::Point(150, 150), cv::Point(170, 170), 0, -1);

    return img;
}

cv::Mat createMazeMap(int width = 200, int height = 200) {
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(255));

    // Outer walls
    cv::rectangle(img, cv::Point(0, 0), cv::Point(width, 8), 0, -1);
    cv::rectangle(img, cv::Point(0, height-8), cv::Point(width, height), 0, -1);
    cv::rectangle(img, cv::Point(0, 0), cv::Point(8, height), 0, -1);
    cv::rectangle(img, cv::Point(width-8, 0), cv::Point(width, height), 0, -1);

    // Maze walls
    cv::rectangle(img, cv::Point(50, 8), cv::Point(58, 60), 0, -1);
    cv::rectangle(img, cv::Point(50, 52), cv::Point(120, 60), 0, -1);
    cv::rectangle(img, cv::Point(112, 52), cv::Point(120, 140), 0, -1);
    cv::rectangle(img, cv::Point(40, 132), cv::Point(120, 140), 0, -1);
    cv::rectangle(img, cv::Point(40, 80), cv::Point(48, 140), 0, -1);
    cv::rectangle(img, cv::Point(48, 80), cv::Point(90, 88), 0, -1);
    cv::rectangle(img, cv::Point(150, 80), cv::Point(158, 160), 0, -1);
    cv::rectangle(img, cv::Point(70, 152), cv::Point(158, 160), 0, -1);
    cv::rectangle(img, cv::Point(70, 100), cv::Point(110, 108), 0, -1);

    return img;
}

cv::Mat createOpenSpaceMap(int width = 200, int height = 200) {
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(255));

    // Outer walls
    cv::rectangle(img, cv::Point(0, 0), cv::Point(width, 6), 0, -1);
    cv::rectangle(img, cv::Point(0, height-6), cv::Point(width, height), 0, -1);
    cv::rectangle(img, cv::Point(0, 0), cv::Point(6, height), 0, -1);
    cv::rectangle(img, cv::Point(width-6, 0), cv::Point(width, height), 0, -1);

    // Scattered obstacles (pillars)
    cv::circle(img, cv::Point(40, 40), 15, 0, -1);
    cv::circle(img, cv::Point(120, 50), 12, 0, -1);
    cv::circle(img, cv::Point(60, 120), 18, 0, -1);
    cv::circle(img, cv::Point(150, 100), 14, 0, -1);
    cv::circle(img, cv::Point(100, 160), 16, 0, -1);
    cv::circle(img, cv::Point(170, 160), 10, 0, -1);
    cv::circle(img, cv::Point(30, 170), 12, 0, -1);

    return img;
}

cv::Mat createLargeWarehouseMap(int width = 400, int height = 400) {
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(255));

    // Outer walls
    cv::rectangle(img, cv::Point(0, 0), cv::Point(width, 10), 0, -1);
    cv::rectangle(img, cv::Point(0, height-10), cv::Point(width, height), 0, -1);
    cv::rectangle(img, cv::Point(0, 0), cv::Point(10, height), 0, -1);
    cv::rectangle(img, cv::Point(width-10, 0), cv::Point(width, height), 0, -1);

    // Shelving units (rows of rectangles)
    for (int row = 0; row < 4; row++) {
        int y = 50 + row * 90;
        for (int col = 0; col < 5; col++) {
            int x = 30 + col * 75;
            cv::rectangle(img, cv::Point(x, y), cv::Point(x + 50, y + 20), 0, -1);
        }
    }

    return img;
}

int main() {
    std::string output_dir = "test_data";
    fs::create_directories(output_dir);

    struct MapInfo {
        std::string name;
        cv::Mat img;
    };

    std::vector<MapInfo> maps = {
        {"corridor", createCorridorMap()},
        {"rooms", createRoomsMap()},
        {"maze", createMazeMap()},
        {"open_space", createOpenSpaceMap()},
        {"warehouse", createLargeWarehouseMap()},
    };

    for (const auto& m : maps) {
        std::string path = output_dir + "/" + m.name + ".png";
        cv::imwrite(path, m.img);
        std::cout << "Created: " << path << " (" << m.img.cols << "x" << m.img.rows << ")" << std::endl;
    }

    std::cout << "\nGenerated " << maps.size() << " test maps in " << output_dir << "/" << std::endl;
    return 0;
}
