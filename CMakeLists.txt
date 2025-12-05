cmake_minimum_required(VERSION 3.16)
project(g1_inspector VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# unitree_sdk2 - find from system install
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")
find_package(unitree_sdk2 QUIET)
if(NOT unitree_sdk2_FOUND)
    message(WARNING "unitree_sdk2 not found - robot features will be disabled")
endif()

# OpenCV
find_package(OpenCV REQUIRED)

# curl
find_package(CURL REQUIRED)

# nlohmann/json
find_package(nlohmann_json 3.2.0 REQUIRED)

# libharu (hpdf)
find_library(HPDF_LIBRARY hpdf REQUIRED)
find_path(HPDF_INCLUDE_DIR hpdf.h)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/sim
    ${OpenCV_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${HPDF_INCLUDE_DIR}
)

# Navigation library
add_library(navigation
    src/navigation/Costmap.cpp
    src/navigation/Planner.cpp
    src/navigation/PathFollower.cpp
)
target_link_libraries(navigation ${OpenCV_LIBS})

# Main executable
add_executable(g1_inspector
    src/main.cpp
)

# Link libraries
target_link_libraries(g1_inspector
    navigation
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    ${HPDF_LIBRARY}
    nlohmann_json::nlohmann_json
)

# Conditionally link unitree_sdk2 if found
if(unitree_sdk2_FOUND)
    target_link_libraries(g1_inspector unitree_sdk2)
    target_compile_definitions(g1_inspector PRIVATE HAS_UNITREE_SDK2)
endif()

# NavSim executable
add_executable(nav_sim
    sim/nav_sim/NavSim.cpp
    sim/nav_sim/SimLocomotion.cpp
    sim/nav_sim/SimSensorSource.cpp
    sim/nav_sim/main.cpp
)
target_link_libraries(nav_sim
    navigation
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

# Unit tests (requires Google Test)
find_package(GTest QUIET)
if(GTest_FOUND)
    enable_testing()
    add_executable(test_navigation test/test_navigation.cpp)
    target_link_libraries(test_navigation
        navigation
        GTest::gtest_main
    )
    add_test(NAME test_navigation COMMAND test_navigation)
else()
    message(WARNING "GTest not found - unit tests will not be built")
endif()
