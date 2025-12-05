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

# ============================================
# SLAM (Story 1-3)
# ============================================

# SLAM library
add_library(slam
    src/slam/GridMapper.cpp
    src/slam/Localizer.cpp
)
target_link_libraries(slam ${OpenCV_LIBS})

# ============================================
# Plan Manager (Story 1-6)
# ============================================

# Check for poppler-cpp (optional PDF support)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(POPPLER poppler-cpp)
endif()

# Plan library
add_library(plan
    src/plan/PlanManager.cpp
)
target_include_directories(plan PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(plan ${OpenCV_LIBS})

if(POPPLER_FOUND)
    target_link_libraries(plan ${POPPLER_LIBRARIES})
    target_include_directories(plan PRIVATE ${POPPLER_INCLUDE_DIRS})
    target_compile_definitions(plan PRIVATE HAS_POPPLER)
    message(STATUS "Poppler found - PDF support enabled")
else()
    message(STATUS "Poppler not found - PDF support disabled (PNG only)")
endif()

# ============================================
# Application (Story 1-6)
# ============================================

# App library (StateMachine, CliHandler)
add_library(app
    src/app/StateMachine.cpp
    src/app/CliHandler.cpp
)
target_include_directories(app PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(app plan)

# ============================================
# Hardware Integration (Story 1-4)
# ============================================

# Sensors library (SensorManager + RealSensorSource)
add_library(sensors
    src/sensors/SensorManager.cpp
    src/sensors/RealSensorSource.cpp
)
target_include_directories(sensors PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Locomotion hardware library (LocoController + RealLocomotion)
add_library(loco_hw
    src/locomotion/LocoController.cpp
    src/locomotion/RealLocomotion.cpp
)
target_include_directories(loco_hw PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Conditionally link unitree_sdk2 to hardware libraries
if(unitree_sdk2_FOUND)
    target_link_libraries(sensors unitree_sdk2)
    target_link_libraries(loco_hw unitree_sdk2)
    target_compile_definitions(sensors PRIVATE HAS_UNITREE_SDK2)
    target_compile_definitions(loco_hw PRIVATE HAS_UNITREE_SDK2)
endif()

# Main executable
add_executable(g1_inspector
    src/main.cpp
)

# Link libraries
target_link_libraries(g1_inspector
    navigation
    slam
    sensors
    loco_hw
    plan
    app
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    ${HPDF_LIBRARY}
    nlohmann_json::nlohmann_json
)

# Conditionally link unitree_sdk2 to main executable
if(unitree_sdk2_FOUND)
    target_link_libraries(g1_inspector unitree_sdk2)
    target_compile_definitions(g1_inspector PRIVATE HAS_UNITREE_SDK2)
endif()

# NavSim library (shared between nav_sim and slam_sim)
add_library(nav_sim_lib
    sim/nav_sim/NavSim.cpp
)
target_link_libraries(nav_sim_lib ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

# NavSim executable
add_executable(nav_sim
    sim/nav_sim/SimLocomotion.cpp
    sim/nav_sim/SimSensorSource.cpp
    sim/nav_sim/main.cpp
)
target_link_libraries(nav_sim
    nav_sim_lib
    navigation
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

# SlamSim executable - reuses NavSim via nav_sim_lib
add_executable(slam_sim
    sim/slam_sim/SlamSim.cpp
    sim/slam_sim/main.cpp
)
target_link_libraries(slam_sim
    slam
    nav_sim_lib
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

    # SLAM unit tests
    add_executable(test_slam test/test_slam.cpp)
    target_link_libraries(test_slam
        slam
        GTest::gtest_main
    )
    add_test(NAME test_slam COMMAND test_slam)

    # State Machine unit tests (Story 1-6)
    add_executable(test_state_machine test/test_state_machine.cpp)
    target_link_libraries(test_state_machine
        app
        GTest::gtest_main
    )
    add_test(NAME test_state_machine COMMAND test_state_machine)

    # Plan Manager unit tests (Story 1-6)
    add_executable(test_plan_manager test/test_plan_manager.cpp)
    target_link_libraries(test_plan_manager
        plan
        GTest::gtest_main
    )
    add_test(NAME test_plan_manager COMMAND test_plan_manager)

    # Hardware unit tests (Story 1-4)
    add_executable(test_hardware test/test_hardware.cpp)
    target_link_libraries(test_hardware
        loco_hw
        GTest::gtest_main
    )
    add_test(NAME test_hardware COMMAND test_hardware)

    # End-to-end test for Story 1-6
    add_test(NAME test_e2e_story_1_6
        COMMAND ${CMAKE_SOURCE_DIR}/test/test_e2e_story_1_6.sh
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
else()
    message(WARNING "GTest not found - unit tests will not be built")
endif()
