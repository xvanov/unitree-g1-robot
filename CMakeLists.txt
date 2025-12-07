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

# zstd compression library
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED libzstd)

# Livox SDK2 (for Mid-360 LiDAR on G1 robot)
# Check multiple locations: external/, /opt (Docker), system install
# IMPORTANT: Prefer shared library to avoid symbol conflicts with rapidjson/spdlog
set(LIVOX_SDK2_FOUND FALSE)
if(EXISTS "${CMAKE_SOURCE_DIR}/external/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so")
    set(LIVOX_SDK2_FOUND TRUE)
    set(LIVOX_SDK2_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/Livox-SDK2/include")
    set(LIVOX_SDK2_LIBRARY "${CMAKE_SOURCE_DIR}/external/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so")
    message(STATUS "Found Livox SDK2 shared lib (local external/): ${LIVOX_SDK2_LIBRARY}")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/external/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_static.a")
    set(LIVOX_SDK2_FOUND TRUE)
    set(LIVOX_SDK2_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/Livox-SDK2/include")
    set(LIVOX_SDK2_LIBRARY "${CMAKE_SOURCE_DIR}/external/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_static.a")
    message(STATUS "Found Livox SDK2 static lib (local external/): ${LIVOX_SDK2_LIBRARY}")
elseif(EXISTS "/opt/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so")
    set(LIVOX_SDK2_FOUND TRUE)
    set(LIVOX_SDK2_INCLUDE_DIR "/opt/Livox-SDK2/include")
    set(LIVOX_SDK2_LIBRARY "/opt/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so")
    message(STATUS "Found Livox SDK2 shared lib (Docker /opt): ${LIVOX_SDK2_LIBRARY}")
elseif(EXISTS "/opt/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_static.a")
    # Docker container location
    set(LIVOX_SDK2_FOUND TRUE)
    set(LIVOX_SDK2_INCLUDE_DIR "/opt/Livox-SDK2/include")
    set(LIVOX_SDK2_LIBRARY "/opt/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_static.a")
    message(STATUS "Found Livox SDK2 static lib (Docker /opt): ${LIVOX_SDK2_LIBRARY}")
else()
    # Check system install
    find_library(LIVOX_SDK2_LIBRARY NAMES livox_lidar_sdk_static livox_lidar_sdk_shared
        PATHS /usr/local/lib /usr/lib)
    find_path(LIVOX_SDK2_INCLUDE_DIR livox_lidar_api.h
        PATHS /usr/local/include /usr/include)
    if(LIVOX_SDK2_LIBRARY AND LIVOX_SDK2_INCLUDE_DIR)
        set(LIVOX_SDK2_FOUND TRUE)
        message(STATUS "Found Livox SDK2 (system): ${LIVOX_SDK2_LIBRARY}")
    else()
        message(STATUS "Livox SDK2 not found - LiDAR features will use DDS fallback")
    endif()
endif()

# msgpack-c (header-only mode)
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp
    PATHS /usr/include /usr/local/include
    PATH_SUFFIXES msgpack
)
if(MSGPACK_INCLUDE_DIR)
    message(STATUS "Found msgpack: ${MSGPACK_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "msgpack-c not found - install libmsgpack-dev")
endif()

# libharu (hpdf)
find_library(HPDF_LIBRARY hpdf REQUIRED)
find_path(HPDF_INCLUDE_DIR hpdf.h)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/sim
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${HPDF_INCLUDE_DIR}
    ${MSGPACK_INCLUDE_DIR}
    ${ZSTD_INCLUDE_DIRS}
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
# Capture (Story 1-7)
# ============================================

# Capture library (ImageCapture, PlanCorrelator)
add_library(capture
    src/capture/ImageCapture.cpp
    src/capture/PlanCorrelator.cpp
)
target_include_directories(capture PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(capture
    plan
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

# ============================================
# Detection (Story 1-8)
# ============================================

# Detection library (VlmClient, ImageAnnotator, DefectTypes)
add_library(detection
    src/detection/VlmClient.cpp
    src/detection/ImageAnnotator.cpp
    src/detection/DefectTypes.cpp
)
target_include_directories(detection PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(detection
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# ============================================
# Report Generation (Story 1-9)
# ============================================

# Report library (ReportGenerator, PlanOverlay)
add_library(report
    src/report/ReportGenerator.cpp
    src/report/PlanOverlay.cpp
)
target_include_directories(report PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(report
    detection    # For Defect types
    capture      # For ImageMetadata
    plan         # For PlanInfo
    ${OpenCV_LIBS}
    ${HPDF_LIBRARY}
    nlohmann_json::nlohmann_json
)

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

# Sensors library (SensorManager + RealSensorSource + LivoxLidar)
add_library(sensors
    src/sensors/SensorManager.cpp
    src/sensors/RealSensorSource.cpp
    src/sensors/LivoxLidar.cpp
)
target_include_directories(sensors PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Conditionally add Livox SDK2 support
if(LIVOX_SDK2_FOUND)
    target_include_directories(sensors PRIVATE ${LIVOX_SDK2_INCLUDE_DIR})
    target_link_libraries(sensors ${LIVOX_SDK2_LIBRARY})
    target_compile_definitions(sensors PRIVATE HAS_LIVOX_SDK2)
    message(STATUS "Livox SDK2 enabled for sensors library")
endif()

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

# ============================================
# Safety System (Story 1-5)
# ============================================

# Safety library
add_library(safety
    src/safety/SafetyMonitor.cpp
)
target_include_directories(safety PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Safety depends on locomotion and sensors
target_link_libraries(safety sensors loco_hw)

# ============================================
# Recording (Story 2-1)
# ============================================

# Recording library (SensorRecorder with msgpack/zstd)
add_library(recording
    src/recording/SensorRecorder.cpp
)
target_include_directories(recording PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(recording
    ${OpenCV_LIBS}
    ${ZSTD_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# ============================================
# Depth Stream Client (no RealSense required - for dev machine)
# ============================================
add_library(depth_stream_client
    src/depth/DepthStreamClient.cpp
)
target_include_directories(depth_stream_client PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${MSGPACK_INCLUDE_DIR}
)
target_link_libraries(depth_stream_client
    ${OpenCV_LIBS}
    pthread
)

# ============================================
# Teleop (Story 2-1)
# ============================================

# Teleop library (TeleopController, KeyboardTeleop, TeleopRunner, DDSVideoClient)
add_library(teleop
    src/teleop/TeleopController.cpp
    src/teleop/KeyboardTeleop.cpp
    src/teleop/TeleopRunner.cpp
    src/teleop/DDSVideoClient.cpp
)
target_include_directories(teleop PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(teleop
    sensors
    loco_hw
    recording
    depth_stream_client
    ${OpenCV_LIBS}
)
if(unitree_sdk2_FOUND)
    target_link_libraries(teleop unitree_sdk2)
    target_compile_definitions(teleop PRIVATE HAS_UNITREE_SDK2)
endif()

# ============================================
# Replay (Story 2-2)
# ============================================

# Replay library (SensorReplayer, ReplaySensorSource, ReplayController, ReplayRunner)
add_library(replay
    src/replay/SensorReplayer.cpp
    src/replay/ReplaySensorSource.cpp
    src/replay/ReplayController.cpp
    src/replay/ReplayRunner.cpp
)
target_include_directories(replay PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(replay
    recording    # For RecordingTypes
    sensors      # For ISensorSource
    ${OpenCV_LIBS}
    ${ZSTD_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# ============================================
# Depth Capture and Mapping
# ============================================

# Try to find RealSense SDK (optional)
# First try standard cmake config, then fall back to ROS-installed version
find_package(realsense2 QUIET)

if(NOT realsense2_FOUND)
    # Try to find librealsense2 from ROS Noetic install (common on Unitree robots)
    find_library(REALSENSE2_LIBRARY realsense2
        PATHS
            /opt/ros/noetic/lib/aarch64-linux-gnu
            /opt/ros/noetic/lib/x86_64-linux-gnu
            /opt/ros/noetic/lib
            /usr/lib/aarch64-linux-gnu
            /usr/lib/x86_64-linux-gnu
            /usr/local/lib
    )
    find_path(REALSENSE2_INCLUDE_DIR librealsense2/rs.hpp
        PATHS
            /opt/ros/noetic/include
            /usr/include
            /usr/local/include
    )
    if(REALSENSE2_LIBRARY AND REALSENSE2_INCLUDE_DIR)
        set(realsense2_FOUND TRUE)
        set(realsense2_LIBRARY ${REALSENSE2_LIBRARY})
        set(realsense2_INCLUDE_DIR ${REALSENSE2_INCLUDE_DIR})
        message(STATUS "Found RealSense (ROS): ${REALSENSE2_LIBRARY}")
    endif()
endif()

# Try to find PCL (optional)
find_package(PCL QUIET COMPONENTS common io)

if(realsense2_FOUND)
    # Depth capture library (requires RealSense)
    add_library(depth_capture
        src/capture/DepthCapture.cpp
    )
    target_include_directories(depth_capture PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        ${REALSENSE2_INCLUDE_DIR}
    )
    target_link_libraries(depth_capture
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
        nlohmann_json::nlohmann_json
        pthread
    )
    target_compile_definitions(depth_capture PUBLIC HAS_REALSENSE)
    message(STATUS "RealSense found - depth capture enabled")
else()
    message(STATUS "RealSense SDK not found - depth capture disabled")
endif()

# Mapping library (point cloud generation - no hardware dependency)
add_library(mapping
    src/mapping/PointCloudGenerator.cpp
)
target_include_directories(mapping PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(mapping
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

if(PCL_FOUND)
    target_include_directories(mapping PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(mapping ${PCL_LIBRARIES})
    target_compile_definitions(mapping PRIVATE HAS_PCL)
    message(STATUS "PCL found - advanced point cloud features enabled")
endif()

# Point cloud generation tool
add_executable(generate_pointcloud tools/generate_pointcloud.cpp)
target_link_libraries(generate_pointcloud mapping ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

# DDS Video test tool
add_executable(test_dds_video tools/test_dds_video.cpp)
target_link_libraries(test_dds_video ${OpenCV_LIBS})
if(unitree_sdk2_FOUND)
    target_link_libraries(test_dds_video unitree_sdk2)
    target_compile_definitions(test_dds_video PRIVATE HAS_UNITREE_SDK2)
endif()

# RealSense test tool
if(realsense2_FOUND)
    add_executable(test_realsense tools/test_realsense.cpp)
    target_include_directories(test_realsense PRIVATE ${REALSENSE2_INCLUDE_DIR})
    target_link_libraries(test_realsense depth_capture ${OpenCV_LIBS})
    target_compile_definitions(test_realsense PRIVATE HAS_REALSENSE)

    # Depth streaming library (server runs on robot, client runs on dev machine)
    add_library(depth_stream
        src/depth/DepthStreamServer.cpp
    )
    target_include_directories(depth_stream PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        ${REALSENSE2_INCLUDE_DIR}
        ${MSGPACK_INCLUDE_DIR}
    )
    target_link_libraries(depth_stream
        depth_capture
        ${OpenCV_LIBS}
        pthread
    )
    target_compile_definitions(depth_stream PUBLIC HAS_REALSENSE)

    # Depth stream server tool (for robot)
    add_executable(depth_stream_server tools/depth_stream_server.cpp)
    target_include_directories(depth_stream_server PRIVATE ${REALSENSE2_INCLUDE_DIR})
    target_link_libraries(depth_stream_server depth_stream ${OpenCV_LIBS})
    target_compile_definitions(depth_stream_server PRIVATE HAS_REALSENSE)
endif()

# Depth capture tool (receives stream from robot, saves locally)
add_executable(depth_capture_tool tools/depth_capture.cpp)
target_link_libraries(depth_capture_tool depth_stream_client ${OpenCV_LIBS})

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
    safety
    plan
    app
    capture
    detection
    report
    teleop
    recording
    replay
    mapping
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    ${HPDF_LIBRARY}
    ${ZSTD_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# Conditionally link depth_capture if RealSense is available
if(realsense2_FOUND)
    target_link_libraries(g1_inspector depth_capture)
    target_compile_definitions(g1_inspector PRIVATE HAS_REALSENSE)
endif()

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

# DetectionSim executable (Story 1-8)
add_executable(detection_sim
    sim/detection_sim/DetectionSim.cpp
    sim/detection_sim/main.cpp
)
target_link_libraries(detection_sim
    detection
    plan
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
    add_test(NAME test_navigation COMMAND test_navigation
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # SLAM unit tests
    add_executable(test_slam test/test_slam.cpp)
    target_link_libraries(test_slam
        slam
        GTest::gtest_main
    )
    add_test(NAME test_slam COMMAND test_slam
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # State Machine unit tests (Story 1-6)
    add_executable(test_state_machine test/test_state_machine.cpp)
    target_link_libraries(test_state_machine
        app
        GTest::gtest_main
    )
    add_test(NAME test_state_machine COMMAND test_state_machine
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # Plan Manager unit tests (Story 1-6)
    add_executable(test_plan_manager test/test_plan_manager.cpp)
    target_compile_definitions(test_plan_manager PRIVATE
        TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test_data"
    )
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
    add_test(NAME test_hardware COMMAND test_hardware
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # Safety unit tests (Story 1-5)
    add_executable(test_safety test/test_safety.cpp)
    target_link_libraries(test_safety
        safety
        GTest::gtest_main
    )
    add_test(NAME test_safety COMMAND test_safety
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # Capture unit tests (Story 1-7)
    add_executable(test_capture test/test_capture.cpp)
    target_link_libraries(test_capture
        capture
        nlohmann_json::nlohmann_json
        GTest::gtest_main
    )
    add_test(NAME test_capture COMMAND test_capture
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # Detection unit tests (Story 1-8)
    add_executable(test_detection test/test_detection.cpp)
    target_link_libraries(test_detection
        detection
        GTest::gtest_main
    )
    add_test(NAME test_detection COMMAND test_detection
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # End-to-end test for Story 1-6
    add_test(NAME test_e2e_story_1_6
        COMMAND ${CMAKE_SOURCE_DIR}/test/test_e2e_story_1_6.sh
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )

    # End-to-end test for Story 1-7 (Visual Capture)
    add_test(NAME test_e2e_story_1_7
        COMMAND ${CMAKE_SOURCE_DIR}/test/test_e2e_story_1_7.sh
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )

    # End-to-end test for Story 1-8 (VLM Defect Detection)
    add_test(NAME test_e2e_story_1_8
        COMMAND ${CMAKE_SOURCE_DIR}/test/test_e2e_story_1_8.sh
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )

    # Report unit tests (Story 1-9)
    add_executable(test_report test/test_report.cpp)
    target_link_libraries(test_report
        report
        GTest::gtest_main
    )
    add_test(NAME test_report COMMAND test_report
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # End-to-end test for Story 1-9 (Report Generation)
    add_test(NAME test_e2e_story_1_9
        COMMAND ${CMAKE_SOURCE_DIR}/test/test_e2e_story_1_9.sh
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )

    # Teleop unit tests (Story 2-1)
    add_executable(test_teleop test/test_teleop.cpp)
    target_link_libraries(test_teleop
        teleop
        GTest::gtest_main
    )
    add_test(NAME test_teleop COMMAND test_teleop
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # Recording unit tests (Story 2-1)
    add_executable(test_recording test/test_recording.cpp)
    target_link_libraries(test_recording
        recording
        GTest::gtest_main
    )
    add_test(NAME test_recording COMMAND test_recording
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # End-to-end test for Story 2-1 (Teleop and Sensor Recording)
    add_test(NAME test_e2e_story_2_1
        COMMAND ${CMAKE_SOURCE_DIR}/test/test_e2e_story_2_1.sh
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )

    # Replay unit tests (Story 2-2)
    add_executable(test_replay test/test_replay.cpp)
    target_link_libraries(test_replay
        replay
        recording
        GTest::gtest_main
    )
    add_test(NAME test_replay COMMAND test_replay
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    # End-to-end test for Story 2-2 (Replay System)
    add_test(NAME test_e2e_story_2_2
        COMMAND ${CMAKE_SOURCE_DIR}/test/test_e2e_story_2_2.sh
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
else()
    message(WARNING "GTest not found - unit tests will not be built")
endif()
