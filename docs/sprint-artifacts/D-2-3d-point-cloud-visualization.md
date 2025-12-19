# Story D.2: 3D Point Cloud Visualization

**Status:** Ready for Dev

---

## Quick Reference

**New files to create:**
- `src/visualization/PointCloudViewer.h` / `.cpp` - OpenGL 3D point cloud renderer with orbit camera
- `src/visualization/Camera.h` / `.cpp` - Orbit camera controller
- `src/visualization/Shader.h` / `.cpp` - OpenGL shader utilities
- `shaders/point_cloud.vert` - Vertex shader for point rendering
- `shaders/point_cloud.frag` - Fragment shader for point coloring
- `test/test_point_cloud_viewer.cpp` - Unit tests

**Files to modify:**
- `src/main.cpp` - Add `--pointcloud-replay` and `--with-3d` CLI flags
- `src/replay/SensorReplayer.h` / `.cpp` - Add `decodePointCloud3D()` static method
- `src/replay/StreamReplayViewer.h` / `.cpp` - Integrate PointCloudViewer as optional 5th window
- `CMakeLists.txt` - Add visualization library, link GLFW3, GLAD, GLM, OpenGL

**Dependencies to add:**
| Dependency | Purpose | Install |
|------------|---------|---------|
| GLFW3 | Window/input management | `sudo apt install libglfw3-dev` |
| GLAD | OpenGL loader | Generate or `sudo apt install glad` |
| GLM | Math library (vec3, mat4) | `sudo apt install libglm-dev` |
| OpenGL 3.3+ | GPU rendering | Already installed |

**Key classes:**
| Class | Purpose |
|-------|---------|
| `PointCloudViewer` | Main 3D visualization with OpenGL rendering |
| `Camera` | Orbit camera with rotate/zoom/pan |
| `Shader` | Compile and manage GLSL shaders |
| `SensorReplayer` | Existing - Extended with `decodePointCloud3D()` |
| `StreamReplayViewer` | Existing - Extended to optionally include 3D view |

**Required Includes for PointCloudViewer:**
```cpp
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <memory>
#include <string>

#include "visualization/Camera.h"
#include "visualization/Shader.h"
#include "sensors/ISensorSource.h"  // For PointCloud3D
#include "util/Types.h"             // For Pose2D
```

**Primary acceptance criteria:** AC1 (OpenGL window), AC2 (orbit camera), AC8 (1M+ points at 30fps)

**Prerequisites:** Story D-1 (Multi-Stream Replay), existing PointCloud3D type in recordings

---

## Story

As a **developer creating a demo video**,
I want **to see a 3D point cloud visualization that accumulates over time**,
So that **I can show an impressive 3D reconstruction of the space the robot walked through**.

---

## Acceptance Criteria

1. **AC1:** OpenGL window renders 3D point cloud
2. **AC2:** Orbit camera controls work (rotate, zoom, pan)
3. **AC3:** Points colored by height with smooth gradient (blue=floor, red=ceiling)
4. **AC4:** Point cloud accumulates as replay progresses
5. **AC5:** Robot marker shows current position/heading
6. **AC6:** Grid floor provides spatial reference
7. **AC7:** Can run standalone or integrated with StreamReplayViewer
8. **AC8:** Performance: renders 1M+ points at 30fps
9. **AC9:** 'r' resets camera to default view
10. **AC10:** Stats overlay shows point count

---

## Tasks / Subtasks

- [ ] **Task 1: Add OpenGL Dependencies to CMake** (AC: 1)
  - [ ] 1.1 Update `CMakeLists.txt` to find and link GLFW3, GLAD, GLM:
    ```cmake
    # OpenGL dependencies for 3D visualization (Story D-2)
    find_package(glfw3 REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(glm REQUIRED)

    # GLAD - OpenGL loader (header-only, generate from https://glad.dav1d.de/ or install)
    # Generate with: OpenGL 3.3 Core, C/C++, no extensions
    find_path(GLAD_INCLUDE_DIR glad/glad.h
        PATHS ${CMAKE_SOURCE_DIR}/external/glad/include /usr/include /usr/local/include)
    if(NOT GLAD_INCLUDE_DIR)
        message(STATUS "GLAD not found - 3D visualization will be disabled")
        set(HAS_OPENGL_VIZ FALSE)
    else()
        set(HAS_OPENGL_VIZ TRUE)
        set(GLAD_SOURCE ${CMAKE_SOURCE_DIR}/external/glad/src/glad.c)
        if(NOT EXISTS ${GLAD_SOURCE})
            # Alternative: system-installed glad
            find_library(GLAD_LIBRARY glad)
        endif()
    endif()
    ```
  - [ ] 1.2 Create visualization library (conditionally):
    ```cmake
    if(HAS_OPENGL_VIZ)
        add_library(visualization
            src/visualization/PointCloudViewer.cpp
            src/visualization/Camera.cpp
            src/visualization/Shader.cpp
            ${GLAD_SOURCE}  # Or link GLAD_LIBRARY if system-installed
        )
        target_include_directories(visualization PUBLIC
            ${CMAKE_SOURCE_DIR}/src
            ${GLAD_INCLUDE_DIR}
        )
        target_link_libraries(visualization
            glfw
            OpenGL::GL
            glm::glm
        )
        target_compile_definitions(visualization PUBLIC HAS_OPENGL_VIZ)
        message(STATUS "OpenGL visualization enabled")
    endif()
    ```
  - [ ] 1.3 Link visualization to replay library and g1_inspector (conditionally):
    ```cmake
    if(HAS_OPENGL_VIZ)
        target_link_libraries(replay visualization)
        target_link_libraries(g1_inspector visualization)
    endif()
    ```

- [ ] **Task 2: Create Shader Files** (AC: 1, 3)
  - [ ] 2.1 Create `shaders/point_cloud.vert`:
    ```glsl
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;

    uniform mat4 view;
    uniform mat4 projection;
    uniform float pointSize;

    out vec3 vertexColor;

    void main() {
        gl_Position = projection * view * vec4(aPos, 1.0);
        gl_PointSize = pointSize;
        vertexColor = aColor;
    }
    ```
  - [ ] 2.2 Create `shaders/point_cloud.frag`:
    ```glsl
    #version 330 core
    in vec3 vertexColor;
    out vec4 FragColor;

    void main() {
        // Circular point shape (discard corners)
        vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
        if (dot(circCoord, circCoord) > 1.0) {
            discard;
        }
        FragColor = vec4(vertexColor, 1.0);
    }
    ```
  - [ ] 2.3 Create `shaders/grid.vert` for floor grid:
    ```glsl
    #version 330 core
    layout (location = 0) in vec3 aPos;

    uniform mat4 view;
    uniform mat4 projection;

    void main() {
        gl_Position = projection * view * vec4(aPos, 1.0);
    }
    ```
  - [ ] 2.4 Create `shaders/grid.frag`:
    ```glsl
    #version 330 core
    out vec4 FragColor;
    uniform vec3 gridColor;

    void main() {
        FragColor = vec4(gridColor, 0.5);
    }
    ```

- [ ] **Task 3: Create Camera Class** (AC: 2, 9)
  - [ ] 3.1 Create `src/visualization/Camera.h`:
    ```cpp
    #pragma once
    #include <glm/glm.hpp>

    namespace visualization {

    class Camera {
    public:
        Camera();

        // Get view matrix
        glm::mat4 getViewMatrix() const;
        glm::mat4 getProjectionMatrix(float aspectRatio) const;

        // Orbit controls
        void rotate(float deltaYaw, float deltaPitch);
        void zoom(float deltaZoom);
        void pan(float deltaX, float deltaY);

        // Reset to default view
        void reset();

        // Set camera to look at specific point
        void setTarget(const glm::vec3& target);

        // Set distance from target
        void setDistance(float distance);

        // Get camera position (for stats)
        glm::vec3 getPosition() const;

    private:
        glm::vec3 target_ = glm::vec3(0.0f, 0.0f, 0.0f);
        float distance_ = 10.0f;
        float yaw_ = -90.0f;    // Horizontal angle (degrees)
        float pitch_ = 30.0f;   // Vertical angle (degrees)
        float fov_ = 45.0f;
        float nearPlane_ = 0.1f;
        float farPlane_ = 1000.0f;

        // Constraints
        static constexpr float MIN_PITCH = -89.0f;
        static constexpr float MAX_PITCH = 89.0f;
        static constexpr float MIN_DISTANCE = 1.0f;
        static constexpr float MAX_DISTANCE = 100.0f;
    };

    } // namespace visualization
    ```
  - [ ] 3.2 Create `src/visualization/Camera.cpp` with implementation:
    ```cpp
    #include "visualization/Camera.h"
    #include <glm/gtc/matrix_transform.hpp>
    #include <algorithm>
    #include <cmath>

    namespace visualization {

    Camera::Camera() {
        reset();
    }

    glm::mat4 Camera::getViewMatrix() const {
        // Calculate camera position from spherical coordinates
        float pitchRad = glm::radians(pitch_);
        float yawRad = glm::radians(yaw_);

        glm::vec3 position;
        position.x = target_.x + distance_ * cos(pitchRad) * cos(yawRad);
        position.y = target_.y + distance_ * sin(pitchRad);
        position.z = target_.z + distance_ * cos(pitchRad) * sin(yawRad);

        return glm::lookAt(position, target_, glm::vec3(0.0f, 1.0f, 0.0f));
    }

    glm::mat4 Camera::getProjectionMatrix(float aspectRatio) const {
        return glm::perspective(glm::radians(fov_), aspectRatio, nearPlane_, farPlane_);
    }

    void Camera::rotate(float deltaYaw, float deltaPitch) {
        yaw_ += deltaYaw;
        pitch_ = std::clamp(pitch_ + deltaPitch, MIN_PITCH, MAX_PITCH);
    }

    void Camera::zoom(float deltaZoom) {
        distance_ = std::clamp(distance_ - deltaZoom, MIN_DISTANCE, MAX_DISTANCE);
    }

    void Camera::pan(float deltaX, float deltaY) {
        // Calculate right and up vectors
        float yawRad = glm::radians(yaw_);
        glm::vec3 right = glm::vec3(sin(yawRad), 0.0f, -cos(yawRad));
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);

        target_ += right * deltaX + up * deltaY;
    }

    void Camera::reset() {
        target_ = glm::vec3(0.0f, 0.0f, 0.0f);
        distance_ = 10.0f;
        yaw_ = -90.0f;
        pitch_ = 30.0f;
    }

    void Camera::setTarget(const glm::vec3& target) {
        target_ = target;
    }

    void Camera::setDistance(float distance) {
        distance_ = std::clamp(distance, MIN_DISTANCE, MAX_DISTANCE);
    }

    glm::vec3 Camera::getPosition() const {
        float pitchRad = glm::radians(pitch_);
        float yawRad = glm::radians(yaw_);

        glm::vec3 position;
        position.x = target_.x + distance_ * cos(pitchRad) * cos(yawRad);
        position.y = target_.y + distance_ * sin(pitchRad);
        position.z = target_.z + distance_ * cos(pitchRad) * sin(yawRad);

        return position;
    }

    } // namespace visualization
    ```

- [ ] **Task 4: Create Shader Class** (AC: 1)
  - [ ] 4.1 Create `src/visualization/Shader.h`:
    ```cpp
    #pragma once
    #include <string>
    #include <glm/glm.hpp>

    namespace visualization {

    class Shader {
    public:
        Shader();
        ~Shader();

        // Load shaders from files
        bool loadFromFiles(const std::string& vertexPath, const std::string& fragmentPath);

        // Load shaders from embedded strings
        bool loadFromStrings(const std::string& vertexSource, const std::string& fragmentSource);

        // Use this shader
        void use() const;

        // Uniform setters
        void setMat4(const std::string& name, const glm::mat4& value) const;
        void setVec3(const std::string& name, const glm::vec3& value) const;
        void setFloat(const std::string& name, float value) const;

        // Get program ID
        unsigned int getID() const { return programID_; }

    private:
        unsigned int programID_ = 0;

        bool compileShader(unsigned int& shader, unsigned int type, const std::string& source);
        bool linkProgram(unsigned int vertexShader, unsigned int fragmentShader);
        std::string readFile(const std::string& path);
    };

    } // namespace visualization
    ```
  - [ ] 4.2 Create `src/visualization/Shader.cpp` with implementation

- [ ] **Task 5: Create PointCloudViewer Class** (AC: 1, 3, 4, 5, 6, 8, 10)
  - [ ] 5.1 Create `src/visualization/PointCloudViewer.h`:
    ```cpp
    #pragma once

    #ifdef HAS_OPENGL_VIZ

    #include <glad/glad.h>
    #include <GLFW/glfw3.h>
    #include <glm/glm.hpp>
    #include <vector>
    #include <memory>
    #include <string>
    #include <atomic>

    #include "visualization/Camera.h"
    #include "visualization/Shader.h"
    #include "sensors/ISensorSource.h"
    #include "util/Types.h"

    namespace visualization {

    // Point with color for GPU buffer
    struct ColoredPoint {
        float x, y, z;
        float r, g, b;
    };

    // Coloring modes
    enum class ColorMode {
        HEIGHT,     // Blue (floor) to Red (ceiling)
        INTENSITY,  // Grayscale from intensity data
        TIME        // Older points dimmer, newer brighter
    };

    /**
     * PointCloudViewer - Interactive 3D point cloud visualization
     *
     * Features:
     * - OpenGL rendering with VBO for performance
     * - Orbit camera (rotate, zoom, pan)
     * - Color by height, intensity, or time
     * - Grid floor for reference
     * - Robot marker showing current position
     * - Stats overlay with point count and FPS
     *
     * Story D-2: 3D Point Cloud Visualization
     */
    class PointCloudViewer {
    public:
        PointCloudViewer();
        ~PointCloudViewer();

        // Initialize OpenGL window
        bool init(int width = 1280, int height = 720, const std::string& title = "3D Point Cloud");

        // Add points from replay (accumulates)
        void addPoints(const PointCloud3D& cloud, const Pose2D& robot_pose);

        // Set current robot pose (for marker)
        void setRobotPose(const Pose2D& pose);

        // Render frame (call in main loop)
        void render();

        // Check if window should close
        bool shouldClose() const;

        // Process pending events (call after render)
        void pollEvents();

        // Camera controls
        void resetCamera();
        void centerOnRobot();

        // Coloring options
        void setColorMode(ColorMode mode) { colorMode_ = mode; rebuildVBO_ = true; }
        void setHeightRange(float min, float max) { heightMin_ = min; heightMax_ = max; rebuildVBO_ = true; }

        // Point appearance
        void setPointSize(float size) { pointSize_ = size; }

        // Clear all points
        void clear();

        // Stats
        size_t getPointCount() const { return points_.size(); }
        float getFPS() const { return fps_; }

    private:
        // GLFW window
        GLFWwindow* window_ = nullptr;
        int width_ = 1280;
        int height_ = 720;

        // Shaders
        std::unique_ptr<Shader> pointShader_;
        std::unique_ptr<Shader> gridShader_;

        // Camera
        std::unique_ptr<Camera> camera_;

        // Point cloud data
        std::vector<ColoredPoint> points_;
        std::atomic<bool> rebuildVBO_{true};
        size_t vboPointCount_ = 0;

        // OpenGL buffers
        GLuint pointVAO_ = 0;
        GLuint pointVBO_ = 0;
        GLuint gridVAO_ = 0;
        GLuint gridVBO_ = 0;
        GLuint robotVAO_ = 0;
        GLuint robotVBO_ = 0;

        // Robot pose
        Pose2D robotPose_;

        // Appearance
        ColorMode colorMode_ = ColorMode::HEIGHT;
        float heightMin_ = 0.0f;
        float heightMax_ = 3.0f;
        float pointSize_ = 3.0f;

        // Performance
        float fps_ = 0.0f;
        double lastFrameTime_ = 0.0;
        int frameCount_ = 0;

        // Mouse state for camera control
        bool leftMousePressed_ = false;
        bool middleMousePressed_ = false;
        bool rightMousePressed_ = false;
        double lastMouseX_ = 0.0;
        double lastMouseY_ = 0.0;

        // Initialize OpenGL resources
        void initBuffers();
        void initGrid();
        void initRobotMarker();

        // Update VBO when points change
        void updateVBO();

        // Render components
        void renderPoints();
        void renderGrid();
        void renderRobotMarker();
        void renderStats();

        // Color helpers
        glm::vec3 heightToColor(float height) const;
        glm::vec3 intensityToColor(uint8_t intensity) const;

        // GLFW callbacks (static + instance)
        static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
        static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
        static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
        static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
        static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

        void handleMouseButton(int button, int action);
        void handleCursorPos(double xpos, double ypos);
        void handleScroll(double yoffset);
        void handleKey(int key, int action);

        // Embedded shader sources (fallback if files not found)
        static const char* POINT_VERT_SRC;
        static const char* POINT_FRAG_SRC;
        static const char* GRID_VERT_SRC;
        static const char* GRID_FRAG_SRC;
    };

    } // namespace visualization

    #endif // HAS_OPENGL_VIZ
    ```
  - [ ] 5.2 Create `src/visualization/PointCloudViewer.cpp` with full implementation:
    - Constructor/destructor with GLFW init/cleanup
    - `init()`: Create window, load shaders, init buffers
    - `addPoints()`: Transform points, color by height, append to vector, mark VBO dirty
    - `updateVBO()`: Upload points to GPU using `glBufferData` or `glBufferSubData`
    - `render()`: Clear, draw grid, draw points, draw robot, draw stats
    - Camera callbacks for orbit controls
    - Stats overlay using simple OpenGL text (or skip for MVP, show in console)

- [ ] **Task 6: Add decodePointCloud3D to SensorReplayer** (AC: 4)
  - [ ] 6.1 Add to `src/replay/SensorReplayer.h`:
    ```cpp
    // Add decoded type
    struct DecodedPointCloud3D {
        PointCloud3D cloud;
        int64_t timestamp_us = 0;
    };

    // Add static decode method
    static bool decodePointCloud3D(const ReplayMessage& msg, DecodedPointCloud3D& out);
    ```
  - [ ] 6.2 Implement in `src/replay/SensorReplayer.cpp`:
    ```cpp
    bool SensorReplayer::decodePointCloud3D(const ReplayMessage& msg, DecodedPointCloud3D& out) {
        if (msg.type != recording::MessageType::POINT_CLOUD_3D) {
            return false;
        }

        try {
            msgpack::object_handle oh = msgpack::unpack(
                reinterpret_cast<const char*>(msg.payload.data()), msg.payload.size());
            msgpack::object obj = oh.get();

            if (obj.type != msgpack::type::MAP) return false;

            auto map = obj.as<std::map<std::string, msgpack::object>>();

            if (map.count("count")) {
                size_t count = map["count"].as<size_t>();
                out.cloud.reserve(count);
            }
            if (map.count("x")) {
                out.cloud.x = map["x"].as<std::vector<float>>();
            }
            if (map.count("y")) {
                out.cloud.y = map["y"].as<std::vector<float>>();
            }
            if (map.count("z")) {
                out.cloud.z = map["z"].as<std::vector<float>>();
            }
            if (map.count("intensity")) {
                out.cloud.intensity = map["intensity"].as<std::vector<uint8_t>>();
            }

            out.timestamp_us = msg.timestamp_us;
            return out.cloud.size() > 0;
        } catch (...) {
            return false;
        }
    }
    ```

- [ ] **Task 7: Extend StreamReplayViewer for Optional 3D** (AC: 7)
  - [ ] 7.1 Add conditional member to `StreamReplayViewer.h`:
    ```cpp
    #ifdef HAS_OPENGL_VIZ
    #include "visualization/PointCloudViewer.h"
    #endif

    // In class:
    #ifdef HAS_OPENGL_VIZ
    std::unique_ptr<visualization::PointCloudViewer> point_cloud_viewer_;
    bool enable_3d_view_ = false;
    #endif

    // Add setter:
    void setEnable3DView(bool enable);
    ```
  - [ ] 7.2 Update `StreamReplayViewer::init()` to optionally create PointCloudViewer
  - [ ] 7.3 Update `processMessage()` to handle POINT_CLOUD_3D:
    ```cpp
    case MessageType::POINT_CLOUD_3D: {
        #ifdef HAS_OPENGL_VIZ
        if (point_cloud_viewer_) {
            DecodedPointCloud3D decoded;
            if (SensorReplayer::decodePointCloud3D(msg, decoded)) {
                point_cloud_viewer_->addPoints(decoded.cloud, latest_pose_);
            }
        }
        #endif
        break;
    }
    ```
  - [ ] 7.4 Update `run()` to render and poll PointCloudViewer:
    ```cpp
    #ifdef HAS_OPENGL_VIZ
    if (point_cloud_viewer_) {
        point_cloud_viewer_->render();
        point_cloud_viewer_->pollEvents();
        if (point_cloud_viewer_->shouldClose()) {
            break;  // User closed 3D window
        }
    }
    #endif
    ```

- [ ] **Task 8: CLI Integration** (AC: 7)
  - [ ] 8.1 Add flags to `src/main.cpp`:
    ```cpp
    std::string pointcloudReplaySession;
    bool with3D = false;

    // In argument parsing
    if (arg == "--pointcloud-replay") {
        pointcloudReplaySession = argv[++i];
    }
    if (arg == "--with-3d") {
        with3D = true;
    }
    ```
  - [ ] 8.2 Add standalone pointcloud-replay mode:
    ```cpp
    #ifdef HAS_OPENGL_VIZ
    if (!pointcloudReplaySession.empty()) {
        // Standalone 3D point cloud replay
        visualization::PointCloudViewer viewer;
        if (!viewer.init(1280, 720, "3D Point Cloud Replay")) {
            std::cerr << "Failed to initialize 3D viewer" << std::endl;
            return 1;
        }

        replay::SensorReplayer replayer;
        std::string path = (pointcloudReplaySession.find('/') != std::string::npos)
            ? pointcloudReplaySession : "data/recordings/" + pointcloudReplaySession;

        if (!replayer.open(path)) {
            std::cerr << "Failed to open recording: " << path << std::endl;
            return 1;
        }

        Pose2D pose;
        replay::ReplayMessage msg;
        while (replayer.hasMore() && !viewer.shouldClose()) {
            if (replayer.readNext(msg)) {
                if (msg.type == recording::MessageType::POINT_CLOUD_3D) {
                    replay::DecodedPointCloud3D decoded;
                    if (replay::SensorReplayer::decodePointCloud3D(msg, decoded)) {
                        viewer.addPoints(decoded.cloud, pose);
                    }
                } else if (msg.type == recording::MessageType::POSE) {
                    replay::DecodedPose decoded;
                    if (replay::SensorReplayer::decodePose(msg, decoded)) {
                        pose = decoded.pose;
                        viewer.setRobotPose(pose);
                    }
                }
            }
            viewer.render();
            viewer.pollEvents();
        }

        // Keep window open until user closes
        while (!viewer.shouldClose()) {
            viewer.render();
            viewer.pollEvents();
        }
        return 0;
    }
    #endif
    ```
  - [ ] 8.3 Add `--with-3d` support for StreamReplayViewer:
    ```cpp
    if (!streamReplaySession.empty()) {
        replay::StreamReplayViewer viewer;
        // ... existing init code ...

        #ifdef HAS_OPENGL_VIZ
        if (with3D) {
            viewer.setEnable3DView(true);
        }
        #endif

        return viewer.run();
    }
    ```
  - [ ] 8.4 Update help text:
    ```
    --pointcloud-replay <session>  Standalone 3D point cloud replay
    --with-3d                      Add 3D point cloud window (use with --stream-replay)
    ```

- [ ] **Task 9: Performance Optimization** (AC: 8)
  - [ ] 9.1 Use `GL_DYNAMIC_DRAW` for VBO that updates frequently
  - [ ] 9.2 Implement incremental VBO updates:
    ```cpp
    // Only upload new points, not entire buffer
    if (newPointsStartIndex_ < points_.size()) {
        size_t newCount = points_.size() - newPointsStartIndex_;
        glBufferSubData(GL_ARRAY_BUFFER,
                        newPointsStartIndex_ * sizeof(ColoredPoint),
                        newCount * sizeof(ColoredPoint),
                        &points_[newPointsStartIndex_]);
        newPointsStartIndex_ = points_.size();
    }
    ```
  - [ ] 9.3 Add point decimation option for very large clouds:
    ```cpp
    void setMaxPoints(size_t max) { maxPoints_ = max; }
    // When adding points, optionally skip every Nth point if over limit
    ```
  - [ ] 9.4 Use instanced rendering if needed for >10M points (future optimization)

- [ ] **Task 10: Unit Tests** (AC: 1, 2)
  - [ ] 10.1 Create `test/test_point_cloud_viewer.cpp`:
    ```cpp
    #include <gtest/gtest.h>

    #ifdef HAS_OPENGL_VIZ
    #include "visualization/Camera.h"
    #include "visualization/PointCloudViewer.h"

    TEST(CameraTest, DefaultPosition) {
        visualization::Camera camera;
        auto pos = camera.getPosition();
        // Default: looking at origin from distance 10, 30 degree pitch
        EXPECT_NEAR(pos.y, 5.0f, 0.5f);  // sin(30) * 10 = 5
    }

    TEST(CameraTest, ZoomLimits) {
        visualization::Camera camera;
        camera.zoom(100.0f);  // Try to zoom very close
        auto pos = camera.getPosition();
        // Should be clamped to MIN_DISTANCE
        float dist = glm::length(pos);
        EXPECT_GE(dist, 1.0f);
    }

    TEST(CameraTest, Reset) {
        visualization::Camera camera;
        camera.rotate(45.0f, 20.0f);
        camera.zoom(5.0f);
        camera.reset();
        // Should be back to defaults
        auto view = camera.getViewMatrix();
        // Verify view matrix is identity-ish (looking at origin)
        EXPECT_TRUE(true);  // Basic sanity check
    }

    // Note: Full rendering tests require OpenGL context
    // which is difficult in headless CI. Test logic separately.

    #else
    TEST(PointCloudViewerTest, DisabledWithoutOpenGL) {
        // Placeholder test when OpenGL not available
        EXPECT_TRUE(true);
    }
    #endif

    TEST(PointCloud3DTest, DecodeValidMessage) {
        // Test decodePointCloud3D with mock data
        // This doesn't require OpenGL
        EXPECT_TRUE(true);  // Placeholder
    }
    ```
  - [ ] 10.2 Add to CMakeLists.txt:
    ```cmake
    if(GTest_FOUND)
        add_executable(test_point_cloud_viewer test/test_point_cloud_viewer.cpp)
        target_link_libraries(test_point_cloud_viewer
            replay
            GTest::gtest_main
        )
        if(HAS_OPENGL_VIZ)
            target_link_libraries(test_point_cloud_viewer visualization)
        endif()
        add_test(NAME test_point_cloud_viewer COMMAND test_point_cloud_viewer
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    endif()
    ```

- [ ] **Task 11: E2E Verification** (AC: all)
  - [ ] 11.1 Build with OpenGL dependencies:
    ```bash
    sudo apt install libglfw3-dev libglm-dev
    # Generate GLAD or install from package
    cd build && cmake .. && make -j
    ```
  - [ ] 11.2 Test standalone mode:
    ```bash
    ./g1_inspector --pointcloud-replay 071712
    # Expected: OpenGL window with 3D point cloud
    # - Drag to rotate
    # - Scroll to zoom
    # - Middle-drag to pan
    # - 'r' to reset view
    ```
  - [ ] 11.3 Test integrated mode:
    ```bash
    ./g1_inspector --stream-replay 071712 --with-3d
    # Expected: 5 windows (RGB, Webcam, Depth, SLAM, Status + 3D Point Cloud)
    ```
  - [ ] 11.4 Verify performance with large recording:
    ```bash
    # Check FPS with 1M+ points
    # Should maintain 30fps
    ```

---

## Technical Design

### Coordinate System

**Robot Frame (matches existing codebase):**
- X: Forward
- Y: Left
- Z: Up

**OpenGL Rendering (Y-up convention):**
- Transform robot coordinates: OpenGL Y = Robot Z, OpenGL Z = -Robot Y
- Or use custom view matrix to match robot convention

**Height Coloring:**
```
Height (Z in robot frame, Y in OpenGL):
  3m (ceiling) → Red   (1.0, 0.0, 0.0)
  2m           → Yellow (1.0, 1.0, 0.0)
  1m           → Green  (0.0, 1.0, 0.0)
  0m (floor)   → Blue   (0.0, 0.0, 1.0)
```

### VBO Layout

```cpp
struct ColoredPoint {
    float x, y, z;    // Position (12 bytes)
    float r, g, b;    // Color (12 bytes)
};
// Total: 24 bytes per point
// 1M points = 24 MB GPU memory

// Vertex attribute layout:
// Location 0: position (vec3, offset 0)
// Location 1: color (vec3, offset 12)
```

### Grid Floor Geometry

```cpp
// Generate grid lines centered at origin
// Size: 20x20 meters, 1m spacing
std::vector<float> gridVertices;
for (float i = -10.0f; i <= 10.0f; i += 1.0f) {
    // Line parallel to X axis
    gridVertices.push_back(-10.0f); gridVertices.push_back(0.0f); gridVertices.push_back(i);
    gridVertices.push_back(10.0f); gridVertices.push_back(0.0f); gridVertices.push_back(i);
    // Line parallel to Z axis
    gridVertices.push_back(i); gridVertices.push_back(0.0f); gridVertices.push_back(-10.0f);
    gridVertices.push_back(i); gridVertices.push_back(0.0f); gridVertices.push_back(10.0f);
}
// Draw with GL_LINES
```

### Robot Marker Geometry

```cpp
// Simple triangle pointing in heading direction
// In robot XY plane (OpenGL XZ plane)
std::vector<float> robotMarker = {
    // Front tip
    0.3f, 0.1f, 0.0f,
    // Back left
    -0.2f, 0.1f, 0.15f,
    // Back right
    -0.2f, 0.1f, -0.15f
};
// Transform by robot pose before rendering
```

### Performance Strategy

**Target: 1M+ points at 30fps**

1. **VBO Upload:** Use `GL_DYNAMIC_DRAW`, only upload new points
2. **Point Rendering:** Use `GL_POINTS` with vertex shader point size
3. **No Sorting:** Points don't need depth sorting (use depth test)
4. **Frustum Culling:** Not needed for MVP (GPU handles off-screen points)
5. **Level of Detail:** Optional decimation for >5M points

**Memory Budget:**
- 1M points × 24 bytes = 24 MB GPU
- 5M points × 24 bytes = 120 MB GPU
- Most GPUs handle this easily

### Window Management

**Two modes:**

1. **Standalone (`--pointcloud-replay`):**
   - Single GLFW window
   - PointCloudViewer owns the window and event loop
   - Simple main loop: read messages → add points → render

2. **Integrated (`--stream-replay --with-3d`):**
   - GLFW window for 3D (PointCloudViewer)
   - OpenCV windows for 2D streams (StreamReplayViewer)
   - Mixed event processing: `cv::waitKey()` + `glfwPollEvents()`
   - StreamReplayViewer owns main loop, calls PointCloudViewer methods

**Event Loop Integration:**
```cpp
// In StreamReplayViewer::run()
while (running) {
    // Process replay messages
    // ...

    // Render OpenCV windows
    updateAllDisplays();

    // Render OpenGL window
    #ifdef HAS_OPENGL_VIZ
    if (point_cloud_viewer_) {
        point_cloud_viewer_->render();
        point_cloud_viewer_->pollEvents();
    }
    #endif

    // Wait for input (OpenCV handles GLFW events too)
    int key = cv::waitKey(33);
    // ...
}
```

---

## Dev Notes

### Critical Architecture Constraints

**MUST USE:**
- GLFW3 for window management (consistent with modern OpenGL)
- GLAD for OpenGL function loading (or GLEW if preferred)
- GLM for math (glm::mat4, glm::vec3, etc.)
- Existing `PointCloud3D` struct from `ISensorSource.h`
- Existing `SensorReplayer` for reading recordings

**DO NOT:**
- Use Qt or other heavy GUI frameworks
- Modify existing replay infrastructure significantly
- Block OpenCV event loop
- Create per-frame GPU buffer allocations (pre-allocate and reuse)

### Existing Code Patterns to Follow

**From StreamReplayViewer (Story D-1):**
- Pre-allocated buffers to avoid per-frame allocation
- Window layout positioning
- Key event handling pattern
- Message processing switch statement

**From SlamVisualizer (Story 3-1):**
- OpenCV window creation and management
- Stats overlay rendering
- Mouse callback pattern
- Update/render separation

### Recording Format for POINT_CLOUD_3D

**From SensorRecorder.cpp (line 258-280):**
```cpp
// Format: {count, x[], y[], z[], intensity[]}
// msgpack map with string keys
pk.pack_map(5);
pk.pack("count");
pk.pack(cloud.size());
pk.pack("x");
pk.pack(cloud.x);
pk.pack("y");
pk.pack(cloud.y);
pk.pack("z");
pk.pack(cloud.z);
pk.pack("intensity");
pk.pack(cloud.intensity);
```

**Typical point counts per message:**
- Livox Mid-360: ~20,000-30,000 points per scan (10Hz)
- 60 second recording: ~18M points total
- Need efficient handling for demo video length recordings

### OpenGL Version Requirements

**Minimum: OpenGL 3.3 Core**
- Widely supported (2010+ hardware)
- Supports VBOs, VAOs, GLSL shaders
- Works on Linux, Mac, Windows

**Shader version:** `#version 330 core`

### GLAD Generation (if not using package)

Generate from https://glad.dav1d.de/:
- Language: C/C++
- Specification: OpenGL
- API: gl 3.3
- Profile: Core
- Extensions: None needed
- Options: Generate a loader

Place in `external/glad/`:
- `include/glad/glad.h`
- `include/KHR/khrplatform.h`
- `src/glad.c`

---

## Dependencies on Previous Stories

**Story D-1 (Multi-Stream Replay):**
- `StreamReplayViewer` class to extend
- `SensorReplayer` decode pattern
- Window management patterns
- Message processing loop structure

**Story 2-1 (Recording):**
- `SensorRecorder::recordPointCloud3D()` writes the data format we read
- `PointCloud3D` struct definition

**Story 2-2 (Replay System):**
- `SensorReplayer` for reading compressed recordings
- `ReplayMessage` struct with payload

**Story 1-4 (Hardware Integration):**
- `PointCloud3D` struct in `ISensorSource.h`
- LiDAR data types

---

## Verification Commands

```bash
# Install dependencies
sudo apt update
sudo apt install libglfw3-dev libglm-dev

# Setup GLAD (if not packaged)
mkdir -p external/glad
# Download from glad.dav1d.de and extract to external/glad/

# Build
cd build && cmake .. && make -j

# Run unit tests
./test_point_cloud_viewer

# Standalone 3D replay
./g1_inspector --pointcloud-replay 071712

# Expected:
# - OpenGL window opens "3D Point Cloud"
# - Points accumulate as recording plays
# - Drag to orbit camera around point cloud
# - Scroll to zoom in/out
# - Middle-drag to pan
# - 'r' resets camera view
# - Stats show: "Points: 1,234,567 | FPS: 60"
# - Colors: blue floor, red ceiling

# Integrated with multi-stream
./g1_inspector --stream-replay 071712 --with-3d

# Expected:
# - 5 windows: RGB, Webcam, Depth, SLAM, Status + 3D Point Cloud
# - All synchronized playback
# - 3D view shows accumulated point cloud
```

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **3D OpenGL window** | `--pointcloud-replay` opens 3D viewer |
| **Orbit camera** | Drag rotates, scroll zooms, middle-drag pans |
| **Height coloring** | Blue floor, green middle, red ceiling |
| **Accumulated cloud** | Points build up as replay progresses |
| **Robot marker** | Triangle shows current robot position/heading |
| **Grid floor** | Reference grid at Z=0 |
| **Integration** | `--with-3d` adds 3D window to multi-stream replay |
| **Performance** | 1M+ points renders at 30fps |
| **Stats** | Point count and FPS displayed |

---

## References

- [StreamReplayViewer](src/replay/StreamReplayViewer.h) - Integration point for --with-3d
- [SensorReplayer](src/replay/SensorReplayer.h) - Add decodePointCloud3D()
- [SensorRecorder](src/recording/SensorRecorder.cpp:258-280) - POINT_CLOUD_3D encoding format
- [RecordingTypes](src/recording/RecordingTypes.h:22) - MessageType::POINT_CLOUD_3D
- [ISensorSource](src/sensors/ISensorSource.h:14-24) - PointCloud3D struct definition
- [Epics](docs/epics.md#story-d-2-3d-point-cloud-visualization) - Original story definition
- [Story D-1](docs/sprint-artifacts/D-1-multi-stream-synchronized-replay-viewer.md) - Prerequisite story
- [GLFW Documentation](https://www.glfw.org/docs/latest/) - Window/input API
- [LearnOpenGL](https://learnopengl.com/) - OpenGL tutorial reference
- [GLM Documentation](https://glm.g-truc.net/0.9.9/) - Math library API

---

## Dev Agent Record

### Context Reference
This story file serves as the complete implementation context.

### Agent Model Used
{{agent_model_name_version}}

### Debug Log References
(To be filled during implementation)

### Completion Notes List
- Story created by create-story workflow for Epic D (Demo Video Deliverables)
- Requires new dependencies: GLFW3, GLAD, GLM
- Can run standalone or integrated with StreamReplayViewer (Story D-1)
- Uses existing PointCloud3D type and recording format
- Performance critical: must handle 1M+ points at 30fps

### File List
**New Files:**
- `src/visualization/PointCloudViewer.h` / `.cpp`
- `src/visualization/Camera.h` / `.cpp`
- `src/visualization/Shader.h` / `.cpp`
- `shaders/point_cloud.vert` / `.frag`
- `shaders/grid.vert` / `.frag`
- `test/test_point_cloud_viewer.cpp`

**Modified Files:**
- `CMakeLists.txt` - Add visualization library and dependencies
- `src/replay/SensorReplayer.h` / `.cpp` - Add decodePointCloud3D()
- `src/replay/StreamReplayViewer.h` / `.cpp` - Add --with-3d support
- `src/main.cpp` - Add --pointcloud-replay and --with-3d flags

---

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-12-07 | Story created with comprehensive implementation context | Create-Story Workflow |
