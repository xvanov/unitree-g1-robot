# Story 2.1: Teleop and Sensor Recording

**Status:** Done

---

## Quick Reference

**New files to create:**
- `src/teleop/TeleopController.h` / `.cpp` - Gamepad input parsing and velocity mapping
- `src/teleop/KeyboardTeleop.h` / `.cpp` - Keyboard fallback with camera view (OpenCV window)
- `src/recording/SensorRecorder.h` / `.cpp` - High-performance binary recording
- `src/recording/RecordingTypes.h` - Message types and timestamps
- `test/test_teleop.cpp` - Unit tests for gamepad parsing and velocity mapping
- `test/test_recording.cpp` - Unit tests for recording and file format

**Files to modify:**
- `CMakeLists.txt` - Add `teleop` and `recording` libraries, link msgpack/zstd
- `src/main.cpp` - Add `--teleop` and `--record` CLI options
- `src/sensors/SensorManager.h/.cpp` - Add raw `wireless_remote` access for gamepad data

**Dependencies to add:**
- `msgpack-c` (header-only) - Binary serialization
- `zstd` - Compression (likely already available via system)

**Integration Points:**
| Component | Uses | Provides |
|-----------|------|----------|
| `TeleopController` | `SensorManager.getRawWirelessRemote()` | `TeleopCommand` struct |
| `SensorRecorder` | SensorManager callbacks (setLidarCallback, setImuCallback) | Binary file + metadata.json |
| `KeyboardTeleop` | cv::VideoCapture, LocoController, SensorRecorder | Visual feedback window |

**Key classes:**
| Class | Purpose |
|-------|---------|
| `TeleopController` | Parse gamepad from `wireless_remote[40]`, map to velocity commands |
| `KeyboardTeleop` | OpenCV window with camera feed, WASD control, visual feedback |
| `SensorRecorder` | Stream LiDAR, IMU, pose, images to compressed binary file |
| `RecordingSession` | Manages file handles, timestamps, session metadata |

**Key function signatures:**
```cpp
// TeleopController - gamepad input
void TeleopController::update(const uint8_t wireless_remote[40]);
TeleopCommand TeleopController::getCommand() const;  // {vx, vy, omega, buttons}

// KeyboardTeleop - visual teleop with camera
void KeyboardTeleop::run();  // Blocking main loop
void KeyboardTeleop::setRecorder(SensorRecorder* recorder);

// SensorRecorder - high-performance recording
bool SensorRecorder::startRecording(const std::string& session_id);
void SensorRecorder::recordLidarScan(const LidarScan& scan);
void SensorRecorder::recordImu(const ImuData& imu);
void SensorRecorder::recordPose(const Pose2D& pose);
void SensorRecorder::recordImage(const cv::Mat& image, const ImageMetadata& meta);
void SensorRecorder::stopRecording();
RecordingStats SensorRecorder::getStats() const;
```

**Primary acceptance criteria:** AC1 (gamepad teleop works), AC2 (keyboard teleop with camera), AC3 (sensor recording to binary), AC4 (real-time performance)

**Prerequisites:** Story 1-4 (Hardware Integration) for SensorManager and LocoController

**Quick Start Implementation Order:**
1. Add dependencies (Task 1)
2. Implement TeleopController parsing (Task 2)
3. Implement SensorRecorder core (Task 5)
4. Wire up in main.cpp (Task 7)
5. Add unit tests (Task 9)

---

## Story

As a **developer**,
I want **to teleoperate the robot while recording all sensor data**,
So that **I can capture real-world data for E2E testing without synthetic noise**.

---

## Acceptance Criteria

1. **AC1:** Gamepad teleop works - left stick forward/back, right stick rotation, buttons for stand/sit
2. **AC2:** Keyboard teleop with camera view - WASD controls, live camera feed in OpenCV window
3. **AC3:** Sensor data recorded to compressed binary format (MessagePack + zstd)
4. **AC4:** Recording maintains real-time performance (<5% CPU overhead during recording)
5. **AC5:** Recording includes: LiDAR scans, IMU data, pose estimates, camera images, timestamps
6. **AC6:** Session metadata saved: start time, duration, robot ID, plan loaded (if any)
7. **AC7:** Recording file size reasonable: <50 MB/minute for full sensor suite
8. **AC8:** CLI commands: `--teleop gamepad`, `--teleop keyboard`, `--record <session_id>`
9. **AC9:** Visual feedback during recording (frame counter, disk usage, duration)
10. **AC10:** Graceful stop on Ctrl+C or 'q' key - flushes all buffers, writes metadata

---

## Technical Design

### Data Flow Architecture

```
┌─────────────────┐     callbacks      ┌─────────────────┐
│  SensorManager  │ ─────────────────► │  SensorRecorder │
│  (LiDAR, IMU)   │                    │                 │
└─────────────────┘                    │  ┌───────────┐  │
                                       │  │Ring Buffer│  │
┌─────────────────┐     push data      │  │  (10MB)   │  │
│ TeleopController│ ─────────────────► │  └─────┬─────┘  │
│   (Pose est.)   │                    │        │        │
└─────────────────┘                    │        ▼        │
                                       │  ┌───────────┐  │
┌─────────────────┐     captureFrame   │  │Writer Thd │  │
│  cv::VideoCapture│ ─────────────────►│  │ (async)   │  │
│   (Camera)      │                    │  └─────┬─────┘  │
└─────────────────┘                    └────────┼────────┘
                                                │
                                                ▼
                                       ┌─────────────────┐
                                       │sensors.bin.zst  │
                                       │metadata.json    │
                                       │images/*.jpg     │
                                       └─────────────────┘
```

### File Format: `sensors.bin.zst`

```
[zstd compressed stream]
├── Message 1: [timestamp_us: i64, type: u8, payload: msgpack]
├── Message 2: [timestamp_us: i64, type: u8, payload: msgpack]
├── ...
└── Message N: [timestamp_us: i64, type: u8, payload: msgpack]
```

**Message Payloads:**

```cpp
// LIDAR_SCAN (type=1)
{
    "ranges": [float...],      // 360 range values
    "angle_min": float,
    "angle_max": float,
    "angle_increment": float
}

// IMU_DATA (type=2)
{
    "quat": [w, x, y, z],      // Orientation quaternion
    "gyro": [x, y, z],         // Angular velocity rad/s
    "accel": [x, y, z],        // Linear acceleration m/s^2
    "rpy": [r, p, y]           // Roll, pitch, yaw radians
}

// POSE (type=3)
{
    "x": float,
    "y": float,
    "theta": float
}

// IMAGE (type=4) - Reference only, actual image in images/ folder
{
    "sequence": int,
    "filename": "img_00000001.jpg"
}
```

### Recording Performance Target

| Data | Rate | Raw Size | Compressed |
|------|------|----------|------------|
| LiDAR | 10 Hz | 1.4 KB/frame | ~500 B/frame |
| IMU | 100 Hz | 100 B/frame | ~40 B/frame |
| Pose | 50 Hz | 12 B/frame | ~5 B/frame |
| Images | 1 Hz | 50 KB/frame | ~50 KB (JPEG) |

**Total:** ~5 MB/min sensors + ~3 MB/min images = **~8 MB/min** (well under 50 MB target)

### Gamepad Button/Axis Reference

See `external/unitree_sdk2/example/state_machine/gamepad.hpp` for complete struct definitions.

**Key fields from `xRockerBtnDataStruct` (40 bytes):**
```cpp
float lx;  // Left stick X (-1.0 to 1.0)
float ly;  // Left stick Y (-1.0 to 1.0)
float rx;  // Right stick X (-1.0 to 1.0)
float ry;  // Right stick Y (-1.0 to 1.0)
// Button bits in btn.components: A, B, X, Y, start, select, L1, R1, etc.
```

**Velocity Mapping:**
- `ly` (left stick Y) → `vx` forward/back: scale by MAX_VX (0.5 m/s)
- `lx` (left stick X) → `vy` strafe: scale by MAX_VY (0.3 m/s)
- `rx` (right stick X) → `omega` rotation: scale by MAX_OMEGA (0.5 rad/s)

### Keyboard Mapping

```
W/S     → Forward/Backward (vx)
A/D     → Strafe Left/Right (vy)
Q/E     → Rotate Left/Right (omega)
SPACE   → Emergency stop (zero velocity)
R       → Toggle recording
+/-     → Adjust max speed
ESC/q   → Quit
```

### Camera View Overlay

```
┌─────────────────────────────────────────┐
│  [RECORDING] 00:05:23  ●  12.3 MB       │
│                                          │
│                                          │
│           [Camera Feed]                  │
│                                          │
│                                          │
│  vx: 0.30   vy: 0.00   ω: 0.10          │
│  Battery: 87%    FSM: WALKING           │
│  Press Q to quit, R to record           │
└─────────────────────────────────────────┘
```

---

## Tasks / Subtasks

- [x] **Task 1: Add msgpack-c and zstd dependencies** (AC: 3, 4, 7)
  - [x] 1.1 Update `CMakeLists.txt` to find/link msgpack and zstd
    - msgpack-c is header-only, can vendor or use system package
    - zstd: `find_package(zstd)` or link `-lzstd`
  - [x] 1.2 Add to Dockerfile if needed: `apt-get install libmsgpack-dev libzstd-dev`
  - [x] 1.3 Verify compilation with basic msgpack encode/decode test

- [x] **Task 2: Implement TeleopController (Gamepad)** (AC: 1, 8)
  - [x] 2.1 Create `src/teleop/TeleopController.h`
    - Use `unitree::common::Gamepad` class from SDK (don't reimplement parsing)
    - `TeleopCommand` struct: `{float vx, vy, omega; bool stand, sit, emergency_stop}`
    - `update(const uint8_t wireless_remote[40])` - parse raw data via SDK Gamepad class
    - `getCommand()` - return current command after deadzone/smoothing
  - [x] 2.2 Create `src/teleop/TeleopController.cpp`
    - Map left stick Y (`ly`) to forward velocity (`vx`)
    - Map left stick X (`lx`) to lateral velocity (`vy`)
    - Map right stick X (`rx`) to rotation (`omega`)
    - A button = stand up, B button = sit down, Start = emergency stop
    - Apply safety limits using constants from LocoController (see SafetyLimits section below)
  - [x] 2.3 Add to SensorManager: `getRawWirelessRemote()` method to expose raw 40-byte gamepad data

- [x] **Task 3: Implement KeyboardTeleop (with Camera View)** (AC: 2, 8)
  - [x] 3.1 Create `src/teleop/KeyboardTeleop.h`
    - Constructor takes `SensorManager*`, `LocoController*`, optional `SensorRecorder*`
    - `run()` - blocking main loop with OpenCV window
    - `stop()` - signal to exit loop
  - [x] 3.2 Create `src/teleop/KeyboardTeleop.cpp`
    - Create OpenCV window "Teleop - Press Q to quit"
    - Display live camera feed (from SensorManager or test pattern if no camera)
    - Overlay: velocity commands, recording status, battery level
    - Key detection: Use `cv::pollKey()` if OpenCV 4.5+, otherwise `cv::waitKey(1)` in non-blocking mode
    - 30 FPS update loop

- [x] **Task 4: Define Recording Types** (AC: 3, 5)
  - [x] 4.1 Create `src/recording/RecordingTypes.h`
    ```cpp
    enum class MessageType : uint8_t {
        LIDAR_SCAN = 1,
        IMU_DATA = 2,
        POSE = 3,
        IMAGE = 4,
        MOTOR_STATE = 5,
        METADATA = 255
    };

    struct RecordedMessage {
        int64_t timestamp_us;  // Microseconds since epoch
        MessageType type;
        // Payload follows (msgpack encoded)
    };

    struct RecordingMetadata {
        std::string session_id;
        int64_t start_time_us;
        int64_t end_time_us;
        std::string robot_id;
        std::string plan_path;  // Empty if no plan loaded
        uint32_t lidar_count;
        uint32_t imu_count;
        uint32_t pose_count;
        uint32_t image_count;
        uint64_t total_bytes;
    };

    struct RecordingStats {
        uint32_t messages_recorded;
        uint64_t bytes_written;
        uint64_t bytes_compressed;
        float compression_ratio;
        float duration_seconds;
        float disk_rate_mbps;
    };
    ```

- [x] **Task 5: Implement SensorRecorder** (AC: 3, 4, 5, 6, 7, 9)
  - [x] 5.1 Create `src/recording/SensorRecorder.h`
    - `startRecording(session_id)` - open file, init compressor
    - `stopRecording()` - flush, write metadata, close
    - `recordXxx()` methods for each sensor type
    - `getStats()` - live statistics
    - `isRecording()` - status check
  - [x] 5.2 Create `src/recording/SensorRecorder.cpp`
    - File format: `data/recordings/<session_id>/sensors.bin.zst`
    - Use streaming zstd compression (`ZSTD_CStream`) with level 3 (fast) - see Dev Notes
    - MessagePack for each message: `[timestamp_us, type, payload]`
    - Separate thread for disk I/O to avoid blocking sensors (see Ring Buffer section)
    - Ring buffer (10MB) between sensor callbacks and writer thread
    - Flush strategy: every 100 messages OR 1MB OR 1 second (whichever first)
  - [x] 5.3 Image recording strategy:
    - Save images as separate JPEG files (not in binary stream)
    - Record image reference in binary: `{timestamp, sequence_num, filename}`
    - Reuse directory creation pattern from `ImageCapture::startCapture()` (Story 1-7)
    - Reuse async save pattern with `std::future` tracking from `ImageCapture` (Story 1-7)
  - [x] 5.4 Write metadata JSON alongside binary:
    - `data/recordings/<session_id>/metadata.json` - human readable session info
    - Written on `stopRecording()`

- [x] **Task 6: Wire Up Teleop Loop** (AC: 1, 2, 4, 10)
  - [x] 6.1 Create teleop main loop that:
    - Initializes SensorManager and LocoController
    - Reads gamepad OR keyboard input
    - Sends velocity commands at 50 Hz
    - Optionally records if `--record` flag set
    - Handles Ctrl+C gracefully (signal handler)
  - [x] 6.2 Register signal handler for SIGINT/SIGTERM
    - **IMPORTANT:** Check if main.cpp already has signal handling (from capture thread)
    - Use shared `g_shutdown_requested` atomic flag if it exists, otherwise create it
    - Set atomic flag to stop loops
    - Wait for recorder flush (max 2 seconds)
    - Clean shutdown of LocoController (stop motion)

- [x] **Task 7: CLI Integration** (AC: 8, 10)
  - [x] 7.1 Update `src/main.cpp`
    ```
    --teleop gamepad      Start gamepad teleop mode
    --teleop keyboard     Start keyboard teleop with camera view
    --record <session_id> Enable recording during teleop
    --plan <path>         Load plan for reference (optional)
    ```
  - [x] 7.2 Implement `runTeleopMode()` function
    - Parse teleop type (gamepad/keyboard)
    - Initialize hardware (SensorManager, LocoController)
    - Create SensorRecorder if --record specified
    - **Wire up SensorManager callbacks to SensorRecorder** (see Sensor Data Flow section)
    - Run appropriate teleop loop
    - Clean shutdown on exit

- [x] **Task 8: CMake Integration** (AC: all)
  - [x] 8.1 Update `CMakeLists.txt`
    ```cmake
    # Recording library
    add_library(recording
        src/recording/SensorRecorder.cpp
    )
    target_link_libraries(recording
        ${OpenCV_LIBS}
        zstd
        nlohmann_json::nlohmann_json
    )

    # Teleop library
    add_library(teleop
        src/teleop/TeleopController.cpp
        src/teleop/KeyboardTeleop.cpp
    )
    target_link_libraries(teleop
        sensors
        loco_hw
        recording
        ${OpenCV_LIBS}
    )
    ```

- [x] **Task 9: Unit Tests** (AC: 1, 3)
  - [x] 9.1 Create `test/test_teleop.cpp`
    - Test gamepad parsing with mock `wireless_remote` data
    - Test deadzone filtering
    - Test velocity mapping and safety limits
    - Test button state detection (pressed, on_press, on_release)
  - [x] 9.2 Create `test/test_recording.cpp`
    - Test recording start/stop
    - Test message serialization with msgpack
    - Test compression ratio (should be >3:1 for sensor data)
    - Test metadata generation
    - Test file format (can be read back)
    - **Test ring buffer overflow behavior** (high load scenario)

- [x] **Task 10: E2E Test** (AC: all)
  - [x] 10.1 Create `test/test_e2e_story_2_1.sh`
    - Test 1: Verify CLI options parse correctly
    - Test 2: Start/stop recording creates expected files
    - Test 3: Recording file can be opened and has valid header
    - Test 4: Metadata JSON is valid
    - Test 5: Simulated sensor data records at expected rate

---

## Dev Notes

### Critical Architecture Constraints

**DO NOT:**
- Block sensor callbacks during disk I/O (use separate writer thread)
- Record at higher rates than actual sensor rates
- Store images in the binary stream (too large, use separate files)
- Use JSON for high-frequency data (too slow)
- Assume gamepad is always connected (handle disconnection gracefully)
- Implement custom ring buffer - use established patterns (see below)

**MUST USE:**
- MessagePack for binary serialization (fast, compact)
- zstd streaming compression (best ratio + speed)
- Ring buffer between sensors and disk writer (see Ring Buffer section)
- SafetyLimits constants from LocoController (see below)
- ImageCapture patterns from Story 1-7: directory creation, async save with std::future, JSON metadata sidecar

### SafetyLimits Constants

Located in `src/locomotion/LocoController.h` (or use inline):
```cpp
// From LocoController - use these for velocity clamping
namespace SafetyLimits {
    constexpr float MAX_VX = 0.5f;      // m/s forward (conservative indoor)
    constexpr float MAX_VY = 0.3f;      // m/s lateral
    constexpr float MAX_OMEGA = 0.5f;   // rad/s rotation
}

// In TeleopController::getCommand():
command.vx = std::clamp(raw_vx * scale, -SafetyLimits::MAX_VX, SafetyLimits::MAX_VX);
command.vy = std::clamp(raw_vy * scale, -SafetyLimits::MAX_VY, SafetyLimits::MAX_VY);
command.omega = std::clamp(raw_omega * scale, -SafetyLimits::MAX_OMEGA, SafetyLimits::MAX_OMEGA);
```

### Sensor Data Flow - Callback Wiring

SensorRecorder receives data via callbacks from SensorManager. Set this up in main.cpp or teleop initialization:

```cpp
// In runTeleopMode() or main teleop setup:
SensorRecorder recorder;
recorder.startRecording(session_id);

// Wire up sensor callbacks to push data to recorder
sensor_manager.setLidarCallback([&recorder](const LidarScan& scan) {
    if (recorder.isRecording()) {
        recorder.recordLidarScan(scan);
    }
});

sensor_manager.setImuCallback([&recorder](const ImuData& imu) {
    if (recorder.isRecording()) {
        recorder.recordImu(imu);
    }
});

// Pose is recorded from teleop loop at 50Hz
// Images are recorded via captureFrame() similar to ImageCapture pattern
```

### Ring Buffer Implementation

Use a simple mutex-protected circular buffer (acceptable for 10-100Hz sensors):

```cpp
#include <mutex>
#include <condition_variable>
#include <vector>

class RingBuffer {
public:
    explicit RingBuffer(size_t capacity_bytes = 10 * 1024 * 1024)
        : capacity_(capacity_bytes), buffer_(capacity_bytes) {}

    // Returns false if buffer is full (overflow)
    bool push(const uint8_t* data, size_t size) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (used_ + size > capacity_) {
            overflow_count_++;
            return false;  // Buffer full - drop data
        }
        // Copy data to buffer (wrap-around logic)
        // ... implementation details
        used_ += size;
        cv_.notify_one();
        return true;
    }

    // Blocking pop - returns bytes read
    size_t pop(uint8_t* out, size_t max_size) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return used_ > 0 || shutdown_; });
        // ... copy and return
    }

    size_t getOverflowCount() const { return overflow_count_; }
    float getUsagePercent() const { return 100.0f * used_ / capacity_; }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<uint8_t> buffer_;
    size_t capacity_;
    size_t used_ = 0;
    size_t write_pos_ = 0;
    size_t read_pos_ = 0;
    std::atomic<size_t> overflow_count_{0};
    std::atomic<bool> shutdown_{false};
};
```

**Overflow Handling Strategy:**
- If ring buffer reaches 90% capacity, log warning
- If ring buffer is full, **drop oldest unwritten frame** and increment overflow counter
- Log overflow count in final metadata.json
- Target: writer thread should keep up with ~5KB/s peak sensor rate

### Signal Handler Integration

**IMPORTANT:** Check if main.cpp already has signal handling. If `g_shutdown_requested` exists, reuse it:

```cpp
// Check for existing signal handler in main.cpp
// If it exists, just add teleop cleanup to the existing shutdown path

// If creating new handler:
#include <csignal>
#include <atomic>

// Use extern if defined elsewhere, or define here
extern std::atomic<bool> g_shutdown_requested;  // If exists in main.cpp
// OR
std::atomic<bool> g_shutdown_requested{false};  // If new

void signalHandler(int signum) {
    (void)signum;
    g_shutdown_requested = true;
}

// In runTeleopMode():
std::signal(SIGINT, signalHandler);
std::signal(SIGTERM, signalHandler);

while (!g_shutdown_requested) {
    // ... teleop logic
}

// Cleanup
if (recorder.isRecording()) {
    std::cout << "[TELEOP] Flushing recording..." << std::endl;
    recorder.stopRecording();  // Waits for pending writes (max 2s)
}
loco_controller.stop();
```

### zstd Compression Levels

```cpp
// Level 3 (default) - best speed/ratio tradeoff for real-time
ZSTD_initCStream(cstream, 3);

// Level 1 - fastest, slightly worse ratio (use if CPU-constrained)
// Level 7+ - better ratio but slower (use for post-processing, not real-time)
```

### msgpack-c Usage

```cpp
#include <msgpack.hpp>

// Packing
msgpack::sbuffer buffer;
msgpack::packer<msgpack::sbuffer> pk(&buffer);
pk.pack_map(3);
pk.pack("x"); pk.pack(pose.x);
pk.pack("y"); pk.pack(pose.y);
pk.pack("theta"); pk.pack(pose.theta);

// Unpacking
msgpack::object_handle oh = msgpack::unpack(data, size);
msgpack::object obj = oh.get();
auto map = obj.as<std::map<std::string, float>>();
```

### zstd Streaming Usage

```cpp
#include <zstd.h>

// Create compression stream
ZSTD_CStream* cstream = ZSTD_createCStream();
ZSTD_initCStream(cstream, 3);  // Compression level 3 (fast)

// Compress chunk
ZSTD_inBuffer input = {data, size, 0};
ZSTD_outBuffer output = {out_buf, out_capacity, 0};
ZSTD_compressStream(cstream, &output, &input);

// Flush and end
ZSTD_endStream(cstream, &output);
ZSTD_freeCStream(cstream);
```

### ImageCapture Patterns to Reuse (Story 1-7)

From `src/capture/ImageCapture.cpp`, reuse these patterns:

1. **Directory creation:**
```cpp
std::filesystem::create_directories(session_dir_ + "/images");
```

2. **Async save with future tracking:**
```cpp
auto future = std::async(std::launch::async, [this, frame, pose, seq]() {
    // Save JPEG, write metadata
});
pending_saves_.push_back(std::move(future));
```

3. **Cleanup pending saves:**
```cpp
void cleanupPendingSaves() {
    pending_saves_.erase(
        std::remove_if(pending_saves_.begin(), pending_saves_.end(),
            [](std::future<void>& f) {
                return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
            }),
        pending_saves_.end());
}
```

4. **Wait for pending on shutdown:**
```cpp
void stopRecording() {
    for (auto& future : pending_saves_) {
        if (future.valid()) future.wait();
    }
}
```

---

## Directory Structure

```
src/teleop/
├── TeleopController.h     # Gamepad parsing
├── TeleopController.cpp
├── KeyboardTeleop.h       # Keyboard + camera view
└── KeyboardTeleop.cpp

src/recording/
├── RecordingTypes.h       # Message types, metadata structs
├── SensorRecorder.h       # Recording interface
└── SensorRecorder.cpp     # Implementation with msgpack/zstd

data/recordings/           # Created at runtime
└── <session_id>/
    ├── sensors.bin.zst    # Compressed sensor stream
    ├── metadata.json      # Session metadata
    └── images/            # Camera frames (JPEG)
        ├── img_00000001.jpg
        └── ...
```

---

## Verification Commands

```bash
# Build
cd build && cmake .. && make -j

# Run unit tests
./test_teleop
./test_recording

# Run E2E test
../test/test_e2e_story_2_1.sh

# Manual testing (requires robot or simulation mode):

# Gamepad teleop without recording
./g1_inspector --teleop gamepad

# Keyboard teleop with camera view
./g1_inspector --teleop keyboard

# Teleop with recording
./g1_inspector --teleop keyboard --record my_session_001

# Check recording output
ls -la ../data/recordings/my_session_001/
cat ../data/recordings/my_session_001/metadata.json
```

---

## Dependencies on Previous Stories

**Story 1-4 (Hardware Integration):**
- `SensorManager` for sensor data access and callbacks
- `LocoController` for velocity commands and SafetyLimits constants
- Need to add `getRawWirelessRemote()` method to SensorManager

**Story 1-7 (Visual Capture):**
- Reuse `ImageCapture` patterns: directory creation, async save with std::future, JSON sidecar
- Reuse session directory structure (`data/recordings/<session_id>/`)

---

## Story Deliverable

| What You Get | How to Verify |
|--------------|---------------|
| **TeleopController** | Gamepad controls robot movement |
| **KeyboardTeleop** | OpenCV window with camera and WASD control |
| **SensorRecorder** | Binary recording with <50 MB/min |
| **CLI integration** | `--teleop` and `--record` commands work |
| **Unit tests** | test_teleop and test_recording pass |

---

## References

- [Unitree SDK gamepad.hpp](external/unitree_sdk2/example/state_machine/gamepad.hpp) - Gamepad parsing reference (use `unitree::common::Gamepad` class)
- [msgpack-c](https://github.com/msgpack/msgpack-c) - Serialization library
- [zstd](https://github.com/facebook/zstd) - Compression library
- [Story 1-4](docs/sprint-artifacts/1-4-hardware-integration.md) - SensorManager, LocoController, SafetyLimits
- [Story 1-7](docs/sprint-artifacts/1-7-visual-capture.md) - ImageCapture patterns (async save, directory creation)

---

## Dev Agent Record

### Context Reference
This story file serves as the complete implementation context.

### File List

**New Files Created:**
- `src/teleop/TeleopController.h` - Gamepad input parsing with deadzone/smoothing
- `src/teleop/TeleopController.cpp` - TeleopController implementation
- `src/teleop/KeyboardTeleop.h` - Keyboard teleop with OpenCV camera view
- `src/teleop/KeyboardTeleop.cpp` - KeyboardTeleop implementation
- `src/teleop/TeleopRunner.h` - Teleop session manager
- `src/teleop/TeleopRunner.cpp` - TeleopRunner implementation
- `src/recording/RecordingTypes.h` - Message types, metadata structs, timestamps
- `src/recording/SensorRecorder.h` - High-performance recording interface
- `src/recording/SensorRecorder.cpp` - Recording with msgpack/zstd, ring buffer
- `test/test_teleop.cpp` - Unit tests for gamepad parsing, velocity mapping
- `test/test_recording.cpp` - Unit tests for recording, compression, metadata
- `test/test_e2e_story_2_1.sh` - E2E test script for CLI and file structure

**Files Modified:**
- `CMakeLists.txt` - Added msgpack/zstd deps, recording and teleop libraries, tests
- `docker/Dockerfile` - Added libmsgpack-dev and libzstd-dev packages
- `src/sensors/SensorManager.h` - Added getRawWirelessRemote() method
- `src/sensors/SensorManager.cpp` - Implemented wireless_remote data capture
- `src/main.cpp` - Added --teleop, --record, --plan CLI options

### Completion Notes List
- Story created by PM agent (John) based on user requirements for real-world E2E testing
- Focuses on capturing real sensor data vs synthetic noise
- Part of 3-story arc: 2-1 (recording), 2-2 (replay), 2-3 (E2E test)
- Implementation uses SafetyLimits constants from LocoController for velocity clamping
- Ring buffer (10MB) with async writer thread prevents blocking sensor callbacks
- zstd streaming compression at level 3 for real-time performance
- Keyboard rotation changed from Q/E to Z/E (Q used for quit)
- Reused async image save pattern from Story 1-7 ImageCapture

### Change Log

- 2025-12-06: Story 2-1 created by create-story workflow
- 2025-12-06: Validation review applied (validate-create-story):
  - **C1 FIXED:** Added SafetyLimits section with exact location and usage code
  - **C2 FIXED:** Added Ring Buffer Implementation section with complete class example and overflow strategy
  - **C3 FIXED:** Added Signal Handler Integration section with guidance to check for existing handlers
  - **C4 FIXED:** Added Sensor Data Flow section showing callback wiring pattern
  - **C5 FIXED:** Added overflow handling strategy (drop oldest, log count, warn at 90%)
  - **E1 FIXED:** Removed duplicate gamepad mapping, now references gamepad.hpp
  - **E2 FIXED:** Added explicit SensorManager callback integration example
  - **E3 FIXED:** Added ring buffer overflow test to Task 9.2
  - **E4 FIXED:** Listed specific ImageCapture patterns to reuse (4 patterns with code)
  - **O1 FIXED:** Added zstd compression level notes (level 3 default, 1 for speed, 7+ for ratio)
  - **O2 FIXED:** Added batch flush strategy (100 messages OR 1MB OR 1s)
  - **O3 FIXED:** Added cv::pollKey() note for OpenCV 4.5+
  - **LLM-OPT:** Added data flow diagram, Integration Points table, Quick Start order
  - **LLM-OPT:** Moved Technical Design before Tasks for better implementation flow
  - Fixed story arc references: 2-1, 2-2, 2-3 (was using old 1-10, 1-11, 1-12 numbering)
  - Validation report: docs/sprint-artifacts/validation-report-2-1-2025-12-06.md
- 2025-12-06: Implementation completed (dev-story workflow):
  - All 10 tasks implemented and verified
  - Created TeleopController with gamepad parsing, deadzone filtering, velocity mapping
  - Created KeyboardTeleop with OpenCV camera view, WASD+Z/E controls
  - Created TeleopRunner for session management and signal handling
  - Created SensorRecorder with ring buffer, msgpack serialization, zstd compression
  - Added recording and teleop libraries to CMakeLists.txt
  - Added --teleop, --record, --plan CLI options to main.cpp
  - Unit tests: test_teleop.cpp (14 tests), test_recording.cpp (12 tests)
  - E2E test: test_e2e_story_2_1.sh validates CLI, file structure, compression
  - Status changed to: Ready for Review
- 2025-12-06: Code Review fixes applied (code-review workflow):
  - **HIGH-1 FIXED:** Added G1Model namespace with EDU_23_DOF/STANDARD_29_DOF/FULL_36_DOF variants
    - LocoController.init() now accepts robot_dof parameter (defaults to 23-DOF EDU)
    - Arm gestures (waveHand, shakeHand) now check hasArmControl() and fail gracefully on EDU
  - **HIGH-2 FIXED:** Replaced custom gamepad parsing with official SDK Gamepad class
    - Now uses `unitree::common::Gamepad` from external/unitree_sdk2/example/g1/low_level/gamepad.hpp
    - Ensures compatibility with Unitree's gamepad data format and firmware versions
  - **HIGH-3 FIXED:** Added RecordingCpuOverhead test to validate AC4 (<5% CPU overhead)
    - Measures baseline vs recording time, verifies no buffer overflow during normal operation
  - **HIGH-4 FIXED:** IMU quaternion now computed from RPY using ZYX Euler convention
    - No longer uses placeholder identity quaternion
  - **MEDIUM-1 FIXED:** TeleopRunner.cpp uses weak symbol for g_running fallback
    - Allows standalone builds (unit tests) without main.cpp linkage
  - **MEDIUM-2 FIXED:** KeyboardTeleop now tries alternate camera indices (0,1,2) if default fails
    - Better support for G1 robot camera configurations
  - **MEDIUM-3 FIXED:** Added comment in overlay code explaining battery shows 0 on G1
    - Battery requires separate BMS topic subscription (not implemented in MVP)
  - **MEDIUM-4 FIXED:** Pose recording now uses SensorManager::getEstimatedPose()
    - Returns IMU yaw-based orientation (x,y remain 0 until SLAM integration)
  - **MEDIUM-5 FIXED:** Added design note explaining mutex choice for ring buffer
    - Mutex overhead negligible at 10-100Hz; async writer prevents blocking sensor callbacks
  - Updated test_teleop.cpp to use SDK REMOTE_DATA_RX union for mock data
  - Status changed to: Done
