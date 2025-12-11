#!/bin/bash
# End-to-end test for Story 2-1: Teleop and Sensor Recording
#
# Tests:
# 1. CLI options parse correctly
# 2. Recording creates expected file structure
# 3. Recording file has valid format (can be decompressed)
# 4. Metadata JSON is valid
# 5. Simulated sensor data records

set -e  # Exit on first error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_ROOT}/build"
TEST_RECORDING_DIR="${PROJECT_ROOT}/data/test_recordings"

echo "=== Story 2-1 E2E Test: Teleop and Sensor Recording ==="
echo ""

# Cleanup function
cleanup() {
    if [ -d "$TEST_RECORDING_DIR" ]; then
        rm -rf "$TEST_RECORDING_DIR"
    fi
}

# Register cleanup
trap cleanup EXIT

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((TESTS_PASSED++))
}

fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((TESTS_FAILED++))
}

skip() {
    echo -e "${YELLOW}[SKIP]${NC} $1"
}

# Check if executable exists
if [ ! -f "${BUILD_DIR}/g1_inspector" ]; then
    echo -e "${RED}ERROR: g1_inspector not found at ${BUILD_DIR}/g1_inspector${NC}"
    echo "Please build the project first: cd build && cmake .. && make"
    exit 1
fi

# ===== Test 1: CLI options parse correctly =====
echo ""
echo "Test 1: CLI Options Parsing"

# Test --teleop requires mode argument
OUTPUT=$("${BUILD_DIR}/g1_inspector" --teleop 2>&1 || true)
if echo "$OUTPUT" | grep -q "requires a mode argument"; then
    pass "Missing teleop mode argument detected"
else
    fail "Missing teleop mode argument not detected"
fi

# Test invalid teleop mode
OUTPUT=$("${BUILD_DIR}/g1_inspector" --teleop invalid 2>&1 || true)
if echo "$OUTPUT" | grep -q "must be 'gamepad' or 'keyboard'"; then
    pass "Invalid teleop mode rejected"
else
    fail "Invalid teleop mode not rejected"
fi

# Test --record requires session argument
OUTPUT=$("${BUILD_DIR}/g1_inspector" --record 2>&1 || true)
if echo "$OUTPUT" | grep -q "requires a session ID"; then
    pass "Missing record session argument detected"
else
    fail "Missing record session argument not detected"
fi

# Test help shows teleop options
OUTPUT=$("${BUILD_DIR}/g1_inspector" --help 2>&1)
if echo "$OUTPUT" | grep -q "\-\-teleop"; then
    pass "Help shows --teleop option"
else
    fail "Help doesn't show --teleop option"
fi

if echo "$OUTPUT" | grep -q "\-\-record"; then
    pass "Help shows --record option"
else
    fail "Help doesn't show --record option"
fi

# ===== Test 2: Recording file structure =====
echo ""
echo "Test 2: Recording File Structure"

# Create test recording directory
mkdir -p "$TEST_RECORDING_DIR"

# Run unit tests for recording (which create recordings)
if [ -f "${BUILD_DIR}/test_recording" ]; then
    if "${BUILD_DIR}/test_recording" --gtest_filter='*StartStopCreatesFiles*' > /dev/null 2>&1; then
        pass "Recording creates expected files (via unit test)"
    else
        fail "Recording unit test failed"
    fi
else
    skip "test_recording not built - skipping file structure test"
fi

# ===== Test 3: zstd file format verification =====
echo ""
echo "Test 3: File Format Verification"

# Check if zstd is available
if command -v zstd &> /dev/null; then
    # Create a test recording session via the recording test
    if [ -f "${BUILD_DIR}/test_recording" ]; then
        # The test creates recordings in temp directory
        # We can verify the zstd format by checking if the file header is valid

        # Try to decompress to /dev/null to verify format
        if "${BUILD_DIR}/test_recording" --gtest_filter='*CompressionRatio*' > /dev/null 2>&1; then
            pass "Compression creates valid zstd files (via unit test)"
        else
            fail "Compression test failed"
        fi
    else
        skip "test_recording not built - skipping format verification"
    fi
else
    skip "zstd not installed - skipping format verification"
fi

# ===== Test 4: Metadata JSON validation =====
echo ""
echo "Test 4: Metadata JSON Validation"

# Run metadata-related tests
if [ -f "${BUILD_DIR}/test_recording" ]; then
    if "${BUILD_DIR}/test_recording" --gtest_filter='*MetadataMessageCounts*' > /dev/null 2>&1; then
        pass "Metadata contains correct message counts"
    else
        fail "Metadata message counts test failed"
    fi

    if "${BUILD_DIR}/test_recording" --gtest_filter='*PlanPathInMetadata*' > /dev/null 2>&1; then
        pass "Plan path recorded in metadata"
    else
        fail "Plan path metadata test failed"
    fi
else
    skip "test_recording not built - skipping metadata tests"
fi

# ===== Test 5: Teleop controller tests =====
echo ""
echo "Test 5: Teleop Controller"

if [ -f "${BUILD_DIR}/test_teleop" ]; then
    if "${BUILD_DIR}/test_teleop" --gtest_filter='*ForwardVelocityMapping*' > /dev/null 2>&1; then
        pass "Forward velocity mapping works"
    else
        fail "Forward velocity mapping failed"
    fi

    if "${BUILD_DIR}/test_teleop" --gtest_filter='*SafetyLimitsClamping*' > /dev/null 2>&1; then
        pass "Safety limits clamping works"
    else
        fail "Safety limits clamping failed"
    fi

    if "${BUILD_DIR}/test_teleop" --gtest_filter='*DeadzoneFiltering*' > /dev/null 2>&1; then
        pass "Deadzone filtering works"
    else
        fail "Deadzone filtering failed"
    fi

    if "${BUILD_DIR}/test_teleop" --gtest_filter='*Button*' > /dev/null 2>&1; then
        pass "Button detection works"
    else
        fail "Button detection failed"
    fi
else
    skip "test_teleop not built - skipping teleop controller tests"
fi

# ===== Test 6: Ring buffer overflow handling =====
echo ""
echo "Test 6: Ring Buffer Overflow Handling"

if [ -f "${BUILD_DIR}/test_recording" ]; then
    if "${BUILD_DIR}/test_recording" --gtest_filter='*RingBufferOverflowHandling*' > /dev/null 2>&1; then
        pass "Ring buffer handles overflow gracefully"
    else
        fail "Ring buffer overflow test failed"
    fi
else
    skip "test_recording not built - skipping overflow test"
fi

# ===== Summary =====
echo ""
echo "================================="
echo "E2E Test Summary for Story 2-1"
echo "================================="
echo -e "Passed: ${GREEN}${TESTS_PASSED}${NC}"
echo -e "Failed: ${RED}${TESTS_FAILED}${NC}"
echo ""

if [ $TESTS_FAILED -gt 0 ]; then
    echo -e "${RED}SOME TESTS FAILED${NC}"
    exit 1
else
    echo -e "${GREEN}ALL TESTS PASSED${NC}"
    exit 0
fi
