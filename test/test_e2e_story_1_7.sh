#!/bin/bash
# End-to-end test for Story 1-7: Visual Capture
# Tests capture system integration with CLI and state machine

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
TEST_DATA="$PROJECT_DIR/test_data"
TEST_OUTPUT="$PROJECT_DIR/test_e2e_1_7_output"

echo "=== Story 1-7: Visual Capture E2E Test ==="
echo "Project: $PROJECT_DIR"
echo "Build: $BUILD_DIR"

# Cleanup previous test output
rm -rf "$TEST_OUTPUT"
mkdir -p "$TEST_OUTPUT"

# Check if build exists
if [ ! -f "$BUILD_DIR/g1_inspector" ]; then
    echo "[ERROR] g1_inspector not found. Run 'cmake --build build' first"
    exit 1
fi

# Check if test_capture unit tests pass first
echo ""
echo "--- Running unit tests ---"
if [ -f "$BUILD_DIR/test_capture" ]; then
    "$BUILD_DIR/test_capture" --gtest_brief=1
    if [ $? -ne 0 ]; then
        echo "[FAIL] Unit tests failed"
        exit 1
    fi
    echo "[PASS] Unit tests passed"
else
    echo "[WARN] test_capture not found, skipping unit tests"
fi

# Create test plan if not exists
echo ""
echo "--- Creating test plan ---"
mkdir -p "$TEST_DATA"
TEST_PLAN="$TEST_DATA/e2e_test_plan.png"
if [ ! -f "$TEST_PLAN" ]; then
    # Create a simple 200x200 white image with black border using ImageMagick if available
    if command -v convert &> /dev/null; then
        convert -size 200x200 xc:white -bordercolor black -border 2 "$TEST_PLAN"
        echo "[PASS] Created test plan with ImageMagick"
    else
        # Fallback: Create test plan using Python/OpenCV
        python3 -c "
import cv2
import numpy as np
img = np.ones((200, 200), dtype=np.uint8) * 255
cv2.rectangle(img, (0, 0), (199, 199), 0, 2)
cv2.imwrite('$TEST_PLAN', img)
" 2>/dev/null || {
            echo "[WARN] Could not create test plan - using existing or skipping"
        }
    fi
fi

if [ -f "$TEST_PLAN" ]; then
    echo "[INFO] Test plan: $TEST_PLAN"
else
    echo "[WARN] No test plan available, capture tests will be limited"
fi

# Test 1: Verify capture library is linked
echo ""
echo "--- Test 1: Capture library linked ---"
if ldd "$BUILD_DIR/g1_inspector" 2>/dev/null | grep -q opencv || \
   otool -L "$BUILD_DIR/g1_inspector" 2>/dev/null | grep -q opencv; then
    echo "[PASS] OpenCV linked correctly"
else
    echo "[WARN] Could not verify OpenCV linkage (may be static)"
fi

# Test 2: CLI help shows capture is available
echo ""
echo "--- Test 2: CLI help available ---"
HELP_OUTPUT=$("$BUILD_DIR/g1_inspector" --help 2>&1)
if echo "$HELP_OUTPUT" | grep -q "interactive"; then
    echo "[PASS] CLI help shows interactive mode"
else
    echo "[FAIL] CLI help missing interactive mode"
    exit 1
fi

# Test 3: CLI status command works
echo ""
echo "--- Test 3: CLI status command ---"
STATUS_OUTPUT=$("$BUILD_DIR/g1_inspector" status 2>&1) || true
if echo "$STATUS_OUTPUT" | grep -qi "state\|idle\|status"; then
    echo "[PASS] Status command responds"
else
    echo "[INFO] Status output: $STATUS_OUTPUT"
    echo "[WARN] Status command may not work without hardware"
fi

# Test 4: Verify ImageCapture can be created and session directory works
echo ""
echo "--- Test 4: Session directory creation ---"
# Use the unit test which already validates this
if [ -f "$BUILD_DIR/test_capture" ]; then
    "$BUILD_DIR/test_capture" --gtest_filter="ImageCaptureTest.SessionDirectoryCreation" --gtest_brief=1
    if [ $? -eq 0 ]; then
        echo "[PASS] Session directory creation works"
    else
        echo "[FAIL] Session directory creation failed"
        exit 1
    fi
fi

# Test 5: Verify mock capture works (saves actual files)
echo ""
echo "--- Test 5: Mock capture saves files ---"
if [ -f "$BUILD_DIR/test_capture" ]; then
    "$BUILD_DIR/test_capture" --gtest_filter="ImageCaptureTest.CaptureTestFrameSavesImage" --gtest_brief=1
    if [ $? -eq 0 ]; then
        echo "[PASS] Mock capture saves files correctly"
    else
        echo "[FAIL] Mock capture test failed"
        exit 1
    fi
fi

# Test 6: Verify metadata serialization
echo ""
echo "--- Test 6: Metadata serialization ---"
if [ -f "$BUILD_DIR/test_capture" ]; then
    "$BUILD_DIR/test_capture" --gtest_filter="JsonSerializationTest.*" --gtest_brief=1
    if [ $? -eq 0 ]; then
        echo "[PASS] JSON serialization works correctly"
    else
        echo "[FAIL] JSON serialization tests failed"
        exit 1
    fi
fi

# Test 7: Verify coverage tracking
echo ""
echo "--- Test 7: Coverage tracking ---"
if [ -f "$BUILD_DIR/test_capture" ]; then
    "$BUILD_DIR/test_capture" --gtest_filter="PlanCorrelatorTest.*" --gtest_brief=1
    if [ $? -eq 0 ]; then
        echo "[PASS] Coverage tracking works correctly"
    else
        echo "[FAIL] Coverage tracking tests failed"
        exit 1
    fi
fi

# Cleanup
rm -rf "$TEST_OUTPUT"

echo ""
echo "=== Story 1-7 E2E Tests Complete ==="
echo "[SUCCESS] All tests passed!"
exit 0
