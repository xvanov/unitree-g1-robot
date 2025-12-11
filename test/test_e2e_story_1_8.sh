#!/bin/bash
# End-to-end test for Story 1-8: VLM Defect Detection
# Tests the detection_sim executable in dry-run mode

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build"
TEST_DATA="$PROJECT_ROOT/test_data/defect_samples"
OUTPUT_DIR="$PROJECT_ROOT/build/test_outputs/detection_e2e"

echo "=== Story 1-8 E2E Test: VLM Defect Detection ==="
echo "Project root: $PROJECT_ROOT"
echo "Build dir: $BUILD_DIR"
echo ""

# Check if detection_sim exists
if [ ! -f "$BUILD_DIR/detection_sim" ]; then
    echo "ERROR: detection_sim not found at $BUILD_DIR/detection_sim"
    echo "Please build the project first: cd build && cmake .. && make"
    exit 1
fi

# Check test data exists
if [ ! -d "$TEST_DATA" ]; then
    echo "ERROR: Test data directory not found: $TEST_DATA"
    exit 1
fi

# Clean up previous test outputs
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

echo "--- Test 1: Dry-run mode with test images ---"
$BUILD_DIR/detection_sim --images "$TEST_DATA" --output "$OUTPUT_DIR" --dry-run

# Verify outputs were created
if [ ! -f "$OUTPUT_DIR/defects.json" ]; then
    echo "FAIL: defects.json not created"
    exit 1
fi
echo "PASS: defects.json created"

# Verify JSON structure
if ! python3 -c "import json; json.load(open('$OUTPUT_DIR/defects.json'))" 2>/dev/null; then
    echo "FAIL: defects.json is not valid JSON"
    exit 1
fi
echo "PASS: defects.json is valid JSON"

# Verify annotated images directory exists
if [ ! -d "$OUTPUT_DIR/annotated" ]; then
    echo "FAIL: annotated/ directory not created"
    exit 1
fi
echo "PASS: annotated/ directory created"

# Check that annotated images were created (dry-run produces mock defects)
ANNOTATED_COUNT=$(ls -1 "$OUTPUT_DIR/annotated/"*.jpg 2>/dev/null | wc -l || echo "0")
if [ "$ANNOTATED_COUNT" -eq 0 ]; then
    echo "FAIL: No annotated images created"
    exit 1
fi
echo "PASS: $ANNOTATED_COUNT annotated image(s) created"

# Verify defects.json contains expected structure
DEFECT_COUNT=$(python3 -c "import json; d=json.load(open('$OUTPUT_DIR/defects.json')); print(d.get('total_defects', 0))" 2>/dev/null || echo "0")
if [ "$DEFECT_COUNT" -eq 0 ]; then
    echo "FAIL: No defects in output (dry-run should produce mock defects)"
    exit 1
fi
echo "PASS: Found $DEFECT_COUNT defects in output"

echo ""
echo "--- Test 2: Help option ---"
$BUILD_DIR/detection_sim --help > /dev/null 2>&1
echo "PASS: --help works"

echo ""
echo "--- Test 3: Missing required argument handling ---"
if $BUILD_DIR/detection_sim 2>&1 | grep -q "Must specify"; then
    echo "PASS: Proper error for missing arguments"
else
    echo "FAIL: Should error when no --images or --session provided"
    exit 1
fi

echo ""
echo "--- Test 4: Unit tests ---"
if [ -f "$BUILD_DIR/test_detection" ]; then
    $BUILD_DIR/test_detection
    echo "PASS: All unit tests passed"
else
    echo "SKIP: test_detection not found (GTest may not be available)"
fi

echo ""
echo "=== All Story 1-8 E2E Tests PASSED ==="
exit 0
