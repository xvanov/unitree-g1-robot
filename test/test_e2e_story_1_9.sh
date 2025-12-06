#!/bin/bash
# End-to-end test for Story 1-9: Report Generation
# Tests PDF and CSV report generation from mock inspection data

# Don't exit on error - we want to run all tests and report failures
# set -e

echo "=== Story 1-9 E2E Test: Report Generation ==="

# Setup
TEST_SESSION="e2e_report_test_$$"
TEST_SESSION_DIR="data/inspections/${TEST_SESSION}"
OUTPUT_DIR="data/reports/${TEST_SESSION}"
PASSED=0
FAILED=0

cleanup() {
    echo "[CLEANUP] Removing test data..."
    rm -rf "${TEST_SESSION_DIR}" "${OUTPUT_DIR}"
}
trap cleanup EXIT

# Create test session directory
mkdir -p "${TEST_SESSION_DIR}/images"
mkdir -p "${TEST_SESSION_DIR}/annotated"

echo "[TEST] Creating mock inspection data..."

# Create mock analysis_results.json
cat > "${TEST_SESSION_DIR}/analysis_results.json" << 'EOF'
{
  "session_id": "e2e_report_test",
  "images_processed": 5,
  "total_defects": 3,
  "defects_by_image": [
    {
      "image": "img_00000001.jpg",
      "defects": [
        {
          "id": "def_001",
          "type": "QUALITY_ISSUE",
          "description": "Scratch on tile surface near entrance",
          "confidence": 0.87,
          "severity": "high",
          "trade": "finishes",
          "image_location": {"x": 320, "y": 240},
          "bounding_box": {"x": 280, "y": 200, "width": 80, "height": 80},
          "plan_location": {"x": 2.5, "y": 1.2}
        }
      ]
    },
    {
      "image": "img_00000003.jpg",
      "defects": [
        {
          "id": "def_002",
          "type": "LOCATION_ERROR",
          "description": "Outlet misaligned by 5cm from plan",
          "confidence": 0.92,
          "severity": "medium",
          "trade": "mep",
          "image_location": {"x": 150, "y": 300},
          "bounding_box": {"x": 120, "y": 270, "width": 60, "height": 60},
          "plan_location": {"x": 4.1, "y": 3.8}
        },
        {
          "id": "def_003",
          "type": "SAFETY_HAZARD",
          "description": "Exposed electrical wiring",
          "confidence": 0.95,
          "severity": "high",
          "trade": "mep",
          "image_location": {"x": 400, "y": 180},
          "bounding_box": {"x": 370, "y": 150, "width": 60, "height": 60},
          "plan_location": {"x": 4.5, "y": 3.5}
        }
      ]
    }
  ]
}
EOF

# Create a simple test plan image using ImageMagick if available
# Note: Report generation works without plan image (just skips overlay page)
if command -v convert &> /dev/null; then
    convert -size 800x600 xc:white \
        -fill lightgray -draw "rectangle 50,50 750,550" \
        -fill black -draw "rectangle 100,100 200,200" \
        "${TEST_SESSION_DIR}/plan.png"
    echo "[TEST] Created plan.png with ImageMagick"
else
    # Skip plan image - report generation handles missing plan gracefully
    echo "[TEST] ImageMagick not available - skipping plan.png (report will generate without plan overlay)"
fi

# Test 1: Run report generation
echo ""
echo "[TEST 1] Running report generation..."
if ./build/g1_inspector --generate-report --inspection "${TEST_SESSION}"; then
    echo "[PASS] Report generation command succeeded"
    ((PASSED++))
else
    echo "[FAIL] Report generation command failed"
    ((FAILED++))
fi

# Test 2: Check PDF file exists
echo ""
echo "[TEST 2] Checking PDF file exists..."
PDF_FILE=$(ls "${OUTPUT_DIR}"/*.pdf 2>/dev/null | head -1)
if [ -n "${PDF_FILE}" ] && [ -f "${PDF_FILE}" ]; then
    echo "[PASS] PDF file exists: ${PDF_FILE}"
    ((PASSED++))
else
    echo "[FAIL] PDF file not found in ${OUTPUT_DIR}"
    ((FAILED++))
fi

# Test 3: Check PDF has content
echo ""
echo "[TEST 3] Checking PDF has content..."
if [ -n "${PDF_FILE}" ] && [ -s "${PDF_FILE}" ]; then
    PDF_SIZE=$(stat -f%z "${PDF_FILE}" 2>/dev/null || stat -c%s "${PDF_FILE}" 2>/dev/null)
    if [ "${PDF_SIZE}" -gt 1000 ]; then
        echo "[PASS] PDF has content: ${PDF_SIZE} bytes"
        ((PASSED++))
    else
        echo "[FAIL] PDF too small: ${PDF_SIZE} bytes"
        ((FAILED++))
    fi
else
    echo "[FAIL] PDF file empty or not accessible"
    ((FAILED++))
fi

# Test 4: Check CSV file exists
echo ""
echo "[TEST 4] Checking CSV punch list exists..."
CSV_FILE="${OUTPUT_DIR}/punch_list.csv"
if [ -f "${CSV_FILE}" ]; then
    echo "[PASS] CSV file exists: ${CSV_FILE}"
    ((PASSED++))
else
    echo "[FAIL] CSV file not found"
    ((FAILED++))
fi

# Test 5: Check CSV format (header + data rows)
echo ""
echo "[TEST 5] Validating CSV format..."
if [ -f "${CSV_FILE}" ]; then
    HEADER=$(head -1 "${CSV_FILE}")
    if echo "${HEADER}" | grep -q '"ID"' && echo "${HEADER}" | grep -q '"Severity"'; then
        ROW_COUNT=$(wc -l < "${CSV_FILE}" | tr -d ' ')
        if [ "${ROW_COUNT}" -ge 4 ]; then  # Header + 3 defects
            echo "[PASS] CSV has correct format: header + ${ROW_COUNT} lines"
            ((PASSED++))
        else
            echo "[FAIL] CSV has insufficient rows: ${ROW_COUNT}"
            ((FAILED++))
        fi
    else
        echo "[FAIL] CSV header format incorrect"
        ((FAILED++))
    fi
else
    echo "[FAIL] CSV file not accessible"
    ((FAILED++))
fi

# Test 6: Check filename pattern (AC5)
echo ""
echo "[TEST 6] Validating filename pattern..."
if [ -n "${PDF_FILE}" ]; then
    BASENAME=$(basename "${PDF_FILE}")
    if echo "${BASENAME}" | grep -qE '^inspection_[0-9]{4}-[0-9]{2}-[0-9]{2}_.*\.pdf$'; then
        echo "[PASS] Filename follows pattern: ${BASENAME}"
        ((PASSED++))
    else
        echo "[FAIL] Filename doesn't match pattern: ${BASENAME}"
        ((FAILED++))
    fi
else
    echo "[FAIL] Cannot check filename - PDF not found"
    ((FAILED++))
fi

# Test 7: Missing session handling
echo ""
echo "[TEST 7] Testing missing session handling..."
OUTPUT=$(./build/g1_inspector --generate-report --inspection "nonexistent_session" 2>&1)
if echo "${OUTPUT}" | grep -qi "not found\|error"; then
    echo "[PASS] Proper error handling for missing session"
    ((PASSED++))
else
    echo "[FAIL] Missing session should produce error message"
    echo "  Output was: ${OUTPUT}"
    ((FAILED++))
fi

# Test 8: Missing --inspection flag
echo ""
echo "[TEST 8] Testing missing --inspection flag..."
if ./build/g1_inspector --generate-report 2>&1 | grep -q "requires"; then
    echo "[PASS] Proper error when --inspection missing"
    ((PASSED++))
else
    echo "[FAIL] Should require --inspection flag"
    ((FAILED++))
fi

# Summary
echo ""
echo "=== E2E Test Summary ==="
echo "Passed: ${PASSED}"
echo "Failed: ${FAILED}"
echo ""

if [ ${FAILED} -eq 0 ]; then
    echo "[SUCCESS] All Story 1-9 E2E tests passed!"
    exit 0
else
    echo "[FAILURE] ${FAILED} test(s) failed"
    exit 1
fi
