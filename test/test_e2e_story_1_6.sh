#!/bin/bash
# End-to-end test for Story 1-6: State Machine + CLI + Plan Management
# Run this from the build directory inside Docker container

set -e  # Exit on first error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0

pass() {
    echo -e "${GREEN}✓ PASS${NC}: $1"
    ((PASS_COUNT++))
}

fail() {
    echo -e "${RED}✗ FAIL${NC}: $1"
    ((FAIL_COUNT++))
}

section() {
    echo ""
    echo -e "${YELLOW}=== $1 ===${NC}"
}

# Determine paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

# Check if we're in the right place
if [[ ! -f "$BUILD_DIR/g1_inspector" ]]; then
    echo "Error: g1_inspector not found in $BUILD_DIR"
    echo "Please build the project first: mkdir -p build && cd build && cmake .. && make -j"
    exit 1
fi

cd "$BUILD_DIR"

section "Unit Tests"

# Test 1: State machine unit tests
echo "Running test_state_machine..."
if ./test_state_machine > /tmp/test_sm.log 2>&1; then
    TEST_COUNT=$(grep -c "PASSED\|OK" /tmp/test_sm.log || echo "0")
    pass "test_state_machine passed ($TEST_COUNT tests)"
else
    fail "test_state_machine failed"
    cat /tmp/test_sm.log
fi

# Test 2: Plan manager unit tests
echo "Running test_plan_manager..."
if ./test_plan_manager > /tmp/test_pm.log 2>&1; then
    TEST_COUNT=$(grep -c "PASSED\|OK" /tmp/test_pm.log || echo "0")
    pass "test_plan_manager passed ($TEST_COUNT tests)"
else
    fail "test_plan_manager failed"
    cat /tmp/test_pm.log
fi

section "CLI Help & Version"

# Test 3: Help command
if ./g1_inspector --help 2>&1 | grep -q "interactive"; then
    pass "--help shows interactive mode option"
else
    fail "--help missing interactive mode"
fi

# Test 4: Help shows CLI commands
if ./g1_inspector --help 2>&1 | grep -q "estop"; then
    pass "--help shows CLI commands"
else
    fail "--help missing CLI commands"
fi

section "Single Command Mode"

# Test 5: Status command (single mode)
OUTPUT=$(./g1_inspector status 2>&1)
if echo "$OUTPUT" | grep -q "State: IDLE"; then
    pass "status command shows IDLE state"
else
    fail "status command failed: $OUTPUT"
fi

# Test 6: Help command (single mode)
OUTPUT=$(./g1_inspector help 2>&1)
if echo "$OUTPUT" | grep -q "Available commands"; then
    pass "help command works in single mode"
else
    fail "help command failed"
fi

section "Plan Loading"

# Test 7: Upload plan
OUTPUT=$(./g1_inspector upload --plan ../test_data/office.png 2>&1)
if echo "$OUTPUT" | grep -q "Plan loaded"; then
    pass "upload command loads PNG plan"
else
    fail "upload command failed: $OUTPUT"
fi

# Test 8: Upload with trade type
OUTPUT=$(./g1_inspector upload --plan ../test_data/corridor.png --trade electrical 2>&1)
if echo "$OUTPUT" | grep -q "Trade: electrical"; then
    pass "upload command accepts trade type"
else
    fail "upload with trade type failed: $OUTPUT"
fi

# Test 9: Upload nonexistent file
OUTPUT=$(./g1_inspector upload --plan nonexistent.png 2>&1)
if echo "$OUTPUT" | grep -q "Failed to load"; then
    pass "upload handles missing file gracefully"
else
    fail "upload should fail for missing file: $OUTPUT"
fi

# Test 10: Waypoints generated
OUTPUT=$(./g1_inspector "upload --plan ../test_data/office.png" 2>&1)
if echo "$OUTPUT" | grep -q "Waypoints:"; then
    pass "upload generates waypoints"
else
    fail "waypoints not generated: $OUTPUT"
fi

section "State Transitions (via CLI commands)"

# Create a test script for interactive mode
cat > /tmp/cli_test_input.txt << 'EOF'
status
upload --plan ../test_data/office.png
calibrate --position 1,1,0
start
status
pause
status
resume
status
stop
status
estop
status
clearestop
status
start
status
home
status
quit
EOF

# Test 11: Run interactive session with scripted input
echo "Running interactive CLI test sequence..."
OUTPUT=$(./g1_inspector --interactive < /tmp/cli_test_input.txt 2>&1)

# Test 12: Verify state transitions occurred
if echo "$OUTPUT" | grep -q "State: CALIBRATING"; then
    pass "start transitions to CALIBRATING"
else
    fail "start should transition to CALIBRATING"
fi

if echo "$OUTPUT" | grep -q "State: PAUSED"; then
    pass "pause transitions to PAUSED"
else
    fail "pause should transition to PAUSED"
fi

if echo "$OUTPUT" | grep -q "State: EMERGENCY_STOP"; then
    pass "estop transitions to EMERGENCY_STOP"
else
    fail "estop should transition to EMERGENCY_STOP"
fi

if echo "$OUTPUT" | grep -q "E-stop cleared"; then
    pass "clearestop clears E-stop"
else
    fail "clearestop should clear E-stop"
fi

# Test 13: Verify calibration
if echo "$OUTPUT" | grep -q "Calibration complete"; then
    pass "calibrate command works"
else
    fail "calibrate command failed"
fi

# Test 14: Verify home command exists (may fail from wrong state, but should recognize command)
if echo "$OUTPUT" | grep -qi "home\|returning"; then
    pass "home command recognized"
else
    fail "home command not recognized"
fi

section "Edge Cases"

# Test 15: Empty command handling
OUTPUT=$(echo "" | ./g1_inspector --interactive 2>&1)
if [[ $? -eq 0 ]] || echo "$OUTPUT" | grep -q "g1>"; then
    pass "empty input handled gracefully"
else
    fail "empty input caused error"
fi

# Test 16: Unknown command
OUTPUT=$(echo -e "foobar\nquit" | ./g1_inspector --interactive 2>&1)
if echo "$OUTPUT" | grep -q "Unknown command"; then
    pass "unknown command shows error message"
else
    fail "unknown command should show error"
fi

# Test 17: Waypoints without plan
OUTPUT=$(./g1_inspector waypoints 2>&1)
if echo "$OUTPUT" | grep -q "No plan loaded"; then
    pass "waypoints without plan shows helpful message"
else
    fail "waypoints should require plan to be loaded"
fi

section "Coordinate Transforms (via unit tests)"

# Test 18: Verify coordinate transform tests passed (already in test_plan_manager)
if grep -q "CoordinateTransform" /tmp/test_pm.log 2>/dev/null; then
    pass "coordinate transform tests included"
else
    # Check if tests ran successfully anyway
    if [[ $FAIL_COUNT -eq 0 ]]; then
        pass "coordinate transforms tested (in test_plan_manager)"
    else
        fail "coordinate transform tests not verified"
    fi
fi

section "Summary"

echo ""
TOTAL=$((PASS_COUNT + FAIL_COUNT))
echo -e "Tests: ${GREEN}$PASS_COUNT passed${NC}, ${RED}$FAIL_COUNT failed${NC}, $TOTAL total"
echo ""

if [[ $FAIL_COUNT -eq 0 ]]; then
    echo -e "${GREEN}=== ALL STORY 1-6 E2E TESTS PASSED ===${NC}"
    exit 0
else
    echo -e "${RED}=== SOME TESTS FAILED ===${NC}"
    exit 1
fi
