#!/bin/bash
# deploy-to-robot.sh - Deploy test scripts to G1 robot
# Copyright 2025 G1 Inspector Project
# Licensed under Apache-2.0
#
# Usage:
#   ./scripts/deploy-to-robot.sh              # Deploy SDK and scripts
#   ./scripts/deploy-to-robot.sh --scripts-only   # Deploy scripts only (skip SDK)
#   ./scripts/deploy-to-robot.sh --run-sensors    # Deploy and run sensor test
#   ./scripts/deploy-to-robot.sh --run-hello      # Deploy and run wave hello
#   ./scripts/deploy-to-robot.sh --run-camera     # Deploy and run camera capture
#   ./scripts/deploy-to-robot.sh --clean          # Remove deployed files from robot
#
# Requirements:
#   - sshpass (for password auth): sudo apt install sshpass
#   - Robot powered on and connected via ethernet
#   - Dev machine on 192.168.123.x subnet

set -e

# ============================================================================
# Configuration
# ============================================================================
ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"
ROBOT_PASS="123"
ROBOT_INTERFACE="eth0"  # Network interface on robot for DDS

# Remote paths (no installation, just copy)
REMOTE_DIR="/home/unitree/g1_test"
REMOTE_SDK_DIR="${REMOTE_DIR}/unitree_sdk2py"

# Local paths (relative to project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOCAL_SDK_DIR="${PROJECT_ROOT}/external/unitree_sdk2_python/unitree_sdk2py"

# Scripts to deploy
DEPLOY_SCRIPTS=(
    "scripts/read_g1_sensors.py"
    "scripts/hello_world_g1.py"
    "scripts/sensor_hello_world.py"
)

# ============================================================================
# Helper Functions
# ============================================================================
log_info() {
    echo "[DEPLOY] $1"
}

log_error() {
    echo "[DEPLOY] ERROR: $1" >&2
}

log_success() {
    echo "[DEPLOY] SUCCESS: $1"
}

check_sshpass() {
    if ! command -v sshpass &> /dev/null; then
        log_error "sshpass not found. Install with: sudo apt install sshpass"
        exit 1
    fi
}

ssh_cmd() {
    sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "$ROBOT_USER@$ROBOT_IP" "$@"
}

scp_cmd() {
    sshpass -p "$ROBOT_PASS" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -r "$@"
}

check_robot_connection() {
    log_info "Checking connection to robot at $ROBOT_IP..."
    if ! ping -c 1 -W 2 "$ROBOT_IP" &> /dev/null; then
        log_error "Cannot reach robot at $ROBOT_IP"
        log_error "Make sure:"
        log_error "  1. Robot is powered on"
        log_error "  2. Ethernet cable is connected"
        log_error "  3. Dev machine is on 192.168.123.x subnet"
        exit 1
    fi
    log_info "Robot is reachable"
}

# ============================================================================
# Deploy Functions
# ============================================================================
deploy_sdk() {
    log_info "Deploying unitree_sdk2py to robot..."

    # Check local SDK exists
    if [ ! -d "$LOCAL_SDK_DIR" ]; then
        log_error "SDK not found at: $LOCAL_SDK_DIR"
        exit 1
    fi

    # Create remote directory
    ssh_cmd "mkdir -p $REMOTE_DIR"

    # Copy SDK (this is the main bulk - ~5MB)
    log_info "Copying SDK (this may take a moment)..."
    scp_cmd "$LOCAL_SDK_DIR" "$ROBOT_USER@$ROBOT_IP:$REMOTE_DIR/"

    log_success "SDK deployed to $REMOTE_SDK_DIR"
}

deploy_scripts() {
    log_info "Deploying test scripts..."

    for script in "${DEPLOY_SCRIPTS[@]}"; do
        local_path="${PROJECT_ROOT}/${script}"
        if [ -f "$local_path" ]; then
            scp_cmd "$local_path" "$ROBOT_USER@$ROBOT_IP:$REMOTE_DIR/"
            log_info "  Deployed: $(basename "$script")"
        else
            log_error "Script not found: $local_path"
        fi
    done

    log_success "Scripts deployed to $REMOTE_DIR"
}

create_run_script() {
    log_info "Creating run helper script on robot..."

    ssh_cmd "cat > $REMOTE_DIR/run_test.sh << 'RUNEOF'
#!/bin/bash
# Run helper for G1 test scripts
# Sets PYTHONPATH and runs the specified script

SCRIPT_DIR=\"\$(cd \"\$(dirname \"\${BASH_SOURCE[0]}\")\" && pwd)\"
export PYTHONPATH=\"\${SCRIPT_DIR}/unitree_sdk2py:\${PYTHONPATH}\"

# Network interface on robot
INTERFACE=\"eth0\"

if [ -z \"\$1\" ]; then
    echo \"Usage: ./run_test.sh <script.py>\"
    echo \"\"
    echo \"Available scripts:\"
    ls -1 \"\${SCRIPT_DIR}\"/*.py 2>/dev/null || echo \"  (none found)\"
    exit 1
fi

SCRIPT=\"\${SCRIPT_DIR}/\$1\"
if [ ! -f \"\$SCRIPT\" ]; then
    echo \"ERROR: Script not found: \$SCRIPT\"
    exit 1
fi

echo \"Running: python3 \$SCRIPT \$INTERFACE\"
python3 \"\$SCRIPT\" \"\$INTERFACE\"
RUNEOF"

    ssh_cmd "chmod +x $REMOTE_DIR/run_test.sh"
    log_success "Run helper created at $REMOTE_DIR/run_test.sh"
}

show_usage_on_robot() {
    echo ""
    echo "============================================================"
    echo "  DEPLOYMENT COMPLETE"
    echo "============================================================"
    echo ""
    echo "Files deployed to robot at: $REMOTE_DIR"
    echo ""
    echo "To run tests on robot, SSH in and use:"
    echo ""
    echo "  ssh $ROBOT_USER@$ROBOT_IP"
    echo "  # password: $ROBOT_PASS"
    echo ""
    echo "  cd $REMOTE_DIR"
    echo "  ./run_test.sh read_g1_sensors.py    # Read IMU/joint data"
    echo "  ./run_test.sh hello_world_g1.py     # Wave hand test"
    echo "  python3 sensor_hello_world.py eth0 --camera  # Camera capture"
    echo ""
    echo "Or run directly from this machine:"
    echo ""
    echo "  ./scripts/deploy-to-robot.sh --run-sensors"
    echo "  ./scripts/deploy-to-robot.sh --run-hello"
    echo "  ./scripts/deploy-to-robot.sh --run-camera"
    echo ""
    echo "To clean up when done:"
    echo ""
    echo "  ./scripts/deploy-to-robot.sh --clean"
    echo ""
}

# ============================================================================
# Run Functions
# ============================================================================
run_sensors() {
    log_info "Running sensor test on robot..."
    echo ""
    echo "============================================================"
    echo "  SENSOR TEST - Press Ctrl+C to stop"
    echo "============================================================"
    echo ""
    ssh_cmd "cd $REMOTE_DIR && ./run_test.sh read_g1_sensors.py"
}

run_hello() {
    log_info "Running hello world (wave) test on robot..."
    echo ""
    echo "============================================================"
    echo "  HELLO WORLD TEST"
    echo "  Robot will: Stand up -> Wave hand -> Enter damp mode"
    echo "============================================================"
    echo ""
    echo "SAFETY: Ensure clear area around robot and E-stop ready!"
    echo ""
    read -p "Press Enter to continue or Ctrl+C to abort..."
    ssh_cmd "cd $REMOTE_DIR && ./run_test.sh hello_world_g1.py"
}

run_camera() {
    log_info "Running camera capture test on robot..."
    echo ""
    echo "============================================================"
    echo "  CAMERA TEST - RealSense D435 (no ROS2 required)"
    echo "============================================================"
    echo ""
    # Camera uses pyrealsense2 directly, no network interface needed for DDS
    # But sensor_hello_world.py requires interface arg for consistency
    ssh_cmd "cd $REMOTE_DIR && python3 sensor_hello_world.py $ROBOT_INTERFACE --camera --output-dir ./sensor_captures"
    echo ""
    log_info "Listing captured files..."
    ssh_cmd "ls -la $REMOTE_DIR/sensor_captures/ 2>/dev/null || echo 'No captures found'"
}

clean_deployment() {
    log_info "Removing deployed files from robot..."
    ssh_cmd "rm -rf $REMOTE_DIR"
    log_success "Cleaned up $REMOTE_DIR on robot"
}

# ============================================================================
# Main
# ============================================================================
main() {
    echo "============================================================"
    echo "  G1 Robot Deploy Script"
    echo "============================================================"
    echo ""

    check_sshpass
    check_robot_connection

    case "${1:-}" in
        --run-sensors)
            run_sensors
            ;;
        --run-hello)
            run_hello
            ;;
        --run-camera)
            run_camera
            ;;
        --clean)
            clean_deployment
            ;;
        --scripts-only)
            ssh_cmd "mkdir -p $REMOTE_DIR"
            deploy_scripts
            create_run_script
            show_usage_on_robot
            ;;
        "")
            deploy_sdk
            deploy_scripts
            create_run_script
            show_usage_on_robot
            ;;
        *)
            echo "Usage: $0 [--scripts-only|--run-sensors|--run-hello|--run-camera|--clean]"
            echo ""
            echo "  (no args)       Deploy SDK and scripts to robot"
            echo "  --scripts-only  Deploy scripts only (SDK already on robot)"
            echo "  --run-sensors   Run sensor reading test (IMU/joints)"
            echo "  --run-hello     Run wave hand test"
            echo "  --run-camera    Run camera capture test (RealSense)"
            echo "  --clean         Remove deployed files from robot"
            exit 1
            ;;
    esac
}

main "$@"
