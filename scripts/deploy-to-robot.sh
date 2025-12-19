#!/bin/bash
# deploy-to-robot.sh - Deploy and build g1_inspector on G1 robot
# Story 1-6: Dual-Environment Deployment
#
# Usage:
#   ./scripts/deploy-to-robot.sh              # Sync code to robot
#   ./scripts/deploy-to-robot.sh --build      # Sync + build on robot
#   ./scripts/deploy-to-robot.sh --run        # Sync + build + run greeter
#   ./scripts/deploy-to-robot.sh --build --run  # Full deployment
#   ./scripts/deploy-to-robot.sh --dry-run    # Show what would be synced
#   ./scripts/deploy-to-robot.sh --clean      # Remove deployed files from robot
#
# Environment:
#   ROBOT_IP   - Override default robot IP (default: 192.168.123.164)
#   ROBOT_USER - Override default user (default: unitree)
#   ROBOT_PASS - Override default password (default: 123)
#
# Requirements:
#   - rsync and ssh (or sshpass for password auth)
#   - Robot powered on and connected via ethernet
#   - Dev machine on 192.168.123.x subnet

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# ============================================================================
# Configuration
# ============================================================================

# Robot connection - support environment variable override
ROBOT_IP="${ROBOT_IP:-192.168.123.164}"
ROBOT_USER="${ROBOT_USER:-unitree}"
ROBOT_PASS="${ROBOT_PASS:-123}"

# Remote paths
REMOTE_DIR="/home/${ROBOT_USER}/g1_inspector"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ============================================================================
# Helper Functions
# ============================================================================

log_info() {
    echo -e "${GREEN}[DEPLOY]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[DEPLOY]${NC} $1"
}

log_error() {
    echo -e "${RED}[DEPLOY]${NC} ERROR: $1" >&2
}

# Check if SSH key auth is available
has_ssh_key() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_IP" "exit" 2>/dev/null
}

# SSH command with appropriate auth method
ssh_cmd() {
    if has_ssh_key; then
        ssh -o ConnectTimeout=10 "$ROBOT_USER@$ROBOT_IP" "$@"
    else
        if ! command -v sshpass &>/dev/null; then
            log_error "SSH key auth not available and sshpass not installed"
            log_error "Either set up SSH keys or: sudo apt install sshpass"
            exit 1
        fi
        sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "$ROBOT_USER@$ROBOT_IP" "$@"
    fi
}

# rsync with appropriate auth method
rsync_cmd() {
    if has_ssh_key; then
        rsync "$@"
    else
        if ! command -v sshpass &>/dev/null; then
            log_error "SSH key auth not available and sshpass not installed"
            exit 1
        fi
        sshpass -p "$ROBOT_PASS" rsync -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" "$@"
    fi
}

check_robot_connection() {
    log_info "Checking connection to robot at $ROBOT_IP..."
    if ! ping -c 1 -W 2 "$ROBOT_IP" &>/dev/null; then
        log_error "Cannot reach robot at $ROBOT_IP"
        log_error "Make sure:"
        log_error "  1. Robot is powered on"
        log_error "  2. Ethernet cable is connected"
        log_error "  3. Dev machine is on 192.168.123.x subnet"
        log_error ""
        log_error "You can override IP: ROBOT_IP=192.168.123.233 $0"
        exit 1
    fi
    log_info "Robot is reachable"
}

# ============================================================================
# Deploy Functions
# ============================================================================

sync_code() {
    local dry_run="$1"
    local rsync_opts="-avz --progress"

    if [ "$dry_run" = "true" ]; then
        rsync_opts="$rsync_opts --dry-run"
        log_info "DRY RUN - showing what would be synced"
    else
        log_info "Syncing code to robot..."
    fi

    # Create remote directory
    if [ "$dry_run" != "true" ]; then
        ssh_cmd "mkdir -p $REMOTE_DIR"
    fi

    # rsync with exclusions (Story 1-6 spec)
    rsync_cmd $rsync_opts \
        --exclude='build/' \
        --exclude='build2/' \
        --exclude='.git/' \
        --exclude='docker/' \
        --exclude='*.o' \
        --exclude='external/unitree_sdk2_python/' \
        --exclude='.cache/' \
        --exclude='__pycache__/' \
        --exclude='*.pyc' \
        --exclude='data/recordings/' \
        "$PROJECT_ROOT/" "$ROBOT_USER@$ROBOT_IP:$REMOTE_DIR/"

    if [ "$dry_run" != "true" ]; then
        log_info "Code synced to $ROBOT_USER@$ROBOT_IP:$REMOTE_DIR"
    fi
}

build_on_robot() {
    log_info "Building on robot..."

    ssh_cmd bash -s << 'BUILDEOF'
set -e
cd ~/g1_inspector

# Source environment
if [ -f config/robot.env ]; then
    source config/robot.env
fi

# Create build directory
mkdir -p build
cd build

# Configure with ROBOT_BUILD=ON (auto-detected on Jetson, but explicit is clearer)
echo "=== CMake Configure ==="
cmake .. -DROBOT_BUILD=ON -DCMAKE_BUILD_TYPE=Release

# Build with parallel jobs
echo "=== Building ==="
make -j$(nproc)

echo ""
echo "=== Build Complete ==="
ls -la g1_inspector 2>/dev/null || echo "Main binary not found - check build output"
BUILDEOF

    log_info "Build complete on robot"
}

run_greeter() {
    log_info "Running greeter demo on robot..."

    ssh_cmd bash -s << 'RUNEOF'
set -e
cd ~/g1_inspector

# Source environment
if [ -f config/robot.env ]; then
    source config/robot.env
fi

echo "=== Running Greeter Demo ==="
echo "Config: config/greeter.yaml"
echo "Mode: --dry-run (no robot movement)"
echo ""

cd build
./g1_inspector --greeter --dry-run --config ../config/greeter.yaml
RUNEOF
}

run_greeter_live() {
    log_info "Running greeter demo LIVE on robot (with robot movement)..."
    log_warn "SAFETY: Ensure clear area around robot and E-stop ready!"

    read -p "Press Enter to continue or Ctrl+C to abort..."

    ssh_cmd bash -s << 'RUNEOF'
set -e
cd ~/g1_inspector

# Source environment
if [ -f config/robot.env ]; then
    source config/robot.env
fi

echo "=== Running Greeter Demo (LIVE) ==="
echo "Config: config/greeter.yaml"
echo ""

cd build
./g1_inspector --greeter --config ../config/greeter.yaml
RUNEOF
}

clean_deployment() {
    log_info "Removing deployed files from robot..."
    ssh_cmd "rm -rf $REMOTE_DIR"
    log_info "Cleaned up $REMOTE_DIR on robot"
}

show_status() {
    log_info "Checking robot status..."

    ssh_cmd bash -s << 'STATUSEOF'
echo "=== Robot Environment ==="
echo ""

# Check project directory
if [ -d ~/g1_inspector ]; then
    echo "Project: ~/g1_inspector exists"
    ls -la ~/g1_inspector/build/g1_inspector 2>/dev/null && echo "Binary: PRESENT" || echo "Binary: NOT BUILT"
else
    echo "Project: NOT DEPLOYED"
fi
echo ""

# Check environment
echo "=== Environment ==="
echo "CYCLONEDDS_URI: ${CYCLONEDDS_URI:-not set}"
echo "ANTHROPIC_API_KEY: ${ANTHROPIC_API_KEY:+set (hidden)}"
echo ""

# Check L4T/Jetson
echo "=== Platform ==="
if [ -f /etc/nv_tegra_release ]; then
    cat /etc/nv_tegra_release
else
    echo "Not Jetson (no /etc/nv_tegra_release)"
fi
STATUSEOF
}

# ============================================================================
# Usage
# ============================================================================

show_help() {
    echo "G1 Robot Deploy Script (Story 1-6)"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  (no args)       Sync code to robot"
    echo "  --build         Sync code and build on robot"
    echo "  --run           Sync, build, and run greeter (dry-run mode)"
    echo "  --run-live      Sync, build, and run greeter (LIVE mode, robot moves)"
    echo "  --dry-run       Show what would be synced (no changes)"
    echo "  --status        Show robot deployment status"
    echo "  --clean         Remove deployed files from robot"
    echo "  --help          Show this help"
    echo ""
    echo "Environment Variables:"
    echo "  ROBOT_IP        Robot IP address (default: 192.168.123.164)"
    echo "  ROBOT_USER      Robot username (default: unitree)"
    echo "  ROBOT_PASS      Robot password (default: 123)"
    echo ""
    echo "Examples:"
    echo "  $0 --build --run              # Full deploy and test"
    echo "  ROBOT_IP=192.168.123.233 $0   # Use different robot IP"
    echo ""
}

# ============================================================================
# Main
# ============================================================================

main() {
    local do_build=false
    local do_run=false
    local do_run_live=false
    local dry_run=false
    local do_clean=false
    local do_status=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --build)
                do_build=true
                shift
                ;;
            --run)
                do_build=true
                do_run=true
                shift
                ;;
            --run-live)
                do_build=true
                do_run_live=true
                shift
                ;;
            --dry-run)
                dry_run=true
                shift
                ;;
            --clean)
                do_clean=true
                shift
                ;;
            --status)
                do_status=true
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done

    echo "============================================================"
    echo "  G1 Robot Deploy Script"
    echo "  Target: $ROBOT_USER@$ROBOT_IP"
    echo "============================================================"
    echo ""

    check_robot_connection

    if [ "$do_status" = true ]; then
        show_status
        exit 0
    fi

    if [ "$do_clean" = true ]; then
        clean_deployment
        exit 0
    fi

    # Sync code (always, unless only status/clean requested)
    sync_code "$dry_run"

    if [ "$dry_run" = true ]; then
        log_info "Dry run complete - no changes made"
        exit 0
    fi

    # Build if requested
    if [ "$do_build" = true ]; then
        build_on_robot
    fi

    # Run if requested
    if [ "$do_run_live" = true ]; then
        run_greeter_live
    elif [ "$do_run" = true ]; then
        run_greeter
    fi

    echo ""
    log_info "Deploy complete!"
    echo ""
    echo "Next steps:"
    echo "  SSH to robot:     ssh $ROBOT_USER@$ROBOT_IP"
    echo "  Run greeter:      cd g1_inspector/build && ./g1_inspector --greeter --dry-run"
    echo ""
}

main "$@"
