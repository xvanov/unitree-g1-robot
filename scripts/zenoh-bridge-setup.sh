#!/bin/bash
# Zenoh-DDS Bridge Setup for Unitree G1 Robot
# This enables reliable DDS communication over WiFi

set -e

ZENOH_BRIDGE="/home/k/zenoh-plugin-dds/target/release/zenoh-bridge-dds"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_usage() {
    echo "Zenoh-DDS Bridge for Unitree G1"
    echo ""
    echo "Usage: $0 <mode> [options]"
    echo ""
    echo "Modes:"
    echo "  robot       Run on the robot (G1) - listens for operator connections"
    echo "  operator    Run on operator workstation - connects to robot"
    echo "  status      Check bridge status"
    echo "  stop        Stop running bridges"
    echo ""
    echo "Options:"
    echo "  --robot-ip <ip>    Robot IP address (required for operator mode)"
    echo "  --port <port>      Zenoh port (default: 7447)"
    echo "  --rest-port <port> REST API port for monitoring (default: 8000)"
    echo "  --background       Run in background"
    echo "  --allow <regex>    Only bridge matching topics (e.g., 'LowState|SportModeState')"
    echo "  --max-freq <hz>    Limit topic frequency (e.g., '.*=50' for 50Hz max)"
    echo ""
    echo "Examples:"
    echo "  # On robot:"
    echo "  $0 robot --background"
    echo ""
    echo "  # On operator workstation:"
    echo "  $0 operator --robot-ip 192.168.123.164"
    echo ""
    echo "  # Check what's being bridged:"
    echo "  curl http://localhost:8000/@/*/dds/route/**"
}

check_bridge() {
    if [ ! -f "$ZENOH_BRIDGE" ]; then
        echo -e "${RED}Error: zenoh-bridge-dds not found at $ZENOH_BRIDGE${NC}"
        echo "Build it with:"
        echo "  cd /home/k/zenoh-plugin-dds"
        echo "  cargo build --release -p zenoh-bridge-dds"
        exit 1
    fi
}

run_robot_mode() {
    local PORT="${1:-7447}"
    local REST_PORT="${2:-8000}"
    local BACKGROUND="${3:-false}"
    local ALLOW="${4:-}"
    local MAX_FREQ="${5:-}"

    echo -e "${GREEN}Starting Zenoh-DDS bridge on ROBOT...${NC}"
    echo "  Listening on: tcp/0.0.0.0:$PORT"
    echo "  REST API on:  http://0.0.0.0:$REST_PORT"

    local CMD="$ZENOH_BRIDGE \
        -m peer \
        -l tcp/0.0.0.0:$PORT \
        -d 0 \
        --rest-http-port $REST_PORT \
        -s 'robot/g1'"

    if [ -n "$ALLOW" ]; then
        CMD="$CMD -a '$ALLOW'"
        echo "  Allowing topics: $ALLOW"
    fi

    if [ -n "$MAX_FREQ" ]; then
        CMD="$CMD --max-frequency '$MAX_FREQ'"
        echo "  Max frequency: $MAX_FREQ"
    fi

    echo ""
    echo -e "${YELLOW}Operator should connect with:${NC}"
    echo "  $0 operator --robot-ip <this-machine-ip>"
    echo ""

    if [ "$BACKGROUND" = "true" ]; then
        echo "Running in background..."
        eval "nohup $CMD > /tmp/zenoh-bridge-robot.log 2>&1 &"
        echo "PID: $!"
        echo "Log: /tmp/zenoh-bridge-robot.log"
    else
        eval "$CMD"
    fi
}

run_operator_mode() {
    local ROBOT_IP="$1"
    local PORT="${2:-7447}"
    local REST_PORT="${3:-8001}"
    local BACKGROUND="${4:-false}"
    local ALLOW="${5:-}"
    local MAX_FREQ="${6:-}"

    if [ -z "$ROBOT_IP" ]; then
        echo -e "${RED}Error: --robot-ip is required for operator mode${NC}"
        print_usage
        exit 1
    fi

    echo -e "${GREEN}Starting Zenoh-DDS bridge on OPERATOR...${NC}"
    echo "  Connecting to robot: tcp/$ROBOT_IP:$PORT"
    echo "  REST API on: http://localhost:$REST_PORT"

    local CMD="$ZENOH_BRIDGE \
        -m client \
        -e tcp/$ROBOT_IP:$PORT \
        --rest-http-port $REST_PORT \
        -s 'operator/ws'"

    if [ -n "$ALLOW" ]; then
        CMD="$CMD -a '$ALLOW'"
        echo "  Allowing topics: $ALLOW"
    fi

    if [ -n "$MAX_FREQ" ]; then
        CMD="$CMD --max-frequency '$MAX_FREQ'"
        echo "  Max frequency: $MAX_FREQ"
    fi

    echo ""
    echo -e "${YELLOW}To check bridged topics:${NC}"
    echo "  curl http://localhost:$REST_PORT/@/*/dds/route/**"
    echo ""

    if [ "$BACKGROUND" = "true" ]; then
        echo "Running in background..."
        eval "nohup $CMD > /tmp/zenoh-bridge-operator.log 2>&1 &"
        echo "PID: $!"
        echo "Log: /tmp/zenoh-bridge-operator.log"
    else
        eval "$CMD"
    fi
}

check_status() {
    echo "Checking Zenoh bridge status..."

    # Check for running processes
    local PIDS=$(pgrep -f zenoh-bridge-dds || true)
    if [ -n "$PIDS" ]; then
        echo -e "${GREEN}Bridge processes running:${NC}"
        ps -p $PIDS -o pid,args
    else
        echo -e "${YELLOW}No bridge processes running${NC}"
    fi

    # Try REST API
    for PORT in 8000 8001; do
        if curl -s "http://localhost:$PORT/@/*/dds/version" > /dev/null 2>&1; then
            echo ""
            echo -e "${GREEN}REST API available on port $PORT:${NC}"
            echo "Routes:"
            curl -s "http://localhost:$PORT/@/*/dds/route/**" 2>/dev/null | head -20
        fi
    done
}

stop_bridges() {
    echo "Stopping Zenoh bridges..."
    pkill -f zenoh-bridge-dds || true
    echo "Done"
}

# Parse arguments
MODE=""
ROBOT_IP=""
PORT="7447"
REST_PORT=""
BACKGROUND="false"
ALLOW=""
MAX_FREQ=""

while [[ $# -gt 0 ]]; do
    case $1 in
        robot|operator|status|stop)
            MODE="$1"
            shift
            ;;
        --robot-ip)
            ROBOT_IP="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --rest-port)
            REST_PORT="$2"
            shift 2
            ;;
        --background)
            BACKGROUND="true"
            shift
            ;;
        --allow)
            ALLOW="$2"
            shift 2
            ;;
        --max-freq)
            MAX_FREQ="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            print_usage
            exit 1
            ;;
    esac
done

if [ -z "$MODE" ]; then
    print_usage
    exit 1
fi

case $MODE in
    robot)
        check_bridge
        REST_PORT="${REST_PORT:-8000}"
        run_robot_mode "$PORT" "$REST_PORT" "$BACKGROUND" "$ALLOW" "$MAX_FREQ"
        ;;
    operator)
        check_bridge
        REST_PORT="${REST_PORT:-8001}"
        run_operator_mode "$ROBOT_IP" "$PORT" "$REST_PORT" "$BACKGROUND" "$ALLOW" "$MAX_FREQ"
        ;;
    status)
        check_status
        ;;
    stop)
        stop_bridges
        ;;
esac
