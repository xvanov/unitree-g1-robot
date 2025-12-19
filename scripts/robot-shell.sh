#!/bin/bash
# robot-shell.sh - Quick SSH access to G1 robot
# Story 1-6: Dual-Environment Deployment
#
# Usage:
#   ./scripts/robot-shell.sh              # Open interactive shell
#   ./scripts/robot-shell.sh "command"    # Run single command
#
# Environment:
#   ROBOT_IP   - Override default robot IP (default: 192.168.123.164)

set -e

# Robot connection - support environment variable override
ROBOT_IP="${ROBOT_IP:-192.168.123.164}"
ROBOT_USER="${ROBOT_USER:-unitree}"
ROBOT_PASS="${ROBOT_PASS:-123}"

# Check if SSH key auth is available
has_ssh_key() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_IP" "exit" 2>/dev/null
}

if [ $# -eq 0 ]; then
    # Interactive shell
    echo "Connecting to $ROBOT_USER@$ROBOT_IP..."

    if has_ssh_key; then
        ssh "$ROBOT_USER@$ROBOT_IP"
    else
        if command -v sshpass &>/dev/null; then
            sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP"
        else
            echo "sshpass not installed, using regular SSH (password: $ROBOT_PASS)"
            ssh "$ROBOT_USER@$ROBOT_IP"
        fi
    fi
else
    # Run command
    if has_ssh_key; then
        ssh "$ROBOT_USER@$ROBOT_IP" "$@"
    else
        if command -v sshpass &>/dev/null; then
            sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" "$@"
        else
            ssh "$ROBOT_USER@$ROBOT_IP" "$@"
        fi
    fi
fi
