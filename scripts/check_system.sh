#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; source "$PROJECT_DIR/scripts/source_env.sh"
echo "=== Network ==="; ping -c 1 -W 1 172.16.0.2 || true
echo "=== Controllers ==="; ros2 control list_controllers || true
echo "=== Gripper actions ==="; ros2 action list | grep panda_gripper || true
echo "=== Servo services ==="; ros2 service list | grep servo_node || true
echo "=== Servo command topic ==="; ros2 topic info /servo_node/delta_twist_cmds --verbose || true
echo "=== Joint state sample ==="; timeout 4 ros2 topic echo /joint_states --once || true
