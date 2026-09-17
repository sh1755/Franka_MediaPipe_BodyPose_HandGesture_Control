#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; source "$PROJECT_DIR/scripts/source_env.sh"
ros2 pkg prefix moveit_servo >/dev/null 2>&1 || { echo "ERROR: moveit_servo package not found"; exit 3; }
exec ros2 launch franka_body_servo_support servo_real_panda.launch.py robot_ip:=172.16.0.2
