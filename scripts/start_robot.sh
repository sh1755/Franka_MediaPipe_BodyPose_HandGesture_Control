#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; source "$PROJECT_DIR/scripts/source_env.sh"
if ps -ef | grep -E "franka_control2_node|ros2_control_node|move_group" | grep -v grep >/dev/null; then echo "ERROR: Existing Franka/MoveIt process found. Close it first."; exit 2; fi
ping -c 2 -W 1 172.16.0.2
echo "Confirm Franka Desk: recovered/unlocked, FCI active, no Reflex."
exec ros2 launch franka_moveit_config moveit_real_arm_platform.launch.py robot_ip:=172.16.0.2 load_camera:=false load_gripper:=true
