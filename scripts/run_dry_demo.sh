#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; [[ -f "$PROJECT_DIR/.venv/bin/activate" ]] || { echo "Run scripts/install.sh first"; exit 1; }
source /opt/ros/humble/setup.bash; source "$HOME/panda_ros2_ws/install/setup.bash"; source "$PROJECT_DIR/.venv/bin/activate"
exec python "$PROJECT_DIR/app/body_pose_hand_control.py" --dry-run --config "$PROJECT_DIR/config/body_control.json"
