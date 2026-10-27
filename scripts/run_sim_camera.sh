#!/usr/bin/env bash
# Existing MediaPipe app, routed ONLY to the local isolated PyBullet ROS domain.
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f "$PROJECT_DIR/.venv/bin/activate" ]] || { echo 'Missing project .venv'; exit 1; }
source /opt/ros/humble/setup.bash
[[ ! -f "$HOME/panda_ros2_ws/install/setup.bash" ]] || source "$HOME/panda_ros2_ws/install/setup.bash"
source "$PROJECT_DIR/.venv/bin/activate"
export ROS_DOMAIN_ID=197
export ROS_LOCALHOST_ONLY=1
# Do not use --dry-run here: publishing goes to isolated simulation, not a real robot.
exec python "$PROJECT_DIR/app/body_pose_hand_control.py" --config "$PROJECT_DIR/config/body_control.json"
