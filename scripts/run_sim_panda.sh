#!/usr/bin/env bash
# Starts ONLY a local ROS2+PyBullet Panda. No hardware/FCI/ethernet.
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'Missing ROS 2 Humble'; exit 1; }
[[ -f "$PROJECT_DIR/.venv/bin/activate" ]] || { echo 'Missing project .venv; existing dry-run must work first'; exit 1; }
source /opt/ros/humble/setup.bash
[[ ! -f "$HOME/panda_ros2_ws/install/setup.bash" ]] || source "$HOME/panda_ros2_ws/install/setup.bash"
source "$PROJECT_DIR/.venv/bin/activate"
export ROS_DOMAIN_ID=197
export ROS_LOCALHOST_ONLY=1
python -c 'import pybullet, rclpy; from franka_msgs.action import Move' || {
  echo 'Install PyBullet into the project environment: .venv/bin/python -m pip install pybullet'; exit 2;
}
exec python "$PROJECT_DIR/simulation/panda_pybullet_bridge.py"
