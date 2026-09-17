#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${CONDA_PREFIX:-}" ]]; then echo "ERROR: Conda active. Run conda deactivate first."; exit 2; fi
[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS 2 Humble not found"; exit 1; }
[[ -f "$HOME/panda_ros2_ws/install/setup.bash" ]] || { echo "~/panda_ros2_ws not found/built"; exit 1; }
source /opt/ros/humble/setup.bash
source "$HOME/panda_ros2_ws/install/setup.bash"
export LD_LIBRARY_PATH="$HOME/libfranka/build:${LD_LIBRARY_PATH:-}"
if ros2 pkg prefix moveit_servo >/dev/null 2>&1; then echo "MoveIt Servo: FOUND ($(ros2 pkg prefix moveit_servo))"; else echo "WARNING: moveit_servo not found. Installer will not run apt automatically."; fi
VENV="$PROJECT_DIR/.venv"; rm -rf "$VENV"; /usr/bin/python3 -m venv --system-site-packages "$VENV"; source "$VENV/bin/activate"
python -m pip install --upgrade pip wheel
python -m pip install -r "$PROJECT_DIR/requirements.txt"
python - <<'PYIMPORT'
import cv2, mediapipe, rclpy
from geometry_msgs.msg import TwistStamped
from franka_msgs.action import Move
print('OpenCV:', cv2.__version__); print('MediaPipe:', mediapipe.__version__); print('ROS Python imports: OK')
PYIMPORT
deactivate
WS="$HOME/panda_body_servo_ws"; mkdir -p "$WS/src"; rm -rf "$WS/src/franka_body_servo_support"; ln -s "$PROJECT_DIR/ros2_ws/src/franka_body_servo_support" "$WS/src/franka_body_servo_support"
cd "$WS"; source /opt/ros/humble/setup.bash; source "$HOME/panda_ros2_ws/install/setup.bash"; colcon build --symlink-install --packages-select franka_body_servo_support
chmod +x "$PROJECT_DIR/app/body_pose_hand_control.py" "$PROJECT_DIR/scripts/"*.sh
echo "Installation complete. Run: bash scripts/run_dry_demo.sh"
