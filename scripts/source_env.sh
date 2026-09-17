#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/panda_ros2_ws/install/setup.bash"
export LD_LIBRARY_PATH="$HOME/libfranka/build:${LD_LIBRARY_PATH:-}"
[[ -f "$HOME/panda_body_servo_ws/install/setup.bash" ]] && source "$HOME/panda_body_servo_ws/install/setup.bash"
