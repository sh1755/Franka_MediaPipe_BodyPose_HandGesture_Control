#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; source "$PROJECT_DIR/scripts/source_env.sh"
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger "{}" || true
