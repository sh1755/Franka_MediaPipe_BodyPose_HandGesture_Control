#!/usr/bin/env bash
# One command: PyBullet robot + MediaPipe camera, on an isolated localhost ROS domain.
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$PROJECT_DIR/scripts/run_sim_panda.sh" &
SIM_PID=$!
cleanup() { kill "$SIM_PID" 2>/dev/null || true; wait "$SIM_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sleep 4
if ! kill -0 "$SIM_PID" 2>/dev/null; then
  echo 'PyBullet simulator failed to start. Try scripts/run_sim_panda.sh to see why.'
  exit 1
fi
"$PROJECT_DIR/scripts/run_sim_camera.sh"
