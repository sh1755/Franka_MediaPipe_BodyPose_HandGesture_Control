#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; [[ -f "$PROJECT_DIR/.venv/bin/activate" ]] || { echo "Run scripts/install.sh first"; exit 1; }
source "$PROJECT_DIR/scripts/source_env.sh"; source "$PROJECT_DIR/.venv/bin/activate"
echo "Starts with arm/gripper OFF. C=calibrate, G=gripper, A=arm; left hand up is deadman."
exec python "$PROJECT_DIR/app/body_pose_hand_control.py" --config "$PROJECT_DIR/config/body_control.json"
