#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="${LIMO_WS:-$HOME/ros2_ws}"

if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ||
      ! -f "$WORKSPACE_DIR/install/limo-camera-enabled" ]]; then
  echo "Build the robot workspace first: bash build-robot.sh [--without-camera]." >&2
  exit 1
fi

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ROS 2 Foxy is missing." >&2
  exit 1
fi

source /opt/ros/foxy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

camera_args=()
if [[ "$(cat "$WORKSPACE_DIR/install/limo-camera-enabled")" == 0 ]]; then
  camera_args+=(use_camera:=false)
fi

exec ros2 launch limo_start limo_real.launch.py \
  "${camera_args[@]}" "$@"