#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Load only explicit profile options; Compose itself reads .env for substitutions.
if [[ -f .env ]]; then
  while IFS= read -r line || [[ -n "$line" ]]; do
    line="${line%$'\r'}"
    case "$line" in
      ENABLE_GPU=0|ENABLE_GPU=1|ENABLE_HOST_NETWORK=0|ENABLE_HOST_NETWORK=1)
        export "$line"
        ;;
    esac
  done < .env
fi

export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

compose=(docker compose -f compose.yaml)
[[ "${ENABLE_GPU:-0}" != 1 ]] || compose+=(-f compose.gpu.yaml)
[[ "${ENABLE_HOST_NETWORK:-0}" != 1 ]] || compose+=(-f compose.host.yaml)

init_workspace() {
  "${compose[@]}" up -d
  "${compose[@]}" exec -T foxy bash -lc '
    mkdir -p "$HOME/ros2_ws/src"
    ln -sfn /limo/ros2_foxy/limo_ros2 "$HOME/ros2_ws/src/limo_ros2"
    ln -sfn /limo/ros2_foxy/ros2_astra_camera "$HOME/ros2_ws/src/ros2_astra_camera"
    ln -sfn /limo/ros2_foxy/ydlidar_ros2_driver "$HOME/ros2_ws/src/ydlidar_ros2_driver"
  '
}

usage() {
  echo "Usage: $0 {build|up|shell|build-workspace|deps|check|gpu|config|down}" >&2
}

action="${1:-shell}"
case "$action" in
  build|up|shell|build-workspace|deps|check|gpu|config|down) ;;
  *) usage; exit 2 ;;
esac

if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
  echo "Docker Desktop/Compose unavailable. Enable WSL Integration for this distribution." >&2
  exit 1
fi

if [[ "$action" == config ]]; then
  "${compose[@]}" config
  exit
fi

if ! docker info >/dev/null 2>&1; then
  echo "Docker is unavailable. Start Docker Desktop and check WSL Integration." >&2
  exit 1
fi

if [[ "$action" != build && "$action" != down ]]; then
  if [[ ! -d /mnt/wslg/.X11-unix ]]; then
    echo "WSLg unavailable. Run 'wsl --update' in Windows, then restart WSL." >&2
    exit 1
  fi
fi

case "$action" in
  build)
    "${compose[@]}" build
    ;;

  up)
    "${compose[@]}" up -d --build
    ;;

  shell)
    "${compose[@]}" up -d
    "${compose[@]}" exec foxy bash
    ;;

  deps)
    init_workspace
    "${compose[@]}" exec foxy workspace-deps
    ;;

  build-workspace)
    init_workspace
    "${compose[@]}" exec -T foxy bash -lc \
      "source /opt/ros/foxy/setup.bash && cd \"\$HOME/ros2_ws\" && colcon build --symlink-install --parallel-workers 2"
    ;;

  check)
    "${compose[@]}" up -d
    "${compose[@]}" exec -T foxy bash -lc '
      set -e
      source /opt/ros/foxy/setup.bash

      echo "ROS_DISTRO=$ROS_DISTRO"
      ros2 --help >/dev/null
      colcon version-check
      python3 -c "import rclpy; print(\"rclpy import OK\")"

      test -L "$HOME/ros2_ws/src/limo_ros2"
      test -d "$HOME/ros2_ws/src/limo_ros2"
      test -f "$HOME/ros2_ws/src/limo_ros2/limo_base/package.xml"

      echo "LIMO source link OK"
      colcon list --base-paths "$HOME/ros2_ws/src"
    '
    ;;

  gpu)
    if [[ "${ENABLE_GPU:-0}" != 1 ]]; then
      echo "Set ENABLE_GPU=1 in .env before using gpu (requires NVIDIA)." >&2
      exit 1
    fi
    "${compose[@]}" up -d
    "${compose[@]}" exec -T foxy nvidia-smi
    ;;

  down)
    "${compose[@]}" down
    ;;
esac