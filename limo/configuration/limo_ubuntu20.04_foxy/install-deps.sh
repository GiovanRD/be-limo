#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SOURCE_DIR="$REPO_ROOT/ros2_foxy"
WORKSPACE_DIR="${LIMO_WS:-$HOME/ros2_ws}"

if [[ "$(id -u)" == 0 ]]; then
  echo "Run as the normal robot user; the script requests sudo when needed." >&2
  exit 1
fi

. /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 20.04 ]]; then
  echo "This profile requires Ubuntu 20.04. Detected: $PRETTY_NAME" >&2
  exit 1
fi

case "$(dpkg --print-architecture)" in
  amd64|arm64) ;;
  *)
    echo "Supported architectures: amd64 and arm64." >&2
    exit 1
    ;;
esac

if [[ ! -d "$SOURCE_DIR" ]]; then
  echo "Missing ROS 2 Foxy sources: $SOURCE_DIR" >&2
  echo "Check that the repository contains ros2_foxy/." >&2
  exit 1
fi

temp_dir="$(mktemp -d)"
pin_path=""

cleanup() {
  if [[ -n "$pin_path" ]]; then
    sudo rm -f "$pin_path"
  fi
  rm -rf "$temp_dir"
}
trap cleanup EXIT

python3 "$SCRIPT_DIR/prepare-workspace.py" \
  "$SOURCE_DIR" \
  "$temp_dir/manifests" \
  --manifests-only

sudo -v

# Temporarily refuse new Gazebo packages, including transitive dependencies.
pin_path="$(sudo mktemp /etc/apt/preferences.d/limo-no-gazebo-XXXXXX)"
sudo install -m 0644 "$SCRIPT_DIR/no-gazebo.pref" "$pin_path"

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg2 locales software-properties-common

sudo add-apt-repository -y universe
sudo locale-gen en_US.UTF-8

curl -fsSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o "$temp_dir/ros.key"

sudo install -m 0644 "$temp_dir/ros.key" \
  /usr/share/keyrings/ros-archive-keyring.gpg

printf '%s\n' \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt-get update

mapfile -t packages < <(
  sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$SCRIPT_DIR/apt-packages.txt"
)

sudo apt-get install -y --no-install-recommends "${packages[@]}"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi

rosdep update --rosdistro foxy --include-eol-distros

rosdep install \
  --from-paths "$temp_dir/manifests" \
  --ignore-src \
  -y \
  --rosdistro foxy

echo "System dependencies installed."
echo "Next: bash setup-devices.sh, then bash build-robot.sh."
echo "The YDLidar SDK is built by build-robot.sh."