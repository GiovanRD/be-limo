#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SOURCE_ROOT="$REPO_ROOT/ros2_foxy"
WORKSPACE_DIR="${LIMO_WS:-$HOME/ros2_ws}"
BUILD_JOBS="${BUILD_JOBS:-2}"

no_build=0
case "${1:-}" in
  "") ;;
  --no-build) no_build=1 ;;
  *)
    echo "Usage: bash setup.sh [--no-build]" >&2
    exit 2
    ;;
esac

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ROS 2 Foxy is missing. Install it before running this script." >&2
  exit 1
fi

for path in \
  "$SOURCE_ROOT/limo_ros2/limo_car/package.xml" \
  "$SOURCE_ROOT/ros2_astra_camera" \
  "$SOURCE_ROOT/ydlidar_ros2_driver/package.xml" \
  "$SOURCE_ROOT/YDLidar-SDK/CMakeLists.txt"
do
  if [[ ! -e "$path" ]]; then
    echo "Required source path not found: $path" >&2
    exit 1
  fi
done

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  ca-certificates \
  cmake \
  git \
  libboost-program-options-dev \
  libeigen3-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  libgl1-mesa-dri \
  libgl1-mesa-glx \
  libglu1-mesa \
  libopencv-dev \
  libturbojpeg0-dev \
  libudev-dev \
  libusb-1.0-0-dev \
  libuvc-dev \
  pkg-config \
  python3-colcon-common-extensions \
  python3-dev \
  python3-matplotlib \
  python3-numpy \
  python3-opencv \
  python3-pip \
  python3-pyqt5 \
  python3-pytest \
  python3-rosdep \
  python3-scipy \
  python3-setuptools \
  python3-vcstool \
  python3-yaml \
  ros-foxy-camera-info-manager \
  ros-foxy-cartographer-ros \
  ros-foxy-cv-bridge \
  ros-foxy-desktop \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-image-geometry \
  ros-foxy-image-publisher \
  ros-foxy-image-transport \
  ros-foxy-joint-state-publisher-gui \
  ros-foxy-nav2-bringup \
  ros-foxy-robot-localization \
  ros-foxy-rqt-robot-steering \
  ros-foxy-slam-toolbox \
  ros-foxy-teleop-twist-keyboard \
  ros-foxy-tf2-eigen \
  ros-foxy-tf2-sensor-msgs \
  ros-foxy-twist-mux \
  ros-foxy-xacro

if ! rosdep db >/dev/null 2>&1; then
  sudo rosdep init 2>/dev/null || true
fi
rosdep update --rosdistro foxy --include-eol-distros

mkdir -p "$WORKSPACE_DIR/src"

link_source() {
  local source="$1"
  local target="$2"

  if [[ -e "$target" && ! -L "$target" ]]; then
    echo "Refusing to overwrite existing path: $target" >&2
    exit 1
  fi

  if [[ -L "$target" && "$(readlink -f "$target")" != "$(readlink -f "$source")" ]]; then
    echo "Error: $target points to an unexpected location." >&2
    exit 1
  fi

  if [[ ! -L "$target" ]]; then
    ln -s "$source" "$target"
  fi
}

link_source "$SOURCE_ROOT/limo_ros2" \
  "$WORKSPACE_DIR/src/limo_ros2"
link_source "$SOURCE_ROOT/ros2_astra_camera" \
  "$WORKSPACE_DIR/src/ros2_astra_camera"
link_source "$SOURCE_ROOT/ydlidar_ros2_driver" \
  "$WORKSPACE_DIR/src/ydlidar_ros2_driver"

# Collect all package manifests, including ignored packages.
MANIFEST_DIR="$(mktemp -d)"
trap 'rm -rf "$MANIFEST_DIR"' EXIT

python3 "$REPO_ROOT/configuration/native_ubuntu20.04_foxy/collect-manifests.py" \
  "$WORKSPACE_DIR/src" "$MANIFEST_DIR"

set +u
source /opt/ros/foxy/setup.bash
set -u

rosdep install \
  --from-paths "$MANIFEST_DIR" \
  --ignore-src \
  -y \
  --rosdistro foxy

if [[ "$no_build" == 1 ]]; then
  echo "Workspace prepared. Build it with: bash '$SCRIPT_DIR/setup.sh'"
  exit 0
fi

SDK_BUILD_DIR="$WORKSPACE_DIR/build/ydlidar_sdk"

cmake -S "$SOURCE_ROOT/YDLidar-SDK" -B "$SDK_BUILD_DIR" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TEST=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3

cmake --build "$SDK_BUILD_DIR" --parallel "$BUILD_JOBS"
sudo cmake --install "$SDK_BUILD_DIR"
sudo ldconfig

cd "$WORKSPACE_DIR"
colcon build \
  --symlink-install \
  --base-paths \
    "$WORKSPACE_DIR/src/limo_ros2" \
    "$WORKSPACE_DIR/src/ros2_astra_camera" \
    "$WORKSPACE_DIR/src/ydlidar_ros2_driver" \
  --parallel-workers "$BUILD_JOBS" \
  --cmake-args -DBUILD_TESTING=OFF

echo "Build complete."
echo "In a new terminal, source:"
echo "  source /opt/ros/foxy/setup.bash"
echo "  source '$WORKSPACE_DIR/install/setup.bash'"