#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SOURCE_DIR="$REPO_ROOT/ros2_foxy"
WORKSPACE_DIR="${LIMO_WS:-$HOME/ros2_ws}"

SDK_SOURCE_DIR="$SOURCE_DIR/YDLidar-SDK"
SDK_BUILD_DIR="$WORKSPACE_DIR/build/ydlidar_sdk"
SDK_INSTALL_DIR="$WORKSPACE_DIR/install/ydlidar_sdk"

OPENNI2_URL="https://dl.orbbec3d.com/dist/openni2/ROS2/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux.tar.gz"

with_camera=1
case "${1:-}" in
  "") ;;
  --without-camera) with_camera=0 ;;
  *)
    echo "Usage: bash build-robot.sh [--without-camera]" >&2
    exit 2
    ;;
esac

if [[ $# -gt 1 ]]; then
  echo "Usage: bash build-robot.sh [--without-camera]" >&2
  exit 2
fi

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ROS 2 Foxy is missing. Run bash install-deps.sh first." >&2
  exit 1
fi

if [[ ! -f "$SDK_SOURCE_DIR/CMakeLists.txt" ]]; then
  echo "Missing YDLidar SDK sources: $SDK_SOURCE_DIR" >&2
  exit 1
fi

if [[ ! -d "$WORKSPACE_DIR/src" ]]; then
  echo "Missing workspace source directory: $WORKSPACE_DIR/src" >&2
  echo "Run prepare-workspace.py first to create the symbolic links." >&2
  exit 1
fi

# Download and prepare the Orbbec OpenNI2 runtime only when needed.
download_openni_runtime() (
  set -euo pipefail

  local redist="$1"
  local platform="$2"
  local temp_dir archive extracted runtime_root

  if ! command -v curl >/dev/null 2>&1; then
    echo "curl is required to download the Orbbec OpenNI2 runtime." >&2
    echo "Run bash install-deps.sh first." >&2
    exit 1
  fi

  temp_dir="$(mktemp -d)"
  trap 'rm -rf "$temp_dir"' EXIT

  archive="$temp_dir/openni-sdk.tar.gz"
  extracted="$temp_dir/extracted"
  mkdir -p "$extracted"

  echo "Downloading the Orbbec OpenNI2 runtime for $platform..."

  curl --fail --location --retry 3 --connect-timeout 20 \
    "$OPENNI2_URL" \
    --output "$archive"

  # Reject archive paths that could escape the temporary extraction directory.
  if ! tar -tzf "$archive" >/dev/null; then
    echo "The downloaded OpenNI2 archive is invalid." >&2
    exit 1
  fi

  if tar -tzf "$archive" |
      grep -Eq '(^/|(^|/)\.\.(/|$))'; then
    echo "Unsafe path found in the downloaded OpenNI2 archive." >&2
    exit 1
  fi

  tar -xzf "$archive" -C "$extracted"

  # Find a complete runtime whose shared library matches the target CPU.
  if ! runtime_root="$(
    python3 - "$extracted" "$platform" <<'PY'
import os
import sys

base, platform = sys.argv[1:3]
expected_machine = {"x64": 62, "arm64": 183}[platform]

for current, _, files in os.walk(base):
    if "libOpenNI2.so" not in files:
        continue

    library = os.path.join(current, "libOpenNI2.so")

    try:
        with open(library, "rb") as stream:
            header = stream.read(20)

        if len(header) < 20 or header[:4] != b"\x7fELF":
            continue

        # OpenNI2 runtime libraries must be 64-bit, little-endian ELF files.
        if header[4] != 2 or header[5] != 1:
            continue

        machine = int.from_bytes(header[18:20], "little")
        if machine != expected_machine:
            continue

        # The expected runtime layout contains OpenNI.ini and driver libraries.
        root = current
        for _ in range(5):
            ini = os.path.join(root, "OpenNI.ini")
            drivers = os.path.join(root, "OpenNI2", "Drivers")

            if os.path.isfile(ini) and os.path.isdir(drivers):
                if any(name.endswith(".so") for name in os.listdir(drivers)):
                    print(root)
                    sys.exit(0)

            parent = os.path.dirname(root)
            if parent == root:
                break
            root = parent

    except OSError:
        continue

sys.exit(1)
PY
  )"; then
    echo "Could not find a complete OpenNI2 runtime for $platform." >&2
    echo "The downloaded archive may have changed or may not support this architecture." >&2
    exit 1
  fi

  mkdir -p "$redist"
  cp -a "$runtime_root/." "$redist/"

  if [[ ! -f "$redist/libOpenNI2.so" ||
        ! -f "$redist/OpenNI.ini" ]] ||
     ! compgen -G "$redist/OpenNI2/Drivers/*.so" >/dev/null; then
    echo "OpenNI2 runtime validation failed after installation: $redist" >&2
    exit 1
  fi

  echo "OpenNI2 runtime installed and validated: $redist"
)

if [[ "$with_camera" == 1 ]]; then
  case "$(dpkg --print-architecture)" in
    arm64) camera_platform=arm64 ;;
    amd64) camera_platform=x64 ;;
    *)
      echo "Unsupported camera architecture." >&2
      exit 1
      ;;
  esac

  redist="$SOURCE_DIR/ros2_astra_camera/astra_camera/openni2_redist/$camera_platform"

  if [[ ! -f "$redist/libOpenNI2.so" ||
        ! -f "$redist/OpenNI.ini" ]] ||
     ! compgen -G "$redist/OpenNI2/Drivers/*.so" >/dev/null; then
    download_openni_runtime "$redist" "$camera_platform"
  else
    echo "OpenNI2 runtime already present: $redist"
  fi
fi

package_paths=(
  "$WORKSPACE_DIR/src/limo_ros2/limo_msgs"
  "$WORKSPACE_DIR/src/limo_ros2/limo_base"
  "$WORKSPACE_DIR/src/limo_ros2/limo_bringup"
  "$WORKSPACE_DIR/src/limo_ros2/limo_description"
  "$WORKSPACE_DIR/src/limo_start"
  "$WORKSPACE_DIR/src/ydlidar_ros2_driver"
)

if [[ "$with_camera" == 1 ]]; then
  package_paths+=(
    "$WORKSPACE_DIR/src/astra_camera/astra_camera_msgs"
    "$WORKSPACE_DIR/src/astra_camera/astra_camera"
  )
fi

for package_path in "${package_paths[@]}"; do
  if [[ ! -f "$package_path/package.xml" ]]; then
    echo "Missing robot package manifest: $package_path/package.xml" >&2
    echo "Run prepare-workspace.py and check the source links." >&2
    exit 1
  fi

  if [[ -e "$package_path/COLCON_IGNORE" ||
        -L "$package_path/COLCON_IGNORE" ]]; then
    echo "COLCON_IGNORE found: $package_path/COLCON_IGNORE" >&2
    echo "Resolve this marker manually if appropriate." >&2
    exit 1
  fi
done

mkdir -p "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/install" "$WORKSPACE_DIR/log"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/foxy/setup.bash

cmake -S "$SDK_SOURCE_DIR" -B "$SDK_BUILD_DIR" \
  -DCMAKE_INSTALL_PREFIX="$SDK_INSTALL_DIR" \
  -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TEST=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=ON \
  -DSWIG_FOUND=FALSE

cmake --build "$SDK_BUILD_DIR" --parallel "${BUILD_JOBS:-2}"
cmake --install "$SDK_BUILD_DIR"

export CMAKE_PREFIX_PATH="$SDK_INSTALL_DIR${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"

cd "$WORKSPACE_DIR"

colcon --log-base "$WORKSPACE_DIR/log" build \
  --symlink-install \
  --base-paths "${package_paths[@]}" \
  --build-base "$WORKSPACE_DIR/build" \
  --install-base "$WORKSPACE_DIR/install" \
  --parallel-workers "${BUILD_JOBS:-2}" \
  --cmake-args -DBUILD_TESTING=OFF

printf '%s\n' "$with_camera" > "$WORKSPACE_DIR/install/limo-camera-enabled"

LIMO_WS="$WORKSPACE_DIR" bash "$SCRIPT_DIR/setup-shell.sh"

echo "Robot workspace built."