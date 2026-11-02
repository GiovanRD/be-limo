#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${HOME}/ros2_ws"
SOURCE_ROOT="/limo/ros2_foxy"

# Source ROS 2 Foxy.
set +u
source /opt/ros/foxy/setup.bash
set -u

# Create the workspace source directory.
mkdir -p "${WORKSPACE}/src"

# Link the ROS 2 packages into the workspace.
for package in limo_ros2 ros2_astra_camera ydlidar_ros2_driver; do
    SOURCE="${SOURCE_ROOT}/${package}"
    LINK="${WORKSPACE}/src/${package}"

    if [[ ! -d "${SOURCE}" ]]; then
        echo "Error: Package sources not found at ${SOURCE}." >&2
        exit 1
    fi

    if [[ -L "${LINK}" ]]; then
        if [[ "$(readlink -f "${LINK}")" != "$(readlink -f "${SOURCE}")" ]]; then
            echo "Error: ${LINK} points to an unexpected location." >&2
            exit 1
        fi
    elif [[ -e "${LINK}" ]]; then
        echo "Error: ${LINK} already exists and is not a symbolic link." >&2
        exit 1
    else
        ln -s "${SOURCE}" "${LINK}"
    fi
done

# Source the workspace overlay if it has been built.
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    set +u
    source "${WORKSPACE}/install/setup.bash"
    set -u
fi

exec "$@"