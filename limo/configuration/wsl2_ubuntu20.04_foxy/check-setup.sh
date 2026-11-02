#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Check that the script runs inside WSL2.
if ! grep -qi microsoft /proc/sys/kernel/osrelease; then
    echo "Error: run this script from Ubuntu under WSL2." >&2
    exit 1
fi

if [[ "$(id -u)" == 0 ]]; then
    echo "Error: run this script as a normal WSL user, not as root." >&2
    exit 1
fi

# Check Docker and Docker Compose.
if ! command -v docker >/dev/null 2>&1 \
    || ! docker compose version >/dev/null 2>&1; then
    echo "Error: Docker or Docker Compose is unavailable." >&2
    echo "Enable this Ubuntu distribution in Docker Desktop > Settings > Resources > WSL Integration." >&2
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "Error: Docker is not accessible." >&2
    echo "Start Docker Desktop and check WSL Integration." >&2
    exit 1
fi

if [[ "$(docker info --format '{{.OSType}}')" != "linux" ]]; then
    echo "Error: Docker must use Linux containers." >&2
    exit 1
fi

# Check the project files.
if [[ ! -f "${SCRIPT_DIR}/Dockerfile" \
    || ! -f "${SCRIPT_DIR}/compose.yaml" ]]; then
    echo "Error: Docker configuration files are missing." >&2
    exit 1
fi

if [[ ! -d "${REPO_ROOT}/ros2_foxy/limo_ros2" \
    || ! -f "${REPO_ROOT}/ros2_foxy/limo_ros2/limo_base/package.xml" ]]; then
    echo "Error: LIMO ROS 2 Foxy packages are missing." >&2
    echo "Check the Git sparse-checkout configuration." >&2
    exit 1
fi

# WSLg is needed for graphical applications such as Gazebo and RViz.
if [[ ! -d /mnt/wslg/.X11-unix ]]; then
    echo "Warning: WSLg was not detected. Gazebo and RViz may not display correctly."
    echo "On Windows, run 'wsl --update' and restart WSL if necessary."
fi

echo
echo "Environment checks passed."
docker --version
docker compose version
echo "Repository: ${REPO_ROOT}"