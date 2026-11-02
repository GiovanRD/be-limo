#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${HOME}/ros2_ws"
SOURCE="${WORKSPACE}/src"

manifest_dir="$(mktemp -d)"
trap 'rm -rf "$manifest_dir"' EXIT

# Collect package manifests, including ignored packages.
collect-manifests "${SOURCE}" "${manifest_dir}"

# Update rosdep database.
rosdep update --rosdistro foxy --include-eol-distros

# Install missing system dependencies.
sudo apt-get update
sudo rosdep install \
    --from-paths "${manifest_dir}" \
    --ignore-src \
    -y \
    --rosdistro foxy