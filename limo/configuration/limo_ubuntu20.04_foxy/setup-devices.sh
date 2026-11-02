#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_FILE="$SCRIPT_DIR/99-limo-hardware.rules"

if [[ "$(id -u)" == 0 ]]; then
  echo "Run as the normal robot user, without sudo in front of the command." >&2
  exit 1
fi

if ! command -v sudo >/dev/null 2>&1 ||
   ! command -v udevadm >/dev/null 2>&1; then
  echo "Missing required command: sudo or udevadm." >&2
  exit 1
fi

if [[ ! -f "$RULES_FILE" ]]; then
  echo "Missing udev rules file: $RULES_FILE" >&2
  exit 1
fi

sudo -v
sudo usermod -aG dialout,video "$(id -un)"
sudo install -m 0644 "$RULES_FILE" \
  /etc/udev/rules.d/99-limo-hardware.rules

sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
sudo udevadm trigger --action=add --subsystem-match=usb
sudo udevadm settle

echo "Log out and back in to apply group changes."
echo "Reconnect the USB lidar and camera."
echo "Check that /dev/ydlidar identifies your lidar before launching the robot."