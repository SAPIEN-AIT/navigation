#!/usr/bin/env bash
# Run ZED2i 2D mapping: depth → virtual laser scan → slam_toolbox
# Launch AFTER run_zed2i_vslam.sh is already running.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"
LAUNCH_FILE="$WS/src/zed2i_isaac_vslam/launch/zed2i_mapping.launch.py"

set +u
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ];     then source "$WS/install/setup.bash"     || true; fi
if [ -f "$WS/src/install/setup.bash" ]; then source "$WS/src/install/setup.bash" || true; fi
set -u

# Install depthimage_to_laserscan if missing
if ! ros2 pkg prefix depthimage_to_laserscan >/dev/null 2>&1; then
  printf 'Installing depthimage_to_laserscan...\n'
  sudo apt-get install -y ros-humble-depthimage-to-laserscan
fi

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  printf 'slam_toolbox not found. Install it: sudo apt install ros-humble-slam-toolbox\n' >&2
  exit 1
fi

printf 'Launching ZED2i mapping (depth→scan→slam_toolbox)...\n'
printf 'Make sure run_zed2i_vslam.sh is running in another terminal.\n'
printf 'Map topic: /map\n'
printf 'Scan topic: /scan_virtual\n\n'

ros2 launch "$LAUNCH_FILE" "$@"
