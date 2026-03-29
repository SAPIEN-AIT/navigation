#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"
CALIB_SRC="$WS/src/zed2i_isaac_vslam/config/SN38462249.conf"
CALIB_DST="/usr/local/zed/settings/SN38462249.conf"
CAMERA_NAME="${CAMERA_NAME:-zed2i}"
ZED_NODE_NAME="${ZED_NODE_NAME:-zed_node}"
LAUNCH_ARGS=()

for arg in "$@"; do
  case "$arg" in
    camera_name:=*) CAMERA_NAME="${arg#camera_name:=}" ;;
    zed_node_name:=*) ZED_NODE_NAME="${arg#zed_node_name:=}" ;;
  esac
  LAUNCH_ARGS+=("$arg")
done

if [ ! -f "$CALIB_SRC" ]; then
  printf 'Missing calibration file: %s\n' "$CALIB_SRC" >&2
  exit 1
fi

sudo mkdir -p /usr/local/zed/settings
sudo ln -sfn "$CALIB_SRC" "$CALIB_DST"

set +u
source /opt/ros/humble/setup.bash

if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash" || true
fi

if [ -f "$WS/src/install/setup.bash" ]; then
  source "$WS/src/install/setup.bash" || true
fi

set -u

if ! ros2 pkg prefix zed_wrapper >/dev/null 2>&1; then
  printf 'Could not find the `zed_wrapper` package. Build the workspace first.\n' >&2
  exit 1
fi

RVIZ_CONFIG="$WS/src/zed2i_isaac_vslam/rviz/zed2i_pointcloud.rviz"

printf 'Launching ZED2i point cloud viewer (camera_name=%s, node=%s)...\n' "$CAMERA_NAME" "$ZED_NODE_NAME"
printf 'Point cloud topic: /%s/%s/point_cloud/cloud_registered\n' "$CAMERA_NAME" "$ZED_NODE_NAME"

ros2 launch zed_wrapper zed_camera.launch.py \
  camera_name:="$CAMERA_NAME" \
  camera_model:=zed2i \
  ros_params_override_path:="$WS/src/zed2i_isaac_vslam/config/zed2i_pointcloud.yaml" \
  publish_urdf:=true \
  publish_tf:=true \
  publish_map_tf:=false \
  publish_imu_tf:=true \
  rviz:=true \
  rviz_config:="$RVIZ_CONFIG" \
  "${LAUNCH_ARGS[@]}"
