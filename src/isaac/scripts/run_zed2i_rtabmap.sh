#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────
# RTABMap SLAM with ZED2i — mapping + localization/navigation
#
# Usage:
#   ./run_zed2i_rtabmap.sh                    # Mapping mode (default)
#   ./run_zed2i_rtabmap.sh mapping            # Mapping mode
#   ./run_zed2i_rtabmap.sh nav /path/map.yaml # Localization + Nav2
#
# After mapping, save the 2D map:
#   ros2 run nav2_map_server map_saver_cli -f ~/my_map
#
# Then navigate:
#   ./run_zed2i_rtabmap.sh nav ~/my_map.yaml
# ─────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source ROS2 workspace
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
elif [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

# ── Check rtabmap_ros is installed ──
if ! ros2 pkg list 2>/dev/null | grep -q rtabmap_slam; then
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║  rtabmap_ros not found! Install it:                        ║"
    echo "║  sudo apt install ros-\${ROS_DISTRO}-rtabmap-ros            ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    exit 1
fi

MODE="${1:-mapping}"
MAP_FILE="${2:-}"

# ── ZED calibration symlink (same as other scripts) ──
CALIB_SRC="/usr/local/zed/settings/SN38462249.conf"
CALIB_DST="$HOME/.config/ros2/camera_info/SN38462249.conf"
if [ -f "$CALIB_SRC" ] && [ ! -f "$CALIB_DST" ]; then
    mkdir -p "$(dirname "$CALIB_DST")"
    ln -sf "$CALIB_SRC" "$CALIB_DST"
    echo "[rtabmap] Symlinked ZED calibration → $CALIB_DST"
fi

PKG_DIR="$SCRIPT_DIR/../src/zed2i_rtabmap"

case "$MODE" in
    mapping|map)
        echo "╔══════════════════════════════════════════════════════════════╗"
        echo "║  RTABMap MAPPING mode                                      ║"
        echo "║  Move around to build the map.                             ║"
        echo "║  When done, save with:                                     ║"
        echo "║    ros2 run nav2_map_server map_saver_cli -f ~/my_map      ║"
        echo "╚══════════════════════════════════════════════════════════════╝"

        ros2 launch zed2i_rtabmap zed2i_rtabmap_mapping.launch.py \
            use_rviz:=true \
            "$@"
        ;;

    nav|navigation|loc|localization)
        if [ -z "$MAP_FILE" ]; then
            echo "Error: navigation mode requires a map file."
            echo "Usage: $0 nav /path/to/my_map.yaml"
            exit 1
        fi

        if [ ! -f "$MAP_FILE" ]; then
            echo "Error: map file not found: $MAP_FILE"
            exit 1
        fi

        echo "╔══════════════════════════════════════════════════════════════╗"
        echo "║  RTABMap LOCALIZATION + Nav2 mode                          ║"
        echo "║  Map: $MAP_FILE"
        echo "║  Use RViz '2D Goal Pose' to send navigation goals.        ║"
        echo "╚══════════════════════════════════════════════════════════════╝"

        ros2 launch zed2i_rtabmap zed2i_rtabmap_localization.launch.py \
            map:="$MAP_FILE" \
            use_rviz:=true \
            "${@:3}"
        ;;

    *)
        echo "Unknown mode: $MODE"
        echo "Usage: $0 [mapping|nav] [map_file.yaml]"
        exit 1
        ;;
esac
