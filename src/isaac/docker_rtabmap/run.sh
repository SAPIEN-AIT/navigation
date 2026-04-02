#!/usr/bin/env bash
set -euo pipefail

xhost +local:docker 2>/dev/null || true

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ISAAC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

docker run -it --gpus all --privileged \
  -v /dev:/dev \
  -v "$ISAAC_DIR":/home/xplore/dev_ws/src \
  -v /usr/local/zed/settings:/usr/local/zed/settings:ro \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  minartabmap