#!/usr/bin/env bash
set -euo pipefail

xhost +local:docker 2>/dev/null || true

docker run -it --gpus all --privileged \
  -v /dev:/dev \
  -v "$(cd "$(dirname "$0")/.." && pwd)":/home/xplore/dev_ws/src \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  minartabmap