#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

docker build -t mina_nav2 -f "$SCRIPT_DIR/Dockerfile" "$SRC_DIR"
