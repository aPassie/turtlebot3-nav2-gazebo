#!/usr/bin/env bash
# Record the demo video without touching your desktop: Gazebo GUI + RViz run on
# a virtual X display inside the container and are captured with ffmpeg.
# Usage: ./tools/record_demo.sh [output.mp4] [loops]   (default: media/demo.mp4)
set -euo pipefail
cd "$(dirname "$0")/.."

OUT_FILE=${1:-media/demo.mp4}
LOOPS=${2:-1}
mkdir -p "$(dirname "$OUT_FILE")"
OUT_DIR=$(cd "$(dirname "$OUT_FILE")" && pwd)
IMAGE=${IMAGE:-tb3-nav2:latest}

source docker/gpu_args.sh

docker run --rm --init --ipc=host "${GPU_ARGS[@]}" -v "$OUT_DIR":/results \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" -e PREVIEW="${PREVIEW:-0}" \
  "$IMAGE" bash -c "
    \$(ros2 pkg prefix tb3_nav2_demo)/lib/tb3_nav2_demo/record_demo.sh /results/$(basename "$OUT_FILE") $LOOPS
    status=\$?
    chown -R \$HOST_UID:\$HOST_GID /results
    exit \$status"
