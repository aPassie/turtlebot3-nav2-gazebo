#!/usr/bin/env bash
# Start the container with Gazebo/RViz shown on your X server.
# ./docker/run.sh [command...]   (no command = full demo)
set -euo pipefail

IMAGE=${IMAGE:-tb3-nav2:latest}

# let the container use the X server
xhost +local:root >/dev/null 2>&1 || echo "warning: xhost not available; GUI may fail"

source "$(dirname "$0")/gpu_args.sh"

TTY_ARGS=()
[[ -t 0 && -t 1 ]] && TTY_ARGS=(-it)

if [[ $# -eq 0 ]]; then
  set -- ros2 launch tb3_nav2_demo demo.launch.py
fi

exec docker run --rm --init "${TTY_ARGS[@]}" \
  --name tb3_nav2 \
  --ipc=host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_QPA_PLATFORM=xcb \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${GPU_ARGS[@]}" \
  "$IMAGE" "$@"
