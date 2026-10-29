#!/usr/bin/env bash
# Run the demo container with GUI (Gazebo + RViz) forwarded to the host X server.
# Usage: ./docker/run.sh [command...]   (default: full demo launch)
set -euo pipefail

IMAGE=${IMAGE:-tb3-nav2:latest}

# Allow local containers to talk to the X server (XWayland works too).
xhost +local:root >/dev/null 2>&1 || echo "warning: xhost not available; GUI may fail"

GPU_ARGS=()
if docker info 2>/dev/null | grep -qi 'runtimes:.*nvidia'; then
  # NVIDIA Container Toolkit installed: use the proprietary driver.
  GPU_ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
else
  # Pass through only DRM nodes that Mesa can drive (Intel/AMD).
  # Exposing an NVIDIA node without the toolkit makes EGL pick it first,
  # fail, and crash Gazebo's Ogre2 renderer (segfault in gz sim).
  for node in /dev/dri/card* /dev/dri/renderD*; do
    [[ -e $node ]] || continue
    drv=$(basename "$(readlink -f "/sys/class/drm/$(basename "$node")/device/driver")" 2>/dev/null || true)
    [[ $drv == nvidia ]] && continue
    GPU_ARGS+=(--device "$node")
  done
fi

TTY_ARGS=()
[[ -t 0 && -t 1 ]] && TTY_ARGS=(-it)

if [[ $# -eq 0 ]]; then
  set -- ros2 launch tb3_nav2_demo demo.launch.py
fi

exec docker run --rm "${TTY_ARGS[@]}" \
  --name tb3_nav2 \
  --ipc=host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_QPA_PLATFORM=xcb \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  ${LIBGL_ALWAYS_SOFTWARE:+-e LIBGL_ALWAYS_SOFTWARE=1} \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${GPU_ARGS[@]}" \
  "$IMAGE" "$@"
