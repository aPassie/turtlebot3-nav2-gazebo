# Sets GPU_ARGS for docker run. Used by docker/run.sh and tools/*.sh.
# NO_GPU=1 forces software rendering.
GPU_ARGS=()
if [[ ${NO_GPU:-0} == 1 ]]; then
  GPU_ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1)
elif docker info 2>/dev/null | grep -qiE 'runtimes:.*nvidia'; then
  # NVIDIA container toolkit. Not tested, I don't have it installed.
  GPU_ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
else
  # Only pass Intel/AMD nodes. If the NVIDIA node is passed without the toolkit,
  # EGL picks it first and Gazebo crashes. No nodes = software rendering.
  for node in /dev/dri/card* /dev/dri/renderD*; do
    [[ -e $node ]] || continue
    drv=$(basename "$(readlink -f "/sys/class/drm/$(basename "$node")/device/driver")" 2>/dev/null || true)
    [[ $drv == nvidia ]] && continue
    GPU_ARGS+=(--device "$node")
  done
fi
