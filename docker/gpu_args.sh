# Sourced by docker/run.sh and tools/*.sh: fills GPU_ARGS for `docker run`.
#   NO_GPU=1   force software rendering (Mesa llvmpipe), e.g. on CI / VMs.
GPU_ARGS=()
if [[ ${NO_GPU:-0} == 1 ]]; then
  GPU_ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1)
elif docker info 2>/dev/null | grep -qiE 'runtimes:.*nvidia'; then
  # NVIDIA Container Toolkit (runtime mode). Untested by the author: the
  # development machine had no toolkit, so the AMD/Intel path below was used.
  GPU_ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
else
  # Pass only DRM nodes Mesa can drive (Intel/AMD). Exposing an NVIDIA node
  # without the toolkit makes EGL pick it first, fail, and crash Gazebo's
  # Ogre2 renderer. With no usable node, Mesa falls back to llvmpipe.
  for node in /dev/dri/card* /dev/dri/renderD*; do
    [[ -e $node ]] || continue
    drv=$(basename "$(readlink -f "/sys/class/drm/$(basename "$node")/device/driver")" 2>/dev/null || true)
    [[ $drv == nvidia ]] && continue
    GPU_ARGS+=(--device "$node")
  done
fi
