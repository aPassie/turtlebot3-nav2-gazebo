#!/usr/bin/env bash
# Run the patrol without GUI, then plot the path and check clearance.
# ./tools/headless_test.sh [tuned|stock|<file>.yaml] [loops] [tag]
# Output goes to results/<tag>/. Exits 1 if a goal fails or the robot hits something.
set -euo pipefail
cd "$(dirname "$0")/.."

PARAMS=${1:-tuned}
LOOPS=${2:-1}
TAG=${3:-$PARAMS}
case $PARAMS in
  tuned) PARAMS_FILE=nav2_params.yaml ;;
  stock) PARAMS_FILE=nav2_params_stock.yaml ;;
  *.yaml) PARAMS_FILE=$PARAMS ;;    # any file in tb3_nav2_demo/config/
  *) echo "unknown params profile: $PARAMS" >&2; exit 2 ;;
esac
OUT="$PWD/results/$TAG"
mkdir -p "$OUT"
IMAGE=${IMAGE:-tb3-nav2:latest}

source docker/gpu_args.sh

docker run --rm --init --ipc=host "${GPU_ARGS[@]}" -v "$OUT":/results \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" -e LOOPS="$LOOPS" -e PARAMS_FILE="$PARAMS_FILE" \
  "$IMAGE" bash -c '
    log=/results/launch.log
    ros2 launch tb3_nav2_demo demo.launch.py headless:=True use_rviz:=False \
        params_file:=$(ros2 pkg prefix tb3_nav2_demo)/share/tb3_nav2_demo/config/$PARAMS_FILE \
        loops:=$LOOPS trajectory_file:=/results/trajectory.csv > $log 2>&1 &
    launch_pid=$!
    deadline=$((SECONDS + 300 * LOOPS))
    until grep -qE "goals reached|process has died" $log; do
      (( SECONDS > deadline )) && { echo "TIMEOUT"; break; }
      kill -0 $launch_pid 2>/dev/null || { echo "launch exited early"; break; }
      sleep 2
    done
    grep -E "Patrol summary|OK  |FAIL |goals reached|Spawn|process has died" $log || true
    status=1
    if grep -qE " ([0-9]+)/\1 goals reached" $log && ! grep -q "process has died" $log; then
      echo; ros2 run tb3_nav2_demo plot_trajectory.py /results/trajectory.csv /results/trajectory.png \
        && status=0
    fi
    kill -TERM $launch_pid
    for _ in $(seq 20); do kill -0 $launch_pid 2>/dev/null || break; sleep 1; done
    kill -KILL $launch_pid 2>/dev/null   # leftovers die with the container (--init)
    chown -R $HOST_UID:$HOST_GID /results
    exit $status'
