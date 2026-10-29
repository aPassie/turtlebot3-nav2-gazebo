#!/usr/bin/env bash
# Self-contained demo recorder (runs INSIDE the container).
# Starts a virtual X display, the full demo with Gazebo GUI + RViz tiled side
# by side, runs the patrol and records the screen to an MP4 with a live caption.
#
#   record_demo.sh [output.mp4] [loops]
set -uo pipefail

OUT=${1:-/results/demo.mp4}
LOOPS=${2:-1}
WORK=$(dirname "$OUT")
W=1920; H=1080; BAR=56            # canvas size and caption bar height
HALF=$((W / 2)); VIEW_H=$((H - BAR))
export DISPLAY=:99

log() { echo "[record] $*"; }

Xvfb $DISPLAY -screen 0 ${W}x${H}x24 +extension GLX +render -noreset >/dev/null 2>&1 &
XVFB_PID=$!
sleep 2

log "launching simulation + Nav2 (GUI)"
ros2 launch tb3_nav2_demo demo.launch.py headless:=False use_rviz:=True patrol:=False \
    rviz_config:="$(ros2 pkg prefix tb3_nav2_demo)/share/tb3_nav2_demo/rviz/record.rviz" \
    > "$WORK/record_launch.log" 2>&1 &
LAUNCH_PID=$!

wait_window() {  # wait_window <name regex> -> prints window id
  local id=""
  for _ in $(seq 1 120); do
    id=$(xdotool search --onlyvisible --name "$1" 2>/dev/null | head -1)
    [[ -n $id ]] && { echo "$id"; return 0; }
    sleep 1
  done
  return 1
}

GZ_WIN=$(wait_window "Gazebo Sim") || { log "Gazebo window never appeared"; exit 1; }
RV_WIN=$(wait_window "RViz") || { log "RViz window never appeared"; exit 1; }
log "windows up: gazebo=$GZ_WIN rviz=$RV_WIN"

# Wait for Nav2 to finish bringing up its lifecycle nodes.
for _ in $(seq 1 120); do
  grep -q "Managed nodes are active" "$WORK/record_launch.log" && break
  sleep 1
done
sleep 3

xdotool windowmove "$GZ_WIN" 0 0 windowsize "$GZ_WIN" $HALF $VIEW_H
xdotool windowmove "$RV_WIN" $HALF 0 windowsize "$RV_WIN" $HALF $VIEW_H

sleep 3   # let both windows re-render at their new size

if [[ ${PREVIEW:-0} == 1 ]]; then      # layout check: one screenshot, then quit
  ffmpeg -loglevel error -y -f x11grab -video_size ${W}x${H} -i $DISPLAY -frames:v 1 \
    "$WORK/preview.png"
  log "preview -> $WORK/preview.png"
  kill -TERM $LAUNCH_PID; wait $LAUNCH_PID 2>/dev/null; kill $XVFB_PID
  exit 0
fi

# Caption text is refreshed from the patrol log twice a second.
CAPTION=/tmp/caption.txt
echo "TurtleBot3 + Nav2  |  ROS 2 Jazzy, Gazebo Harmonic  |  starting patrol..." > $CAPTION
PATROL_LOG="$WORK/record_patrol.log"
: > "$PATROL_LOG"
(
  while sleep 0.5; do
    line=$(grep -E "Heading to|Spawning|SUCCEEDED|FAILED|ABORTED|goals reached" "$PATROL_LOG" \
             | tail -1 | sed -E 's/^.*\]: +//; s/\[loop [0-9]+\/[0-9]+\] //')
    [[ -n $line ]] && printf 'TurtleBot3 + Nav2  |  %s\n' "$line" > $CAPTION.tmp \
      && mv $CAPTION.tmp $CAPTION
  done
) &
CAPTION_PID=$!

FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
log "recording -> $OUT"
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -video_size ${W}x${H} -framerate 25 -i $DISPLAY \
  -vf "drawbox=x=0:y=$VIEW_H:w=$W:h=$BAR:color=0x1c1f24@1:t=fill,\
drawtext=fontfile=$FONT:textfile=$CAPTION:reload=1:fontcolor=white:fontsize=26:x=24:y=$VIEW_H+14,\
drawtext=fontfile=$FONT:text='Gazebo (simulation)':fontcolor=white:fontsize=20:box=1:boxcolor=black@0.5:x=12:y=12,\
drawtext=fontfile=$FONT:text='RViz (map, costmaps, plan)':fontcolor=white:fontsize=20:box=1:boxcolor=black@0.5:x=$HALF+12:y=$VIEW_H-40" \
  -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p "$OUT" &
FFMPEG_PID=$!
sleep 2

log "starting patrol"
ros2 run tb3_nav2_demo patrol.py --ros-args -p use_sim_time:=true -p loops:=$LOOPS \
    -p trajectory_file:="$WORK/record_trajectory.csv" > "$PATROL_LOG" 2>&1
STATUS=$?
sleep 4                                   # linger on the final pose

kill -TERM $FFMPEG_PID; wait $FFMPEG_PID 2>/dev/null
kill $CAPTION_PID 2>/dev/null
kill -TERM $LAUNCH_PID; wait $LAUNCH_PID 2>/dev/null
kill $XVFB_PID 2>/dev/null
grep -E "OK  |FAIL |goals reached" "$PATROL_LOG"
log "done (patrol exit $STATUS)"
exit $STATUS
