# TurtleBot3 + Nav2 autonomous navigation in Gazebo

A TurtleBot3 Waffle in **Gazebo Harmonic** uses the **ROS 2 Jazzy Navigation2 stack** to drive a
5-goal patrol through the TurtleBot3 world. Obstacles that are **not in the static map** block every
leg, and a "surprise" box is dropped onto the robot's route mid-run. Nav2 has to see each obstacle
with the LiDAR, mark it in the costmaps, and replan around it.

> **Demo video (79 s):** [`media/demo.mp4`](media/demo.mp4): Gazebo on the left, RViz on the
> right, live caption at the bottom. Recorded with `./tools/record_demo.sh`.

![Surprise obstacle: Nav2 replans live](media/surprise_reroute.gif)

*The surprise box (magenta, left) appears on the robot's route. The LiDAR marks it in the local
costmap (right), the green global plan swings west through the pillars, and the robot follows it.*

![Driven path vs. obstacles](media/trajectory.png)

*Path driven in the recorded demo: 5/5 goals, 67.2 s, 0 recoveries, never closer than 0.09 m to
an obstacle beyond its own footprint. Magenta shapes are **not** in the map Nav2 was given. Stars
are goals.*

---

## What's in here

| Path | Purpose |
|---|---|
| `Dockerfile`, `docker/` | Reproducible ROS 2 Jazzy + Gazebo Harmonic + Nav2 image, GUI run script |
| `tb3_nav2_demo/launch/demo.launch.py` | Wraps the stock `nav2_bringup/tb3_simulation_launch.py` with this package's world, params and RViz config. Optionally starts the patrol |
| `tb3_nav2_demo/worlds/tb3_obstacles.sdf.xacro` | TurtleBot3 sandbox world plus 3 extra obstacles that are not in the map |
| `tb3_nav2_demo/config/nav2_params.yaml` | Nav2 params (stock + documented `[tuned]` changes) |
| `tb3_nav2_demo/config/nav2_params_stock.yaml` | Untouched upstream params, for A/B comparison |
| `tb3_nav2_demo/config/waypoints.yaml` | Patrol route and the surprise-obstacle spec |
| `tb3_nav2_demo/scripts/patrol.py` | Sends goals with `nav2_simple_commander`, logs results, path and recoveries, spawns the surprise obstacle |
| `tb3_nav2_demo/scripts/plot_trajectory.py` | Plots the driven path over the map, computes clearance to every obstacle and the reaction (stall) time at the surprise box |
| `tb3_nav2_demo/rviz/` | `nav2_demo.rviz` (interactive, full panels) and `record.rviz` (clean layout for the video) |
| `tools/headless_test.sh` | End-to-end test without GUI. Exits non-zero if a goal fails or the robot touches an obstacle |
| `tools/record_demo.sh` | Records the demo video on a virtual display inside the container |
| `results/` | Logs, CSVs and plots from the experiment runs below |
| `media/` | Demo video, GIF, and path/plot of the recorded run |

No custom navigation algorithms. Planning, control, localisation and recovery are all stock Nav2
(NavFn planner, MPPI controller, AMCL, behaviour-tree navigator). The work here is integration,
configuration, test design and measurement.

## Stack

- ROS 2 **Jazzy** (LTS), Ubuntu 24.04 (in Docker)
- Gazebo **Harmonic** via `ros_gz`, robot and world from `nav2_minimal_tb3_sim`
- Nav2: `map_server` + `amcl` (localisation), `planner_server` (NavFn), `controller_server` (MPPI),
  `bt_navigator`, `behavior_server`, `velocity_smoother`, `collision_monitor`
- TurtleBot3 **Waffle** with a 360° LiDAR

```
Gazebo Harmonic ──(ros_gz_bridge: /scan /odom /tf /clock, cmd_vel)──► ROS 2
   ▲                                                                   │
   │ cmd_vel                    map_server (static map, no extra obstacles)
   │                            amcl ──► map→odom
   └── collision_monitor ◄── velocity_smoother ◄── controller_server (MPPI, local costmap: LiDAR voxel layer)
                                                         ▲
                                     bt_navigator ──► planner_server (NavFn, global costmap: static + LiDAR obstacle layer)
                                         ▲
                                   patrol.py (NavigateToPose goals)
```

## Quick start (Docker, any Linux with X11/XWayland)

```bash
git clone <this repo> && cd turtlebot3-nav2-gazebo
docker build -t tb3-nav2 .

# Full demo: Gazebo GUI + RViz, robot runs the patrol on its own
./docker/run.sh
```

`docker/run.sh` forwards X11 and passes through Intel/AMD GPUs for hardware OpenGL (see
[Problems](#problems-i-hit-and-how-i-fixed-them) for NVIDIA notes). For software rendering add
`LIBGL_ALWAYS_SOFTWARE=1`.

Other ways to run it:

```bash
# Send goals yourself with "Nav2 Goal" in RViz (no patrol)
./docker/run.sh ros2 launch tb3_nav2_demo demo.launch.py patrol:=False

# Patrol several times (obstacles stay in the costmap between loops)
./docker/run.sh ros2 launch tb3_nav2_demo demo.launch.py loops:=3

# Compare against the untouched upstream parameters
./docker/run.sh ros2 launch tb3_nav2_demo demo.launch.py \
    params_file:=/ws/install/tb3_nav2_demo/share/tb3_nav2_demo/config/nav2_params_stock.yaml

# Headless end-to-end test + plot + clearance check  -> results/<tag>/
./tools/headless_test.sh tuned        # or: stock
# Record the demo video (no screen capture of your desktop needed) -> media/demo.mp4
./tools/record_demo.sh
```

### Native install (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb3-sim \
                 ros-jazzy-nav2-simple-commander ros-jazzy-ros-gz
# Work around the upstream mesh-path bug so RViz can draw the robot (see Problems #3)
cd /opt/ros/jazzy/share/nav2_minimal_tb3_sim/models && \
  for f in turtlebot3_model/meshes/*.dae; do sudo ln -sf "$f" .; done; cd -
mkdir -p ~/tb3_ws/src && cp -r tb3_nav2_demo ~/tb3_ws/src/
cd ~/tb3_ws && colcon build && source install/setup.bash
ros2 launch tb3_nav2_demo demo.launch.py
```

## How the demo is set up

**Map vs. world.** Nav2 gets the stock `tb3_sandbox` map (hexagon walls, 3×3 pillars). The Gazebo
world adds a crate, a barrel and a barrier that are not in that map. Nav2 only finds out about
them through the LiDAR-fed `voxel_layer` (local costmap) and `obstacle_layer` (global costmap).

**Placement is data-driven.** Each obstacle sits on the shortest path the planner took for one
leg in an earlier run, so every leg has to replan (see Experiments, run 0 → 1).

**Surprise obstacle.** On the `south` leg, `patrol.py` uses `ros_gz_sim create` to spawn a
0.4 m box on the robot's current route once the robot is within 1.2 m of it. The gaps either
side are narrower than the robot, so it has to turn away and reroute through the pillars.

**Patrol.** `start → north_west → east → south → centre → home`, one `NavigateToPose` goal at a
time. Each goal reports success/failure, duration and the number of recoveries.

## Experiments

Every run is reproducible with `./tools/headless_test.sh <stock|tuned|file.yaml>`.

- **Clearance**: minimum distance between the robot footprint (r = 0.22 m) and any unmapped
  obstacle surface, computed from the logged path.
- **Stall**: time the robot sat almost still (< 0.05 m/s) between the surprise box appearing
  and the robot driving past it.

Logs, CSVs and plots are in [`results/`](results).

| Run | Change being tested | Params | Goals | Driving time | Recoveries | Min clearance | Stall at surprise box |
|---|---|---|---|---|---|---|---|
| 0 | First obstacle layout, timer-triggered surprise box | stock | 5/5 | 78.3 s | 1 (controller stuck at the barrel) | +0.102 m | n/a |
| 1 | Obstacles moved onto the planner's actual paths | stock | 5/5 | 51.1 s | 0 | +0.215 m* | n/a* |
| 2 | Surprise box triggered by **proximity** instead of a timer | stock | 5/5 | 59.9 s | 0 | +0.114 m | 2.1 s |
| 4 | repeat of 2 | stock | 5/5 | 58.0 s | 0 | +0.124 m | 1.6 s |
| 3 | **Real TB3 Waffle speed limits** + local costmap 5 → 10 Hz | tuned v1 | 5/5 | 69.0 s | 0 | +0.096 m | 0.5 s |
| 5 | repeat of 3 | tuned v1 | 5/5 | 67.0 s | 0 | +0.107 m | 0.5 s |
| 6a | + **global costmap 1 → 5 Hz** | tuned (final) | 5/5 | 67.5 s | 0 | +0.120 m | 0.0 s |
| 6b | repeat of 6a | tuned (final) | 5/5 | 68.6 s | 0 | +0.155 m | 0.5 s |
| demo | the recorded video run (GUI + RViz) | tuned (final) | 5/5 | 67.2 s | 0 | +0.094 m | 0.0 s |

\* In run 1 the timer-spawned box landed where the robot never went. The large clearance means
the test was broken, not that navigation improved.

The final config differs from upstream `nav2_params.yaml` only in what's marked `[tuned]`
(`diff tb3_nav2_demo/config/nav2_params_stock.yaml tb3_nav2_demo/config/nav2_params.yaml`):

| Parameter | Stock | Tuned | Why |
|---|---|---|---|
| MPPI `vx_max` / `vx_min` / `wz_max` | 0.5 / −0.35 / 1.9 | 0.26 / −0.26 / 1.82 | Real TurtleBot3 Waffle limits |
| `velocity_smoother` max/min velocity | ±0.5, ±2.0 | ±0.26, ±1.82 | Same limits after smoothing |
| Local costmap `update_frequency` | 5 Hz | 10 Hz | Sudden obstacles reach the controller sooner |
| Global costmap `update_frequency` | 1 Hz | 5 Hz | Planner sees new obstacles sooner and reroutes earlier |
| AMCL `set_initial_pose` | off | on, at the spawn pose | No manual "2D Pose Estimate" needed |

**What I learned from each step**

- **Run 0, the obstacles weren't in the way.** I first placed obstacles in the "corridors"
  between the walls and pillars. The plot showed NavFn takes the shortest route *between* the
  pillars, so 3 of 4 obstacles were never closer than 0.4 m to the path. The barrel was the
  exception: MPPI got stuck next to it ("Failed to make progress"), a recovery ran, and the leg
  took 33 s. I moved every obstacle onto the paths from that run.
- **Run 1, a timer is the wrong trigger.** Without the old east barrier, the robot took a
  different route on the `south` leg, so the box spawned 6 s into the leg landed on a route it
  never used. I replaced the timer with a proximity trigger. From run 2 on, the box appears about
  1 m ahead of the robot, the leg takes ~16 s instead of 9 s, and the plot shows the reroute.
- **Runs 2–5, real speed limits.** Stock Nav2 drives the TB3 at 0.5 m/s, but a real Waffle tops
  out at 0.26 m/s. I expected the slower robot to keep more distance from the surprise box.
  **It didn't.** Clearance was the same within noise. The speed profile showed why: in both
  profiles the robot *stops* ~0.55 m short of the box, and the closest approach happens later,
  while it drives the new path past the box's corner. That distance is set by the costmap
  inflation and the route, not by speed. Two things did change. The stock robot stalled
  1.6–2.1 s before moving off, the tuned one only 0.5 s (it was already slower and eased off
  instead of braking hard). The full patrol took ~15% longer (≈59 s → ≈68 s), not 2×, because the
  robot rarely reaches 0.5 m/s in this tight arena. I kept the real limits: a config that matches
  the hardware is the one you'd deploy.
- **Run 6, why the stall?** The global costmap updates at 1 Hz and the behaviour tree replans at
  1 Hz, so up to ~2 s can pass before the planner knows the box exists. Raising the global
  costmap to 5 Hz (negligible CPU on this map) made the robot reroute ~1 s sooner: `south` leg
  15.7–16.2 s vs. 16.6–17.1 s, stall 0–0.5 s. Clearance stayed within the same noise band
  (+0.09 to +0.16 m), so the gain is responsiveness, not safety margin. It's small and only
  n = 3, but it was consistent across all runs.
- Inflation (0.7 m radius, scaling 3.0) stayed at stock values. With it there were no recoveries
  and no contact in any run after run 0, so I had no evidence to justify changing it.

## Problems I hit and how I fixed them

1. **Gazebo segfaulted on start (hybrid AMD + NVIDIA laptop, no NVIDIA Container Toolkit).**
   With all of `/dev/dri` passed in, EGL picked the NVIDIA render node first. Mesa has no driver
   for it, so Ogre2 failed ("Failed to create dummy render window") and `gz sim` crashed. That
   left no LiDAR and no AMCL pose, and the patrol waited forever. I traced it with `eglinfo` and
   `/sys/class/drm/*/device/driver`. The run scripts now pass only Mesa-capable DRM nodes, and
   the container gets hardware rendering on the iGPU (AMD Radeon 780M, radeonsi). If the NVIDIA
   runtime is installed, `run.sh` uses `--gpus all` instead.
2. **Test harness hung after a successful run.** Two bash bugs: a regex that didn't match the
   log format (`]:   5/5` vs. `] 5/5`), and `kill -INT` on a background job. Non-interactive
   bash starts background jobs with SIGINT ignored, so the launch never exited. Fixed with a
   correct regex and SIGTERM.
3. **RViz couldn't show the robot.** Upstream `nav2_minimal_tb3_sim`'s URDF references
   `package://nav2_minimal_tb3_sim/models/<mesh>.dae`, but the meshes are installed under
   `models/turtlebot3_model/meshes/`. Also, the stock RViz config subscribes to the latched
   `/robot_description` with Volatile QoS, so it never receives it. The Dockerfile symlinks the
   meshes, and the RViz configs use Transient Local.
4. **"Goal SUCCEEDED" isn't proof of obstacle avoidance.** The first runs reported 5/5, but only
   the trajectory plot showed the obstacles weren't actually being tested. That's why the repo
   logs the driven path and checks clearance automatically.
5. **Recording on Wayland (niri).** Rather than depend on the host compositor, the recorder runs
   Gazebo + RViz on Xvfb inside the container, tiles them with `xdotool`, and captures with
   ffmpeg plus a caption bar that follows the patrol log. First take: Gazebo's inspector panels
   and RViz's docks covered half the screen, and the map wasn't visible. Fixes: a `<gui>` block in
   the world with a fixed camera (setting it via `/gui/move_to/pose` didn't take effect), and an
   RViz layout without panels (the `Hide Dock` flags are overridden by the saved window state).

## Credits

Built on [Nav2](https://docs.nav2.org/) (`nav2_bringup`, `nav2_minimal_tb3_sim`,
`nav2_simple_commander`), [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
models by ROBOTIS, and [Gazebo](https://gazebosim.org/). The map, robot model and base world come
from those packages unchanged.
