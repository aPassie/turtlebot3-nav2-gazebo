# TurtleBot3 + Nav2 in Gazebo

A TurtleBot3 Waffle driving itself around the TurtleBot3 world with ROS 2 Jazzy, Gazebo Harmonic and
Nav2. It runs a 5-goal patrol, the world has obstacles that aren't on Nav2's map, and a box drops in
front of the robot mid-run. Nav2 has to spot them with the LiDAR and plan around them. You can also
send your own goals from RViz.

## Setup

You need Docker and a desktop with X11 or XWayland.

```bash
git clone https://github.com/aPassie/turtlebot3-nav2-gazebo.git
cd turtlebot3-nav2-gazebo
docker build -t tb3-nav2 .     # ~5 GB, takes 5-10 min the first time
```

## Run

```bash
# Gazebo + RViz, the robot starts its patrol by itself
./docker/run.sh

# no patrol, send goals yourself with "Nav2 Goal" in RViz
./docker/run.sh ros2 launch tb3_nav2_demo demo.launch.py patrol:=False
```

AMD/Intel GPUs get passed into the container automatically. If rendering doesn't work,
`NO_GPU=1 ./docker/run.sh` uses software rendering. Rebuild the image after changing anything in
`tb3_nav2_demo/`.

Without Docker (Ubuntu 24.04 + ROS 2 Jazzy, same steps as the Dockerfile, I haven't tried it myself)

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/tb3_ws/src && cp -r tb3_nav2_demo ~/tb3_ws/src/ && cd ~/tb3_ws
rosdep install --from-paths src --ignore-src -y
# fix for the upstream mesh paths, otherwise RViz can't draw the robot
(cd /opt/ros/jazzy/share/nav2_minimal_tb3_sim/models && \
  for f in turtlebot3_model/meshes/*.dae; do sudo ln -sf "$f" .; done)
colcon build && source install/setup.bash
ros2 launch tb3_nav2_demo demo.launch.py
```

## How it works

The navigation itself is all stock Nav2 (AMCL, NavFn, MPPI, the behaviour tree and the collision
monitor). I didn't write navigation code, I set it up, tuned it and tested it.

- `launch/demo.launch.py` starts nav2_bringup's TurtleBot3 sim with my world, params and RViz config, then the patrol
- `worlds/tb3_obstacles.sdf.xacro` adds a crate, a barrel and a barrier that aren't in the map
- `scripts/patrol.py` sends the 5 goals and drops the surprise box once the robot is within 1.2 m of it
- `config/nav2_params.yaml` is the stock config with my changes marked `[tuned]`

## Testing

"Goal reached" doesn't tell you much, so the headless test logs the path the robot actually drove,
then checks how close it got to the obstacles and how long it stalled at the surprise box.

```bash
./tools/headless_test.sh tuned    # or stock, or any yaml in tb3_nav2_demo/config/
```

Logs and plots for every run are in `results/`.

| Run | What changed | Goals | Driving time | Clearance* | Stall at box | Collision monitor stops |
|---|---|---|---|---|---|---|
| 0 | first obstacle layout, box on a timer | 5/5 | 78.3 s | +0.102 m | – | – |
| 1 | obstacles moved onto the planner's paths | 5/5 | 51.1 s | +0.215 m** | – | – |
| 2, 4 | box spawned by distance, stock params | 5/5 | 59.9 / 58.0 s | +0.114 / +0.124 m | 2.1 / 1.6 s | 1 / 1 |
| 3, 5 | real Waffle speed limits + 10 Hz local costmap | 5/5 | 69.0 / 67.0 s | +0.096 / +0.107 m | 0.5 / 0.5 s | 0 / 0 |
| 6a, 6b | + 5 Hz global costmap (current default) | 5/5 | 67.5 / 68.6 s | +0.120 / +0.155 m | 0.0 / 0.5 s | 0 / 0 |
| 7a, 7b | only the speed limits | 5/5 | 68.1 / 66.6 s | +0.135 / +0.084 m | 0.0 / 0.0 s | 0 / 0 |
| 8 | default config, no GPU | 5/5 | 67.4 s | +0.121 m | 0.0 s | 0 |

\* closest the robot's 0.22 m footprint got to an obstacle that isn't on the map
\*\* not a real result, the box landed where the robot never went

It's only 2-4 runs per config and the positions are AMCL estimates that can be off by a few cm, so
small differences are just noise.

## Problems I ran into

**My tests passed but weren't really testing anything.** All 5 goals succeeded in the first runs, but
when I plotted the path the robot never went near most of my obstacles. The planner cuts between the
pillars instead of using the corridors where I'd put them, and the surprise box was on a timer, so it
showed up somewhere the robot wasn't. I moved the obstacles onto the routes the planner actually
takes and made the box spawn based on distance. That's also why the test logs the path now instead
of trusting "goal succeeded".

**The robot froze in front of the surprise box.** With stock params it stopped about half a meter
short for ~2 s before rerouting. I assumed the planner was slow, but the logs showed the collision
monitor was stopping it (`Robot to approach for 1.2 seconds away from collision`). Its 1.2 s
look-ahead at 0.5 m/s is about 0.6 m, which is exactly where it stopped. I'd changed the speed limit
and two costmap rates at the same time and first credited the wrong one, so I ran it again with only
the speed changed (run 7). At the real TurtleBot3 speed of 0.26 m/s the freeze is gone and the
collision monitor never kicks in. The costmap changes made no difference, the LiDAR only runs at
5 Hz anyway.

**RViz couldn't show the robot.** Two upstream issues in nav2_minimal_tb3_sim. The URDF points to the
wrong mesh folder, and the RViz config subscribes to `/robot_description` as Volatile while it's
published latched (Transient Local), so RViz never gets it. The Dockerfile links the meshes and my
RViz configs use Transient Local.
