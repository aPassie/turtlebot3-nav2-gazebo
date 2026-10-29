#!/usr/bin/env python3
"""Drive the TurtleBot3 through a list of goals using Nav2's Simple Commander API.

Each waypoint is sent as an individual NavigateToPose goal so that the result
(succeeded / failed / canceled), the time taken and the number of recoveries
can be reported per goal. Path planning and obstacle avoidance are done
entirely by the stock Nav2 stack (planner_server + controller_server + costmaps).

Extras for the demo / experiments:
  * a waypoint may carry a `spawn` spec: a box is inserted into Gazebo (it is
    not in the map) once the robot gets within `trigger_distance` of it;
  * `trajectory_file` logs the pose from Nav2 feedback for plot_trajectory.py.
Times are wall-clock; the simulation runs at ~1.0 real-time factor.
"""
import math
import subprocess
import sys
import threading
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def make_pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


BOX_SDF = (
    "<sdf version='1.6'><model name='{name}'><static>true</static><link name='link'>"
    "<collision name='c'><geometry><box><size>{s} {s} {h}</size></box></geometry></collision>"
    "<visual name='v'><geometry><box><size>{s} {s} {h}</size></box></geometry>"
    "<material><ambient>0.8 0.1 0.6 1</ambient><diffuse>0.8 0.1 0.6 1</diffuse></material>"
    "</visual></link></model></sdf>")


def spawn_box(nav: BasicNavigator, spec: dict) -> None:
    """Insert a static box into the running Gazebo world (not in the map)."""
    size = float(spec.get('size', 0.3))
    sdf = BOX_SDF.format(name=spec['name'], s=size, h=0.4)
    cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'default',
           '-name', spec['name'], '-string', sdf,
           '-x', str(spec['x']), '-y', str(spec['y']), '-z', '0.2']
    nav.get_logger().warn(
        f"Spawning unmapped obstacle '{spec['name']}' at ({spec['x']}, {spec['y']})")
    res = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if res.returncode != 0:
        nav.get_logger().error(f'Spawn failed: {res.stderr.strip()}')


def main() -> int:
    rclpy.init()
    nav = BasicNavigator()

    nav.declare_parameter('waypoints_file', '')
    nav.declare_parameter('loops', 1)
    nav.declare_parameter('set_initial_pose', True)
    nav.declare_parameter('trajectory_file', '')
    path = nav.get_parameter('waypoints_file').value or (
        get_package_share_directory('tb3_nav2_demo') + '/config/waypoints.yaml')
    loops = int(nav.get_parameter('loops').value)
    traj_path = nav.get_parameter('trajectory_file').value
    traj = open(traj_path, 'w') if traj_path else None
    if traj:
        traj.write('t,leg,x,y\n')

    with open(path) as f:
        cfg = yaml.safe_load(f)

    if nav.get_parameter('set_initial_pose').value:
        ip = cfg['initial_pose']
        nav.setInitialPose(make_pose(nav, ip['x'], ip['y'], ip['yaw']))
        nav.get_logger().info(f"Initial pose set to ({ip['x']}, {ip['y']})")

    nav.get_logger().info('Waiting for Nav2 to become active...')
    nav.waitUntilNav2Active()

    results = []
    spawned = set()
    for loop in range(loops):
        for wp in cfg['waypoints']:
            name = wp.get('name', f"({wp['x']}, {wp['y']})")
            nav.get_logger().info(f'[loop {loop + 1}/{loops}] Heading to {name}')
            start = time.monotonic()
            nav.goToPose(make_pose(nav, wp['x'], wp['y'], wp.get('yaw', 0.0)))

            # Optional obstacle that appears on this leg once the robot gets
            # within trigger_distance of it (so it lands right in its way).
            spec = wp.get('spawn')
            if spec and spec['name'] in spawned:
                spec = None

            last_log = 0.0
            recoveries = 0
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                now = time.monotonic()
                if not fb:
                    continue
                p = fb.current_pose.pose.position
                recoveries = fb.number_of_recoveries
                if traj:
                    traj.write(f'{now - start:.2f},{name},{p.x:.3f},{p.y:.3f}\n')
                if spec and math.hypot(p.x - spec['x'], p.y - spec['y']) < \
                        float(spec.get('trigger_distance', 1.0)):
                    spawned.add(spec['name'])
                    threading.Thread(target=spawn_box, args=(nav, spec)).start()
                    spec = None
                if now - last_log > 3.0:
                    last_log = now
                    nav.get_logger().info(
                        f'  {name}: {fb.distance_remaining:.2f} m remaining, '
                        f'{fb.number_of_recoveries} recoveries')

            elapsed = time.monotonic() - start
            result = nav.getResult()
            ok = result == TaskResult.SUCCEEDED
            results.append((name, ok, elapsed, recoveries))
            nav.get_logger().info(
                f'  {name}: {"SUCCEEDED" if ok else result.name} in {elapsed:.1f} s '
                f'({recoveries} recoveries)')

    if traj:
        traj.close()
        nav.get_logger().info(f'Trajectory written to {traj_path}')

    nav.get_logger().info('===== Patrol summary =====')
    for name, ok, elapsed, rec in results:
        nav.get_logger().info(
            f'  {"OK  " if ok else "FAIL"} {name:<16} {elapsed:6.1f} s  {rec} recoveries')
    succeeded = sum(r[1] for r in results)
    total_t = sum(r[2] for r in results)
    total_rec = sum(r[3] for r in results)
    nav.get_logger().info(
        f'  {succeeded}/{len(results)} goals reached, {total_t:.1f} s driving, '
        f'{total_rec} recoveries')

    nav.destroy_node()
    rclpy.shutdown()
    return 0 if succeeded == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
