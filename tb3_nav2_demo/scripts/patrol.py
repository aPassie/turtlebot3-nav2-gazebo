#!/usr/bin/env python3
"""Drive the TurtleBot3 through a list of goals using Nav2's Simple Commander API.

Each waypoint is sent as an individual NavigateToPose goal so that the result
(succeeded / failed / canceled), the time taken and the distance remaining can
be reported per goal. Path planning and obstacle avoidance are done entirely by
the stock Nav2 stack (planner_server + controller_server + costmaps).
"""
import math
import sys
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


def main() -> int:
    rclpy.init()
    nav = BasicNavigator()

    nav.declare_parameter('waypoints_file', '')
    nav.declare_parameter('loops', 1)
    nav.declare_parameter('set_initial_pose', True)
    path = nav.get_parameter('waypoints_file').value or (
        get_package_share_directory('tb3_nav2_demo') + '/config/waypoints.yaml')
    loops = int(nav.get_parameter('loops').value)

    with open(path) as f:
        cfg = yaml.safe_load(f)

    if nav.get_parameter('set_initial_pose').value:
        ip = cfg['initial_pose']
        nav.setInitialPose(make_pose(nav, ip['x'], ip['y'], ip['yaw']))
        nav.get_logger().info(f"Initial pose set to ({ip['x']}, {ip['y']})")

    nav.get_logger().info('Waiting for Nav2 to become active...')
    nav.waitUntilNav2Active()

    results = []
    for loop in range(loops):
        for wp in cfg['waypoints']:
            name = wp.get('name', f"({wp['x']}, {wp['y']})")
            nav.get_logger().info(f'[loop {loop + 1}/{loops}] Heading to {name}')
            start = time.monotonic()
            nav.goToPose(make_pose(nav, wp['x'], wp['y'], wp.get('yaw', 0.0)))

            last_log = 0.0
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                now = time.monotonic()
                if fb and now - last_log > 3.0:
                    last_log = now
                    nav.get_logger().info(
                        f'  {name}: {fb.distance_remaining:.2f} m remaining, '
                        f'{fb.number_of_recoveries} recoveries')

            elapsed = time.monotonic() - start
            result = nav.getResult()
            ok = result == TaskResult.SUCCEEDED
            results.append((name, ok, elapsed))
            nav.get_logger().info(
                f'  {name}: {"SUCCEEDED" if ok else result.name} in {elapsed:.1f} s')

    nav.get_logger().info('===== Patrol summary =====')
    for name, ok, elapsed in results:
        nav.get_logger().info(f'  {"OK  " if ok else "FAIL"} {name:<16} {elapsed:6.1f} s')
    succeeded = sum(ok for _, ok, _ in results)
    nav.get_logger().info(f'  {succeeded}/{len(results)} goals reached')

    nav.destroy_node()
    rclpy.shutdown()
    return 0 if succeeded == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
