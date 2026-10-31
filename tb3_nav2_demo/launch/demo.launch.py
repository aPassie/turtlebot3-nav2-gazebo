"""TurtleBot3 + Nav2 demo.

Uses nav2_bringup's tb3_simulation_launch.py for everything (Gazebo, bridge,
Nav2, RViz) with my world, params and RViz config, and starts patrol.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tb3_nav2_demo')
    bringup = get_package_share_directory('nav2_bringup')

    args = [
        DeclareLaunchArgument('headless', default_value='False',
                              description='Run Gazebo without its GUI'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            pkg, 'worlds', 'tb3_obstacles.sdf.xacro')),
        DeclareLaunchArgument('map', default_value=os.path.join(
            bringup, 'maps', 'tb3_sandbox.yaml'),
            description='Static map (the extra obstacles are not in it)'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            pkg, 'rviz', 'nav2_demo.rviz')),
        DeclareLaunchArgument('patrol', default_value='True',
                              description='Automatically drive the waypoint route'),
        DeclareLaunchArgument('loops', default_value='1'),
        DeclareLaunchArgument('trajectory_file', default_value='',
                              description='Optional CSV path to log the driven path'),
        DeclareLaunchArgument('patrol_delay', default_value='5.0',
                              description='Seconds to wait before starting the patrol'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'rviz_config_file': LaunchConfiguration('rviz_config'),
            'x_pose': '-2.0',
            'y_pose': '-0.5',
        }.items(),
    )

    patrol = TimerAction(
        period=LaunchConfiguration('patrol_delay'),
        actions=[Node(
            package='tb3_nav2_demo',
            executable='patrol.py',
            name='patrol',
            output='screen',
            condition=IfCondition(LaunchConfiguration('patrol')),
            parameters=[{
                'use_sim_time': True,
                'loops': LaunchConfiguration('loops'),
                'trajectory_file': LaunchConfiguration('trajectory_file'),
                'waypoints_file': os.path.join(pkg, 'config', 'waypoints.yaml'),
            }],
        )],
    )

    return LaunchDescription(args + [sim, patrol])
