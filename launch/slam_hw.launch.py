#!/usr/bin/env python3
# slam_hw.launch.py -- slam_toolbox for the REAL TurtleBot 4 (14 Sep 2026). HARDWARE ONLY.
# Wraps the stock turtlebot4_navigation slam.launch.py unchanged and adds two remaps
# for every node it starts: /parameter_events -> /pc/parameter_events and
# /rosout -> /pc/rosout. Reason: the Pi's camera node put its RELIABLE
# /parameter_events writer into a heartbeat storm toward slam_toolbox's reader
# (capture 02, CHANGELOG_2026-09-11.md §2.4). Renaming the topic on the PC side
# means the endpoints never match. The simulation never reads this file.
#
#   ros2 launch launch/slam_hw.launch.py namespace:=/robot1 \
#        params:=<repo>/config/slam_vla.yaml
#
# `params` and `namespace` default to the repository's own config and to
# $VLA_NS, so neither has to be passed.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def _vla_root():
    """Repository root, from this file's location unless VLA_ROOT says otherwise."""
    return os.environ.get('VLA_ROOT') or os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    stock = os.path.join(get_package_share_directory('turtlebot4_navigation'),
                         'launch', 'slam.launch.py')
    config_dir = os.environ.get('VLA_CONFIG_DIR') or os.path.join(_vla_root(), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('namespace',
                              default_value=os.environ.get('VLA_NS') or '/robot1'),
        DeclareLaunchArgument('params',
                              default_value=os.path.join(config_dir, 'slam_vla.yaml')),
        DeclareLaunchArgument('sync', default_value='true'),
        GroupAction([
            SetRemap('/parameter_events', '/pc/parameter_events'),
            SetRemap('/rosout', '/pc/rosout'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stock),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'params': LaunchConfiguration('params'),
                    'sync': LaunchConfiguration('sync'),
                }.items()),
        ]),
    ])
