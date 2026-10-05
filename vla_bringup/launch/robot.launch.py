#!/usr/bin/env python3
"""Hardware bring-up: SLAM, then Nav2, with the Fast DDS storm fixes applied.

    source config/robot.env
    ros2 launch vla_bringup robot.launch.py

Arguments:
    namespace        default /robot1, or $VLA_NS if set
    slam             true|false   start SLAM            (default true)
    nav2             true|false   start Nav2            (default true)
    nav2_params      params file for Nav2               (default: Nav2's own)
    slam_params      params file for SLAM               (default: slam_vla.yaml)
    bond_timeout     lifecycle bond timeout in seconds  (default 30.0)
    slam_delay       seconds to wait before Nav2 starts (default 15.0)

WHY NAV2 IS DELAYED
    SLAM has to be publishing map->odom before Nav2's lifecycle manager starts
    bonding, or the costmaps come up with no map and the manager times out. The
    composed Nav2 already delays its own STARTUP by 20 s; this adds the gap
    between the two launches.

WHAT THIS DOES NOT DO
    It does not set the DDS environment. A launch file cannot export variables
    into its own process, so `source config/robot.env` first. Without it, ROS
    either finds no discovery server or -- worse -- uses the simulation's
    settings and silently discovers nothing.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('vla_bringup')
    launch_dir = os.path.join(share, 'launch')
    config_dir = os.path.join(share, 'config')

    namespace = LaunchConfiguration('namespace')
    slam_params = LaunchConfiguration('slam_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=os.environ.get('VLA_NS') or '/robot1',
            description='ROS namespace the robot publishes under'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(config_dir, 'slam_vla.yaml'),
            description='SLAM Toolbox parameters'),
        DeclareLaunchArgument(
            'nav2_params', default_value='',
            description='Nav2 parameters; empty means the stock /opt file. '
                        'Pass config/nav2_hw_slow.yaml for gentler speeds.'),
        DeclareLaunchArgument(
            'bond_timeout', default_value='30.0',
            description='Lifecycle bond timeout. The stock 4 s is what made the '
                        'lifecycle manager kill Nav2 mid-bring-up.'),
        DeclareLaunchArgument(
            'slam_delay', default_value='15.0',
            description='Seconds between SLAM and Nav2, so map->odom exists first'),

        LogInfo(msg=['vla_bringup: hardware. namespace=', namespace,
                     ' -- /parameter_events and /rosout are remapped off the '
                     'Wi-Fi link; do NOT substitute the stock launch files.']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'slam_hw.launch.py')),
            condition=IfCondition(LaunchConfiguration('slam')),
            launch_arguments={'namespace': namespace,
                              'params': slam_params}.items()),

        TimerAction(
            period=LaunchConfiguration('slam_delay'),
            actions=[
                LogInfo(msg='vla_bringup: starting Nav2 (composed, bond_timeout 30 s)'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'nav2_hw_composed.launch.py')),
                    condition=IfCondition(LaunchConfiguration('nav2')),
                    launch_arguments={
                        'namespace': namespace,
                        'bond_timeout': LaunchConfiguration('bond_timeout'),
                    }.items()),
            ]),
    ])
