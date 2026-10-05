#!/usr/bin/env python3
"""Simulation bring-up: Gazebo (Ignition) with SLAM and Nav2.

    source config/sim.env
    ros2 launch vla_bringup sim.launch.py

Headless, for CI and for the smoke test:

    xvfb-run -a ros2 launch vla_bringup sim.launch.py

Arguments:
    model     lite|standard   TurtleBot 4 model     (default lite)
    slam      true|false                            (default true)
    nav2      true|false                            (default true)
    rviz      true|false                            (default false)
    world     Ignition world name                   (default warehouse)

WHY rviz DEFAULTS TO false
    RViz segfaults in the ubuntu22-gpu container ("egl: failed to create dri2
    screen", exit -11) and takes the whole launch down with it. The operator
    console's own camera panel replaces it.

WHY THERE IS NO `headless` ARGUMENT
    The stock turtlebot4_ignition launch builds its own `ign_args` string and
    offers no way to add `-s` or `--headless-rendering`, so headless has to come
    from outside the launch system. `xvfb-run` is how, and it is installed in
    docker/ros.Dockerfile so this does not depend on one machine.

WHY THE DOMAIN IS NOT SET
    Domain 42 was tried and rejected (handout 5.1): gz_ros2_control runs INSIDE
    the Gazebo process and does not inherit ROS_DOMAIN_ID, so controller_manager
    came up on 0 while the spawner looked on 42 -- the robot spawned and could
    not move. Isolation comes from config/sim.env unsetting the discovery-server
    variables instead.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration('model')

    stock = PathJoinSubstitution([
        FindPackageShare('turtlebot4_ignition_bringup'),
        'launch', 'turtlebot4_ignition.launch.py',
    ])

    warn = []
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        warn.append(LogInfo(
            msg='vla_bringup WARNING: ROS_DISCOVERY_SERVER is set, which points '
                'at the robot. The simulation will discover nothing, silently. '
                'Run `source config/sim.env` first.'))

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='lite',
                              choices=['lite', 'standard']),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz segfaults in the container; keep false'),
        DeclareLaunchArgument('world', default_value='warehouse'),

        *warn,
        LogInfo(msg=['vla_bringup: simulation, model=', model,
                     '. For headless, wrap the whole command in `xvfb-run -a`.']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stock),
            launch_arguments={
                'model': model,
                'slam': LaunchConfiguration('slam'),
                'nav2': LaunchConfiguration('nav2'),
                'rviz': LaunchConfiguration('rviz'),
                'world': LaunchConfiguration('world'),
            }.items()),
    ])
