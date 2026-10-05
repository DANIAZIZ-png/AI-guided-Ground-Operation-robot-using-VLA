#!/usr/bin/env python3
# nav2_hw.launch.py -- Nav2 bring-up for the REAL TurtleBot 4 (namespace /robot1).
# Created 8 Sep 2026. HARDWARE ONLY. The simulation launches Nav2 through
# turtlebot4_ignition_bringup and never reads this file.
#
# Why this exists. The stock command
#     ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1
# runs nav2_bringup/navigation_launch.py with the lifecycle manager's default
# bond_timeout of 4.0 s, and neither file offers a way to change it. Through
# the discovery server on the Pi, a new topic connection between two processes
# on this PC can take several seconds to be matched, so the heartbeat "bond"
# each Nav2 server opens to the lifecycle manager often does not form inside
# the window (half the timeout) and the manager aborts the whole bring-up:
#     Server bt_navigator was unable to be reached after 4.00s by bond.
#     Failed to bring up all requested nodes. Aborting bringup.
# (observed 8 Sep 2026, 11:59.) Every server after the failure -- including
# velocity_smoother, the node that actually publishes /robot1/cmd_vel -- is
# then never activated, so goals are accepted but the robot never moves.
#
# Second symptom, same cause (8 Sep, 12:06): with the timeout raised, the
# manager's very first configure calls raced the discovery server. It asked
# smoother_server to configure less than a second after starting; the server
# did so, but rmw_fastrtps drops a service reply if the caller's reply
# channel is not matched within ~0.1 s, and the manager then waits forever:
#     [smoother_server.rclcpp]: failed to send response to
#         /robot1/smoother_server/change_state (timeout)
# Measured the same day: a fresh local subscriber needs 2.0-2.4 s to be
# matched to an existing local publisher through the Pi's discovery server.
#
# So this file differs from the stock path in exactly two ways:
#   1. bond_timeout launch argument, default 30 s (stock 4 s, not settable).
#   2. autostart is OFF; a timer sends the lifecycle manager its STARTUP
#      command startup_delay seconds later (default 20 s), by which time every
#      request/reply channel between the manager and its servers has matched.
# Nodes, remaps and parameters are otherwise identical to the stock path
# (stock /opt nav2.yaml, use_sim_time false, no composition, scan remapped
# into both costmaps).
#
# Usage, from a robot-mode terminal inside ubuntu22-gpu:
#     ros2 launch ~/nav2_hw.launch.py
# Optional:  params_file:=/path/to/nav2.yaml  bond_timeout:=30.0  startup_delay:=20.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

NAMESPACE = '/robot1'
LIFECYCLE_NODES = ['controller_server', 'smoother_server', 'planner_server',
                   'behavior_server', 'bt_navigator', 'waypoint_follower',
                   'velocity_smoother']


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    bond_timeout = LaunchConfiguration('bond_timeout')
    startup_delay = LaunchConfiguration('startup_delay')
    log_level = LaunchConfiguration('log_level')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('turtlebot4_navigation'), 'config', 'nav2.yaml'),
            description='Nav2 parameters (stock TurtleBot 4 file by default)'),
        DeclareLaunchArgument(
            'bond_timeout', default_value='30.0',
            description='Seconds the lifecycle manager allows each server heartbeat bond (stock 4.0)'),
        DeclareLaunchArgument(
            'startup_delay', default_value='20.0',
            description='Seconds after launch before the lifecycle manager is told to STARTUP'),
        DeclareLaunchArgument('log_level', default_value='info', description='log level'),
    ]

    # /tf and /tf_static are absolute in the nodes; map them to relative names so
    # the pushed namespace turns them into /robot1/tf and /robot1/tf_static.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=NAMESPACE,
            param_rewrites={'use_sim_time': 'false', 'autostart': 'true'},
            convert_types=True),
        allow_substs=True)

    def server(package, executable, name, extra_remaps=()):
        return Node(
            package=package, executable=executable, name=name, output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + list(extra_remaps))

    nav2 = GroupAction([
        PushRosNamespace(NAMESPACE),
        # Costmap nodes live in /robot1/<costmap>/, so a relative "scan" would
        # resolve to /robot1/global_costmap/scan. Point both at the LiDAR.
        SetRemap(NAMESPACE + '/global_costmap/scan', NAMESPACE + '/scan'),
        SetRemap(NAMESPACE + '/local_costmap/scan', NAMESPACE + '/scan'),
        # 11 Sep: keep parameter events and logs off the Wi-Fi link -- see the
        # comment in nav2_hw_composed.launch.py (Fast DDS heartbeat storm).
        SetRemap('/parameter_events', '/pc/parameter_events'),
        SetRemap('/rosout', '/pc/rosout'),

        server('nav2_controller', 'controller_server', 'controller_server',
               [('cmd_vel', 'cmd_vel_nav')]),
        server('nav2_smoother', 'smoother_server', 'smoother_server'),
        server('nav2_planner', 'planner_server', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', 'behavior_server'),
        server('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        server('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        server('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
               [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': False,
                'autostart': False,
                'node_names': LIFECYCLE_NODES,
                'bond_timeout': ParameterValue(bond_timeout, value_type=float),
            }]),
    ])

    # Delayed STARTUP (command 0 of nav2_msgs/srv/ManageLifecycleNodes). The
    # caller is a fresh participant, so its own reply may be lost; the
    # timeout wrapper stops it hanging the launch. The manager runs the
    # start-up regardless and reports "Managed nodes are active" itself.
    delayed_startup = TimerAction(
        period=startup_delay,
        actions=[ExecuteProcess(
            name='nav2_startup',
            cmd=['timeout', '300', 'ros2', 'service', 'call',
                 NAMESPACE + '/lifecycle_manager_navigation/manage_nodes',
                 'nav2_msgs/srv/ManageLifecycleNodes', '{command: 0}'],
            output='screen')])

    ld = LaunchDescription(args)
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(nav2)
    ld.add_action(delayed_startup)
    return ld
