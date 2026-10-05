#!/usr/bin/env python3
# nav2_hw_composed.launch.py -- Nav2 for the REAL TurtleBot 4, ALL SERVERS IN ONE
# PROCESS. Created 11 Sep 2026. HARDWARE ONLY; the simulation never reads it.
#
# Same nodes, parameters, remaps, bond_timeout and delayed STARTUP as
# nav2_hw.launch.py (read that file's header for why those exist). The one
# difference: the seven servers and the lifecycle manager are loaded as
# components into a single component_container_isolated, as nav2_bringup does
# with use_composition:=True.
#
# Why (11 Sep, CHANGELOG_2026-09-11.md): every ROS 2 process is one DDS
# participant, and every rclcpp node both publishes and subscribes to
# /parameter_events with RELIABLE QoS. Over the Wi-Fi link each PC participant's
# /parameter_events writer stays matched to the readers inside all eight Pi
# nodes, and after the ~90 parameter events Nav2 emits at configure time those
# writers fell into a HEARTBEAT storm (about 550 heartbeats/s per Pi reader,
# 7,400 packets/s from bt_navigator alone; 1.7 MB/s PC->Pi, packet capture on
# the Pi). Eight separate processes = eight storms. One container = one.
#
# Usage, from a robot-mode terminal inside ubuntu22-gpu:
#     ros2 launch ~/nav2_hw_composed.launch.py
# Optional:  params_file:=...  bond_timeout:=30.0  startup_delay:=20.0
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

NAMESPACE = '/robot1'
CONTAINER = 'nav2_container'
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

    # tf remaps as in the stock launch; the scan remaps replace nav2_hw.launch.py's
    # SetRemap actions, which only apply to separate Node processes. Given to the
    # container as process-wide rules they reach the costmap sub-nodes too.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
                  (NAMESPACE + '/global_costmap/scan', NAMESPACE + '/scan'),
                  (NAMESPACE + '/local_costmap/scan', NAMESPACE + '/scan'),
                  # 11 Sep: keep this PC's parameter events and log stream OFF the
                  # Wi-Fi link. Every rclcpp node on the Pi subscribes to
                  # /parameter_events RELIABLE; Nav2's configure-time burst of
                  # several hundred events to those eight readers put Fast DDS
                  # into a HEARTBEAT storm (2-2.6 MB/s PC->Pi, ~7,400 pkt/s,
                  # Pi at load 7 / 84 C) that never recovered. Renaming the
                  # topics on the PC side means the two machines' endpoints
                  # never match, so the reliable relationship never exists.
                  # Nothing on the robot needs this PC's parameter events or
                  # logs; the launch still tees stdout to ~/vla_logs/.
                  ('/parameter_events', '/pc/parameter_events'),
                  ('/rosout', '/pc/rosout')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=NAMESPACE,
            param_rewrites={'use_sim_time': 'false', 'autostart': 'true'},
            convert_types=True),
        allow_substs=True)

    def component(package, plugin, name, extra_remaps=()):
        return ComposableNode(
            package=package, plugin=plugin, name=name,
            parameters=[configured_params],
            remappings=remappings + list(extra_remaps))

    container = Node(
        name=CONTAINER, package='rclcpp_components',
        executable='component_container_isolated', output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings)

    load = LoadComposableNodes(
        target_container=NAMESPACE + '/' + CONTAINER,
        composable_node_descriptions=[
            component('nav2_controller', 'nav2_controller::ControllerServer',
                      'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
            component('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server'),
            component('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
            component('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server'),
            component('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator'),
            component('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower',
                      'waypoint_follower'),
            component('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
                      'velocity_smoother',
                      [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                remappings=remappings,   # components do not inherit the container's remaps
                parameters=[{
                    'use_sim_time': False,
                    'autostart': False,
                    'node_names': LIFECYCLE_NODES,
                    'bond_timeout': ParameterValue(bond_timeout, value_type=float),
                }]),
        ])

    nav2 = GroupAction([PushRosNamespace(NAMESPACE), container, load])

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
