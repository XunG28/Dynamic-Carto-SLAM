# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""
Offline localization on a rosbag (no Gazebo).

- Replay a baseline bag (with `/clock` when `use_sim_time:=true`), load a pbstream, and run Cartographer pure localization to output `/tracked_pose`.
- In parallel, record a new rosbag2 directory containing `/tracked_pose`, sensors, ground truth, TF, and `/clock` (when sim time is used). Recording uses `--use-sim-time` so all messages share the bag time axis.
- When bag playback ends, the launch shuts down and recording stops.

Usage:
  ros2 launch dynamic_carto_demo offline_localization.launch.py \\
    bag:=/path/to/baseline \\
    pbstream_in:=/path/map.pbstream \\
    bag_out:=/path/loc_out

  If the input bag does not contain `/clock`: set `use_sim_time:=false`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _use_sim_flag(use_sim_str: str) -> bool:
    return use_sim_str.lower() in ('true', '1', 'yes')


def launch_setup(context, *args, **kwargs):
    demo_share = get_package_share_directory('dynamic_carto_demo')
    qos_overrides = os.path.join(demo_share, 'config', 'record_qos_overrides.yaml')

    bag = LaunchConfiguration('bag').perform(context)
    bag_out = LaunchConfiguration('bag_out').perform(context)
    pbstream_in = LaunchConfiguration('pbstream_in').perform(context)
    use_sim_str = LaunchConfiguration('use_sim_time').perform(context)
    sim = _use_sim_flag(use_sim_str)

    config_dir = LaunchConfiguration('config_dir').perform(context)
    config_basename = LaunchConfiguration('config_basename').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': sim}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_basename,
            '-load_state_filename', pbstream_in,
        ],
        remappings=[
            ('scan', scan_topic),
            ('odom', odom_topic),
            ('imu', imu_topic),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': sim, 'resolution': 0.05}],
    )

    record_cmd = [
        'ros2', 'bag', 'record',
        '-o', bag_out,
        '--qos-profile-overrides-path', qos_overrides,
    ]
    if sim:
        record_cmd.append('--use-sim-time')
    record_topics = [
        '/tf',
        '/tf_static',
        '/scan',
        '/odom',
        '/imu',
        '/tracked_pose',
        '/eval/ground_truth/pose',
        '/eval/ground_truth/odom',
    ]
    if sim:
        record_topics = ['/clock'] + record_topics
    record_cmd.extend(record_topics)

    play_topics = [
        '/tf',
        '/tf_static',
        '/scan',
        '/odom',
        '/imu',
        '/eval/ground_truth/pose',
        '/eval/ground_truth/odom',
    ]
    if sim:
        play_topics = ['/clock'] + play_topics

    play_cmd = ['ros2', 'bag', 'play', bag, '--topics'] + play_topics

    rosbag_record = ExecuteProcess(cmd=record_cmd, output='screen')
    bag_play = ExecuteProcess(cmd=play_cmd, output='screen')

    on_play_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                EmitEvent(event=Shutdown(reason='ros2 bag play finished; stopping launch')),
            ],
        )
    )

    return [
        cartographer_node,
        occupancy_grid_node,
        rosbag_record,
        bag_play,
        on_play_exit,
    ]


def generate_launch_description():
    demo_share = get_package_share_directory('dynamic_carto_demo')
    _ = os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

    return LaunchDescription(
        [
            DeclareLaunchArgument('bag', description='Path to input rosbag2 (directory).'),
            DeclareLaunchArgument('pbstream_in', default_value='/tmp/offline_map.pbstream'),
            DeclareLaunchArgument(
                'bag_out',
                default_value='/tmp/offline_loc_bag',
                description='Output directory for new rosbag (localization run).',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='true: input bag has /clock; record with --use-sim-time.',
            ),
            DeclareLaunchArgument('config_dir', default_value=os.path.join(demo_share, 'config')),
            DeclareLaunchArgument(
                'config_basename',
                default_value='tb3_warehouse_lds_2d_localization.lua',
            ),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('odom_topic', default_value='/odom'),
            DeclareLaunchArgument('imu_topic', default_value='/imu'),
            OpaqueFunction(function=launch_setup),
        ]
    )
