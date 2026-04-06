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

<<<<<<< HEAD
  If the input bag does not contain `/clock`: set `use_sim_time:=false`.
=======
  打滑 / 劣化里程计（bag 内需有 /odom_noisy）:
    odom_topic:=/odom_noisy
    （默认自动把 child_frame 改成 base_footprint、frame_id 改成 odom，供 Cartographer 与 bag 内 TF 一致）

  无 /clock 的 bag: use_sim_time:=false
>>>>>>> 8cad010 (feat(demo): add odom-noisy localization and fused KISS-ICP EKF pipeline)
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
    play_tf_mode = LaunchConfiguration('play_tf').perform(context).lower()

    fix_arg = LaunchConfiguration('fix_odom_frames_for_cartographer').perform(context)
    fa = fix_arg.lower()
    if fa == 'auto':
        use_odom_fix = odom_topic == '/odom_noisy'
    elif fa in ('false', '0', 'no'):
        use_odom_fix = False
    else:
        # true: only applies to noisy odom (standard /odom does not need rewriting)
        use_odom_fix = odom_topic == '/odom_noisy'

    # odom_noisy 默认 child_frame=base_footprint_noisy、frame_id=odom_noisy，与 bag 内 TF（odom->base_footprint）不一致
    carto_odom_topic = '/odom_noisy_carto' if use_odom_fix else odom_topic

    odom_fix_node = None
    if use_odom_fix:
        odom_fix_node = Node(
            package='dynamic_carto_demo',
            executable='odom_child_frame_fix',
            name='odom_child_frame_fix',
            output='screen',
            parameters=[
                {
                    'use_sim_time': sim,
                    'input_topic': odom_topic,
                    'output_topic': carto_odom_topic,
                    'frame_id': 'odom',
                    'child_frame_id': 'base_footprint',
                }
            ],
        )

    # When feeding Cartographer with /odom_noisy (fixed to /odom_noisy_carto), we usually want TF
    # to be consistent with that odometry, not with the bag-recorded true wheel TF. In that case,
    # do not play /tf from bag; instead broadcast odom->base_footprint from the odometry topic.
    if play_tf_mode == 'auto':
        play_tf = not use_odom_fix
    else:
        play_tf = play_tf_mode in ('true', '1', 'yes')

    odom_to_tf_node = None
    if not play_tf:
        odom_to_tf_node = Node(
            package='dynamic_carto_demo',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
            parameters=[
                {
                    'use_sim_time': sim,
                    'input_topic': carto_odom_topic,
                }
            ],
        )

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
            ('odom', carto_odom_topic),
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
    # 录制时同时保留干净 /odom、bag 里原始 odom 输入、以及 Cartographer 实际订阅的 topic
    odom_record_topics = ['/odom']
    if odom_topic not in odom_record_topics:
        odom_record_topics.append(odom_topic)
    if use_odom_fix and carto_odom_topic not in odom_record_topics:
        odom_record_topics.append(carto_odom_topic)

    record_topics = [
        '/tf',
        '/tf_static',
        '/scan',
        '/imu',
        '/tracked_pose',
        '/eval/ground_truth/pose',
        '/eval/ground_truth/odom',
    ]
    if sim:
        record_topics = ['/clock'] + record_topics
    # insert odom topics after /scan for readability
    idx = record_topics.index('/scan') + 1
    for t in reversed(odom_record_topics):
        record_topics.insert(idx, t)
    record_cmd.extend(record_topics)

    play_topics = [
        '/tf_static',
        '/scan',
        odom_topic,
        '/imu',
        '/eval/ground_truth/pose',
        '/eval/ground_truth/odom',
    ]
    if play_tf:
        play_topics.insert(0, '/tf')
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

    nodes = []
    if odom_fix_node is not None:
        nodes.append(odom_fix_node)
    if odom_to_tf_node is not None:
        nodes.append(odom_to_tf_node)
    nodes.extend(
        [
            cartographer_node,
            occupancy_grid_node,
            rosbag_record,
            bag_play,
            on_play_exit,
        ]
    )
    return nodes


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
            DeclareLaunchArgument(
                'odom_topic',
                default_value='/odom',
                description='Cartographer odom input; use /odom_noisy for slip-injected wheel odom from bag.',
            ),
            DeclareLaunchArgument(
                'fix_odom_frames_for_cartographer',
                default_value='auto',
                description='auto|true|false: rewrite odom frame_id/child_frame for Cartographer+TF (auto=on for /odom_noisy).',
            ),
            DeclareLaunchArgument(
                'play_tf',
                default_value='auto',
                description='auto|true|false: play /tf from bag. auto=false for /odom_noisy (use odom_to_tf instead).',
            ),
            DeclareLaunchArgument('imu_topic', default_value='/imu'),
            OpaqueFunction(function=launch_setup),
        ]
    )
