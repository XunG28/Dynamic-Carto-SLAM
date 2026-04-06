# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""
Offline localization with KISS-ICP + EKF fused odometry -> Cartographer.

Single bag play: odom_noisy -> fix -> EKF; scan -> points -> KISS; EKF out /odometry/filtered -> Carto.

Usage:
  ros2 launch dynamic_carto_demo offline_localization_fused.launch.py \\
    bag:=/path/to/bag \\
    pbstream_in:=/path/map.pbstream \\
    bag_out:=/path/out_fused
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
    kiss_cfg = os.path.join(demo_share, 'config', 'kiss_icp_tb3_warehouse.yaml')
    ekf_cfg = os.path.join(demo_share, 'config', 'tb3_wheel_kiss_ekf_tuned_v2.yaml')

    bag = LaunchConfiguration('bag').perform(context)
    bag_out = LaunchConfiguration('bag_out').perform(context)
    pbstream_in = LaunchConfiguration('pbstream_in').perform(context)
    use_sim_str = LaunchConfiguration('use_sim_time').perform(context)
    sim = _use_sim_flag(use_sim_str)

    config_dir = LaunchConfiguration('config_dir').perform(context)
    config_basename = LaunchConfiguration('config_basename').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)

    carto_odom_topic = '/odometry/filtered'

    odom_fix = Node(
        package='dynamic_carto_demo',
        executable='odom_child_frame_fix',
        name='odom_child_frame_fix',
        output='screen',
        parameters=[
            {
                'use_sim_time': sim,
                'input_topic': '/odom_noisy',
                'output_topic': '/odom_noisy_carto',
                'frame_id': 'odom',
                'child_frame_id': 'base_footprint',
            }
        ],
    )

    scan_to_cloud = Node(
        package='dynamic_carto_demo',
        executable='laserscan_to_pointcloud2',
        name='laserscan_to_pointcloud2',
        output='screen',
        parameters=[{'use_sim_time': sim}],
    )

    kiss_icp = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        parameters=[
            kiss_cfg,
            {'use_sim_time': sim},
        ],
        remappings=[
            ('pointcloud_topic', '/points'),
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_cfg,
            {'use_sim_time': sim},
            # Publish odom->base_footprint TF from the fused estimate.
            # During bag playback we do NOT play the bag-recorded /tf to avoid mixing "true odom TF"
            # with the noisy/filtered odometry used by Cartographer.
            {'publish_tf': True},
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

    record_topics = [
        '/clock',
        '/tf',
        '/tf_static',
        '/scan',
        '/odom',
        '/odom_noisy',
        '/odom_noisy_carto',
        '/imu',
        '/points',
        '/kiss/odometry',
        '/odometry/filtered',
        '/tracked_pose',
        '/eval/ground_truth/pose',
        '/eval/ground_truth/odom',
    ]
    if not sim:
        record_topics = [t for t in record_topics if t != '/clock']
    record_cmd.extend(record_topics)

    play_topics = [
        '/tf_static',
        '/scan',
        '/odom',
        '/odom_noisy',
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
        odom_fix,
        scan_to_cloud,
        kiss_icp,
        ekf,
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
            DeclareLaunchArgument(
                'bag',
                description='Path to input rosbag2 directory (must include /odom_noisy, /scan, /imu, /tf, /clock if use_sim_time:=true).',
            ),
            DeclareLaunchArgument(
                'pbstream_in',
                default_value='/tmp/offline_map.pbstream',
                description='Cartographer load_state pbstream (same map as offline_localization).',
            ),
            DeclareLaunchArgument(
                'bag_out',
                default_value='/tmp/offline_loc_fused',
                description='Output directory for recorded localization bag.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='true: bag provides /clock; nodes use sim time.',
            ),
            DeclareLaunchArgument(
                'config_dir',
                default_value=os.path.join(demo_share, 'config'),
                description='Cartographer -configuration_directory.',
            ),
            DeclareLaunchArgument(
                'config_basename',
                default_value='tb3_warehouse_lds_2d_localization.lua',
                description='Cartographer -configuration_basename (pure localization Lua).',
            ),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('imu_topic', default_value='/imu'),
            OpaqueFunction(function=launch_setup),
        ]
    )
