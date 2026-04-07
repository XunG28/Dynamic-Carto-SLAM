# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""
Offline localization with Bayesian scan filter pre-processing.

Pipeline
--------
  rosbag play  →  scan_filter_node (/scan → /scan_filtered)
                                        ↓
                             cartographer_node (uses /scan_filtered)
                                        ↓
                             rosbag record (saves /tracked_pose + other topics)

This launch file is a thin wrapper around offline_localization.launch.py that
inserts the scan_filter_node and re-maps Cartographer's scan input to
/scan_filtered.  All arguments from the parent launch are forwarded unchanged;
three new arguments are added for the scan filter.

Usage
-----
  ros2 launch dynamic_carto_demo offline_localization_filtered.launch.py \\
      bag:=/path/to/dynamic_v3 \\
      pbstream_in:=/path/to/static_v2.pbstream \\
      bag_out:=/tmp/loc_dynamic_v3_filtered

  # Use median reference for harder dynamics:
  ros2 launch dynamic_carto_demo offline_localization_filtered.launch.py \\
      bag:=... pbstream_in:=... bag_out:=... \\
      filter_reference_mode:=median filter_buffer_size:=10

Remapping note
--------------
  Cartographer's scan input is remapped to /scan_filtered by setting the
  launch argument  scan_topic:=/scan_filtered  (passed to the inner launch).
  The filter node always subscribes to /scan (raw) and publishes /scan_filtered.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(s: str) -> bool:
    return s.lower() in ('true', '1', 'yes')


# ---------------------------------------------------------------------------
def launch_setup(context, *args, **kwargs):
    demo_share    = get_package_share_directory('dynamic_carto_demo')
    qos_overrides = os.path.join(demo_share, 'config', 'record_qos_overrides.yaml')
    filter_params = os.path.join(demo_share, 'config', 'scan_filter_params.yaml')

    # ── Common arguments (mirrors offline_localization.launch.py) ─────────
    bag          = LaunchConfiguration('bag').perform(context)
    bag_out      = LaunchConfiguration('bag_out').perform(context)
    pbstream_in  = LaunchConfiguration('pbstream_in').perform(context)
    use_sim_str  = LaunchConfiguration('use_sim_time').perform(context)
    sim          = _bool(use_sim_str)

    config_dir      = LaunchConfiguration('config_dir').perform(context)
    config_basename = LaunchConfiguration('config_basename').perform(context)
    odom_topic      = LaunchConfiguration('odom_topic').perform(context)
    imu_topic       = LaunchConfiguration('imu_topic').perform(context)
    play_tf_mode    = LaunchConfiguration('play_tf').perform(context).lower()
    fix_arg         = LaunchConfiguration('fix_odom_frames_for_cartographer').perform(context).lower()

    # ── Scan filter arguments ──────────────────────────────────────────────
    filter_ref_mode   = LaunchConfiguration('filter_reference_mode').perform(context)
    filter_buf_size   = LaunchConfiguration('filter_buffer_size').perform(context)
    filter_dyn_thr    = LaunchConfiguration('filter_dyn_threshold').perform(context)
    filter_motion_grd = LaunchConfiguration('filter_motion_guard_frac').perform(context)

    # Cartographer always uses the filtered topic
    carto_scan_topic = '/scan_filtered'

    # ── odom_noisy fix (same logic as offline_localization) ────────────────
    if fix_arg == 'auto':
        use_odom_fix = odom_topic == '/odom_noisy'
    elif fix_arg in ('false', '0', 'no'):
        use_odom_fix = False
    else:
        use_odom_fix = odom_topic == '/odom_noisy'

    carto_odom_topic = '/odom_noisy_carto' if use_odom_fix else odom_topic

    if play_tf_mode == 'auto':
        play_tf = not use_odom_fix
    else:
        play_tf = play_tf_mode in ('true', '1', 'yes')

    # ── Nodes ──────────────────────────────────────────────────────────────

    odom_fix_node = None
    if use_odom_fix:
        odom_fix_node = Node(
            package='dynamic_carto_demo',
            executable='odom_child_frame_fix',
            name='odom_child_frame_fix',
            output='screen',
            parameters=[{
                'use_sim_time': sim,
                'input_topic': odom_topic,
                'output_topic': carto_odom_topic,
                'frame_id': 'odom',
                'child_frame_id': 'base_footprint',
            }],
        )

    odom_to_tf_node = None
    if not play_tf:
        odom_to_tf_node = Node(
            package='dynamic_carto_demo',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
            parameters=[{
                'use_sim_time': sim,
                'input_topic': carto_odom_topic,
            }],
        )

    # Scan filter node – reads /scan, publishes /scan_filtered
    scan_filter_node = Node(
        package='scan_filter',
        executable='scan_filter_node',
        name='scan_filter_node',
        output='screen',
        parameters=[
            filter_params,                   # base YAML
            {                                # CLI overrides win
                'use_sim_time': sim,
                'reference_mode':      filter_ref_mode,
                'buffer_size':         int(filter_buf_size),
                'dyn_threshold':       float(filter_dyn_thr),
                'motion_guard_frac':   float(filter_motion_grd),
            },
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
            ('scan', carto_scan_topic),   # ← key remap: use filtered scan
            ('odom', carto_odom_topic),
            ('imu',  imu_topic),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': sim, 'resolution': 0.05}],
    )

    # ── Bag record ─────────────────────────────────────────────────────────
    record_cmd = [
        'ros2', 'bag', 'record',
        '-o', bag_out,
        '--qos-profile-overrides-path', qos_overrides,
    ]
    if sim:
        record_cmd.append('--use-sim-time')

    odom_record_topics = ['/odom']
    if odom_topic not in odom_record_topics:
        odom_record_topics.append(odom_topic)
    if use_odom_fix and carto_odom_topic not in odom_record_topics:
        odom_record_topics.append(carto_odom_topic)

    record_topics = [
        '/tf', '/tf_static',
        '/scan', '/scan_filtered',
        '/imu',
        '/tracked_pose',
        '/eval/ground_truth/pose',
        '/eval/ground_truth/odom',
    ]
    if sim:
        record_topics = ['/clock'] + record_topics
    idx = record_topics.index('/scan') + 1
    for t in reversed(odom_record_topics):
        record_topics.insert(idx, t)
    record_cmd.extend(record_topics)

    # ── Bag play ───────────────────────────────────────────────────────────
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

    play_cmd  = ['ros2', 'bag', 'play', bag, '--topics'] + play_topics
    rosbag_record = ExecuteProcess(cmd=record_cmd, output='screen')
    bag_play      = ExecuteProcess(cmd=play_cmd,   output='screen')

    on_play_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[EmitEvent(event=Shutdown(reason='bag play finished'))],
        )
    )

    # ── Assemble node list ─────────────────────────────────────────────────
    nodes = []
    if odom_fix_node is not None:
        nodes.append(odom_fix_node)
    if odom_to_tf_node is not None:
        nodes.append(odom_to_tf_node)
    nodes.extend([
        scan_filter_node,
        cartographer_node,
        occupancy_grid_node,
        rosbag_record,
        bag_play,
        on_play_exit,
    ])
    return nodes


# ---------------------------------------------------------------------------
def generate_launch_description():
    demo_share = get_package_share_directory('dynamic_carto_demo')
    _ = os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

    return LaunchDescription([
        # ── Standard arguments (same as offline_localization) ─────────────
        DeclareLaunchArgument('bag', description='Path to input rosbag2 directory.'),
        DeclareLaunchArgument('pbstream_in', default_value='/tmp/offline_map.pbstream'),
        DeclareLaunchArgument(
            'bag_out',
            default_value='/tmp/offline_loc_filtered_bag',
            description='Output directory for the filtered localization bag.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_dir', default_value=os.path.join(demo_share, 'config')),
        DeclareLaunchArgument(
            'config_basename',
            default_value='tb3_warehouse_lds_2d_localization.lua',
        ),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'fix_odom_frames_for_cartographer', default_value='auto'),
        DeclareLaunchArgument('play_tf', default_value='auto'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),

        # ── Scan filter arguments ─────────────────────────────────────────
        DeclareLaunchArgument(
            'filter_reference_mode',
            default_value='last',
            description='"last" (single-frame lookback) or "median" (buffer median).',
        ),
        DeclareLaunchArgument(
            'filter_buffer_size',
            default_value='5',
            description='Number of recent scans in the rolling reference buffer.',
        ),
        DeclareLaunchArgument(
            'filter_dyn_threshold',
            default_value='0.65',
            description='P(dynamic) threshold above which a beam is removed.',
        ),
        DeclareLaunchArgument(
            'filter_motion_guard_frac',
            default_value='0.40',
            description='Fraction of changing beams that triggers motion-guard bypass.',
        ),

        OpaqueFunction(function=launch_setup),
    ])
