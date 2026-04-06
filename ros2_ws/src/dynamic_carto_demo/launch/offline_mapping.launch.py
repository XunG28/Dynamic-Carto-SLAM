# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""
Offline mapping on a rosbag (no Gazebo).

Usage:
  ros2 launch dynamic_carto_demo offline_mapping.launch.py bag:=/path/to/bag pbstream_out:=/tmp/map.pbstream
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    pbstream_out = LaunchConfiguration('pbstream_out', default='/tmp/offline_map.pbstream')

    # Default to TB3 mapping lua (apt).
    tb3_carto_share = get_package_share_directory('turtlebot3_cartographer')
    config_dir = LaunchConfiguration('config_dir', default=os.path.join(tb3_carto_share, 'config'))
    config_basename = LaunchConfiguration('config_basename', default='turtlebot3_lds_2d.lua')

    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    imu_topic = LaunchConfiguration('imu_topic', default='/imu')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_basename,
            '-save_state_filename', pbstream_out,
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
        parameters=[{'use_sim_time': True, 'resolution': 0.05}],
    )

    # If the bag already contains `/clock`, do not add `--clock`, otherwise dual clocks may cause time jumps.
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to rosbag2 (directory).'),
        DeclareLaunchArgument('pbstream_out', default_value='/tmp/offline_map.pbstream'),
        DeclareLaunchArgument('config_dir', default_value=os.path.join(tb3_carto_share, 'config')),
        DeclareLaunchArgument('config_basename', default_value='turtlebot3_lds_2d.lua'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        cartographer_node,
        occupancy_grid_node,
        bag_play,
    ])

