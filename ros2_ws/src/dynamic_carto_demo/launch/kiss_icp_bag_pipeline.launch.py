#!/usr/bin/env python3
"""
Offline: play bag (--clock) + LaserScan->PointCloud2 + KISS-ICP.

TF warnings from kiss_icp_node's tf2 buffer during bag replay are common; /kiss/odometry still publishes.

Usage:
  ros2 launch dynamic_carto_demo kiss_icp_bag_pipeline.launch.py \\
    bag:=/path/to/bag_dir
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dynamic_carto_demo')
    kiss_cfg = os.path.join(pkg, 'config', 'kiss_icp_tb3_warehouse.yaml')

    bag = LaunchConfiguration('bag')
    visualize = LaunchConfiguration('visualize', default='false')

    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--clock'],
        output='screen',
    )

    scan_to_cloud = Node(
        package='dynamic_carto_demo',
        executable='laserscan_to_pointcloud2',
        name='laserscan_to_pointcloud2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    kiss_icp = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        parameters=[
            kiss_cfg,
            {'use_sim_time': True},
        ],
        remappings=[
            ('pointcloud_topic', '/points'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to rosbag2 directory.'),
        DeclareLaunchArgument(
            'visualize',
            default_value='false',
            description='Reserved for future RViz.',
        ),
        play,
        scan_to_cloud,
        kiss_icp,
    ])
