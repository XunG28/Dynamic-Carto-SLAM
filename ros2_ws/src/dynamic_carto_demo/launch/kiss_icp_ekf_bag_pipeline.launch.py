#!/usr/bin/env python3
"""
Offline pipeline: bag + odom frame fix + LaserScan->PointCloud2 + KISS-ICP + EKF
-> /odometry/filtered (for Cartographer odom_topic).

Usage:
  ros2 launch dynamic_carto_demo kiss_icp_ekf_bag_pipeline.launch.py \\
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
    ekf_cfg = os.path.join(pkg, 'config', 'tb3_wheel_kiss_ekf.yaml')

    bag = LaunchConfiguration('bag')

    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--clock'],
        output='screen',
    )

    odom_fix = Node(
        package='dynamic_carto_demo',
        executable='odom_child_frame_fix',
        name='odom_child_frame_fix',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
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

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_cfg,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to rosbag2 directory.'),
        play,
        odom_fix,
        scan_to_cloud,
        kiss_icp,
        ekf,
    ])
