# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""
Cartographer mapping + rviz（without Gazebo）, for tb3_gazebo_industrial_warehouse。

Save: Ctrl+C in this terminal, cartographer_node will write the map to pbstream_out（see -save_state_filename）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_carto_share = get_package_share_directory('turtlebot3_cartographer')
    demo_share = get_package_share_directory('dynamic_carto_demo')
    rviz_config = os.path.join(tb3_carto_share, 'rviz', 'tb3_cartographer.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    pbstream_out = LaunchConfiguration('pbstream_out')
    mapping_config_dir = LaunchConfiguration('mapping_config_dir')
    mapping_config_basename = LaunchConfiguration('mapping_config_basename')
    scan_topic_in = LaunchConfiguration('scan_topic_in')
    odom_topic_in = LaunchConfiguration('odom_topic_in')
    imu_topic_in = LaunchConfiguration('imu_topic_in')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', mapping_config_dir,
            '-configuration_basename', mapping_config_basename,
            '-save_state_filename', pbstream_out,
        ],
        remappings=[
            ('scan', scan_topic_in),
            ('odom', odom_topic_in),
            ('imu', imu_topic_in),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'resolution': 0.05}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'pbstream_out',
            default_value='/tmp/industrial_map.pbstream',
        ),
        DeclareLaunchArgument(
            'mapping_config_dir',
            default_value=os.path.join(demo_share, 'config'),
        ),
        DeclareLaunchArgument('mapping_config_basename', default_value='tb3_warehouse_lds_2d_mapping.lua'),
        DeclareLaunchArgument('scan_topic_in', default_value='/scan'),
        DeclareLaunchArgument('odom_topic_in', default_value='/odom'),
        DeclareLaunchArgument('imu_topic_in', default_value='/imu'),
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
