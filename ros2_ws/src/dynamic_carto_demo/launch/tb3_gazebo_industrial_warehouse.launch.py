# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""
TB3 Waffle + industrial_warehouse_classic.world（AWS GAZEBO_MODEL_PATH + 官方 spawn_turtlebot3）。

Default spawn point is the same as the previous one when it can be displayed/controlled normally. If you want to move to another empty place, see the coordinates in Gazebo and then change x_pose/y_pose.

Remote control (open another terminal):
  export TURTLEBOT3_MODEL=waffle
  source /opt/ros/humble/setup.bash
  ros2 run turtlebot3_teleop teleop_keyboard（default model is waffle）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    launch_file_dir = get_package_share_directory('turtlebot3_gazebo') + '/launch'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('dynamic_carto_demo')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')

    aws_models_default = os.path.join(
        os.path.expanduser('~'), 'third_party',
        'aws-robomaker-small-warehouse-world', 'models')
    world_default = os.path.join(pkg_share, 'worlds', 'industrial_warehouse_classic.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='1.7')
    y_pose = LaunchConfiguration('y_pose', default='-10.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')
    world = LaunchConfiguration('world', default=world_default)
    aws_models = LaunchConfiguration('aws_warehouse_models', default=aws_models_default)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    model_name = 'waffle'
    model_sdf_path = os.path.join(tb3_share, 'models', f'turtlebot3_{model_name}', 'model.sdf')
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model_name,
            '-file', model_sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw_pose,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='1.7'),
        DeclareLaunchArgument('y_pose', default_value='-10.0'),
        DeclareLaunchArgument(
            'yaw_pose',
            default_value='0.0',
            description='Initial yaw (rad). Try 0, 1.5708, 3.1416, -1.5708.',
        ),
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument('aws_warehouse_models', default_value=aws_models_default),
        AppendEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=aws_models,
            prepend=True,
        ),
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
    ])
