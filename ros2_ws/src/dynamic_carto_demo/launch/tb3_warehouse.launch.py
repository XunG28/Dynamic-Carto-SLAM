#!/usr/bin/env python3

"""
TurtleBot3 (selectable model) + warehouse world + optional ground truth publisher.

Defaults:
- TURTLEBOT3_MODEL: waffle
- World: industrial_warehouse_classic.world
- Automatically start gazebo_ground_truth_node (/eval/ground_truth/*, use_sim_time=true)

You can override via launch arguments:
- model:=waffle|burger|waffle_pi ...
- world:=<path/to/world.world>
- x_pose:=... y_pose:=... yaw_pose:=...
- use_ground_truth:=true|false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TB3 model selected via launch argument (affects TURTLEBOT3_MODEL and entity name)
    model = LaunchConfiguration('model', default='waffle')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('dynamic_carto_demo')
    pkg_actor_proxy_share = get_package_share_directory('gazebo_actor_proxy_follower')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')

    aws_models_default = os.path.join(
        os.path.expanduser('~'),
        'third_party',
        'aws-robomaker-small-warehouse-world',
        'models',
    )
    world_classic = os.path.join(pkg_share, 'worlds', 'industrial_warehouse_classic.world')
    world_actor_with_proxy = os.path.join(pkg_share, 'worlds', 'industrial_warehouse_classic_actor_with_proxy.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_actor_people = LaunchConfiguration('use_actor_people', default='false')
    # Default spawn: match GT map generation init_robot_x/y, facing +X
    x_pose = LaunchConfiguration('x_pose', default='1.7')
    y_pose = LaunchConfiguration('y_pose', default='-9.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='1.5708')
    world = LaunchConfiguration('world', default=world_classic)
    aws_models = LaunchConfiguration('aws_warehouse_models', default=aws_models_default)
    use_ground_truth = LaunchConfiguration('use_ground_truth', default='true')
    use_noisy_odom = LaunchConfiguration('use_noisy_odom', default='true')
    use_world_odom_bridge = LaunchConfiguration('use_world_odom_bridge', default='true')

    # Add this package's models and AWS warehouse models to GAZEBO_MODEL_PATH
    local_models = os.path.join(pkg_share, 'models')
    # Add actor-proxy follower plugin path to GAZEBO_PLUGIN_PATH
    actor_proxy_prefix = os.path.dirname(os.path.dirname(pkg_actor_proxy_share))
    actor_proxy_lib = os.path.join(actor_proxy_prefix, 'lib')

    # TURTLEBOT3_MODEL environment variable for gazebo_ros / teleop, default waffle
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    # Robot state publisher from turtlebot3_gazebo
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Use TurtleBot3 model SDF from turtlebot3_gazebo (default waffle path as example)
    model_sdf_path = os.path.join(
        tb3_share,
        'models',
        'turtlebot3_waffle',
        'model.sdf',
    )

    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            model,
            '-file',
            model_sdf_path,
            '-x',
            x_pose,
            '-y',
            y_pose,
            '-z',
            '0.01',
            '-Y',
            yaw_pose,
        ],
        output='screen',
    )

    # Ground truth node (enabled by default, can be disabled via use_ground_truth:=false)
    ground_truth_node = Node(
        package='dynamic_carto_demo',
        executable='gazebo_gt_publisher',
        name='gazebo_ground_truth_publisher',
        output='screen',
        parameters=[{
            # Keep sim time consistent with Gazebo
            'use_sim_time': use_sim_time,
            # Avoid TF conflict with wheel odom (odom -> base_footprint)
            'output_frame': 'world',
            'child_frame': 'gt_base_footprint',
            'publish_tf': True,
        }],
        condition=IfCondition(use_ground_truth),
    )

    odom_noisy_node = Node(
        package='dynamic_carto_demo',
        executable='odom_noisy',
        name='odom_noisy',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/odom',
            'output_topic': '/odom_noisy',
            'output_frame': 'odom_noisy',
            'output_child_frame': 'base_footprint_noisy',
            # Event-style slip: quiet straights (|wz| below threshold), bursts when turning ~0.1 rad/s.
            # v≈0.1 m/s, corners |wz|≈0.1: use threshold slightly below 0.1.
            'min_abs_yaw_rate': 0.065,
            'drift_pos_rw': 0.012,
            'drift_yaw_rw': 0.008,
            'slip_burst_prob': 0.02,
            'slip_burst_xy': 0.22,
            'slip_burst_yaw': 0.06,
            'seed': 42,
        }],
        condition=IfCondition(use_noisy_odom),
    )

    world_odom_bridge_node = Node(
        package='dynamic_carto_demo',
        executable='world_odom_bridge',
        name='world_odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # dynamic: continuously align frames
            # static_init: align once, then show accumulating drift
            'mode': 'dynamic',
            'gt_odom_topic': '/eval/ground_truth/odom',
            'odom_topic': '/odom',
            'odom_noisy_topic': '/odom_noisy',
            'world_frame': 'world',
            'odom_frame': 'odom',
            'odom_noisy_frame': 'odom_noisy',
        }],
        condition=IfCondition(use_world_odom_bridge),
    )

    return LaunchDescription([
        # Common launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='waffle',
            description='TurtleBot3 model type (e.g., waffle, burger).',
        ),
        DeclareLaunchArgument('x_pose', default_value='1.7'),
        DeclareLaunchArgument('y_pose', default_value='-9.0'),
        DeclareLaunchArgument(
            'yaw_pose',
            default_value='1.5708',
            description='Initial yaw (rad). Try 0, 1.5708, 3.1416, -1.5708.',
        ),
        DeclareLaunchArgument(
            'use_actor_people',
            default_value='false',
            description='If true, load industrial_warehouse_classic_actor_with_proxy.world (animated walking people + collision proxy).',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_classic,
            description='World SDF to load.',
        ),
        DeclareLaunchArgument(
            'aws_warehouse_models',
            default_value=aws_models_default,
            description='Path to AWS RoboMaker warehouse models.',
        ),
        DeclareLaunchArgument(
            'use_ground_truth',
            default_value='true',
            description='If true, start gazebo_ground_truth_node to publish /eval/ground_truth/*.',
        ),
        DeclareLaunchArgument(
            'use_noisy_odom',
            default_value='true',
            description='If true, start odom_noisy node to publish /odom_noisy with frame_id=odom_noisy.',
        ),
        DeclareLaunchArgument(
            'use_world_odom_bridge',
            default_value='true',
            description='If true, broadcast TF world->odom and world->odom_noisy for RViz visualization.',
        ),

        # Gazebo model search paths
        AppendEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=local_models,
            prepend=True,
        ),
        AppendEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=aws_models,
            prepend=True,
        ),
        AppendEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=actor_proxy_lib,
            prepend=True,
        ),

        # If enabled, override world to include animated people + collision proxy
        SetLaunchConfiguration(
            name='world',
            value=world_actor_with_proxy,
            condition=IfCondition(use_actor_people),
        ),

        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        ground_truth_node,
        odom_noisy_node,
        world_odom_bridge_node,
    ])

