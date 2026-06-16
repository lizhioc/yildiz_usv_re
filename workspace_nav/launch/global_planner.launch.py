#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_MAP_FILENAME = 'roboboat_manual_map_2_nav.yaml'


def resolve_default_map(package_name):
    env_map = os.environ.get('YILDIZ_MAP_FILE')
    if env_map:
        return str(Path(env_map).expanduser().resolve())

    source_map = Path.home() / 'yildiz_ws' / 'src' / 'YILDIZ-USV' / 'maps' / DEFAULT_MAP_FILENAME
    candidates = [source_map]

    try:
        package_share_path = Path(get_package_share_directory(package_name))
        candidates.append(package_share_path / 'maps' / DEFAULT_MAP_FILENAME)
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    return str(source_map)


def generate_launch_description():
    package_name = 'workspace_nav'
    package_share = FindPackageShare(package_name)

    nav2_config = PathJoinSubstitution([package_share, 'config', 'nav2_params.yaml'])
    gps_map_tf_config = PathJoinSubstitution([package_share, 'config', 'gps_map_tf.yaml'])
    map_config = resolve_default_map(package_name)

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_amcl = LaunchConfiguration('use_amcl')
    use_gps_map_tf = LaunchConfiguration('use_gps_map_tf')
    use_rviz_goal_planner = LaunchConfiguration('use_rviz_goal_planner')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=IfCondition(use_amcl),
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    gps_map_tf = Node(
        package=package_name,
        executable='gps_map_tf',
        name='gps_map_tf',
        output='screen',
        condition=IfCondition(use_gps_map_tf),
        parameters=[
            gps_map_tf_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_goal_planner = Node(
        package=package_name,
        executable='rviz_goal_planner',
        name='rviz_goal_planner',
        output='screen',
        condition=IfCondition(use_rviz_goal_planner),
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'goal_topic': '/goal_pose',
                'path_topic': '/planned_global_path',
                'action_name': 'compute_path_to_pose',
                'planner_id': 'GridBased',
            }
        ],
    )

    lifecycle_manager_without_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_global_planner',
        output='screen',
        condition=UnlessCondition(use_amcl),
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server', 'planner_server'],
            }
        ],
    )

    lifecycle_manager_with_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_global_planner',
        output='screen',
        condition=IfCondition(use_amcl),
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server', 'amcl', 'planner_server'],
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_config,
            description='Full path to static map yaml file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_config,
            description='Full path to the ROS2 parameters file',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup map_server and planner_server',
        ),
        DeclareLaunchArgument(
            'use_amcl',
            default_value='false',
            description='Use AMCL to estimate map->odom from the static map and live laser scan',
        ),
        DeclareLaunchArgument(
            'use_gps_map_tf',
            default_value='false',
            description='Use GPS/odometry plus the map georeference config to publish map->odom',
        ),
        DeclareLaunchArgument(
            'use_rviz_goal_planner',
            default_value='true',
            description='Listen to RViz /goal_pose and publish /planned_global_path',
        ),
        gps_map_tf,
        rviz_goal_planner,
        map_server,
        amcl,
        planner_server,
        lifecycle_manager_without_amcl,
        lifecycle_manager_with_amcl,
    ])
