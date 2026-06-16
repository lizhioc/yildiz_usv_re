#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Launch file for bringing up the Nav2 navigation stack for the RoboBoat project.
#  It declares configurable launch arguments (map, params_file, use_sim_time, autostart, log_level)
#  and includes the upstream nav2_bringup bringup_launch.py with those arguments resolved.
#  The configuration ensures Nav2 runs using the provided map and parameter set for both
#  simulation and real-world deployments.
# ----------------------------------------------------------------------------------------------- #

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

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
    map_config = resolve_default_map(package_name)

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_config,
        description='Full path to static map yaml file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_config,
        description='Full path to the ROS2 parameters file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for Nav2 nodes (e.g. debug, info, warn, error)'
    )

    bringup_launch_path = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'log_level': log_level
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(bringup_launch)

    return ld
