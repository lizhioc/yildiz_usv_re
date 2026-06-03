#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('workspace_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    display = LaunchConfiguration('display')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock when true.'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            package_share,
            'config',
            'mapper_params_online_async.yaml'
        ]),
        description='Full path to the slam_toolbox parameter file.'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with the SLAM mapping display configuration.'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            package_share,
            'rviz',
            'slam_mapping.rviz'
        ]),
        description='Full path to the RViz2 configuration file.'
    )

    declare_display = DeclareLaunchArgument(
        'display',
        default_value=':0',
        description='Display used by RViz2 when running on the Jetson desktop.'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_rviz,
        declare_rviz_config,
        declare_display,
        SetEnvironmentVariable('DISPLAY', display),
        slam_toolbox_node,
        rviz_node
    ])
