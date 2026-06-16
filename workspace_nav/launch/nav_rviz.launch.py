#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('workspace_nav')

    rviz_config = LaunchConfiguration('rviz_config')
    display = LaunchConfiguration('display')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                package_share,
                'rviz',
                'static_navigation.rviz',
            ]),
            description='Full path to the static navigation RViz configuration.',
        ),
        DeclareLaunchArgument(
            'display',
            default_value=':0',
            description='Display used by RViz2 when running on the Jetson desktop.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true.',
        ),
        SetEnvironmentVariable('DISPLAY', display),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_navigation',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
