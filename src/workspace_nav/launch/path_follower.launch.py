#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare('workspace_nav')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    path_topic = LaunchConfiguration('path_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    start_converter = LaunchConfiguration('start_converter')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                nav_share,
                'config',
                'path_follower.yaml',
            ]),
            description='Full path to the path follower parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true.',
        ),
        DeclareLaunchArgument(
            'path_topic',
            default_value='/planned_global_path',
            description='nav_msgs/Path topic to follow.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel_nav',
            description='Twist topic consumed by the thruster converter.',
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='2.0',
            description='Pure-pursuit lookahead distance in meters.',
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.8',
            description='Distance in meters used to stop at the final path point.',
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='1.2',
            description='Maximum forward speed command.',
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='0.5',
            description='Maximum yaw rate command.',
        ),
        DeclareLaunchArgument(
            'start_converter',
            default_value='true',
            description='Start workspace_ros converter to drive thrusters from cmd_vel.',
        ),
        Node(
            package='workspace_nav',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'path_topic': path_topic,
                    'cmd_vel_topic': cmd_vel_topic,
                    'lookahead_distance': ParameterValue(lookahead_distance, value_type=float),
                    'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),
                    'max_linear_speed': ParameterValue(max_linear_speed, value_type=float),
                    'max_angular_speed': ParameterValue(max_angular_speed, value_type=float),
                },
            ],
        ),
        Node(
            package='workspace_ros',
            executable='converter',
            name='converter',
            output='screen',
            condition=IfCondition(start_converter),
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ])
