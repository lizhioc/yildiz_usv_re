#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Launch file for initializing the Gazebo Garden simulation of the RoboBoat.
#  It sets up environment paths, generates the robot description from a Xacro file,
#  spawns the robot into the simulation world, and launches core ROS 2 publisher nodes.
#  The file also bridges key Gazebo topics—such as clock, sensors, and thruster commands—
#  enabling seamless ROS 2 interaction with the simulated environment.
# ----------------------------------------------------------------------------------------------- #

from launch_ros.descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix

def generate_launch_description():
    package_name = 'workspace_gz'

    package_prefix = FindPackagePrefix(package_name)
    package_share = FindPackageShare(package_name)

    plugin_path = PathJoinSubstitution([package_prefix, 'lib'])
    world_path = PathJoinSubstitution([package_share, 'worlds', 'world.sdf'])
    model_path = PathJoinSubstitution([package_share, 'models'])
    xacro_path = PathJoinSubstitution([package_share, 'description', 'roboboat', 'roboboat.xacro'])
    headless = LaunchConfiguration('headless')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server without the GUI client.'
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=plugin_path
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            condition=UnlessCondition(headless),
            output='screen'
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', world_path],
            condition=IfCondition(headless),
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description}
            ],
            output='screen'
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description}
            ],
            output='screen'
        ),

        Node(
            package=package_name,
            executable='garden_bridge',
            name='garden_bridge',
            parameters=[
                {'use_sim_time': True}
            ],
            output='screen'
        ),
    ])
