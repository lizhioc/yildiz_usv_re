#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Launch file for initializing the localization stack of the RoboBoat.
#  It fuses IMU, GPS, and odometry data using the robot_localization package
#  to provide a continuous state estimate for navigation and control.
#  The file also republishes sensor data with fixed covariances and sets up
#  static transforms required for consistent frame alignment across the system.
# ----------------------------------------------------------------------------------------------- #

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'workspace_ros'
    package_share = FindPackageShare(package_name)

    ekf_path = PathJoinSubstitution([package_share, 'config', 'ekf.yaml'])
    navsat_path = PathJoinSubstitution([package_share, 'config', 'navsat.yaml'])
    static_transform_path = PathJoinSubstitution([package_share, 'config', 'static_transform.yaml'])
    publish_map_to_odom = LaunchConfiguration('publish_map_to_odom')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_z = LaunchConfiguration('map_to_odom_z')
    map_to_odom_roll = LaunchConfiguration('map_to_odom_roll')
    map_to_odom_pitch = LaunchConfiguration('map_to_odom_pitch')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')
    publish_sensor_static_tf = LaunchConfiguration('publish_sensor_static_tf')
    gps_horizontal_stddev = LaunchConfiguration('gps_horizontal_stddev')
    gps_vertical_stddev = LaunchConfiguration('gps_vertical_stddev')
    imu_orientation_stddev = LaunchConfiguration('imu_orientation_stddev')
    imu_angular_velocity_stddev = LaunchConfiguration('imu_angular_velocity_stddev')
    imu_linear_acceleration_stddev = LaunchConfiguration('imu_linear_acceleration_stddev')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_map_to_odom',
            default_value='false',
            description='Publish a static map->odom transform. Keep false while SLAM or localization publishes map->odom.'
        ),
        DeclareLaunchArgument(
            'map_to_odom_x',
            default_value='0.0',
            description='Static map->odom x used only when publish_map_to_odom is true.'
        ),
        DeclareLaunchArgument(
            'map_to_odom_y',
            default_value='0.0',
            description='Static map->odom y used only when publish_map_to_odom is true.'
        ),
        DeclareLaunchArgument(
            'map_to_odom_z',
            default_value='0.0',
            description='Static map->odom z used only when publish_map_to_odom is true.'
        ),
        DeclareLaunchArgument(
            'map_to_odom_roll',
            default_value='0.0',
            description='Static map->odom roll used only when publish_map_to_odom is true.'
        ),
        DeclareLaunchArgument(
            'map_to_odom_pitch',
            default_value='0.0',
            description='Static map->odom pitch used only when publish_map_to_odom is true.'
        ),
        DeclareLaunchArgument(
            'map_to_odom_yaw',
            default_value='0.0',
            description='Static map->odom yaw used only when publish_map_to_odom is true.'
        ),
        DeclareLaunchArgument(
            'publish_sensor_static_tf',
            default_value='false',
            description='Publish sensor static TF from workspace_ros/config/static_transform.yaml. Keep false when robot_state_publisher is running.'
        ),
        DeclareLaunchArgument(
            'gps_horizontal_stddev',
            default_value='0.05',
            description='Horizontal GPS standard deviation in meters used for NavSatFix covariance.'
        ),
        DeclareLaunchArgument(
            'gps_vertical_stddev',
            default_value='0.20',
            description='Vertical GPS standard deviation in meters used for NavSatFix covariance.'
        ),
        DeclareLaunchArgument(
            'imu_orientation_stddev',
            default_value='0.01',
            description='IMU orientation standard deviation in radians used for Imu covariance.'
        ),
        DeclareLaunchArgument(
            'imu_angular_velocity_stddev',
            default_value='0.0032',
            description='IMU angular velocity standard deviation in rad/s used for Imu covariance.'
        ),
        DeclareLaunchArgument(
            'imu_linear_acceleration_stddev',
            default_value='0.20',
            description='IMU linear acceleration standard deviation in m/s^2 used for Imu covariance.'
        ),

        Node(
            package=package_name,
            executable='imu_covariance_repub',
            name='imu_covariance_repub',
            parameters=[{
                'use_sim_time': True,
                'orientation_stddev': ParameterValue(imu_orientation_stddev, value_type=float),
                'angular_velocity_stddev': ParameterValue(
                    imu_angular_velocity_stddev, value_type=float
                ),
                'linear_acceleration_stddev': ParameterValue(
                    imu_linear_acceleration_stddev, value_type=float
                ),
            }],
        ),

        Node(
            package=package_name,
            executable='gps_covariance_repub',
            name='gps_covariance_repub',
            parameters=[{
                'use_sim_time': True,
                'horizontal_stddev': ParameterValue(gps_horizontal_stddev, value_type=float),
                'vertical_stddev': ParameterValue(gps_vertical_stddev, value_type=float),
            }],
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_path, {'use_sim_time': True}],
            remappings=[
                ('imu', '/imu/fixed_cov'),
                ('gps/fix', '/gps/fixed_cov')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[ekf_path, {'use_sim_time': True}]
        ),

        Node(
            package=package_name,
            executable='static_transform_publisher',
            name='static_transforms_publisher',
            condition=IfCondition(publish_sensor_static_tf),
            parameters=[
                {'static_transform_file': static_transform_path},
                {'use_sim_time': True}
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            condition=IfCondition(publish_map_to_odom),
            parameters=[{'use_sim_time': True}],
            arguments=[
               '--x', map_to_odom_x,
               '--y', map_to_odom_y,
               '--z', map_to_odom_z,
               '--roll', map_to_odom_roll,
               '--pitch', map_to_odom_pitch,
               '--yaw', map_to_odom_yaw,
               '--frame-id', 'map',
               '--child-frame-id', 'odom'
            ]
        )
    ])
