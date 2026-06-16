#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that republishes IMU (sensor_msgs/Imu) data with a defined covariance matrix for improved
#  localization consistency. It assigns fixed covariance values to ensure stable input for
#  sensor fusion processes such as robot_localization.
# ----------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovarianceRepub(Node):
    def __init__(self):
        super().__init__('imu_covariance_repub')
        self.declare_parameter_if_missing('use_sim_time', False)
        self.declare_parameter_if_missing('orientation_stddev', 0.01)
        self.declare_parameter_if_missing('angular_velocity_stddev', 0.0032)
        self.declare_parameter_if_missing('linear_acceleration_stddev', 0.20)

        self.subscription = self.create_subscription(
            Imu,
            '/roboboat/sensors/imu/imu',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(
            Imu,
            '/imu/fixed_cov',
            10
        )

    def declare_parameter_if_missing(self, name, value):
        try:
            self.declare_parameter(name, value)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

    def imu_callback(self, msg):
        msg.header.frame_id = 'imu_link'
        orientation_var = float(self.get_parameter('orientation_stddev').value) ** 2
        angular_velocity_var = float(self.get_parameter('angular_velocity_stddev').value) ** 2
        linear_acceleration_var = float(self.get_parameter('linear_acceleration_stddev').value) ** 2
        msg.orientation_covariance = [
            orientation_var, 0.0, 0.0,
            0.0, orientation_var, 0.0,
            0.0, 0.0, orientation_var
        ]
        msg.angular_velocity_covariance = [
            angular_velocity_var, 0.0, 0.0,
            0.0, angular_velocity_var, 0.0,
            0.0, 0.0, angular_velocity_var
        ]
        msg.linear_acceleration_covariance = [
            linear_acceleration_var, 0.0, 0.0,
            0.0, linear_acceleration_var, 0.0,
            0.0, 0.0, linear_acceleration_var
        ]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceRepub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
