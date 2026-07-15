#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that republishes GPS (NavSatFix) data with an assigned covariance matrix for localization
#  consistency. It adjusts the covariance values to provide stable input for sensor fusion nodes
#  such as robot_localization.
# ----------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsCovarianceRepub(Node):
    def __init__(self):
        super().__init__('gps_covariance_repub')
        self.declare_parameter_if_missing('use_sim_time', False)
        self.declare_parameter_if_missing('horizontal_stddev', 0.05)
        self.declare_parameter_if_missing('vertical_stddev', 0.20)

        self.subscription = self.create_subscription(
            NavSatFix,
            '/roboboat/sensors/gps/navsat',
            self.gps_callback,
            10
        )
        self.publisher = self.create_publisher(
            NavSatFix,
            '/gps/fixed_cov',
            10
        )

    def declare_parameter_if_missing(self, name, value):
        try:
            self.declare_parameter(name, value)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

    def gps_callback(self, msg):
        msg.header.frame_id = 'gps_link'
        horizontal_var = float(self.get_parameter('horizontal_stddev').value) ** 2
        vertical_var = float(self.get_parameter('vertical_stddev').value) ** 2
        msg.position_covariance = [
            horizontal_var, 0.0, 0.0,
            0.0, horizontal_var, 0.0,
            0.0, 0.0, vertical_var
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsCovarianceRepub()
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
