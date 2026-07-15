#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


class InitialPosePublisher(Node):
    def __init__(self, args):
        super().__init__('set_initial_pose')
        self.args = args
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, args.topic, 10)

    def build_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.args.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.args.x
        msg.pose.pose.position.y = self.args.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.args.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.args.yaw * 0.5)

        covariance = [0.0] * 36
        covariance[0] = self.args.xy_stddev ** 2
        covariance[7] = self.args.xy_stddev ** 2
        covariance[35] = self.args.yaw_stddev ** 2
        msg.pose.covariance = covariance
        return msg

    def publish_for_duration(self):
        msg = self.build_msg()
        deadline = time.monotonic() + self.args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            self.get_logger().info(
                f'Published initial pose: x={self.args.x:.2f}, '
                f'y={self.args.y:.2f}, yaw={self.args.yaw:.2f}'
            )
            rclpy.spin_once(self, timeout_sec=self.args.interval)


def parse_args(argv=None):
    raw_args = sys.argv if argv is None else [sys.argv[0]] + list(argv)
    clean_args = remove_ros_args(args=raw_args)[1:]

    parser = argparse.ArgumentParser(description='Publish an AMCL initial pose in the map frame.')
    parser.add_argument('--x', type=float, required=True, help='Initial x in map frame, meters.')
    parser.add_argument('--y', type=float, required=True, help='Initial y in map frame, meters.')
    parser.add_argument('--yaw', type=float, default=0.0, help='Initial yaw in radians.')
    parser.add_argument('--frame-id', default='map', help='Initial pose frame, normally map.')
    parser.add_argument('--topic', default='/initialpose', help='Initial pose topic.')
    parser.add_argument('--xy-stddev', type=float, default=0.25, help='XY standard deviation in meters.')
    parser.add_argument('--yaw-stddev', type=float, default=0.25, help='Yaw standard deviation in radians.')
    parser.add_argument('--duration', type=float, default=2.0, help='Seconds to keep publishing.')
    parser.add_argument('--interval', type=float, default=0.2, help='Seconds between publications.')
    return parser.parse_args(clean_args)


def main(args=None):
    cli_args = parse_args(args)
    rclpy.init(args=args)
    node = InitialPosePublisher(cli_args)
    try:
        node.publish_for_duration()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
