#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args


GREEN = '\x1b[32m'
RESET = '\x1b[0m'


def make_pose(node: Node, frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
    pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
    return pose


def path_length(path: Path) -> float:
    if len(path.poses) < 2:
        return 0.0

    total = 0.0
    prev = path.poses[0].pose.position
    for pose in path.poses[1:]:
        cur = pose.pose.position
        total += math.hypot(cur.x - prev.x, cur.y - prev.y)
        prev = cur
    return total


class GlobalPathClient(Node):
    def __init__(self, cli_args):
        super().__init__('global_path_client')
        self.cli_args = cli_args
        self.action_client = ActionClient(self, ComputePathToPose, cli_args.action_name)

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, cli_args.path_topic, qos)

    def _log_info_green(self, text: str):
        self.get_logger().info(f'{GREEN}{text}{RESET}')

    def build_goal(self) -> ComputePathToPose.Goal:
        args = self.cli_args
        goal = ComputePathToPose.Goal()
        goal.goal = make_pose(self, args.frame_id, args.goal_x, args.goal_y, args.goal_yaw)
        goal.planner_id = args.planner_id

        if args.start_x is not None and args.start_y is not None:
            goal.start = make_pose(self, args.frame_id, args.start_x, args.start_y, args.start_yaw)
            goal.use_start = True
        else:
            goal.use_start = False

        return goal

    def run(self) -> int:
        args = self.cli_args
        self.get_logger().info(f'Waiting for action server: {args.action_name}')
        if not self.action_client.wait_for_server(timeout_sec=args.wait_timeout):
            self.get_logger().error(f'Action server not available: {args.action_name}')
            return 1

        for attempt in range(1, args.retries + 2):
            exit_code = self.try_plan(attempt)
            if exit_code == 0:
                return 0
            if attempt <= args.retries:
                self.get_logger().warning(
                    f'Planning attempt {attempt} failed; retrying '
                    f'({attempt}/{args.retries}).'
                )

        return 1

    def try_plan(self, attempt: int) -> int:
        args = self.cli_args
        goal = self.build_goal()
        if goal.use_start:
            self.get_logger().info(
                f'Planning attempt {attempt} in {args.frame_id}: '
                f'start=({args.start_x:.2f}, {args.start_y:.2f}) '
                f'goal=({args.goal_x:.2f}, {args.goal_y:.2f})'
            )
        else:
            self.get_logger().info(
                f'Planning attempt {attempt} in {args.frame_id}: current robot pose -> '
                f'goal=({args.goal_x:.2f}, {args.goal_y:.2f})'
            )

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=args.action_timeout)
        if not send_future.done():
            self.get_logger().error(
                f'Timed out waiting for planner goal response after {args.action_timeout:.1f} s.'
            )
            return 1

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error('Failed to send global planning goal.')
            return 1
        if not goal_handle.accepted:
            self.get_logger().error('Global planning goal was rejected.')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=args.action_timeout)
        if not result_future.done():
            self.get_logger().error(
                f'Timed out waiting for planner result after {args.action_timeout:.1f} s.'
            )
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            except Exception:
                pass
            return 1

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error('Global planner returned no result.')
            return 1
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Global planner failed with status={wrapped_result.status}.')
            return 1

        result = wrapped_result.result
        planned_path = result.path
        length_m = path_length(planned_path)
        planning_time = result.planning_time.sec + result.planning_time.nanosec * 1.0e-9

        self.path_pub.publish(planned_path)
        self._log_info_green(
            f'Path ready: poses={len(planned_path.poses)}, '
            f'length={length_m:.2f} m, planning_time={planning_time:.3f} s'
        )
        self.get_logger().info(f'Published path topic: {args.path_topic}')

        self.republish_for(args.keep_alive_sec, planned_path)
        return 0

    def republish_for(self, keep_alive_sec: float, planned_path: Path):
        if keep_alive_sec <= 0.0:
            return

        deadline = time.monotonic() + keep_alive_sec
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.path_pub.publish(planned_path)
                next_publish = now + 1.0
            rclpy.spin_once(self, timeout_sec=0.1)


def parse_args(argv=None):
    raw_args = sys.argv if argv is None else [sys.argv[0]] + list(argv)
    clean_args = remove_ros_args(args=raw_args)[1:]

    parser = argparse.ArgumentParser(
        description='Request a Nav2 global path on the static map and publish it for RViz.'
    )
    parser.add_argument('--goal-x', type=float, required=True, help='Goal x in the map frame, meters.')
    parser.add_argument('--goal-y', type=float, required=True, help='Goal y in the map frame, meters.')
    parser.add_argument('--goal-yaw', type=float, default=0.0, help='Goal yaw in radians.')
    parser.add_argument('--start-x', type=float, help='Optional start x in the map frame, meters.')
    parser.add_argument('--start-y', type=float, help='Optional start y in the map frame, meters.')
    parser.add_argument('--start-yaw', type=float, default=0.0, help='Optional start yaw in radians.')
    parser.add_argument('--frame-id', default='map', help='Planning frame, normally map.')
    parser.add_argument('--planner-id', default='GridBased', help='Planner plugin id in nav2_params.yaml.')
    parser.add_argument('--action-name', default='compute_path_to_pose', help='ComputePathToPose action name.')
    parser.add_argument('--path-topic', default='/planned_global_path', help='Topic used to publish nav_msgs/Path.')
    parser.add_argument('--wait-timeout', type=float, default=30.0, help='Seconds to wait for the planner action server.')
    parser.add_argument('--action-timeout', type=float, default=15.0, help='Seconds to wait for each planner action step.')
    parser.add_argument('--retries', type=int, default=1, help='Retry count after planner action timeout or failure.')
    parser.add_argument('--keep-alive-sec', type=float, default=60.0, help='Seconds to keep republishing the path.')
    args = parser.parse_args(clean_args)

    if (args.start_x is None) != (args.start_y is None):
        parser.error('--start-x and --start-y must be provided together.')

    return args


def main(args=None):
    cli_args = parse_args(args)
    rclpy.init(args=args)
    node = GlobalPathClient(cli_args)
    exit_code = 1

    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        try:
            node.get_logger().info('Global path client interrupted.')
        except Exception:
            pass
        exit_code = 130
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
