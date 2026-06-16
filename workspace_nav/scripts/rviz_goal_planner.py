#!/usr/bin/env python3

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


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


class RvizGoalPlanner(Node):
    def __init__(self):
        super().__init__('rviz_goal_planner')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_global_path')
        self.declare_parameter('action_name', 'compute_path_to_pose')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('action_timeout', 15.0)

        self.goal_topic = self.get_parameter('goal_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.action_name = self.get_parameter('action_name').value
        self.planner_id = self.get_parameter('planner_id').value
        self.action_timeout = float(self.get_parameter('action_timeout').value)

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, self.path_topic, qos)

        self.action_client = ActionClient(self, ComputePathToPose, self.action_name)
        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal, 10)

        self.active_goal = None
        self.get_logger().info(
            f'RViz goal planner active: {self.goal_topic} -> {self.action_name} -> {self.path_topic}'
        )

    def on_goal(self, msg: PoseStamped):
        if self.active_goal is not None:
            self.get_logger().warning('Ignoring RViz goal because a planning request is already active.')
            return

        if not self.action_client.server_is_ready():
            self.get_logger().info(f'Waiting for planner action server: {self.action_name}')
            if not self.action_client.wait_for_server(timeout_sec=self.action_timeout):
                self.get_logger().error(f'Action server not available: {self.action_name}')
                return

        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.planner_id = self.planner_id
        goal.use_start = False

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f'Planning from current robot pose to RViz goal ({gx:.2f}, {gy:.2f}).')

        send_future = self.action_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)
        self.active_goal = send_future

    def on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error('Failed to send RViz planning goal.')
            self.active_goal = None
            return
        if not goal_handle.accepted:
            self.get_logger().error('RViz planning goal was rejected.')
            self.active_goal = None
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)
        self.active_goal = result_future

    def on_result(self, future):
        wrapped_result = future.result()
        self.active_goal = None

        if wrapped_result is None:
            self.get_logger().error('Planner returned no result for RViz goal.')
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Planner failed for RViz goal with status={wrapped_result.status}.')
            return

        path = wrapped_result.result.path
        self.path_pub.publish(path)
        self.get_logger().info(
            f'Published RViz global path: poses={len(path.poses)}, length={path_length(path):.2f} m.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RvizGoalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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
