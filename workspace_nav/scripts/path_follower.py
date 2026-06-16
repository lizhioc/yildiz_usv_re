#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


GREEN = '\x1b[32m'
RESET = '\x1b[0m'


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def path_length(points: List[Tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0

    total = 0.0
    prev_x, prev_y = points[0]
    for x, y in points[1:]:
        total += math.hypot(x - prev_x, y - prev_y)
        prev_x, prev_y = x, y
    return total


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('path_topic', '/planned_global_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('lookahead_topic', '/path_follower/lookahead_pose')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('path_frame', '')
        self.declare_parameter('default_path_frame', 'map')
        self.declare_parameter('control_frequency', 5.0)
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('goal_tolerance', 0.8)
        self.declare_parameter('min_path_point_distance', 0.05)
        self.declare_parameter('max_linear_speed', 1.2)
        self.declare_parameter('min_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('yaw_kp', 0.8)
        self.declare_parameter('rotate_in_place_angle', 1.2)
        self.declare_parameter('slow_down_angle', 0.7)
        self.declare_parameter('slow_down_distance', 3.0)
        self.declare_parameter('tf_timeout', 0.2)
        self.declare_parameter('stop_on_completion', True)
        self.declare_parameter('stop_on_tf_failure', True)

        self.path_topic = self.get_parameter('path_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.lookahead_topic = self.get_parameter('lookahead_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.path_frame_override = self.get_parameter('path_frame').value
        self.default_path_frame = self.get_parameter('default_path_frame').value
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.min_path_point_distance = float(self.get_parameter('min_path_point_distance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.rotate_in_place_angle = float(self.get_parameter('rotate_in_place_angle').value)
        self.slow_down_angle = float(self.get_parameter('slow_down_angle').value)
        self.slow_down_distance = float(self.get_parameter('slow_down_distance').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.stop_on_completion = bool(self.get_parameter('stop_on_completion').value)
        self.stop_on_tf_failure = bool(self.get_parameter('stop_on_tf_failure').value)

        self._validate_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = QoSReliabilityPolicy.RELIABLE
        path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, path_qos)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.lookahead_pub = self.create_publisher(PoseStamped, self.lookahead_topic, 10)

        self.path_points: List[Tuple[float, float]] = []
        self.path_frame = self.default_path_frame
        self.closest_index = 0
        self.active = False
        self.last_path_signature: Optional[Tuple[int, float, float, float, float]] = None

        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f'Path follower waiting: {self.path_topic} -> {self.cmd_vel_topic}, '
            f'base_frame={self.base_frame}, frequency={self.control_frequency:.1f} Hz'
        )

    def _validate_parameters(self):
        self.control_frequency = max(1.0, self.control_frequency)
        self.lookahead_distance = max(0.1, self.lookahead_distance)
        self.goal_tolerance = max(0.05, self.goal_tolerance)
        self.min_path_point_distance = max(0.0, self.min_path_point_distance)
        self.max_linear_speed = max(0.0, self.max_linear_speed)
        self.min_linear_speed = clamp(self.min_linear_speed, 0.0, self.max_linear_speed)
        self.max_angular_speed = max(0.0, self.max_angular_speed)
        self.yaw_kp = max(0.0, self.yaw_kp)
        self.rotate_in_place_angle = clamp(self.rotate_in_place_angle, 0.0, math.pi)
        self.slow_down_angle = clamp(self.slow_down_angle, 0.01, math.pi)
        self.slow_down_distance = max(self.goal_tolerance, self.slow_down_distance)
        self.tf_timeout = max(0.0, self.tf_timeout)

    def _log_info_green(self, text: str):
        self.get_logger().info(f'{GREEN}{text}{RESET}')

    def on_path(self, msg: Path):
        points = self._extract_points(msg)
        if len(points) < 2:
            self.get_logger().warning('Received path has fewer than two usable poses; stopping follower.')
            self.path_points = []
            self.active = False
            self.publish_stop()
            return

        signature = self._path_signature(points)
        if signature == self.last_path_signature:
            return

        self.last_path_signature = signature
        self.path_points = points
        self.path_frame = self._resolve_path_frame(msg)
        self.closest_index = 0
        self.active = True

        self._log_info_green(
            f'Path follower armed: poses={len(points)}, length={path_length(points):.2f} m, '
            f'frame={self.path_frame}'
        )

    def _extract_points(self, msg: Path) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []
        last_point: Optional[Tuple[float, float]] = None

        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            if last_point is not None:
                if math.hypot(x - last_point[0], y - last_point[1]) < self.min_path_point_distance:
                    continue
            points.append((x, y))
            last_point = (x, y)

        return points

    def _resolve_path_frame(self, msg: Path) -> str:
        if self.path_frame_override:
            return self.path_frame_override
        if msg.header.frame_id:
            return msg.header.frame_id
        if msg.poses and msg.poses[0].header.frame_id:
            return msg.poses[0].header.frame_id
        return self.default_path_frame

    def _path_signature(self, points: List[Tuple[float, float]]) -> Tuple[int, float, float, float, float]:
        first_x, first_y = points[0]
        last_x, last_y = points[-1]
        return (
            len(points),
            round(first_x, 3),
            round(first_y, 3),
            round(last_x, 3),
            round(last_y, 3),
        )

    def control_loop(self):
        if not self.active or len(self.path_points) < 2:
            return

        pose = self.lookup_robot_pose()
        if pose is None:
            if self.stop_on_tf_failure:
                self.publish_stop()
            return

        robot_x, robot_y, robot_yaw = pose
        goal_x, goal_y = self.path_points[-1]
        goal_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)

        self.closest_index = self.find_closest_index(robot_x, robot_y)
        if goal_distance <= self.goal_tolerance and self.closest_index >= len(self.path_points) - 2:
            self.active = False
            if self.stop_on_completion:
                self.publish_stop()
            self._log_info_green('Path follower reached the final goal and stopped.')
            return

        target_index = self.find_lookahead_index(robot_x, robot_y)
        target_x, target_y = self.path_points[target_index]

        heading = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = normalize_angle(heading - robot_yaw)
        linear_x = self.compute_linear_speed(abs(heading_error), goal_distance)
        angular_z = clamp(self.yaw_kp * heading_error, -self.max_angular_speed, self.max_angular_speed)

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        self.publish_lookahead(target_x, target_y, heading)

    def lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for TF {self.path_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw

    def find_closest_index(self, robot_x: float, robot_y: float) -> int:
        start_index = max(0, min(self.closest_index, len(self.path_points) - 1))
        best_index = start_index
        best_distance = float('inf')

        for index in range(start_index, len(self.path_points)):
            x, y = self.path_points[index]
            distance = math.hypot(x - robot_x, y - robot_y)
            if distance < best_distance:
                best_distance = distance
                best_index = index

        return best_index

    def find_lookahead_index(self, robot_x: float, robot_y: float) -> int:
        accumulated = 0.0
        previous_x, previous_y = robot_x, robot_y

        for index in range(self.closest_index, len(self.path_points)):
            x, y = self.path_points[index]
            accumulated += math.hypot(x - previous_x, y - previous_y)
            if accumulated >= self.lookahead_distance:
                return index
            previous_x, previous_y = x, y

        return len(self.path_points) - 1

    def compute_linear_speed(self, abs_heading_error: float, goal_distance: float) -> float:
        if abs_heading_error >= self.rotate_in_place_angle:
            return 0.0

        speed = self.max_linear_speed

        if abs_heading_error > self.slow_down_angle:
            ratio = (math.pi - abs_heading_error) / (math.pi - self.slow_down_angle)
            speed = min(speed, self.max_linear_speed * clamp(ratio, 0.0, 1.0))

        if goal_distance < self.slow_down_distance:
            speed = min(speed, self.max_linear_speed * goal_distance / self.slow_down_distance)

        return clamp(speed, self.min_linear_speed, self.max_linear_speed)

    def publish_lookahead(self, x: float, y: float, yaw: float):
        pose = PoseStamped()
        pose.header.frame_id = self.path_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.lookahead_pub.publish(pose)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
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
