#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
import utm
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def rotate(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


class GpsMapTf(Node):
    def __init__(self):
        super().__init__('gps_map_tf')

        self.declare_parameter('gps_topic', '/gps/fixed_cov')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('datum_latitude', 37.210394)
        self.declare_parameter('datum_longitude', 27.579437)
        self.declare_parameter('map_yaw', 0.0)
        self.declare_parameter('gps_offset_x', 0.325)
        self.declare_parameter('gps_offset_y', 0.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_gps_age', 1.0)
        self.declare_parameter('max_odom_age', 1.0)

        self.gps_topic = self.get_parameter('gps_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.datum_latitude = float(self.get_parameter('datum_latitude').value)
        self.datum_longitude = float(self.get_parameter('datum_longitude').value)
        self.map_yaw = float(self.get_parameter('map_yaw').value)
        self.gps_offset_x = float(self.get_parameter('gps_offset_x').value)
        self.gps_offset_y = float(self.get_parameter('gps_offset_y').value)
        self.max_gps_age = float(self.get_parameter('max_gps_age').value)
        self.max_odom_age = float(self.get_parameter('max_odom_age').value)

        self.datum_easting, self.datum_northing, self.zone_number, self.zone_letter = utm.from_latlon(
            self.datum_latitude,
            self.datum_longitude,
        )

        self.last_gps: Optional[NavSatFix] = None
        self.last_odom: Optional[Odometry] = None
        self.warned_no_data = False

        gps_qos = QoSProfile(depth=10)
        gps_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, gps_qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.create_timer(1.0 / publish_rate, self.publish_transform)

        self.get_logger().info(
            'GPS map TF active: '
            f'datum=({self.datum_latitude:.8f}, {self.datum_longitude:.8f}), '
            f'map_yaw={self.map_yaw:.3f}'
        )

    def on_gps(self, msg: NavSatFix):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.last_gps = msg

    def on_odom(self, msg: Odometry):
        self.last_odom = msg

    def stamp_age(self, stamp) -> float:
        now = self.get_clock().now()
        msg_time = Time.from_msg(stamp)
        return (now - msg_time).nanoseconds * 1.0e-9

    def have_fresh_data(self) -> bool:
        if self.last_gps is None or self.last_odom is None:
            if not self.warned_no_data:
                self.get_logger().warning('Waiting for GPS and odometry before publishing map->odom.')
                self.warned_no_data = True
            return False

        gps_age = self.stamp_age(self.last_gps.header.stamp)
        odom_age = self.stamp_age(self.last_odom.header.stamp)
        if gps_age > self.max_gps_age or odom_age > self.max_odom_age:
            self.get_logger().warning(
                f'Skipping stale localization data: gps_age={gps_age:.2f}s, odom_age={odom_age:.2f}s',
                throttle_duration_sec=2.0,
            )
            return False
        return True

    def gps_position_in_map(self) -> Tuple[float, float]:
        easting, northing, zone_number, zone_letter = utm.from_latlon(
            self.last_gps.latitude,
            self.last_gps.longitude,
        )
        if zone_number != self.zone_number or zone_letter != self.zone_letter:
            self.get_logger().warning(
                'GPS UTM zone changed from '
                f'{self.zone_number}{self.zone_letter} to {zone_number}{zone_letter}.'
            )

        dx = easting - self.datum_easting
        dy = northing - self.datum_northing
        return rotate(dx, dy, self.map_yaw)

    def publish_transform(self):
        if not self.have_fresh_data():
            return

        odom_pose = self.last_odom.pose.pose
        odom_x = odom_pose.position.x
        odom_y = odom_pose.position.y
        odom_yaw = yaw_from_quaternion(
            odom_pose.orientation.x,
            odom_pose.orientation.y,
            odom_pose.orientation.z,
            odom_pose.orientation.w,
        )

        gps_map_x, gps_map_y = self.gps_position_in_map()
        base_yaw_map = self.map_yaw + odom_yaw
        gps_offset_map_x, gps_offset_map_y = rotate(
            self.gps_offset_x,
            self.gps_offset_y,
            base_yaw_map,
        )
        base_map_x = gps_map_x - gps_offset_map_x
        base_map_y = gps_map_y - gps_offset_map_y

        odom_in_map_x, odom_in_map_y = rotate(odom_x, odom_y, self.map_yaw)
        map_to_odom_x = base_map_x - odom_in_map_x
        map_to_odom_y = base_map_y - odom_in_map_y

        transform = TransformStamped()
        transform.header.stamp = self.last_odom.header.stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = map_to_odom_x
        transform.transform.translation.y = map_to_odom_y
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(self.map_yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GpsMapTf()
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
