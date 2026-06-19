#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import math


class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')
        self.declare_parameter('origin_lat', -7.052600)
        self.declare_parameter('origin_lon', 110.434800)

        origin_lat = self.get_parameter('origin_lat').value
        origin_lon = self.get_parameter('origin_lon').value
        self.origin = (origin_lat, origin_lon)

        self.bridge = CvBridge()

        # Camera: Gazebo → /asv/kamera/utama
        self.pub_kamera = self.create_publisher(Image, '/asv/kamera/utama', 10)
        self.sub_camera = self.create_subscription(
            Image, '/camera/image_raw', self._cb_camera, 10)

        # LiDAR: Gazebo /scan → /asv/obstacle
        self.pub_obstacle = self.create_publisher(Float32MultiArray, '/asv/obstacle', 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self._cb_scan, 10)

        # cmd_vel: /asv/cmd_vel → Gazebo /model/kapal/cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist, '/model/kapal/cmd_vel', 10)
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/asv/cmd_vel', self._cb_cmd_vel, 10)

        # Odometry → /asv/telemetri fake GPS (timer tiap 200ms)
        self._last_odom = None
        self.pub_telemetri = self.create_publisher(Float32MultiArray, '/asv/telemetri', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/model/kapal/odometry', self._cb_odom, 10)
        self.create_timer(0.2, self._publish_gps_telemetry)

        self.get_logger().info('sim_bridge started')

    def _publish_gps_telemetry(self):
        """Publish fake GPS telemetry from odometry, or origin if none yet."""
        lat = self.origin[0]
        lon = self.origin[1]
        hdg = 0.0
        try:
            odom = self._last_odom
            if odom is not None:
                x = odom.pose.pose.position.x
                y = odom.pose.pose.position.y
                lat = self.origin[0] + y / 111320.0
                lon = self.origin[1] + x / (111320.0 * math.cos(math.radians(self.origin[0])))
                q = odom.pose.pose.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                hdg = math.degrees(math.atan2(siny_cosp, cosy_cosp)) % 360
        except AttributeError:
            pass
        out = Float32MultiArray()
        out.data = [lat, lon, hdg, hdg, 0.0, 100.0]
        self.pub_telemetri.publish(out)

    def _cb_camera(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
            out_msg.header = msg.header
            self.pub_kamera.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'Camera bridge error: {e}')

    def _cb_scan(self, msg):
        ranges = msg.ranges
        if not ranges:
            return
        n = len(ranges)
        num_sectors = 4
        sector_size = n // num_sectors
        min_distances = []
        for i in range(num_sectors):
            start = i * sector_size
            end = start + sector_size if i < num_sectors - 1 else n
            sector = [r for r in ranges[start:end] if msg.range_min < r < msg.range_max]
            min_dist = min(sector) if sector else msg.range_max
            min_distances.append(min_dist)
        out = Float32MultiArray()
        out.data = min_distances
        self.pub_obstacle.publish(out)

    def _cb_cmd_vel(self, msg):
        self.pub_cmd_vel.publish(msg)

    def _cb_odom(self, msg):
        self._last_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
