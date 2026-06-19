#!/usr/bin/env python3
"""Lihat kamera + deteksi YOLO langsung."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class Viewer(Node):
    def __init__(self):
        super().__init__('viewer')
        self.bridge = CvBridge()
        self.vizu = None
        self.create_subscription(Image, '/asv/vizu', self.cb_vizu, 10)
        self.create_timer(0.05, self.show)
        self.get_logger().info('Tekan ESC di window untuk keluar')

    def cb_vizu(self, msg):
        self.vizu = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def show(self):
        if self.vizu is not None:
            cv2.imshow('ASV - YOLO Detection', self.vizu)
        if cv2.waitKey(1) == 27:
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
