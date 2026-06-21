#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class NodeKamera(Node):
    def __init__(self):
        super().__init__('node_kamera')
        self.declare_parameter('device_utama', '0')
        self.declare_parameter('device_bawah', '')
        self.declare_parameter('device_samping', '')
        self.declare_parameter('fps_utama', 30)
        self.declare_parameter('fps_bawah', 10)
        self.declare_parameter('fps_samping', 10)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        self._dev_utama = self.get_parameter('device_utama').value
        self._dev_bawah = self.get_parameter('device_bawah').value
        self._dev_samping = self.get_parameter('device_samping').value
        fps_utama = self.get_parameter('fps_utama').value
        fps_bawah = self.get_parameter('fps_bawah').value
        fps_samping = self.get_parameter('fps_samping').value
        self._width = self.get_parameter('frame_width').value
        self._height = self.get_parameter('frame_height').value

        self.bridge = CvBridge()
        self.caps = {}
        self.timer_list = []
        self.frame_counts = {'utama': 0, 'bawah': 0, 'samping': 0}
        self._cam_failures = {'utama': 0, 'bawah': 0, 'samping': 0}

        self.get_logger().info('=== KAMERA NODE START ===')
        self.get_logger().info(f'Konfigurasi: utama={self._dev_utama}, bawah={self._dev_bawah}, samping={self._dev_samping}')
        self.get_logger().info(f'Resolusi: {self._width}x{self._height}, fps: utama={fps_utama}, bawah={fps_bawah}, samping={fps_samping}')

        cameras = [
            ('utama', self._dev_utama, fps_utama),
            ('bawah', self._dev_bawah, fps_bawah),
            ('samping', self._dev_samping, fps_samping),
        ]

        for name, dev, fps in cameras:
            topic = f'/asv/kamera/{name}'
            pub = self.create_publisher(Image, topic, 10)

            if isinstance(dev, str) and dev == '':
                self.get_logger().info(f'Camera {name} disabled')
                self.caps[name] = None
                continue

            cap = cv2.VideoCapture(int(dev) if isinstance(dev, str) else dev)
            if not cap.isOpened():
                self.get_logger().warn(f'Camera {name} ({dev}) failed to open')
                self.caps[name] = None
                continue

            actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info(f'Camera {name} opened: {dev} -> res={actual_w:.0f}x{actual_h:.0f}')

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self.caps[name] = (cap, pub)

            interval = max(0.03, 1.0 / fps)
            timer = self.create_timer(interval, lambda n=name: self._publish(n))
            self.timer_list.append(timer)
            self.get_logger().info(f'Camera {name} -> topic {topic} @ {fps}fps')

        aktif = sum(1 for c in self.caps.values() if c is not None)
        self.get_logger().info(f'Kamera siap: {aktif}/3 aktif')

    def _publish(self, name):
        entry = self.caps.get(name)
        if entry is None:
            return
        cap, pub = entry
        ret, frame = cap.read()
        if not ret:
            self._cam_failures[name] += 1
            self.get_logger().warn(f'Camera {name} read failed ({self._cam_failures[name]}x)')
            if self._cam_failures[name] >= 5:
                self._reopen_camera(name)
            return
        self._cam_failures[name] = 0
        self.frame_counts[name] += 1
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.frame_id = f'kamera_{name}'
        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

    def _reopen_camera(self, name):
        self.get_logger().info(f'Reopening camera {name}...')
        old = self.caps.get(name)
        if old is not None:
            try:
                old[0].release()
            except Exception:
                pass

        devs = {'utama': self._dev_utama, 'bawah': self._dev_bawah, 'samping': self._dev_samping}
        dev = devs[name]
        if isinstance(dev, str) and dev == '':
            self.caps[name] = None
            return

        cap = cv2.VideoCapture(int(dev) if isinstance(dev, str) else dev)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            pub = old[1] if old else self.create_publisher(Image, f'/asv/kamera/{name}', 10)
            self.caps[name] = (cap, pub)
            self._cam_failures[name] = 0
            self.get_logger().info(f'Camera {name} reopened success')
        else:
            self.caps[name] = None
            self.get_logger().warn(f'Camera {name} reopen failed')

    def destroy_node(self):
        for entry in self.caps.values():
            if entry is not None:
                try:
                    entry[0].release()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeKamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
