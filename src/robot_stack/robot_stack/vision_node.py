#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.5)

        self.use_yolo = False
        model_path = self.get_parameter('model_path').value
        if model_path:
            self.use_yolo = True
            try:
                from ultralytics import YOLO
                self.model = YOLO(model_path)
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.use_yolo = False

        self.bridge = CvBridge()
        self.conf_threshold = self.get_parameter('conf_threshold').value

        camera_topic = self.get_parameter('camera_topic').value
        self.pub = self.create_publisher(Float32MultiArray, '/robot/detections', 10)
        self.sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.get_logger().info('vision_node started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        if self.use_yolo:
            detections = self._detect_yolo(cv_image)
        else:
            detections = self._detect_color_based(cv_image)

        detection_msg = Float32MultiArray()
        flat = []
        for det in detections:
            flat.extend(det)
        detection_msg.data = flat
        self.pub.publish(detection_msg)

    def _detect_yolo(self, cv_image):
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        detections = []
        if results and len(results) > 0:
            boxes = results[0].boxes
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    h, w = cv_image.shape[:2]
                    cx = (x1 + x2) / 2 / w
                    cy = (y1 + y2) / 2 / h
                    bw = (x2 - x1) / w
                    bh = (y2 - y1) / h
                    detections.append([float(cls_id), cx, cy, bw, bh, conf])
        return detections

    def _detect_color_based(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        detections = []
        h, w = cv_image.shape[:2]

        # green ball
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        dets = self._find_contours(mask_green, 0, w, h)
        detections.extend(dets)

        # red ball (red wraps around hue 0/180)
        lower_red1 = np.array([0, 40, 40])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 40, 40])
        upper_red2 = np.array([180, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        dets = self._find_contours(mask_red, 1, w, h)
        detections.extend(dets)

        return detections

    def _find_contours(self, mask, cls_id, w, h):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            cx = (x + bw / 2) / w
            cy = (y + bh / 2) / h
            detections.append([float(cls_id), cx, cy, bw / w, bh / h, 1.0])
        return detections


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
