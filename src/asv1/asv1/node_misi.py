#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge
import cv2
import io
import time
import os
from datetime import datetime
from PIL import Image as PILImage


class NodeMisi(Node):
    def __init__(self):
        super().__init__('node_misi')
        self.declare_parameter('firebase_key_path', '/home/aji/robot_ws/firebase-key.json')
        self.declare_parameter('firebase_url', '')
        self.declare_parameter('firebase_ref', '/kapal/tim-asv-01')
        self.declare_parameter('cloudinary_cloud', '')
        self.declare_parameter('cloudinary_key', '')
        self.declare_parameter('cloudinary_folder', 'asv_lomba')
        self.declare_parameter('capture_cooldown', 5.0)

        key_path = self.get_parameter('firebase_key_path').value
        fb_url = self.get_parameter('firebase_url').value
        fb_ref = self.get_parameter('firebase_ref').value
        cld_cloud = self.get_parameter('cloudinary_cloud').value
        cld_key = self.get_parameter('cloudinary_key').value
        self.cld_folder = self.get_parameter('cloudinary_folder').value
        self.capture_cooldown = self.get_parameter('capture_cooldown').value

        # CRITICAL FIX: Read secrets from environment variables, NOT params.yaml
        cld_secret = os.getenv('CLOUDINARY_SECRET', '')

        self.bridge = CvBridge()
        self.last_capture_time = 0
        self.trigger = 0
        self.telemetri = None
        self.latest_frame = {'utama': None, 'bawah': None, 'samping': None}
        self._cam_received = {'utama': False, 'bawah': False, 'samping': False}

        self.get_logger().info('=== MISI NODE START ===')
        self.get_logger().info(f'Firebase: key={key_path}, url={fb_url[:30] if fb_url else "NONE"}...')
        self.get_logger().info(f'Cloudinary: cloud={cld_cloud}, folder={self.cld_folder}')

        # Firebase
        self.fb_ref = None
        if key_path and fb_url:
            self._init_firebase(key_path, fb_url, fb_ref)
        else:
            self.get_logger().info('Firebase disabled (isi firebase_key_path & firebase_url di params.yaml)')

        # Cloudinary
        if cld_cloud and cld_key and cld_secret:
            self._init_cloudinary(cld_cloud, cld_key, cld_secret)
        else:
            self.get_logger().info('Cloudinary disabled (isi CLOUDINARY_SECRET env var + cloudinary config di params.yaml)')

        # Data payload
        self.data_payload = {
            "position_log": {
                "preparation": "In Progress",
                "start": "Pending",
                "floating_ball": 0,
                "surface_imaging": "Pending",
                "underwater_imaging": "Pending",
                "finish": "Pending"
            },
            "attitude": {"sog": 0.0, "cog": 0.0, "heading": 0.0},
            "local_position": {"x": 0.0, "y": 0.0},
            "gps_location": {"lat": 0.0, "lon": 0.0},
            "current_mission": "Autonomous",
            "mission_images": {"surface": None, "underwater": None},
        }

        self.sub_trigger = self.create_subscription(Int32, '/asv/trigger', self._cb_trigger, 10)
        self.sub_telemetri = self.create_subscription(
            Float32MultiArray, '/asv/telemetri', self._cb_telemetri, 10)

        for cam in ['utama', 'bawah', 'samping']:
            self.create_subscription(
                Image, f'/asv/kamera/{cam}',
                lambda m, c=cam: self._store(c, m), 1)

        self.create_timer(1.0, self._worker)
        self.get_logger().info('node_misi started')

    def _init_firebase(self, key_path, url, ref_path):
        try:
            import firebase_admin
            from firebase_admin import credentials, db
            if not firebase_admin._apps:
                cred = credentials.Certificate(key_path)
                firebase_admin.initialize_app(cred, {'databaseURL': url})
            self.fb_ref = db.reference(ref_path)
            self.get_logger().info('Firebase connected')
        except Exception as e:
            self.get_logger().warn(f'Firebase init failed: {e}')

    def _init_cloudinary(self, cloud, key, secret):
        try:
            import cloudinary
            import cloudinary.uploader
            cloudinary.config(cloud_name=cloud, api_key=key, api_secret=secret)
            self.get_logger().info('Cloudinary configured')
        except Exception as e:
            self.get_logger().warn(f'Cloudinary init failed: {e}')

    def _store(self, camera, msg):
        try:
            self.latest_frame[camera] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if not self._cam_received[camera]:
                self._cam_received[camera] = True
                self.get_logger().info(f'Camera {camera} frame received')
        except Exception as e:
            self.get_logger().warn(f'Camera {camera} decode error: {e}')

    def _cb_trigger(self, msg):
        self.trigger = msg.data
        if self.trigger > 0:
            jenis = 'surface' if self.trigger == 1 else 'underwater'
            self.get_logger().info(f'TRIGGER diterima: {jenis}')

    def _cb_telemetri(self, msg):
        if len(msg.data) >= 6:
            self.telemetri = msg.data

    def _upload_to_cloudinary(self, frame):
        try:
            import cloudinary.uploader
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = PILImage.fromarray(frame_rgb)
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG', quality=80)
            buffer.seek(0)
            self.get_logger().info('Uploading to Cloudinary...')
            res = cloudinary.uploader.upload(buffer, folder=self.cld_folder)
            url = res.get('secure_url')
            if url:
                self.get_logger().info(f'Upload OK: {url}')
            return url
        except Exception as e:
            self.get_logger().warn(f'Upload failed: {e}')
            return None

    def _worker(self):
        now = time.time()
        trigger = self.trigger
        self.trigger = 0

        # Update telemetry in payload
        if self.telemetri is not None:
            self.data_payload["gps_location"]["lat"] = self.telemetri[0]
            self.data_payload["gps_location"]["lon"] = self.telemetri[1]
            self.data_payload["attitude"]["heading"] = self.telemetri[2]

        # Process capture trigger
        if trigger > 0 and (now - self.last_capture_time) > self.capture_cooldown:
            if trigger == 1:
                cam_name = 'utama'
                status_key = 'surface_imaging'
            else:
                cam_name = 'bawah'
                status_key = 'underwater_imaging'

            frame = self.latest_frame.get(cam_name)
            if frame is not None:
                self.data_payload["position_log"][status_key] = "In Progress"
                url = self._upload_to_cloudinary(frame)
                if url:
                    self.data_payload["mission_images"][
                        'surface' if trigger == 1 else 'underwater'] = url
                    self.data_payload["position_log"][status_key] = "Done"
                    self.get_logger().info(f'Captured {cam_name} -> {url}')
                else:
                    self.data_payload["position_log"][status_key] = "Failed"
                self.last_capture_time = now

        # Push to Firebase
        if self.fb_ref is not None:
            try:
                self.fb_ref.set(self.data_payload)
            except Exception as e:
                self.get_logger().warn(f'Firebase write error: {e}')

    def destroy_node(self):
        # CRITICAL FIX: Cleanup resources and clear sensitive data
        try:
            # Clear frame cache
            self.latest_frame = {'utama': None, 'bawah': None, 'samping': None}
        except Exception:
            pass
        try:
            # Clear Firebase reference
            self.fb_ref = None
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeMisi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
