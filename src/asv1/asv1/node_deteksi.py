#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort

CLASS_NAMES = {0: 'green_ball', 1: 'red_ball', 2: 'target_surface', 3: 'target_underwater'}
CLASS_COLORS = [(0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]


class NodeDeteksi(Node):
    def __init__(self):
        super().__init__('node_deteksi')
        self.declare_parameter('model_path', '/home/aji/yolov8n.onnx')
        self.declare_parameter('conf_threshold', 0.6)
        self.declare_parameter('mission_conf_threshold', 0.6)
        self.declare_parameter('required_frames', 5)
        self.declare_parameter('horizon_ratio', 0.33)
        self.declare_parameter('inference_interval_ms', 100)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.mission_conf_threshold = self.get_parameter('mission_conf_threshold').value
        self.required_frames = self.get_parameter('required_frames').value
        self.horizon_ratio = self.get_parameter('horizon_ratio').value
        interval = max(0.05, self.get_parameter('inference_interval_ms').value / 1000.0)

        self.bridge = CvBridge()
        self.model = None
        self.model_path = model_path
        self.input_name = None
        self.input_size = None

        self.latest = {'utama': None, 'bawah': None, 'samping': None}
        self.detection_counter = {'surface': 0, 'underwater': 0}
        self.skip_counter = {'bawah': 0, 'samping': 0}
        self.inference_skip = 5

        self._inference_errors = 0

        self.pub_deteksi = self.create_publisher(Float32MultiArray, '/asv/deteksi', 10)
        self.pub_tracking = self.create_publisher(Float32MultiArray, '/asv/tracking', 10)
        self.pub_trigger = self.create_publisher(Int32, '/asv/trigger', 10)
        self.pub_vizu = self.create_publisher(Image, '/asv/vizu', 10)

        self.sub_utama = self.create_subscription(
            Image, '/asv/kamera/utama', lambda m: self._store('utama', m), 1)
        self.sub_bawah = self.create_subscription(
            Image, '/asv/kamera/bawah', lambda m: self._store('bawah', m), 1)
        self.sub_samping = self.create_subscription(
            Image, '/asv/kamera/samping', lambda m: self._store('samping', m), 1)

        self.create_timer(interval, self._proses)
        self.get_logger().info(f'node_deteksi started')
        self.get_logger().info(f'Subscribe: /asv/kamera/utama, /asv/kamera/bawah, /asv/kamera/samping')
        self.get_logger().info(f'Inference interval: {interval:.3f}s, conf_threshold: {self.conf_threshold}')

        self._load_model(model_path)

    def _load_model(self, path):
        try:
            self.model = ort.InferenceSession(
                path, providers=['CPUExecutionProvider'],
                sess_options=ort.SessionOptions())
            inp = self.model.get_inputs()[0]
            self.input_name = inp.name
            self.input_size = inp.shape[2]  # 320
            names = ['green_ball', 'red_ball']
            self.get_logger().info(f'Model ONNX loaded: {path}')
            self.get_logger().info(f'  Input: {inp.shape} -> {self.input_size}x{self.input_size}')
            self.get_logger().info(f'  Classes: {names}')
            self.get_logger().info(f'  Threshold: {self.conf_threshold}')
        except Exception as e:
            self.get_logger().error(f'GAGAL load model ONNX: {e}')
            self.get_logger().error(f'  Cek path: {path}')

    def _store(self, camera, msg):
        try:
            self.latest[camera] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if not hasattr(self, f'_first_{camera}'):
                setattr(self, f'_first_{camera}', True)
                h, w = self.latest[camera].shape[:2]
                self.get_logger().info(f'First frame from {camera}: {w}x{h}')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error ({camera}): {e}')

    def _reload_model(self):
        try:
            import onnxruntime as ort
            self.model = ort.InferenceSession(
                self.model_path, providers=['CPUExecutionProvider'],
                sess_options=ort.SessionOptions())
            inp = self.model.get_inputs()[0]
            self.input_name = inp.name
            self.input_size = inp.shape[2]
            self._inference_errors = 0
            self.get_logger().info(f'Model ONNX reloaded: {self.model_path}')
        except Exception as e:
            self.get_logger().warn(f'Model reload failed: {e}')
            self.model = None

    def _proses(self):
        # CRITICAL FIX: Check if model is loaded
        if self.model is None:
            return

        try:
            vizu_frame = None
            all_detections = []
            trigger = 0

            frame = self.latest.get('utama')
            if frame is not None:
                dets, tracking, trigger_cam, annotated = self._proses_kamera(frame, 'utama')
                vizu_frame = annotated

                all_detections.extend(dets)

                if trigger_cam > 0:
                    trigger = trigger_cam

                if tracking['status'] == 1:
                    off = tracking['offset']
                    self.get_logger().info(f'TRACKING: midpoint offset={off}px', throttle_duration_sec=2)

                self._publish_tracking(tracking)
            else:
                if not hasattr(self, '_warn_no_utama'):
                    self.get_logger().warn('Menunggu frame dari kamera utama...')
                    self._warn_no_utama = True

            for cam in ['bawah', 'samping']:
                self.skip_counter[cam] += 1
                if self.skip_counter[cam] < self.inference_skip:
                    continue
                self.skip_counter[cam] = 0

                frame = self.latest.get(cam)
                if frame is None:
                    continue
                dets, _, trigger_cam, _ = self._proses_kamera(frame, cam)
                all_detections.extend(dets)
                if trigger_cam > 0:
                    trigger = trigger_cam

            if all_detections:
                msg = Float32MultiArray()
                flat = []
                for d in all_detections:
                    flat.extend(d)
                msg.data = flat
                self.pub_deteksi.publish(msg)
                if not hasattr(self, '_last_det_count') or self._last_det_count != len(all_detections):
                    self.get_logger().info(f'Deteksi: {len(all_detections)} objek terdeteksi')
                    self._last_det_count = len(all_detections)

            self.pub_trigger.publish(Int32(data=trigger))
            if trigger > 0:
                self.get_logger().info(f'TRIGGER: {trigger} (1=surface, 2=underwater)')

            if vizu_frame is not None:
                try:
                    msg = self.bridge.cv2_to_imgmsg(vizu_frame, 'bgr8')
                    msg.header.frame_id = 'kamera_utama'
                    self.pub_vizu.publish(msg)
                except Exception:
                    pass

            self._inference_errors = 0

        except Exception as e:
            self._inference_errors += 1
            self.get_logger().warn(f'Inference error ({self._inference_errors}x): {e}')
            safe = Float32MultiArray()
            safe.data = [0.0] * 12
            self.pub_tracking.publish(safe)
            if self._inference_errors >= 5:
                self._reload_model()

    def _nms(self, detections, iou_thresh=0.45):
        if not detections:
            return []
        boxes = np.array([[d[2], d[3], d[4], d[5]] for d in detections], dtype=np.float32)
        scores = np.array([d[1] for d in detections], dtype=np.float32)
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            if order.size == 1:
                break
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[1:][iou <= iou_thresh]
        return [detections[i] for i in keep]

    def _decode_yolo(self, output, conf_thresh, scale):
        out = output[0]
        cx = out[0]
        cy = out[1]
        w = out[2]
        h = out[3]
        cls_scores = out[4:]

        # Model output already has sigmoid-ed class scores
        confs = cls_scores.max(axis=0)
        cls_ids = cls_scores.argmax(axis=0)

        mask = confs > conf_thresh
        if not mask.any():
            return []

        cx, cy, w, h = cx[mask], cy[mask], w[mask], h[mask]
        confs = confs[mask]
        cls_ids = cls_ids[mask]

        x1 = np.clip(cx - w / 2, 0, self.input_size)
        y1 = np.clip(cy - h / 2, 0, self.input_size)
        x2 = np.clip(cx + w / 2, 0, self.input_size)
        y2 = np.clip(cy + h / 2, 0, self.input_size)

        dets = []
        for i in range(len(confs)):
            dets.append([int(cls_ids[i]), float(confs[i]),
                         float(x1[i]) * scale, float(y1[i]) * scale,
                         float(x2[i]) * scale, float(y2[i]) * scale])
        dets = self._nms(dets, 0.45)
        return dets

    def _detect_colors(self, frame_640):
        hsv = cv2.cvtColor(frame_640, cv2.COLOR_BGR2HSV)
        dets = []

        green_mask = cv2.inRange(hsv, (20, 40, 40), (100, 255, 255))
        red_mask = cv2.inRange(hsv, (0, 40, 40), (20, 255, 255)) | \
                   cv2.inRange(hsv, (160, 40, 40), (180, 255, 255))

        for cls_id, mask in [(0, green_mask), (1, red_mask)]:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for cnt in cnts:
                if cv2.contourArea(cnt) < 80:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                dets.append([cls_id, 0.85, float(x), float(y),
                             float(x + w), float(y + h)])

        dets = self._nms(dets, 0.45)
        return dets

    def _proses_kamera(self, frame, camera):
        h_orig, w_orig = frame.shape[:2]
        inference_size = self.input_size
        scale = 640.0 / inference_size  # scale model output → 640 coord space

        frame_resized = cv2.resize(frame, (inference_size, inference_size))
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        blob = frame_rgb.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))
        blob = np.expand_dims(blob, axis=0)

        outputs = self.model.run(None, {self.input_name: blob})
        raw_boxes = self._decode_yolo(outputs[0], self.conf_threshold, scale)

        h_inf = inference_size
        detections = []
        tracking = {'status': 0, 'offset': 0, 'max_area': 0, 'coords': {}}
        trigger = 0
        annotated = cv2.resize(frame, (640, 640))

        horizon_y = int(640 * self.horizon_ratio)

        if camera == 'utama':
            green_list = []
            red_list = []
            surface_detected = False
            underwater_detected = False
        elif camera == 'bawah':
            underwater_detected = False
        else:
            surface_detected = False

        for det in raw_boxes:
            cls_id = int(det[0])
            conf = float(det[1])
            x1, y1, x2, y2 = int(det[2]), int(det[3]), int(det[4]), int(det[5])
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            area = (x2 - x1) * (y2 - y1)

            detections.append([float(cls_id), conf,
                               cx / 640, cy / 640,
                               (x2 - x1) / 640, (y2 - y1) / 640])

            # trigger + tracking logic
            if camera == 'utama':
                if cls_id == 2 and conf > self.mission_conf_threshold:
                    surface_detected = True
                elif cls_id == 3 and conf > self.mission_conf_threshold:
                    underwater_detected = True
                elif cls_id in (0, 1) and cy >= horizon_y:
                    target = {'x': cx, 'y': cy, 'area': area}
                    if cls_id == 0:
                        green_list.append(target)
                    else:
                        red_list.append(target)
            elif camera == 'bawah' and cls_id == 3 and conf > self.mission_conf_threshold:
                underwater_detected = True
            elif camera == 'samping' and cls_id == 2 and conf > self.mission_conf_threshold:
                surface_detected = True

        if camera == 'utama':
            if not surface_detected:
                self.detection_counter['surface'] = 0
            else:
                self.detection_counter['surface'] += 1
                if self.detection_counter['surface'] >= self.required_frames:
                    trigger = 1

            if not underwater_detected:
                self.detection_counter['underwater'] = 0
            else:
                self.detection_counter['underwater'] += 1
                if self.detection_counter['underwater'] >= self.required_frames:
                    trigger = 2

            if green_list:
                green_list.sort(key=lambda b: b['area'], reverse=True)
                best = green_list[0]
                tracking['coords']['green'] = (int(best['x']), int(best['y']))
                tracking['max_area'] = max(tracking['max_area'], best['area'])
            if red_list:
                red_list.sort(key=lambda b: b['area'], reverse=True)
                best = red_list[0]
                tracking['coords']['red'] = (int(best['x']), int(best['y']))
                tracking['max_area'] = max(tracking['max_area'], best['area'])

            # draw — cuma bola terpilih (terbesar) + gate
            if 'green' in tracking['coords']:
                gx, gy = tracking['coords']['green']
                cv2.rectangle(annotated, (gx - 15, gy - 15), (gx + 15, gy + 15), (0, 255, 0), 2)
                cv2.putText(annotated, 'GREEN', (gx - 20, gy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if 'red' in tracking['coords']:
                rx, ry = tracking['coords']['red']
                cv2.rectangle(annotated, (rx - 15, ry - 15), (rx + 15, ry + 15), (0, 0, 255), 2)
                cv2.putText(annotated, 'RED', (rx - 15, ry - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if 'red' in tracking['coords'] and 'green' in tracking['coords']:
                rx, ry = tracking['coords']['red']
                gx, gy = tracking['coords']['green']
                mx = (rx + gx) // 2
                my = (ry + gy) // 2
                tracking['coords']['midpoint'] = (mx, my)
                tracking['offset'] = mx - 320
                tracking['status'] = 1

                cv2.circle(annotated, (mx, my), 8, (0, 255, 255), -1)
                cv2.line(annotated, (gx, gy), (rx, ry), (255, 255, 0), 2)
                cv2.line(annotated, (320, 640), (mx, my), (255, 0, 255), 3)
            else:
                tracking['status'] = 0

        elif camera == 'bawah':
            if not underwater_detected:
                self.detection_counter['underwater'] = 0
            else:
                self.detection_counter['underwater'] += 1
                if self.detection_counter['underwater'] >= self.required_frames:
                    trigger = 2

        elif camera == 'samping':
            if not surface_detected:
                self.detection_counter['surface'] = 0
            else:
                self.detection_counter['surface'] += 1
                if self.detection_counter['surface'] >= self.required_frames:
                    trigger = 1

        return detections, tracking, trigger, annotated

    def _publish_tracking(self, tracking):
        c = tracking.get('coords', {})
        msg = Float32MultiArray()
        msg.data = [
            float(tracking['status']),
            float(tracking['offset']),
            1.0 if 'red' in c else 0.0,
            float(c.get('red', (0, 0))[0]),
            float(c.get('red', (0, 0))[1]),
            1.0 if 'green' in c else 0.0,
            float(c.get('green', (0, 0))[0]),
            float(c.get('green', (0, 0))[1]),
            1.0 if 'midpoint' in c else 0.0,
            float(c.get('midpoint', (0, 0))[0]),
            float(c.get('midpoint', (0, 0))[1]),
            float(tracking.get('max_area', 0)),
        ]
        self.pub_tracking.publish(msg)

    def destroy_node(self):
        # CRITICAL FIX: Cleanup resources
        try:
            # Clear frame buffer to free memory
            self.latest = {'utama': None, 'bawah': None, 'samping': None}
        except Exception:
            pass
        try:
            # Release ONNX model
            if self.model:
                self.model = None
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeDeteksi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
