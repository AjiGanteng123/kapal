#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import serial
import math
import time
import struct
import threading
import csv
import os
import laspy


SYNC_BYTE = b'\xa5'
SYNC_BYTE2 = b'\x5a'
CMD_STOP = 0x25
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52

DESCRIPTOR_LEN = 7


class RPLidarC1:
    def __init__(self, port, baudrate=460800):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.ser.setDTR(False)
        self._lock = threading.Lock()
        self._buf = b''

    def _send_cmd(self, cmd):
        self.ser.write(SYNC_BYTE + bytes([cmd]))

    def _read_descriptor(self):
        raw = self.ser.read(DESCRIPTOR_LEN)
        if len(raw) != DESCRIPTOR_LEN:
            raise Exception(f'Descriptor length mismatch: got {len(raw)}')
        if raw[0:2] != SYNC_BYTE + SYNC_BYTE2:
            raise Exception(f'Bad descriptor sync: {raw.hex()}')
        dsize = raw[2]
        is_single = raw[-2] == 0
        dtype = raw[-1]
        return dsize, is_single, dtype

    def _read_response(self, dsize):
        data = self.ser.read(dsize)
        if len(data) != dsize:
            raise Exception(f'Response size mismatch: got {len(data)} expected {dsize}')
        return data

    def get_info(self):
        self._send_cmd(CMD_STOP)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self._send_cmd(CMD_GET_INFO)
        dsize, _, _ = self._read_descriptor()
        data = self._read_response(dsize)
        model = data[0]
        fw_ver = (data[2], data[1])
        hw_ver = data[3]
        serialnum = data[4:].hex().upper()
        return {'model': model, 'firmware': fw_ver, 'hardware': hw_ver, 'serialnumber': serialnum}

    def get_health(self):
        self._send_cmd(CMD_GET_HEALTH)
        dsize, _, _ = self._read_descriptor()
        data = self._read_response(dsize)
        status = ['Good', 'Warning', 'Error'][data[0]]
        code = (data[1] << 8) | data[2]
        return status, code

    def start_scan(self):
        self._send_cmd(CMD_STOP)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self._send_cmd(CMD_SCAN)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != 5:
            raise Exception(f'Wrong scan reply length: {dsize}')
        return dsize

    def read_scan_sample(self, dsize=5):
        raw = self.ser.read(dsize)
        if len(raw) != dsize:
            return None
        b = raw
        quality = (b[0] >> 2) & 0x3F
        checkbit = (b[0] >> 1) & 0x01
        new_scan = (b[0] & 0x01)
        angle_raw = ((b[2] << 8) | b[1]) >> 1
        angle = angle_raw / 64.0
        distance = ((b[4] << 8) | b[3]) / 4.0
        return new_scan, quality, angle, distance

    def stop(self):
        try:
            self._send_cmd(CMD_STOP)
        except Exception:
            pass

    def close(self):
        try:
            self.stop()
            self.ser.close()
        except Exception:
            pass


class NodeLidar(Node):
    def __init__(self):
        super().__init__('node_lidar')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('num_sectors', 4)
        self.declare_parameter('obstacle_distance', 0.5)
        self.declare_parameter('publish_scan', True)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('csv_path', '/tmp/lidar_data.csv')
        self.declare_parameter('csv_raw', False)
        self.declare_parameter('las_output', True)
        self.declare_parameter('las_dir', '/tmp/lidar_las')

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baudrate').value
        self.num_sectors = self.get_parameter('num_sectors').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        pub_scan = self.get_parameter('publish_scan').value
        self.frame_id = self.get_parameter('frame_id').value
        csv_path = self.get_parameter('csv_path').value
        self._csv_raw = self.get_parameter('csv_raw').value

        self.pub_obstacle = self.create_publisher(Float32MultiArray, '/asv/obstacle', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10) if pub_scan else None

        self._latest_scan = None
        self._scan_lock = threading.Lock()
        self._scan_thread = None

        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        if self._csv_raw:
            self._csv_writer.writerow(['time_s', 'depan_m', 'kanan_m', 'belakang_m', 'kiri_m'] + [f'a{i}_m' for i in range(360)])
        else:
            self._csv_writer.writerow(['time_s', 'depan_m', 'kanan_m', 'belakang_m', 'kiri_m'])
        self._csv_count = 0

        self._las_output = self.get_parameter('las_output').value
        self._las_dir = self.get_parameter('las_dir').value
        self._las_file = None
        self._las_pts = []
        self._las_scan_count = 0
        if self._las_output:
            os.makedirs(self._las_dir, exist_ok=True)

        self.get_logger().info('=== LIDAR NODE START ===')
        self.get_logger().info(f'Konfigurasi: port={self._port}, baud={self._baud}, sektor={self.num_sectors}')
        self.get_logger().info(f'CSV logging: {csv_path} (raw={self._csv_raw})')
        self.get_logger().info(f'LAS output: {self._las_output} → {self._las_dir}/')

        self.c1 = None
        self._scan_dsize = 5
        self._connected_once = False
        self._lidar_failures = 0

        if self._port:
            self._connect()
        else:
            self.get_logger().info('LiDAR disabled (serial_port empty)')

        # CRITICAL FIX: Use timer to publish latest data, not read serial
        self.create_timer(0.1, self._publish)
        self.get_logger().info(f'node_lidar started ({"LIVE" if self.c1 else "DRY MODE"})')

    def _connect(self):
        try:
            self.c1 = RPLidarC1(self._port, self._baud)
            info = self.c1.get_info()
            health = self.c1.get_health()
            self.get_logger().info(f'RPLidar C1 connected: {info}')
            self.get_logger().info(f'Health: {health}')

            time.sleep(1)
            self._scan_dsize = self.c1.start_scan()
            self.get_logger().info(f'Scan started (dsize={self._scan_dsize})')
            
            # CRITICAL FIX: Start scan reading in separate thread
            self._scan_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._scan_thread.start()
        except Exception as e:
            self.get_logger().warn(f'RPLidar C1 not available ({e})')
            if self.c1:
                self.c1.close()
            self.c1 = None

    def _reconnect_lidar(self):
        self.get_logger().info('Reconnecting LiDAR...')
        try:
            if self.c1:
                self.c1.close()
        except Exception:
            pass
        self.c1 = None
        time.sleep(1)
        self._connect()
        self._lidar_failures = 0
        if self.c1:
            self.get_logger().info('LiDAR reconnect success')
        else:
            self.get_logger().warn('LiDAR reconnect failed')

    def _read_loop(self):
        """
        CRITICAL FIX: Continuous scan reading in separate thread
        This prevents blocking ROS timer callbacks
        """
        while rclpy.ok() and self.c1 is not None:
            try:
                self._read_and_process_scan()
            except Exception as e:
                self._lidar_failures += 1
                self.get_logger().warn(f'LiDAR error ({self._lidar_failures}x): {e}')
                if self._lidar_failures >= 3:
                    self._reconnect_lidar()
                    break
                time.sleep(0.1)

    def _read_and_process_scan(self):
        """Read one complete scan and store to latest_scan"""
        if self.c1 is None:
            return

        if not self._connected_once:
            self._connected_once = True
            self.get_logger().info('RPLidar C1 scan membaca...')

        sample = self.c1.read_scan_sample(self._scan_dsize)
        if sample is None:
            return

        new_scan, quality, angle, distance = sample

        scan = []
        scan.append((new_scan, quality, angle, distance))

        for _ in range(359):
            s = self.c1.read_scan_sample(self._scan_dsize)
            if s is None:
                break
            scan.append((s[0], s[1], s[2], s[3]))

        self._lidar_failures = 0

        sector_size = 360 // self.num_sectors
        min_distances = []
        for i in range(self.num_sectors):
            start_deg = i * sector_size
            end_deg = start_deg + sector_size
            sector_ranges = []
            for m in scan:
                deg = m[2]
                if start_deg <= deg < end_deg:
                    d = m[3] / 1000.0
                    if 0.1 < d < 12.0:
                        sector_ranges.append(d)
            if sector_ranges:
                min_distances.append(min(sector_ranges))
            else:
                min_distances.append(12.0)

        # Store to latest_scan with lock
        with self._scan_lock:
            self._latest_scan = {
                'min_distances': min_distances,
                'scan': scan,
                'timestamp': time.time()
            }

    def _publish(self):
        """Publish latest scan data from timer callback (non-blocking)"""
        with self._scan_lock:
            if self._latest_scan is None:
                return
            data = self._latest_scan.copy()

        min_distances = data['min_distances']
        scan = data['scan']

        # Publish obstacle
        msg = Float32MultiArray()
        msg.data = min_distances
        self.pub_obstacle.publish(msg)

        # CSV logging
        now = self.get_clock().now().nanoseconds / 1e9
        if self._csv_raw:
            scan_ranges = [12.0] * 360
            for m in scan:
                deg = int(m[2]) % 360
                d = m[3] / 1000.0
                if 0.1 < d < 12.0:
                    scan_ranges[deg] = d
            self._csv_writer.writerow([f'{now:.3f}'] + [f'{d:.3f}' for d in min_distances] + [f'{d:.3f}' for d in scan_ranges])
        else:
            self._csv_writer.writerow([f'{now:.3f}'] + [f'{d:.3f}' for d in min_distances])
        self._csv_count += 1
        if self._csv_count % 100 == 0:
            self._csv_file.flush()
            self.get_logger().info(f'CSV: {self._csv_count} rows written')

        # Warnings
        if min_distances[0] < self.obstacle_distance:
            self.get_logger().warn(f'OBSTACLE DEPAN: {min_distances[0]:.2f}m!', throttle_duration_sec=1)
        sektor_labels = ['depan', 'kanan', 'belakang', 'kiri']
        for i, d in enumerate(min_distances):
            if d < self.obstacle_distance:
                self.get_logger().warn(f'  {sektor_labels[i]}: {d:.2f}m', throttle_duration_sec=2)

        # Publish LaserScan
        if self.pub_scan:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = self.frame_id
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.angle_min = 0.0
            scan_msg.angle_max = 2 * math.pi
            scan_msg.angle_increment = (2 * math.pi) / 360
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = 0.1
            scan_msg.range_max = 12.0
            scan_msg.ranges = [12.0] * 360
            for m in scan:
                deg = int(m[2]) % 360
                d = m[3] / 1000.0
                if 0.1 < d < 12.0:
                    scan_msg.ranges[deg] = d
            self.pub_scan.publish(scan_msg)

        # LAS output
        if self._las_output:
            for m in scan:
                deg_rad = math.radians(m[2])
                d = m[3] / 1000.0
                if 0.1 < d < 12.0:
                    x = d * math.cos(deg_rad)
                    y = d * math.sin(deg_rad)
                    self._las_pts.append([x, y, 0.0])
            self._las_scan_count += 1
            if self._las_scan_count % 10 == 0:
                ts = time.strftime('%H%M%S')
                fname = f'{self._las_dir}/scan_{ts}.las'
                header = laspy.LasHeader(point_format=3, version="1.2")
                header.x_scale = 0.001
                header.y_scale = 0.001
                header.z_scale = 0.001
                header.x_offset = 0
                header.y_offset = 0
                header.z_offset = 0
                las = laspy.LasData(header)
                las.x = [p[0] for p in self._las_pts]
                las.y = [p[1] for p in self._las_pts]
                las.z = [p[2] for p in self._las_pts]
                las.write(fname)
                n = len(self._las_pts)
                self._las_pts = []
                self.get_logger().info(f'LAS saved: {fname} ({n} points)')

    def _save_las(self, pts, suffix='final'):
        if not pts:
            return
        fname = f'{self._las_dir}/scan_{suffix}.las'
        try:
            header = laspy.LasHeader(point_format=3, version="1.2")
            header.x_scale = 0.001
            header.y_scale = 0.001
            header.z_scale = 0.001
            header.x_offset = 0
            header.y_offset = 0
            header.z_offset = 0
            las = laspy.LasData(header)
            las.x = [p[0] for p in pts]
            las.y = [p[1] for p in pts]
            las.z = [p[2] for p in pts]
            las.write(fname)
            self.get_logger().info(f'LAS saved: {fname} ({len(pts)} pts)')
        except Exception as e:
            self.get_logger().warn(f'LAS save error: {e}')

    def destroy_node(self):
        if self.c1 is not None:
            try:
                self.c1.close()
            except Exception:
                pass
        try:
            self._csv_file.close()
            self.get_logger().info(f'CSV saved: {self._csv_count} rows to /tmp/lidar_data.csv')
        except Exception:
            pass
        if self._las_output and self._las_pts:
            self._save_las(self._las_pts, 'final')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
