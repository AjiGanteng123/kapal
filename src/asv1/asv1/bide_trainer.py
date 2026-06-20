#!/usr/bin/env python3
"""
bide_trainer.py — Training Data Recorder for Obstacle Avoidance

Records LiDAR 4-sector obstacle detection + cmd_vel control inputs
for offline training of avoidance models.

Output: /tmp/training_data.csv
Columns: time_s, front_m, right_m, back_m, left_m, linear_x, angular_z
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import csv
import time
import os


class BideTrainer(Node):
    def __init__(self):
        super().__init__('bide_trainer')
        self.declare_parameter('output_path', '/tmp/training_data.csv')
        self.declare_parameter('enable_recording', True)
        
        self.output_path = self.get_parameter('output_path').value
        self.enable_recording = self.get_parameter('enable_recording').value
        
        self.csv_file = None
        self.csv_writer = None
        self.start_time = time.time()
        self.last_obstacle = [99.0, 99.0, 99.0, 99.0]  # [front, right, back, left]
        self.last_cmd_vel = [0.0, 0.0]  # [linear.x, angular.z]
        self.record_count = 0
        
        self.get_logger().info('=== BIDE TRAINER START ===')
        self.get_logger().info(f'Output: {self.output_path}')
        self.get_logger().info(f'Recording: {self.enable_recording}')
        
        if self.enable_recording:
            self._init_csv()
        
        # Subscribe to LiDAR and cmd_vel
        self.sub_obstacle = self.create_subscription(
            Float32MultiArray, '/asv/obstacle',
            self.cb_obstacle, 10)
        
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/asv/cmd_vel',
            self.cb_cmd_vel, 10)
        
        # Timer to periodically write to CSV (10Hz)
        self.create_timer(0.1, self.write_record)
        
        self.get_logger().info('bide_trainer started')
    
    def _init_csv(self):
        """Initialize CSV file with headers"""
        try:
            os.makedirs(os.path.dirname(self.output_path) or '.', exist_ok=True)
            self.csv_file = open(self.output_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['time_s', 'front_m', 'right_m', 'back_m', 'left_m', 'linear_x', 'angular_z'])
            self.csv_file.flush()
            self.get_logger().info(f'CSV initialized: {self.output_path}')
        except Exception as e:
            self.get_logger().error(f'CSV init failed: {e}')
            self.csv_file = None
    
    def cb_obstacle(self, msg):
        """Receive LiDAR 4-sector obstacle data"""
        if len(msg.data) >= 4:
            self.last_obstacle = [
                float(msg.data[0]),  # front
                float(msg.data[1]),  # right
                float(msg.data[2]),  # back
                float(msg.data[3])   # left
            ]
    
    def cb_cmd_vel(self, msg):
        """Receive motor control commands"""
        self.last_cmd_vel = [
            float(msg.linear.x),
            float(msg.angular.z)
        ]
    
    def write_record(self):
        """Write current sensor state to CSV"""
        if not self.enable_recording or self.csv_writer is None:
            return
        
        try:
            elapsed_time = time.time() - self.start_time
            row = [
                f'{elapsed_time:.3f}',
                f'{self.last_obstacle[0]:.3f}',
                f'{self.last_obstacle[1]:.3f}',
                f'{self.last_obstacle[2]:.3f}',
                f'{self.last_obstacle[3]:.3f}',
                f'{self.last_cmd_vel[0]:.3f}',
                f'{self.last_cmd_vel[1]:.3f}'
            ]
            self.csv_writer.writerow(row)
            self.record_count += 1
            
            # Flush every 100 records
            if self.record_count % 100 == 0:
                self.csv_file.flush()
                self.get_logger().info(f'Training data: {self.record_count} samples')
        except Exception as e:
            self.get_logger().warn(f'Write error: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f'Training data saved: {self.output_path} ({self.record_count} samples)')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BideTrainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
