import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('num_sectors', 4)
        self.declare_parameter('obstacle_distance', 0.5)

        scan_topic = self.get_parameter('scan_topic').value
        self.num_sectors = self.get_parameter('num_sectors').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value

        self.pub = self.create_publisher(Float32MultiArray, '/robot/obstacle', 10)
        self.sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.get_logger().info('lidar_node started')

    def scan_callback(self, msg):
        if len(msg.ranges) == 0:
            return

        sector_size = len(msg.ranges) // self.num_sectors
        min_distances = []

        for i in range(self.num_sectors):
            start = i * sector_size
            end = start + sector_size if i < self.num_sectors - 1 else len(msg.ranges)
            sector_ranges = [r for r in msg.ranges[start:end] if r > msg.range_min and r < msg.range_max]
            min_dist = min(sector_ranges) if sector_ranges else msg.range_max
            min_distances.append(min_dist)

        # sectors: 0=front, 1=right, 2=back, 3=left
        front, right, back, left = min_distances[0], min_distances[1], min_distances[2], min_distances[3]

        obstacle_msg = Float32MultiArray()
        obstacle_msg.data = [front, right, back, left]
        self.pub.publish(obstacle_msg)

        # log if obstacle too close
        if front < self.obstacle_distance:
            self.get_logger().warn(f'Obstacle front: {front:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
