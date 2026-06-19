#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class AutonomousNode(Node):
    def __init__(self):
        super().__init__('autonomous_node')
        self.declare_parameter('obstacle_stop_dist', 0.4)
        self.declare_parameter('search_speed', 0.3)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.4)

        self.obstacle_stop = self.get_parameter('obstacle_stop_dist').value
        self.search_speed = self.get_parameter('search_speed').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        self.front_dist = 10.0
        self.right_dist = 10.0
        self.back_dist = 10.0
        self.left_dist = 10.0
        self.green_ball = None
        self.red_ball = None

        self.state = 'SEARCH'

        self.cmd_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)

        self.create_subscription(Float32MultiArray, '/robot/obstacle', self.obstacle_cb, 10)
        self.create_subscription(Float32MultiArray, '/robot/detections', self.detection_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('autonomous_node started - SEARCH state')

    def obstacle_cb(self, msg):
        if len(msg.data) >= 4:
            self.front_dist = msg.data[0]
            self.right_dist = msg.data[1]
            self.back_dist = msg.data[2]
            self.left_dist = msg.data[3]

    def detection_cb(self, msg):
        self.green_ball = None
        self.red_ball = None
        data = msg.data
        i = 0
        while i < len(data):
            cls_id = int(data[i])
            cx = data[i + 1]
            cy = data[i + 2]
            bw = data[i + 3]
            bh = data[i + 4]
            conf = data[i + 5] if i + 5 < len(data) else 1.0
            if cls_id == 0:
                self.green_ball = (cx, cy, bw, bh, conf)
            elif cls_id == 1:
                self.red_ball = (cx, cy, bw, bh, conf)
            i += 6

    def control_loop(self):
        # obstacle avoidance override
        if self.front_dist < self.obstacle_stop:
            self._publish_cmd(0.0, self.turn_speed)
            return

        cmd = Twist()

        if self.state == 'SEARCH':
            self._do_search(cmd)
        elif self.state == 'APPROACH_GREEN':
            self._do_approach_green(cmd)
        elif self.state == 'PASS_THROUGH':
            self._do_pass_through(cmd)
        elif self.state == 'APPROACH_RED':
            self._do_approach_red(cmd)

        self.cmd_pub.publish(cmd)

    def _do_search(self, cmd):
        if self.green_ball is not None:
            self.state = 'APPROACH_GREEN'
            self.get_logger().info('Green ball found! Switching to APPROACH_GREEN')
            self._do_approach_green(cmd)
            return
        cmd.angular.z = self.search_speed

    def _do_approach_green(self, cmd):
        if self.green_ball is None and self.red_ball is None:
            self.state = 'SEARCH'
            self._do_search(cmd)
            return

        if self.green_ball is not None:
            cx, cy, bw, bh, _ = self.green_ball
            err = cx - 0.5
            cmd.angular.z = -err * self.turn_speed * 2
            cmd.linear.x = self.forward_speed * (1.0 - abs(err))

            # also check if red ball is visible too
            if self.red_ball is not None:
                self.state = 'PASS_THROUGH'
                self.get_logger().info('Both balls visible! Switching to PASS_THROUGH')
                self._do_pass_through(cmd)
                return

            # if green ball is close (large), transition
            if bw > 0.3:
                self.state = 'APPROACH_RED' if self.red_ball is not None else 'SEARCH'
                self.get_logger().info(f'Green ball close. State -> {self.state}')
        else:
            # lost green, go back to search
            self.state = 'SEARCH'
            self._do_search(cmd)

    def _do_pass_through(self, cmd):
        if self.green_ball is None or self.red_ball is None:
            self.state = 'APPROACH_GREEN' if self.green_ball else 'SEARCH'
            self._do_search(cmd)
            return

        gx, gy, gw, gh, _ = self.green_ball
        rx, ry, rw, rh, _ = self.red_ball

        # steer toward midpoint between green and red
        mid_x = (gx + rx) / 2
        err = mid_x - 0.5
        cmd.angular.z = -err * self.turn_speed * 2
        cmd.linear.x = self.forward_speed * (1.0 - abs(err))

        # check if we've passed through (both balls are to the sides)
        if abs(gx - 0.5) > 0.4 and abs(rx - 0.5) > 0.4:
            if gx < 0.3 and rx > 0.7 or rx < 0.3 and gx > 0.7:
                self.state = 'SEARCH'
                self.get_logger().info('Passed through! Back to SEARCH')

    def _do_approach_red(self, cmd):
        if self.red_ball is None:
            self.state = 'SEARCH'
            self._do_search(cmd)
            return

        cx, cy, bw, bh, _ = self.red_ball
        err = cx - 0.5
        cmd.angular.z = -err * self.turn_speed * 2
        cmd.linear.x = self.forward_speed * (1.0 - abs(err))

        if bw > 0.3:
            self.state = 'SEARCH'
            self.get_logger().info('Red ball reached! Back to SEARCH')

    def _publish_cmd(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
