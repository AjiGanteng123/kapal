import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('protocol', 'mavlink')
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.8)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.protocol = self.get_parameter('protocol').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.serial = None
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to {port} @ {baud} baud')
        except Exception as e:
            self.get_logger().warn(f'Serial not available ({e}), running in dry mode')

        self.sub = self.create_subscription(Twist, '/robot/cmd_vel', self.cmd_callback, 10)
        self.get_logger().info(f'motor_node started (protocol={self.protocol})')

    def cmd_callback(self, msg):
        if self.serial is None:
            return

        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        if self.protocol == 'mavlink':
            self._send_mavlink(linear, angular)
        elif self.protocol == 'msp':
            self._send_msp(linear, angular)
        else:
            self._send_raw(linear, angular)

    def _send_raw(self, linear, angular):
        left = linear - angular
        right = linear + angular
        data = f"{left:.2f},{right:.2f}\n"
        try:
            self.serial.write(data.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _send_msp(self, linear, angular):
        left = int(1500 + (linear - angular) * 500)
        right = int(1500 + (linear + angular) * 500)
        left = max(1000, min(2000, left))
        right = max(1000, min(2000, right))
        try:
            buf = struct.pack('<BBHHHHHHHHHHHHHHHH', 36, 8, left, right, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)
            checksum = 0
            for b in buf:
                checksum ^= b
            buf += struct.pack('<B', checksum)
            self.serial.write(b'$M<' + buf)
        except Exception as e:
            self.get_logger().error(f'MSP error: {e}')

    def _send_mavlink(self, linear, angular):
        try:
            from pymavlink import mavutil
            if not hasattr(self, 'mav'):
                self.mav = mavutil.mavlink_connection(
                    self.serial.port,
                    baud=self.serial.baudrate,
                    source_system=1
                )
                self.mav.wait_heartbeat(timeout=2)
                self.get_logger().info('MAVLink heartbeat received')

            self.mav.mav.manual_control_send(
                0,
                int(linear * 1000),
                int(angular * 1000),
                500,
                0,
                0
            )
        except ImportError:
            self.get_logger().warn('pymavlink not installed, using raw serial')
            self._send_raw(linear, angular)
        except Exception as e:
            self.get_logger().error(f'MAVLink error: {e}')

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
