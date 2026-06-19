#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import threading
import time
import math

class NodeMotor(Node):
    def __init__(self):
        super().__init__('node_motor')
        self.declare_parameter('serial_port', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('protocol', 'mavlink')
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('auto_control', False)

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baudrate').value
        self.protocol = self.get_parameter('protocol').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.auto_control = self.get_parameter('auto_control').value

        self.mav = None
        self._mavutil = None
        self.lat = self.lon = self.heading = self.cog = self.sog = self.battery = 0.0
        self._gps_received = False
        self._target_sys = 1
        self._target_comp = 1
        self._armed = False
        self._last_cmd_time = 0.0
        self._last_heartbeat = 0.0
        self._auto_ready = False
        self._reconnecting = False
        self._fc_mode = 0
        self._startup_cycles = 40

        self.get_logger().info('=== MOTOR NODE START ===')
        self.get_logger().info(f'Konfigurasi: port={self._port}, baud={self._baud}')
        self.get_logger().info(f'Max: linear={self.max_linear}, angular={self.max_angular}')
        self.get_logger().info(f'Auto control: {self.auto_control}')

        if self._port and self.protocol == 'mavlink':
            self._connect_mavlink(self._port, self._baud)
        else:
            self.get_logger().info('Motor disabled (port kosong / bukan mavlink)')

        self.pub_telemetri = self.create_publisher(Float32MultiArray, '/asv/telemetri', 10)
        self.sub_cmd = self.create_subscription(Twist, '/asv/cmd_vel', self.cb_cmd, 10)
        self.create_timer(1.0, self.publish_telemetry)
        self.create_timer(1.0, self._watchdog)

        status = "LIVE" if self.mav else "DRY"
        self.get_logger().info(f'node_motor started ({status})')

    def _connect_mavlink(self, port, baud):
        try:
            import serial as _serial
            from pymavlink import mavutil
            self._mavutil = mavutil

            _s = _serial.Serial(port, baud, timeout=1)
            _s.setDTR(False); time.sleep(0.5)
            _s.setDTR(True); time.sleep(0.2)
            _s.setDTR(False); time.sleep(0.5)
            _s.close(); time.sleep(1)

            self.mav = mavutil.mavlink_connection(port, baud, dialect='ardupilotmega')
            if self.mav is None:
                raise RuntimeError('mavlink_connection returned None')
            self.get_logger().info(f'MAVLink connected to {port} @ {baud}')

            deadline = time.time() + 8
            while time.time() < deadline and rclpy.ok():
                msg = self.mav.recv_match(type='HEARTBEAT', blocking=False, timeout=1.0)
                if msg and msg.get_srcSystem() != 255:
                    self._target_sys = msg.get_srcSystem()
                    self._target_comp = msg.get_srcComponent()
                    self._armed = bool(msg.base_mode & 128)
                    self._fc_mode = msg.custom_mode
                    self._last_heartbeat = time.time()
                    self.get_logger().info(f'FC heartbeat: sys={msg.get_srcSystem()}, armed={self._armed}, mode={msg.custom_mode}')
                    break

            threading.Thread(target=self._heartbeat_loop, daemon=True).start()
            threading.Thread(target=self._read_loop, daemon=True).start()

            if self._last_heartbeat > 0:
                self._setup_auto_control()
            else:
                self.get_logger().warn('No heartbeat dalam 8s, skip setup')

            self._reconnecting = False
            return True
        except Exception as e:
            self.get_logger().warn(f'MAVLink connect gagal: {e}')
            self.mav = None
            return False

    def _heartbeat_loop(self):
        while rclpy.ok() and self.mav is not None:
            try:
                self.mav.mav.heartbeat_send(
                    type=18, autopilot=0,
                    base_mode=64, custom_mode=0,
                    system_status=4,
                )
            except Exception:
                pass
            time.sleep(1)

    def _read_loop(self):
        while rclpy.ok() and self.mav is not None:
            try:
                msg = self.mav.recv_match(blocking=False, timeout=0.5)
                if msg is None:
                    continue
                t = msg.get_type()
                if t == 'HEARTBEAT':
                    src = msg.get_srcSystem()
                    armed = bool(msg.base_mode & 128)
                    mode = msg.custom_mode
                    self._last_heartbeat = time.time()
                    if src != 255 and src != self.mav.source_system:
                        self._armed = armed
                        self._fc_mode = mode
                    if not self._gps_received and src != 255:
                        self.get_logger().info(f'HEARTBEAT — sys={src}, armed={armed}, mode={mode}')
                        self._gps_received = True
                    if self._armed and not self._auto_ready:
                        self._auto_ready = True
                        self.get_logger().info('FC ARMED — siap kirim DO_SET_SERVO')
                elif t == 'GLOBAL_POSITION_INT':
                    self.lat = msg.lat / 1e7
                    self.lon = msg.lon / 1e7
                    self.heading = msg.hdg / 100.0
                    self.cog = msg.cog / 100.0
                    self.sog = msg.vx * 0.01
                    if not self._gps_received:
                        self._gps_received = True
                        self.get_logger().info(f'GPS FIX: lat={self.lat:.6f}, lon={self.lon:.6f}')
                elif t == 'SYS_STATUS':
                    self.battery = msg.voltage_battery / 1000.0
                elif t == 'STATUSTEXT':
                    sev = 'INFO'
                    if msg.severity <= 3:
                        sev = 'ERR'
                    elif msg.severity <= 5:
                        sev = 'WARN'
                    self.get_logger().info(f'FC: [{sev}] {msg.text}')
            except Exception:
                pass

    _log_count = 0

    def cb_cmd(self, msg):
        if self.mav is None:
            return
        now = time.time()
        if now - self._last_cmd_time < 0.05:
            return

        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        if self._startup_cycles > 0:
            self._startup_cycles -= 1
            linear = 0.0
            angular = 0.0

        steer_pwm = int(1500 + (angular / self.max_angular) * 400)
        thr_pwm = int(1500 + (linear / self.max_linear) * 400)
        steer_pwm = max(1100, min(1900, steer_pwm))
        thr_pwm = max(1100, min(1900, thr_pwm))

        try:
            # Throttle: S5 (ESC/SERVO5), Rudder: S1+S8 (dual)
            self.mav.mav.command_long_send(
                self._target_sys, self._target_comp,
                self._mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, 5, thr_pwm, 0, 0, 0, 0, 0
            )
            self.mav.mav.command_long_send(
                self._target_sys, self._target_comp,
                self._mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, 1, steer_pwm, 0, 0, 0, 0, 0
            )
            self.mav.mav.command_long_send(
                self._target_sys, self._target_comp,
                self._mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, 8, steer_pwm, 0, 0, 0, 0, 0
            )
        except Exception as e:
            self.get_logger().warn(f'DO_SET_SERVO error: {e}')

        self._last_cmd_time = time.time()
        self._log_count += 1

        if self._log_count % 50 == 0:
            motor = "NYALA" if abs(thr_pwm - 1500) > 20 else "MATI"
            servo = "KANAN" if steer_pwm > 1520 else ("KIRI" if steer_pwm < 1480 else "LURUS")
            self.get_logger().info(f'DO_SET_SERVO: S5={thr_pwm} ({motor}), S1+S8={steer_pwm} ({servo})')

    def _set_fc_params(self):
        if self.mav is None:
            return
        servo_channels = [1, 5]
        params = [
            (b'ARMING_CHECK', 0.0),
            (b'BRD_SAFETYENABLE', 0.0),
            (b'FS_ACTION', 0.0),
            (b'FS_TIMEOUT', 2.0),
            (b'FS_THR_ENABLE', 0.0),
        ]
        # Semua channel pake DO_SET_SERVO override langsung
        for ch in [1, 5, 8]:
            params.append((f'SERVO{ch}_FUNCTION'.encode(), 0.0))
        for name, val in params:
            try:
                self.mav.mav.param_set_send(
                    self._target_sys, self._target_comp,
                    name, val, 9
                )
                time.sleep(0.15)
            except Exception as e:
                self.get_logger().warn(f'Param {name} set error: {e}')
        self.get_logger().info('FC params: SAFETY=0, SERVO[1,5,8]_FUNCTION=0')

    def _setup_auto_control(self):
        self.get_logger().info('=== AUTO CONTROL SETUP ===')
        self._set_fc_params()
        time.sleep(0.5)

        desired_mode = 4 if self.auto_control else 0
        label = 'GUIDED' if self.auto_control else 'MANUAL'
        current_mode = self._fc_mode

        if current_mode != desired_mode:
            try:
                self.mav.mav.command_long_send(
                    self._target_sys, self._target_comp,
                    self._mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0, 1, desired_mode, 0, 0, 0, 0, 0
                )
                self.get_logger().info(f'Mode set: {label} ({desired_mode})')
                time.sleep(1)
            except Exception as e:
                self.get_logger().warn(f'Set mode error: {e}')
        else:
            self.get_logger().info(f'Already in {label} mode — skip')

        if not self._armed:
            try:
                self.mav.mav.command_long_send(
                    self._target_sys, self._target_comp,
                    self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 21196, 0, 0, 0, 0, 0
                )
                self.get_logger().info('Arm command sent (bypass 21196)')
            except Exception as e:
                self.get_logger().warn(f'Arm command error: {e}')
            time.sleep(2)
        else:
            self.get_logger().info('Already armed — skip arm')

        if self._armed:
            self._auto_ready = True
            self.get_logger().info(f'AUTO CONTROL READY — mode {label}')
        else:
            self.get_logger().warn('FC belum armed — cek ulang setup MAVLink')

    def _reconnect(self):
        self._reconnecting = True
        self.get_logger().info('MAVLink reconnect...')
        old_mav = self.mav
        self.mav = None
        if old_mav:
            try:
                old_mav.close()
            except Exception:
                pass
        time.sleep(2)
        for attempt in range(3):
            if self._connect_mavlink(self._port, self._baud):
                self.get_logger().info('Reconnect sukses')
                return True
            self.get_logger().warn(f'Reconnect attempt {attempt+1}/3 gagal, retry...')
            time.sleep(2)
        self.get_logger().warn('Reconnect gagal setelah 3 percobaan')
        return False

    def _watchdog(self):
        if self.mav is None:
            return

        now = time.time()

        if now - self._last_heartbeat > 5.0 and self._last_heartbeat > 0:
            self.get_logger().warn(f'Heartbeat lost ({now - self._last_heartbeat:.0f}s)')
            self._reconnect()

        if now - self._last_cmd_time > 3.0 and self._last_cmd_time > 0:
            self.get_logger().warn('No cmd_vel >3s — sending stop')
            for ch in [1, 5, 8]:
                try:
                    self.mav.mav.command_long_send(
                        self._target_sys, self._target_comp,
                        self._mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0, ch, 1500, 0, 0, 0, 0, 0
                    )
                except Exception:
                    pass

    def publish_telemetry(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.lat), float(self.lon), float(self.heading),
            float(self.cog), float(self.sog), float(self.battery)
        ]
        self.pub_telemetri.publish(msg)

    def destroy_node(self):
        if self.mav:
            try:
                for ch in [1, 5, 8]:
                    self.mav.mav.command_long_send(
                        self._target_sys, self._target_comp,
                        self._mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0, ch, 1500, 0, 0, 0, 0, 0
                    )
                time.sleep(0.2)
                self.mav.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
