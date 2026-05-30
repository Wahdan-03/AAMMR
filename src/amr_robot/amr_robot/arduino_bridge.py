#!/usr/bin/env python3
"""
arduino_bridge.py
─────────────────
ROS2 ↔ Arduino serial bridge.

LiDAR-primary SLAM mode:
  - Publishes /odom with HIGH COVARIANCE so SLAM Toolbox ignores wheel odometry
  - Broadcasts odom→base_link TF on EVERY timer tick (not just when serial data arrives)
    to prevent TF gaps that cause the SLAM message filter queue to overflow.

Deadband compensation:
  - Motors require a minimum PWM/speed before they actually move (deadband).
  - Nav2's RPP controller can command very small velocities (especially on
    approach or during fine heading corrections) that fall below this deadband,
    causing the robot to stutter or not move at all.
  - For each wheel independently: if the commanded speed is nonzero but below
    the configured minimum, it is lifted to that minimum while preserving sign.
  - Left and right minimums are separate parameters because your left wheel
    has a lower deadband than the right.
  - Zero commands are always passed through as zero (clean stop).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('serial_port',  '/dev/arduino_mega')
        self.declare_parameter('baud_rate',    115200)
        self.declare_parameter('wheel_base',   0.165)
        self.declare_parameter('publish_rate', 20.0)

        # Deadband parameters — minimum wheel speed (m/s) that actually
        # causes motor movement. Anything nonzero below this is lifted to
        # this value. Tune these to the smallest value that reliably moves
        # each motor during manual testing (you said ~0.5–0.8).
        # Left and right are separate because your motors differ.
        self.declare_parameter('min_wheel_speed_left',  0.55)
        self.declare_parameter('min_wheel_speed_right', 0.65)

        port            = self.get_parameter('serial_port').value
        baud            = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        rate            = self.get_parameter('publish_rate').value
        self.min_vL     = self.get_parameter('min_wheel_speed_left').value
        self.min_vR     = self.get_parameter('min_wheel_speed_right').value

        # ── Last known pose (initialised at origin) ──────────
        # Published every tick so the TF tree never has gaps.
        self.last_x     = 0.0
        self.last_y     = 0.0
        self.last_theta = 0.0   # degrees, converted in publish_data()
        self.last_vL    = 0.0
        self.last_vR    = 0.0

        # ── Serial Setup ─────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            import time
            self.ser.setDTR(False)
            time.sleep(1)
            self.ser.reset_input_buffer()
            self.ser.setDTR(True)
            time.sleep(2)
            self.get_logger().info(f'Connected to Arduino on {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Could not open serial port: {e}')
            raise SystemExit(1)

        self.get_logger().info(
            f'Deadband: left={self.min_vL:.3f} m/s  right={self.min_vR:.3f} m/s'
        )

        # ── ROS2 Publishers / Subscribers ────────────────────
        self.odom_pub       = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub        = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10
        )

        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info('Bridge ready – publishing /odom (high covariance) and TF every tick.')

    # ── Deadband helper ───────────────────────────────────────
    @staticmethod
    def _apply_deadband(speed: float, minimum: float) -> float:
        """
        If |speed| is nonzero but below 'minimum', lift it to 'minimum'
        while preserving the sign.  Zero is always returned as zero so
        the robot can still make a clean stop.
        """
        if speed == 0.0:
            return 0.0
        if abs(speed) < minimum:
            return math.copysign(minimum, speed)
        return speed

    # ── Command velocity → serial ─────────────────────────────
    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Inverse differential-drive kinematics
        vL_raw = v - (w * self.wheel_base / 2.0)
        vR_raw = v + (w * self.wheel_base / 2.0)

        # Apply per-wheel deadband compensation.
        # This lifts any sub-threshold nonzero command up to the minimum
        # speed that actually moves each motor, fixing the slow-stutter
        # behaviour Nav2 produces when it commands small corrections.
        vL = self._apply_deadband(vL_raw, self.min_vL)
        vR = self._apply_deadband(vR_raw, self.min_vR)

        cmd = f'CMD,{vL:.4f},{vR:.4f}\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ── Main timer callback ───────────────────────────────────
    def update(self):
        # 1. Always publish last known pose first.
        #    This keeps odom→base_link TF continuous even when serial
        #    is silent, preventing the SLAM queue from filling up.
        self.publish_data(
            self.last_x, self.last_y, self.last_theta,
            self.last_vL, self.last_vR
        )

        # 2. Try to read new odometry from serial and update stored pose.
        try:
            if self.ser.in_waiting == 0:
                return

            raw_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='replace')
            lines = raw_data.split('\r\n')

            # Use the most recent ODO line in the buffer
            target_line = None
            for line in reversed(lines):
                if line.startswith('ODO,'):
                    target_line = line
                    break

            if not target_line:
                return

            parts = target_line.split(',')
            if len(parts) < 6:
                return

            self.last_x     = float(parts[1])
            self.last_y     = float(parts[2])
            self.last_theta = float(parts[3])
            self.last_vL    = float(parts[4])
            self.last_vR    = float(parts[5])

        except Exception as e:
            self.get_logger().debug(f'Read error: {e}')

    # ── Publish TF + Odometry ─────────────────────────────────
    def publish_data(self, x, y, theta_deg, vL, vR):
        theta = math.radians(theta_deg)
        now   = self.get_clock().now().to_msg()

        v_body = (vL + vR) / 2.0
        w_body = (vR - vL) / self.wheel_base

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        # 1. Broadcast odom → base_link transform
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish /odom with HIGH COVARIANCE
        #    SLAM Toolbox / AMCL will discount this and rely on LiDAR instead.
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Position uncertainty: ~0.7 m std dev → SLAM ignores this
        odom.pose.covariance[0]  = 0.5   # x  (m²)
        odom.pose.covariance[7]  = 0.5   # y  (m²)
        odom.pose.covariance[35] = 1.0   # yaw (rad²)

        odom.twist.twist.linear.x  = v_body
        odom.twist.twist.angular.z = w_body
        odom.twist.covariance[0]   = 0.1
        odom.twist.covariance[35]  = 0.5

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
