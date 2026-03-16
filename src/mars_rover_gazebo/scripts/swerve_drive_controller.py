#!/usr/bin/env python3
"""Swerve Drive Controller for Mars Rover.

Converts /cmd_vel (geometry_msgs/Twist) into individual per-wheel steering
angles and drive velocities for a 6-wheel swerve drive configuration.

Supports:
  - Forward / backward driving  (linear.x)
  - Lateral crab walking        (linear.y)
  - In-place rotation           (angular.z)
  - Any combination of the above

Steering commands (rad)   → /gz/steer/{wheel_name}  (std_msgs/Float64)
Drive velocity (rad/s)    → /gz/drive/{wheel_name}  (std_msgs/Float64)
Odometry                  → /odom                   (nav_msgs/Odometry)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.10  # metres

# Wheel positions (x, y) relative to base_link centre (from URDF geometry)
WHEELS = {
    'left_front':  ( 0.32,  0.30),
    'right_front': ( 0.32, -0.30),
    'left_mid':    ( 0.00,  0.30),
    'right_mid':   ( 0.00, -0.30),
    'left_rear':   (-0.44,  0.30),
    'right_rear':  (-0.44, -0.30),
}


def _yaw_to_quat(yaw: float):
    """Return (x, y, z, w) quaternion for a pure yaw rotation."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class SwerveDriveController(Node):

    def __init__(self):
        super().__init__('swerve_drive_controller')

        # Per-wheel publishers
        self._steer_pub: dict[str, rclpy.publisher.Publisher] = {}
        self._drive_pub: dict[str, rclpy.publisher.Publisher] = {}
        for name in WHEELS:
            self._steer_pub[name] = self.create_publisher(
                Float64, f'/gz/steer/{name}', 10)
            self._drive_pub[name] = self.create_publisher(
                Float64, f'/gz/drive/{name}', 10)

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Commanded velocity (used for odometry integration)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        # Integrated pose
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._last_t = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_timer(0.02, self._odom_timer_cb)  # 50 Hz odometry

        self.get_logger().info('Swerve drive controller ready')

    # ── Command velocity callback ──────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist) -> None:
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        self._vx = vx
        self._vy = vy
        self._wz = wz

        for name, (wx, wy) in WHEELS.items():
            # Velocity contribution at this wheel position
            vwx = vx - wz * wy
            vwy = vy + wz * wx
            speed_ms = math.hypot(vwx, vwy)

            if speed_ms < 1e-6:
                steer_angle = 0.0
                drive_rads  = 0.0
            else:
                steer_angle = math.atan2(vwy, vwx)
                # Keep steer within ±π/2 by reversing drive if needed
                if steer_angle > math.pi / 2:
                    steer_angle -= math.pi
                    speed_ms = -speed_ms
                elif steer_angle < -math.pi / 2:
                    steer_angle += math.pi
                    speed_ms = -speed_ms
                drive_rads = speed_ms / WHEEL_RADIUS

            steer_msg = Float64()
            steer_msg.data = steer_angle
            self._steer_pub[name].publish(steer_msg)

            drive_msg = Float64()
            drive_msg.data = drive_rads
            self._drive_pub[name].publish(drive_msg)

    # ── Odometry timer ────────────────────────────────────────────────

    def _odom_timer_cb(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now

        # Integrate body-frame velocity into world-frame pose
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        self._x   += (self._vx * cos_y - self._vy * sin_y) * dt
        self._y   += (self._vx * sin_y + self._vy * cos_y) * dt
        self._yaw += self._wz * dt

        qx, qy, qz, qw = _yaw_to_quat(self._yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp         = stamp
        odom.header.frame_id      = 'odom'
        odom.child_frame_id       = 'base_footprint'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = self._vx
        odom.twist.twist.linear.y  = self._vy
        odom.twist.twist.angular.z = self._wz
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
