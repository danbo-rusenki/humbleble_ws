#!/usr/bin/env python3
"""Relay /cmd_vel (Nav2 velocity_smoother output) → /rover_twist (mecanum relay input).

In nav2_bringup navigation_launch.py the velocity_smoother is remapped
(cmd_vel_smoothed → cmd_vel, cmd_vel → cmd_vel_nav), so /cmd_vel is ALREADY the
smoothed output and the raw controller output is on /cmd_vel_nav. Subscribe to /cmd_vel
here so the smoothed (ramped-acceleration) command reaches the wheels.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self._pub = self.create_publisher(Twist, '/rover_twist', 10)
        self.create_subscription(Twist, '/cmd_vel', self._pub.publish, 10)
        self.get_logger().info('cmd_vel_relay: /cmd_vel → /rover_twist')


def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
