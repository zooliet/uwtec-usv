#! /usr/bin/env python

import sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from uwtec_nav.utils.gps_utils import (
    euler_from_quaternion,
)
from geometry_msgs.msg import TwistStamped
import argparse


class NavDemo(Node):
    def __init__(self, params):
        super().__init__("nav_demo_node")
        self.interval = float(params.get("interval", 1.0))
        self.duration = int(params.get("duration", 30))
        self.angular = float(params.get("angular", 0.5))
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
        )

        self.diff_drive_cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            # TwistStamped, "/diff_drive_base_controller/cmd_vel", 10
            "/cmd_vel_nav",
            10,
        )
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(
            f"\nHeading: {self.heading:.2f}, Angular: {self.angular:.2f}"
        )

        if self.duration > 0:
            self.go_dancing()
            self.duration -= 1

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude

    def go_dancing(self):
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"
        twist_msg.twist.angular.z = 1.0
        self.diff_drive_cmd_vel_publisher.publish(twist_msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=1.0, help="timer interval")
    ap.add_argument("-d", "--duration", type=int, default=30, help="duration")
    ap.add_argument(
        "-a", "--angular", type=float, default=0.5, help="angular velocity (-1 ~ 1)"
    )
    args = vars(ap.parse_args())
    print(args)

    rclpy.init()
    node = NavDemo(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
