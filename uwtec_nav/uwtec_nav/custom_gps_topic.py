#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from uwtec_nav.utils.gps_utils import (
    euler_from_quaternion,
)

import argparse


class GPSDemo(Node):
    def __init__(self, params):
        super().__init__("gps_demo_node")

        self.interval = float(params.get("interval", 1.0))
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
        )
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(
            f"\nLatitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, Heading: {self.heading:.2f}"
        )

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=1.0, help="timer interval")
    args = vars(ap.parse_args())
    print(args)

    rclpy.init()
    node = GPSDemo(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
